#!/usr/bin/env python3
"""New-URDF SuperMimic commissioning. Uses the existing stand/lie and native-PD chain."""
import argparse,hashlib,importlib.util,json,math,os,socket,sys,time
import gc
from collections import deque
from pathlib import Path
import numpy as np
from deploy_050 import Deployment,FeedbackStaleError
from motion_cli import ROOT,serve
from dog_motion.deploy050 import Actor,LEGS,finite
from dog_motion.deploy_inputs import axis,OmniStickGate,omni_command
from fast_command_reference import fast_goal
from instant_center_reference import immediate_center_reference
from usb_reconnect import USBReconnect

class ContinuousActor(Actor):
    def __init__(self,bundle):
        super().__init__(bundle)
        for name in ['command_reference.py','command_gait_table.npz']:
            if hashlib.sha256((self.bundle/name).read_bytes()).hexdigest()!=self.file_hashes[name]:
                raise ValueError('连续参考文件校验失败: '+name)
        spec=importlib.util.spec_from_file_location('verified_command_reference',self.bundle/'command_reference.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.reference_family=self.contract.get('reference_family','legacy')
        self.position_rate_limit=bool(self.contract.get('position_rate_limit',False))
        if self.position_rate_limit:
            self.target_velocity_limits=finite(self.contract['target_velocity_limits_rad_s'],(12,))
            if (np.any(self.target_velocity_limits<=0) or
                    np.any(self.target_velocity_limits>np.tile([30.1,30.1,15.05],4))):
                raise ValueError('策略位置目标限速参数不合法')
        self.bootstrap_reference=self.contract.get('bootstrap_reference','references/cmd025_forward.npz')
        if self.reference_family=='legged_control_trot':
            name='legged_gait.py'
            if hashlib.sha256((self.bundle/name).read_bytes()).hexdigest()!=self.file_hashes[name]:
                raise ValueError('步态生成文件校验失败：'+name)
        self.planner_class=immediate_center_reference(module.CommandReference)
        self.reset_planner()
    def reset_planner(self):self.planner=self.planner_class(self.bundle/'command_gait_table.npz')
    def reference(self,name):
        if name not in ('forward','forward_left','left','back_left','back','back_right','right','forward_right','ccw','cw'):raise ValueError('未知移动方向')
        # Bootstrap pose only; runtime motion uses the continuous 3-D command planner.
        p=self.bundle/self.bootstrap_reference
        if not p.resolve().is_relative_to((self.bundle/'references').resolve()):raise ValueError('无效起始参考路径')
        if hashlib.sha256(p.read_bytes()).hexdigest()!=self.file_hashes[self.bootstrap_reference]:raise ValueError('参考校验失败')
        with np.load(p) as z:return {k:z[k].copy() for k in z.files}
    def tick_command(self,q,dq,R,gyro,command,immediate_zero=True):
        R=finite(R,(3,3));command=finite(command,(3,))
        # Apply zero before building this frame's look-ahead reference as well
        # as its copies. Do not reset phase, pose, heading, or actor history.
        self.planner.immediate_zero=immediate_zero
        self.planner.prepare_command(command)
        obs=np.r_[finite(q,(12,))-self.q0,finite(dq,(12,)),finite(gyro,(3,)),-R[2],self.previous,(R.T@self.planner.rotation())[:,:2].reshape(6)]
        # Batched interpolation preserves dynamic timing and the distinct
        # settling gait command; exported source remains the parity oracle.
        goal=fast_goal(self.planner,command)
        previous=self.previous.copy()
        action,desired=self.infer(obs,goal)
        if self.position_rate_limit:
            applied=(desired-self.q0)/self.scale
            delta=self.target_velocity_limits*float(self.contract['policy_period_s'])/self.scale
            self.previous=previous+np.clip(applied-previous,-delta,delta)
            desired=np.clip(self.q0+self.scale*self.previous,-1.55,1.55)
            action=self.previous.copy()
        self.planner.advance(command)
        return action,desired

class NewDeployment(USBReconnect,Deployment):
    def __init__(self,gamepad=False):
        super().__init__(gamepad,ROOT/'config/deploy_newurdf.yaml',ContinuousActor)
        self.walk_unlocked=False;self.last_command=np.zeros(3);self.stop_requested_at=None
        self.reset_stick_gate()
        self.report_directory=ROOT/'reports/newurdf_policy'
        self.gc_events=deque(maxlen=128);self.gc_started={}
        self.gc_callback=self.record_gc
        gc.callbacks.append(self.gc_callback)
        # Reference counting stays active. Defer cyclic GC pauses until all
        # motors are disabled; the monitor calls tick under the command lock.
        self.restore_automatic_gc=gc.isenabled()
        gc.disable()
        self.last_idle_gc=time.monotonic()
        self.previous_switch_interval=sys.getswitchinterval()
        sys.setswitchinterval(.001)
        self.feedback_wait_events=deque(maxlen=64)
        self.setup_usb_reconnect(ROOT)
    def record_gc(self,phase,info):
        generation=info['generation'];now=time.monotonic()
        if phase=='start':self.gc_started[generation]=now
        elif generation in self.gc_started:
            started=self.gc_started.pop(generation)
            if now-started>=.0005:
                self.gc_events.append(dict(start=started,end=now,duration_ms=(now-started)*1000,generation=generation))
    def close_inputs(self):
        if getattr(self,'gc_callback',None) in gc.callbacks:gc.callbacks.remove(self.gc_callback)
        try:super().close_inputs()
        finally:
            if getattr(self,'restore_automatic_gc',False):gc.enable()
            if hasattr(self,'previous_switch_interval'):sys.setswitchinterval(self.previous_switch_interval)
    def status(self):
        d=super().status();d['deployment'].update(walk_unlocked=self.walk_unlocked,
            max_reference_speed_mps=self.cfg['commissioning_max_forward_mps'],max_yaw_rate_rad_s=self.cfg['commissioning_max_yaw_rad_s'],omnidirectional=True,command=self.last_command.tolist(),continuous_command=True,
            automatic_cyclic_gc=gc.isenabled(),python_switch_interval_ms=sys.getswitchinterval()*1000,
            feedback_wait_events=list(getattr(self,'feedback_wait_events',[])))
        d['deployment']['neutral_behavior']='zero_velocity_keep_policy'
        d['deployment']['neutral_reference_immediate']=True
        d['deployment']['reference_family']=getattr(self.actor,'reference_family','legacy')
        d['deployment']['position_target_rate_limit']=getattr(self.actor,'position_rate_limit',False)
        d['deployment']['reference_clearance_m']=self.actor.contract.get('reference_clearance_m')
        d['deployment']['stick_gate']={'stop_on_neutral':self.stick_gate.stop_on_neutral,
                                      'start_threshold':self.stick_gate.start_threshold}
        d['deployment']['usb_reconnect']=getattr(self,'usb_status',{})
        if self.pad is not None:
            try:
                axes,buttons,armed=self.pad.snapshot()
                d['deployment']['gamepad_readiness']=dict(armed=armed,
                    axes={str(i):float(axes[i]) for i in (0,1,3)},
                    pressed_buttons=[i for i,v in enumerate(buttons) if v],
                    neutral=not any(buttons) and max(abs(axis(axes,i)) for i in (0,1,3))<=.12)
            except RuntimeError as exc:
                d['deployment']['gamepad_readiness']=dict(error=str(exc))
        return d
    def latch_fault(self,reason,snapshots):
        self.walk_unlocked=False
        first=not self.fault
        timing=dict(trigger_monotonic_s=time.monotonic(),reason=reason,gc_events=list(getattr(self,'gc_events',[])))
        timing['feedback_gate_at_rejection']=getattr(self,'last_feedback_gate',None)
        # Capture even expired IMU evidence; snapshot() deliberately raises
        # after30ms, so it cannot be used to diagnose a stalled reader.
        if hasattr(self.imu,'lock'):
            with self.imu.lock:
                timing['imu_last_sample']=dict(self.imu.latest) if self.imu.latest else None
                timing['imu_error']=self.imu.error
        super().latch_fault(reason,snapshots)
        if first and self.fault_detail:
            # Save diagnostics only after the existing release/fault path has run.
            try:
                p=Path(self.fault_detail['report_path']).with_suffix('.timing.json')
                p.write_text(json.dumps(timing,ensure_ascii=False,indent=2)+'\n')
            except OSError:pass
    def state(self):
        # Give completed serial transactions a bounded chance to publish after
        # host scheduling jitter. Never return the stale snapshot to the actor.
        deadline=time.perf_counter()+.008
        while True:
            self.check_feedback_hard_deadline()
            imu=self.imu.snapshot()
            try:
                if imu['age_s']<=self.cfg['imu_age_max_s']:
                    # Pass the frame actually checked, rather than accepting
                    # an old pose because a newer frame arrived afterwards.
                    return super().state(imu_sample=imu)
            except FeedbackStaleError:
                if time.perf_counter()>=deadline:raise
            remaining=deadline-time.perf_counter()
            if remaining<=0:raise RuntimeError('新策略IMU反馈超过20毫秒；等待新帧未恢复')
            time.sleep(min(.0005,remaining))

    def check_feedback_hard_deadline(self):
        # Match the existing bus watchdog: waiting never extends its 20 ms
        # deadline, and no new target is accepted with feedback older than12 ms.
        snapshots=self.require_ready()
        if any(m['feedback_age_s']>=.02 for m in snapshots):
            raise FeedbackStaleError('关节反馈超过20毫秒硬时限（新目标仍要求12毫秒内）')
        imu=self.imu.snapshot()
        age=float(imu['age_s'])
        if not math.isfinite(age) or age<0 or age>=.03:
            raise RuntimeError('IMU反馈达到30毫秒硬时限或时间戳无效')
    def begin(self,mode,clip):
        if mode=='PLAY' and not self.walk_unlocked:raise RuntimeError('本轮行走未解锁；完成静态检查、确认地面条件后 arm-walk')
        # Reject missing inputs before the parent installs native-PD targets.
        # Loss during an active movement still follows the existing fault path.
        if mode in ('CHECK','PLAY') and self.pad is not None:
            try:axes,buttons,armed=self.pad.snapshot()
            except RuntimeError as exc:
                raise RuntimeError(f'未进入检查/策略，电机目标与增益未改变：{exc}') from exc
            if not armed:
                values=', '.join(f'{i}={axes[i]:.3f}' for i in (0,1,3))
                pressed=[i for i,v in enumerate(buttons) if v]
                raise RuntimeError(f'未进入检查/策略：手柄尚未回中就绪；请松开按键、摇杆回中。当前轴 {values}；按下按键 {pressed}')
            if mode=='CHECK' and (any(buttons) or max(abs(axis(axes,i)) for i in (0,1,3))>.12):
                raise RuntimeError('未进入静态检查：请松开手柄按键、摇杆回中')
        result=super().begin(mode,clip);self.actor.reset_planner();self.last_command=np.zeros(3);self.stop_requested_at=None
        if mode in ('PLAY','SHADOW'):result.update(duration_s=None,message='全向移动；左摇杆平移、右摇杆旋转，回中给零速度并保持策略，再推即响应；B减速后趴下' if mode=='PLAY' else '8秒连续参考影子推理；电机保持站姿')
        return result
    def begin_stop(self,force_return=False):
        if self.mode=='PLAY' and not force_return:
            if not self.decelerating:
                self.decelerating=True;self.stop_requested_at=time.monotonic()
            return
        return super().begin_stop(force_return=True)
    def on_static_tracking_failure(self,q):
        self.session_pd_verified=False;self.walk_unlocked=False
        # A failed gain experiment is not a passed check. Only recover while
        # the existing feedback, attitude and motor protections remain healthy.
        self.check_posture_imu()
        error=np.abs(q-self.check_q);index=int(np.argmax(error))
        reason=f'静态检查未通过：{LEGS[index//3]} {("髋","大腿","小腿")[index%3]}偏差{math.degrees(error[index]):.2f}°，恢复原站姿保持'
        self.servo_check_result=dict(passed=False,state='returning_to_stand',reason=reason,
            max_error_rad=float(error[index]),joint_errors_rad=error.tolist(),restored_stand=False)
        self.last_policy_error=reason
        if self.log:self.log.write(json.dumps(dict(event='static_check_failed',**self.servo_check_result),ensure_ascii=False)+'\n')
        self.begin_stop(force_return=True)
        self.servo_check_abort_pending=True
    def requested_command(self):
        if self.decelerating:return np.zeros(3)
        speed=float(self.cfg['commissioning_max_forward_mps'])
        if self.mode=='SHADOW':return np.array([speed,0,0])
        if self.pad:
            axes,buttons,armed=self.pad.snapshot()
            if not armed:raise RuntimeError('手柄尚未回中解锁')
            command=np.asarray(omni_command(axes,speed,float(self.cfg['commissioning_max_yaw_rad_s'])))
            # Center is a zero velocity command, not a request to leave PLAY.
            # Keep planner/history/gains continuous for immediate re-acceleration.
            # Explicit policy-stop and B still use the controlled exit path.
            return command
        return np.array([speed,0,0])
    def fresh_feedback_after_compute(self):
        started=time.perf_counter();deadline=started+.008
        while True:
            snapshots=self.require_ready()
            joint_age=max(m['feedback_age_s'] for m in snapshots)
            imu_age=self.imu.snapshot()['age_s']
            self.last_feedback_gate=dict(phase='after_compute',monotonic_s=time.monotonic(),
                joint_age_ms=joint_age*1000,imu_age_ms=imu_age*1000,
                waited_ms=(time.perf_counter()-started)*1000)
            if not math.isfinite(imu_age) or imu_age<0 or imu_age>=.03:
                raise RuntimeError('IMU反馈达到30毫秒硬时限或时间戳无效')
            if joint_age>=.02:raise FeedbackStaleError('目标计算后关节反馈超过20毫秒硬时限（新目标仍要求12毫秒内）')
            if joint_age<=self.cfg['joint_age_max_s'] and imu_age<=self.cfg['imu_age_max_s']:
                waited=(time.perf_counter()-started)*1000
                if waited>=2 and hasattr(self,'feedback_wait_events'):
                    self.feedback_wait_events.append(dict(monotonic_s=time.monotonic(),wait_ms=waited,phase='after_compute'))
                return joint_age,imu_age,waited
            remaining=deadline-time.perf_counter()
            if remaining<=0:
                if imu_age>self.cfg['imu_age_max_s']:raise RuntimeError('目标计算后IMU反馈超过20毫秒；等待新帧未恢复')
                raise FeedbackStaleError('目标计算后关节反馈超过12毫秒')
            time.sleep(min(.0005,remaining))
    def tick(self):
        if self.usb_reconnect_step():return
        if (hasattr(self,'last_idle_gc') and not self.ctrl.is_enabled and not self.ctrl.is_moving
                and self.mode in ('IDLE','FAULT') and time.monotonic()-self.last_idle_gc>=30):
            gc.collect()
            self.last_idle_gc=time.monotonic()
        if self.mode not in ('PLAY','SHADOW'):return super().tick()
        now=time.monotonic();interval=now-self.last_tick
        if interval>.04:raise RuntimeError('新策略循环超过40毫秒')
        self.last_tick=now
        q,dq,R,gyro=self.state();command=self.requested_command();self.last_command=command.copy()
        started=time.perf_counter();action,desired=self.actor.tick_command(q,dq,R,gyro,command,
            immediate_zero=not self.decelerating)
        self.inference_ms.append((time.perf_counter()-started)*1000);self.frame+=1
        # Also check freshness after computing the target, before sending it.
        joint_age,imu_age,feedback_wait_ms=self.fresh_feedback_after_compute()
        if time.monotonic()-now>.04:raise RuntimeError('本帧目标计算超过40毫秒')
        if self.mode=='PLAY':
            for leg,bus in zip(self.ctrl.legs,self.ctrl.buses):
                j=LEGS.index(leg['code'])*3;bus.update_joint_drive(desired[j:j+3],np.full(3,self.policy_kp),self.policy_kd[j:j+3],[23.7,23.7,42.66])
        if self.log:
            self.log.write(json.dumps(dict(event='state',monotonic_s=now,mode=self.mode,frame=self.frame,q=q.tolist(),dq=dq.tolist(),R=R.tolist(),gyro=gyro.tolist(),action=action.tolist(),q_des=desired.tolist(),command=command.tolist(),filtered_command=self.actor.planner.command.tolist(),loop_interval_s=interval,compute_ms=self.inference_ms[-1],feedback_wait_ms=feedback_wait_ms,joint_age_s_max=joint_age,imu_age_s=imu_age))+'\n');self.log.flush()
        if self.mode=='SHADOW' and now-self.started>=8:
            self.close_log('shadow_complete');self.mode='IDLE'
        elif self.mode=='PLAY' and self.decelerating and now-self.stop_requested_at>=1.5 and np.max(np.abs(self.actor.planner.command))<1e-6:
            super().begin_stop(force_return=True)
    def execute(self,command,params=None):
        with self.lock:
            if command=='reconnect':
                if self.usb_output_active():
                    raise RuntimeError('使能或运动期间禁止手动重连；先停止并松弛')
                self.usb_pending=True;self.usb_reconnect_step(force=True)
                if self.usb_status['state']!='connected':
                    raise RuntimeError('重连尚未完成：'+self.usb_status['message'])
                return dict(self.usb_status,enabled=False,initialized=self.initialized)
            if command in ('init-lie','stand','servo-check','arm-walk','clear-fault'):
                self.usb_reconnect_step(force=True)
            if command=='arm-walk':
                if self.mode!='IDLE' or self.ctrl.is_moving or not self.ctrl.is_enabled or not self.session_pd_verified:
                    raise RuntimeError('先完成起立和本次servo-check，并等待恢复站姿')
                self.state()
                if self.pad:
                    axes,buttons,armed=self.pad.snapshot()
                    if not armed or any(buttons) or max(abs(axis(axes,i)) for i in (0,1,3))>.12:raise RuntimeError('先松开所有按键并使摇杆回中')
                self.walk_unlocked=True;self.stick_gate.neutral_seen=False
                speed=float(self.cfg['commissioning_max_forward_mps'])
                return {'session':self.session_id,'message':f'已解锁全向平移最高{speed:g}m/s、旋转最高{self.cfg["commissioning_max_yaw_rad_s"]:g}rad/s；先回中，左摇杆平移、右摇杆旋转；回中给零速度并保持策略，再推即响应，B趴下'}
            if command in ('init-lie','restore-lie','clear-fault','release','stop','emergency-stop'):self.walk_unlocked=False
            return super().execute(command,params)

def wait_servo_check(call,timeout_s=8.):
    deadline=time.monotonic()+timeout_s
    while time.monotonic()<deadline:
        time.sleep(.1)
        response=call({'command':'status'})
        if not response.get('ok'):return response
        status=response['result'];deployment=status['deployment']
        if status.get('fault') or deployment['mode']=='FAULT':
            return dict(ok=False,result=dict(message='静态检查触发故障保护',fault=status.get('fault'),enabled=status['enabled']))
        check=deployment.get('servo_check_result')
        if deployment['mode']=='IDLE' and check and check.get('state') in ('passed','failed_returned_to_stand'):
            return dict(ok=check['passed'],result=dict(check,
                message='静态检查通过，已恢复站姿' if check['passed'] else '静态检查未通过，已恢复原站姿保持；行走未解锁',
                enabled=status['enabled'],walk_unlocked=deployment.get('walk_unlocked',False)))
    return dict(ok=False,result=dict(message='等待检查结果超时，请用 status 查看；未发送额外动作'))

def main():
    p=argparse.ArgumentParser();p.add_argument('command',choices=['serve','status','watch','reconnect','init-lie','stand','lie','release','clear-fault','stop','shadow','play','policy-stop','servo-check','arm-walk']);p.add_argument('--gamepad',action='store_true')
    p.add_argument('--no-watch',action='store_true',help='arm-walk 解锁后立即退出，不持续显示运行故障')
    a=p.parse_args()
    key=hashlib.sha256(str(ROOT).encode()).hexdigest()[:10];path=Path('/tmp')/('rcdog27-%d-%s.sock'%(os.getuid(),key))
    if a.command=='serve':a.sim=False;serve(a,path,NewDeployment(a.gamepad));return
    request={'command':{'shadow':'policy-shadow','play':'policy-start'}.get(a.command,a.command),'clip':'forward'}
    def call(request):
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
            s.settimeout(5);s.connect(str(path));s.sendall((json.dumps(request)+'\n').encode())
            with s.makefile('rb') as f:return json.loads(f.readline(131072))
    try:
        if a.command=='watch':
            from deployment_diagnostics import watch_runtime
            result=watch_runtime(call,sys.stdout)
        else:result=call(request)
        if a.command=='servo-check' and result.get('ok'):
            print('静态检查中，等待检查及恢复站姿完成……',flush=True)
            result=wait_servo_check(call)
    except (OSError,ValueError) as exc:
        result=dict(ok=False,error=f'无法完成后台请求：{exc}',
                    outcome_unknown=not isinstance(exc,(FileNotFoundError,ConnectionRefusedError)))
    print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    if a.command=='arm-walk' and result.get('ok') and not a.no_watch:
        from deployment_diagnostics import watch_runtime
        result=watch_runtime(call,sys.stdout,result.get('result',{}).get('session'))
        print(json.dumps(result,ensure_ascii=False,indent=2),flush=True)
    if not result.get('ok'):
        from deployment_diagnostics import report_failure
        report_failure(a.command,result,call,ROOT,sys.stderr)
    raise SystemExit(0 if result.get('ok') else 1)
if __name__=='__main__':main()
