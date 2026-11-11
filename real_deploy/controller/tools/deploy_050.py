#!/usr/bin/env python3
"""RCDog27 0.5 m/s deployment. Startup is read-only; PLAY is explicit."""
import argparse
from datetime import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import time
from zoneinfo import ZoneInfo
import numpy as np
import yaml

from motion_cli import Session,serve,ROOT
from dog_motion.deploy050 import Actor,Transmission,NativePD,LEGS,EXTRA,GEAR,quaternion_matrix,rotation_z,validate_deployment_gains,joint_gains,operator_walk_authorized
from dog_motion.deploy_inputs import HI12,QueuedJoystick,direction,ForwardStickGate,fresh_imu_snapshot
from dog_motion.reference_playback import jump_to_deceleration,validate_forward_loop,forward_loop_frame

class FeedbackStaleError(RuntimeError):
    pass

class Deployment(Session):
    def __init__(self,gamepad=False,config_path=None,actor_class=Actor):
        super().__init__(False)
        self.cfg=yaml.safe_load(Path(config_path or ROOT/'config/deploy_050.yaml').read_text())
        if self.cfg.get('controller')!='motor_native_pd':
            raise ValueError('实机只允许电机内部 PD；旧主机纯力矩环已停用')
        from dog_motion.bus import GEAR as sdk_gear
        if abs(sdk_gear-GEAR)>.001 or self.cfg['sdk_internal_gear_ratio']!=6.33 or self.cfg['extra_ratio_per_leg']!=[1,1,2]:
            raise ValueError('电机内部减速比或小腿额外减速比不匹配')
        self.ctrl.rate_hz=1./float(self.cfg.get('motor_period_s',.004))
        if not (ROOT/self.cfg['bundle']/'export_report.json').is_file():
            raise RuntimeError('SuperMimic新参数策略正在训练，尚未导出到 '+self.cfg['bundle'])
        self.actor=actor_class(ROOT/self.cfg['bundle'])
        self.policy_kp=float(self.cfg['policy_kp_joint']);self.policy_kd=joint_gains(self.cfg['policy_kd_joint'])
        if self.actor.hist!=self.cfg['history_length'] or abs(self.actor.scale-self.cfg['action_scale'])>1e-8:
            raise ValueError('SuperMimic训练与部署参数不符')
        self.experimental_pd=validate_deployment_gains(self.actor.contract,self.policy_kp,self.policy_kd,
            self.cfg.get('experimental_pd_override'))
        self.session_pd_verified=False
        self.servo_check_result=None
        self.servo_check_abort_pending=False
        if self.cfg.get('checkpoint_sha256') is not None and self.actor.contract['checkpoint_sha256']!=self.cfg['checkpoint_sha256']:
            raise ValueError('策略校验值与配置不符')
        self.mode='IDLE';self.selected='forward';self.active_clip=None;self.frame=0;self.log=None
        self.maps={};self.reference=None;self.imu_error=None;self.gamepad_error=None
        self.last_tick=None;self.R_WN=None;self.started=0.;self.inference_ms=[]
        self.last_policy_error=None
        self.decelerating=False;self.pending_lie=False
        self.imu=HI12(self.cfg['imu']['device'],ROOT/self.cfg['imu']['parser'])
        try:self.imu.start()
        except Exception as exc:self.imu_error=str(exc)
        self.pad=QueuedJoystick(self.cfg['gamepad']['device']) if gamepad else None
        self.stick_gate=ForwardStickGate()

    def close_log(self,reason):
        if self.log:
            self.log.write(json.dumps(dict(event='end',reason=reason,
                inference_ms_max=max(self.inference_ms,default=0)),ensure_ascii=False)+'\n')
            self.log.close();self.log=None

    def close_inputs(self):
        self.close_log('server_stopped');self.imu.close()
        if self.pad:self.pad.close()

    def status(self):
        d=super().status()
        try:imu=self.imu.snapshot()
        except Exception as exc:imu={'error':self.imu_error or str(exc)}
        d['deployment']={'mode':self.mode,'selected_clip':self.selected,'frame':self.frame,
            'active_clip':self.active_clip if self.mode in ('SHADOW','PLAY','CHECK','STOPPING') else None,
            'bundle':self.cfg['bundle'],'max_reference_speed_mps':.5,
            'mapping_verified':{k:v['mapping_verified'] for k,v in self.cfg['legs'].items()},
            'imu_orientation_verified':self.cfg['imu']['orientation_verified'],
            'imu':imu,'gamepad_error':self.gamepad_error,
            'gamepad_enabled':self.pad is not None,'actor_sha256':self.actor.actor_sha256,
            'controller':self.cfg['controller'],'framework':'Uni-SuperMimic',
            'kp_joint':self.policy_kp,'kd_joint':joint_gains(self.policy_kd).tolist(),'history_length':self.actor.hist,
            'action_scale':self.actor.scale,'policy_period_s':.02,
            'native_pd_verified':bool(self.cfg.get('native_pd_verified',False)),
            'session_pd_verified':self.session_pd_verified,'experimental_pd':self.experimental_pd,
            'servo_check_result':getattr(self,'servo_check_result',None),
            'trained_kd_joint':self.actor.contract['kd_joint'],
            'trial_max_s':self.cfg.get('policy_trial_max_s'),
            'stick_control':bool(self.cfg.get('stick_control',False)),
            'continuous_forward':bool(self.cfg.get('continuous_forward',False)),
            'sim2sim_verified':bool(self.cfg.get('sim2sim_verified',False)),
            'operator_walk_authorized':operator_walk_authorized(self.cfg,self.actor.contract,'forward'),
            'absolute_joint_reference_verified':bool(self.cfg.get('absolute_joint_reference_verified',False)),
            'gamepad_device':self.pad.device_info if self.pad else None,
            'last_policy_error':self.last_policy_error,
            'decelerating':self.decelerating,'pending_lie':self.pending_lie,
            'last_inference_ms':self.inference_ms[-1] if self.inference_ms else None}
        return d

    def build_maps(self):
        snapshots=self.snaps()
        for i,leg in enumerate(self.ctrl.legs):
            code=leg['code'];c=self.cfg['legs'][code]
            ms=snapshots[i*3:i*3+3]
            theta=[m['zero_rotor_rad']/GEAR+m['direction']*math.radians(v)
                   for m,v in zip(ms,c['measured_stand_relative_motor_output_deg'])]
            ci=LEGS.index(code)*3
            self.maps[code]=Transmission(c['A_joint_from_motor_output'],theta,self.actor.q0[ci:ci+3])

    def check_posture_imu(self):
        """Supported stand/lie and legacy holding also need fresh, level-body feedback."""
        imu=fresh_imu_snapshot(self.imu,float(self.cfg['imu_age_max_s']))
        age=float(imu['age_s'])
        if not math.isfinite(age) or age<0 or age>self.cfg['imu_age_max_s']:
            raise RuntimeError(f'起立／保持期间 IMU 数据过期：{age*1000:.1f} ms，允许 {self.cfg["imu_age_max_s"]*1000:g} ms')
        R=quaternion_matrix(imu['quat_wxyz'])@np.asarray(self.cfg['imu']['R_CI'],float).T
        tilt=math.degrees(math.acos(float(np.clip(R[2,2],-1.,1.))))
        if self.recent_samples:
            self.recent_samples[-1]['imu']=dict(imu)
            self.recent_samples[-1]['body_tilt_deg']=tilt
        limit=float(self.cfg['posture_tilt_limit_deg'])
        if tilt>limit:
            raise RuntimeError(f'起立／保持倾角 {tilt:.1f}° 超过 {limit:.1f}°，停止出力')
        return tilt

    def latch_fault(self,reason,snapshots):
        super().latch_fault(reason,snapshots)
        # Power may have been cycled while faulted; clearing a fault must not
        # authorize movement using the old session's lie reference.
        self.initialized=False;self.policy_ready=False;self.maps={}
        self.session_pd_verified=False

    def state(self,*,imu_sample=None):
        snapshots=self.require_ready();qs={};dqs={}
        for i,leg in enumerate(self.ctrl.legs):
            ms=snapshots[3*i:3*i+3]
            age_limit=min(.02,float(self.cfg.get('joint_age_max_s',.02)))
            if any(m['feedback_age_s']>age_limit for m in ms):
                raise FeedbackStaleError(f'关节反馈超过{age_limit*1000:g}毫秒')
            if any(m['temp']>=self.cfg['max_temperature_c'] for m in ms):raise RuntimeError('电机温度越限')
            qs[leg['code']],dqs[leg['code']]=self.maps[leg['code']].feedback(
                [m['q_rotor_rad'] for m in ms],[math.radians(m['dq_deg'])*GEAR for m in ms])
        imu=self.imu.snapshot() if imu_sample is None else imu_sample
        R_NI=quaternion_matrix(imu['quat_wxyz'])
        R_CI=np.asarray(self.cfg['imu']['R_CI'],float)
        R_NC=R_NI@R_CI.T
        R=R_NC if self.R_WN is None else self.R_WN@R_NC
        gyro=R_CI@np.asarray(imu['gyro_rad_s'])
        if R[2,2]<math.cos(math.radians(self.cfg['tilt_limit_deg'])):
            raise RuntimeError('IMU 倾角超过配置范围')
        return np.concatenate([qs[k] for k in LEGS]),np.concatenate([dqs[k] for k in LEGS]),R,gyro

    def begin(self,mode,clip):
        if self.mode!='IDLE' or self.ctrl.is_moving:raise RuntimeError('先等待当前动作结束')
        self.require_ready()
        if not self.ctrl.is_enabled:raise RuntimeError('先完成默认起立并保持')
        if mode in ('PLAY','CHECK'):
            missing=[k+' URDF 方向/传动' for k,v in self.cfg['legs'].items() if not v['mapping_verified']]
            if not self.cfg['imu']['orientation_verified']:missing.append('IMU 方向核对')
            if missing:raise RuntimeError('实机策略尚缺：'+'、'.join(missing)+'；可先影子推理')
            for c in self.cfg['legs'].values():
                A=np.asarray(c['A_joint_from_motor_output'])
                if not np.allclose(A,np.diag(np.diag(A))) or not np.allclose(np.abs(np.diag(A)),[1,1,.5]):
                    raise RuntimeError('当前实机退出逻辑只支持已核对的独立轴1/1/2传动；耦合矩阵需另行验证')
            if mode=='PLAY' and not (self.cfg.get('native_pd_verified',False) or getattr(self,'session_pd_verified',False)):
                raise RuntimeError('修正版电机内部 PD 尚未完成带支撑静态核对；先 servo-check，现场确认后再解锁行走')
        if mode=='PLAY' and not self.cfg.get('sim2sim_verified',False) and not operator_walk_authorized(self.cfg,self.actor.contract,clip):
            raise RuntimeError('SuperMimic新参数策略尚未通过MuJoCo验证')
        if mode=='PLAY' and not self.cfg.get('absolute_joint_reference_verified',False):
            raise RuntimeError('站姿仅由操作者锚定q0，尚需核对实物P/K/脚垫几何与URDF绝对关节角')
        if mode=='PLAY' and clip not in self.cfg.get('commissioning_allowed_clips',[]):
            raise RuntimeError('本轮实机试验未开放该方向；已开放：'+','.join(self.cfg.get('commissioning_allowed_clips',[])))
        if mode=='CHECK':
            self.session_pd_verified=False
            self.servo_check_result={'passed':False,'state':'running'}
            self.servo_check_abort_pending=False
        self.build_maps();self.R_WN=None
        q,dq,R,gyro=self.state()
        if np.max(np.abs(q-self.actor.q0))>self.cfg['entry_pose_tolerance_rad']:
            raise RuntimeError('实物未进入已记录的站姿范围')
        self.reference=self.actor.reference(clip)
        if mode=='PLAY' and self.cfg.get('continuous_forward',False):
            if clip!='forward':raise RuntimeError('连续参考仅验证了前进')
            validate_forward_loop(self.reference)
        refR=self.reference['R_ref'][0]
        self.R_WN=rotation_z(math.atan2(refR[1,0],refR[0,0])-math.atan2(R[1,0],R[0,0]))
        self.actor.reset();self.started=time.monotonic();self.last_tick=self.started
        self.last_policy_error=None
        self.frame=0;self.selected=clip;self.active_clip=clip;self.inference_ms=[]
        self.playback_start_frame=0;self.playback_started=self.started
        self.decelerating=False;self.pending_lie=False
        name=datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_%f')
        directory=getattr(self,'report_directory',ROOT/'reports/policy050');directory.mkdir(parents=True,exist_ok=True)
        self.log=(directory/(name+'_'+mode.lower()+'.jsonl')).open('x')
        self.log.write(json.dumps(dict(event='start',mode=mode,clip=clip,
            candidate_mapping=not all(v['mapping_verified'] for v in self.cfg['legs'].values()),
            actor_sha256=self.actor.actor_sha256,configuration=self.cfg),ensure_ascii=False)+'\n')
        if mode in ('PLAY','CHECK'):
            snaps=self.snaps()
            prepared=[];check_targets={}
            for i,(leg,bus) in enumerate(zip(self.ctrl.legs,self.ctrl.buses)):
                j=LEGS.index(leg['code'])*3
                ms=snaps[3*i:3*i+3]
                rotor=[m['zero_rotor_rad']+m['direction']*math.radians(t)*GEAR
                       for m,t in zip(ms,self.ctrl.stand[3*i:3*i+3])]
                check_targets[leg['code']]=self.maps[leg['code']].feedback(rotor,[0,0,0])[0]
                drive=NativePD(self.maps[leg['code']],check_targets[leg['code']],
                    np.full(3,self.policy_kp),joint_gains(self.policy_kd)[j:j+3],[23.7,23.7,42.66],
                    speed_limits=self.cfg['commissioning_speed_limits_rad_s'])
                drive.start_from_native(rotor,np.full(3,self.ctrl.kp/GEAR**2),
                    np.full(3,self.ctrl.kd/GEAR**2),self.started)
                prepared.append((bus,drive))
            self.check_q=np.concatenate([check_targets[k] for k in LEGS])
            try:
                for bus,drive in prepared:bus.set_joint_drive(drive)
            except Exception:
                self.latch_fault('进入电机内部 PD 失败',self.snaps())
                self.mode='FAULT';self.close_log('native_pd_entry_failed');raise
        self.mode=mode
        return {'message':{'SHADOW':'影子推理，电机保持站姿','CHECK':'2秒电机内部PD静态核对，不运行策略网络','PLAY':'开始实机参考片段'}[mode],
                'clip':clip,'duration_s':2. if mode=='CHECK' else (len(self.reference['goal'])-1)*.02}

    def begin_stop(self,force_return=False):
        if self.mode=='SHADOW':self.close_log('shadow_stop');self.mode='IDLE';return
        if self.mode not in ('PLAY','CHECK'):return
        self.state()
        if self.mode=='PLAY' and not force_return:
            if self.decelerating:return
            target=jump_to_deceleration(self.reference,self.frame)
            if target is not None:
                old=self.frame;self.frame=target
                self.playback_start_frame=target;self.playback_started=time.monotonic()
                self.decelerating=True
                if self.log:self.log.write(json.dumps(dict(event='decelerate',from_frame=old,to_frame=target))+'\n')
                return
        references={leg['code']:bus.freeze_joint_reference()
                    for leg,bus in zip(self.ctrl.legs,self.ctrl.buses)}
        self.stop_q=np.concatenate([references[k][0] for k in LEGS])
        self.stop_kp=np.concatenate([references[k][1] for k in LEGS])
        self.stop_kd=np.concatenate([references[k][2] for k in LEGS])
        self.stop_duration=1.5 if self.mode=='PLAY' and self.frame<80 else 1.
        self.stop_started=time.monotonic();self.mode='STOPPING'

    def on_static_tracking_failure(self,q):
        raise RuntimeError('电机内部PD静态检查偏差超过0.15rad')

    def tick(self):
        if self.mode not in ('PLAY','CHECK','SHADOW','STOPPING'):return
        now=time.monotonic()
        if now-self.last_tick>.06:raise RuntimeError('策略循环超过60毫秒')
        self.last_tick=now
        q,dq,R,gyro=self.state()
        self.check_trial_timeout(now)
        if self.mode=='STOPPING':
            if getattr(self,'servo_check_abort_pending',False):self.check_posture_imu()
            u=min(1.,(now-self.stop_started)/self.stop_duration);u=.5-.5*math.cos(math.pi*u)
            targets={};kp=np.full(12,self.policy_kp);kd=np.full(12,self.policy_kd)
            for i,leg in enumerate(self.ctrl.legs):
                j=LEGS.index(leg['code'])*3
                ms=self.snaps()[3*i:3*i+3]
                rotor=[m['zero_rotor_rad']+m['direction']*math.radians(t)*GEAR
                       for m,t in zip(ms,self.ctrl.stand[3*i:3*i+3])]
                targets[leg['code']]=self.maps[leg['code']].feedback(rotor,[0,0,0])[0]
                kp[j:j+3]=(1-u)*self.stop_kp[j:j+3]+u*self.ctrl.kp*EXTRA**2
                kd[j:j+3]=(1-u)*self.stop_kd[j:j+3]+u*self.ctrl.kd*EXTRA**2
            desired=(1-u)*self.stop_q+u*np.concatenate([targets[k] for k in LEGS])
            action=None
        elif self.mode=='CHECK':
            desired=self.check_q;action=None;kp=np.full(12,self.policy_kp);kd=np.full(12,self.policy_kd)
            if now-self.started>.75 and np.max(np.abs(q-self.check_q))>.15:
                self.on_static_tracking_failure(q)
                return
        else:
            self.frame=self.playback_start_frame+int((now-self.playback_started)/.02)
            if self.mode=='PLAY' and self.cfg.get('continuous_forward',False) and not self.decelerating:
                self.frame=forward_loop_frame(self.frame)
            else:self.frame=min(self.frame,len(self.reference['goal'])-1)
            t=time.perf_counter()
            action,desired=self.actor.tick(q,dq,R,gyro,self.reference,self.frame)
            self.inference_ms.append((time.perf_counter()-t)*1000)
            kp=np.full(12,self.policy_kp);kd=np.full(12,self.policy_kd)
        if self.mode in ('PLAY','CHECK','STOPPING'):
            for leg,bus in zip(self.ctrl.legs,self.ctrl.buses):
                j=LEGS.index(leg['code'])*3
                bus.update_joint_drive(desired[j:j+3],kp[j:j+3],kd[j:j+3],[23.7,23.7,42.66])
        if self.log:
            self.log.write(json.dumps(dict(event='state',monotonic_s=now,mode=self.mode,
                frame=self.frame,q=q.tolist(),dq=dq.tolist(),R=R.tolist(),gyro=gyro.tolist(),
                action=action.tolist() if action is not None else None,q_des=desired.tolist()),ensure_ascii=False)+'\n')
            self.log.flush()
        if self.mode=='STOPPING' and now-self.stop_started>=self.stop_duration:
            self.ctrl.apply_pose(self.ctrl.stand)
            for bus in self.ctrl.buses:bus.set_gains(self.ctrl.kp,self.ctrl.kd);bus.set_mode('hybrid')
            if getattr(self,'servo_check_abort_pending',False):
                self.servo_check_result.update(state='failed_returned_to_stand',restored_stand=True)
                self.servo_check_abort_pending=False
                reason='static_check_failed_returned_to_stand'
            else:reason='returned_to_default_stand'
            self.close_log(reason);self.mode='IDLE'
            self.decelerating=False
            if self.pending_lie:
                self.pending_lie=False
                self.execute('lie')
        elif self.mode=='CHECK' and now-self.started>=2.:
            self.session_pd_verified=True
            self.servo_check_result={'passed':True,'state':'passed'}
            self.begin_stop()
        elif self.mode in ('PLAY','SHADOW') and self.frame==len(self.reference['goal'])-1:
            if self.mode=='PLAY':self.begin_stop(force_return=True)
            else:self.close_log('shadow_complete');self.mode='IDLE'

    def check_trial_timeout(self,now):
        limit=self.cfg.get('policy_trial_max_s')
        if self.mode=='PLAY' and limit is not None and not self.decelerating and now-self.started>=float(limit):
            if self.log:self.log.write(json.dumps(dict(event='trial_time_limit',seconds=float(limit)))+'\n')
            self.begin_stop()

    def execute(self,command,params=None):
        with self.lock:
            if command=='emergency-stop':
                self.latch_fault('手柄 LB+RB+A 急停',self.snaps())
                self.close_log('emergency_stop');self.mode='FAULT'
                return {'message':'急停已锁存，全部松弛；不会自动恢复'}
            if command=='policy-start':return self.begin('PLAY',(params or {}).get('clip','forward'))
            if command=='policy-shadow':return self.begin('SHADOW',(params or {}).get('clip','forward'))
            if command=='servo-check':return self.begin('CHECK','forward')
            if command=='policy-stop':
                if self.mode in ('PLAY','CHECK','SHADOW'):self.begin_stop()
                elif self.mode=='IDLE' and self.ctrl.is_enabled:self.ctrl.stop_motion()
                return {'message':'停止策略并返回默认站姿；影子推理仅停止记录'}
            if command=='lie' and self.mode in ('PLAY','CHECK','STOPPING'):
                self.pending_lie=True
                self.begin_stop()
                return {'message':'已请求减速，回到默认站姿后执行趴下'}
            if command in ('release','stop'):
                self.session_pd_verified=False
                result=super().execute(command,params)
                self.close_log(command);self.mode='IDLE';return result
            if command=='clear-fault':
                result=super().execute(command,params)
                for bus in self.ctrl.buses:
                    if bus is not None:bus.control_error=None
                self.mode='IDLE';self.last_policy_error=None;return result
            if self.mode in ('PLAY','CHECK','STOPPING') and command not in ('status',):
                raise RuntimeError('先 policy-stop，待回到默认站姿后再操作')
            if command in ('stand','stand-original','lie','jog-group'):
                self.require_ready()
                self.check_posture_imu()
            result=super().execute(command,params)
            if command in ('init-lie','restore-lie'):
                self.session_pd_verified=False
                self.build_maps()
            return result

    def monitor(self):
        deadline=time.monotonic()
        while self.running:
            with self.lock:
                try:
                    super().monitor_once()
                    if self.ctrl.is_enabled and self.mode in ('IDLE','SHADOW'):
                        self.check_posture_imu()
                    errors=[b.control_error for b in self.ctrl.buses if b is not None and b.control_error]
                    if errors:
                        self.latch_fault('; '.join(errors),self.snaps())
                        self.mode='FAULT';self.close_log('; '.join(errors))
                    if self.fault and self.mode in ('PLAY','CHECK','STOPPING'):
                        self.mode='FAULT';self.close_log(self.fault)
                    self.tick()
                    if self.pad:
                        try:
                            event=self.pad.poll();self.selected=direction(self.pad.axes)
                            if self.gamepad_error==getattr(self,'last_gamepad_transport_error',None):
                                self.gamepad_error=None
                            if self.cfg.get('stick_control',False):
                                axes,buttons,armed=self.pad.snapshot()
                                self.selected=direction(axes)
                                ready=(armed and self.mode=='IDLE' and self.ctrl.is_enabled
                                    and not self.ctrl.is_moving and self.session_pd_verified and not self.fault)
                                active=self.mode=='PLAY' and not self.decelerating
                                stick_event=self.stick_gate.step(axes,buttons,ready,active,event)
                                # RB+Y no longer starts playback in direct-stick mode.
                                if event=='play':event=None
                                if stick_event:event=stick_event
                        except Exception as exc:
                            self.gamepad_error=str(exc)
                            self.last_gamepad_transport_error=self.gamepad_error
                            if self.mode in ('PLAY','CHECK','STOPPING'):raise
                            event=None
                        if event:
                            try:
                                if event=='emergency-stop':self.execute('emergency-stop')
                                elif event=='lie':self.execute('lie')
                                elif event=='posture':self.execute('lie' if self.ctrl.is_enabled else 'stand')
                                elif event=='policy-stop':self.execute('policy-stop')
                                elif event=='play':self.execute('policy-start',{'clip':self.selected})
                                self.gamepad_error=None
                            except RuntimeError as exc:self.gamepad_error='按键命令未执行：'+str(exc)
                except Exception as exc:
                    self.last_policy_error=str(exc)
                    if self.mode in ('PLAY','CHECK','STOPPING') or self.ctrl.is_enabled or self.ctrl.is_moving:
                        self.latch_fault(str(exc),self.snaps());self.mode='FAULT'
                    elif self.mode=='SHADOW':self.mode='IDLE'
                    self.close_log(str(exc))
            deadline+=.02
            time.sleep(max(0.,deadline-time.monotonic()))
            if time.monotonic()-deadline>.02:deadline=time.monotonic()

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('command',choices=['serve','status','init-lie','restore-lie','stand','lie','release',
        'clear-fault','stop','shadow','play','policy-stop','servo-check'])
    p.add_argument('--gamepad',action='store_true')
    p.add_argument('--clip',default='forward')
    p.add_argument('--reference-report',help='本次带时间戳的趴姿交接记录；只恢复软件零位，不使能')
    a=p.parse_args()
    key=hashlib.sha256(str(ROOT).encode()).hexdigest()[:10]
    path=Path('/tmp')/('rcdog27-%d-%s.sock'%(os.getuid(),key))
    if a.command=='serve':
        a.sim=False;session=Deployment(a.gamepad)
        serve(a,path,session);return
    request={'command':{'shadow':'policy-shadow','play':'policy-start'}.get(a.command,a.command),'clip':a.clip}
    if a.reference_report:request['reference_report']=a.reference_report
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
        s.settimeout(3);s.connect(str(path));s.sendall((json.dumps(request)+'\n').encode())
        with s.makefile('rb') as f:result=json.loads(f.readline(65536))
    print(json.dumps(result,ensure_ascii=False,indent=2))
    if not result.get('ok'):raise SystemExit(1)

if __name__=='__main__':main()
