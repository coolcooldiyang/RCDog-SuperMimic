#!/usr/bin/env python3
"""Persistent command-line front end. Only `serve` owns the motor ports."""
from __future__ import annotations
import argparse
from collections import deque
from datetime import datetime
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import socketserver
import sys
import threading
import time
import uuid
import yaml
from zoneinfo import ZoneInfo

ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT/'config/policy_stand.yaml'

def read_policy():
    return yaml.safe_load(POLICY.read_text())

def atomic_yaml(path, data):
    tmp=path.with_suffix('.tmp')
    tmp.write_text(yaml.safe_dump(data,allow_unicode=True,sort_keys=False))
    os.replace(tmp,path)

class Session:
    def __init__(self, sim=False):
        from dog_motion import MotionController
        self.ctrl=MotionController.from_config(sim=sim)
        import unitree_actuator_sdk as sdk
        self.sdk_releases_gil=bool(getattr(sdk,'send_recv_releases_gil',False))
        self.sim=sim
        self.session_id=uuid.uuid4().hex
        self.calfile=ROOT/'config'/('policy_encoder_reference.sim.yaml' if sim else 'policy_encoder_reference.yaml')
        self.initialized=False
        from dog_motion.policy_pose import has_confirmed_default_stand
        self.default_stand_ready=has_confirmed_default_stand(self.ctrl.pose_path)
        self.policy_ready=False
        self.fault=None
        self.lock=threading.RLock()
        self.running=True
        self.pose_source=None
        self.fault_detail=None
        self.fault_dir=ROOT/'reports/faults';self.fault_dir.mkdir(parents=True,exist_ok=True)
        self.recent_samples=deque(maxlen=250)  # 监控线程50Hz，保留约5秒
        self.recent_commands=deque(maxlen=40)
        self.post_fault_captures=[]
        self.post_fault_seconds=3.0

    def snaps(self):
        return [s for b in self.ctrl.buses if b is not None for s in b.snapshot()]

    def require_ready(self, initialized=True):
        s=self.snaps()
        if len(s)!=12 or any(not x['online'] or x['err'] or
                              not math.isfinite(x['q_deg']) for x in s):
            raise RuntimeError('需要全部 12 轴在线、反馈有效且故障码为 0；运行 status 查看')
        if initialized and (not self.initialized or any(not x['zero_valid'] for x in s)):
            raise RuntimeError('本次会话还未建立趴下零位：狗在原录制趴下姿态时运行 init-lie')
        if self.fault:raise RuntimeError('故障已锁存：'+self.fault+'；检查后运行 clear-fault')
        return s

    def zeros(self):
        from dog_motion.bus import GEAR
        # SimulatedBus has no absolute rotor encoder; its coordinates start at 0.
        return [float(x.get('zero_rotor_rad',0.))/GEAR for x in self.snaps()]

    def topology(self):
        return [{'code':l['code'],'port':l['port'],'ids':l['ids'],
                 'directions':l['directions']} for l in self.ctrl.legs]

    def feedback_events(self):
        return [dict(leg=leg['code'],events=bus.feedback_events())
                for leg,bus in zip(self.ctrl.legs,self.ctrl.buses)
                if bus is not None and hasattr(bus,'feedback_events')]

    def capture_fault_aftermath(self, snapshots, *, finish=False):
        """松弛后继续只读记录三秒，保留恢复首帧；从不复位或重新使能。"""
        now=time.monotonic()
        remaining=[]
        for capture in self.post_fault_captures:
            stream=capture['stream']
            complete=finish or now>=capture['deadline']
            row=dict(event='capture_complete' if complete else 'state',
                     monotonic_s=now,motors=snapshots)
            if complete:
                row['feedback_events']=self.feedback_events()
                row['ended_early']=finish and now<capture['deadline']
            try:
                stream.write(json.dumps(row,ensure_ascii=False)+'\n')
                stream.flush()
            except OSError as exc:
                print(json.dumps(dict(event='post_fault_write_error',error=str(exc)),
                                 ensure_ascii=False),flush=True)
                complete=True
            if complete:stream.close()
            else:remaining.append(capture)
        self.post_fault_captures=remaining

    def resolve_target(self):
        from dog_motion.policy_pose import resolve_policy_target
        self.policy_ready=False
        self.ctrl.policy_reference_path=str(self.calfile)
        try:
            policy_target=resolve_policy_target(POLICY,self.calfile,
                self.topology(),self.zeros(),self.sim)
            if not self.default_stand_ready:self.ctrl.stand=policy_target
        except (ValueError,OSError):return False
        self.policy_ready=True
        return True

    def status(self):
        s=self.snaps()
        return dict(mode='SIMULATED_BUS' if self.sim else 'HARDWARE',session=self.session_id,
                    sdk_send_recv_releases_gil=self.sdk_releases_gil,
                    initialized=self.initialized,policy_stand_ready=self.policy_ready,
                    default_stand_ready=self.default_stand_ready,
                    moving=self.ctrl.is_moving,enabled=self.ctrl.is_enabled,
                    online=sum(x['online'] for x in s),total=12,fault=self.fault,
                    fault_detail=self.fault_detail,
                    motion_error=self.ctrl.motion_error,
                    max_target_error_deg=max((abs(x['target_deg']-x['q_deg']) for x in s),default=None),
                    leg_order=[l['code'] for l in self.ctrl.legs],
                    stand_motor_output_relative_deg=self.ctrl.stand if self.policy_ready or self.default_stand_ready else None,
                    pose_source=self.pose_source,
                    motion=self.ctrl._motion['name'] if self.ctrl._motion else None,
                    motors=s)

    def execute(self, command, params=None):
        params=params or {}
        with self.lock:
            if command=='status':return self.status()
            self.recent_commands.append(dict(monotonic_s=time.monotonic(),command=command,
                params={k:v for k,v in params.items() if k!='command'}))
            if command=='hold':
                self.require_ready()
                if not self.ctrl.is_enabled:raise RuntimeError('电机未使能，没有保持目标')
                self.ctrl.stop_motion()
                return {'message':'已停止移动，保持当前目标；没有松弛'}
            if command=='jog-group':
                self.require_ready()
                self.ctrl.start_jog_group(params.get('joint'),float(params.get('delta_deg',0)),
                                           float(params.get('speed_deg_s',2)))
                self.pose_source='manual-tuning'
                return {'message':'四腿对应电机已开始同步微调'}
            if command=='restore-lie':
                if self.sim:raise RuntimeError('假总线不用恢复实机零位')
                if self.ctrl.is_moving or self.ctrl.is_enabled:raise RuntimeError('使能时不能恢复零位')
                self.require_ready(initialized=False)
                source=ROOT/'reports/lying_zero_captured_20261103.json'
                handoff=params.get('reference_report')
                if handoff:
                    source=(ROOT/handoff).resolve()
                    if not source.is_relative_to((ROOT/'reports').resolve()):
                        raise RuntimeError('会话交接记录必须位于本项目 reports 内')
                payload=json.loads(source.read_text())
                saved=payload['status']['result']
                current=self.snaps();previous=saved['motors']
                if (saved['mode']!='HARDWARE' or not saved['initialized'] or
                        saved['leg_order']!=[l['code'] for l in self.ctrl.legs] or len(previous)!=12):
                    raise RuntimeError('先前趴姿零位记录的模式或腿序不一致')
                if handoff:
                    age=time.time()-float(payload['captured_at_unix_s'])
                    if (payload.get('kind')!='software_reference_handoff_v1' or
                            not math.isfinite(age) or not 0<=age<=120 or
                            payload.get('topology')!=self.topology() or
                            saved.get('enabled') or saved.get('moving') or saved.get('fault')):
                        raise RuntimeError('交接记录过期、拓扑不一致或保存时未处于无故障松弛状态')
                from dog_motion.bus import GEAR
                for now,old in zip(current,previous):
                    if (not old['zero_valid'] or now['id']!=old['id'] or now['direction']!=old['direction'] or
                            not math.isfinite(old['zero_rotor_rad']) or
                            abs(now['q_rotor_rad']-old['zero_rotor_rad'])/GEAR>math.radians(3)):
                        raise RuntimeError('当前编码器与已采集趴姿相差超过 3° 或轴映射不符，未恢复零位')
                    if handoff and (not math.isfinite(old['q_rotor_rad']) or
                            abs(now['q_rotor_rad']-old['q_rotor_rad'])/GEAR>math.radians(1)):
                        raise RuntimeError('交接期间实测角度变化超过 1°，未恢复零位')
                for i,bus in enumerate(self.ctrl.buses):
                    bus.restore_zero_rotor([m['zero_rotor_rad'] for m in previous[3*i:3*i+3]])
                self.initialized=True
                self.resolve_target()
                return {'message':'已恢复先前采集的趴姿软件零位；没有使能或重设电机零点'}
            if command=='release':
                self.ctrl.release();return {'message':'全部松弛；零位仍保留'}
            if command=='stop':
                self.ctrl.release();self.running=False;return {'message':'退出状态机并松弛'}
            if command=='clear-fault':
                self.ctrl.release()
                old=self.fault;old_detail=self.fault_detail;self.fault=None
                try:self.require_ready(initialized=False)
                except Exception:self.fault=old;self.fault_detail=old_detail;raise
                self.ctrl.motion_error=None
                self.ctrl.motion_fault_detail=None
                self.fault_detail=None
                return {'message':'故障复位；未使能'}
            if command=='init-lie':
                if self.ctrl.is_moving or self.ctrl.is_enabled:
                    raise RuntimeError('读零前先 release，并将狗摆回原录制趴下姿态')
                self.require_ready(initialized=False)
                n=self.ctrl.zero()
                if n!=12:raise RuntimeError('读零不足 12 轴')
                self.initialized=True;self.policy_ready=False
                self.resolve_target()
                return {'message':'已建立本次趴下零位；未使能',
                        'policy_stand_ready':self.policy_ready}
            if command=='record-policy-stand':
                if self.ctrl.is_moving:
                    raise RuntimeError('先等待移动结束或 hold，再采样站姿')
                self.require_ready()
                q=self.ctrl.current_pose(require_online=True)
                if self.ctrl.is_enabled and max(abs(a['target_deg']-a['q_deg']) for a in self.snaps())>3:
                    raise RuntimeError('实际角度尚未进入目标 3° 容差，暂不保存站姿')
                z=self.zeros()
                directions=[v for l in self.ctrl.legs for v in l['directions']]
                absolute=[a+s*math.radians(b) for a,s,b in zip(z,directions,q)]
                d={'version':1,'recorded_at_policy_q0':True,
                   'policy_sha256':read_policy()['checkpoint_sha256'],
                   'q0_urdf_rad':read_policy()['q0_urdf_rad'],
                   'source':'operator_aligned_to_model_q0','topology':self.topology(),
                   'lie_zero_output_rad':z,'stand_output_rad':absolute,
                   'stand_relative_deg_at_recording':q,'simulated':self.sim}
                atomic_yaml(self.calfile,d)
                if not self.resolve_target():raise RuntimeError('记录后的姿态校验失败')
                return {'message':'已把当前实物姿态绑定为策略 q0；使能状态保持不变','stand_deg':self.ctrl.stand}
            if command not in ('stand','stand-original','lie'):raise ValueError('未知命令')
            self.require_ready()
            if self.ctrl.is_moving:raise RuntimeError('上一个动作尚未完成')
            # Validate the new target BEFORE enabling any position gains.
            if command=='stand' and not (self.policy_ready or self.default_stand_ready):
                raise RuntimeError('策略站姿尚未标定；不会执行旧的 97°/104° 目标。先 record-policy-stand')
            if command=='stand-original':
                from dog_motion.poses import load_poses
                target=load_poses(self.ctrl.pose_path,12)[1]
            else:target=self.ctrl.stand if command=='stand' else self.ctrl.lie
            if not all(math.isfinite(x) and abs(x)<=180 for x in target):
                raise RuntimeError('电机输出轴目标超出本调试器 ±180° 范围')
            if command=='lie' and not self.ctrl.is_enabled:self.ctrl.enable()
            self.ctrl.start_motion(command)
            self.pose_source={'stand-original':'original-state-machine','stand':'policy-calibrated','lie':'lying'}[command]
            return {'message':'已开始'+command,'duration_s':self.ctrl.duration,
                    'target_motor_output_relative_deg':list(target)}

    def latch_fault(self, reason, snapshots):
        """先撤去出力，再保存首个异常及其前约5秒；不因后续恢复丢失现场。"""
        if self.fault:return
        bad=[]
        for i,snap in enumerate(snapshots):
            kinds=[]
            if not snap['online']:kinds.append('feedback_timeout')
            if snap['err']:kinds.append('motor_error_%d'%snap['err'])
            if not math.isfinite(snap['q_deg']):kinds.append('nonfinite_angle')
            if kinds:
                leg=self.ctrl.legs[i//3]
                bad.append(dict(leg=leg['code'],joint=leg['joints'][i%3],
                                id=snap['id'],port=leg['port'],reasons=kinds,
                                feedback_age_s=snap.get('feedback_age_s'),
                                err=snap['err']))
        if len(snapshots)!=12:bad.append(dict(reasons=['incomplete_bus_connections']))
        first_motion_fault=self.ctrl.motion_fault_detail
        axis_names=['%s %s ID%d'%(x['leg'],x['joint'],x['id']) for x in bad if 'leg' in x]
        if not axis_names and first_motion_fault:
            axis_names=['%s %s ID%d'%(first_motion_fault['leg'],first_motion_fault['joint'],
                                    first_motion_fault['feedback']['id'])]
        self.fault=reason+('：'+'、'.join(axis_names) if axis_names else '')
        self.ctrl.release()
        now=datetime.now(ZoneInfo('Asia/Shanghai'))
        self.fault_detail=dict(time=now.isoformat(),reason=reason,bad_axes=bad,
                              motion_fault=first_motion_fault,report_path=None)
        report=dict(session=self.session_id,simulated=self.sim,fault=self.fault,
                    detail=self.fault_detail,trigger_snapshot=snapshots,
                    topology=self.topology(),recent_samples=list(self.recent_samples),
                    recent_commands=list(self.recent_commands),feedback_timeout_s=.1,
                    feedback_events=self.feedback_events(),
                    policy_control_samples=[dict(leg=leg['code'],samples=bus.control_samples())
                        for leg,bus in zip(self.ctrl.legs,self.ctrl.buses)
                        if bus is not None and hasattr(bus,'control_samples')])
        try:
            self.fault_dir.mkdir(parents=True,exist_ok=True)
            path=self.fault_dir/('fault_%s_%s.json'%(now.strftime('%Y%m%d_%H%M%S_%f'),self.session_id[:8]))
            self.fault_detail['report_path']=str(path)
            post_path=path.with_suffix('.post.jsonl')
            self.fault_detail['post_report_path']=str(post_path)
            path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            stream=post_path.open('x')
            self.post_fault_captures.append(dict(stream=stream,
                deadline=time.monotonic()+self.post_fault_seconds))
        except Exception as exc:self.fault_detail['write_error']=str(exc)
        print(json.dumps(dict(event='fault_latched',fault=self.fault,detail=self.fault_detail),
                         ensure_ascii=False),flush=True)

    def monitor_once(self):
        with self.lock:
            s=self.snaps()
            self.recent_samples.append(dict(monotonic_s=time.monotonic(),motors=s,
                motion=self.ctrl._motion['name'] if self.ctrl._motion else None,
                pose_source=self.pose_source))
            if any(x['enabled'] for x in s) or self.ctrl.is_moving:
                if len(s)!=12 or any(not x['online'] or x['err'] or
                                      not math.isfinite(x['q_deg']) for x in s):
                    self.latch_fault('运动或保持期间电机离线／故障',s)
            if self.ctrl.motion_error:
                self.latch_fault(self.ctrl.motion_error,s)
            self.capture_fault_aftermath(self.snaps() if self.fault else s)

    def monitor(self):
        while self.running:
            self.monitor_once()
            time.sleep(.02)

def serve(args, path, session=None):
    session=Session(args.sim) if session is None else session
    lockpath=Path(str(path)+'.lock')
    lock=lockpath.open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    if path.exists():path.unlink()
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            try:
                line=self.rfile.readline(4096)
                request=json.loads(line)
                result={'ok':True,'result':session.execute(request['command'],request)}
            except Exception as exc:result={'ok':False,'error':str(exc)}
            try:self.wfile.write((json.dumps(result,ensure_ascii=False)+'\n').encode())
            except (BrokenPipeError,ConnectionResetError):pass
    server=socketserver.UnixStreamServer(str(path),Handler)
    os.chmod(path,0o600);server.timeout=.1
    def stop_signal(*_):session.running=False
    signal.signal(signal.SIGINT,stop_signal);signal.signal(signal.SIGTERM,stop_signal)
    try:
        online=session.connect_hardware() if hasattr(session,'connect_hardware') else session.ctrl.connect()
        print(json.dumps({'listening':str(path),'online':online,'enabled':False,
                          'message':'未读零、未使能；趴下姿态准备好后运行 init-lie'},ensure_ascii=False),flush=True)
        monitor=threading.Thread(target=session.monitor,daemon=True);monitor.start()
        while session.running:server.handle_request()
    finally:
        session.running=False
        session.ctrl.release()
        session.capture_fault_aftermath(session.snaps(),finish=True)
        if hasattr(session,'close_inputs'):session.close_inputs()
        session.ctrl.close();server.server_close();path.unlink(missing_ok=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--sim',action='store_true',help='Fake bus; never a physical simulation')
    p.add_argument('command',choices=['serve','status','init-lie','restore-lie','record-policy-stand',
                                    'stand','stand-original','lie','hold','jog-group','release','clear-fault','stop','target'])
    p.add_argument('--joint',type=int,choices=[0,1,2])
    p.add_argument('--delta-deg',type=float,default=0.)
    p.add_argument('--speed-deg-s',type=float,default=2.)
    a=p.parse_args()
    if a.command=='target':print(json.dumps(read_policy(),ensure_ascii=False,indent=2));return 0
    key=hashlib.sha256(str(ROOT).encode()).hexdigest()[:10]
    path=Path('/tmp')/('rcdog27-%d-%s%s.sock'%(os.getuid(),key,'-sim' if a.sim else ''))
    if a.command=='serve':serve(a,path);return 0
    try:
        with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as s:
            s.settimeout(10);s.connect(str(path))
            s.sendall((json.dumps({'command':a.command,'joint':a.joint,
                                  'delta_deg':a.delta_deg,'speed_deg_s':a.speed_deg_s})+'\n').encode())
            f=s.makefile('rb');r=json.loads(f.readline())
        print(json.dumps(r,ensure_ascii=False,indent=2))
        return 0 if r['ok'] else 2
    except OSError as exc:
        print('无法连接状态机：先在另一个终端运行 ./dogctl%s serve；%s'%(' --sim' if a.sim else '',exc),file=sys.stderr)
        return 2

if __name__=='__main__':sys.exit(main())
