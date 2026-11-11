"""Exercise Deployment.begin/tick/stop and NativePD against a CPU MuJoCo plant.

No real buses, serial ports, or IMU are opened. Delay and transport are assumed
models; passing this check does not independently establish hardware zeroes.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import mujoco
import numpy as np
import yaml

from deploy_050 import Deployment
from motion_cli import ROOT
from dog_motion.deploy050 import Actor, Transmission, LEGS, GEAR, quaternion_matrix,joint_gains

MODEL=ROOT.parents[1]/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'


class PlantBus:
    def __init__(self, plant, leg):
        self.plant=plant;self.leg=leg;self.code=leg['code'];self.i=LEGS.index(self.code)
        self.sl=slice(self.i*3,self.i*3+3);self.drive=None;self.control_error=None
        c=plant.cfg['legs'][self.code]
        theta=np.array(leg['directions'])*np.deg2rad(c['measured_stand_relative_motor_output_deg'])
        self.tr=Transmission(c['A_joint_from_motor_output'],theta,plant.actor.q0[self.sl])
        self.cmds=[None]*3

    def snapshot(self):
        qp,qv,tau=self.plant.history[0]
        qr=self.tr.target_rotor(qp[7:][self.sl]);dqr=GEAR*self.tr.inverse@qv[6:][self.sl]
        motor_tau=self.tr.A.T@tau[self.sl]
        return [dict(id=j+1,online=True,err=0,temp=27,enabled=True,zero_valid=True,
            zero_rotor_rad=0.,q_rotor_rad=float(qr[j]),direction=self.leg['directions'][j],
            q_deg=float(math.degrees(qr[j]/GEAR)*self.leg['directions'][j]),
            dq_deg=float(math.degrees(dqr[j]/GEAR)),feedback_age_s=self.plant.delay*.001,
            tau_feedback_out=float(motor_tau[j])) for j in range(3)]

    def set_joint_drive(self, drive):self.drive=drive
    def update_joint_drive(self,q,kp,kd,effort):self.drive.update(q,kp,kd,effort)
    def freeze_joint_reference(self):return self.drive.freeze_reference()
    def set_gains(self,kp,kd):pass
    def set_mode(self,mode):
        if mode=='hybrid':self.drive=None

    def step_commands(self, step):
        # Three serialized packets per leg, four independent legs. Stagger
        # each motor's update instead of updating all twelve simultaneously.
        qp,qv,tau=self.plant.history[0]
        feedback_qr=self.tr.target_rotor(qp[7:][self.sl])
        feedback_dqr=GEAR*self.tr.inverse@qv[6:][self.sl]
        if self.drive:
            for j in range(3):
                if self.cmds[j] is None or step%self.plant.period==j%self.plant.period:
                    c=self.drive.motor_command(feedback_qr,feedback_dqr,self.tr.A.T@tau[self.sl])
                    self.cmds[j]={k:float(c[k][j]) for k in ('q','kp','kd','dq','tau')}
        else:
            mi=self.plant.codes.index(self.code)*3
            targets=self.plant.ctrl.pose[mi:mi+3]
            for j in range(3):
                self.cmds[j]=dict(q=self.leg['directions'][j]*math.radians(targets[j])*GEAR,
                    kp=self.plant.ctrl.kp/GEAR**2,kd=self.plant.ctrl.kd/GEAR**2,dq=0,tau=0)
        qr=self.tr.target_rotor(self.plant.data.qpos[7:][self.sl])
        dqr=GEAR*self.tr.inverse@self.plant.data.qvel[6:][self.sl]
        commands=self.cmds
        if self.plant.gain_quantization!='ideal':
            # Verified against the deployed GO SDK's in-memory packet encoder:
            # gain integers = trunc(gain * 1280), position = trunc(q * 32768/2pi).
            round_gain=np.floor if self.plant.gain_quantization=='floor' else lambda x:np.floor(x+.5)
            commands=[dict(c,kp=round_gain(c['kp']*1280)/1280,
                kd=round_gain(c['kd']*1280)/1280,
                q=np.trunc(c['q']*32768/(2*np.pi))*(2*np.pi)/32768) for c in self.cmds]
        output=np.array([GEAR*(c['kp']*(c['q']-qr[j])+c['kd']*(c['dq']-dqr[j])+c['tau'])
                         for j,c in enumerate(commands)])
        return np.linalg.solve(self.tr.A.T,output)


class PlantIMU:
    def __init__(self,plant):self.plant=plant
    def snapshot(self):
        qp,qv,_=self.plant.history[0]
        mount=np.asarray(self.plant.cfg['imu']['R_CI'])
        R=quaternion_matrix(qp[3:7])@mount
        quat=np.zeros(4);mujoco.mju_mat2Quat(quat,R.reshape(9))
        return dict(quat_wxyz=quat,gyro_rad_s=mount.T@qv[3:6],age_s=self.plant.delay*.001)


class NativePlant:
    def __init__(self,delay_ms=8,period_ms=4,speed_limits=None,friction=1.,mass_scale=1.,gain_quantization='ideal',config_path=None,model_path=None,actor_class=Actor,deployment_class=Deployment):
        if gain_quantization not in ('ideal','floor','nearest'):raise ValueError('gain_quantization')
        self.gain_quantization=gain_quantization
        self.cfg=yaml.safe_load(Path(config_path or ROOT/'config/deploy_newurdf.yaml').read_text())
        # In-memory test preconditions only. Never modifies the live gates.
        self.cfg.update(native_pd_verified=True,sim2sim_verified=True,absolute_joint_reference_verified=True,commissioning_allowed_clips=[])
        if speed_limits:self.cfg['commissioning_speed_limits_rad_s']=list(speed_limits)
        self.actor=actor_class(ROOT/self.cfg['bundle']);self.now=100.
        self.model=mujoco.MjModel.from_xml_path(str(model_path or MODEL));self.model.opt.timestep=.001
        self.model.geom_friction[:,0]=friction
        self.model.body_mass[1]*=mass_scale;self.model.body_inertia[1]*=mass_scale
        self.data=mujoco.MjData(self.model);mujoco.mj_setConst(self.model,self.data)
        mujoco.mj_resetDataKeyframe(self.model,self.data,0)
        self.effort=np.tile([23.7,23.7,42.66],4)
        calibration=ROOT/('config' if (ROOT/'config/leg_direction_mapping.yaml').exists() else 'examples')
        legs=yaml.safe_load((calibration/'leg_direction_mapping.yaml').read_text())['legs']
        for leg in legs:leg['ids']=[1,2,3];leg['joints']=['髋','大腿','小腿']
        self.codes=[leg['code'] for leg in legs]
        stand=yaml.safe_load((calibration/'stand_poses.yaml').read_text())['stand']
        self.ctrl=SimpleNamespace(legs=legs,buses=[],is_enabled=True,is_moving=False,kp=60.,kd=3.,stand=stand,pose=stand)
        self.ctrl.apply_pose=lambda pose:setattr(self.ctrl,'pose',list(pose))
        self.delay=int(round(delay_ms));self.period=int(round(period_ms))
        self.history=deque([(self.data.qpos.copy(),self.data.qvel.copy(),np.zeros(12))]*(self.delay+1),maxlen=self.delay+1)
        self.ctrl.buses=[PlantBus(self,leg) for leg in legs]
        # Settle under the actual legacy motor-output-side stand controller.
        for k in range(1000):self.physics(k)
        d=deployment_class.__new__(deployment_class)
        d.ctrl=self.ctrl;d.cfg=self.cfg;d.actor=self.actor;d.mode='IDLE';d.initialized=True;d.fault=None
        d.maps={};d.R_WN=None;d.imu=PlantIMU(self);d.log=None;d.inference_ms=[]
        d.last_policy_error=None;d.decelerating=False;d.pending_lie=False
        d.policy_kp=float(self.cfg['policy_kp_joint']);d.policy_kd=joint_gains(self.cfg['policy_kd_joint'])
        d.report_directory=ROOT/'reports/native_assessment_simulation'
        self.session=d

    def physics(self,step):
        for bus in self.ctrl.buses:self.data.ctrl[bus.sl]=bus.step_commands(step)
        mujoco.mj_step(self.model,self.data)
        self.history.append((self.data.qpos.copy(),self.data.qvel.copy(),self.data.qfrc_actuator[6:].copy()))
        self.now+=.001

    def run(self,clip,stop_time=None,max_duration_s=15.):
        s=self.session;rows=[];peak=np.zeros(12);failure=None;finished=None
        s.cfg['commissioning_allowed_clips']=[clip]
        with patch('time.monotonic',lambda:self.now):
            s.begin('PLAY',clip);s.close_log('offline_simulation')
            ref=s.actor.reference(clip)
            for step in range(round(max_duration_s*1000)):
                t=step*.001
                try:
                    if step%20==0:
                        if stop_time is not None and t>=stop_time and not getattr(self,'stop_sent',False):
                            s.begin_stop();self.stop_sent=True
                        s.tick()
                    self.physics(step)
                    peak=np.maximum(peak,np.abs(self.data.qvel[6:]))
                    if step%20==19:
                        R=quaternion_matrix(self.data.qpos[3:7])
                        tilt=math.acos(float(np.clip(R[2,2],-1,1)))
                        rows.append([t,*self.data.qpos[:3],tilt,math.atan2(R[1,0],R[0,0])])
                        if self.data.qpos[2]<.18 or tilt>math.radians(self.cfg['tilt_limit_deg']):
                            raise RuntimeError('body_height_or_tilt')
                    if s.mode=='IDLE':
                        if finished is None:finished=t
                        if t-finished>.5:break
                except RuntimeError as exc:
                    failure=str(exc);break
        rows=np.asarray(rows)
        steady=(rows[:,0]>=2.2)&(rows[:,0]<=6.5)
        measured=np.polyfit(rows[steady,0],rows[steady,1:3],1)[0] if steady.sum()>5 else None
        measured_yaw=float(np.polyfit(rows[steady,0],np.unwrap(rows[:,5])[steady],1)[0]) if steady.sum()>5 else None
        times=np.arange(len(ref['goal']))*.02;window=(times>=2.2)&(times<=6.5)
        vref=np.polyfit(times[window],ref['root_p_ref'][window,:2],1)[0]
        yawref=np.unwrap(np.arctan2(ref['R_ref'][:,1,0],ref['R_ref'][:,0,0]))
        wref=float(np.polyfit(times[window],yawref[window],1)[0])
        tracking=(stop_time is not None or (measured is not None and np.linalg.norm(measured-vref)<.08 and abs(measured_yaw-wref)<.15))
        return dict(clip=clip,stop_requested_s=stop_time,passes=bool(failure is None and finished is not None and tracking),
            failed=failure,finished_s=finished,tracking_ok=bool(tracking),
            measured_velocity_xy_mps=measured.tolist() if measured is not None else None,
            measured_yaw_rate_rad_s=measured_yaw,peak_joint_speed_rad_s=peak.tolist(),
            max_tilt_deg=math.degrees(float(rows[:,4].max())),min_height_m=float(rows[:,3].min()),
            delay_ms=self.delay,motor_period_ms=self.period,live_config_changed=False,
            gain_quantization=self.gain_quantization,
            speed_limits=self.cfg['commissioning_speed_limits_rad_s'],actor_sha256=self.actor.actor_sha256)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--clips',nargs='+',default=['forward','forward_left','left','back_left','back','back_right','right','forward_right','ccw','cw'])
    p.add_argument('--delay-ms',type=float,default=8)
    p.add_argument('--period-ms',type=float,default=4)
    p.add_argument('--speed-limits',type=float,nargs=3)
    p.add_argument('--stops',type=float,nargs='*',default=[.2,.7,2.,4.,6.])
    p.add_argument('--all-clips-stops',action='store_true')
    p.add_argument('--friction',type=float,default=1.)
    p.add_argument('--mass-scale',type=float,default=1.)
    p.add_argument('--gain-quantization',choices=['ideal','floor','nearest'],default='ideal')
    a=p.parse_args();runs=[]
    cases=[(name,None) for name in a.clips]
    cases += [(name,t) for name in (a.clips if a.all_clips_stops else ['forward']) for t in a.stops]
    for name,t in cases:
        plant=NativePlant(a.delay_ms,a.period_ms,a.speed_limits,a.friction,a.mass_scale,a.gain_quantization)
        result=plant.run(name,t);runs.append(result);print(json.dumps(result),flush=True)
    report=dict(passed=sum(r['passes'] for r in runs),total=len(runs),all_passed=all(r['passes'] for r in runs),
        actual_deployment_begin_tick_stop_reused=True,real_devices_opened=False,
        mass_scale=a.mass_scale,friction=a.friction,mujoco_version=mujoco.__version__,runs=runs)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='runs'}))


if __name__=='__main__':main()
