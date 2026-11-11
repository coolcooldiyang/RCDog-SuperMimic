"""CPU MuJoCo continuous-command policy runner; never accesses real robot buses."""
from pathlib import Path
import os
from collections import deque
import numpy as np
import mujoco
from command_reference import CommandReference
ROOT=Path(__file__).resolve().parent
MODEL=ROOT/'Uni-SuperMimic/src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'
class Actor:
 def __init__(self,checkpoint):
  self.runtime=None
  if str(checkpoint).endswith('.onnx'):
   import onnxruntime as ort
   opts=ort.SessionOptions();opts.intra_op_num_threads=2;opts.inter_op_num_threads=1
   self.runtime=ort.InferenceSession(str(checkpoint),sess_options=opts,providers=['CPUExecutionProvider']);return
  import torch,sys
  sys.path.insert(0,str(ROOT/'Uni-SuperMimic/src'))
  from unisupermimic.rl.algos.supermimic import networks
  torch.set_num_threads(2);networks.AMP_DTYPE=None
  c=torch.load(checkpoint,map_location='cpu',weights_only=True);args=c['args'];self.torch=torch
  self.net=networks.Policy(48,390,hist=5,z_dim=int(args['z_dim']),num_joints=12,hidden=tuple(args['policy_hidden'])).eval();self.net.load_state_dict(c['pi'])
 def __call__(self,history,goal):
  if self.runtime is not None:return self.runtime.run(None,{"obs_hist":np.asarray(history,dtype=np.float32)[None],"goal":np.asarray(goal,dtype=np.float32)[None]})[0][0]
  with self.torch.inference_mode():return self.net(self.torch.tensor(history,dtype=self.torch.float32)[None],self.torch.tensor(goal,dtype=self.torch.float32)[None])[0].numpy()
class CommandSim:
 def __init__(self,actor,delay_ms=0,lag_ms=0,quantized=False,seed=0,noise=False,kp=20.0,kd=0.5,speed_limits=None,rate_limit=None,feedback_delay_ms=None):
  self.actor=actor;self.model=mujoco.MjModel.from_xml_path(str(MODEL));self.model.opt.timestep=.001;self.data=mujoco.MjData(self.model);mujoco.mj_resetDataKeyframe(self.model,self.data,0)
  self.planner=CommandReference(Path(os.environ.get('RCDOG_GAIT_TABLE',str(ROOT/'command_gait_table.npz'))));self.q0=self.planner.q0[7:].copy();self.history=deque(maxlen=5);self.previous=np.zeros(12);self.target=self.q0.copy();self.delay=int(delay_ms);self.feedback_delay=self.delay if feedback_delay_ms is None else int(feedback_delay_ms);self.lag=lag_ms/1000;self.quantized=quantized;self.rng=np.random.default_rng(seed);self.noise=noise
  if not (np.isfinite(kp) and np.isfinite(kd) and kp>0 and kd>0):raise ValueError('PD gains must be finite and positive')
  self.kp=np.full(12,float(kp));self.kd=np.full(12,float(kd));self.ratio2=np.tile([6.33**2,6.33**2,12.66**2],4)
  if quantized:self.kp=np.floor(self.kp/self.ratio2*1280)/1280*self.ratio2;self.kd=np.floor(self.kd/self.ratio2*1280)/1280*self.ratio2
  self.effort=np.tile([23.7,23.7,42.66],4);self.filtered=np.zeros(12);self.frames=0;self.fault=None;self.target_step=0.
  self.rate_limit=(os.environ.get('RCDOG_POSITION_RATE_LIMIT','1')=='1') if rate_limit is None else rate_limit
  self.target_velocity_limits=np.tile([30.1,30.1,15.05],4)
  self.speed_limits=(self.target_velocity_limits.copy() if self.rate_limit else np.tile([8.,10.,10.],4)) if speed_limits is None else np.asarray(speed_limits,float)
  if self.speed_limits.shape!=(12,) or not np.isfinite(self.speed_limits).all() or np.any(self.speed_limits<=0) or np.any(self.speed_limits>np.tile([30.1,30.1,15.05],4)):raise ValueError('Invalid model diagnostic speed limits')
  self.samples=deque([(self.data.qpos.copy(),self.data.qvel.copy())]*(self.feedback_delay+1),maxlen=self.feedback_delay+1);self.targets=deque([self.target.copy()]*(self.delay+1),maxlen=self.delay+1)
  self.foot_ids=[self.model.body(l+'_foot').id for l in ['rr','rl','fr','fl']]
 def step(self,command):
  if self.planner.family=='legged_control_trot' and not np.any(command):self.planner.command[:]=0.
  qp,qv=self.samples[0];quat=qp[3:7];R=np.empty(9);mujoco.mju_quat2Mat(R,quat);R=R.reshape(3,3)
  obs=np.r_[qp[7:]-self.q0,qv[6:],qv[3:6],-R[2,:],self.previous,(R.T@self.planner.rotation())[:,:2].reshape(6)]
  if self.noise:obs+=self.rng.uniform(-1,1,48)*np.r_[np.full(12,.01),np.full(12,.5),np.full(3,.2),np.full(3,.05),np.zeros(18)]
  if not self.history:self.history.extend([obs.copy() for _ in range(5)])
  else:self.history.append(obs)
  raw_action=self.actor(np.asarray(self.history),self.planner.goal(command))
  desired=np.clip(self.q0+.25*raw_action,-1.55,1.55)
  if self.rate_limit:
   applied=(desired-self.q0)/.25;delta=self.target_velocity_limits*.02/.25
   self.previous=self.previous+np.clip(applied-self.previous,-delta,delta)
  else:self.previous=raw_action
  target=np.clip(self.q0+.25*self.previous,-1.55,1.55)
  self.target_step=float(np.max(abs(target-self.target)));self.target=target
  for _ in range(20):
   self.targets.append(self.target.copy());delayed=self.targets[0];torque=np.clip(self.kp*(delayed-self.data.qpos[7:])-self.kd*self.data.qvel[6:],-self.effort,self.effort)
   alpha=1. if not self.lag else 1-np.exp(-.001/self.lag);self.filtered+=alpha*(torque-self.filtered);self.data.ctrl[:]=self.filtered;mujoco.mj_step(self.model,self.data)
   self.samples.append((self.data.qpos.copy(),self.data.qvel.copy()))
   if not np.isfinite(self.data.qpos).all():self.fault='nonfinite';break
   if np.any(abs(self.data.qvel[6:])>self.speed_limits):self.fault='joint_speed';break
   if np.any(abs(self.data.qpos[7:])>1.57):self.fault='joint_angle';break
  self.planner.advance(command);self.frames+=1
  R=np.empty(9);mujoco.mju_quat2Mat(R,self.data.qpos[3:7]);R=R.reshape(3,3);tilt=float(np.degrees(np.arccos(np.clip(R[2,2],-1,1))))
  if self.data.qpos[2]<.20 or tilt>30:self.fault=self.fault or 'height_or_tilt'
  measured=np.r_[-R[:,1]@self.data.qvel[:3],R[:,0]@self.data.qvel[:3],(R@self.data.qvel[3:6])[2]]
  return {'time':self.frames*.02,'measured':measured,'command':self.planner.command.copy(),'tilt_deg':tilt,'root_height':float(self.data.qpos[2]),'joint_speed_max':float(max(abs(self.data.qvel[6:]))),'target_step':self.target_step,'qpos':self.data.qpos.copy(),'qvel':self.data.qvel.copy(),'action':self.previous.copy(),'torque':self.data.ctrl.copy()}
