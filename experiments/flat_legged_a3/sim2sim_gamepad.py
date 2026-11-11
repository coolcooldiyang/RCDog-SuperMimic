"""Continuous commands in CPU MuJoCo only. No real robot control."""
import argparse,json,time,queue,math
from pathlib import Path
import numpy as np
import mujoco
from command_sim import Actor,CommandSim,ROOT
from command_reference import CommandReference
from joystick_input import Joystick,axis

class Simulation:
 def __init__(self,actor):
  self.actor=actor;self.reset()
 def reset(self):
  self.sim=CommandSim(self.actor);self.model=self.sim.model;self.data=self.sim.data
  self.stand_q=self.sim.q0.copy();self.lie_q=self.stand_q.copy()
  for i,sign in enumerate((1,-1,1,-1)):
   self.lie_q[3*i+1]=sign*1.5;self.lie_q[3*i+2]=-sign*1.15
  self.data.qpos[2]=.085;self.data.qpos[7:]=self.lie_q;mujoco.mj_forward(self.model,self.data)
  self.mode='LIE';self.target=self.lie_q.copy();self.kp=60.;self.kd=3.;self.command=np.zeros(3);self.pending_lie=False;self.stop_frames=0
 def pose(self,target,final,seconds=6.):
  self.start_q=self.target.copy();self.end_q=target.copy();self.start_gain=(self.kp,self.kd);self.start_time=self.data.time;self.seconds=seconds;self.final=final;self.mode='TRANSITION'
 def stand(self):
  if self.mode=='LIE':self.pose(self.stand_q,'STAND')
 def lie(self):
  self.command[:]=0
  if self.mode=='POLICY':self.pending_lie=True;self.stop_frames=0
  elif self.mode=='STAND':self.pose(self.lie_q,'LIE')
 def play(self):
  if self.mode!='STAND':return
  self.sim.planner=CommandReference(ROOT/'reference_candidates/legged_control/command_gait_table.npz')
  R=np.empty(9);mujoco.mju_quat2Mat(R,self.data.qpos[3:7]);R=R.reshape(3,3)
  self.sim.planner.heading=math.atan2(R[1,0],R[0,0])-math.pi/2
  self.sim.history.clear();self.sim.previous[:]=0;self.sim.frames=0;self.sim.fault=None;self.sim.target=self.target.copy();self.sim.filtered[:]=0
  self.sim.samples.clear();self.sim.samples.append((self.data.qpos.copy(),self.data.qvel.copy()))
  self.mode='POLICY';self.kp=20.;self.kd=.5;self.pending_lie=False
 def step(self):
  if self.mode=='POLICY':
   self.sim.step(np.zeros(3) if self.pending_lie else self.command);self.target=self.sim.target.copy()
   if self.sim.fault:self.mode='FAULT';self.command[:]=0;return
   if self.pending_lie:
    self.stop_frames+=1
    if self.stop_frames>=150:self.pending_lie=False;self.pose(self.stand_q,'LIE_WAIT',1.)
   return
  if self.mode=='TRANSITION':
   u=np.clip((self.data.time-self.start_time)/self.seconds,0,1);u=.5-.5*np.cos(np.pi*u)
   self.target=(1-u)*self.start_q+u*self.end_q;self.kp=(1-u)*self.start_gain[0]+u*60;self.kd=(1-u)*self.start_gain[1]+u*3
   if self.data.time-self.start_time>=self.seconds:
    self.mode=self.final
    if self.mode=='LIE_WAIT':self.pose(self.lie_q,'LIE')
  for _ in range(20):
   self.data.ctrl[:]=0 if self.mode=='FAULT' else np.clip(self.kp*(self.target-self.data.qpos[7:])-self.kd*self.data.qvel[6:],-self.sim.effort,self.sim.effort)
   mujoco.mj_step(self.model,self.data)
  R=np.empty(9);mujoco.mju_quat2Mat(R,self.data.qpos[3:7])
  if not np.isfinite(self.data.qpos).all() or R[8]<np.cos(.75):self.mode='FAULT'
 def snapshot(self):return {'mode':self.mode,'sim_time':self.data.time,'requested_command':self.command.tolist(),'filtered_command':self.sim.planner.command.tolist(),'qpos':self.data.qpos.tolist(),'fault':self.sim.fault,'policy_kp':20,'policy_kd':.5,'physics_hz':1000,'policy_hz':50,'real_hardware_accessed':False}

def physical_controller_active():
 for proc in Path('/proc').glob('[0-9]*/cmdline'):
  try:
   s=proc.read_bytes().replace(b'\0',b' ').decode(errors='ignore')
   if ('dogdeploy' in s or 'dog_motion.deploy050' in s or 'deploy_newurdf.py' in s) and 'serve' in s and 'python' in s:return True
  except (OSError,PermissionError):pass
 return False

def main():
 p=argparse.ArgumentParser();p.add_argument('--actor',type=Path,default=ROOT.parents[1]/'real_deploy/policies/newurdf_legged_a3_1000/actor_fp32.onnx');p.add_argument('--cap',type=float,default=.5,choices=[.5,1.5]);p.add_argument('--joystick',default='/dev/input/js0');p.add_argument('--keyboard-only',action='store_true');p.add_argument('--headless-posture-check',action='store_true');a=p.parse_args()
 (ROOT.parents[1]/'outputs').mkdir(exist_ok=True)
 sim=Simulation(Actor(a.actor))
 if a.headless_posture_check:
  for _ in range(100):sim.step()
  sim.stand()
  for _ in range(350):sim.step()
  assert sim.mode=='STAND',sim.snapshot()
  stood=sim.snapshot();sim.play()
  for _ in range(150):sim.step()
  assert sim.mode=='POLICY',sim.snapshot()
  sim.lie()
  for _ in range(600):sim.step()
  assert sim.mode=='LIE',sim.snapshot()
  (ROOT.parents[1]/'outputs/flat_legged_a3_posture_check.json').write_text(json.dumps({'stand':stood,'final':sim.snapshot()},indent=2));print('stand / policy / stop / lie passed');return
 if physical_controller_active():raise SystemExit('检测到实机 dogdeploy 服务。请先让实机趴下并停止该服务，再使用同一手柄控制仿真。')
 import mujoco.viewer
 events=queue.Queue();pad=None if a.keyboard_only else Joystick(a.joystick);retry=0.;lastmode=None
 print('Experimental a3: full stability58/60, gait57/60; cap=',a.cap)
 print('LB+A 起立 | RB+Y 接入策略 | 左摇杆前后/横移 | 右摇杆旋转 | 松杆减速停止 | B 趴下\n键盘: 1起立 2接策略 B趴下 WASD移动 QE旋转 空格停；键盘指令保持至空格')
 with mujoco.viewer.launch_passive(sim.model,sim.data,key_callback=events.put) as viewer:
  viewer.cam.distance=2.2;viewer.cam.elevation=-20;viewer.cam.azimuth=130
  while viewer.is_running():
   started=time.monotonic()
   if pad and started>=retry:
    try:
     event=pad.poll()
     if event=='posture':sim.stand() if sim.mode=='LIE' else sim.lie()
     elif event=='lie':sim.lie()
     elif event=='play':sim.play()
     elif event=='emergency-stop':sim.mode='FAULT'
     if pad.armed:
      sim.command=np.array([axis(pad.axes,1)*a.cap,axis(pad.axes,0)*a.cap,axis(pad.axes,3)*a.cap])
    except (OSError,RuntimeError) as e:
     print('手柄:',e);pad.close();pad.armed=False;sim.command[:]=0;retry=started+1
   while not events.empty():
    key=events.get()
    if key==49:sim.stand()
    elif key==50:sim.play()
    elif key in (66,98):sim.lie()
    elif key==32:sim.command[:]=0
    elif key in (87,83,65,68,81,69):
     sim.command=np.array({87:[a.cap,0,0],83:[-a.cap,0,0],65:[0,a.cap,0],68:[0,-a.cap,0],81:[0,0,a.cap],69:[0,0,-a.cap]}[key])
   sim.step()
   if sim.mode!=lastmode:print('状态:',sim.mode,sim.sim.fault or '');lastmode=sim.mode
   viewer.cam.lookat[:]=sim.data.qpos[:3];viewer.sync()
   (ROOT.parents[1]/'outputs/flat_legged_a3_status.json').write_text(json.dumps(sim.snapshot()))
   time.sleep(max(0,.02-(time.monotonic()-started)))
 if pad:pad.close()
if __name__=='__main__':main()
