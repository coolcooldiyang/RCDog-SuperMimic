"""The same continuous reference generator is used for data and joystick sim2sim."""
from pathlib import Path
import numpy as np
class CommandReference:
 def __init__(self,table):
  z=np.load(table);self.table=z['q'];self.grids=[z['grid_x'],z['grid_y'],z['grid_w']];self.q0=z['q0'];self.period=float(z['period']);self.period_table=z['period_table'] if 'period_table' in z else None;self.height_table=z['height_table'] if 'height_table' in z else None;self.duty=float(z['duty']) if 'duty' in z else .5;self.duty_table=z['duty_table'] if 'duty_table' in z else None;self.family=str(z['family'].item()) if 'family' in z else 'legacy';self.phase=0.;self.command=np.zeros(3);self.gait_command=np.zeros(3);self.position=self.q0[:3].copy();self.heading=0.;self.yaw0=2*np.arctan2(self.q0[6],self.q0[3])
 def blend_scalar(self,table):
  lo=np.array([np.clip(np.searchsorted(g,c,side='right')-1,0,len(g)-2) for g,c in zip(self.grids,self.gait_command if self.family=='legged_control_trot' else self.command)]);weight=np.array([np.clip((c-g[i])/(g[i+1]-g[i]),0,1) for g,c,i in zip(self.grids,self.gait_command if self.family=='legged_control_trot' else self.command,lo)])
  value=0.
  for x in range(2):
   for y in range(2):
    for z in range(2):
     bits=np.array([x,y,z]);value+=np.prod(np.where(bits,weight,1-weight))*table[tuple(lo+bits)]
  return float(value)
 def swing_details(self):
  ph=(self.phase+np.array([0,.5,.5,0]))%1;stance=ph<self.duty;s=np.clip((ph-self.duty)/(1-self.duty),0,1)
  B=lambda t:3*t*t-2*t*t*t
  swing=np.where(s<.5,B(2*s),1-B(2*s-1))*(~stance)
  if self.family=='legged_control_trot':
   from legged_gait import swing_z
   height=self.blend_scalar(self.height_table);actual=swing_z(s,self.period*(1-self.duty),height,min(1.,height/.08));swing=actual/max(1e-9,height)*(~stance)
  if np.linalg.norm(self.gait_command if self.family=='legged_control_trot' else self.command)<1e-6:stance[:]=True
  return swing,stance.astype(float),np.full(4,self.blend_scalar(self.height_table))
 def copy(self):
  import copy
  p=copy.copy(self);p.command=self.command.copy();p.gait_command=self.gait_command.copy();p.position=self.position.copy();return p
 def advance(self,requested,dt=.02):
  requested=np.asarray(requested,float).copy();requested=np.clip(requested,[-1.5,-1.5,-1.5],[1.5,1.5,1.5])
  requested[:2]*=min(1.,1.5/max(1e-12,np.linalg.norm(requested[:2])))
  if self.family=='legged_control_trot' and not np.any(requested):
   self.command[:]=0.;self.gait_command+=np.clip(-self.gait_command,-5.*dt,5.*dt)
  else:
   delta=requested-self.command;limit=np.array([.5,.5,1.])*dt;delta=np.clip(delta,-limit,limit);self.command+=delta
   self.gait_command=self.command.copy()
  self.command[:2]*=min(1.,1.5/max(1e-12,np.linalg.norm(self.command[:2])))
  if self.period_table is not None:self.period=self.blend_scalar(self.period_table)
  if self.duty_table is not None:self.duty=self.blend_scalar(self.duty_table)
  c,s=np.cos(self.heading),np.sin(self.heading);self.position[:2]+=np.array([[c,-s],[s,c]])@self.command[:2]*dt;self.heading+=self.command[2]*dt;self.phase=(self.phase+dt/self.period)%1
 def joints(self):
  lo=np.array([np.clip(np.searchsorted(g,c,side="right")-1,0,len(g)-2) for g,c in zip(self.grids,self.gait_command if self.family=='legged_control_trot' else self.command)]);weight=np.array([np.clip((c-g[i])/(g[i+1]-g[i]),0,1) for g,c,i in zip(self.grids,self.gait_command if self.family=='legged_control_trot' else self.command,lo)])
  phase=self.phase*self.table.shape[3];i=int(phase)%self.table.shape[3];j=(i+1)%self.table.shape[3];f=phase-int(phase);q=np.zeros(12)
  for x in range(2):
   for y in range(2):
    for z in range(2):
     bits=np.array([x,y,z]);w=np.prod(np.where(bits,weight,1-weight));ix,iy,iz=lo+bits
     q+=w*((1-f)*self.table[ix,iy,iz,i]+f*self.table[ix,iy,iz,j])
  return q
 def rotation(self):
  a=self.yaw0+self.heading;c,s=np.cos(a),np.sin(a);return np.array([[c,-s,0],[s,c,0],[0,0,1.]])
 def quaternion_xyzw(self):
  a=self.yaw0+self.heading;return np.array([0,0,np.sin(a/2),np.cos(a/2)])
 def goal(self,requested):
  p=self.copy();R=self.rotation();position=self.position.copy();features=[]
  for _ in range(10):
   previous_q=p.joints();previous_position=p.position.copy();previous_heading=p.heading
   p.advance(requested);q=p.joints();future=p.copy();future.advance(requested);qd=(future.joints()-previous_q)/.04
   c,s=np.cos(p.heading),np.sin(p.heading);world_v=(future.position-previous_position)/.04
   features.append(np.r_[R.T@(p.position-position),(R.T@p.rotation())[:,:2].reshape(6),p.rotation().T@world_v,[0,0,(future.heading-previous_heading)/.04],q,qd*.1])
  return np.concatenate(features)
