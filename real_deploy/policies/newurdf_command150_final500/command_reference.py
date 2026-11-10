"""The same continuous reference generator is used for data and joystick sim2sim."""
from pathlib import Path
import numpy as np
class CommandReference:
 def __init__(self,table):
  z=np.load(table);self.table=z['q'];self.grids=[z['grid_x'],z['grid_y'],z['grid_w']];self.q0=z['q0'];self.period=float(z['period']);self.phase=0.;self.command=np.zeros(3);self.position=self.q0[:3].copy();self.heading=0.;self.yaw0=2*np.arctan2(self.q0[6],self.q0[3])
 def copy(self):
  import copy
  p=copy.copy(self);p.command=self.command.copy();p.position=self.position.copy();return p
 def advance(self,requested,dt=.02):
  requested=np.asarray(requested,float).copy();requested=np.clip(requested,[-1.5,-1.5,-1.5],[1.5,1.5,1.5])
  requested[:2]*=min(1.,1.5/max(1e-12,np.linalg.norm(requested[:2])))
  delta=requested-self.command;limit=np.array([.5,.5,1.])*dt;delta=np.clip(delta,-limit,limit);self.command+=delta
  self.command[:2]*=min(1.,1.5/max(1e-12,np.linalg.norm(self.command[:2])))
  c,s=np.cos(self.heading),np.sin(self.heading);self.position[:2]+=np.array([[c,-s],[s,c]])@self.command[:2]*dt;self.heading+=self.command[2]*dt;self.phase=(self.phase+dt/self.period)%1
 def joints(self):
  lo=np.array([np.clip(np.searchsorted(g,c,side="right")-1,0,len(g)-2) for g,c in zip(self.grids,self.command)]);weight=np.array([np.clip((c-g[i])/(g[i+1]-g[i]),0,1) for g,c,i in zip(self.grids,self.command,lo)])
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
