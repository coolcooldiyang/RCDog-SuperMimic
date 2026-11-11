"""IK-retarget legged_control-style foot trajectories to this RCDog model."""
from pathlib import Path
import json,time
import numpy as np
import mujoco
from scipy.optimize import least_squares
from legged_gait import stance_xy,swing_xy,swing_z

R=Path(__file__).resolve().parent;O=R/'reference_candidates/legged_control';O.mkdir(parents=True,exist_ok=True)
m=mujoco.MjModel.from_xml_path(str(R.parents[1]/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'))
d=mujoco.MjData(m);q0=m.key_qpos[0].copy();d.qpos[:]=q0;mujoco.mj_forward(m,d)
ids=[m.body(l+'_foot').id for l in ['rr','rl','fr','fl']];feet=d.xpos[ids].copy();nominal=feet[:,:2]-q0[:2]
g=np.array([-1.5,-1.,-.5,-.25,-.1,0.,.1,.25,.5,1.,1.5]);wgrid=np.array([-1.5,-.75,0.,.75,1.5]);N=60
shape=(len(g),len(g),len(wgrid));table=np.zeros((*shape,N,12));periods=np.zeros(shape);heights=np.zeros(shape);duties=np.zeros(shape)
jp=np.zeros((3,m.nv));jr=np.zeros_like(jp);worst=0.;max_speed=0.;max_q=0.;started=time.monotonic()
for ix,vx in enumerate(g):
 for iy,vy in enumerate(g):
  for iw,yaw in enumerate(wgrid):
   velocity=np.array([vx,vy]);velocity*=min(1.,1.5/max(1e-9,np.linalg.norm(velocity)))
   foot_velocity=velocity+yaw*np.column_stack((-nominal[:,1],nominal[:,0]))
   peak=float(np.linalg.norm(foot_velocity,axis=1).max())
   # Interpolate the upstream .6s/.5 trot and .4s/.375 flying trot using
   # maximum foot speed; joint workspace and velocity are checked below.
   mix=float(np.clip((peak-.5)/1.,0,1));mix=3*mix*mix-2*mix**3
   T=.6-.2*mix;duty=.5-.125*mix;H=.08*min(1.,peak/.1);Ts=T*duty;Tw=T*(1-duty)
   periods[ix,iy,iw]=T;heights[ix,iy,iw]=H;duties[ix,iy,iw]=duty;previous=q0[7:].copy()
   starts=np.array([stance_xy(p,velocity,yaw,-Ts/2) for p in nominal])
   ends=np.array([stance_xy(p,velocity,yaw,Ts/2) for p in nominal])
   for ip in range(N):
    ph=(ip/N+np.array([0,.5,.5,0]))%1;stance=ph<duty;s=np.clip((ph-duty)/(1-duty),0,1)
    target=feet.copy()
    for leg in range(4):
     xy=(stance_xy(nominal[leg],velocity,yaw,(ph[leg]/duty-.5)*Ts)
         if stance[leg] else swing_xy(ends[leg],starts[leg],velocity,yaw,s[leg],Tw))
     target[leg,:2]=q0[:2]+xy
    target[:,2]+=swing_z(s,Tw,H,min(1.,peak/.1))*(~stance)
    def residual(q):
     d.qpos[:]=q0;d.qpos[7:]=q;mujoco.mj_forward(m,d)
     return np.r_[(d.xpos[ids]-target).ravel(),(q-q0[7:])*1e-5]
    def jac(q):
     d.qpos[:]=q0;d.qpos[7:]=q;mujoco.mj_forward(m,d);J=[]
     for bid in ids:mujoco.mj_jacBody(m,d,jp,jr,bid);J.append(jp[:,6:].copy())
     return np.vstack([*J,np.eye(12)*1e-5])
    sol=least_squares(residual,previous,jac=jac,bounds=(-1.5,1.5),max_nfev=60,
                      ftol=1e-9,xtol=1e-9,gtol=1e-9)
    previous=sol.x;table[ix,iy,iw,ip]=previous
    err=float(np.linalg.norm(residual(previous)[:12].reshape(4,3),axis=1).max());worst=max(worst,err)
   q=table[ix,iy,iw];qd=(np.roll(q,-1,0)-np.roll(q,1,0))/(2*T/N)
   max_speed=max(max_speed,float((abs(qd)/np.tile([30.1,30.1,15.05],4)).max()))
   max_q=max(max_q,float(abs(q).max()))
 print('vx',vx,'max_ik_m',worst,'max_speed_ratio',max_speed,'elapsed_s',round(time.monotonic()-started,1),flush=True)
report=dict(family='legged_control_trot',upstream_commit='a7f381c0367e98e31c01336e678eef47e304d40d',
    lift_m=.08,base_period_s=.6,period_range_s=[float(periods.min()),float(periods.max())],duty_range=[float(duties.min()),float(duties.max())],
    max_ik_error_m=worst,max_joint_speed_ratio=max_speed,max_abs_joint_rad=max_q,
    samples=int(np.prod(table.shape[:-1])),dynamic_validation_pending=True)
(O/'kinematic_audit.json').write_text(json.dumps(report,indent=2)+'\n')
assert worst<.003,report
assert max_speed<.9,report
np.savez_compressed(O/'command_gait_table.npz',q=table,q0=q0,grid_x=g,grid_y=g,grid_w=wgrid,
    period=.6,period_table=periods,height_table=heights,duty=.5,duty_table=duties,family='legged_control_trot')
print(json.dumps(report),flush=True)
