from pathlib import Path
import json
import numpy as np,mujoco
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parent;REPO=ROOT.parents[1];OUT=REPO/'src/unisupermimic/assets/motions/rcdog';OUT.mkdir(exist_ok=True)
m=mujoco.MjModel.from_xml_path(str(REPO/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'));d=mujoco.MjData(m);q0=m.key_qpos[0].copy();d.qpos[:]=q0;mujoco.mj_forward(m,d)
ids=[m.body(l+'_foot').id for l in ['rr','rl','fr','fl']];feet=d.xpos[ids].copy();off=feet[:,:2]-q0[:2]
gx=np.array([-1.5,-.75,0,.75,1.5]);gy=np.array([-1.5,-.75,0,.75,1.5]);gw=np.array([-1.5,0,1.5]);period=.32;duty=.5;phases=20;table=np.zeros((len(gx),len(gy),len(gw),phases,12));worst=0.;bad=[]
for ix,vx in enumerate(gx):
 for iy,vy in enumerate(gy):
  for iw,w in enumerate(gw):
   planar=np.array([vx,vy])
   vel=planar+w*np.column_stack((-off[:,1],off[:,0]));activity=min(1.,max(np.hypot(vx,vy)/.25,abs(w)/.25));previous=q0[7:].copy()
   for ip in range(phases):
    ph=(ip/phases+np.array([0,.5,.5,0]))%1;stance=ph<duty;s=np.clip((ph-duty)/(1-duty),0,1)[:,None];stride=vel*period*duty
    stanceoff=stride*(.5-ph[:,None]/duty);ramp=.2;H=np.where(s<ramp,(s-ramp/np.pi*np.sin(np.pi*s/ramp))/2,np.where(s>1-ramp,1-ramp-((1-s)-ramp/np.pi*np.sin(np.pi*(1-s)/ramp))/2,s-ramp/2));swingoff=-stride/2-vel*period*(1-duty)*s+(stride+vel*period*(1-duty))*H/(1-ramp)
    target=feet.copy();target[:,:2]+=np.where(stance[:,None],stanceoff,swingoff);target[:,2]+= .025*np.sin(np.pi*s[:,0])**2*(~stance)*activity
    def residual(q):
     d.qpos[:]=q0;d.qpos[7:]=q;mujoco.mj_forward(m,d);return np.r_[(d.xpos[ids]-target).ravel(),(q-q0[7:])*.00001]
    sol=least_squares(residual,previous,bounds=(-1.5,1.5),max_nfev=70,ftol=1e-9,xtol=1e-9,gtol=1e-9);previous=sol.x;err=float(np.linalg.norm(residual(previous)[:12].reshape(4,3),axis=1).max());worst=max(worst,err)
    if err>.003:bad.append([vx,vy,w,ip,err])
    table[ix,iy,iw,ip]=previous
   print('table',vx,vy,w,'worst',worst,flush=True)
(ROOT/'ik_audit.json').write_text(json.dumps(dict(max_error_m=worst,unreachable=bad),indent=2))
assert worst<.003,('IK exceeds 3 mm',worst)
np.savez_compressed(ROOT/'command_gait_table.npz',q=table,q0=q0,grid_x=gx,grid_y=gy,grid_w=gw,period=period)
from command_reference import CommandReference
clips=[];categories={}
def gen(name,fn,seconds=18):
 p=CommandReference(ROOT/'command_gait_table.npz');rows=[]
 for i in range(round(seconds/.02)+1):
  p.advance(fn(i*.02));rows.append(np.r_[p.position,p.quaternion_xyzw(),p.joints()])
 file=OUT/(name+'.csv');np.savetxt(file,rows,delimiter=',',fmt='%.10g');clips.append('src/unisupermimic/assets/motions/rcdog/'+file.name)
for speed in [.25,.5,.75,1.,1.25,1.5]:
 for label,v in [('left',[0,speed,0]),('right',[0,-speed,0]),('ccw',[0,0,speed]),('cw',[0,0,-speed])]:
  name=f'lat{int(speed*100):03}_{label}';gen(name,lambda t,v=v:v if 2<t<11 else np.zeros(3));categories[name]=speed
for speed in [.25,.5,.75,1.,1.25,1.5]:
 for label,v in [('forward',[speed,0,0]),('back',[-speed,0,0])]:gen(f'lat{int(speed*100):03}_{label}',lambda t,v=v:v if 2<t<11 else np.zeros(3))
for speed in [.75,1.5]:
 for label,x,y in [('fl',1,1),('fr',1,-1),('bl',-1,1),('br',-1,-1)]:
  v=[x*speed/np.sqrt(2),y*speed/np.sqrt(2),0];gen(f'diag{int(speed*100):03}_{label}',lambda t,v=v:v if 2<t<11 else [0,0,0])
# Preserve bootstrapping reference filename expected by the existing native actor.
gen('cmd025_forward',lambda t:[.25,0,0] if 2<t<11 else [0,0,0])
for i in range(3):gen(f'lat_stand_{i}',lambda t:[0,0,0],12)
for stage,cap in [('low',.5),('mid',1.),('high',1.5)]:
 for seed in range(8):
  rng=np.random.default_rng(7700+seed);v=rng.uniform(-1,1,(8,3))*[cap,cap,cap];v[::2]=0
  if seed<4:v[:,1]=0;v[:,2]=0
  gen(f'lat_{stage}_transition_{seed:02}',lambda t,v=v:v[min(int(t/6),7)],48)
plan={'clips':clips,'categories':categories,'period':period,'kp':20,'kd':.5,'max_commands':[1.5,1.5,1.5],'planar_norm_limit':1.5,'acceleration_limits':[.5,.5,1.],'ik_error_m':worst}
(ROOT/'plan.json').write_text(json.dumps(plan,indent=2));print('clips',len(clips),flush=True)
