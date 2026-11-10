import sys,json,math
from pathlib import Path
import torch
r=Path(__file__).resolve().parent;sys.path.insert(0,str(r.parents[1]/'src'))
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.kinematics import RCDogKinematics
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.motion import MotionLib
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.mdp import MDP
kin=RCDogKinematics(str(r.parents[1]/'src/unisupermimic/assets/robots/rcdog/supermimic_motor.xml'),'cpu');motion=MotionLib([str(r.parents[1]/'src/unisupermimic/assets/motions/rcdog/lat_stand_0.csv')],kin,'cpu',src_fps=50.);mdp=MDP(kin,motion,motion.q[0]);f=torch.tensor([100]);s=list(mdp.ref_state(f));a=torch.zeros(1,12);rows=[]
for label,angle,dq,da in [('still',0,0,0),('lean10',10,0,0),('lean20',20,0,0),('lean26',26,0,0),('joint_jitter',0,1,0),('action_jitter',0,0,1)]:
 st=[x.clone() for x in s];t=math.radians(angle);R=torch.tensor([[1.,0,0],[0,math.cos(t),-math.sin(t)],[0,math.sin(t),math.cos(t)]])[None];st[1]=R@st[1];st[5]+=dq
 reward,terms=mdp.reward(tuple(st),f,a+da,a,terms=True);rows.append(dict(case=label,reward=float(reward),stability=float(terms['stability']),stop_weight=float(terms['stop_weight']),terminated=bool(mdp.termination(tuple(st),f))))
assert rows[0]['stability']>rows[1]['stability']>rows[2]['stability']
assert not rows[2]['terminated'] and rows[3]['terminated']
assert rows[4]['stability']<rows[0]['stability'] and rows[5]['stability']<rows[0]['stability']
a=torch.ones(1,12,requires_grad=True);rew=mdp.reward(tuple(s),f,a,torch.zeros_like(a));rew.sum().backward();assert torch.isfinite(a.grad).all() and a.grad.abs().sum()>0
st=[x.clone() for x in s];st[5][0,0]=8.1;assert bool(mdp.termination(tuple(st),f));assert float(mdp.soft_termination(tuple(st),f))>.5
(r/'reward_validation_high.json').write_text(json.dumps({'passed':True,'cases':rows,'differentiable':True},indent=2));print(json.dumps(rows))
