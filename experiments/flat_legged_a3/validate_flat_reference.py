"""CPU FK, no-slip reference, phase consistency and differentiable-reward checks."""
from pathlib import Path
import os,sys,json
R=Path(__file__).resolve().parent
os.environ.pop('RCDOG_TERRAIN_MANIFEST',None)
os.environ.update(RCDOG_REFERENCE_BEZIER='1',RCDOG_FLAT_GAIT='1',RCDOG_POSITION_RATE_LIMIT='1')
sys.path.insert(0,str(R.parents[1]/'src'))
import torch,numpy as np,mujoco
from command_reference import CommandReference
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.kinematics import RCDogKinematics
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.motion import MotionLib
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.mdp import MDP

T=R/'reference_candidates/legged_control/command_gait_table.npz'
xml=R.parents[1]/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'
m=mujoco.MjModel.from_xml_path(str(xml));d=mujoco.MjData(m)
ids=[m.body(l+'_foot').id for l in ['rr','rl','fr','fl']];lifts={};max_slip=0.
for target in ([.1,0,0],[.5,0,0],[1.5,0,0],[0,1.5,0],[0,0,1.5],[.8,.8,.8]):
    p=CommandReference(T);p.command[:]=target;p.gait_command[:]=target
    p.period=p.blend_scalar(p.period_table);p.duty=p.blend_scalar(p.duty_table)
    d.qpos[:]=p.q0;mujoco.mj_forward(m,d);ground=d.xpos[ids,2].copy();feet=[];phases=[]
    n=600
    for k in range(n):
        p.phase=k/n;d.qpos[:]=p.q0;d.qpos[7:]=p.joints();mujoco.mj_forward(m,d)
        feet.append(d.xpos[ids].copy());phases.append((p.phase+np.array([0,.5,.5,0]))%1)
    feet=np.array(feet);phases=np.array(phases);height=feet[:,:,2].max(0)-ground
    assert height.min()>.075,(target,height)
    derivative=(np.roll(feet,-1,axis=0)-np.roll(feet,1,axis=0))/(2*p.period/n)
    relative=feet[:,:,:2]-p.q0[:2]
    world_v=derivative[:,:,:2]+np.asarray(target[:2])+target[2]*np.stack((-relative[:,:,1],relative[:,:,0]),-1)
    stance=(phases>.04)&(phases<p.duty-.04)
    peak=float(np.linalg.norm(world_v[stance],axis=-1).max());max_slip=max(max_slip,peak)
    assert peak<.06,(target,peak)
    lifts[str(target)]=dict(min_lift_m=float(height.min()),max_lift_m=float(height.max()),
                           reference_stance_xy_speed_max_mps=peak,period_s=p.period,duty=p.duty)
kin=RCDogKinematics(str(xml),'cpu')
file=R.parents[1]/'src/unisupermimic/assets/motions/rcdog_flat_legged/flat_lc_050_forward.csv'
motion=MotionLib([str(file)],kin,'cpu',src_fps=50);p=CommandReference(T)
for k in range(202):p.advance([.5,0,0] if 2<=k*.02<11 else [0,0,0])
error=float(abs(p.goal([.5,0,0])-motion.goal(torch.tensor([201]),10)[0].numpy()).max())
assert error<3e-4,error
mdp=MDP(kin,motion,torch.tensor(p.q0[7:],dtype=torch.float32),coupled_limits=False)
frames=torch.tensor([201,202]);state=list(motion.ref_state(frames))
state[0]=state[0].clone().requires_grad_(True);action=torch.zeros(2,12,requires_grad=True)
reward=mdp.reward(tuple(state),frames,action,torch.zeros_like(action));reward.sum().backward()
assert torch.isfinite(reward).all() and torch.isfinite(action.grad).all() and torch.isfinite(state[0].grad).all()
# Immediate zero demand does not reset phase or heading; gait settles <=.3s.
p.command[:]=[1.5,0,1.5];p.gait_command=p.command.copy();phase=p.phase
p.advance([0,0,0]);assert np.array_equal(p.command,np.zeros(3)) and p.phase!=phase
for _ in range(15):p.advance([0,0,0])
assert np.max(abs(p.gait_command))<1e-6
report=dict(passed=True,goal_max_abs_error=error,fk_lifts=lifts,max_reference_stance_speed_mps=max_slip,
    finite_reward_gradients=True,immediate_zero_reference_root_velocity=True,terrain_enabled=False,
    hardware_accessed=False,gpu_smoke_completed=False)
(R/'reference_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report),flush=True)
