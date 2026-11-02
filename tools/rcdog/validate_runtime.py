import argparse,json
from pathlib import Path
import numpy as np
import mujoco,torch
from hydra import compose,initialize_config_dir
import unisupermimic
from unilab.training import ensure_registries
from unilab.base.config_adapter import create_env
from unisupermimic.scripts.train_supermimic import build_env_override
from unisupermimic.rl.contracts.supermimic import SUPERMIMIC_STATE_KEY,SUPERMIMIC_TRANSITION_KEY
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.kinematics import RCDogKinematics
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.robot import TRACKED_BODIES
from unisupermimic.rl.algos.supermimic.geometry import quat_to_mat
p=argparse.ArgumentParser();p.add_argument('--backend',default='mujoco');p.add_argument('--num-envs',type=int,default=4);a=p.parse_args()
kin=RCDogKinematics('src/unisupermimic/assets/robots/rcdog/supermimic_motor.xml','cpu')
m=mujoco.MjModel.from_xml_path('src/unisupermimic/assets/robots/rcdog/supermimic_motor.xml');d=mujoco.MjData(m)
max_error=0.;max_R=0.
for seed in range(10):
 rng=np.random.default_rng(seed);d.qpos[:]=m.key_qpos[0];d.qpos[7:]+=rng.uniform(-.2,.2,12);mujoco.mj_forward(m,d)
 pp=torch.tensor(d.qpos[:3],dtype=torch.float32).unsqueeze(0);R=quat_to_mat(torch.tensor(d.qpos[3:7],dtype=torch.float32).unsqueeze(0));q=torch.tensor(d.qpos[7:],dtype=torch.float32).unsqueeze(0)
 zero=torch.zeros(1,3);qzero=torch.zeros(1,12)
 kp,kR,_,_=kin.forward(pp,R,zero,zero,q,qzero)
 ids=[m.body(n).id for n in TRACKED_BODIES]
 max_error=max(max_error,float(abs(kp[0].numpy()-d.xpos[ids]).max()));max_R=max(max_R,float(abs(kR[0].numpy()-d.xmat[ids].reshape(-1,3,3)).max()))
assert max_error<1e-6 and max_R<1e-6,(max_error,max_R)
with initialize_config_dir(version_base='1.3',config_dir=str((Path(unisupermimic.__file__).parent/'conf/supermimic').resolve())):
 cfg=compose(config_name='config',overrides=[f'task=rcdog_super_mimic/{a.backend}'])
ensure_registries();env=create_env(cfg,num_envs=a.num_envs,env_cfg_override=build_env_override(cfg))
try:
 _,info=env.reset();current=info[SUPERMIMIC_STATE_KEY]
 assert current['s'].shape==(a.num_envs,37),current['s'].shape
 assert current['xi'].shape==(a.num_envs,16)
 mdp=env.supermimic_mdp
 assert (mdp.motion.bin_start < mdp.motion.clip_end[mdp.motion.bin_clip]).all(), "terminal-only sampling bin"
 env.reset_to_frames(mdp.motion.clip_end[:1].expand(a.num_envs)-1)
 terminations=0
 for step in range(100):
  state=env.step(torch.zeros((a.num_envs,12),device=env.device));t=state.info[SUPERMIMIC_TRANSITION_KEY]
  assert all(torch.isfinite(v).all() for v in t.values());terminations+=int(t['term'].sum())
 frames=torch.zeros(a.num_envs,dtype=torch.long,device=env.device);reference=mdp.ref_state(frames)
 q=reference[4].detach().clone().requires_grad_();inputs=(*reference[:4],q,reference[5]);reward=mdp.reward(inputs,frames,torch.zeros_like(q),torch.zeros_like(q));reward.sum().backward();assert torch.isfinite(q.grad).all()
 print(json.dumps({'backend':a.backend,'num_envs':a.num_envs,'steps':100,'finite':True,'terminations':terminations,'fk_position_error':max_error,'fk_rotation_error':max_R,'reward_gradient_finite':True,'obs_dim':mdp.obs_dim,'goal_dim':mdp.goal_frame_dim,'context_dim':mdp.context_dim}),flush=True)
finally:env.close();env.backend.close()
