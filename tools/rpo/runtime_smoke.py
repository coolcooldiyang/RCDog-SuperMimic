"""Exercise the actual UniLab runtime with the RPO task before training."""
import argparse
import json
from pathlib import Path
import torch
from hydra import compose, initialize_config_dir
import unisupermimic
from unilab.training import ensure_registries
from unilab.base.config_adapter import create_env
from unisupermimic.scripts.train_supermimic import build_env_override
from unisupermimic.rl.contracts.supermimic import SUPERMIMIC_STATE_KEY,SUPERMIMIC_TRANSITION_KEY

p=argparse.ArgumentParser();p.add_argument('--backend',default='mjwarp');p.add_argument('--num-envs',type=int,default=32);p.add_argument('--coupled',action='store_true')
a=p.parse_args()
config=Path(unisupermimic.__file__).parent/'conf/supermimic'
with initialize_config_dir(version_base='1.3',config_dir=str(config.resolve())):
    overrides=[f'task=rpo_super_mimic/{a.backend}']
    if a.coupled:
        overrides.extend(['env.scene.model_file=src/unisupermimic/assets/robots/rpo/scene_supermimic_coupled.xml',
                          'env.commands.motion.mjcf_path=src/unisupermimic/assets/robots/rpo/supermimic_coupled.xml'])
    cfg=compose(config_name='config',overrides=overrides)
ensure_registries()
env=create_env(cfg,num_envs=a.num_envs,env_cfg_override=build_env_override(cfg))
try:
    _,info=env.reset()
    current=info[SUPERMIMIC_STATE_KEY]
    assert current['s'].shape==(a.num_envs,59)
    assert current['xi'].shape==(a.num_envs,27)
    terminations=0
    for i in range(100):
        state=env.step(torch.zeros((a.num_envs,23),device=env.device))
        transition=state.info[SUPERMIMIC_TRANSITION_KEY]
        assert all(torch.isfinite(v).all() for v in transition.values())
        terminations+=int(transition['term'].sum())
    print(json.dumps({'backend':a.backend,'num_envs':a.num_envs,'steps':100,
                      'all_transitions_finite':True,'zero_policy_terminations':terminations}))
finally:
    env.close();env.backend.close()
