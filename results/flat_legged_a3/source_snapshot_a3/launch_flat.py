"""Flat SuperMimic training only; no robot hardware imports or terrain runner."""
from pathlib import Path
import argparse,json,os,subprocess,sys
R=Path(__file__).resolve().parent
p=argparse.ArgumentParser();p.add_argument('--smoke',action='store_true');p.add_argument('--iterations',type=int,default=5000)
p.add_argument('--dry-run',action='store_true')
p.add_argument('--focus-clips',default='');p.add_argument('--reward-v3',action='store_true');p.add_argument('--parent',default='parent_policy.pt');p.add_argument('--run-name',default='flat_legged_a0');p.add_argument('--lr',type=float,default=1e-4)
a=p.parse_args();clips=[c['path'] for c in json.loads((R/'flat_clips.json').read_text())]
cmd=[sys.executable,'-m','unisupermimic.scripts.train_supermimic','task=rcdog_super_mimic/mjwarp',
    f'algo.num_envs={16 if a.smoke else 512}',f'algo.iterations={2 if a.smoke else a.iterations}',
    'algo.hist=5','algo.wm_hidden=[512,512,512]','algo.policy_hidden=[512,256,128]',
    f'algo.wm_batch={32 if a.smoke else 512}',f'algo.pi_batch={32 if a.smoke else 512}',
    f'algo.wm_batches={1 if a.smoke else 10}',f'algo.wm_only_iters={0 if a.smoke else 250}',
    f'algo.ckpt_every={2 if a.smoke else 250}',f'algo.log_every={1 if a.smoke else 25}',
    f'+training.init_checkpoint={(R/a.parent).resolve()}',f'training.log_dir=runs/{a.run_name}',
    f'algo.lr={a.lr}','env.sim_dt=.001',
    'env.scene.model_file=src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml',
    'env.max_episode_seconds=30',
    'env.commands.motion.max_episode_steps=1500',
    'env.commands.motion.motion_files=['+','.join(clips)+']']
if a.smoke:cmd+=['algo.buffer_steps=128','algo.horizon=8','algo.warmup_steps=64','algo.compile=false']
env={k:v for k,v in os.environ.items() if not k.startswith('RCDOG_TERRAIN_')}
env.update(PYTHONPATH=str(R/'Uni-SuperMimic/src'),RCDOG_TRAIN_DR='1',RCDOG_SUBSTEP_SPEED_CHECK='1',
    RCDOG_REFERENCE_BEZIER='1',RCDOG_REFERENCE_FAMILY='legged_control_trot',RCDOG_FLAT_GAIT='1',
    RCDOG_POSITION_RATE_LIMIT='1',RCDOG_GAIT_TABLE=str(R/'reference_candidates/legged_control/command_gait_table.npz'),
    OMP_NUM_THREADS='4',MKL_NUM_THREADS='4',MUJOCO_GL='egl',PYTHONUNBUFFERED='1',
    TORCHINDUCTOR_CACHE_DIR=str(R/'torchinductor_cache'),TRITON_CACHE_DIR=str(R/'triton_cache'))
if a.reward_v3:env['RCDOG_FLAT_SLIP_V3']='1'
if a.focus_clips:env.update(RCDOG_FLAT_FOCUS_CLIPS=a.focus_clips,RCDOG_FLAT_FOCUS_FRACTION='.2')
(R/(a.run_name+'_command.json')).write_text(json.dumps(dict(argv=cmd,
    environment={k:v for k,v in env.items() if k.startswith('RCDOG_')},
    actor_terrain_inputs=0,kp=20,kd=.5,physics_dt=.001,policy_hz=50,
    calf_extra_ratio=2,motor_internal_ratio=6.33),indent=2)+'\n')
if a.dry_run:
    print(json.dumps(cmd));raise SystemExit(0)
raise SystemExit(subprocess.call(cmd,cwd=R/'Uni-SuperMimic',env=env))
