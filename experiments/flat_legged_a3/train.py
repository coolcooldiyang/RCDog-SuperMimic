"""Reproduce the a0 -> a2 -> a3 flat training chain; no hardware access."""
import argparse,json,os,shlex,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;REPO=HERE.parents[1]
FOCUS='flat_lc_100_ccw,flat_lc_150_bl,flat_lc_150_br,flat_lc_150_fl,flat_lc_150_left,flat_lc_150_right'
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--stage',choices=['a0','a2','a3'],required=True)
 p.add_argument('--parent',type=Path);p.add_argument('--name');p.add_argument('--iterations',type=int);p.add_argument('--lr',type=float)
 p.add_argument('--smoke',action='store_true');p.add_argument('--print-command',action='store_true');p.add_argument('--feedback-delay-ms',type=float,default=0.)
 a=p.parse_args();iterations,lr,parent={'a0':(5000,1e-4,'results/command150/ckpt_000500.pt'),'a2':(1500,2e-5,'results/flat_legged_a3/a0/ckpt_005000.pt'),'a3':(1000,1e-5,'results/flat_legged_a3/a2/ckpt_001500.pt')}[a.stage]
 parent=(a.parent or REPO/parent).resolve();name=a.name or 'flat_legged_'+a.stage+('_smoke' if a.smoke else '');run=REPO/'runs'/name
 if not parent.is_file():p.error('Missing parent: '+str(parent))
 if run.exists() and not a.print_command:p.error('Run exists; choose another --name: '+str(run))
 if not 0<=a.feedback_delay_ms<=15:p.error('feedback-delay-ms must be in[0,15]')
 clips=[c['path'] for c in json.loads((HERE/'flat_clips.json').read_text())]
 if any(not (REPO/f).is_file() for f in clips):p.error('Missing generated CSV; run prepare_flat_motions.py')
 cmd=[sys.executable,'-m','unisupermimic.scripts.train_supermimic','task=rcdog_super_mimic/mjwarp',
  f'algo.num_envs={16 if a.smoke else 512}',f'algo.iterations={2 if a.smoke else a.iterations or iterations}',
  'algo.hist=5','algo.wm_hidden=[512,512,512]','algo.policy_hidden=[512,256,128]',
  f'algo.wm_batch={32 if a.smoke else 512}',f'algo.pi_batch={32 if a.smoke else 512}',
  f'algo.wm_batches={1 if a.smoke else 10}',f'algo.wm_only_iters={0 if a.smoke else 250}',
  f'algo.ckpt_every={2 if a.smoke else 250}',f'algo.log_every={1 if a.smoke else 25}',
  f'+training.init_checkpoint={parent}',f'training.log_dir={run}',f'algo.lr={a.lr or lr}',
  'env.sim_dt=.001','env.scene.model_file=src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml',
  'env.max_episode_seconds=30','env.commands.motion.max_episode_steps=1500',
  'env.commands.motion.motion_files=['+','.join(clips)+']']
 if a.smoke:cmd+=['algo.buffer_steps=128','algo.horizon=8','algo.warmup_steps=64','algo.compile=false']
 env={k:v for k,v in os.environ.items() if not k.startswith('RCDOG_')}
 env.update(PYTHONPATH=str(REPO/'src'),RCDOG_TRAIN_DR='1',RCDOG_SUBSTEP_SPEED_CHECK='1',
  RCDOG_REFERENCE_BEZIER='1',RCDOG_REFERENCE_FAMILY='legged_control_trot',RCDOG_FLAT_GAIT='1',
  RCDOG_POSITION_RATE_LIMIT='1',RCDOG_GAIT_TABLE=str(HERE/'reference_candidates/legged_control/command_gait_table.npz'),
  RCDOG_FLAT_SLIP_V3='0' if a.stage=='a0' else '1',RCDOG_FEEDBACK_DELAY_MS=str(a.feedback_delay_ms),
  OMP_NUM_THREADS='4',MKL_NUM_THREADS='4',MUJOCO_GL='egl',PYTHONUNBUFFERED='1')
 if a.stage=='a3':env.update(RCDOG_FLAT_FOCUS_CLIPS=FOCUS,RCDOG_FLAT_FOCUS_FRACTION='.2')
 print(shlex.join(cmd),flush=True)
 print(json.dumps({k:v for k,v in env.items() if k.startswith('RCDOG_')},indent=2),flush=True)
 if a.print_command:return
 record=REPO/'outputs/flat_training_commands';record.mkdir(parents=True,exist_ok=True)
 (record/(name+'.json')).write_text(json.dumps(dict(argv=cmd,environment={k:v for k,v in env.items() if k.startswith('RCDOG_')}),indent=2)+'\n')
 subprocess.run(cmd,cwd=REPO,env=env,check=True)
if __name__=='__main__':main()
