"""Launch the published command150 refinement stages; no robot device access."""
import argparse
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',choices=['mid','high','final'],required=True)
    p.add_argument('--parent',type=Path,required=True)
    p.add_argument('--iterations',type=int)
    p.add_argument('--name')
    p.add_argument('--smoke',action='store_true')
    p.add_argument('--print-command',action='store_true')
    a=p.parse_args()
    parent=a.parent.resolve()
    if not parent.is_file():p.error('parent checkpoint not found: '+str(parent))
    iterations=a.iterations or {'mid':750,'high':2000,'final':750}[a.stage]
    name=a.name or 'command150_'+a.stage+('_smoke' if a.smoke else '')
    run=REPO/'runs'/name
    if run.exists() and not a.print_command:p.error('run exists; choose a new --name: '+str(run))
    cap=100 if a.stage=='mid' else 150
    def keep(path):
        n=Path(path).stem
        if n.startswith('lat') and n[3:6].isdigit():
            return int(n[3:6])<=cap or not ('forward' in n or 'back' in n)
        if 'transition' in n:return 'high' not in n or a.stage!='mid'
        return True
    clips=[f for f in json.loads((HERE/'plan.json').read_text())['clips'] if keep(f)]
    missing=[f for f in clips if not (REPO/f).exists()]
    if missing:p.error('missing motions: '+str(missing))
    args=[sys.executable,'-m','unisupermimic.scripts.train_supermimic',
          'task=rcdog_super_mimic/mjwarp',f'algo.num_envs={32 if a.smoke else 1024}',
          f'algo.iterations={2 if a.smoke else iterations}','algo.hist=5',
          'algo.wm_hidden=[512,512,512]','algo.policy_hidden=[512,256,128]',
          f'algo.wm_batch={64 if a.smoke else 1024}',f'algo.pi_batch={64 if a.smoke else 1024}',
          f'algo.wm_batches={1 if a.smoke else 10}',
          f'algo.wm_only_iters={0 if a.smoke else (100 if a.stage=="final" else 200)}',
          f'algo.ckpt_every={2 if a.smoke else 250}',f'algo.log_every={1 if a.smoke else 25}',
          '+training.init_checkpoint='+str(parent),'training.log_dir='+str(run),
          'env.commands.motion.motion_files=['+','.join(clips)+']']
    if a.smoke:args+=['algo.buffer_steps=128','algo.horizon=8','algo.warmup_steps=64','algo.compile=false']
    print(shlex.join(args),flush=True)
    if a.print_command:return
    env=dict(os.environ,PYTHONPATH=str(REPO/'src')+os.pathsep+os.environ.get('PYTHONPATH',''),
             RCDOG_TRAIN_DR='1',OMP_NUM_THREADS='4',MKL_NUM_THREADS='4',
             MUJOCO_GL='egl',PYTHONUNBUFFERED='1')
    subprocess.run(args,cwd=REPO,env=env,check=True)


if __name__=='__main__':main()
