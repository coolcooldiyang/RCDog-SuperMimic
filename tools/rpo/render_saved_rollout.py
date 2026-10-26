"""Render actual recorded coupled-physics states with the reference ghost."""
import argparse,json
from pathlib import Path
import numpy as np
from unisupermimic.tasks.motion_tracking.rpo.supermimic.render import render_comparison

p=argparse.ArgumentParser();p.add_argument('--rollout',type=Path,required=True);p.add_argument('--motion',required=True);p.add_argument('--output',type=Path,required=True)
a=p.parse_args()
data=np.load(a.rollout);actual=data['qpos']
raw=np.loadtxt(f'src/unisupermimic/assets/motions/rpo/{a.motion}_reference.csv',delimiter=',')
reference=np.column_stack((raw[:,:3],raw[:,3:7][:,[3,0,1,2]],raw[:,7:]))[:len(actual)]
count=render_comparison(reference,actual,a.output,clip_name=f'{a.motion} / coupled MuJoCo / actual rollout',
                        width=960,height=540,global_ghost=True)
print(json.dumps({'output':str(a.output),'frames':count,'mode':'recorded coupled physics, not kinematic replay'}))
