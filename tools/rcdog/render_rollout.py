"""Render already-recorded physical trajectories; no policy playback substitution."""
import argparse
from pathlib import Path
import numpy as np
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.render import render_comparison
p=argparse.ArgumentParser()
p.add_argument('--rollout',type=Path,required=True);p.add_argument('--reference',type=Path,required=True)
p.add_argument('--output',type=Path,required=True);p.add_argument('--label',required=True)
a=p.parse_args()
actual=np.load(a.rollout)['qpos'];raw=np.loadtxt(a.reference,delimiter=',')
ref=np.column_stack((raw[:,:3],raw[:,3:7][:,[3,0,1,2]],raw[:,7:]))[:len(actual)]
n=render_comparison(ref,actual,a.output,clip_name=a.label,global_ghost=True)
print('Rendered',n,'frames')
