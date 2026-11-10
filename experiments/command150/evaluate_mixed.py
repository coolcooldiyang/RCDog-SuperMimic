"""Continuous lateral reversal/yaw/start-stop test, no policy resets between segments."""
import argparse,json
from pathlib import Path
import numpy as np
from command_sim import Actor,CommandSim
p=argparse.ArgumentParser();p.add_argument('--checkpoint',required=True);p.add_argument('--output',required=True);p.add_argument('--delay',type=int,default=8);p.add_argument('--noise',action='store_true');p.add_argument('--cap',type=float,default=1.5);a=p.parse_args();s=a.cap
seq=[(3,[0,0,0]),(7,[s,0,0]),(6,[0,0,0]),(7,[-s,0,0]),(6,[0,0,0]),(7,[0,s,0]),(6,[0,0,0]),(7,[0,-s,0]),(6,[0,0,0]),(6,[0,0,s]),(6,[0,0,0]),(6,[0,0,-s]),(6,[0,0,0]),(6,[.25,s*.5,s*.5]),(6,[-.25,-s*.5,-s*.5]),(.4,[0,0,0]),(.4,[0,.5,0]),(.4,[0,0,0]),(.4,[0,-.5,0]),(6,[0,0,0])]
sim=CommandSim(Actor(a.checkpoint),delay_ms=a.delay,noise=a.noise,quantized=True,seed=512);rows=[];segments=[];start=0
for duration,cmd in seq:
 begin=len(rows)
 for _ in range(round(duration/.02)):
  rows.append(sim.step(cmd))
  if sim.fault:break
 steady=rows[begin+round(max(2.,duration-2)/.02):];mean=np.mean([x['measured'] for x in steady],axis=0) if steady else None
 segments.append(dict(start=start,duration=duration,command=cmd,measured=None if mean is None else mean.tolist()));start+=duration
 if sim.fault:break
stopped=rows[-100:] if sim.fault is None else [];q=np.array([x['qpos'][7:] for x in stopped]);targets=np.array([np.clip(sim.q0+.25*x['action'],-1.55,1.55) for x in stopped]);tilt=max(x['tilt_deg'] for x in rows)
valid_tracking=[r for r in segments if r['duration']>=6 and r['measured'] is not None]
tracking=all(np.linalg.norm(np.array(r['measured'][:2])-r['command'][:2])<.15 and abs(r['measured'][2]-r['command'][2])<.15 for r in valid_tracking)
report=dict(checkpoint=a.checkpoint,delay_ms=a.delay,noise=a.noise,fault=sim.fault,completed_seconds=rows[-1]['time'],max_tilt_deg=tilt,stop_joint_ptp_deg=float(np.ptp(q,axis=0).max()*180/np.pi) if len(q) else 999,stop_target_ptp_deg=float(np.ptp(targets,axis=0).max()*180/np.pi) if len(q) else 999,segments=segments,tracking_passed=tracking,real_hardware_accessed=False)
report['passed']=bool(sim.fault is None and tracking and tilt<25 and report['stop_joint_ptp_deg']<2 and report['stop_target_ptp_deg']<(6 if a.noise else 3))
out=Path(a.output);out.parent.mkdir(exist_ok=True,parents=True);out.write_text(json.dumps(report,indent=2)+'\n');np.savez_compressed(out.with_suffix('.npz'),**{k:np.array([r[k] for r in rows]) for k in rows[0]});print(json.dumps(report,indent=2))
