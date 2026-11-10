"""CPU MuJoCo held-out speed, tilt, stop-jitter and restart tests."""
import argparse,json
from pathlib import Path
import numpy as np
from command_sim import Actor,CommandSim
p=argparse.ArgumentParser();p.add_argument('--checkpoint',required=True);p.add_argument('--output',required=True);p.add_argument('--cap',type=float,default=1.5);p.add_argument('--stress',action='store_true');p.add_argument('--quick',action='store_true');p.add_argument('--reference-table');p.add_argument('--cases');a=p.parse_args();actor=Actor(a.checkpoint);results=[];out=Path(a.output);out.parent.mkdir(parents=True,exist_ok=True)
speeds=[a.cap] if a.quick else sorted(set([.25,.5,min(1.,a.cap),a.cap]))
cases=[(f'{name}_{s:g}',np.array(v)*s) for s in speeds for name,v in [('forward',[1,0,0]),('backward',[-1,0,0]),('left',[0,1,0]),('right',[0,-1,0]),('ccw',[0,0,1]),('cw',[0,0,-1])]]
cases += [('stand',np.zeros(3))]
if not a.quick:
 cases += [('mixed',np.array([.3,.6*a.cap,.5*a.cap]))]
 cases += [(f'diag_{name}',np.array([x,y,0])*a.cap/np.sqrt(2)) for name,x,y in [('fl',1,1),('fr',1,-1),('bl',-1,1),('br',-1,-1)]]
if a.cases:cases=[(n,v) for n,v in cases if n in a.cases.split(',')]
assert cases,'No test cases selected'
for delay,noise in ([(0,False),(8,True)] if a.stress else [(0,False)]):
 for name,target in cases:
  sim=CommandSim(actor,delay_ms=delay,quantized=True,noise=noise,seed=2401);rows=[]
  if a.reference_table:
   from command_reference import CommandReference
   sim.planner=CommandReference(a.reference_table)
  for i in range(900):
   t=i*.02;cmd=target if 2<=t<11 else np.zeros(3);rows.append(sim.step(cmd))
   if sim.fault:break
  times=np.array([r['time'] for r in rows]);steady=(times>7)&(times<10.5);stopped=times>16
  vel=np.array([r['measured'] for r in rows]);qp=np.array([r['qpos'] for r in rows]);qv=np.array([r['qvel'] for r in rows]);act=np.array([r['action'] for r in rows]);mean=vel[steady].mean(0) if steady.any() else np.full(3,99)
  qp_ptp=float(np.ptp(qp[stopped,7:],axis=0).max()*180/np.pi) if stopped.sum()>5 else 999
  stop_speed=float(np.linalg.norm(vel[stopped,:2],axis=1).mean()) if stopped.any() else 999
  stop_yaw=float(abs(vel[stopped,2]).mean()) if stopped.any() else 999
  tilt=max(r['tilt_deg'] for r in rows);error=mean-target
  target_ptp=float(np.ptp(np.clip(sim.q0+.25*act[stopped],-1.55,1.55),axis=0).max()*180/np.pi) if stopped.any() else 999
  passed=bool(sim.fault is None and np.linalg.norm(error[:2])<max(.1,.1*np.linalg.norm(target[:2])) and abs(error[2])<max(.12,.1*abs(target[2])) and tilt<25 and qp_ptp<2 and stop_speed<.03 and stop_yaw<.05 and target_ptp<(6 if noise else 3))
  result=dict(case=name,target=target.tolist(),measured=mean.tolist(),fault=sim.fault,delay_ms=delay,noise=noise,tilt_max_deg=tilt,stop_joint_ptp_deg=qp_ptp,stop_target_ptp_deg=target_ptp,stop_joint_speed_rms=float(np.sqrt(np.mean(qv[stopped,6:]**2))) if stopped.any() else None,stop_action_ptp=float(np.ptp(act[stopped],axis=0).max()) if stopped.any() else None,stop_speed_mps=stop_speed,stop_yaw_rad_s=stop_yaw,passed=passed);results.append(result);print(json.dumps(result),flush=True)
  folder=out.with_suffix('');folder.mkdir(exist_ok=True);np.savez_compressed(folder/f'{name}_d{delay}.npz',**{key:np.array([r[key] for r in rows]) for key in rows[0]})
report=dict(checkpoint=a.checkpoint,cap=a.cap,all_passed=all(r['passed'] for r in results),passed=sum(r['passed'] for r in results),total=len(results),real_hardware_accessed=False,results=results)
out.write_text(json.dumps(report,indent=2)+'\n')
