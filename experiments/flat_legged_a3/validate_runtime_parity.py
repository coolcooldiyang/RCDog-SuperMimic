from pathlib import Path
import sys,os,json,time
import numpy as np
F=Path(__file__).resolve().parent;REPO=F.parents[1];H=REPO/'real_deploy/controller';B=REPO/'real_deploy/policies/newurdf_legged_a3_1000'
sys.path[:0]=[str(H/'tools'),str(H),str(F)]
os.environ['RCDOG_GAIT_TABLE']=str(B/'command_gait_table.npz');os.environ['RCDOG_POSITION_RATE_LIMIT']='1'
(REPO/'outputs').mkdir(exist_ok=True)
from deploy_newurdf import ContinuousActor
from command_sim import CommandSim,Actor
from dog_motion.deploy050 import quaternion_matrix
actual=ContinuousActor(B);sim=CommandSim(Actor(B/'actor_fp32.onnx'),quantized=True);errors=[];compute=[]
for k in range(500):
 cmd=[.5,0,0] if 20<=k<170 else ([0,.5,.2] if 200<=k<300 else [0,0,0])
 qp,qv=sim.samples[0];R=quaternion_matrix(qp[3:7]);before=time.perf_counter()
 action,desired=actual.tick_command(qp[7:],qv[6:],R,qv[3:6],cmd)
 compute.append((time.perf_counter()-before)*1000)
 sim.step(cmd)
 errors.append(max(float(abs(action-sim.previous).max()),float(abs(desired-sim.target).max())))
 assert errors[-1]<2e-5,(k,errors[-1])
report=dict(frames=len(errors),max_action_or_target_abs_error=max(errors),compute_ms_p95=float(np.percentile(compute,95)),compute_ms_max=max(compute),same_lookahead_reference_and_applied_previous_action=True,hardware_accessed=False)
(REPO/'outputs/flat_runtime_parity.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
