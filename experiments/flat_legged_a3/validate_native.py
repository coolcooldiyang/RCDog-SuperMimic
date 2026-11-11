import sys,json,tempfile,time,math,argparse
from pathlib import Path
from unittest.mock import patch
import numpy as np,yaml,mujoco
F=Path(__file__).resolve().parent
REPO=F.parents[1];H=REPO/'real_deploy/controller'
sys.path[:0]=[str(H/'tools'),str(H),str(F)]
from deploy_newurdf import ContinuousActor,NewDeployment
from assess_native_deployment import NativePlant
from dog_motion.deploy050 import quaternion_matrix
class Pad:
 def __init__(self):self.axes=[0.]*4
 def snapshot(self):return self.axes.copy(),[0]*6,True
p=argparse.ArgumentParser();p.add_argument('--bundle',required=True);p.add_argument('--output',required=True);p.add_argument('--cases',default='forward,left,yaw');p.add_argument('--speed',type=float,default=.5);p.add_argument('--contract-limits',action='store_true');a=p.parse_args()
config=yaml.safe_load((H/'examples/deploy_legged_a3.yaml').read_text());config['commissioning_max_forward_mps']=a.speed;config['commissioning_max_yaw_rad_s']=a.speed;config['bundle']=str(Path(a.bundle).resolve());config['commissioning_allowed_clips']=['forward'];config['checkpoint_sha256']=json.loads((Path(a.bundle)/'policy_contract.json').read_text())['checkpoint_sha256']
if a.contract_limits:config['commissioning_speed_limits_rad_s']=[30.1,30.1,15.05]
Path(a.output).parent.mkdir(parents=True,exist_ok=True)
rows=[];start=time.perf_counter()
with tempfile.NamedTemporaryFile(mode='w',suffix='.yaml') as file:
 yaml.safe_dump(config,file);file.flush()
 for delay in (0,8):
  for name in a.cases.split(','):
   vec={'stand':[0,0,0],'forward':[a.speed,0,0],'backward':[-a.speed,0,0],'left':[0,a.speed,0],'right':[0,-a.speed,0],'yaw':[0,0,a.speed],'cw':[0,0,-a.speed],'fl':[a.speed/2**.5,a.speed/2**.5,0],'fr':[a.speed/2**.5,-a.speed/2**.5,0],'bl':[-a.speed/2**.5,a.speed/2**.5,0],'br':[-a.speed/2**.5,-a.speed/2**.5,0]}[name]
   plant=NativePlant(delay_ms=delay,period_ms=5,gain_quantization='floor',config_path=Path(file.name),model_path=REPO/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml',actor_class=ContinuousActor,deployment_class=NewDeployment)
   d=plant.session;d.cfg['commissioning_allowed_clips']=['forward'];d.walk_unlocked=True;d.pad=Pad();d.last_command=np.zeros(3);d.stop_requested_at=None
   k=-1;peak=np.zeros(12);tilt=0.;failure=None;zero=True;ptimes=[];finalv=[]
   with patch('time.monotonic',lambda:plant.now):
    try:
     d.begin('PLAY','forward');d.close_log('offline_new_policy_test')
     for k in range(10000):
      t=k*.001;cmd=vec if 1<=t<5 or 5.4<=t<7 else [0.,0.,0.]
      def raw(v,cap):return 0. if v==0 else float(np.sign(v)*(.12+.88*abs(v)/cap))
      # Encode against the actual configured joystick scale, avoiding an understated test command.
      speed_cap=float(d.cfg['commissioning_max_forward_mps']);yaw_cap=float(d.cfg['commissioning_max_yaw_rad_s'])
      d.pad.axes=[raw(cmd[1],speed_cap),raw(cmd[0],speed_cap),0.,raw(cmd[2],yaw_cap)]
      if k%20==0:
       before=time.perf_counter();d.tick();ptimes.append((time.perf_counter()-before)*1000)
       assert np.allclose(d.last_command,cmd,atol=1e-8),(d.last_command,cmd)
       if not any(cmd):zero &= bool(np.array_equal(d.actor.planner.command,np.zeros(3)))
      plant.physics(k);peak=np.maximum(peak,abs(plant.data.qvel[6:]));R=quaternion_matrix(plant.data.qpos[3:7]);tilt=max(tilt,float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))))
      if plant.data.qpos[2]<.2 or tilt>30:raise RuntimeError('body_height_or_tilt')
      if k>9000:finalv.append(np.linalg.norm(plant.data.qvel[:2]))
    except Exception as e:failure=str(e)
   record=dict(speed_limits_rad_s=config['commissioning_speed_limits_rad_s'],case=name,command=vec,delay_ms=delay,completed_s=k*.001,passed=failure is None and zero,failure=failure,neutral_reference_exact_zero=zero,peak_joint_velocity_rad_s=peak.tolist(),max_tilt_deg=tilt,stop_xy_speed_mps=float(np.mean(finalv)) if finalv else None,compute_ms_p95=float(np.percentile(ptimes,95)) if ptimes else None,compute_ms_max=max(ptimes,default=0),hardware_accessed=False)
   rows.append(record);print(json.dumps(record),flush=True)
   Path(a.output).write_text(json.dumps(dict(bundle=a.bundle,results=rows,passed=sum(x['passed'] for x in rows),total=len(rows),hardware_accessed=False,elapsed_s=time.perf_counter()-start),indent=2)+'\n')
raise SystemExit(0 if all(x['passed'] for x in rows) else 1)
