"""MuJoCo-only stop/restart regression for the deployed continuous actor."""
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
from assess_native_deployment import NativePlant,ROOT
from deploy_newurdf import NewDeployment,ContinuousActor
from dog_motion.deploy050 import quaternion_matrix

MODEL=ROOT.parents[1]/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'


class Pad:
    def __init__(self):self.axes=[0.]*4
    def snapshot(self):return self.axes.copy(),[0]*6,True


def run(name,command,delay):
    plant=NativePlant(delay_ms=delay,period_ms=5,gain_quantization='floor',
        config_path=ROOT/'config/deploy_newurdf.yaml',model_path=MODEL,
        actor_class=ContinuousActor,deployment_class=NewDeployment)
    d=plant.session;d.walk_unlocked=True;d.pad=Pad()
    d.last_command=np.zeros(3);d.stop_requested_at=None
    d.cfg['commissioning_allowed_clips']=['forward']
    events=[];failure=None;tilts=[];speeds=[];zero_exact=True
    with patch('time.monotonic',lambda:plant.now):
        d.begin('PLAY','forward');d.close_log('offline_neutral_regression')
        for k in range(10000):
            t=k*.001
            moving=1<=t<5 or 5.4<=t<7
            target=np.asarray(command if moving else [0.,0.,0.])
            def raw(v):return 0. if v==0 else float(np.sign(v)*(.12+.88*abs(v)/1.5))
            d.pad.axes=[raw(target[1]),raw(target[0]),0.,raw(target[2])]
            try:
                if k%20==0:
                    d.tick()
                    if not moving:
                        zero_exact=zero_exact and bool(np.array_equal(d.actor.planner.command,np.zeros(3)))
                    if k in (4980,5000,5380,5400,6980,7000):
                        events.append(dict(time_s=t,mode=d.mode,requested=d.last_command.tolist(),
                                           reference=d.actor.planner.command.tolist()))
                    if d.mode!='PLAY':raise RuntimeError('neutral left PLAY')
                plant.physics(k)
                R=quaternion_matrix(plant.data.qpos[3:7])
                tilts.append(float(np.degrees(np.arccos(np.clip(R[2,2],-1,1)))))
                speeds.append(float(np.linalg.norm(plant.data.qvel[:2])))
                if plant.data.qpos[2]<.2:raise RuntimeError('body height below 0.2 m')
                if tilts[-1]>30:raise RuntimeError('tilt over 30 degrees')
            except Exception as exc:
                failure=str(exc);break
    result=dict(case=name,delay_ms=delay,failure=failure,final_mode=d.mode,
        simulated_seconds=(k+1)*.001,neutral_reference_exact_zero=zero_exact,
        max_tilt_deg=max(tilts,default=0),max_xy_speed_mps=max(speeds,default=0),
        events=events,hardware_accessed=False)
    print(json.dumps(result),flush=True)
    return result


if __name__=='__main__':
    cases=[('forward05',[.5,0,0]),('forward15',[1.5,0,0]),
           ('lateral05',[0,.5,0]),('yaw05',[0,0,.5]),
           ('lateral15',[0,1.5,0]),('yaw15',[0,0,1.5])]
    results=[run(name,c,delay) for delay in (0,8) for name,c in cases]
    out=ROOT/'reports/neutral_continuity_20261110.json'
    out.write_text(json.dumps(dict(results=results,hardware_accessed=False),indent=2)+'\n')
    raise SystemExit(0 if all(r['failure'] is None and r['neutral_reference_exact_zero'] for r in results) else 1)
