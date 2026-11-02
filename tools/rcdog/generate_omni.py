"""Eight directions at a given vector speed plus both pure yaw directions."""
import argparse
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from scipy.optimize import least_squares

SCRIPT_DIR=Path(__file__).resolve().parent
REPO=SCRIPT_DIR.parents[1]
ROOT=SCRIPT_DIR
OUT=REPO/'src/unisupermimic/assets/motions/rcdog'
MANIFEST=json.loads((ROOT/'model_audit.json').read_text())
MODEL=mujoco.MjModel.from_xml_path(str(REPO/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'))
DATA=mujoco.MjData(MODEL)
Q0=np.array(MANIFEST['standing_qpos'])
IDS=[MODEL.body(x+'_foot').id for x in ['rr','rl','fr','fl']]
DATA.qpos[:]=Q0
mujoco.mj_forward(MODEL,DATA)
FEET0=DATA.xpos[IDS].copy()
R0=FEET0[:,:2]-Q0[:2]
DT=.02
TIMES=np.arange(451)*DT
def smooth(x):
    x=np.clip(x,0,1)
    return x*x*(3-2*x)
ENVELOPE=smooth((TIMES-1)/1.2)*(1-smooth((TIMES-6.8)/1.2))
TRAVEL=np.r_[0,np.cumsum((ENVELOPE[:-1]+ENVELOPE[1:])*.5*DT)]

def generate(name,stage,twist,period,duty,clearance):
    vx,vy,wz=twist
    previous=Q0[7:].copy()
    rows=[];targets=[];stances=[];errors=[];bad_frames=0;worst_penetration=0.
    yaw0=2*np.arctan2(Q0[6],Q0[3])
    floor=MODEL.geom('floor').id
    for i,t in enumerate(TIMES):
        psi=wz*TRAVEL[i]
        rotation=np.array([[np.cos(psi),-np.sin(psi)],[np.sin(psi),np.cos(psi)]])
        offsets=R0@rotation.T
        center=Q0[:2]+np.array([vx,vy])*TRAVEL[i]
        tangent=np.column_stack((-offsets[:,1],offsets[:,0]))
        velocity=ENVELOPE[i]*(np.array([vx,vy])+wz*tangent)
        phase=((t-1)/period+np.array([0,.5,.5,0]))%1
        stance=phase<duty
        swing=np.clip((phase-duty)/(1-duty),0,1)
        stride=velocity*period*duty
        stance_offset=stride*(.5-phase[:,None]/duty)
        s=swing[:,None]
        swing_time=period*(1-duty)
        swing_offset=-stride/2-velocity*swing_time*s+(stride+velocity*swing_time)*(10*s**3-15*s**4+6*s**5)
        target=FEET0.copy()
        target[:,:2]=center+offsets+np.where(stance[:,None],stance_offset,swing_offset)
        target[:,2]+=clearance*np.sin(np.pi*swing)**2*(~stance)*ENVELOPE[i]
        yaw=yaw0+psi
        DATA.qpos[:3]=[*center,Q0[2]]
        DATA.qpos[3:7]=[np.cos(yaw/2),0,0,np.sin(yaw/2)]
        def residual(q):
            DATA.qpos[7:]=q
            mujoco.mj_forward(MODEL,DATA)
            return np.r_[(DATA.xpos[IDS]-target).ravel(),(q-previous)*.00003]
        solved=least_squares(residual,previous,bounds=(-1.5,1.5),max_nfev=70,ftol=1e-9,xtol=1e-9,gtol=1e-9)
        previous=solved.x
        residual(previous)
        errors.append(float(np.linalg.norm(DATA.xpos[IDS]-target,axis=1).max()))
        bad=False
        for c in DATA.contact:
            if floor not in (int(c.geom1),int(c.geom2)) and c.dist<-.002:
                bad=True;worst_penetration=min(worst_penetration,float(c.dist))
        bad_frames+=bad
        rows.append(np.r_[DATA.qpos[:3],DATA.qpos[3:7][[1,2,3,0]],previous])
        targets.append(target);stances.append(stance|(ENVELOPE[i]<.01))
    rows=np.array(rows)
    speeds=np.abs(np.gradient(rows[:,7:],DT,axis=0)).max(0)
    limits=np.array([30.1,30.1,15.05]*4)
    report={'name':name,'stage':stage,'target_twist_world':list(twist),'period_seconds':period,
            'duty_factor':duty,'clearance_m':clearance,'fps':50,'frames':len(rows),'seconds':9.,
            'steady_interval_seconds':[2.5,6.8],'acceleration_seconds':1.2,
            'reference_displacement':[vx*TRAVEL[-1],vy*TRAVEL[-1],wz*TRAVEL[-1]],
            'max_ik_error_m':max(errors),'max_joint_speed_rad_s':speeds.tolist(),
            'max_nominal_speed_fraction':float((speeds/limits).max()),
            'self_penetration_frames':int(bad_frames),'worst_self_penetration_m':worst_penetration,
            'source':'user URDF; procedural diagonal trot; bounded IK','hardware_ready':False}
    print(json.dumps({k:report[k] for k in ['name','max_ik_error_m','max_nominal_speed_fraction','self_penetration_frames','worst_self_penetration_m']}),flush=True)
    assert max(errors)<.001,(name,'IK',max(errors))
    assert np.all(speeds<limits),(name,'nominal joint speed',speeds)
    assert not bad_frames,(name,'self collision',worst_penetration)
    dest=OUT/(name+'.csv')
    np.savetxt(dest,rows,delimiter=',',fmt='%.10g')
    report['sha256']=hashlib.sha256(dest.read_bytes()).hexdigest()
    dest.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savez(OUT/(name+'_detail.npz'),times=TIMES,stance=stances,foot_targets=targets)
    return str(dest.relative_to(REPO))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--only',choices=['050','100','150']);a=p.parse_args()
    definitions=[('050',.5,.75,.40,.60,.04),('100',1.,1.,.32,.58,.035),('150',1.5,1.5,.28,.56,.03)]
    directions=[('forward',0),('forward_left',45),('left',90),('back_left',135),('back',180),('back_right',225),('right',270),('forward_right',315)]
    plan={'stages':{}}
    for tag,speed,yaw_rate,period,duty,clearance in definitions:
        if a.only and tag!=a.only:continue
        stage='omni'+tag;clips=[]
        for label,angle in directions:
            theta=np.deg2rad(angle);vx=float(speed*np.cos(theta));vy=float(speed*np.sin(theta))
            if abs(vx)<1e-8:vx=0.
            if abs(vy)<1e-8:vy=0.
            clips.append(generate(stage+'_'+label,stage,(vx,vy,0.),period,duty,clearance))
        for label,sign in [('ccw',1),('cw',-1)]:
            clips.append(generate(stage+'_'+label,stage,(0.,0.,sign*yaw_rate),.36,.60,.035))
        plan['stages'][stage]={'clips':clips,'iterations':8000,'perturbation_trials':8 if tag=='150' else 3,
                               'speed_m_s':speed,'yaw_rate_rad_s':yaw_rate,'acceptance_fraction':.10}
    (ROOT/('plan_'+a.only+'.json' if a.only else 'plan.json')).write_text(json.dumps(plan,indent=2)+'\n')
