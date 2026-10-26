"""Robot-to-robot IK of public G1 references, with named RPO joints and limits.

This produces kinematic references, not a claim of dynamic or real-robot success.
"""
from pathlib import Path
import argparse,json,hashlib,xml.etree.ElementTree as ET
import numpy as np
import mujoco
import osqp
from scipy import sparse
from scipy.optimize import least_squares
from scipy.interpolate import PchipInterpolator
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation,Slerp

ROOT=Path(__file__).resolve().parents[2]
REPO=ROOT
ASSET=REPO/'src/unisupermimic/assets/robots'
OUT=REPO/'src/unisupermimic/assets/motions/rpo'
OUT.mkdir(parents=True,exist_ok=True)
source=ROOT/'data/source';source.mkdir(parents=True,exist_ok=True)

def bare_g1():
    root=ET.parse(ASSET/'g1/supermimic.xml').getroot()
    root.find('compiler').attrib.pop('meshdir',None)
    for body in root.iter('body'):
        for geom in list(body.findall('geom')):
            if geom.get('type')=='mesh':body.remove(geom)
    asset=root.find('asset')
    for item in list(asset):
        if item.tag=='mesh':asset.remove(item)
    p=ROOT/'data/source/g1_fk.xml';ET.ElementTree(root).write(p,encoding='unicode')
    return mujoco.MjModel.from_xml_path(str(p))

def support_z(model,data,body_ids):
    values=[]
    for bid in body_ids:
        for gi in range(model.body_geomadr[bid],model.body_geomadr[bid]+model.body_geomnum[bid]):
            if model.geom_contype[gi]==0:continue
            rot=data.geom_xmat[gi].reshape(3,3);size=model.geom_size[gi];z=data.geom_xpos[gi,2]
            typ=model.geom_type[gi]
            if typ==mujoco.mjtGeom.mjGEOM_BOX:values.append(z-np.dot(abs(rot[2]),size))
            elif typ==mujoco.mjtGeom.mjGEOM_CAPSULE:values.append(z-size[0]-size[1]*abs(rot[2,2]))
            elif typ==mujoco.mjtGeom.mjGEOM_CYLINDER:values.append(z-size[0]*np.sqrt(max(0,1-rot[2,2]**2))-size[1]*abs(rot[2,2]))
            elif typ==mujoco.mjtGeom.mjGEOM_SPHERE:values.append(z-size[0])
            elif typ==mujoco.mjtGeom.mjGEOM_MESH:
                mi=model.geom_dataid[gi];a=model.mesh_vertadr[mi];n=model.mesh_vertnum[mi]
                values.append(float((model.mesh_vert[a:a+n]@rot.T+data.geom_xpos[gi])[:,2].min()))
    if not values:raise ValueError('No foot collision geometry')
    return min(values)

def smooth_with_limits(q, lo, hi, vmax, dt):
    """Nearest smooth sequence with hard per-step joint speed/position bounds."""
    n=len(q)
    d1=sparse.diags([-np.ones(n-1),np.ones(n-1)],[0,1],shape=(n-1,n),format='csc')
    d2=sparse.diags([np.ones(n-2),-2*np.ones(n-2),np.ones(n-2)],[0,1,2],shape=(n-2,n),format='csc')
    P=sparse.csc_matrix(sparse.eye(n)+20*d2.T@d2)
    A=sparse.vstack([sparse.eye(n),d1],format='csc')
    result=np.empty_like(q)
    for j in range(q.shape[1]):
        opt=osqp.OSQP()
        opt.setup(P=P,q=-q[:,j],A=A,l=np.r_[np.full(n,lo[j]),np.full(n-1,-vmax[j]*dt)],
                  u=np.r_[np.full(n,hi[j]),np.full(n-1,vmax[j]*dt)],
                  verbose=False,eps_abs=1e-7,eps_rel=1e-7,max_iter=20000,polishing=True)
        solved=opt.solve()
        if solved.info.status_val not in (1,2):raise RuntimeError(solved.info.status)
        result[:,j]=solved.x
    return result

def smooth_parallel_ankles(q, lo, hi, dt):
    """Constrain both motor speeds with the Torch port of the RDK geometry."""
    import torch
    from unisupermimic.tasks.motion_tracking.rpo.supermimic.parallel import ankle_jacobian
    def jacobian(pair,left):
        value=torch.as_tensor(pair,dtype=torch.float64)
        return ankle_jacobian(value,left)[0].detach().numpy()
    n=len(q)
    d1=sparse.diags([-np.ones(n-1),np.ones(n-1)],[0,1],shape=(n-1,n),format='csc')
    d2=sparse.diags([np.ones(n-2),-2*np.ones(n-2),np.ones(n-2)],[0,1,2],shape=(n-2,n),format='csc')
    D=sparse.kron(d1,sparse.eye(2),format='csc');D2=sparse.kron(d2,sparse.eye(2),format='csc')
    P=sparse.csc_matrix(sparse.eye(2*n)+20*D2.T@D2)
    for ids,left in [([4,5],True),([10,11],False)]:
        target=q[:,ids].copy();x=target.copy()
        for _ in range(5):
            J=sparse.block_diag([jacobian((x[i]+x[i+1])/2,left) for i in range(n-1)],format='csc')
            A=sparse.vstack([sparse.eye(2*n),J@D],format='csc')
            opt=osqp.OSQP()
            opt.setup(P=P,q=-target.ravel(),A=A,
                      l=np.r_[np.tile(lo[ids],n),np.full(2*(n-1),-7.0*dt)],
                      u=np.r_[np.tile(hi[ids],n),np.full(2*(n-1),7.0*dt)],
                      verbose=False,eps_abs=1e-7,eps_rel=1e-7,max_iter=20000)
            solved=opt.solve()
            if solved.info.status_val not in (1,2):raise RuntimeError(solved.info.status)
            x=solved.x.reshape(n,2)
        q[:,ids]=x
    return q

def retarget(raw,name,kind):
    g=bare_g1();gd=mujoco.MjData(g)
    m=mujoco.MjModel.from_xml_path(str(ASSET/'rpo/supermimic.xml'));d=mujoco.MjData(m)
    q0=np.asarray(m.key_qpos[0,7:]);lo=m.jnt_range[1:,0]+1e-4;hi=m.jnt_range[1:,1]-1e-4
    gjoints={g.joint(i).name:i for i in range(1,g.njnt)}
    rnames=[m.joint(i).name for i in range(1,m.njnt)]
    gb=lambda s:g.body(s).id
    rb=lambda s:m.body(s).id
    sf=[gb('left_ankle_roll_link'),gb('right_ankle_roll_link')]
    rf=[rb('left_ankle_roll_link'),rb('right_ankle_roll_link')]
    gp=[gb(n) for n in ['left_ankle_roll_link','right_ankle_roll_link','left_knee_link','right_knee_link','left_elbow_link','right_elbow_link','left_wrist_yaw_link','right_wrist_yaw_link']]
    rp=[rb(n) for n in ['left_ankle_roll_link','right_ankle_roll_link','left_knee_link','right_knee_link','left_elbow_pitch_link','right_elbow_pitch_link']]
    hands=[m.site(n).id for n in ['left_hand_tip','right_hand_tip']]
    weights=np.array([8,8,3,3,1,1,2,2])[:,None]
    collision_pairs=[('left_elbow_pitch','left_thigh_pitch'),('left_elbow_pitch','left_knee'),
                     ('right_elbow_pitch','right_thigh_pitch'),('right_elbow_pitch','right_knee'),
                     ('base','left_elbow_pitch'),('base','right_elbow_pitch'),
                     ('left_thigh_pitch','right_thigh_pitch'),('left_knee','right_knee')]
    collision_pairs=[tuple(m.geom(n+'_collision').id for n in pair) for pair in collision_pairs]
    def rpoints(q):
        d.qpos[:]=np.r_[0,0,0,1,0,0,0,q];mujoco.mj_forward(m,d)
        return np.vstack([d.xpos[rp],d.site_xpos[hands]])
    def gpoints(q):
        gd.qpos[:]=np.r_[0,0,0,1,0,0,0,q];mujoco.mj_forward(g,gd)
        return gd.xpos[gp].copy()
    g0=np.zeros(29)
    for i in range(29):
        n=g.joint(i+1).name
        if 'hip_pitch' in n:g0[i]=-.312
        elif 'knee' in n:g0[i]=.669
        elif 'ankle_pitch' in n:g0[i]=-.363
        elif 'elbow_joint' in n:g0[i]=.6
        elif 'shoulder_pitch' in n:g0[i]=.2
        elif n=='left_shoulder_roll_joint':g0[i]=.2
        elif n=='right_shoulder_roll_joint':g0[i]=-.2
    gn=gpoints(g0);rn=rpoints(q0)
    scales=np.ones((8,1));scales[:4]=abs(rn[:2,2].mean()/gn[:2,2].mean());scales[4:]=.95
    q_previous=q0.copy();qs=[];errs=[];clearance=[]
    for fi,row in enumerate(raw):
        target=rn+(gpoints(row[7:])-gn)*scales
        gfoot_R=gd.xmat[sf].copy().reshape(2,3,3)
        def objective(q):
            pp=rpoints(q)
            orientation=(d.xmat[rf].reshape(2,3,3)-gfoot_R).ravel()*.15
            clearance_penalty=[max(0,.004-mujoco.mj_geomDistance(m,d,a,b,.05,None))*5 for a,b in collision_pairs]
            rest_weight=np.full(23,.015);rest_weight[[12,17,22]]=.12
            return np.r_[((pp-target)*weights).ravel(),orientation,(q-q_previous)*.07,
                         (q-q0)*rest_weight,clearance_penalty]
        result=least_squares(objective,np.clip(q_previous,lo,hi),bounds=(lo,hi),max_nfev=55,ftol=1e-5,xtol=1e-5,gtol=1e-5)
        q_previous=result.x.copy();qs.append(result.x);errs.append(float(np.linalg.norm(rpoints(result.x)[:2]-target[:2],axis=1).max()))
        gd.qpos[:]=np.r_[row[:3],row[3:7][[3,0,1,2]],row[7:]];mujoco.mj_forward(g,gd)
        clearance.append(support_z(g,gd,sf))
        if fi%60==0:print(name,fi,'/',len(raw),'feet_error',round(errs[-1],4),flush=True)
    qs=np.asarray(qs);qs=np.clip(savgol_filter(qs,5,3,axis=0),lo,hi)
    times=np.arange(len(raw))/30;newt=np.arange(int(times[-1]*50)+1)/50
    qnew=PchipInterpolator(times,qs)(newt)
    # Nominal URDF limits; the torso additionally uses a conservative 8 rad/s
    # retargeting cap to suppress rapid redundant IK twists.
    import xml.etree.ElementTree as ET
    urdf=ET.parse(ASSET/'rpo/rpo.urdf').getroot()
    speed_by_name={j.get('name'):float(j.find('limit').get('velocity')) for j in urdf.findall('joint') if j.find('limit') is not None}
    nominal_vmax=np.array([speed_by_name[n] for n in rnames])
    vmax=nominal_vmax.copy();vmax[12]=min(vmax[12],8.)
    q_before=qnew.copy()
    qnew=smooth_with_limits(qnew,lo,hi,.95*vmax,.02)
    qnew=smooth_parallel_ankles(qnew,lo,hi,.02)
    quat=Slerp(times,Rotation.from_quat(raw[:,3:7]))(newt).as_quat()
    root=PchipInterpolator(times,raw[:,:3])(newt);root[:,:2]-=root[0,:2]
    # Preserve measured source sole clearance (including flight), adjusting only
    # for the difference in skeletons and foot collision geometry.
    src_clear=np.interp(newt,times,clearance);correction=[]
    for i in range(len(newt)):
        d.qpos[:]=np.r_[root[i],quat[i][[3,0,1,2]],qnew[i]];mujoco.mj_forward(m,d)
        correction.append(max(0,src_clear[i])-support_z(m,d,rf))
    correction=savgol_filter(correction,9,3)
    root[:,2]+=correction
    output=np.column_stack((root,quat,qnew))
    dest=OUT/(name+'.csv');np.savetxt(dest,output,delimiter=',',fmt='%.9g')
    vel=np.gradient(qnew,.02,axis=0)
    accumulated=(Rotation.from_quat(quat[:-1]).inv()*Rotation.from_quat(quat[1:])).as_rotvec().sum(0)
    report={'source_name':name,'kind':kind,'frames':len(output),'fps':50,'seconds':(len(output)-1)/50,'joint_names':rnames,'max_ik_foot_error_m_before_smoothing':max(errs),'mean_ik_foot_error_m_before_smoothing':float(np.mean(errs)),'speed_smoothing_max_change_rad':float(abs(qnew-q_before).max()),'max_joint_speed_rad_s':np.max(abs(vel),axis=0).tolist(),'joint_speed_limits_rad_s':nominal_vmax.tolist(),'retarget_speed_caps_rad_s':vmax.tolist(),'parallel_motor_speed_cap_rad_s':7.,'integrated_body_rotation_rad':accumulated.tolist(),'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'method':'named robot-to-robot bounded IK with collision penalty and constrained temporal smoothing, including parallel ankle motor speed; source foot clearance preserved','dynamic_feasibility_verified':False,'hardware_ready':False}
    dest.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-source',type=Path,required=True,help='Authorized G1 run1_subject2.csv, 30 Hz')
    parser.add_argument('--backflip-source',type=Path,required=True,help='Authorized G1 90_19_stageii_unitree_g1.csv, 30 Hz')
    args=parser.parse_args()
    run_path=args.run_source
    raw=np.loadtxt(run_path,delimiter=',');length=240
    candidates=[]
    for start in range(0,len(raw)-length,30):
        x=raw[start:start+length];v=np.diff(x[:,:2],axis=0)*30;speed=np.linalg.norm(v,axis=1)
        straight=np.linalg.norm(x[-1,:2]-x[0,:2])/max(np.linalg.norm(v,axis=1).sum()/30,1e-8)
        if straight>.97 and .7<speed.mean()<1.5:
            candidates.append((abs(speed.mean()-1.1)+speed.std(),start))
    if not candidates:raise ValueError('No straight run segment')
    start=min(candidates)[1];run=raw[start:start+length].copy()
    np.savetxt(source/'run_g1_source_30hz.csv',run,delimiter=',')
    print('selected run source frames',start,start+length,flush=True)
    flip_path=args.backflip_source
    flip=np.loadtxt(flip_path,delimiter=',')
    np.savetxt(source/'backflip_g1_source_30hz.csv',flip,delimiter=',')
    reports=[retarget(run,'run_reference','running'),retarget(flip,'backflip_reference','backflip_candidate')]
    manifest={'run':{'source':'https://github.com/wenconggan/Lafan1_retarget','clip':'run1_subject2','frame_range':[start,start+length],'fps':30},'backflip':{'source':'https://github.com/lez666/chunwan-flip','clip':'CMU 90_19 monkey backflip via AMASS/GMR','fps':30},'reports':reports,'notes':'Public source provenance retained; references are not actuator commands.'}
    (ROOT/'data/reference_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')

if __name__=='__main__':main()
