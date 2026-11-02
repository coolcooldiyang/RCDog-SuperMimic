"""Build a floating-base RCDog model from the source URDF in ../source."""
from pathlib import Path
import json,hashlib,shutil,xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.optimize import least_squares

SCRIPT_DIR=Path(__file__).resolve().parent
REPO=SCRIPT_DIR.parents[1]
ROOT=REPO
OUT=REPO/'src/unisupermimic/assets/robots/rcdog'
OUT.mkdir(parents=True,exist_ok=True)
shutil.copytree(ROOT/'source/meshes',OUT/'meshes',dirs_exist_ok=True)
# Rebuild the importer input from the untouched source URDF.
source_tree=ET.parse(ROOT/'source/robot.urdf');source_root=source_tree.getroot()
for link in source_root.findall('link'):
 for collision in list(link.findall('collision')):link.remove(collision)
mujoco_tag=source_root.find('mujoco')
if mujoco_tag is None:mujoco_tag=ET.SubElement(source_root,'mujoco')
compiler=mujoco_tag.find('compiler')
if compiler is None:compiler=ET.SubElement(mujoco_tag,'compiler')
compiler.set('discardvisual','false');compiler.set('fusestatic','false')
ET.SubElement(source_root,'link',name='world')
joint=ET.SubElement(source_root,'joint',name='floating_base_joint',type='floating')
ET.SubElement(joint,'parent',link='world');ET.SubElement(joint,'child',link='base_link')
source_tree.write(ROOT/'source/floating_visual.urdf')
imported=mujoco.MjModel.from_xml_path(str(ROOT/'source/floating_visual.urdf'))
mujoco.mj_saveLastXML(str(ROOT/'source/imported_visual.xml'),imported)
r=ET.parse(ROOT/'source/imported_visual.xml').getroot();r.set('model','rcdog27')
ET.SubElement(r,'option',timestep='0.004',integrator='implicitfast',solver='Newton',iterations='10',gravity='0 0 -9.81',cone='pyramidal')
default=ET.SubElement(r,'default')
ET.SubElement(default,'joint',damping='0.02',frictionloss='0.02',armature='0.005')
ET.SubElement(default,'geom',condim='3',friction='0.8 0.02 0.001',solref='0.006 1')
ET.SubElement(r,'visual');ET.SubElement(r.find('visual'),'global',offwidth='1280',offheight='720')
world=r.find('worldbody');base=world.find("body[@name='base_link']")
assert base is not None
# Meshes remain visual only. Primitive collision approximations are explicit.
for body in base.iter('body'):
 for g in body.findall('geom'):
  g.set('contype','0');g.set('conaffinity','0');g.set('group','2');g.set('rgba','0.68 0.72 0.78 1')
source=mujoco.MjModel.from_xml_path(str(ROOT/'source/imported_visual.xml'));sd=mujoco.MjData(source);mujoco.mj_forward(source,sd)
foot_points={};foot_mesh={}
for leg in ['rr','rl','fr','fl']:
 bid=source.body(leg+'_calf').id;candidates=[]
 for gid in range(source.body_geomadr[bid],source.body_geomadr[bid]+source.body_geomnum[bid]):
  mid=source.geom_dataid[gid];a=source.mesh_vertadr[mid];n=source.mesh_vertnum[mid]
  v=source.mesh_vert[a:a+n]@sd.geom_xmat[gid].reshape(3,3).T+sd.geom_xpos[gid]
  candidates.append((float(v[:,2].min()),gid,v))
 _,gid,v=min(candidates,key=lambda x:x[0]);center=(v.min(0)+v.max(0))/2;center[2]=v[:,2].min()+.02
 local=sd.xmat[bid].reshape(3,3).T@(center-sd.xpos[bid]);foot_points[leg]=local;foot_mesh[leg]=source.mesh(source.geom_dataid[gid]).name
 calf=base.find(f".//body[@name='{leg}_calf']")
 foot=ET.SubElement(calf,'body',name=leg+'_foot',pos=' '.join(map(str,local)))
 ET.SubElement(foot,'geom',name=leg+'_foot_collision',type='sphere',size='.02',mass='0',rgba='.2 .24 .3 1',group='1')
 ET.SubElement(foot,'site',name=leg+'_foot_site',size='.006',rgba='1 .3 .1 1')
 for part,radius in [('hip',.025),('thigh',.019),('calf',.011)]:
  b=base.find(f".//body[@name='{leg}_{part}']")
  endpoint=np.fromstring(b.find('body').get('pos'),sep=' ')
  start=np.zeros(3);end=endpoint*.88
  if part=='thigh':
   # Structural side plates are outboard of the motor axis: preserve the
   # lateral mounting offset rather than sweeping a capsule through the chassis.
   start[2]=endpoint[2];end[2]=endpoint[2]
  ET.SubElement(b,'geom',name=leg+'_'+part+'_collision',type='capsule',fromto=' '.join(map(str,np.r_[start,end])),size=str(radius),rgba='.5 .55 .6 .4',group='3')
# Per-part boxes preserve the chassis openings; one solid trunk box collided
# with the front thigh capsules even in a valid standing assembly.
bid=source.body('base_link').id
for k,gid in enumerate(range(source.body_geomadr[bid],source.body_geomadr[bid]+source.body_geomnum[bid])):
    mid=source.geom_dataid[gid];a=source.mesh_vertadr[mid];n=source.mesh_vertnum[mid]
    vertices=source.mesh_vert[a:a+n]@sd.geom_xmat[gid].reshape(3,3).T+sd.geom_xpos[gid]
    lower,upper=vertices.min(0),vertices.max(0)
    center=(lower+upper)/2;size=np.maximum((upper-lower)*.48,.0005)
    ET.SubElement(base,'geom',name=f'base_collision_{k:02d}',type='box',pos=' '.join(map(str,center)),size=' '.join(map(str,size)),rgba='.5 .55 .6 .3',group='3')
# Inertial properties are preserved; reflected rotor inertia is an explicit assumption.
joints=[j.get('name') for j in base.iter('joint') if j.get('type')!='free']
assert len(joints)==12,joints
actuator=ET.SubElement(r,'actuator')
for name in joints:
 calf='_calf_' in name;effort=23.7*2*.9 if calf else 23.7
 j=base.find(f".//joint[@name='{name}']");j.set('armature','0.020' if calf else '0.005');j.set('actuatorfrcrange',f'{-effort} {effort}')
 ET.SubElement(actuator,'motor',name=name,joint=name,gear='1',ctrllimited='true',ctrlrange=f'{-effort} {effort}')
def save():
 ET.indent(r);ET.ElementTree(r).write(OUT/'supermimic_motor.xml',encoding='unicode')
save();m=mujoco.MjModel.from_xml_path(str(OUT/'supermimic_motor.xml'));d=mujoco.MjData(m)
quat=np.array([np.sqrt(.5),0,0,np.sqrt(.5)]);height=.35
initial=np.r_[0.,0.,height,quat,np.zeros(12)]
d.qpos[:]=initial;mujoco.mj_forward(m,d)
legs=['rr','rl','fr','fl'];foot_ids=[m.body(l+'_foot').id for l in legs]
target=np.array([[(-.2065 if l[0]=='r' else .2065),(.14 if l[1]=='l' else -.14),.02] for l in legs])
def objective(q):
 d.qpos[7:]=q;mujoco.mj_forward(m,d)
 return np.r_[(d.xpos[foot_ids]-target).ravel(),q*.0001]
sol=least_squares(objective,np.zeros(12),bounds=(-1.5,1.5),max_nfev=300,ftol=1e-11,xtol=1e-11,gtol=1e-11)
q0=sol.x;initial[7:]=q0;d.qpos[:]=initial;mujoco.mj_forward(m,d)
error=np.max(np.linalg.norm(d.xpos[foot_ids]-target,axis=1));assert error<.0001,error
key=ET.SubElement(r,'keyframe');ET.SubElement(key,'key',name='stand',qpos=' '.join(map(str,initial)),ctrl=' '.join(['0']*12));save()
scene='''<mujoco model="rcdog27_trot"><include file="supermimic_motor.xml"/>
<asset><texture name="checker" type="2d" builtin="checker" rgb1=".16 .21 .26" rgb2=".24 .3 .35" width="512" height="512"/><material name="ground" texture="checker" texrepeat="4 4" texuniform="true" reflectance=".1"/></asset>
<worldbody><light pos="0 0 4" dir="0 0 -1" directional="true"/><geom name="floor" type="plane" size="0 0 .1" material="ground" friction=".8 .02 .001" condim="3"/></worldbody></mujoco>'''
(OUT/'scene_supermimic_motor.xml').write_text(scene)
tracked=['base_link']+[l+'_'+part for l in legs for part in ['thigh','calf','foot']]
manifest={'source_urdf_sha256':hashlib.sha256((ROOT/'source/robot.urdf').read_bytes()).hexdigest(),'mass_kg':float(m.body_mass.sum()),'joint_names':joints,'tracked_bodies':tracked,'standing_qpos':initial.tolist(),'standing_foot_positions':d.xpos[foot_ids].tolist(),'foot_points_local':{k:v.tolist() for k,v in foot_points.items()},'foot_source_mesh':foot_mesh,'model_forward_source':'-Y, inferred from front/rear leg names and transforms','world_heading_quaternion_wxyz':quat.tolist(),'actuation':{'motor':'Unitree Go1 output shaft','gear_ratio':[2 if '_calf_' in j else 1 for j in joints],'assumed_extra_gear_efficiency':.9,'motor_peak_torque_Nm':23.7,'motor_speed_metadata_rad_s':30.1,'source':'https://github.com/unitreerobotics/unitree_ros/blob/master/robots/go1_description/urdf/go1.urdf','pd_kp':60.,'pd_kd':2.,'armature_assumption_kg_m2':{'hip_thigh':.005,'calf':.02}},'collision_model':'visual STL retained; per-part chassis boxes, leg capsules and four massless sole spheres; self collisions enabled','collision_geom_names':[m.geom(i).name for i in range(m.ngeom) if m.geom_contype[i]],'hardware_ready':False}
(OUT/'model_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(ROOT/'model_audit.json').write_text(json.dumps(manifest,indent=2)+'\n')
# Simple free-base standing check; no fixed root or external support.
m=mujoco.MjModel.from_xml_path(str(OUT/'scene_supermimic_motor.xml'));d=mujoco.MjData(m);mujoco.mj_resetDataKeyframe(m,d,0)
effort=np.array([42.66 if '_calf_' in j else 23.7 for j in joints]);max_tau=np.zeros(12)
for i in range(1250):
 d.ctrl[:]=np.clip(60*(q0-d.qpos[7:])-2*d.qvel[6:],-effort,effort);max_tau=np.maximum(max_tau,abs(d.ctrl));mujoco.mj_step(m,d)
result={'seconds':5,'root_position':d.qpos[:3].tolist(),'root_quaternion':d.qpos[3:7].tolist(),'root_speed':float(np.linalg.norm(d.qvel[:3])),'max_torque':max_tau.tolist(),'ncon':d.ncon,'finite':bool(np.isfinite(d.qpos).all()),'standing_ik_max_foot_error_m':float(error)}
(ROOT/'standing_check.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));print('q0',q0);print('joint_names',joints)
