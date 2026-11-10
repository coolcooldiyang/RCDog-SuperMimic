"""Additional explicit full-speed diagonals for a later refinement stage."""
from pathlib import Path
import json,numpy as np
from command_reference import CommandReference
r=Path(__file__).resolve().parent;folder=r/'diagonal_curriculum';folder.mkdir(exist_ok=True);files=[];audit=[]
def generate(name,cmd):
 p=CommandReference(r/'command_gait_table.npz');rows=[]
 for i in range(901):
  t=i*.02;p.advance(cmd if 2<t<11 else [0,0,0]);rows.append(np.r_[p.position,p.quaternion_xyzw(),p.joints()])
 arr=np.asarray(rows);f=folder/(name+'.csv');np.savetxt(f,arr,delimiter=',',fmt='%.10g');files.append(str(f));audit.append({'clip':name,'command':cmd,'max_ref_joint_speed':np.abs(np.gradient(arr[:,7:],.02,axis=0)).max(0).tolist()})
for speed in [.75,1.5]:
 for name,x,y in [('fl',1,1),('fr',1,-1),('bl',-1,1),('br',-1,-1)]:generate(f'diag{int(speed*100):03}_{name}',[x*speed/np.sqrt(2),y*speed/np.sqrt(2),0])
(folder/'audit.json').write_text(json.dumps(audit,indent=2));print('generated',len(files),'extra clips')
