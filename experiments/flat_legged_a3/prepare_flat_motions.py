"""50Hz all-direction flat clips with immediate zero-speed and short restarts."""
from pathlib import Path
import json
import numpy as np
from command_reference import CommandReference
R=Path(__file__).resolve().parent;T=R/'reference_candidates/legged_control/command_gait_table.npz'
O=R.parents[1]/'src/unisupermimic/assets/motions/rcdog_flat_legged';O.mkdir(exist_ok=True)
clips=[];maxratio=0.


def gen(name,command,seconds):
    global maxratio
    p=CommandReference(T);data=[];sw=[];st=[];h=[]
    for k in range(round(seconds/.02)+1):
        p.advance(command(k*.02));data.append(np.r_[p.position,p.quaternion_xyzw(),p.joints()])
        a,b,c=p.swing_details();sw.append(a);st.append(b);h.append(c)
    data=np.array(data);qd=np.gradient(data[:,7:],.02,axis=0)
    ratio=float(np.max(abs(qd)/np.tile([30.1,30.1,15.05],4)));maxratio=max(maxratio,ratio)
    assert ratio<1.,(name,ratio)
    f=O/(name+'.csv');np.savetxt(f,data,delimiter=',',fmt='%.10g')
    np.savez_compressed(f.with_suffix('.gait.npz'),swing=sw,stance=st,lift=h)
    clips.append(dict(name=name,path=str(f.relative_to(R.parents[1])),seconds=seconds,
                      reference_joint_speed_ratio=ratio))


for speed in [.1,.25,.5,.75,1.,1.5]:
    for name,v in [('forward',[speed,0,0]),('back',[-speed,0,0]),('left',[0,speed,0]),
                   ('right',[0,-speed,0]),('ccw',[0,0,speed]),('cw',[0,0,-speed])]:
        gen(f'flat_lc_{int(speed*100):03}_{name}',lambda t,v=v:v if 2<=t<11 else [0,0,0],16)
for speed in [.5,1.,1.5]:
    for name,x,y in [('fl',1,1),('fr',1,-1),('bl',-1,1),('br',-1,-1)]:
        v=[x*speed/np.sqrt(2),y*speed/np.sqrt(2),0]
        gen(f'flat_lc_{int(speed*100):03}_{name}',lambda t,v=v:v if 2<=t<11 else [0,0,0],16)
for i in range(4):gen(f'flat_lc_stand{i}',lambda t:[0,0,0],12)
for seed in range(12):
    rng=np.random.default_rng(61110+seed);commands=rng.uniform(-1,1,(10,3))*[1.5,1.5,1.5]
    commands[::2]=0
    gen(f'flat_lc_transition{seed}',lambda t,c=commands:c[min(int(t/3),9)],30)
for i,v in enumerate(([1.5,0,0],[0,-1.5,0],[0,0,1.5],[.8,.8,.8])):
    gen(f'flat_lc_fast_restart{i}',lambda t,v=v:v if (1<=t<5 or 5.4<=t<9) else [0,0,0],13)
(R/'flat_clips.json').write_text(json.dumps(clips,indent=2)+'\n')
(R/'reference_candidates/legged_control/motion_audit.json').write_text(json.dumps(dict(
    clips=len(clips),max_reference_joint_velocity_ratio=maxratio,src_fps=50,
    reference_family='legged_control_trot',phase_labels_independent_of_height=True,
    neutral_root_velocity_immediate_zero=True,neutral_swing_settle_max_s=.3),indent=2)+'\n')
print('clips',len(clips),'max_speed_ratio',maxratio,flush=True)
