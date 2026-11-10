"""Render selected CPU MuJoCo rollouts from the final evaluation."""
import argparse,json
from pathlib import Path
import numpy as np,mujoco,imageio.v2 as imageio
from PIL import Image,ImageDraw,ImageFont
r=Path(__file__).resolve().parent
font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',15)
small=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',13)
p=argparse.ArgumentParser();p.add_argument('--rollout-dir',required=True);p.add_argument('--output',default='sim2sim_lateral150.mp4');a=p.parse_args();root=Path(a.rollout_dir)
names=['forward_1.5_d0','backward_1.5_d0','left_1.5_d0','cw_1.5_d0'];labels=['FORWARD: 1.5 m/s','BACKWARD: 1.5 m/s','LEFT: 1.5 m/s','CW: 1.5 rad/s']
records=[np.load(root/(n+'.npz')) for n in names]
model=mujoco.MjModel.from_xml_path(str(r.parents[1]/'src/unisupermimic/assets/robots/rcdog/scene_supermimic_motor.xml'));data=mujoco.MjData(model);renderer=mujoco.Renderer(model,height=360,width=640);cam=mujoco.MjvCamera();cam.distance=1.7;cam.elevation=-20;cam.azimuth=140
with imageio.get_writer(r/a.output,fps=25,codec='libx264',quality=7,macro_block_size=16) as writer:
 for i in range(0,max(len(x['time']) for x in records),2):
  panels=[]
  for rec,label in zip(records,labels):
   j=min(i,len(rec['time'])-1);data.qpos[:]=rec['qpos'][j];data.qvel[:]=rec['qvel'][j];mujoco.mj_forward(model,data);cam.lookat[:]=data.qpos[:3];renderer.update_scene(data,camera=cam);im=Image.fromarray(renderer.render().copy());draw=ImageDraw.Draw(im);draw.rectangle((0,0,639,52),fill=(15,20,25));t=float(rec['time'][j]);phase='ZERO COMMAND' if t<2 or t>=11 else 'MOVE';draw.text((10,8),f'{label} | t={t:.2f}s | {phase}',fill='white',font=font);v=rec['measured'][j];draw.text((10,28),f'measured: vx={v[0]:.2f} vy={v[1]:.2f} yaw={v[2]:.2f} | tilt={rec["tilt_deg"][j]:.1f} deg',fill='yellow',font=small);panels.append(np.asarray(im))
  frame=np.concatenate((np.concatenate(panels[:2],axis=1),np.concatenate(panels[2:],axis=1)),axis=0);writer.append_data(frame)
renderer.close();print(r/a.output)
