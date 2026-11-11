"""Training-only analytical terrain queries; actor receives no terrain measurements."""
import json,os
from pathlib import Path
import torch
from unisupermimic.rl.algos.supermimic.geometry import yaw_mat
MANIFEST=Path(os.environ['RCDOG_TERRAIN_MANIFEST']) if os.environ.get('RCDOG_TERRAIN_MANIFEST') else None
ENABLED=MANIFEST is not None
REWARD_V2=os.environ.get('RCDOG_TERRAIN_REWARD_V2','0')=='1'
REWARD_V3=os.environ.get('RCDOG_TERRAIN_REWARD_V3','0')=='1'
_DATA=json.loads(MANIFEST.read_text()) if ENABLED else None
_CACHE={}
def tensors(device,dtype):
 key=(str(device),dtype)
 if key not in _CACHE:
  rows=_DATA['rows'];rough=torch.zeros(len(rows),12,6,device=device,dtype=dtype)
  for s in _DATA['surfaces']:
   if s['name'].startswith('rough_'):
    x0,x1,y0,y1=s['bounds'];r=round(((y0+y1)/2)/4);i=round((x0-11)/.25);j=round((y0-(rows[r]['y']-.9))/.3);rough[r,i,j]=s['plane'][2]
  _CACHE[key]=(torch.tensor([[r['step'],r['tread'],r['slope_deg']] for r in rows],device=device,dtype=dtype),rough)
 return _CACHE[key]
def height(xy):
 table,rough=tensors(xy.device,xy.dtype);x,y=xy[...,0],xy[...,1]
 row=torch.round(y/4).long().clamp(0,len(_DATA['rows'])-1);cy=row*4
 h,t,slope=table[row].unbind(-1);start=1.2;top=start+4*t;down=top+.8;end=down+4*t
 up=((torch.floor((x-start)/t)+1).clamp(0,4))*h
 descending=(3-torch.floor((x-down)/t)).clamp(0,3)*h
 z=torch.where(x<down,up,descending);z=torch.where((x>=start)&(x<end),z,torch.zeros_like(z))
 ramp=torch.minimum((x-6.5).clamp(0,1.2),(9.5-x).clamp(0,1.2))*torch.tan(slope*torch.pi/180)
 z=torch.maximum(z,ramp)
 i=torch.floor((x-11)/.25).long().clamp(0,11);j=torch.floor((y-cy+.9)/.3).long().clamp(0,5)
 zz=rough[row,i,j];z=torch.where((x>=11)&(x<14)&((y-cy).abs()<.9),zz,z)
 return torch.where((y-cy).abs()<1.2,z,torch.zeros_like(z))
def scan(state):
 p,R=state[:2];u=torch.linspace(-.6,.6,5,device=p.device,dtype=p.dtype);v=torch.linspace(-.4,.4,5,device=p.device,dtype=p.dtype)
 xx,yy=torch.meshgrid(u,v,indexing='ij');offset=torch.stack((xx.flatten(),yy.flatten(),torch.zeros(25,device=p.device,dtype=p.dtype)),-1)
 xy=p.unsqueeze(-2)+(yaw_mat(R).unsqueeze(-3)@offset.unsqueeze(-1)).squeeze(-1)
 return ((height(xy[...,:2])-p[...,2:3]+.35)/.2).clamp(-5,5)
def support_height(position):
 # Smooth body support reference across stair edges using a body-sized stencil.
 delta=torch.tensor([[-.22,-.14],[-.22,.14],[.22,-.14],[.22,.14],[0,0]],device=position.device,dtype=position.dtype)
 return height(position[...,:2].unsqueeze(-2)+delta).mean(-1)

def swing_clearance_target(foot_p, swing, reference_velocity, base_clearance=.04):
 # Training reward only: include the upcoming toe path before crossing a riser.
 direction=reference_velocity[...,:2]
 direction=direction/torch.linalg.vector_norm(direction,dim=-1,keepdim=True).clamp_min(.05)
 offsets=torch.tensor([0.,.04,.08,.12],device=foot_p.device,dtype=foot_p.dtype)
 path=foot_p[...,:2].unsqueeze(-2)+direction[...,None,None,:]*offsets[...,None]
 current=height(foot_p[...,:2]);highest=height(path).amax(-1)
 return current+.02+(highest-current+base_clearance)*swing

def calf_clearance_cost(knee, foot):
 # The MJCF calf collision is an 11 mm radius capsule along knee->foot.
 # Sample its centreline with a 5 mm clearance margin; sole contact is allowed.
 u=torch.linspace(0.,1.,9,device=knee.device,dtype=knee.dtype)
 points=knee.unsqueeze(-2)+(foot-knee).unsqueeze(-2)*u[...,None]
 shortfall=torch.relu(height(points[...,:2])+.016-points[...,2])
 return (shortfall/.025).square().amax(-1).mean(-1)
