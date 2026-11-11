from pathlib import Path
import sys,json
sys.path.insert(0,str(Path(__file__).resolve().parent/'Uni-SuperMimic/src'))
import torch
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.mdp import MDP
from unisupermimic.rl.algos.supermimic.geometry import so3_exp
m=object.__new__(MDP);B=3
v=torch.tensor([[1.,2.,3.]]).repeat(B,1);omega=torch.tensor([[.1,-.2,1.5]]).repeat(B,1)
qv=torch.full((B,12),90.);q0=torch.zeros(B,12)
old=(torch.zeros(B,3),so3_exp(torch.zeros(B,3)),v,omega,q0,qv)
new=(.02*v,so3_exp(.02*omega),v,omega,.02*qv,qv)
for age in [0.,.4,.75]:
 sensor=m.feedback_state(old,new,torch.full((B,),age));t=.02*(1-age)
 torch.testing.assert_close(sensor[0],t*v);torch.testing.assert_close(sensor[4],t*qv)
 torch.testing.assert_close(sensor[1],so3_exp(t*omega),atol=3e-6,rtol=1e-5)
 assert bool((sensor[5]>30.1).all())  # Feedback is aged, never clipped.
a=torch.zeros(B,3,requires_grad=True);c=torch.tensor([[.01,-.01,.02]]).repeat(B,1).requires_grad_()
old2=list(old);new2=list(new);old2[1]=so3_exp(a);new2[1]=so3_exp(c)
z=m.feedback_state(tuple(old2),tuple(new2),torch.full((B,),.4));z[1].square().sum().backward()
assert torch.isfinite(a.grad).all() and torch.isfinite(c.grad).all()
histold=tuple(x[:,None].repeat((1,5)+(1,)*(x.ndim-1)) for x in old)
histnew=tuple(x[:,None].repeat((1,5)+(1,)*(x.ndim-1)) for x in new)
h=m.feedback_state(histold,histnew,torch.full((B,1),.4));assert h[1].shape==(B,5,3,3) and h[4].shape==(B,5,12)
report=dict(passed=True,constant_velocity_8ms=True,rotation_8ms=True,history_shape=True,
 finite_gradients=True,feedback_not_clipped=True,method='20ms state interpolation; exact1ms CPU delay audit remains unchanged')
Path('feedback_latency_unit_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
