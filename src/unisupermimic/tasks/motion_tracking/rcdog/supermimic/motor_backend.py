# RCDog adaptation, 2026-10-27; derived from Uni-SuperMimic where applicable.
# RCDog extension / combined software: GPL-3.0-only. See LICENSE and THIRD_PARTY.md.
"""Motor-space saturation at every native physics substep.

The public backend tensor APIs keep the existing UniLab reset/read lifecycle.
Position targets are converted to torques before each 4 ms physics step, so PD
feedback and parallel motor saturation also match the independent CPU audit.
"""
from types import MethodType
import torch
def bounded_pd(target,q,qd,kp,kd,effort):
    return (kp*(target.clamp(-1.55,1.55)-q)-kd*qd).clamp(-effort,effort), torch.ones(q.shape[0],device=q.device,dtype=torch.bool)

from .robot import KP,KD,EFFORT

def install_motor_pd(backend):
    original=backend.step_tensor
    first=backend.get_state_views(('qpos','qvel'))
    device=first['qpos'].device
    assert first['qpos'].shape[1]==19 and first['qvel'].shape[1]==18
    kp=torch.tensor(KP,device=device,dtype=torch.float32)
    kd=torch.tensor(KD,device=device,dtype=torch.float32)
    effort=torch.tensor(EFFORT,device=device,dtype=torch.float32)
    backend.parallel_invalid=torch.zeros(first['qpos'].shape[0],device=device,dtype=torch.bool)
    compute=torch.compile(bounded_pd) if device.type=='cuda' else bounded_pd
    def advance(self,target,nsteps):
        self.parallel_invalid.zero_()
        result=None
        for _ in range(nsteps):
            # GPU views are live; CPU backends return fresh detached snapshots.
            view=self.get_state_views(('qpos','qvel'))
            torque,valid=compute(target,view['qpos'][:,7:],view['qvel'][:,6:],kp,kd,effort)
            self.parallel_invalid.logical_or_(~valid)
            result=original(torque,1)
        return result
    def substeps(self,target,nsteps=1):
        # The backend writes torque controls during the loop. Preserve position
        # targets even when the caller's control carrier aliases backend memory.
        target=target.to(device).clone()
        if device.type!='cuda':
            return advance(self,target,nsteps)
        import warp as wp
        current=torch.cuda.current_stream(device)
        external=wp.stream_to_torch(wp.get_stream(str(device)))
        external.wait_stream(current)
        with torch.cuda.stream(external):
            result=advance(self,target,nsteps)
        current.wait_stream(external)
        return result
    backend.step_tensor=MethodType(substeps,backend)
