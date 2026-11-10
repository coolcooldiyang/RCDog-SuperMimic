# RCDog adaptation, 2026-11-07; derived from Uni-SuperMimic where applicable.
# RCDog extension / combined software: GPL-3.0-only. See LICENSE and THIRD_PARTY.md.
"""RCDog motor-space saturation at every native physics substep.

The public backend tensor APIs keep the existing UniLab reset/read lifecycle.
Position targets are converted to torques before each configured physics step (5 ms for GYM-aligned controls), so PD
feedback and parallel motor saturation also match the independent CPU audit.
"""
from types import MethodType
import torch
import os
import math
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
    # Optional training-only latency and first-order torque response perturbations.
    # Motor feedback stays local/current; USB transport delays targets, not native PD.
    randomized=os.environ.get('RCDOG_TRAIN_DR','0')=='1'
    n=first['qpos'].shape[0]
    ratio2=torch.tensor([6.33**2,6.33**2,12.66**2]*4,device=device)
    if randomized:
        blend=torch.rand(n,1,device=device)
        kp=kp+blend*(torch.floor(kp/ratio2*1280)/1280*ratio2-kp)
        kd=kd+blend*(torch.floor(kd/ratio2*1280)/1280*ratio2-kd)
    delay=torch.randint(0,4,(n,),device=device) if randomized else torch.zeros(n,device=device,dtype=torch.long)
    lag=torch.rand(n,1,device=device)*.005 if randomized else torch.zeros(n,1,device=device)
    response=1.-torch.exp(-.005/lag.clamp_min(1e-6))
    queue=None;last_q=None;filtered=torch.zeros(n,12,device=device)
    index=torch.arange(n,device=device)
    def advance(self,target,nsteps):
        nonlocal queue,last_q,filtered
        self.parallel_invalid.zero_()
        result=None
        for _ in range(nsteps):
            # GPU views are live; CPU backends return fresh detached snapshots.
            view=self.get_state_views(('qpos','qvel'))
            q=view['qpos'][:,7:]
            if queue is None:
                queue=target[None].repeat(4,1,1);last_q=q.clone()
            reset=(q-last_q).abs().amax(-1)>.3
            queue=torch.cat((target[None],queue[:-1]),0)
            queue[:,reset]=target[reset];filtered[reset]=0.
            delayed=queue[delay,index]
            torque,valid=compute(delayed,q,view['qvel'][:,6:],kp,kd,effort)
            filtered=filtered+response*(torque-filtered)
            torque=filtered
            last_q=q.clone()
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
