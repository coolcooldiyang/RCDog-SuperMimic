# RPO adaptation, 2026-10-26; derived from Uni-SuperMimic where applicable.
# RPO extension / combined software: GPL-3.0-only. See LICENSE and THIRD_PARTY.md.
# SPDX-License-Identifier: GPL-3.0
# Geometry adapted from Roboparty decouple_rpo.cpp, Copyright (C) 2025-2026 Luo1imasi.
"""Differentiable RPO parallel-ankle geometry and bounded motor torques.

Lengths and branch selection match the deployed RDK implementation. Invalid
poses are reported explicitly; finite clamping only keeps simulation/gradients
defined until the environment terminates those rows.
"""
import torch

def ankle_jacobian(q: torch.Tensor, left: bool):
    pitch,roll=q.unbind(-1)
    cp,sp,cr,sr=pitch.cos(),pitch.sin(),roll.cos(),roll.sin()
    spacing=.04235 if left else -.04235
    rows=[];angles=[];margins=[]
    for x0,length,bar_x in [(-.02,.18,-.02),(.02,.11,.02)]:
        C=torch.stack((cp*x0+sp*sr*spacing,cr*spacing,-sp*x0+cp*sr*spacing),-1)
        A=torch.tensor([0.,spacing,length],device=q.device,dtype=q.dtype)
        a,b=C[...,0],length-C[...,2]
        c=(length**2-.02**2-((C-A)**2).sum(-1))/.04
        ab=a*a+b*b
        discriminant=a*a*(ab-c*c)
        raw=(b*c+torch.sqrt(discriminant.clamp(min=1e-16)))/ab.clamp(min=1e-10)
        theta=torch.asin(raw.clamp(-1+1e-7,1-1e-7))*torch.where(a<0,1.,-1.)
        bar=torch.stack((bar_x*theta.cos(),torch.zeros_like(theta),-bar_x*theta.sin()),-1)
        rod=C-A-bar
        cross=torch.cross(C,rod,dim=-1)
        den=torch.cross(bar,rod,dim=-1)[...,1]
        safe_den=torch.where(den>=0,den.clamp(min=1e-8),den.clamp(max=-1e-8))
        rows.append(torch.stack((cross[...,1],cross[...,0]*cp-cross[...,2]*sp),-1)/safe_den[...,None])
        angles.append(theta)
        margins.append(torch.minimum(1-raw.abs(),(ab-c*c)/ab.clamp(min=1e-10)))
    J=torch.stack(rows,-2)
    margin=torch.stack(margins,-1).min(-1).values
    det=J[...,0,0]*J[...,1,1]-J[...,0,1]*J[...,1,0]
    valid=(margin>1e-6)&(det.abs()>.02)&(J.abs().amax(dim=(-2,-1))<20)
    return J,torch.stack(angles,-1),valid,margin

def bounded_pd(target,q,qd,kp,kd,effort):
    requested=kp*(target-q)-kd*qd
    torque=requested.clamp(-effort,effort)
    valid=torch.ones(q.shape[:-1],device=q.device,dtype=torch.bool)
    for start,left in [(4,True),(10,False)]:
        J,_,ok,_=ankle_jacobian(q[...,start:start+2],left)
        a,b,c,d=J[...,0,0],J[...,0,1],J[...,1,0],J[...,1,1]
        det=a*d-b*c
        det=torch.where(det>=0,det.clamp(min=1e-5),det.clamp(max=-1e-5))
        inv_t=torch.stack((d,-c,-b,a),-1).reshape(J.shape)/det[...,None,None]
        motor=(inv_t@requested[...,start:start+2,None]).squeeze(-1).clamp(-27,27)
        joint=(J.transpose(-1,-2)@motor[...,None]).squeeze(-1)
        torque[...,start:start+2]=torch.where(ok[...,None],joint,0.)
        valid=valid&ok
    return torque,valid
