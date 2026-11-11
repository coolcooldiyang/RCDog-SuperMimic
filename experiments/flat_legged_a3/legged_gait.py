"""Go1 legged_control trot timing and OCS2's two-node-pair cubic swing.

Upstream: Qiayuan Liao et al., legged_control, BSD-3-Clause; OCS2
SplineCpg, Farbod Farshidian, BSD-3-Clause. See reference_candidates.
Horizontal support trajectories and RCDog IK are adaptations in this project.
"""
import numpy as np


def hermite(p0,p1,v0,v1,u,duration):
    return ((2*u**3-3*u*u+1)*p0+(u**3-2*u*u+u)*duration*v0
            +(-2*u**3+3*u*u)*p1+(u**3-u*u)*duration*v1)


def swing_z(s,duration,height,activity=1.):
    scaling=min(1.,duration/.15)
    return np.where(s<.5,
        hermite(0.,height,.05*activity*scaling,0.,2*s,duration/2),
        hermite(height,0.,0.,-.1*activity*scaling,2*s-1,duration/2))


def stance_xy(nominal,velocity,yaw,tau):
    """Body-relative foot position with exactly zero world support-foot speed."""
    angle=yaw*tau;c,s=np.cos(angle),np.sin(angle)
    delta=velocity*tau if abs(yaw)<1e-8 else np.array([
        (s*velocity[0]-(1-c)*velocity[1])/yaw,
        ((1-c)*velocity[0]+s*velocity[1])/yaw])
    return np.array([[c,s],[-s,c]])@(nominal-delta)


def swing_xy(start,end,velocity,yaw,s,duration):
    def relative_v(p):return -velocity-yaw*np.array([-p[1],p[0]])
    return hermite(start,end,relative_v(start),relative_v(end),s,duration)
