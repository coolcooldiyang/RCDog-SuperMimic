"""Phase-matched stopping of the exported nine-second locomotion references."""
import math
import numpy as np


def validate_forward_loop(reference, start=150, end=250):
    """Only loop the verified steady forward section; keep actor history intact."""
    if not 0 <= start < end < len(reference['goal']):
        raise ValueError('无效连续步态区间')
    for key, tolerance in [('q_ref', 1e-5), ('qd_ref', 1e-3),
                           ('goal', 1e-4), ('R_ref', 1e-5)]:
        if not np.allclose(reference[key][start], reference[key][end], atol=tolerance, rtol=0):
            raise ValueError('连续步态接点不匹配：'+key)


def forward_loop_frame(frame, start=150, end=250):
    return frame if frame < end else start+(frame-start) % (end-start)


def deceleration_frame(reference, frame, dt=.02):
    """Return a tail frame, or None while still in the entry section.

    These bundles have a two-second deceleration/settling tail. Search one
    gait period near its start; do not reset the actor's observation history.
    """
    count=len(reference['goal'])
    frame=max(0,min(int(frame),count-1))
    if frame*dt<1.6:return None
    tail=max(0,count-1-round(2./dt))
    if frame>=tail:return frame
    candidates=np.arange(tail,min(count-1,tail+round(.4/dt)+1))
    if not len(candidates):raise ValueError('参考片段没有减速段')
    q=np.asarray(reference['q_ref']);dq=np.asarray(reference['qd_ref'])
    cost=np.sum((q[candidates]-q[frame])**2,axis=1)
    cost+=.002*np.sum((dq[candidates]-dq[frame])**2,axis=1)
    return int(candidates[np.argmin(cost)])


def jump_to_deceleration(reference, frame, dt=.02):
    target=deceleration_frame(reference,frame,dt)
    if target is None:return None
    # Preserve the current reference heading when skipping a turning segment.
    r=reference['R_ref']
    a=math.atan2(r[frame,1,0],r[frame,0,0])-math.atan2(r[target,1,0],r[target,0,0])
    c,s=math.cos(a),math.sin(a)
    reference['R_ref']=np.array([[c,-s,0],[s,c,0],[0,0,1.]])@r
    return target
