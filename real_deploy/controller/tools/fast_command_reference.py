"""Batch interpolation for the verified continuous reference, without changing its math."""
import numpy as np

CORNERS=np.array([[x,y,z] for x in range(2) for y in range(2) for z in range(2)])

def batched_joints(planner,commands,phases):
    if hasattr(planner,'grids'):
        # Exported grids may have different ranges and node counts per axis.
        lo=np.stack([np.clip(np.searchsorted(g,commands[:,axis],side='right')-1,0,len(g)-2)
                     for axis,g in enumerate(planner.grids)],axis=1)
        weight=np.stack([np.clip((commands[:,axis]-g[lo[:,axis]])/
                         (g[lo[:,axis]+1]-g[lo[:,axis]]),0,1)
                         for axis,g in enumerate(planner.grids)],axis=1)
    else:
        # Original immutable 0.5 m/s bundle predates explicit grid metadata.
        coordinate=np.clip((commands+.5)/.5,0,2)
        lo=np.minimum(coordinate.astype(int),1);weight=coordinate-lo
    indexes=lo[:,None,:]+CORNERS[None,:,:]
    weights=np.prod(np.where(CORNERS[None,:,:],weight[:,None,:],1-weight[:,None,:]),axis=2)
    phase=phases*planner.table.shape[3];base=phase.astype(int);fraction=phase-base
    i=base%planner.table.shape[3];j=(i+1)%planner.table.shape[3]
    x,y,z=indexes[:,:,0],indexes[:,:,1],indexes[:,:,2]
    a=planner.table[x,y,z,i[:,None]];b=planner.table[x,y,z,j[:,None]]
    return np.sum(weights[:,:,None]*((1-fraction[:,None,None])*a+fraction[:,None,None]*b),axis=1)

def fast_goal(planner,requested):
    p=planner.copy();positions=[p.position.copy()];headings=[p.heading]
    commands=[p.command.copy()];phases=[p.phase]
    for _ in range(11):
        p.advance(requested);positions.append(p.position.copy());headings.append(p.heading)
        commands.append(p.command.copy());phases.append(p.phase)
    positions=np.array(positions);headings=np.array(headings)
    q=batched_joints(planner,np.array(commands),np.array(phases))
    angles=planner.yaw0+headings[1:11];c,s=np.cos(angles),np.sin(angles)
    rotations=np.zeros((10,3,3));rotations[:,0,0]=c;rotations[:,0,1]=-s
    rotations[:,1,0]=s;rotations[:,1,1]=c;rotations[:,2,2]=1
    R=planner.rotation();world_v=(positions[2:]-positions[:-2])/.04
    angular=np.zeros((10,3));angular[:,2]=(headings[2:]-headings[:-2])/.04
    return np.concatenate(((positions[1:11]-positions[0])@R,
        (R.T@rotations)[:,:,:2].reshape(10,6),
        (rotations.transpose(0,2,1)@world_v[:,:,None])[:,:,0],angular,
        q[1:11],(q[2:]-q[:-2])/.04*.1),axis=1).reshape(-1)
