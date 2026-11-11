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
    if getattr(planner,'family',None)=='legged_control_trot':
        return fast_legged_goal(planner,requested)
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


def fast_legged_goal(planner,requested):
    """Batch the immutable legged_control lookahead, including its settling gait.

    Root command and swing command are distinct at center. Period is blended
    at EACH future frame; a fixed-period legacy shortcut changes the policy
    input. Only interpolation/geometry are batched here, not the transition.
    """
    requested=np.clip(np.asarray(requested,float),-1.5,1.5).copy()
    requested[:2]*=min(1.,1.5/max(1e-12,np.linalg.norm(requested[:2])))
    commands=[planner.command.copy()];gaits=[planner.gait_command.copy()]
    zero=not np.any(requested)
    for _ in range(11):
        command=commands[-1].copy()
        if zero:
            command[:]=0.
            gait=gaits[-1]+np.clip(-gaits[-1],-.1,.1)
        else:
            command+=np.clip(requested-command,[-.01,-.01,-.02],[.01,.01,.02])
            gait=command.copy()
        # Match advance(): gait is copied before this final root normalization.
        command[:2]*=min(1.,1.5/max(1e-12,np.linalg.norm(command[:2])))
        commands.append(command);gaits.append(gait)
    commands=np.asarray(commands);gaits=np.asarray(gaits)
    lo=np.stack([np.clip(np.searchsorted(g,gaits[:,i],side='right')-1,0,len(g)-2)
                 for i,g in enumerate(planner.grids)],axis=1)
    fraction=np.stack([np.clip((gaits[:,i]-g[lo[:,i]])/(g[lo[:,i]+1]-g[lo[:,i]]),0,1)
                       for i,g in enumerate(planner.grids)],axis=1)
    indexes=lo[:,None,:]+CORNERS
    weights=np.prod(np.where(CORNERS,fraction[:,None,:],1-fraction[:,None,:]),axis=2)
    ix,iy,iz=indexes.transpose(2,0,1)
    periods=np.sum(weights*planner.period_table[ix,iy,iz],axis=1)
    phases=[planner.phase];headings=[planner.heading];positions=[planner.position.copy()]
    for k in range(1,12):
        c,s=np.cos(headings[-1]),np.sin(headings[-1])
        pos=positions[-1].copy()
        pos[:2]+=np.array([[c,-s],[s,c]])@commands[k,:2]*.02
        positions.append(pos);headings.append(headings[-1]+commands[k,2]*.02)
        phases.append((phases[-1]+.02/periods[k])%1)
    positions=np.asarray(positions);headings=np.asarray(headings)
    q=batched_joints(planner,gaits,np.asarray(phases))
    angles=planner.yaw0+headings[1:11];c,s=np.cos(angles),np.sin(angles)
    rotations=np.zeros((10,3,3));rotations[:,0,0]=c;rotations[:,0,1]=-s
    rotations[:,1,0]=s;rotations[:,1,1]=c;rotations[:,2,2]=1
    R=planner.rotation();world_v=(positions[2:]-positions[:-2])/.04
    angular=np.zeros((10,3));angular[:,2]=(headings[2:]-headings[:-2])/.04
    return np.concatenate(((positions[1:11]-positions[0])@R,
        (R.T@rotations)[:,:,:2].reshape(10,6),
        (rotations.transpose(0,2,1)@world_v[:,:,None])[:,:,0],angular,
        q[1:11],(q[2:]-q[:-2])/.04*.1),axis=1).reshape(-1)
