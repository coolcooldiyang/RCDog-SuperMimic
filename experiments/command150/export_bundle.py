"""Offline-only policy export and reference-table generation. No device access."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import shutil

import numpy as np
import torch

HERE=Path(__file__).resolve().parent
EXPERIMENT=HERE
sys.path.insert(0,str(EXPERIMENT.parents[1]/'src'))
from unisupermimic.rl.algos.supermimic import networks
from unisupermimic.rl.algos.supermimic.geometry import quat_to_mat
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.kinematics import RCDogKinematics
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.motion import MotionLib
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.mdp import MDP

@torch.inference_mode()
def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',type=Path,default=EXPERIMENT)
    p.add_argument('--checkpoint',type=Path,default=HERE.parents[1]/'results/command150/ckpt_000500.pt')
    p.add_argument('--output',type=Path,default=HERE.parents[1]/'outputs/command150_export')
    p.add_argument('--torchscript-only',action='store_true')
    a=p.parse_args();source=a.source.resolve();repo=HERE.parents[1];ckpt=a.checkpoint.resolve();out=a.output.resolve()
    if (out/'export_report.json').exists():raise FileExistsError('Export already exists; choose another --output: '+str(out))
    out.mkdir(parents=True,exist_ok=True);(out/'references').mkdir(exist_ok=True)
    shutil.copy2(source/'command_gait_table.npz',out/'command_gait_table.npz')
    shutil.copy2(source/'command_reference.py',out/'command_reference.py')
    torch.set_num_threads(2)
    saved=torch.load(ckpt,map_location='cpu',weights_only=True);args=saved['args'];cfg=json.loads((ckpt.parent/'run_config.json').read_text())
    files=[repo/f for f in cfg['config']['env']['commands']['motion']['motion_files']]
    audit=json.loads((source/'model_audit.json').read_text());q0=torch.tensor(audit['standing_qpos'][7:],dtype=torch.float32)
    kin=RCDogKinematics(str(repo/'src/unisupermimic/assets/robots/rcdog/supermimic_motor.xml'),'cpu')
    motion=MotionLib([str(f) for f in files],kin,'cpu',src_fps=50.)
    mdp=MDP(kin,motion,q0);networks.AMP_DTYPE=None
    actor=networks.Policy(48,390,hist=int(args['hist']),z_dim=int(args['z_dim']),num_joints=12,hidden=tuple(args['policy_hidden'])).eval()
    assert int(args['hist'])==5 and int(args['goal_frames'])==10
    actor.load_state_dict(saved['pi'])
    all_frames=torch.arange(motion.num_frames)
    goals=mdp.goal(all_frames,10).numpy()
    for ci,name in enumerate(motion.names):
        start=int(motion.clip_start[ci]);end=int(motion.clip_end[ci])+1
        np.savez_compressed(out/'references'/(name+'.npz'),goal=goals[start:end],R_ref=motion.root_R[start:end].numpy(),q_ref=motion.q[start:end].numpy(),qd_ref=motion.qd[start:end].numpy(),root_p_ref=motion.root_p[start:end].numpy(),q0=q0.numpy(),dt=np.array(.02))
    # Build export checks from the recorded physical trajectories where present.
    summary=source/'local_sim2sim/omni150.json'
    recordings={}
    if summary.exists():
        for record in json.loads(summary.read_text())['reports']:
            recordings[record['clip']]=np.load(repo/record['rollout_file'])
    history_samples=[];goal_samples=[]
    for ci,name in enumerate(motion.names):
        start=int(motion.clip_start[ci]);count=int(motion.clip_len[ci]);buffer=None
        chosen={min(i,count-2) for i in [0,1,49,50,100,125,200,300,400,449]}
        data=recordings.get(name)
        for i in range(max(chosen)+1):
            frame=torch.tensor([start+i])
            if data is not None:
                qp=data['qpos'][i];qv=data['qvel'][i]
                R=quat_to_mat(torch.tensor(qp[3:7],dtype=torch.float32)[None])
                state=(torch.tensor(qp[:3],dtype=torch.float32)[None],R,torch.tensor(qv[:3],dtype=torch.float32)[None],(R@torch.tensor(qv[3:6],dtype=torch.float32).reshape(1,3,1)).squeeze(-1),torch.tensor(qp[7:],dtype=torch.float32)[None],torch.tensor(qv[6:],dtype=torch.float32)[None])
                prev=torch.zeros(1,12) if i==0 else torch.tensor(data['actions'][i-1],dtype=torch.float32)[None]
            else:
                state=mdp.ref_state(frame);prev=torch.zeros(1,12)
            context=torch.zeros(1,16);context[:,0]=1.
            obs=mdp.obs(state,prev,context,frame)
            buffer=obs[:,None].repeat(1,5,1) if buffer is None else torch.cat((buffer[:,1:],obs[:,None]),1)
            if i in chosen:
                history_samples.append(buffer.clone());goal_samples.append(mdp.goal(frame,10))
    obs=torch.cat(history_samples);goal=torch.cat(goal_samples);expected=actor(obs,goal)
    traced=torch.jit.trace(actor,(obs[:1],goal[:1]));traced.save(str(out/'actor_fp32.ts'))
    jit_error=float((traced(obs,goal)-expected).abs().max());assert jit_error<1e-5,jit_error
    np.savez_compressed(out/'golden_inputs.npz',obs_hist=obs.numpy(),goal=goal.numpy(),action=expected.numpy())
    report={'checkpoint':str(ckpt),'checkpoint_sha256':hashlib.sha256(ckpt.read_bytes()).hexdigest(),'clips':motion.names,'input_shapes':{'obs_hist':[1,5,48],'goal':[1,390]},'output_shape':[1,12],'verification_samples':len(obs),'continuous_command_reference':'command_gait_table.npz','framework':'Uni-SuperMimic','source_urdf_sha256':audit['source_urdf_sha256'],'mass_kg':audit['mass_kg'],'torchscript_max_abs_error':jit_error,'hardware_ready':False}
    if not a.torchscript_only:
        import onnx
        import onnxruntime as ort
        torch.onnx.export(actor,(obs[:1],goal[:1]),str(out/'actor_fp32.onnx'),opset_version=17,dynamo=False,input_names=['obs_hist','goal'],output_names=['action'],dynamic_axes={'obs_hist':{0:'batch'},'goal':{0:'batch'},'action':{0:'batch'}})
        onnx.checker.check_model(onnx.load(str(out/'actor_fp32.onnx')))
        options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
        runtime=ort.InferenceSession(str(out/'actor_fp32.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
        actual=runtime.run(['action'],{'obs_hist':obs.numpy(),'goal':goal.numpy()})[0]
        error=float(np.abs(actual-expected.numpy()).max());assert error<1e-4,error
        report.update(onnx_max_abs_error=error,onnx_opset=17,onnxruntime=ort.__version__)
    contract=json.loads((HERE/'policy_contract.json').read_text());contract.update(obs_hist_shape=[1,5,48],action_scale=.25,kp_joint=[20.]*12,kd_joint=[.5]*12,simulation_period_s=.005,control_decimation=4,framework='Uni-SuperMimic',checkpoint=str(ckpt),checkpoint_sha256=report['checkpoint_sha256'],q0_rad=q0.tolist(),q0_deg=np.rad2deg(q0.numpy()).tolist(),clips=motion.names)
    (out/'policy_contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n')
    report['files_sha256']={str(f.relative_to(out)):hashlib.sha256(f.read_bytes()).hexdigest() for f in out.rglob('*') if f.is_file()}
    (out/'export_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
