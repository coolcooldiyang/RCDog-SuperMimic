"""Offline-only policy export and reference-table generation. No device access."""
import argparse
import os
import hashlib
import json
from pathlib import Path
import sys
import shutil

import numpy as np
import torch
os.environ.update(RCDOG_REFERENCE_BEZIER='1',RCDOG_REFERENCE_FAMILY='legged_control_trot',RCDOG_FLAT_GAIT='1',RCDOG_POSITION_RATE_LIMIT='1')

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
    p.add_argument('--checkpoint',type=Path,default=HERE.parents[1]/'results/flat_legged_a3/a3/ckpt_001000.pt')
    p.add_argument('--output',type=Path,default=HERE.parents[1]/'outputs/flat_legged_a3_export')
    p.add_argument('--torchscript-only',action='store_true')
    a=p.parse_args();source=a.source.resolve();repo=HERE.parents[1];ckpt=a.checkpoint.resolve();out=a.output.resolve()
    if (out/'export_report.json').exists():raise FileExistsError('Export already exists; choose another --output: '+str(out))
    out.mkdir(parents=True,exist_ok=True);(out/'references').mkdir(exist_ok=True)
    shutil.copy2(source/'reference_candidates/legged_control/command_gait_table.npz',out/'command_gait_table.npz')
    shutil.copy2(source/'legged_gait.py',out/'legged_gait.py')
    shutil.copy2(source/'command_sim.py',out/'command_sim.py')
    # Keep the offline ONNX runner self-contained, including relative mesh paths.
    model_rel=Path('src/unisupermimic/assets/robots/rcdog')
    model_out=out/model_rel;model_out.mkdir(parents=True,exist_ok=True)
    for name in ('scene_supermimic_motor.xml','supermimic_motor.xml','model_manifest.json'):
        shutil.copy2(repo/model_rel/name,model_out/name)
    shutil.copytree(repo/model_rel/'meshes',model_out/'meshes',dirs_exist_ok=True)
    shutil.copy2(source/'reference_candidates/legged_control/source_provenance.json',out/'reference_provenance.json')
    shutil.copytree(source/'reference_candidates/legged_control/upstream',out/'upstream_attribution',dirs_exist_ok=True)
    shutil.copy2(source/'command_reference.py',out/'command_reference.py')
    torch.set_num_threads(2)
    saved=torch.load(ckpt,map_location='cpu',weights_only=True);args=saved['args'];cfg=json.loads((ckpt.parent/'run_config.json').read_text())
    files=[repo/f for f in cfg['config']['env']['commands']['motion']['motion_files']]
    contract_source=json.loads((source/'policy_contract.json').read_text())
    audit={'source_urdf_sha256':contract_source['model_source_urdf_sha256'],'mass_kg':contract_source['mass_kg']}
    q0=torch.tensor(np.load(source/'reference_candidates/legged_control/command_gait_table.npz')['q0'][7:],dtype=torch.float32)
    kin=RCDogKinematics(str(repo/'src/unisupermimic/assets/robots/rcdog/supermimic_motor.xml'),'cpu')
    motion=MotionLib([str(f) for f in files],kin,'cpu',src_fps=50.)
    mdp=MDP(kin,motion,q0,coupled_limits=False);networks.AMP_DTYPE=None
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
    contract=json.loads((source/'policy_contract.json').read_text());contract.update(obs_hist_shape=[1,5,48],action_scale=.25,kp_joint=[20.]*12,kd_joint=[.5]*12,simulation_period_s=.001,control_decimation=20,position_rate_limit=True,target_velocity_limits_rad_s=[30.1,30.1,15.05]*4,framework='Uni-SuperMimic',checkpoint=str(ckpt),checkpoint_sha256=report['checkpoint_sha256'],q0_rad=q0.tolist(),q0_deg=np.rad2deg(q0.numpy()).tolist(),clips=motion.names)
    command_record=ckpt.parent/'command.json'
    train_env=json.loads(command_record.read_text()).get('environment',{}) if command_record.exists() else {}
    feedback_max_ms=float(train_env.get('RCDOG_FEEDBACK_DELAY_MS','0'))
    contract['training_feedback_delay_ms']=[0.,feedback_max_ms]
    contract['training_feedback_delay_model']='Adjacent20ms physical-state interpolation; CPU audit uses exact1ms samples'
    contract['inference_add_artificial_feedback_delay']=False
    contract['bootstrap_reference']='references/flat_lc_stand0.npz'
    contract['reference_table']='command_gait_table.npz'
    contract['reference_provenance']='reference_provenance.json'
    contract['model_file']=str(model_rel/'scene_supermimic_motor.xml')
    contract['default_offline_rate_limit']=True
    for field in contract['obs_fields']:
        if field['slice']==[30,42]:field['value']='previous applied action after angle and position-target rate limits'
    # Simulate the exported actor with identical reference/limit processing.
    runner=(out/'command_sim.py').read_text().replace("MODEL=ROOT.parents[1]/'src","MODEL=ROOT/'src").replace("str(ROOT/'reference_candidates/legged_control/command_gait_table.npz')","str(ROOT/'command_gait_table.npz')").replace("os.environ.get('RCDOG_POSITION_RATE_LIMIT','0')", "os.environ.get('RCDOG_POSITION_RATE_LIMIT','1')")
    (out/'command_sim.py').write_text(runner)
    (out/'policy_contract.json').write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n')
    report['files_sha256']={str(f.relative_to(out)):hashlib.sha256(f.read_bytes()).hexdigest() for f in out.rglob('*') if f.is_file()}
    (out/'export_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
