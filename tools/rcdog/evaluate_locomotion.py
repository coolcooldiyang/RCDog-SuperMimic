"""Run selected policies in independent CPU MuJoCo, without training runtime."""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
import torch
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.kinematics import RCDogKinematics
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.motion import MotionLib
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.mdp import MDP, MAX_STEP_REWARD
from unisupermimic.tasks.motion_tracking.rcdog.supermimic.robot import DEFAULT_JOINT_POS, KP, KD, EFFORT, VELOCITY_LIMIT
from unisupermimic.rl.algos.supermimic import networks
from unisupermimic.rl.algos.supermimic.geometry import quat_to_mat

ROOT = Path(__file__).resolve().parent

@torch.inference_mode()
def evaluate(ckpt, stage, output, trials=1, noise=0.):
    torch.set_num_threads(4)
    files = json.loads((ROOT/'plan.json').read_text())['stages'][stage]['clips']
    asset = Path('src/unisupermimic/assets/robots/rcdog')
    kin = RCDogKinematics(str(asset/'supermimic_motor.xml'), 'cpu')
    motion = MotionLib(files, kin, 'cpu', src_fps=50.)
    mdp = MDP(kin, motion, torch.tensor(DEFAULT_JOINT_POS))
    checkpoint = torch.load(ckpt, map_location='cpu', weights_only=True)
    args = checkpoint['args']
    networks.AMP_DTYPE = None  # CPU policy inference is fp32, as in existing CPU verification.
    policy = networks.Policy(mdp.obs_dim, mdp.goal_frame_dim*int(args['goal_frames']),
                             hist=int(args['hist']), z_dim=int(args['z_dim']), num_joints=12,
                             hidden=tuple(args['policy_hidden']))
    policy.load_state_dict(checkpoint['pi'])
    policy.eval()
    output.parent.mkdir(parents=True, exist_ok=True)
    reports = []
    for ci, filename in enumerate(files):
        reference = json.loads(Path(filename).with_suffix('.json').read_text())
        for trial in range(trials):
            report = rollout(policy, args, mdp, motion, ci, reference, asset, seed=101+trial, noise=noise)
            path = output.with_name(output.stem+'_'+reference['name']+f'_seed{101+trial}.json')
            trajectory = report.pop('_trajectory')
            np.savez(path.with_suffix('.npz'), **trajectory)
            report.update(clip=reference['name'], checkpoint=str(ckpt), iteration=checkpoint['iteration'],
                          checkpoint_sha256=hashlib.sha256(ckpt.read_bytes()).hexdigest(),
                          reference_file=filename, rollout_file=str(path.with_suffix('.npz')),
                          mujoco_version=mujoco.__version__, torch_version=torch.__version__, hardware_ready=False)
            path.write_text(json.dumps(report, indent=2)+'\n')
            reports.append(report)
    summary = {'stage': stage, 'passed': sum(r['pass'] for r in reports), 'total': len(reports),
               'all_passed': all(r['pass'] for r in reports), 'reports': reports}
    output.write_text(json.dumps(summary, indent=2)+'\n')
    return summary

def rollout(policy, args, mdp, motion, ci, reference, asset, seed, noise):
    model = mujoco.MjModel.from_xml_path(str(asset/'scene_supermimic_motor.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    model.geom_friction[:, 0] = 1.
    rng = np.random.default_rng(seed)
    data.qpos[7:] += rng.uniform(-noise, noise, 12)
    data.qpos[:2] += rng.uniform(-noise/2, noise/2, 2)
    data.qvel[:3] += rng.uniform(-3*noise, 3*noise, 3)
    data.qvel[3:6] += rng.uniform(-5*noise, 5*noise, 3)
    mujoco.mj_forward(model, data)
    default, kp, kd, effort = map(np.array, (DEFAULT_JOINT_POS, KP, KD, EFFORT))
    feet_ids = [model.body(x+'_foot').id for x in ['rr', 'rl', 'fr', 'fl']]
    foot_geoms = [model.geom(x+'_foot_collision').id for x in ['rr', 'rl', 'fr', 'fl']]
    floor = model.geom('floor').id
    xi = torch.zeros(1, 16); xi[:, 0] = 1.
    previous = torch.zeros(1, 12); history = None
    qpos, qvel, feet = [data.qpos.copy()], [data.qvel.copy()], [data.xpos[feet_ids].copy()]
    contacts, torques, actions, rewards, bad_contacts = [], [], [], [], []
    peak_qd = np.abs(data.qvel[6:]).copy()
    hard = False
    def state():
        R = quat_to_mat(torch.tensor(data.qpos[3:7], dtype=torch.float32).unsqueeze(0))
        return (torch.tensor(data.qpos[:3], dtype=torch.float32)[None], R,
                torch.tensor(data.qvel[:3], dtype=torch.float32)[None],
                (R @ torch.tensor(data.qvel[3:6], dtype=torch.float32).reshape(1, 3, 1)).squeeze(-1),
                torch.tensor(data.qpos[7:], dtype=torch.float32)[None],
                torch.tensor(data.qvel[6:], dtype=torch.float32)[None])
    start = int(motion.clip_start[ci]); count = int(motion.clip_len[ci])
    for step in range(count-1):
        f = torch.tensor([start+step])
        obs = mdp.obs(state(), previous, xi, f)
        history = obs[:, None].repeat(1, int(args['hist']), 1) if history is None else torch.cat((history[:, 1:], obs[:, None]), 1)
        action = policy(history, mdp.goal(f, int(args['goal_frames']))).float()
        target = np.clip(default+.35*action[0].numpy(), -1.55, 1.55)
        peak = np.zeros(12)
        for _ in range(5):
            data.ctrl[:] = np.clip(kp*(target-data.qpos[7:])-kd*data.qvel[6:], -effort, effort)
            peak = np.maximum(peak, np.abs(data.ctrl))
            mujoco.mj_step(model, data)
            peak_qd = np.maximum(peak_qd, np.abs(data.qvel[6:]))
        mujoco.mj_forward(model, data)
        f1 = f+1
        rewards.append(float(mdp.reward(state(), f1, action, previous)))
        current = np.zeros(4, bool); bad = False
        for contact in data.contact:
            pair = {int(contact.geom1), int(contact.geom2)}
            if floor in pair:
                other = next((g for g in pair if g != floor), floor)
                if other in foot_geoms: current[foot_geoms.index(other)] = True
                else: bad = True
        qpos.append(data.qpos.copy()); qvel.append(data.qvel.copy()); feet.append(data.xpos[feet_ids].copy())
        contacts.append(current); bad_contacts.append(bad); torques.append(peak); actions.append(action[0].numpy())
        previous = action
        if bool(mdp.termination(state(), f1)) or not np.isfinite(data.qpos).all():
            hard = True
            break
    qpos, qvel, feet, contacts, torques = map(np.array, (qpos, qvel, feet, contacts, torques))
    times = np.arange(1, len(qpos))*.02
    interval=reference.get('steady_interval_seconds',[2.,7.2])
    steady = (times >= interval[0]) & (times <= interval[1])
    quat = qpos[:, 3:7]; w, x, y, z = quat.T
    yaw = np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))
    actual = contacts[steady]
    expected = (((times[steady, None]-1)/reference['period_seconds'] + np.array([0, .5, .5, 0])) % 1) < reference['duty_factor']
    alignment = float(np.mean(np.column_stack((actual[:, 0] == actual[:, 3], actual[:, 1] == actual[:, 2])))) if len(actual) else 0.
    phase_accuracy = float((actual == expected).mean()) if len(actual) else 0.
    air = (~actual).mean(0) if len(actual) else np.zeros(4)
    clearance = np.quantile(feet[1:][steady, :, 2]-.02, .95, axis=0) if steady.any() else np.zeros(4)
    lifts = ((actual[:-1]) & (~actual[1:])).sum(0) if len(actual)>1 else np.zeros(4)
    measured = np.array([np.polyfit(times[steady], values[steady], 1)[0] for values in [qpos[1:, 0], qpos[1:, 1], yaw[1:]]]) if steady.sum()>1 else np.zeros(3)
    displacement = np.r_[qpos[-1, :2]-qpos[0, :2], yaw[-1]-yaw[0]]
    target = np.array(reference['target_twist_world']); desired = np.array(reference['reference_displacement'])
    active = np.abs(target)>1e-6
    fraction=.10
    speed_ok = np.all(np.abs(measured[active]-target[active]) < fraction*np.abs(target[active]))
    displacement_ok = np.all(np.abs(displacement[active]-desired[active]) < fraction*np.abs(desired[active]))
    if active[2]:
        drift_ok = np.linalg.norm(displacement[:2]) < .2 and np.max(np.linalg.norm(qpos[:, :2]-qpos[0, :2], axis=1)) < .25 and np.linalg.norm(measured[:2]) < .08
    else:
        heading=target[:2]/np.linalg.norm(target[:2])
        cross_track=abs(heading[0]*displacement[1]-heading[1]*displacement[0])
        drift_ok = cross_track < .25 and abs(displacement[2]) < .25
    completed = len(qpos) == count and not hard
    passed = bool(completed and speed_ok and displacement_ok and drift_ok and np.all(air>.12) and np.all(clearance>.015) and alignment>.72 and phase_accuracy>.65 and np.all(lifts>=4) and not any(bad_contacts) and np.all(peak_qd <= np.array(VELOCITY_LIMIT)) and np.abs(qpos[:, 7:]).max()<1.59)
    return {'pass': passed, 'completed': completed, 'hard_termination': hard, 'seed': seed, 'noise_rad': noise,
            'frames': len(qpos), 'target_twist': target.tolist(), 'measured_twist': measured.tolist(),
            'displacement_xy_yaw': displacement.tolist(), 'reference_displacement_xy_yaw': desired.tolist(),
            'speed_ok': bool(speed_ok), 'displacement_ok': bool(displacement_ok), 'drift_ok': bool(drift_ok), 'acceptance_fraction':fraction,
            'norm_return': float(sum(rewards)/(len(rewards)*MAX_STEP_REWARD)),
            'diagonal_contact_agreement': alignment, 'stance_phase_accuracy': phase_accuracy,
            'air_fraction': air.tolist(), 'p95_clearance_m': clearance.tolist(), 'lift_events': lifts.tolist(),
            'nonfoot_ground_frames': sum(bad_contacts), 'max_joint_speed_rad_s': peak_qd.tolist(),
            'max_joint_torque_Nm': torques.max(0).tolist(),
            '_trajectory': {'qpos': qpos, 'qvel': qvel, 'feet': feet, 'contacts': contacts, 'peak_torque': torques, 'actions': actions}}

if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--ckpt', type=Path, required=True); p.add_argument('--stage', required=True)
    p.add_argument('--output', type=Path, required=True); p.add_argument('--trials', type=int, default=1)
    p.add_argument('--noise', type=float, default=0.)
    a = p.parse_args()
    summary = evaluate(a.ckpt, a.stage, a.output, a.trials, a.noise)
    print(json.dumps({'stage': a.stage, 'passed': summary['passed'], 'total': summary['total'],
                      'results': [{k:r[k] for k in ['clip','pass','measured_twist','displacement_xy_yaw','speed_ok','displacement_ok','drift_ok','norm_return']} for r in summary['reports']]}), flush=True)
