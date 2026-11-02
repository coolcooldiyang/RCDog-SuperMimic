"""Sequential training, physical acceptance and recorded-contract MuJoCo checks."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from framework_check import find_evaluation_json

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
PLAN = json.loads((ROOT/'plan.json').read_text())
ENV = dict(os.environ, PYTHONPATH=str(REPO/'src'), OMP_NUM_THREADS='4', MUJOCO_GL='egl',
           PYTHONUNBUFFERED='1', TORCHINDUCTOR_CACHE_DIR=str(REPO/'torchinductor_cache'),
           TRITON_CACHE_DIR=str(REPO/'triton_cache'))
STATUS = {'status': 'training', 'stages': {}, 'hardware_ready': False}
DEADLINE = time.monotonic()+10800

def write_status():
    tmp = ROOT/'pipeline_status.tmp'
    tmp.write_text(json.dumps(STATUS, indent=2)+'\n')
    tmp.replace(ROOT/'pipeline_status.json')

def stop_training(process, run_name):
    if process.poll() is not None: return
    cmd = Path(f'/proc/{process.pid}/cmdline').read_bytes()
    if b'unisupermimic.scripts.train_supermimic' not in cmd or ('training.log_dir=runs/'+run_name).encode() not in cmd:
        raise RuntimeError('Refusing to stop an unrecognized process')
    os.killpg(process.pid, signal.SIGTERM)
    try: process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=10)

def run_evaluation(stage, checkpoint, output, trials=1):
    command = [sys.executable, str(ROOT/'evaluate_locomotion.py'), '--stage', stage,
               '--ckpt', str(checkpoint), '--output', str(output), '--trials', str(trials)]
    if trials>1: command += ['--noise', '0.01']
    p = subprocess.run(command, cwd=REPO, env=ENV, text=True, capture_output=True, timeout=360)
    output.with_suffix('.log').write_text(p.stdout+p.stderr)
    if p.returncode: raise RuntimeError('Evaluation failed: '+str(output.with_suffix('.log')))
    return json.loads(output.read_text())

def train_stage(stage, definition, initial):
    run_name = stage+'_v1'
    run = REPO/'runs'/run_name
    report_dir = REPO/'evaluations'/stage
    report_dir.mkdir(parents=True, exist_ok=True)
    logs = REPO/'logs'; logs.mkdir(exist_ok=True)
    if (run/'run_config.json').exists(): raise RuntimeError('Run already exists: '+str(run))
    command = [sys.executable, '-m', 'unisupermimic.scripts.train_supermimic',
               'task=rcdog_super_mimic/mjwarp', 'algo.num_envs=1024',
               'algo.iterations='+str(definition['iterations']),
               'algo.wm_hidden=[512,512,512]', 'algo.policy_hidden=[512,256,128]',
               'algo.wm_batch=1024', 'algo.pi_batch=1024', 'algo.wm_batches=10',
               'algo.wm_only_iters=250', 'algo.ckpt_every=500', 'algo.log_every=25',
               '+training.init_checkpoint='+str(initial),
               'env.commands.motion.motion_files=['+','.join(definition['clips'])+']',
               'training.log_dir=runs/'+run_name]
    with (logs/(stage+'.log')).open('w') as log:
        process = subprocess.Popen(command, cwd=REPO, env=ENV, stdout=log, stderr=subprocess.STDOUT,
                                   stdin=subprocess.DEVNULL, start_new_session=True)
    STATUS['stages'][stage] = {'status': 'training', 'pid': process.pid, 'run_name': run_name, 'initial_checkpoint':str(initial)}
    write_status()
    print(json.dumps({'event': 'training_started', 'stage': stage, 'pid': process.pid}), flush=True)
    seen = set()
    try:
        while time.monotonic()<DEADLINE:
            for ckpt in sorted(run.glob('ckpt_*.pt')):
                if ckpt.name in seen: continue
                seen.add(ckpt.name)
                output = report_dir/(ckpt.stem+'.json')
                result = run_evaluation(stage, ckpt, output)
                print(json.dumps({'event': 'checkpoint_evaluated', 'stage': stage, 'checkpoint': ckpt.name,
                                  'passed': result['passed'], 'total': result['total'],
                                  'metrics': [{k:r[k] for k in ['clip','measured_twist','displacement_xy_yaw','pass','drift_ok','norm_return']} for r in result['reports']]}), flush=True)
                if not result['all_passed']: continue
                trials_file = report_dir/(ckpt.stem+'_trials.json')
                perturbed = run_evaluation(stage, ckpt, trials_file, trials=definition['perturbation_trials'])
                if not perturbed['all_passed']:
                    print(json.dumps({'event':'perturbations_failed','stage':stage,'passed':perturbed['passed'],'total':perturbed['total']}),flush=True)
                    continue
                stop_training(process, run_name)
                dest = REPO/'results'/stage; dest.mkdir(parents=True, exist_ok=True)
                for source in [ckpt, run/'run_config.json', output, trials_file]:
                    shutil.copy2(source, dest/source.name)
                for r in result['reports']:
                    trajectory = REPO/r['rollout_file']
                    shutil.copy2(trajectory, dest/trajectory.name)
                    shutil.copy2(trajectory.with_suffix('.json'), dest/trajectory.with_suffix('.json').name)
                STATUS['stages'][stage] = {'status': 'accepted_in_cloud_mujoco', 'checkpoint': str(dest/ckpt.name),
                                           'reports': result['reports'], 'perturbations_passed': perturbed['passed'],
                                           'perturbations_total': perturbed['total'], 'training_stopped': True}
                write_status()
                print(json.dumps({'event':'stage_accepted','stage':stage,'checkpoint':ckpt.name}),flush=True)
                return dest/ckpt.name
            if process.poll() is not None: raise RuntimeError(stage+' exited without acceptance')
            time.sleep(20)
        raise TimeoutError('Training pipeline time limit')
    finally:
        stop_training(process, run_name)

try:
    initial=REPO/'initial_policy/strafe025.pt'
    for stage, definition in PLAN['stages'].items():
        initial=train_stage(stage, definition, initial)
    STATUS['status'] = 'framework_mujoco_sim2sim'
    write_status()
    for stage, info in STATUS['stages'].items():
        ckpt = Path(info['checkpoint'])
        command = [sys.executable, '-m', 'unisupermimic.tasks.motion_tracking.rcdog.supermimic.evaluate',
                   '--ckpt', str(ckpt), '--backend', 'mujoco']
        p = subprocess.run(command, cwd=REPO, env=ENV, text=True, capture_output=True, timeout=180)
        (ckpt.parent/'framework_eval.log').write_text(p.stdout+p.stderr)
        if p.returncode: raise RuntimeError(stage+' framework MuJoCo check failed')
        result = find_evaluation_json(p.stdout)
        if result['n_failed']: raise RuntimeError(stage+' framework hard termination')
        (ckpt.parent/'framework_eval.json').write_text(json.dumps(result,indent=2)+'\n')
        info['framework_mujoco'] = result
        write_status()
    STATUS['status'] = 'cloud_training_and_sim2sim_complete'
    write_status()
    print(json.dumps({'event': 'all_stages_complete'}), flush=True)
except BaseException as exc:
    STATUS['status'] = 'needs_attention'
    STATUS['error'] = repr(exc)
    write_status()
    raise
