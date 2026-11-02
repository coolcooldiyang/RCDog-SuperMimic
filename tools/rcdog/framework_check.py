"""Resume only the framework MuJoCo checks after training has completed."""
import json
import os
from pathlib import Path
import subprocess
import sys

def find_evaluation_json(text):
    decoder = json.JSONDecoder()
    offset = 0
    found = None
    for line in text.splitlines(keepends=True):
        start = offset+len(line)-len(line.lstrip())
        if line.lstrip().startswith('{'):
            try:
                candidate, _ = decoder.raw_decode(text[start:])
                if isinstance(candidate, dict) and {'n_failed', 'norm_return'} <= candidate.keys():
                    found = candidate
            except json.JSONDecodeError:
                pass
        offset += len(line)
    if found is None:
        raise ValueError('No complete evaluation JSON object in framework stdout')
    return found

if __name__ == '__main__':
    root = Path(__file__).resolve().parent
    repo = root.parents[1]
    status = json.loads((root/'pipeline_status.json').read_text())
    env = dict(os.environ, PYTHONPATH=str(repo/'src'), OMP_NUM_THREADS='4', MUJOCO_GL='egl')
    for stage, info in status['stages'].items():
        if not info.get('training_stopped'):
            raise RuntimeError('Training not yet accepted: '+stage)
        ckpt = Path(info['checkpoint'])
        command = [sys.executable, '-m', 'unisupermimic.tasks.motion_tracking.rcdog.supermimic.evaluate',
                   '--ckpt', str(ckpt), '--backend', 'mujoco']
        p = subprocess.run(command, cwd=repo, env=env, capture_output=True, text=True, timeout=180)
        (ckpt.parent/'framework_eval.log').write_text(p.stdout+p.stderr)
        if p.returncode: raise RuntimeError(stage+' framework evaluation failed')
        result = find_evaluation_json(p.stdout)
        if result['n_failed']: raise RuntimeError(stage+' terminated in framework MuJoCo')
        (ckpt.parent/'framework_eval.json').write_text(json.dumps(result, indent=2)+'\n')
        info['framework_mujoco'] = result
        print(stage, result, flush=True)
    status['status'] = 'cloud_training_and_sim2sim_complete'
    status.pop('error', None)
    (root/'pipeline_status.json').write_text(json.dumps(status, indent=2)+'\n')
