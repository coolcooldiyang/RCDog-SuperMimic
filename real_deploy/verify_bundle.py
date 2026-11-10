"""Offline export parity and local inference timing; never opens a device."""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
import onnxruntime as ort
p=argparse.ArgumentParser();p.add_argument('--bundle',type=Path,required=True);p.add_argument('--repeat',type=int,default=500);a=p.parse_args()
report=json.loads((a.bundle/'export_report.json').read_text())
for relative,expected in report.get('files_sha256',{}).items():
    file=(a.bundle/relative).resolve()
    if not file.is_relative_to(a.bundle.resolve()):raise ValueError('Invalid bundle path: '+relative)
    actual=hashlib.sha256(file.read_bytes()).hexdigest()
    if actual!=expected:raise ValueError('Bundle hash mismatch: '+relative)
d=np.load(a.bundle/'golden_inputs.npz');options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
s=ort.InferenceSession(str(a.bundle/'actor_fp32.onnx'),sess_options=options,providers=['CPUExecutionProvider'])
actual=s.run(['action'],{'obs_hist':d['obs_hist'],'goal':d['goal']})[0];error=float(np.abs(actual-d['action']).max());assert error<1e-4,error
inputs={'obs_hist':d['obs_hist'][:1],'goal':d['goal'][:1]}
for _ in range(20):s.run(['action'],inputs)
times=[]
for _ in range(a.repeat):
 t=time.perf_counter();s.run(['action'],inputs);times.append((time.perf_counter()-t)*1000)
print(json.dumps({'export_parity_max_abs_error':error,'samples':len(actual),'repeats':a.repeat,'inference_ms_p50_p95_p99':np.quantile(times,[.5,.95,.99]).tolist(),'hardware_control_tested':False},indent=2))
