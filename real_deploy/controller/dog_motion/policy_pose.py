"""Resolve a measured policy-q0 posture into legacy motor-output coordinates."""
import math
from pathlib import Path
import yaml

def has_confirmed_default_stand(pose_path):
    d=yaml.safe_load(Path(pose_path).read_text())
    return (d.get('default_stand_confirmed') is True and
            d.get('coordinate_domain')=='signed_motor_output_deg_relative_lie' and
            len(d.get('stand',[]))==12 and
            all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<=180
                for v in d['stand']))

def resolve_policy_target(spec_path, reference_path, topology, zeros, sim):
    if not Path(reference_path).exists():
        raise ValueError('策略站姿尚未标定：先 record-policy-stand')
    spec=yaml.safe_load(Path(spec_path).read_text())
    d=yaml.safe_load(Path(reference_path).read_text())
    if d.get('simulated') is not sim or d.get('topology')!=topology:
        raise ValueError('站姿标定的硬件拓扑或仿真模式不匹配')
    if (d.get('policy_sha256')!=spec['checkpoint_sha256'] or
            d.get('q0_urdf_rad')!=spec['q0_urdf_rad']):
        raise ValueError('策略或 q0 已变化，需要重新标定站姿')
    previous=d.get('lie_zero_output_rad',[])
    absolute=d.get('stand_output_rad',[])
    if (len(previous)!=12 or len(absolute)!=12 or len(zeros)!=12 or
            not all(math.isfinite(v) for v in previous+absolute+zeros)):
        raise ValueError('站姿标定长度或数值无效')
    if max(abs(a-b) for a,b in zip(previous,zeros))>math.radians(3):
        raise ValueError('本次趴下零位与标定偏差超过 3°，检查初始姿态和编码器零点')
    directions=[v for l in topology for v in l['directions']]
    return [s*math.degrees(t-z) for s,t,z in zip(directions,absolute,zeros)]
