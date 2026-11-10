"""Numerical interfaces for the exported RCDog SuperMimic actor.

No device is opened on import. URDF signs must be verified separately from
the legacy stand/lie direction table.
"""
from pathlib import Path
from collections import deque
import math
import time
import numpy as np

LEGS=('RR','RL','FR','FL')
EXTRA=np.array([1.,1.,2.])
GEAR=6.33

def finite(value, shape=None):
    a=np.asarray(value,dtype=float)
    if (shape is not None and a.shape!=shape) or not np.isfinite(a).all():
        raise ValueError('数值非有限或形状不符')
    return a

def joint_gains(value):
    """Expand scalar / hip-thigh-calf / policy-order 12-axis gains."""
    a=finite(value)
    if a.ndim==0:return np.full(12,float(a))
    if a.shape==(3,):return np.tile(a,4)
    if a.shape==(12,):return a.copy()
    raise ValueError('PD增益必须是标量、每腿3项或策略顺序12项')


def validate_deployment_gains(contract,kp,kd,override=None):
    """Keep the trained contract immutable; require an explicit experimental override."""
    kp,kd=joint_gains(kp),joint_gains(kd)
    if np.any(kp<=0) or np.any(kd<0):raise ValueError('无效策略 PD')
    if np.allclose(contract['kp_joint'],kp) and np.allclose(contract['kd_joint'],kd):
        return False
    if not isinstance(override,dict) or not override.get('reason'):
        raise ValueError('SuperMimic训练与部署PD不符，缺少明确试验配置')
    for key,expected in [('trained_kp_joint',contract['kp_joint']),('trained_kd_joint',contract['kd_joint']),
                         ('kp_joint',kp),('kd_joint',kd)]:
        if key not in override or not np.allclose(joint_gains(override[key]),joint_gains(expected)):
            raise ValueError('PD试验配置与训练/运行参数不符：'+key)
    return True


def operator_walk_authorized(cfg, contract, clip):
    """Explicit experiment authorization, scoped to one policy, direction and PD."""
    a=cfg.get('experimental_walk_authorization')
    if not isinstance(a,dict) or a.get('enabled') is not True:return False
    if not a.get('reason') or clip!='forward' or a.get('clip')!=clip:return False
    if a.get('checkpoint_sha256')!=contract.get('checkpoint_sha256'):return False
    try:
        return all(np.array_equal(joint_gains(a[key]),joint_gains(cfg['policy_'+key]))
                   for key in ('kp_joint','kd_joint'))
    except (ValueError,TypeError,KeyError):return False

class Transmission:
    def __init__(self, A, theta_cal, q_cal):
        self.A=finite(A,(3,3));self.theta_cal=finite(theta_cal,(3,))
        self.q_cal=finite(q_cal,(3,))
        if abs(np.linalg.det(self.A))<1e-8:raise ValueError('传动矩阵不可逆')
        self.inverse=np.linalg.inv(self.A)

    def feedback(self, rotor_q, rotor_dq):
        return (self.q_cal+self.A@(finite(rotor_q,(3,))/GEAR-self.theta_cal),
                self.A@finite(rotor_dq,(3,))/GEAR)

    def target_rotor(self,q):
        return GEAR*(self.theta_cal+self.inverse@(finite(q,(3,))-self.q_cal))

    def torque_output(self,tau_joint):
        return self.A.T@finite(tau_joint,(3,))

class JointPD:
    """Per-leg host PD. Torque is evaluated at the motor polling rate."""
    def __init__(self,transmission,q_des,kp,kd,effort,motor_limit=23.7):
        self.transmission=transmission
        self.motor_limit=float(motor_limit)
        self.update(q_des,kp,kd,effort)
        self.blend_from=None;self.blend_started=0.;self.blend_seconds=.5

    def update(self,q_des,kp,kd,effort):
        self.q_des=finite(q_des,(3,)).copy()
        if np.any(np.abs(self.q_des)>1.55):raise ValueError('目标 URDF 关节角越限')
        self.kp=finite(kp,(3,)).copy();self.kd=finite(kd,(3,)).copy()
        self.effort=finite(effort,(3,)).copy()
        if np.any(self.kp<0) or np.any(self.kd<0) or np.any(self.effort<=0):
            raise ValueError('无效 PD 参数或力矩范围')
        self.updated_at=time.monotonic()

    def output(self,rotor_q,rotor_dq,now=None):
        now=time.monotonic() if now is None else now
        if now-self.updated_at>.06:raise RuntimeError('策略目标超过60毫秒未更新')
        q,dq=self.transmission.feedback(rotor_q,rotor_dq)
        if np.any(np.abs(q)>1.57):raise RuntimeError('实际 URDF 关节角越限')
        if np.any(np.abs(dq)>np.array([30.1,30.1,15.05])):
            raise RuntimeError('实际 URDF 关节速度越限')
        tau=np.clip(self.kp*(self.q_des-q)-self.kd*dq,-self.effort,self.effort)
        out=self.transmission.torque_output(tau)
        if self.blend_from is not None:
            alpha=np.clip((now-self.blend_started)/self.blend_seconds,0,1)
            out=(1-alpha)*self.blend_from+alpha*out
        return np.clip(out,-self.motor_limit,self.motor_limit)

class NativePD(JointPD):
    """Map diagonal joint PD to the motor's local feedback loop.

The SDK has no total-torque-limit field. Gains are scaled when the torque
estimated at the latest feedback would exceed a limit; measured torque is
also guarded. This is not a hard current limit between received samples.
"""
    def __init__(self,transmission,q_des,kp,kd,effort,motor_limit=23.7,
                 speed_limits=(8.,8.,6.)):
        super().__init__(transmission,q_des,kp,kd,effort,motor_limit)
        A=self.transmission.A
        if not np.allclose(A,np.diag(np.diag(A))):
            raise ValueError('电机内部 PD 仅支持独立轴传动')
        self.speed_limits=finite(speed_limits,(3,))
        if np.any(self.speed_limits<=0):raise ValueError('无效调试速度限制')
        self.entry=None

    def update(self,q_des,kp,kd,effort):
        super().update(q_des,kp,kd,effort)
        # Targets/gains change at 50 Hz; do not rebuild them for every one
        # of the 2400 motor transactions per second under the bus lock.
        a=np.diag(self.transmission.A)
        self.native_target=self.transmission.target_rotor(self.q_des)
        self.native_kp=self.kp*a*a/(GEAR*GEAR)
        self.native_kd=self.kd*a*a/(GEAR*GEAR)
        self.native_limits=np.minimum(self.motor_limit,self.effort*np.abs(a))

    def start_from_native(self,rotor_target,kp_rotor,kd_rotor,started=None,seconds=.5):
        self.entry=(finite(rotor_target,(3,)).copy(),finite(kp_rotor,(3,)).copy(),
                    finite(kd_rotor,(3,)).copy())
        self.blend_started=time.monotonic() if started is None else started
        self.blend_seconds=float(seconds)
        if self.blend_seconds<=0:raise ValueError('进入时间必须为正')

    def motor_command(self,rotor_q,rotor_dq,tau_output=None,now=None):
        now=time.monotonic() if now is None else now
        if now-self.updated_at>.06:raise RuntimeError('策略目标超过60毫秒未更新')
        # The native drive accepts diagonal transmissions only. Scalar checks
        # avoid repeated tiny NumPy kernels/GIL handoffs in four bus threads.
        # Keep every position, speed, measured-torque and finite-value guard.
        qr=finite(rotor_q,(3,));dqr=finite(rotor_dq,(3,))
        measured=None if tau_output is None else finite(tau_output,(3,))
        tr=self.transmission;limits=self.native_limits
        for i in range(3):
            a=tr.A[i,i]
            q=tr.q_cal[i]+a*(qr[i]/GEAR-tr.theta_cal[i])
            dq=a*dqr[i]/GEAR
            if abs(q)>1.57:raise RuntimeError('实际 URDF 关节角越限')
            if abs(dq)>(15.05 if i==2 else 30.1):raise RuntimeError('实际 URDF 关节速度越限')
            if abs(dq)>self.speed_limits[i]:raise RuntimeError('调试关节速度越限，停止策略')
            if measured is not None and abs(measured[i])>limits[i]+.25:
                raise RuntimeError('实测电机输出轴力矩越限')
        target,kp,kd=self.active_reference(now)
        predicted=(kp*(target-qr)-kd*dqr)*GEAR
        scale=np.minimum(1.,limits/np.maximum(np.abs(predicted),1e-12))
        kp=kp*scale;kd=kd*scale
        return dict(q=target,kp=kp,kd=kd,dq=np.zeros(3),tau=np.zeros(3),
                    estimated_output_torque=(kp*(target-qr)-kd*dqr)*GEAR,
                    limit_scale=scale)

    def active_reference(self,now=None):
        now=time.monotonic() if now is None else now
        target=self.native_target.copy()
        kp=self.native_kp.copy();kd=self.native_kd.copy()
        if self.entry is not None:
            u=float(np.clip((now-self.blend_started)/self.blend_seconds,0,1))
            u=.5-.5*math.cos(math.pi*u)
            old_target,old_kp,old_kd=self.entry
            target=(1-u)*old_target+u*target
            kp=(1-u)*old_kp+u*kp;kd=(1-u)*old_kd+u*kd
        return target,kp,kd

    def freeze_reference(self,now=None):
        target,kp,kd=self.active_reference(now)
        q,_=self.transmission.feedback(target,np.zeros(3))
        scale=GEAR*GEAR/np.diag(self.transmission.A)**2
        self.update(q,kp*scale,kd*scale,self.effort)
        self.entry=None
        return q.copy(),self.kp.copy(),self.kd.copy()

def rotation_z(angle):
    c,s=math.cos(angle),math.sin(angle)
    return np.array([[c,-s,0],[s,c,0],[0,0,1.]])

def quaternion_matrix(wxyz):
    q=finite(wxyz,(4,));n=np.linalg.norm(q)
    if not .9<n<1.1:raise ValueError('IMU 四元数范数异常')
    w,x,y,z=q/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-w*z),2*(x*z+w*y)],
                     [2*(x*y+w*z),1-2*(x*x+z*z),2*(y*z-w*x)],
                     [2*(x*z-w*y),2*(y*z+w*x),1-2*(x*x+y*y)]])

class Actor:
    def __init__(self,bundle):
        import json,hashlib
        import onnxruntime as ort
        self.bundle=Path(bundle)
        self.contract=json.loads((self.bundle/'policy_contract.json').read_text())
        report=json.loads((self.bundle/'export_report.json').read_text())
        if report['checkpoint_sha256']!=self.contract['checkpoint_sha256']:
            raise ValueError('导出报告与策略接口不匹配')
        self.file_hashes=report['files_sha256']
        for name in ('actor_fp32.onnx','policy_contract.json','golden_inputs.npz'):
            if hashlib.sha256((self.bundle/name).read_bytes()).hexdigest()!=self.file_hashes[name]:
                raise ValueError('策略文件校验失败：'+name)
        self.q0=finite(self.contract['q0_rad'],(12,))
        options=ort.SessionOptions();options.intra_op_num_threads=2;options.inter_op_num_threads=1
        self.session=ort.InferenceSession(str(self.bundle/'actor_fp32.onnx'),
            sess_options=options,providers=['CPUExecutionProvider'])
        self.actor_sha256=hashlib.sha256((self.bundle/'actor_fp32.onnx').read_bytes()).hexdigest()
        with np.load(self.bundle/'golden_inputs.npz') as golden:
            actual=self.session.run(['action'],{'obs_hist':golden['obs_hist'][:1],'goal':golden['goal'][:1]})[0]
            if np.max(np.abs(actual-golden['action'][:1]))>=1e-4:raise ValueError('策略固定输入核对失败')
        self.hist=int(self.contract['obs_hist_shape'][1])
        self.scale=float(self.contract['action_scale'])
        if self.contract['obs_hist_shape']!=[1,self.hist,48] or self.hist not in (5,10):
            raise ValueError('SuperMimic历史输入接口不符')
        self.history=deque(maxlen=self.hist);self.previous=np.zeros(12)

    def reset(self):self.history.clear();self.previous=np.zeros(12)

    def infer(self,obs,goal):
        obs=finite(obs,(48,)).astype(np.float32)
        if not self.history:self.history.extend([obs.copy() for _ in range(self.hist)])
        else:self.history.append(obs)
        inputs={'obs_hist':np.asarray(self.history,np.float32)[None],
                'goal':finite(goal,(390,)).astype(np.float32)[None]}
        action=finite(self.session.run(['action'],inputs)[0][0],(12,))
        self.previous=action.copy()
        return action,np.clip(self.q0+self.scale*action,-1.55,1.55)

    def tick(self,q,dq,R,gyro,reference,frame):
        R=finite(R,(3,3))
        obs=np.concatenate((finite(q,(12,))-self.q0,finite(dq,(12,)),
            finite(gyro,(3,)),-R[2,:],self.previous,
            (R.T@reference['R_ref'][frame])[:,:2].reshape(6)))
        return self.infer(obs,reference['goal'][frame])

    def reference(self,name):
        if name not in ('forward','forward_left','left','back_left','back',
                        'back_right','right','forward_right','ccw','cw'):
            raise ValueError('未知参考片段')
        p=self.bundle/'references'/('omni050_'+name+'.npz')
        import hashlib
        if hashlib.sha256(p.read_bytes()).hexdigest()!=self.file_hashes['references/'+p.name]:
            raise ValueError('参考片段校验失败')
        with np.load(p) as z:data={k:z[k].copy() for k in z.files}
        if float(data['dt'])!=.02 or data['goal'].shape[1:]!=(390,):
            raise ValueError('参考时间步或维度不符')
        return data
