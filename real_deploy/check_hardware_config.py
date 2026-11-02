"""Offline consistency checks only. Does not open serial ports or enable motors."""
import argparse
from pathlib import Path
import sys
import numpy as np
import yaml
p=argparse.ArgumentParser(description=__doc__);p.add_argument('config',type=Path);p.add_argument('--check-device-paths',action='store_true');a=p.parse_args()
c=yaml.safe_load(a.config.read_text());errors=[];ports=[]
def require(condition,message):
 if not condition:errors.append(message)
def finite(value):
 return isinstance(value,(int,float)) and not isinstance(value,bool) and np.isfinite(value)
require(c.get('physical_front_confirmed') is True,'physical_front_confirmed: 尚未确认机头与腿名')
require(bool(c.get('controller_platform')),'controller_platform: 尚未确认主控')
act=c.get('actuator',{});require(bool(act.get('sdk_motor_type')),'actuator.sdk_motor_type: 尚未确认具体型号')
require(act.get('sdk_feedback_domain') in ['rotor','output'],'sdk_feedback_domain: 必须核对 rotor/output')
if act.get('sdk_feedback_domain')=='rotor':require(finite(act.get('sdk_internal_gear_ratio')) and act['sdk_internal_gear_ratio']>0,'sdk_internal_gear_ratio: 需要实测/SDK查询值')
require(bool(act.get('bus_type')) and finite(act.get('baud')) and act['baud']>0,'bus_type/baud: 尚未确认通信参数')
for leg in ['rr','rl','fr','fl']:
 bus=c.get('buses',{}).get(leg,{})
 require(bus.get('leg_identity_verified') is True,leg+': 端口所属腿未现场核对')
 require(bus.get('transmission_verified') is True,leg+': 同步带传动关系尚未验证')
 require(bus.get('transmission_model') in ['diagonal','matrix'],leg+': transmission_model 需标为 diagonal 或 matrix')
 device=bus.get('device');require(isinstance(device,str) and device.startswith('/dev/'),leg+': 缺少端口路径')
 if isinstance(device,str):
  ports.append(str(Path(device).resolve()))
  if a.check_device_paths:require(Path(device).exists(),leg+': 端口路径不存在')
 for expected,joint in enumerate(['hip','thigh','calf'],1):
  m=bus.get('motors',{}).get(joint,{});name=leg+'_'+joint
  require(m.get('id')==expected,name+': ID 应为 '+str(expected))
  require(m.get('extra_ratio')==(2 if joint=='calf' else 1),name+': 额外减速比错误')
  require(m.get('sign') in [-1,1],name+': sign 尚未标定')
  require(finite(m.get('theta_output_cal_rad')) and finite(m.get('q_cal_rad')),name+': 缺少同一已知姿态下的两侧角度')
  bounds=m.get('physical_joint_limits_rad');valid=isinstance(bounds,list) and len(bounds)==2 and all(finite(x) for x in bounds)
  require(valid,name+': 缺少实测关节角范围')
  if valid:
   require(-1.57<=bounds[0]<bounds[1]<=1.57,name+': 使用实测范围与训练范围的交集')
   if finite(m.get('q_cal_rad')):require(bounds[0]<=m['q_cal_rad']<=bounds[1],name+': 标定姿态不在允许范围内')
 try:
  A=np.array(bus.get('A_joint_from_output'),dtype=float)
  valid_A=A.shape==(3,3) and np.isfinite(A).all() and abs(np.linalg.det(A))>1e-6
  require(valid_A,leg+': A_joint_from_output 需为标定过的非奇异 3x3 矩阵')
  if valid_A and bus.get('transmission_model')=='diagonal':
   signs=[bus.get('motors',{}).get(j,{}).get('sign') for j in ['hip','thigh','calf']]
   if all(x in [-1,1] for x in signs):require(np.allclose(A,np.diag(np.array(signs)/[1,1,2]),atol=1e-6),leg+': 对角传动矩阵与 sign / ratio 不一致')
 except (TypeError,ValueError):errors.append(leg+': 传动矩阵无效')
require(len(ports)==len(set(ports)),'不同腿不能映射到同一个串口设备')
imu=c.get('imu',{});require(bool(imu.get('device')),'imu.device: 需要显式绑定，不能自动挑第一个 CP210x')
if imu.get('device'):
 require(str(Path(imu['device']).resolve()) not in ports,'IMU 与电机使用了同一设备')
 if a.check_device_paths:require(Path(imu['device']).exists(),'IMU 端口不存在')
require(imu.get('quat_direction_verified') is True,'IMU 四元数方向尚未做安装姿态验证')
try:
 R=np.array(imu.get('R_CI'),dtype=float)
 require(R.shape==(3,3) and np.isfinite(R).all() and np.allclose(R.T@R,np.eye(3),atol=1e-5) and abs(np.linalg.det(R)-1)<1e-5,'imu.R_CI: 需要正交且 det=+1 的实测安装旋转')
except (TypeError,ValueError):errors.append('imu.R_CI: 安装旋转无效')
require(c.get('calibration_verified') is True,'calibration_verified: 方向、零位与小角度测试尚未全部确认')
if errors:
 print('未通过离线配置检查：\n'+'\n'.join('- '+e for e in errors));sys.exit(2)
print('离线配置字段检查通过；不代表实机控制、通信频率或机械安全已经验证。')
