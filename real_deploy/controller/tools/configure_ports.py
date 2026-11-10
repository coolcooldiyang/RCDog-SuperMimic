"""Prepare local configuration from examples; never open a serial device."""
import argparse
from pathlib import Path
import shutil
import yaml

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--motor-prefix',help='by-id path before -if00-port0')
    p.add_argument('--imu',help='IMU stable by-id path')
    p.add_argument('--force',action='store_true',help='overwrite existing local configuration')
    a=p.parse_args()
    links=Path('/dev/serial/by-id')
    if not a.motor_prefix:
        candidates=list(links.glob('usb-FTDI_*-if00-port0'))
        candidates=[f for f in candidates if all(Path(str(f).replace('-if00-',f'-if{i:02d}-')).exists() for i in range(4))]
        if len(candidates)!=1:p.error('不能唯一识别四路FTDI；使用 --motor-prefix 指定设备前缀')
        a.motor_prefix=str(candidates[0]).removesuffix('-if00-port0')
    if not a.imu:
        candidates=list(links.glob('usb-Silicon_Labs_CP2102N_*-if00-port0'))
        if len(candidates)!=1:p.error('不能唯一识别IMU；使用 --imu 指定稳定路径')
        a.imu=str(candidates[0])
    target=ROOT/'config';target.mkdir(exist_ok=True)
    names=['config.json','deploy_newurdf.yaml','leg_direction_mapping.yaml','stand_poses.yaml','policy_stand.yaml']
    existing=[str(target/n) for n in names if (target/n).exists()]
    if existing and not a.force:p.error('已有本机配置，未覆盖；需要覆盖时加 --force：'+', '.join(existing))
    for name in names:shutil.copy2(ROOT/'examples'/name,target/name)
    f=target/'leg_direction_mapping.yaml';mapping=yaml.safe_load(f.read_text())
    interface={'FL':0,'FR':1,'RL':2,'RR':3}
    for leg in mapping['legs']:leg['port']=a.motor_prefix+f'-if{interface[leg["code"]]:02d}-port0'
    f.write_text(yaml.safe_dump(mapping,allow_unicode=True,sort_keys=False))
    f=target/'deploy_newurdf.yaml';cfg=yaml.safe_load(f.read_text());cfg['imu']['device']=a.imu
    f.write_text(yaml.safe_dump(cfg,allow_unicode=True,sort_keys=False))
    (ROOT/'reports').mkdir(exist_ok=True)
    print('本机配置已写入 config/；没有打开端口、读零或使能。')
    print('方向、传动矩阵与站姿来自本项目RCDog实例；另一台机器需重新标定。')


if __name__=='__main__':main()
