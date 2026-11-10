#!/usr/bin/env python3
"""Set the configured robot's four-port FTDI latency timer; no motor commands."""
import argparse,os
from pathlib import Path
SERIAL=None
def discover(base=Path('/sys/bus/usb-serial/devices'), serial_filter=None):
 found=[]
 for dev in base.glob('ttyUSB*'):
  path=dev.resolve();serial=None
  for ancestor in (path,*path.parents):
   if (ancestor/'serial').exists():serial=(ancestor/'serial').read_text().strip();break
  if (serial_filter is None or serial==serial_filter) and (dev/'latency_timer').exists():found.append(dev/'latency_timer')
 if serial_filter is not None and len(found)!=4:raise RuntimeError(f'Expected exactly four ports for {serial_filter}, found{len(found)}; no changes')
 return sorted(found)
def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--ms',type=int,choices=[1,16],default=1);p.add_argument('--check',action='store_true');p.add_argument('--serial',help='FTDI adapter serial number');a=p.parse_args()
 serial_filter=a.serial
 if not serial_filter:
  import yaml
  cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'config/leg_direction_mapping.yaml').read_text())
  expected={Path(leg['port']).resolve().name for leg in cfg['legs']}
  paths=[path for path in discover() if path.parent.name in expected]
  if len(paths)!=4:raise RuntimeError('Configured four motor ports do not match sysfs; no changes')
 else:paths=discover(serial_filter=serial_filter)
 old={path:path.read_text().strip() for path in paths}
 if a.check:
  for path,value in old.items():print(f'{path.parent.name}: {value} ms')
  return
 if os.geteuid()!=0:raise SystemExit('Requires sudo to write Linux USB driver settings; no changes made.')
 changed=[]
 try:
  for path in paths:
   path.write_text(str(a.ms)+'\n');changed.append(path)
   if path.read_text().strip()!=str(a.ms):raise RuntimeError('Readback mismatch: '+str(path))
 except Exception:
  for path in changed:path.write_text(old[path]+'\n')
  raise
 for path in paths:print(f'{path.parent.name}: {old[path]} -> {path.read_text().strip()} ms')
 print('No motor command was sent. Replug/reboot may reset the driver value; use --check to verify.')
if __name__=='__main__':main()
