"""Reopen configured by-id devices only after output is disabled.

Changing ttyUSB numbers is harmless to by-id names, but an already-open file
descriptor remains dead after unplugging. Never scan IDs or remap legs by
enumeration order, and never resume motion/clear a fault on reconnection.
"""
import json
import time
import uuid
from pathlib import Path
from dog_motion.deploy_inputs import HI12,QueuedJoystick,OmniStickGate


def device_identity(path):
    p=Path(path)
    try:
        s=p.stat()
        return (str(p.resolve()),s.st_rdev,s.st_ino,s.st_ctime_ns)
    except OSError:return None


class USBReconnect:
    def reset_stick_gate(self):
        # Both initial startup and USB recovery must use the same behavior.
        # Neutral-before-entry is retained; center never leaves PLAY.
        self.stick_gate=OmniStickGate(stop_on_neutral=False,start_threshold=0.)

    def setup_usb_reconnect(self,root):
        self.usb_root=Path(root);self.usb_signature=None;self.usb_pending=False
        self.usb_next_check=0.;self.usb_generation=0
        self.usb_status={'state':'starting','generation':0}

    def serial_devices(self):
        return [(leg['code'],leg['port']) for leg in self.ctrl.legs]+[('IMU',self.cfg['imu']['device'])]

    def serial_signature(self):
        return tuple(device_identity(path) for _,path in self.serial_devices())

    def describe_usb(self,state,message):
        self.usb_status=dict(state=state,message=message,generation=self.usb_generation,
            devices=[dict(name=name,by_id=str(path),device=str(Path(path).resolve()) if Path(path).exists() else None)
                     for name,path in self.serial_devices()])

    def connect_hardware(self):
        # Keep IPC alive if launched before the USB adapter is attached.
        self.usb_signature=self.serial_signature()
        if None in self.usb_signature:
            self.usb_pending=True
            self.describe_usb('waiting_devices','等待配置中的四路485和IMU；后台保持运行，插入后自动重连')
            return 0
        try:
            online=self.ctrl.connect()
            self.usb_pending=(None in self.usb_signature or any(b is None for b in self.ctrl.buses)
                              or bool(self.imu_error))
            self.describe_usb('waiting_devices' if self.usb_pending else 'connected','按设备序列号连接；未使能')
            return online
        except Exception as exc:
            self.usb_pending=True;self.describe_usb('waiting_devices',str(exc));return 0

    def invalidate_usb_reference(self):
        self.initialized=False;self.policy_ready=False;self.maps={}
        self.session_pd_verified=False;self.walk_unlocked=False
        self.servo_check_result=None;self.servo_check_abort_pending=False
        self.pending_lie=False;self.decelerating=False;self.R_WN=None
        self.reset_stick_gate()

    def usb_output_active(self):
        # Controller.is_enabled means ALL axes enabled. A single live axis
        # must also prevent reopening its transport.
        return self.ctrl.is_enabled or self.ctrl.is_moving or any(m.get('enabled') for m in self.snaps())

    def close_usb_transports(self):
        # Do not use controller.close(), which intentionally suppresses stop
        # errors. Reopening must wait for every old polling thread to exit.
        for bus in self.ctrl.buses:
            if bus is not None:
                bus.stop()
                if bus._thread is not None and bus._thread.is_alive():
                    raise RuntimeError('旧485线程未退出，禁止打开第二个连接')
                bus._port=None  # release the SDK's descriptor after thread exit
        self.ctrl.buses=[]
        self.imu.close()
        if hasattr(self.imu,'thread') and self.imu.thread.is_alive():
            raise RuntimeError('旧IMU线程未退出，禁止重新连接')
        if self.pad:
            self.pad.close()
            if self.pad.thread.is_alive():raise RuntimeError('旧手柄线程未退出，禁止重新连接')

    def usb_reconnect_step(self,force=False):
        if not hasattr(self,'usb_signature'):return False  # simulation harness
        now=time.monotonic()
        if not force and now<self.usb_next_check:return False
        self.usb_next_check=now+1.
        signature=self.serial_signature()
        if signature==self.usb_signature and not self.usb_pending:return False
        if self.usb_output_active():
            self.latch_fault('USB设备连接发生变化；停止出力，重连后需重新读零',self.snaps())
            self.mode='FAULT';self.close_log('usb_disconnected_during_motion')
            return True  # no reconnect until a later disabled iteration
        self.invalidate_usb_reference()
        self.usb_pending=True
        self.close_log('usb_reconnect')
        if None in signature:
            self.describe_usb('waiting_devices','等待配置中的四路485和IMU重新出现；不会自动使能')
            return True
        try:
            # Keep a fault's old feedback evidence before discarding buses.
            if self.post_fault_captures:self.capture_fault_aftermath(self.snaps(),finish=True)
            self.close_usb_transports()
            online=self.ctrl.connect()
            if any(b is None for b in self.ctrl.buses):raise RuntimeError('; '.join(self.ctrl.errors))
            self.imu=HI12(self.cfg['imu']['device'],self.usb_root/self.cfg['imu']['parser'])
            self.imu.start();self.imu_error=None
            if self.pad:self.pad=QueuedJoystick(self.cfg['gamepad']['device'])
            self.gamepad_error=None
            self.ctrl.motion_error=None;self.ctrl.motion_fault_detail=None
            self.mode='FAULT' if self.fault else 'IDLE'
            self.usb_signature=signature;self.usb_pending=False
            self.usb_generation+=1;self.session_id=uuid.uuid4().hex
            self.describe_usb('connected',f'USB已自动重连，电机在线{online}/12；未读零、未使能，请重新执行init-lie')
            print(json.dumps(dict(event='usb_reconnected',**self.usb_status),ensure_ascii=False),flush=True)
        except Exception as exc:
            self.describe_usb('retrying',str(exc))
        return True
