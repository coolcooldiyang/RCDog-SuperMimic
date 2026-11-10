"""Linux joystick reader, adapted from this robot's deploy_inputs.py.
Axis conventions follow dm_dog_controller/gamepad_command_node.py.
Only opens /dev/input; no actuator or IMU interface.
"""
import array, fcntl, os, select, struct, math
from pathlib import Path

def axis(axes,index,deadzone=.12):
    v=max(-1.,min(1.,float(axes[index]))) if index<len(axes) else 0.
    return 0. if abs(v)<=deadzone else math.copysign((abs(v)-deadzone)/(1-deadzone),v)
def button(buttons,index):return index<len(buttons) and bool(buttons[index])

class Joystick:
    def __init__(self,path):
        self.path=str(path);self.fd=None;self.axes=[0.]*16;self.buttons=[0]*32
        self.armed=False;self.error=None
        self.axis_lookup={};self.button_lookup={};self.device_info={}

    def open(self):
        if not Path(self.path).exists():
            candidates=[p for p in Path('/dev/input/by-id').glob('usb-shanwan*joystick')
                        if '-event-' not in p.name]
            if len(candidates)!=1:raise RuntimeError('无法唯一识别 shanwan 手柄')
            self.path=str(candidates[0])
        self.fd=os.open(self.path,os.O_RDONLY|os.O_NONBLOCK)
        amap=array.array('B',[0]*64);bmap=array.array('H',[0]*512)
        count_a=array.array('B',[0]);count_b=array.array('B',[0])
        fcntl.ioctl(self.fd,0x80016a11,count_a);fcntl.ioctl(self.fd,0x80016a12,count_b)
        fcntl.ioctl(self.fd,0x80406a32,amap);fcntl.ioctl(self.fd,0x84006a34,bmap)
        ac=amap.tolist()[:count_a[0]];bc=bmap.tolist()[:count_b[0]]
        # Android mode exposes Z/RZ right stick and additional C/Z buttons;
        # Xbox mode uses RX/RY. Normalize to the donor ROS Joy indices.
        yaw_code=3 if 3 in ac else 2
        self.axis_lookup={ac.index(code):canonical for code,canonical in [(0,0),(1,1),(yaw_code,3)]}
        self.button_lookup={bc.index(code):canonical for canonical,code in
            enumerate([304,305,307,308,310,311]) if code in bc}
        if len(self.button_lookup)!=6:raise RuntimeError('缺少 A/B/X/Y/LB/RB 按键映射')
        self.device_info={'path':self.path,'axis_codes':ac,'button_codes':bc,
                          'raw_axis_to_canonical':self.axis_lookup,
                          'raw_button_to_canonical':self.button_lookup}

    def poll(self):
        if self.fd is None:self.open()
        initial=True;changed=False
        previous=self.buttons.copy()
        while select.select([self.fd],[],[],0)[0]:
            data=os.read(self.fd,8)
            if len(data)!=8:raise RuntimeError('手柄设备断开')
            _,value,kind,index=struct.unpack('<IhBB',data)
            init=bool(kind&0x80);kind&=0x7f
            if kind==1 and index in self.button_lookup:
                self.buttons[self.button_lookup[index]]=int(value!=0)
            if kind==2 and index in self.axis_lookup:
                self.axes[self.axis_lookup[index]]=-max(-1.,value/32767.)
            changed|=not init;initial&=init
        if not self.armed:
            if not any(self.buttons) and max(abs(axis(self.axes,i)) for i in (0,1,3))<.2:self.armed=True
            return None
        if not changed or initial:return None
        b=self.buttons
        if button(b,4) and button(b,5) and button(b,0):return 'emergency-stop'
        if button(b,1) and not button(previous,1):return 'lie'
        if button(b,4) and button(b,0) and not button(b,5):
            return 'posture' if not (button(previous,4) and button(previous,0)) else None
        if button(b,0) and not button(previous,0):return 'policy-stop'
        if button(b,3) and not button(previous,3) and button(b,5):return 'play'
        if button(previous,5) and not button(b,5):return 'policy-stop'
        return None

    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None
