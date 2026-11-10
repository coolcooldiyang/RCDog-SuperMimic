"""HI12 reader and standalone port of the user's gamepad command helpers."""
import importlib.util
import array
from collections import deque
import fcntl
import math
import os
import queue
from pathlib import Path
import select
import struct
import threading
import time

def axis(axes,index,deadzone=.12):
    v=max(-1.,min(1.,float(axes[index]))) if 0<=index<len(axes) else 0.
    return 0. if abs(v)<=deadzone else math.copysign((abs(v)-deadzone)/(1-deadzone),v)

def button(buttons,index):return 0<=index<len(buttons) and bool(buttons[index])

def direction(axes):
    # Same ROS Joy indices and signs as dm_dog_controller/gamepad_command_node.py.
    x,y,w=axis(axes,1),axis(axes,0),axis(axes,3)
    if abs(w)>max(abs(x),abs(y),.2):return 'ccw' if w>0 else 'cw'
    if math.hypot(x,y)<.2:return 'forward'
    names=('forward','forward_left','left','back_left','back','back_right','right','forward_right')
    return names[int(math.floor((math.atan2(y,x)+math.pi/8)/(math.pi/4)))%8]


class ForwardStickGate:
    """Neutral-before-start, stop on release; no automatic restart after a stop."""
    def __init__(self):
        self.neutral_seen=False

    def step(self, axes, buttons, ready, active, event=None):
        x,y,w=axis(axes,1),axis(axes,0),axis(axes,3)
        neutral=max(abs(x),abs(y),abs(w))<.12
        forward=x>.25 and abs(y)<.2 and abs(w)<.2
        if event is not None:
            self.neutral_seen=False
            return None
        if active:
            if not (x>.12 and abs(y)<.25 and abs(w)<.25):
                self.neutral_seen=False
                return 'policy-stop'
            return None
        if not ready or any(buttons):
            self.neutral_seen=False
            return None
        if neutral:self.neutral_seen=True
        if forward and self.neutral_seen:
            self.neutral_seen=False
            return 'play'
        return None

def omni_command(axes, speed=.5, yaw_rate=.5):
    """Body frame: forward/left/CCW positive; diagonal speed stays bounded."""
    x,y,w=axis(axes,1),axis(axes,0),axis(axes,3)
    norm=max(1.,math.hypot(x,y))
    return [speed*x/norm,speed*y/norm,yaw_rate*w]


class OmniStickGate:
    """Require neutral before entry; optionally keep the policy active at center."""
    def __init__(self,stop_on_neutral=True,start_threshold=.25):
        self.neutral_seen=False
        self.stop_on_neutral=stop_on_neutral
        self.start_threshold=start_threshold
    def step(self,axes,buttons,ready,active,event=None):
        magnitude=max(abs(axis(axes,i)) for i in (0,1,3))
        if event is not None:
            self.neutral_seen=False
            return None
        if active:
            if magnitude==0 and self.stop_on_neutral:
                self.neutral_seen=False
                return 'policy-stop'
            return None
        if not ready or any(buttons):
            self.neutral_seen=False
            return None
        if magnitude==0:self.neutral_seen=True
        if magnitude>self.start_threshold and self.neutral_seen:
            self.neutral_seen=False
            return 'play'
        return None


class Joystick:
    def __init__(self,path):
        self.path=str(path);self.fd=None;self.axes=[0.]*16;self.buttons=[0]*32
        self.armed=False;self.error=None
        self.axis_lookup={};self.button_lookup={};self.device_info={}

    def open(self):
        if not Path(self.path).exists():
            candidates=[p for p in Path('/dev/input/by-id').glob('usb-shanwan*joystick')
                        if '-event-' not in p.name]
            if not candidates:raise RuntimeError('未找到 shanwan 手柄接收器')
            if len(candidates)>1:raise RuntimeError('发现多个 shanwan 手柄，请只连接一个')
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


class QueuedJoystick:
    """Read at 200 Hz; preserve brief edges while the control thread is busy."""
    def __init__(self,path):
        self.axes=[0.]*16;self.buttons=[0]*32;self.device_info={}
        self.events=queue.Queue();self.lock=threading.Lock();self.finished=threading.Event()
        self.pending=deque()
        self.latest=(self.axes.copy(),self.buttons.copy());self.error=None
        self.sample_time=0.;self.input_armed=False
        self.raw=Joystick(path)
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()

    def loop(self):
        retry_at=0.
        while not self.finished.is_set():
            try:
                if self.raw.fd is None and time.monotonic()<retry_at:
                    self.finished.wait(.01);continue
                event=self.raw.poll()
                with self.lock:
                    self.latest=(self.raw.axes.copy(),self.raw.buttons.copy())
                    self.device_info=dict(self.raw.device_info);self.error=None
                    self.sample_time=time.monotonic();self.input_armed=self.raw.armed
                if event:self.events.put((time.monotonic(),event,*self.latest))
            except (OSError,RuntimeError) as exc:
                self.raw.close();self.raw.armed=False
                with self.lock:self.error=str(exc)
                retry_at=time.monotonic()+1.
            self.finished.wait(.005)

    def snapshot(self):
        with self.lock:
            if self.error:raise RuntimeError(self.error)
            if time.monotonic()-self.sample_time>.1:
                raise RuntimeError('手柄采集线程超过100毫秒未更新')
            return self.latest[0].copy(),self.latest[1].copy(),self.input_armed

    def poll(self):
        with self.lock:
            axes,buttons=self.latest;error=self.error
            self.axes=axes.copy();self.buttons=buttons.copy()
        if error:raise RuntimeError(error)
        now=time.monotonic();pending=list(self.pending);self.pending.clear()
        while not self.events.empty():pending.append(self.events.get_nowait())
        if any(x[1]=='emergency-stop' for x in pending):return 'emergency-stop'
        valid=[]
        for item in pending:
            stamp,event,axes,buttons=item
            if event not in ('policy-stop',) and now-stamp>.1:continue
            if event=='play' and not self.buttons[5]:continue
            valid.append(item)
        if not valid:return None
        stamp,event,axes,buttons=valid.pop(0)
        # Preserve the remaining event order, including a press then release.
        self.pending.extend(valid)
        self.axes=axes.copy()
        return event

    def close(self):
        self.finished.set();self.thread.join(timeout=2);self.raw.close()

def fresh_imu_snapshot(reader,max_age_s,wait_s=.008):
    """Wait briefly for publication; never consume an expired orientation.

    This only retries soft freshness misses. A reader error, its existing
    30 ms expiry, or the 8 ms wait budget still fails immediately/boundedly.
    During a posture check the existing fixed motor target remains in force.
    """
    deadline=time.monotonic()+wait_s
    while True:
        sample=reader.snapshot()
        age=float(sample['age_s'])
        if math.isfinite(age) and 0<=age<=max_age_s:
            return sample
        remaining=min(deadline-time.monotonic(),.03-age)
        if not math.isfinite(age) or age<0 or remaining<=0:
            raise RuntimeError(f'IMU 数据过期：{age*1000:.1f} ms，允许 {max_age_s*1000:g} ms；等待新帧未恢复')
        time.sleep(min(.0005,remaining))


class HI12:
    def __init__(self,device,parser_path):
        spec=importlib.util.spec_from_file_location('rcdog_hi12_parser',str(parser_path))
        parser=importlib.util.module_from_spec(spec);spec.loader.exec_module(parser)
        self.parser=parser;self.device=str(device)
        self.latest=None;self.error=None;self.running=False;self.lock=threading.Lock()

    def start(self):
        import serial
        self.serial=serial.Serial(self.device,115200,timeout=.01,exclusive=True)
        self.running=True
        self.thread=threading.Thread(target=self.loop,daemon=True);self.thread.start()

    def loop(self):
        parser=self.parser.FrameParser(check_crc=True);previous=None
        valid_frames=0;last_received=None;max_gap_ms=0.
        while self.running:
            try:
                # read(512) waits for512 bytes OR the10ms serial timeout,
                # even when a complete packet is already buffered. Block for
                # one byte when idle, otherwise drain available bytes now.
                chunk=self.serial.read(max(1,min(4096,self.serial.in_waiting)))
                received_at=time.monotonic()
                for frame in parser.feed(chunk):
                    sample=self.parser.ImuSample(frame)
                    with self.lock:
                        if previous is not None and sample.ts<previous:
                            self.error='IMU 时间戳回退，需重新对齐';continue
                        if previous is not None and sample.ts==previous:continue
                        device_gap=None if previous is None else sample.ts-previous
                        previous=sample.ts
                        gap_ms=0. if last_received is None else (received_at-last_received)*1000
                        max_gap_ms=max(max_gap_ms,gap_ms);last_received=received_at;valid_frames+=1
                        self.latest={'monotonic_s':received_at,'ts_ms':sample.ts,
                            'quat_wxyz':sample.quat,'gyro_rad_s':[v*math.pi/180 for v in sample.gyr],
                            'acc_g':sample.acc,
                            'bad_crc':parser.bad_crc,'valid_frames':valid_frames,
                            'last_receive_gap_ms':gap_ms,'max_receive_gap_ms':max_gap_ms,
                            'device_sample_gap_ms':device_gap}
            except Exception as exc:
                self.error=str(exc);self.running=False

    def snapshot(self):
        with self.lock:
            if self.error:raise RuntimeError(self.error)
            if self.latest is None:raise RuntimeError('尚无有效 IMU 数据')
            d=dict(self.latest)
        d['age_s']=time.monotonic()-d['monotonic_s']
        if d['age_s']>.03:raise RuntimeError('IMU 数据超过30毫秒未更新')
        return d

    def close(self):
        self.running=False
        if hasattr(self,'thread'):self.thread.join(timeout=1)
        if hasattr(self,'serial'):self.serial.close()
