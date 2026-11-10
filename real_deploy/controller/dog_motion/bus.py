#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""四腿 GO-M8010-6 总线封装（只依赖 unitree_actuator_sdk）。

安全约定
--------
- 未使能时每一帧都显式置 kp = kd = q = dq = tau = 0，电机不出力；
- 力矩前馈 tau 恒为 0，只有位置刚度 kp 和阻尼 kd；
- 使能前必须有点，没有就自动把「当前角度」设成零点，避免朝绝对 0 冲；
- kp/kd 按「关节侧」给，内部换算到转子侧：rotor = joint / r²。
"""
from __future__ import annotations

import ctypes
from collections import deque
import math
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import List, Optional

from .repo import find_repo_root

_LIB = os.path.join(find_repo_root(), "third_party", "unitree_actuator_sdk", "lib")
if _LIB not in sys.path:
    sys.path.insert(0, _LIB)
try:  # 让扩展模块能找到 libUnitreeMotorSDK_Linux64.so
    ctypes.CDLL(os.path.join(_LIB, "libUnitreeMotorSDK_Linux64.so"))
except OSError:
    pass

from unitree_actuator_sdk import (  # noqa: E402
    MotorCmd,
    MotorData,
    MotorMode,
    MotorType,
    SerialPort,
    queryGearRatio,
    queryMotorMode,
)

MOTOR_TYPE = MotorType.GO_M8010_6
MODE_FOC = queryMotorMode(MOTOR_TYPE, MotorMode.FOC)
GEAR = float(queryGearRatio(MOTOR_TYPE))  # 6.33


@dataclass
class MotorState:
    motor_id: int
    online: bool = False
    last_seen: float = 0.0
    q_rotor: float = 0.0
    dq_rotor: float = 0.0
    tau_rotor: float = 0.0      # 反馈里的实际力矩（转子侧）
    temp: int = 0
    err: int = 0
    valid_frames: int = 0
    failed_frames: int = 0
    consecutive_failures: int = 0
    last_attempt: float = 0.0
    last_poll_gap_ms: float = 0.0
    last_io_ms: float = 0.0
    max_io_ms: float = 0.0
    last_io_exception: Optional[str] = None
    last_failure_reason: Optional[str] = None
    feedback_mode: int = 0
    extra_ratio: float = 1.0

    # 控制状态
    enabled: bool = False
    zero_valid: bool = False
    zero_rotor: float = 0.0
    target_deg: float = 0.0
    direction: int = 1
    tau_cmd: float = 0.0           # 前馈力矩（关节侧 N·m）
    last_command: Optional[dict] = None


class LegBus:
    """一条腿（一个 485 通道上的 3 个电机），独立线程轮询。"""

    def __init__(self, name: str, port: str, ids: List[int], joints: List[str],
                 kp_joint: float = 2.0, kd_joint: float = 0.1, rate_hz: float = 100.0):
        self.name = name
        self.port = port
        self.ids = list(ids)
        self.joints = list(joints)
        self._kp_joint = float(kp_joint)
        self._kd_joint = float(kd_joint)
        self._rate_hz = max(20.0, min(500.0, rate_hz))

        self._lock = threading.Lock()
        self._motors = [MotorState(motor_id=i,extra_ratio=2.0 if index==2 else 1.0)
                        for index,i in enumerate(self.ids)]
        self._feedback_events = deque(maxlen=128)
        self._control_samples = deque(maxlen=4500)  # 3 motors x 250 Hz x 6 s
        self._joint_drive=None
        self.control_error=None
        self._mode = "hybrid"          # hybrid（力位混合）/ damping（只阻尼）
        self._running = False
        self._thread: Optional[threading.Thread] = None

        self._port = SerialPort(port)          # 默认 4 Mbps，GO 用
        self._cmd = MotorCmd()
        self._data = MotorData()
        self._cmd.motorType = MOTOR_TYPE
        self._data.motorType = MOTOR_TYPE
        self._cmd.mode = MODE_FOC

    # -- 生命周期 ---------------------------------------------------------
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, name="bus-%s" % self.port,
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():
                raise RuntimeError('485 线程尚未退出，禁止并发复用同一串口')
            self._thread = None
        # Polling has stopped: flush a final zero-command frame to every ID.
        self.set_enabled(False)
        for idx in range(len(self.ids)):
            self._step(idx)

    def _loop(self) -> None:
        period = 1.0 / self._rate_hz
        while self._running:
            t0 = time.perf_counter()
            for idx in range(len(self.ids)):
                self._step(idx)
            rest = period - (time.perf_counter() - t0)
            # 超时导致本轮超期时也让出调度，避免忙轮询拖住监控线程。
            time.sleep(max(0.0, rest))

    def _step(self, idx: int) -> None:
        with self._lock:
            m = self._motors[idx]
            enabled = m.enabled
            zero = m.zero_rotor if m.zero_valid else 0.0
            target = m.target_deg
            direction = m.direction
            kp_joint = self._kp_joint
            kd_joint = self._kd_joint
            tau_joint = m.tau_cmd
            mode = self._mode
            policy_torque=0.0
            native_command=None
            if enabled and mode in ('joint_pd','native_pd'):
                try:
                    now=time.monotonic()
                    if any(not a.online or a.err or now-a.last_seen>.02 for a in self._motors):
                        raise RuntimeError('关节 PD 的反馈超过20毫秒或存在电机故障')
                    if mode=='native_pd':
                        native_command=self._joint_drive.motor_command(
                            [a.q_rotor for a in self._motors],
                            [a.dq_rotor for a in self._motors],
                            [a.tau_rotor*GEAR for a in self._motors])
                    else:
                        policy_torque=float(self._joint_drive.output(
                            [a.q_rotor for a in self._motors],
                            [a.dq_rotor for a in self._motors])[idx])
                except Exception as exc:
                    self.control_error=str(exc)
                    for a in self._motors:a.enabled=False
                    enabled=False

        cmd = self._cmd
        cmd.id = m.motor_id
        if enabled and mode == 'native_pd':
            for key in ('kp','kd','q','dq','tau'):
                setattr(cmd,key,float(native_command[key][idx]))
        elif enabled and mode == 'joint_pd':
            cmd.kp=cmd.kd=cmd.q=cmd.dq=0.0
            cmd.tau=policy_torque/GEAR
        elif enabled and mode == "damping":
            # 只阻尼：kp=0、tau=0，腿可以被手慢慢摆，松手不会自由落体
            cmd.kp = 0.0
            cmd.kd = kd_joint / (GEAR * GEAR)
            cmd.q = 0.0
            cmd.dq = 0.0
            cmd.tau = 0.0
        elif enabled:
            # 力位混合：位置刚度/阻尼 + 前馈力矩（输出侧 -> 转子侧除以 r）
            cmd.kp = kp_joint / (GEAR * GEAR)
            cmd.kd = kd_joint / (GEAR * GEAR)
            cmd.q = zero + direction * math.radians(target) * GEAR
            cmd.dq = 0.0
            cmd.tau = tau_joint / GEAR
        else:
            cmd.kp = cmd.kd = cmd.q = cmd.dq = 0.0
            cmd.tau = 0.0

        self._data.correct = False
        started = time.monotonic()
        sent=dict(monotonic_s=started,mode=mode,kp=float(cmd.kp),kd=float(cmd.kd),
                  q_rotor_rad=float(cmd.q),dq_rotor_rad_s=float(cmd.dq),tau_rotor_nm=float(cmd.tau))
        io_exception = None
        try:
            received = self._port.sendRecv(cmd, self._data)
        except Exception as exc:
            received = False
            io_exception = str(exc)
        data = self._data
        now = time.monotonic()
        with self._lock:
            m.last_command=sent
            m.last_poll_gap_ms = (started-m.last_attempt)*1000 if m.last_attempt else 0.0
            m.last_attempt = started
            m.last_io_ms = (now-started)*1000
            m.max_io_ms = max(m.max_io_ms,m.last_io_ms)
            m.last_io_exception = io_exception
            if received and data.correct and int(data.motor_id) == m.motor_id:
                recovering = m.consecutive_failures
                last_q = m.q_rotor
                last_seen = m.last_seen
                m.valid_frames += 1
                m.consecutive_failures = 0
                m.online = True
                m.last_seen = now
                m.q_rotor = float(data.q)
                m.dq_rotor = float(data.dq)
                m.tau_rotor = float(data.tau)
                m.temp = int(data.temp)
                m.err = int(data.merror)
                m.feedback_mode = int(data.mode)
                if mode in ('native_pd','joint_pd') and hasattr(self,'_control_samples'):
                    self._control_samples.append(dict(monotonic_s=now,id=m.motor_id,
                        q_rotor_rad=m.q_rotor,dq_rotor_rad_s=m.dq_rotor,
                        tau_rotor_nm=m.tau_rotor,command=sent,err=m.err))
                if recovering:
                    self._feedback_events.append(dict(
                        event='feedback_recovered', monotonic_s=now, id=m.motor_id,
                        failed_polls=recovering, gap_s=now-last_seen,
                        previous_q_rotor_rad=last_q, q_rotor_rad=m.q_rotor,
                        dq_rotor_rad_s=m.dq_rotor, tau_rotor_nm=m.tau_rotor,
                        mode=m.feedback_mode, temp=m.temp, err=m.err,
                        enabled=m.enabled))
            else:
                m.failed_frames += 1
                m.consecutive_failures += 1
                # SDK 的 false 同时可能表示收帧或解包失败，不能直接称为 CRC 错误。
                m.last_failure_reason = ('sdk_exception' if io_exception else
                    'send_recv_false' if not received else
                    'invalid_feedback' if not data.correct else 'feedback_id_mismatch')
                self._feedback_events.append(dict(
                    event='feedback_failed', monotonic_s=now, id=m.motor_id,
                    reason=m.last_failure_reason, io_ms=m.last_io_ms,
                    feedback_age_s=now-m.last_seen,
                    consecutive_failures=m.consecutive_failures,
                    q_rotor_rad=m.q_rotor, enabled=m.enabled))
                if io_exception or now - m.last_seen > 0.1:
                    m.online = False

    def feedback_events(self) -> list:
        """逐帧失败和恢复首帧，供故障记录使用；不影响控制目标。"""
        with self._lock:
            return list(self._feedback_events)

    def control_samples(self) -> list:
        with self._lock:return list(self._control_samples)

    def set_joint_drive(self,drive):
        with self._lock:
            if not all(m.enabled for m in self._motors):
                raise RuntimeError('进入策略前须先起立保持')
            self._joint_drive=drive;self.control_error=None
            self._mode='native_pd' if hasattr(drive,'motor_command') else 'joint_pd'

    def update_joint_drive(self,q_des,kp,kd,effort):
        with self._lock:
            if self._mode not in ('joint_pd','native_pd'):raise RuntimeError('关节 PD 未启动')
            self._joint_drive.update(q_des,kp,kd,effort)

    def freeze_joint_reference(self):
        with self._lock:
            if self._mode!='native_pd':raise RuntimeError('电机内部 PD 未启动')
            return self._joint_drive.freeze_reference()

    # -- 控制接口 ---------------------------------------------------------
    def set_enabled(self, enabled: bool) -> int:
        """使能/松弛一条腿，返回实际切换的电机数。"""
        changed = 0
        with self._lock:
            for m in self._motors:
                if enabled:
                    if not m.zero_valid:
                        if m.online and time.monotonic() - m.last_seen < 0.1:
                            m.zero_rotor = m.q_rotor
                            m.zero_valid = True
                        else:
                            continue
                    m.enabled = True
                    changed += 1
                else:
                    m.enabled = False
                    changed += 1
        return changed

    def capture_zero(self) -> int:
        """把各电机当前角度设为关节零点，返回成功个数。"""
        count = 0
        now = time.monotonic()
        with self._lock:
            for m in self._motors:
                if m.online and now - m.last_seen < 0.1:
                    m.zero_rotor = m.q_rotor
                    m.zero_valid = True
                    count += 1
        return count

    def restore_zero_rotor(self, values) -> None:
        """仅用于同次上电的只读进程交接；不修改电机内部零点。"""
        values = list(values)
        if len(values) != len(self.ids) or not all(math.isfinite(v) for v in values):
            raise ValueError('软件零位数量或数值无效')
        with self._lock:
            if any(m.enabled for m in self._motors):
                raise RuntimeError('使能时禁止恢复零位')
            for m, zero in zip(self._motors, values):
                m.zero_rotor = float(zero)
                m.zero_valid = True
                m.target_deg = 0.0

    def set_target(self, idx: int, deg: float) -> None:
        with self._lock:
            self._motors[idx].target_deg = float(deg)

    def set_direction(self, idx: int, direction: int) -> None:
        with self._lock:
            self._motors[idx].direction = 1 if direction >= 0 else -1

    def set_tau(self, idx: int, nm: float) -> None:
        """设置关节侧前馈力矩（N·m），只有使能时才下发。"""
        with self._lock:
            self._motors[idx].tau_cmd = float(nm)

    def set_mode(self, mode: str) -> None:
        """hybrid = 力位混合；damping = 只阻尼（kp=0），便于手动摆姿态。"""
        with self._lock:
            self._mode = "damping" if mode == "damping" else "hybrid"

    @property
    def mode(self) -> str:
        return self._mode

    def set_gains(self, kp_joint: float, kd_joint: float) -> None:
        with self._lock:
            self._kp_joint = max(0.0, float(kp_joint))
            self._kd_joint = max(0.0, float(kd_joint))

    def zero_all_targets(self) -> None:
        with self._lock:
            for m in self._motors:
                m.target_deg = 0.0

    def snapshot(self) -> list:
        now = time.monotonic()
        with self._lock:
            out = []
            for m in self._motors:
                online = m.online and (now - m.last_seen < 0.1)
                q_deg = 0.0
                if m.zero_valid:
                    q_deg = math.degrees((m.q_rotor - m.zero_rotor) / GEAR) * m.direction
                enabled_now = m.enabled
                out.append({
                    "id": m.motor_id,
                    "online": online,
                    "temp": m.temp,
                    "err": m.err,
                    "enabled": m.enabled,
                    "zero_valid": m.zero_valid,
                    "target_deg": m.target_deg,
                    "direction": m.direction,
                    "q_deg": q_deg,
                    "q_rotor_rad": m.q_rotor,
                    "zero_rotor_rad": m.zero_rotor,
                    "extra_ratio": m.extra_ratio,
                    "total_ratio": GEAR*m.extra_ratio,
                    "q_reduced_relative_deg": q_deg/m.extra_ratio,
                    "target_reduced_relative_deg": m.target_deg/m.extra_ratio,
                    "dq_reduced_deg_s": math.degrees(m.dq_rotor/GEAR/m.extra_ratio)*m.direction,
                    "tau_reduced_nm": m.tau_rotor*GEAR*m.extra_ratio*m.direction,
                    "feedback_age_s": max(0.0, now - m.last_seen),
                    "valid_frames": m.valid_frames,
                    "failed_frames": m.failed_frames,
                    "consecutive_failures": m.consecutive_failures,
                    "last_poll_gap_ms": m.last_poll_gap_ms,
                    "last_io_ms": m.last_io_ms,
                    "max_io_ms": m.max_io_ms,
                    "last_io_exception": m.last_io_exception,
                    "last_failure_reason": m.last_failure_reason,
                    "feedback_mode": m.feedback_mode,
                    "dq_deg": math.degrees(m.dq_rotor / GEAR),
                    "tau_feedback_out": m.tau_rotor * GEAR,
                    "tau_out": m.tau_rotor * GEAR if enabled_now else 0.0,
                    "control_mode": self._mode,
                    "last_command": m.last_command,
                })
            return out


class SimulatedBus:
    """无硬件时的仿真总线，接口与 LegBus 一致，用于验证界面和方向逻辑。"""

    def __init__(self, name: str, port: str, ids: List[int], joints: List[str],
                 kp_joint: float = 2.0, kd_joint: float = 0.1, rate_hz: float = 100.0):
        self.name = name
        self.port = port + " (sim)"
        self.ids = list(ids)
        self.joints = list(joints)
        self._kp_joint = kp_joint
        self._kd_joint = kd_joint
        self._lock = threading.Lock()
        self._q = [0.0] * len(ids)          # 关节角度（度）
        self._target = [0.0] * len(ids)
        self._tau = [0.0] * len(ids)        # 前馈力矩（关节侧 N·m）
        self._enabled = [False] * len(ids)
        self._dir = [1] * len(ids)
        self._mode = "hybrid"
        self._zero_valid = [False] * len(ids)
        self._running = False
        self._thread = None
        self._rate_hz = rate_hz

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=1.0)

    def _loop(self) -> None:
        dt = 1.0 / self._rate_hz
        while self._running:
            with self._lock:
                for i in range(len(self.ids)):
                    if self._enabled[i] and self._mode != "damping":
                        # 与真实总线一致：指令先乘方向，读数再乘回来
                        raw_target = self._dir[i] * self._target[i]
                        self._q[i] += (raw_target - self._q[i]) * min(1.0, dt * 4.0)
            time.sleep(dt)

    def set_enabled(self, enabled: bool) -> int:
        with self._lock:
            for i in range(len(self.ids)):
                if enabled:
                    self._zero_valid[i] = True
                self._enabled[i] = enabled
            return len(self.ids)

    def capture_zero(self) -> int:
        with self._lock:
            for i in range(len(self.ids)):
                self._zero_valid[i] = True
            return len(self.ids)

    def set_target(self, idx: int, deg: float) -> None:
        with self._lock:
            self._target[idx] = float(deg)

    def set_direction(self, idx: int, direction: int) -> None:
        with self._lock:
            self._dir[idx] = 1 if direction >= 0 else -1

    def set_tau(self, idx: int, nm: float) -> None:
        with self._lock:
            self._tau[idx] = float(nm)

    def set_mode(self, mode: str) -> None:
        with self._lock:
            self._mode = "damping" if mode == "damping" else "hybrid"

    @property
    def mode(self) -> str:
        return self._mode

    def set_gains(self, kp_joint: float, kd_joint: float) -> None:
        self._kp_joint = kp_joint
        self._kd_joint = kd_joint

    def zero_all_targets(self) -> None:
        with self._lock:
            for i in range(len(self.ids)):
                self._target[i] = 0.0

    def snapshot(self) -> list:
        with self._lock:
            return [{
                "id": self.ids[i],
                "online": True,
                "temp": 30,
                "err": 0,
                "enabled": self._enabled[i],
                "zero_valid": self._zero_valid[i],
                "target_deg": self._target[i],
                "direction": self._dir[i],
                "q_deg": self._q[i] * self._dir[i],
                "dq_deg": 0.0,
                "tau_out": ((0.0 if self._mode == "damping" else
                             self._kp_joint * math.radians(self._target[i] - self._q[i])
                             + self._tau[i])
                            if self._enabled[i] else 0.0),
            } for i in range(len(self.ids))]
