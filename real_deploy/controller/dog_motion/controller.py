# -*- coding: utf-8 -*-
"""四腿动作控制：连接 -> 读零 -> 使能 -> 起立/趴下。

面向状态机编程：所有动作非阻塞，用 is_moving / wait 判断完成。
"""
from __future__ import annotations

import os
import math
import threading
import time

from .bus import LegBus, SimulatedBus, GEAR
from .policy_pose import resolve_policy_target
from .config import load_config
from .poses import load_poses, load_timing, save_poses
from .trajectory import interpolate, lie_profile, stand_profile

from .repo import find_repo_root

_ROOT = find_repo_root()
DEFAULT_CONFIG = os.path.join(_ROOT, "config", "config.json")
DEFAULT_MAPPING = os.path.join(_ROOT, "config", "leg_direction_mapping.yaml")
DEFAULT_POSES = os.path.join(_ROOT, "config", "stand_poses.yaml")


class MotionError(RuntimeError):
    """连接、使能或运动前置条件不满足。"""


class MotionController:
    """四条腿的起立/趴下动作库。

    典型状态机用法::

        ctrl = MotionController.from_config()
        ctrl.connect()
        ctrl.zero()            # 上电读 0
        ctrl.enable()          # 力位混合
        ctrl.start_motion("stand")
        ctrl.wait()

    姿态 lie / stand 是电机输出轴角度（度，相对软件零点），可以修改，或用
    save_current_as() 从当前姿态采样。
    """

    def __init__(self, legs: list, pose_path: str = DEFAULT_POSES,
                 kp: float = 60.0, kd: float = 3.0, rate_hz: float = 100.0,
                 damping_kd: float = 3.0, sim: bool = False,
                 motion_period: float = 0.02,
                 auto_disable_after_lie: bool = True,
                 auto_enable_stand: bool = True,
                 lie_settle_tolerance_deg: float = 3.0,
                 lie_settle_timeout: float = 5.0):
        self.legs = legs
        self.pose_path = pose_path
        self.kp = float(kp)
        self.kd = float(kd)
        self.damping_kd = float(damping_kd)
        self.rate_hz = float(rate_hz)
        self.sim = sim
        self.motion_period = float(motion_period)
        # 趴下「完全到位」后自动松弛；起立前如果没使能则自动使能
        self.auto_disable_after_lie = bool(auto_disable_after_lie)
        self.auto_enable_stand = bool(auto_enable_stand)
        self.lie_settle_tolerance_deg = float(lie_settle_tolerance_deg)
        self.lie_settle_timeout = float(lie_settle_timeout)

        self.size = sum(len(leg["ids"]) for leg in legs)
        self.lie, self.stand, self.duration = load_poses(pose_path, self.size)
        # 起立时前后腿的相位差（0~0.9，占过渡时间比例）
        self.front_delay, self.rear_delay = load_timing(pose_path)

        self.buses = []
        self.errors = []
        self._motion = None
        self._thread = None
        self.motion_error = None
        self.motion_fault_detail = None
        self.last_feedback_fault = None
        self.policy_spec_path = os.path.join(_ROOT, 'config', 'policy_stand.yaml')
        self.policy_reference_path = os.path.join(_ROOT, 'config',
            'policy_encoder_reference.sim.yaml' if self.sim else 'policy_encoder_reference.yaml')

    # -- 构造 -------------------------------------------------------------
    @classmethod
    def from_config(cls, config_path: str = DEFAULT_CONFIG,
                    mapping_path: str = DEFAULT_MAPPING,
                    pose_path: str = DEFAULT_POSES,
                    sim: bool = False, **kwargs) -> "MotionController":
        cfg = load_config(config_path, mapping_path)
        return cls(cfg["legs"], pose_path,
                   kp=cfg["gains"]["kp"], kd=cfg["gains"]["kd"],
                   rate_hz=cfg["rate_hz"], sim=sim, **kwargs)

    # -- 连接与清理 -------------------------------------------------------
    def connect(self) -> int:
        """打开四条腿的总线并启动轮询，返回在线电机数。"""
        if self.buses:
            return self.online_count()
        bus_cls = SimulatedBus if self.sim else LegBus
        self.errors = []
        self.buses = []
        for leg in self.legs:
            try:
                bus = bus_cls(leg["name"], leg["port"], leg["ids"], leg["joints"],
                              kp_joint=self.kp, kd_joint=self.kd,
                              rate_hz=self.rate_hz)
                for index, direction in enumerate(leg.get("directions", [])):
                    bus.set_direction(index, direction)
                bus.start()
            except Exception as exc:  # noqa: BLE001
                self.errors.append("%s（%s）：%s" %
                                   (leg["name"], leg["port"], exc))
                bus = None
            self.buses.append(bus)

        time.sleep(0.05 if self.sim else 0.3)  # 等第一轮反馈
        if all(bus is None for bus in self.buses):
            raise MotionError("所有通道都打不开：%s" % "; ".join(self.errors))
        return self.online_count()

    def online_count(self) -> int:
        count = 0
        for bus in self.buses:
            if bus is not None:
                count += sum(1 for snap in bus.snapshot() if snap["online"])
        return count

    def close(self) -> None:
        self.stop_motion()
        self.release()
        for bus in self.buses:
            if bus is not None:
                try:
                    bus.stop()
                except Exception:  # noqa: BLE001
                    pass
        self.buses = []

    def __enter__(self) -> "MotionController":
        self.connect()
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()

    # -- 状态 -------------------------------------------------------------
    def capture_zero(self) -> int:
        """把当前姿态记为 0°，返回成功个数。"""
        count = 0
        for bus in self.buses:
            if bus is None:
                continue
            count += bus.capture_zero()
            bus.zero_all_targets()
        return count

    #: 兼容别名
    zero = capture_zero

    def enable(self) -> int:
        """力位混合使能：保持当前姿态，返回使能个数。"""
        if not self.buses:
            raise MotionError("未连接，先调用 connect()")
        pose = self.current_pose(require_online=True)
        self.apply_pose(pose)
        count = 0
        for bus in self.buses:
            if bus is None:
                continue
            bus.set_mode("hybrid")
            bus.set_gains(self.kp, self.kd)
            count += bus.set_enabled(True)
        if count == 0:
            raise MotionError("没有在线电机")
        return count

    def enable_damping(self) -> int:
        """只阻尼使能（kp=0、kd=damping_kd），便于手动摆姿态。"""
        if not self.buses:
            raise MotionError("未连接，先调用 connect()")
        self.current_pose(require_online=True)
        count = 0
        for bus in self.buses:
            if bus is None:
                continue
            bus.set_mode("damping")
            bus.set_gains(0.0, self.damping_kd)
            count += bus.set_enabled(True)
        return count

    def release(self) -> None:
        """全部松弛：kp = kd = tau = 0。"""
        self._disable_all()
        self.stop_motion()

    def _disable_all(self) -> None:
        """只关使能，不中断动作线程（供动作线程内部调用）。"""
        for bus in self.buses:
            if bus is not None:
                bus.set_enabled(False)

    @property
    def is_enabled(self) -> bool:
        if not self.buses or any(bus is None for bus in self.buses):
            return False
        return all(all(snap["enabled"] for snap in bus.snapshot())
                   for bus in self.buses)

    @property
    def is_moving(self) -> bool:
        return self._motion is not None

    # -- 动作 -------------------------------------------------------------
    def start_motion(self, name: str, duration: float = None) -> bool:
        """启动起立/趴下（非阻塞）。name = stand | lie。"""
        name = name.lower()
        original = name == 'stand-original'
        if name not in ("stand", "stand-original", "lie"):
            raise ValueError("动作只能是 stand、stand-original 或 lie")
        if self.is_moving:
            raise MotionError("上一个动作还没结束")
        if not self.buses:
            raise MotionError("未连接，先调用 connect()")
        from .policy_pose import has_confirmed_default_stand
        if name == 'stand' and has_confirmed_default_stand(self.pose_path):
            self.stand=load_poses(self.pose_path,self.size)[1]
        elif name == 'stand' and os.path.isfile(self.policy_spec_path):
            self.current_pose(require_online=True)
            snaps = [s for b in self.buses for s in b.snapshot()]
            if any(not s['zero_valid'] for s in snaps):
                raise MotionError('尚未建立趴下零位')
            topology = [{k:l[k] for k in ('code','port','ids','directions')} for l in self.legs]
            zeros = [s.get('zero_rotor_rad',0.)/GEAR for s in snaps]
            try:
                self.stand = resolve_policy_target(self.policy_spec_path,
                    self.policy_reference_path,topology,zeros,self.sim)
            except ValueError as exc:
                raise MotionError(str(exc)) from exc
        if original:
            with open(self.pose_path, encoding='utf-8') as stream:
                import yaml
                original_pose = yaml.safe_load(stream)['stand']
            if (len(original_pose) != self.size or
                    not all(math.isfinite(v) and abs(v) <= 180 for v in original_pose)):
                raise MotionError('原起立目标必须为 12 个有效电机输出轴角度（±180°）')
        if not self.is_enabled:
            if name in ("stand", "stand-original") and self.auto_enable_stand:
                self.enable()      # 起立前自动使能，保持当前姿态再起
            else:
                raise MotionError("电机未使能，先调用 enable()")

        start = self.current_pose(require_online=True)
        end = list(original_pose if original else self.stand if name == "stand" else self.lie)
        if len(end) != self.size or not all(math.isfinite(v) for v in end):
            raise MotionError("动作目标长度或数值无效")
        self.motion_error = None
        self.motion_fault_detail = None
        self._motion = {
            "name": name,
            "start": start,
            "end": end,
            "t0": time.monotonic(),
            "duration": float(duration or self.duration or 3.0),
            "profile": (lie_profile(self.legs) if name == "lie"
                        else stand_profile(self.legs, self.front_delay,
                                           self.rear_delay)),
        }
        self._thread = threading.Thread(target=self._run_motion,
                                        name="motion-%s" % name, daemon=True)
        self._thread.start()
        return True

    def _run_motion(self) -> None:
        motion = self._motion
        completed = False
        while motion is not None and self._motion is motion:
            try:
                self.current_pose(require_online=True)
            except MotionError as exc:
                self.motion_error = str(exc)
                self.motion_fault_detail = self.last_feedback_fault
                self._disable_all()
                self._motion = None
                return
            progress = ((time.monotonic() - motion["t0"]) / motion["duration"])
            progress = max(0.0, min(1.0, progress))
            pose = interpolate(motion["start"], motion["end"],
                               motion["profile"], progress)
            self.apply_pose(pose)
            if progress >= 1.0:
                completed = True
                break
            time.sleep(self.motion_period)
        if motion is None or self._motion is not motion:
            return
        # 趴下：轨迹走完后先等实测角度真正到位，再失能
        if (completed and motion["name"] == "lie"
                and self.auto_disable_after_lie):
            if not self._wait_lie_settled(motion):
                self.motion_error = '趴下超时，实测角度未进入目标容差'
            if self._motion is not motion:      # 等待期间被 stop_motion 取消
                return
            self._disable_all()
        self._motion = None

    def start_jog(self, index: int, target_deg: float, speed_deg_s: float = 2.0) -> None:
        """单轴微调：每次最多 5°，余弦缓动的峰值速度不超过给定速度。"""
        if self.is_moving or not self.is_enabled:
            raise MotionError('微调前须使能保持，且上一次移动已结束')
        if not isinstance(index, int) or not 0 <= index < self.size:
            raise ValueError('无效的电机索引')
        if (not math.isfinite(target_deg) or abs(target_deg) > 180
                or not math.isfinite(speed_deg_s) or not 0.2 <= speed_deg_s <= 5):
            raise ValueError('目标须在 ±180° 内，速度须在 0.2～5°/s 内')
        self.current_pose(require_online=True)
        start = [s['target_deg'] for b in self.buses for s in b.snapshot()]
        delta = target_deg - start[index]
        if not math.isfinite(delta) or abs(delta) > 5.000001:
            raise ValueError('每次微调最多 5°；请分次调整')
        end = list(start)
        end[index] = float(target_deg)
        self.motion_error = None
        self.motion_fault_detail = None
        self._motion = dict(name='jog', start=start, end=end,
                            t0=time.monotonic(), profile=None,
                            duration=max(0.25, math.pi * abs(delta) / (2 * speed_deg_s)))
        self._thread = threading.Thread(target=self._run_motion,
                                        name='motion-jog', daemon=True)
        self._thread.start()

    def start_jog_group(self, joint: int, delta_deg: float,
                        speed_deg_s: float = 2.0) -> None:
        """四腿同类电机同步微调；保留原有各腿差值，使用同一条缓动轨迹。"""
        if self.is_moving or not self.is_enabled:
            raise MotionError('先起立保持，并等待上一个动作结束')
        if type(joint) is not int or not 0 <= joint < 3:
            raise ValueError('关节索引须为 0（髋）、1（大腿）、2（小腿）')
        if (not math.isfinite(delta_deg) or not 0 < abs(delta_deg) <= 5 or
                not math.isfinite(speed_deg_s) or not 0.2 <= speed_deg_s <= 5):
            raise ValueError('每次微调须大于 0 且不超过 5°，速度须在 0.2～5°/s 内')
        current = self.current_pose(require_online=True)
        start = [s['target_deg'] for b in self.buses for s in b.snapshot()]
        if max(abs(q-t) for q,t in zip(current,start)) > 10:
            raise MotionError('实际角度与目标相差超过 10°，先检查支撑和姿态再继续微调')
        end = list(start)
        for leg_index in range(len(self.legs)):
            end[3*leg_index+joint] += float(delta_deg)
        if not all(math.isfinite(v) and abs(v) <= 180 for v in end):
            raise ValueError('微调后的电机输出轴目标超出 ±180°')
        self.motion_error = None
        self.motion_fault_detail = None
        self._motion = dict(name='jog-group', start=start, end=end,
                            t0=time.monotonic(), profile=None,
                            duration=max(0.25, math.pi*abs(delta_deg)/(2*speed_deg_s)))
        self._thread = threading.Thread(target=self._run_motion,
                                        name='motion-jog-group', daemon=True)
        self._thread.start()

    def _wait_lie_settled(self, motion) -> bool:
        """等 12 个关节实测角度进入趴下目标容差，超时则返回 False。

        等待期间 is_moving 仍为 True，防止中途插入起立。
        """
        deadline = time.monotonic() + max(0.0, self.lie_settle_timeout)
        while self._motion is motion and time.monotonic() < deadline:
            try:
                pose = self.current_pose(require_online=True)
            except MotionError:
                pose = None
            if pose is not None:
                error = max(abs(pose[i] - motion["end"][i])
                            for i in range(self.size))
                if error <= self.lie_settle_tolerance_deg:
                    return True
            time.sleep(self.motion_period)
        return False

    def stop_motion(self) -> None:
        self._motion = None
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.0)
            self._thread = None

    def wait(self, timeout: float = None) -> bool:
        """等待当前动作结束；返回是否已结束。"""
        if self._thread is not None:
            self._thread.join(timeout=timeout)
        return not self.is_moving

    # -- 目标与姿态 -------------------------------------------------------
    def apply_pose(self, pose) -> None:
        offset = 0
        for leg_index, leg in enumerate(self.legs):
            bus = self.buses[leg_index] if leg_index < len(self.buses) else None
            if bus is not None:
                for motor_index in range(len(leg["ids"])):
                    bus.set_target(motor_index, pose[offset + motor_index])
            offset += len(leg["ids"])

    def current_pose(self, require_online: bool = False) -> list:
        pose = []
        for leg_index, leg in enumerate(self.legs):
            bus = self.buses[leg_index] if leg_index < len(self.buses) else None
            if bus is None:
                raise MotionError("腿 %s 未连接" % leg["name"])
            for snap in bus.snapshot():
                if require_online and (not snap["online"] or snap["err"] != 0
                                       or not math.isfinite(snap["q_deg"])):
                    self.last_feedback_fault = dict(leg=leg['code'],port=leg['port'],
                        joint=leg['joints'][leg['ids'].index(snap['id'])],feedback=dict(snap))
                    raise MotionError("%s 电机 ID %d 离线、故障或角度无效" % (leg["code"], snap["id"]))
                pose.append(snap["q_deg"])
        return pose

    def save_current_as(self, name: str) -> list:
        """把当前实际姿态存为起立/趴下并写入 YAML。"""
        name = name.lower()
        if name not in ("stand", "lie"):
            raise ValueError("只能是 stand 或 lie")
        pose = self.current_pose(require_online=True)
        if name == "stand":
            self.stand = list(pose)
        else:
            self.lie = list(pose)
        save_poses(self.pose_path, self.lie, self.stand, self.duration,
                   self.front_delay, self.rear_delay)
        return pose

    def set_stand_timing(self, front_delay: float, rear_delay: float) -> None:
        """设置起立前后腿相位差（0~0.9）。前翻就加大 rear_delay。"""
        self.front_delay = max(0.0, min(0.9, float(front_delay)))
        self.rear_delay = max(0.0, min(0.9, float(rear_delay)))

    def set_gains(self, kp: float, kd: float) -> None:
        self.kp, self.kd = float(kp), float(kd)
        for bus in self.buses:
            if bus is None:
                continue
            if bus.mode == "damping":
                bus.set_gains(0.0, self.damping_kd)
            else:
                bus.set_gains(self.kp, self.kd)
