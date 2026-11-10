# -*- coding: utf-8 -*-
"""姿态插值：余弦缓动 + 分段调度（趴下时小腿先收）。"""
from __future__ import annotations

import math


def smoothstep(progress: float) -> float:
    """0..1 的余弦缓动，起止速度都为 0。"""
    p = max(0.0, min(1.0, progress))
    return 0.5 - 0.5 * math.cos(math.pi * p)


def leg_is_front(leg: dict) -> bool:
    """按 code（FL/FR/RL/RR）或名字判断前腿；未知默认算前腿。"""
    code = (leg.get("code") or "").upper()
    if code in ("FL", "FR"):
        return True
    if code in ("RL", "RR"):
        return False
    name = leg.get("name", "")
    if "前" in name:
        return True
    if "后" in name:
        return False
    return True


def stand_profile(legs: list, front_delay: float = 0.0,
                  rear_delay: float = 0.15) -> list:
    """起立时序：按腿给出 (delay, span)，用来错开前后腿。

    狗往前翻 → 后腿慢一点（增大 rear_delay）；
    狗往后仰 → 前腿慢一点（增大 front_delay）。
    两侧结束时间都在 100%，所以只是起跑早晚不同。
    """
    profile = []
    for leg in legs:
        delay = front_delay if leg_is_front(leg) else rear_delay
        delay = max(0.0, min(0.9, float(delay)))
        span = 1.0 - delay
        for _ in leg["ids"]:
            profile.append((delay, span))
    return profile


def lie_profile(legs: list, calf_name: str = "小腿",
                calf_span: float = 0.50,
                other_delay: float = 0.45,
                other_span: float = 0.55) -> list:
    """趴下时序：小腿先行，髋/大腿后半程落下。

    返回每个关节的 (delay, span) 占比，顺序与 legs/ids 一致。
    起立不需要 profile，直接同步插值。
    """
    profile = []
    for leg in legs:
        joints = leg["joints"]
        calf_index = joints.index(calf_name) if calf_name in joints else 2
        for motor_index in range(len(leg["ids"])):
            if motor_index == calf_index:
                profile.append((0.0, calf_span))
            else:
                profile.append((other_delay, other_span))
    return profile


def interpolate(start, end, profile, progress: float) -> list:
    """按进度采样目标姿态；profile 为 None 时所有关节同步。"""
    pose = []
    for index in range(len(start)):
        delay, span = profile[index] if profile else (0.0, 1.0)
        local = (progress - delay) / span if span > 0 else 1.0
        weight = smoothstep(local)
        pose.append(start[index] + (end[index] - start[index]) * weight)
    return pose
