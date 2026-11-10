# -*- coding: utf-8 -*-
"""起立 / 趴下姿态 YAML 的读写。"""
from __future__ import annotations

import os
import shutil

try:
    import yaml
except ImportError:
    yaml = None

POSE_HEADER = """# 起立 / 趴下姿态（关节侧，单位：度，相对上电零点）
#
# 顺序：腿1[髋, 大腿, 小腿] 腿2[...] 腿3[...] 腿4[...]
#
# 使用流程：
#   1. 「连接（读零）」把当前姿态记为 0°
#   2. 电机未使能时手动摆好姿态，点「记录起立姿态」/「记录趴下姿态」
#   3. 「全部使能」后，用「起立 / 趴下」按钮做动作调试
#   4. 调好后点「保存姿态」写回本文件
#
# front_delay / rear_delay：起立时前腿/后腿的起步延时（占过渡时间的比例）。
#   狗往前翻 → 增大 rear_delay；狗往后仰 → 增大 front_delay。"""

DEFAULT_FRONT_DELAY = 0.0
DEFAULT_REAR_DELAY = 0.15


def load_poses(path: str, size: int, default_duration: float = 3.0):
    """返回 (lie, stand, duration)；文件不存在时返回全 0（占位）。"""
    lie = [0.0] * size
    stand = [0.0] * size
    duration = default_duration
    if not os.path.isfile(path) or yaml is None:
        return lie, stand, duration

    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}

    def fix(seq):
        values = [float(v) for v in (seq or [])][:size]
        values += [0.0] * (size - len(values))
        return values

    return (fix(data.get("lie")), fix(data.get("stand")),
            float(data.get("duration", default_duration)))


def load_timing(path: str):
    """返回 (front_delay, rear_delay)；文件没有这两个键时用默认值。"""
    if not os.path.isfile(path) or yaml is None:
        return DEFAULT_FRONT_DELAY, DEFAULT_REAR_DELAY
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    return (float(data.get("front_delay", DEFAULT_FRONT_DELAY)),
            float(data.get("rear_delay", DEFAULT_REAR_DELAY)))


def save_poses(path: str, lie, stand, duration: float,
               front_delay: float = DEFAULT_FRONT_DELAY,
               rear_delay: float = DEFAULT_REAR_DELAY) -> None:
    text = [POSE_HEADER, "",
            "duration: %.1f" % duration, "",
            "front_delay: %.2f" % front_delay,
            "rear_delay:  %.2f" % rear_delay,
            "",
            "lie:   [%s]" % ", ".join("%.1f" % v for v in lie),
            "stand: [%s]" % ", ".join("%.1f" % v for v in stand), ""]
    parent = os.path.dirname(os.path.abspath(path))
    os.makedirs(parent, exist_ok=True)
    # 覆盖前留一份 .bak，避免误存后无法回退
    if os.path.isfile(path):
        shutil.copyfile(path, path + ".bak")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(text))
