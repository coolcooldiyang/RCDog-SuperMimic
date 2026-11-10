# -*- coding: utf-8 -*-
"""机器人拓扑配置：运行参数 JSON + 方向映射 YAML。"""
from __future__ import annotations

import json
import os

try:
    import yaml
except ImportError:  # 需要 python3-yaml
    yaml = None

MAPPING_HEADER = """# 四腿关节方向映射
#
# 约定
#   - 关节顺序固定为 [髋, 大腿, 小腿]，与电机 ID [1, 2, 3] 一一对应
#   - direction = +1：指令为正时，关节朝机器人标准正方向运动
#   - direction = -1：该关节需要取反（指令和反馈都取反）
#   - 每条腿独占一个 485 通道，电机 ID 在不同通道上可以重复
#   - 本表由 testgui 逐腿测试得出（滑杆给正、看实际转向后确定符号）"""


def _require_yaml() -> None:
    if yaml is None:
        raise RuntimeError("缺少 PyYAML：sudo apt install python3-yaml")


def load_mapping(path: str) -> list:
    """读取方向映射 YAML，返回腿列表（含 port/ids/joints/directions）。"""
    _require_yaml()
    with open(path, "r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    joints = data.get("joint_order") or ["髋", "大腿", "小腿"]
    motor_ids = data.get("motor_ids") or [1, 2, 3]
    legs = []
    for index, leg in enumerate(data["legs"], 1):
        leg_id = int(leg.get("id", index))
        code = leg.get("code", "")
        legs.append({
            "id": leg_id,
            "short_name": leg.get("name", "腿%d" % leg_id),
            "code": code,
            "name": ("腿 %d · %s %s" % (leg_id, leg.get("name", ""), code)).strip(),
            "port": leg["port"],
            "ids": list(motor_ids),
            "joints": list(joints),
            "directions": [int(d) for d in leg.get("directions", [1] * len(motor_ids))],
        })
    return legs


def save_mapping(path: str, legs: list, directions: list) -> None:
    """把方向写回 YAML（保留表头注释）。"""
    lines = [MAPPING_HEADER, "", "version: 1", "",
             "joint_order: [髋, 大腿, 小腿]", "motor_ids: [1, 2, 3]", "", "legs:"]
    for leg, dirs in zip(legs, directions):
        lines += [
            "  - id: %d" % leg.get("id", 0),
            "    name: %s" % leg.get("short_name", leg.get("name", "")),
            "    code: %s" % leg.get("code", ""),
            "    port: %s" % leg["port"],
            "    directions: [%s]" % ", ".join(str(int(d)) for d in dirs),
            "",
        ]
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines).rstrip() + "\n")


def load_config(path: str, mapping_path: str) -> dict:
    """读取运行参数；腿的拓扑和方向以映射 YAML 为准。"""
    with open(path, "r", encoding="utf-8") as fh:
        cfg = json.load(fh)
    cfg.setdefault("rate_hz", 100)
    cfg.setdefault("range_deg", 45)
    cfg.setdefault("debug_range_deg", 180)
    cfg.setdefault("gains", {"kp": 60.0, "kd": 3.0})

    if os.path.isfile(mapping_path):
        cfg["legs"] = load_mapping(mapping_path)
    elif cfg.get("legs"):
        for index, leg in enumerate(cfg["legs"], 1):
            leg.setdefault("joints", ["髋", "大腿", "小腿"])
            leg.setdefault("directions", [1] * len(leg["ids"]))
            leg.setdefault("id", index)
            leg.setdefault("short_name", leg["name"])
            leg.setdefault("code", "")
    else:
        raise RuntimeError("没有找到 %s，也没有 config.json 里的 legs" % mapping_path)
    return cfg
