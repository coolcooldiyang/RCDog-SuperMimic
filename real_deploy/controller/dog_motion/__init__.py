# -*- coding: utf-8 -*-
"""dog_motion —— 四腿起立/趴下动作库（宇树 GO-M8010-6）。

    from dog_motion import MotionController

    ctrl = MotionController.from_config()
    ctrl.connect()
    ctrl.zero()
    ctrl.enable()
    ctrl.start_motion("stand")
    ctrl.wait()
"""
from .bus import LegBus, SimulatedBus, MotorState
from .gamepad import Gamepad, GamepadEvent, find_gamepads
from .config import load_config, load_mapping, save_mapping
from .poses import load_poses, load_timing, save_poses
from .trajectory import (interpolate, leg_is_front, lie_profile, smoothstep,
                         stand_profile)
from .controller import MotionController, MotionError

__all__ = [
    "LegBus", "SimulatedBus", "MotorState",
    "Gamepad", "GamepadEvent", "find_gamepads",
    "load_config", "load_mapping", "save_mapping",
    "load_poses", "save_poses", "load_timing",
    "interpolate", "lie_profile", "stand_profile", "leg_is_front", "smoothstep",
    "MotionController", "MotionError",
]
