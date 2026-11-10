# -*- coding: utf-8 -*-
"""Linux joystick（/dev/input/js*）读取，纯标准库实现。"""
from __future__ import annotations

import glob
import os
import select
import struct
from dataclasses import dataclass

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

_JS_EVENT = struct.Struct("<IhBB")


@dataclass
class GamepadEvent:
    kind: str          # "button" / "axis"
    number: int
    value: int
    initial: bool = False

    @property
    def pressed(self) -> bool:
        return self.kind == "button" and self.value == 1


def find_gamepads() -> list:
    """返回 [(设备路径, 名称)]。"""
    result = []
    for path in sorted(glob.glob("/dev/input/js*")):
        name = ""
        sysfs = os.path.join("/sys/class/input", os.path.basename(path),
                             "device/name")
        try:
            with open(sysfs, "r", encoding="utf-8") as fh:
                name = fh.read().strip()
        except OSError:
            pass
        result.append((path, name))
    return result


class Gamepad:
    """非阻塞读取手柄事件；维护 buttons / axes 当前状态。"""

    def __init__(self, device: str = None):
        if device is None:
            pads = find_gamepads()
            if not pads:
                raise RuntimeError("没有找到手柄（/dev/input/js*）")
            device = pads[0][0]
        self.device = device
        self.fd = os.open(device, os.O_RDONLY | os.O_NONBLOCK)
        self.buttons = {}
        self.axes = {}
        self._buffer = b""

    @property
    def name(self) -> str:
        sysfs = os.path.join("/sys/class/input", os.path.basename(self.device),
                             "device/name")
        try:
            with open(sysfs, "r", encoding="utf-8") as fh:
                return fh.read().strip()
        except OSError:
            return self.device

    def decode(self, data: bytes) -> list:
        """把原始字节流解析成事件（便于离线测试）。"""
        events = []
        self._buffer += data
        while len(self._buffer) >= _JS_EVENT.size:
            raw, self._buffer = self._buffer[:_JS_EVENT.size], self._buffer[_JS_EVENT.size:]
            _time, value, ev_type, number = _JS_EVENT.unpack(raw)
            initial = bool(ev_type & JS_EVENT_INIT)
            base = ev_type & 0x7F
            if base == JS_EVENT_BUTTON:
                kind = "button"
                self.buttons[number] = value
            elif base == JS_EVENT_AXIS:
                kind = "axis"
                self.axes[number] = value
            else:
                continue
            events.append(GamepadEvent(kind, number, value, initial))
        return events

    def poll(self, timeout: float = 0.05) -> list:
        """最多等 timeout 秒，返回这一批事件。"""
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if not ready:
            return []
        try:
            data = os.read(self.fd, _JS_EVENT.size * 64)
        except BlockingIOError:
            return []
        return self.decode(data)

    def close(self) -> None:
        if self.fd is not None:
            try:
                os.close(self.fd)
            finally:
                self.fd = None

    def __enter__(self) -> "Gamepad":
        return self

    def __exit__(self, *exc_info) -> None:
        self.close()
