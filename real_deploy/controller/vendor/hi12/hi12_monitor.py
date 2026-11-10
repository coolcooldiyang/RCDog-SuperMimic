#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HI12M0-MI1-000 实时数据监控
============================

硬件 : HiPNUC HI12 系列 6-DoF IMU (HI12M0)，经 CP2102N USB-UART 桥接器接入
协议 : HiPNUC "CHAOHE" 协议，帧头 5A A5，6 字节帧头，数据包 tag = 0x91
速率 : 默认 115200 8N1，输出 100 Hz（实测 99 帧/秒，时间戳每帧 +10 ms）

帧结构（实测 + 官方 hipnuc.h 的 hi91_t + HiPNUC 手册 0x91 字段表三方确认）：

    [0:2]    5A A5            同步头
    [2:4]    u16 LE           数据长度 = 76
    [4:6]    u16 LE           CRC16（poly 0x1021, init 0, 非反射）
    [6:82]   载荷 hi91_t（76 字节）
                off  0   u8       tag         固定 0x91
                off  1   u16      pps_sync_ms / 保留
                off  3   i8       temp        温度 (degC)
                off  4   f32      prs         气压 (Pa)，6-DoF 机型恒 0
                off  8   u32      ts          时间戳 (ms)
                off 12   f32 x3   acc         加速度 (g)     <- 见下方单位说明
                off 24   f32 x3   gyr         角速度 (deg/s) <- 手册明确
                off 36   f32 x3   mag         磁场 (uT)，6-DoF 机型恒 0
                off 48   f32 x3   roll,pitch,yaw 欧拉角 (deg)
                off 60   f32 x4   quat        四元数 (W, X, Y, Z)
    [82]     下一帧

CRC 校验范围（手册 8.5.1 节）：crc = 0; update(buf[0:4]); update(payload)
已用实测 99 帧 + 手册自带示例帧双重验证通过。

单位说明（重要）
  * 陀螺 deg/s：HiPNUC 官方手册 0x91 字段表明确标注 "deg/s"。
  * 加速度 g：官方字段表标注 "mg"，但实测静止时数值模长 = 0.9986~1.0000，
    物理上只能是 g（若为 mg 则意味着自由落体）。故按 g 显示。
    如需按字面 mg 处理，用 --accel-unit mg（数值会显示为 0.0009，无物理意义）。

欧拉角约定（与常见 IMU 不同，易踩坑，但不是故障）
  Z-X-Y (312) 旋转序，且 Roll 绕 Y 轴、Pitch 绕 X 轴、Yaw 绕 Z 轴。
  用常规 ZYX 公式反算会对不上。已对实测帧验证：由四元数按该约定反算，
  与设备上报的 roll/pitch/yaw 最大偏差 0.0001 度。

用法：
    python3 hi12_monitor.py                  # 实时刷新显示（Ctrl-C 退出）
    python3 hi12_monitor.py --csv log.csv    # 同时存 CSV
    python3 hi12_monitor.py --raw            # 打印每帧原始字节
    python3 hi12_monitor.py --verify         # 用四元数交叉校验 euler 字段
    python3 hi12_monitor.py --port /dev/ttyUSB0 --baud 115200
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import struct
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺少 pyserial，请先执行： pip3 install --user pyserial")

# ---------------------------------------------------------------- 协议常量
SYNC = b"\x5a\xa5"
HDR_SIZE = 6
TAG_IMU_FLOAT = 0x91
PAYLOAD_LEN = 76
FRAME_LEN = HDR_SIZE + PAYLOAD_LEN          # 82
DEFAULT_BAUD = 115200
CRC_POLY = 0x1021

# 载荷内字段偏移（字节）
P_TAG, P_PPS, P_TEMP, P_PRS, P_TS = 0, 1, 3, 4, 8
P_ACC, P_GYR, P_MAG, P_EUL, P_QUAT = 12, 24, 36, 48, 60
assert P_QUAT + 16 == PAYLOAD_LEN, "字段偏移与载荷长度不一致"

# 单位换算
DPS_TO_RADS = math.pi / 180.0
G_TO_MS2 = 9.80665


def crc16_update(crc: int, data: bytes) -> int:
    """HiPNUC 手册给出的 crc16_update：poly 0x1021, init 0, 非反射。"""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def frame_crc(frame: bytes) -> int:
    """帧内 CRC：先算 buf[0:4]（同步头+长度），再算载荷（跳过 CRC 两字节）。"""
    ln = int.from_bytes(frame[2:4], "little")
    return crc16_update(crc16_update(0, frame[:4]), frame[HDR_SIZE:HDR_SIZE + ln])


# ---------------------------------------------------------------- 帧解析
class FrameParser:
    """增量式字节流解析器：从串口噪声中同步并切出完整帧。"""

    def __init__(self, check_crc: bool = True) -> None:
        self.buf = bytearray()
        self.check_crc = check_crc
        self.bad = 0            # 同步失败 / 长度或 tag 不符
        self.bad_crc = 0        # CRC 校验失败
        self.dropped_bytes = 0

    def feed(self, data: bytes):
        self.buf += data
        frames = []
        while True:
            i = self.buf.find(SYNC)
            if i < 0:
                if len(self.buf) > 1:          # 保留末字节，防同步头被切断
                    self.dropped_bytes += len(self.buf) - 1
                    del self.buf[:-1]
                break
            if i > 0:
                self.dropped_bytes += i
                del self.buf[:i]
            if len(self.buf) < HDR_SIZE:
                break
            ln = int.from_bytes(self.buf[2:4], "little")
            if ln != PAYLOAD_LEN or (len(self.buf) > HDR_SIZE
                                     and self.buf[HDR_SIZE] != TAG_IMU_FLOAT):
                self.bad += 1                  # 假同步头，跳过 2 字节重找
                del self.buf[:2]
                continue
            if len(self.buf) < FRAME_LEN:
                break
            frame = bytes(self.buf[:FRAME_LEN])
            del self.buf[:FRAME_LEN]
            if self.check_crc and frame_crc(frame) != int.from_bytes(frame[4:6], "little"):
                self.bad_crc += 1
                continue
            frames.append(frame)
        return frames


class ImuSample:
    __slots__ = ("tag", "pps_sync_ms", "temp", "prs", "ts", "acc", "gyr",
                 "mag", "roll", "pitch", "yaw", "quat")

    def __init__(self, frame: bytes) -> None:
        p = frame[HDR_SIZE:HDR_SIZE + PAYLOAD_LEN]
        self.tag = p[P_TAG]
        self.pps_sync_ms = struct.unpack_from("<H", p, P_PPS)[0]
        self.temp = struct.unpack_from("<b", p, P_TEMP)[0]
        self.prs = struct.unpack_from("<f", p, P_PRS)[0]
        self.ts = struct.unpack_from("<I", p, P_TS)[0]
        self.acc = struct.unpack_from("<3f", p, P_ACC)
        self.gyr = struct.unpack_from("<3f", p, P_GYR)
        self.mag = struct.unpack_from("<3f", p, P_MAG)
        self.roll, self.pitch, self.yaw = struct.unpack_from("<3f", p, P_EUL)
        self.quat = struct.unpack_from("<4f", p, P_QUAT)


def quat_to_dcm(q):
    """四元数 (w,x,y,z) -> 3x3 旋转矩阵（行优先）。"""
    w, x, y, z = q
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - w * z),     2 * (x * z + w * y)),
        (2 * (x * y + w * z),     1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
        (2 * (x * z - w * y),     2 * (y * z + w * x),     1 - 2 * (x * x + y * y)),
    )


def quat_to_euler_zxy(q):
    """按设备约定 (Z-X-Y / 312 序, Roll绕Y, Pitch绕X) 由四元数反算 euler(deg)。

    已对实测帧验证：与该设备上报的 roll/pitch/yaw 最大偏差 0.0001 度。
    """
    r = quat_to_dcm(q)
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, r[2][1]))))
    roll = math.degrees(math.atan2(-r[2][0], r[2][2]))
    yaw = math.degrees(math.atan2(-r[0][1], r[1][1]))
    return roll, pitch, yaw


# ---------------------------------------------------------------- 端口探测
def autodetect_port():
    by_id = sorted(glob.glob("/dev/serial/by-id/*CP210*"))
    if by_id:
        return by_id[0]
    for p in list_ports.comports():
        if p.vid == 0x10C4:               # Silicon Labs
            return p.device
    ttys = sorted(glob.glob("/dev/ttyUSB*"))
    return ttys[0] if ttys else None


# ---------------------------------------------------------------- 显示
def _units(s, args):
    a, g = s.acc, s.gyr
    if args.accel_unit == "ms2":
        a, au = tuple(v * G_TO_MS2 for v in a), "m/s²"
    elif args.accel_unit == "mg":
        a, au = tuple(v * 1000.0 for v in a), "mg"
    else:
        a, au = a, "g"
    if args.gyro_unit == "rad":
        g, gu = tuple(v * DPS_TO_RADS for v in g), "rad/s"
    else:
        g, gu = g, "°/s"
    return a, au, g, gu


def render(s, args, stats, uptime):
    a, au, g, gu = _units(s, args)
    norm_a = math.sqrt(sum(v * v for v in s.acc))
    L = [
        f"HI12M0-MI1-000  实时数据     端口 {args.port}   {args.baud} 8N1",
        f"运行 {uptime:7.1f}s   帧率 {stats['hz']:6.1f} Hz   有效帧 {stats['frames']:<8d}"
        f"  CRC错误 {stats['bad_crc']:<4d}  解析错误 {stats['bad']:<4d}"
        f"  时间戳丢帧 {stats['gaps']:<4d}",
        "",
        "── 姿态 euler (deg, 设备约定 Z-X-Y / Roll绕Y, Pitch绕X) ─────────",
        f"    Roll (绕Y) : {s.roll:+10.3f}      Pitch (绕X) : {s.pitch:+10.3f}"
        f"      Yaw (绕Z) : {s.yaw:+10.3f}",
        "",
        f"── 加速度 ({au}) ────────────────────────────  |a| = {norm_a:.4f} g",
        f"    X {a[0]:+10.5f}      Y {a[1]:+10.5f}      Z {a[2]:+10.5f}",
        "",
        f"── 角速度 ({gu}) ───────────────────────────────────────",
        f"    X {g[0]:+10.5f}      Y {g[1]:+10.5f}      Z {g[2]:+10.5f}",
        "",
        "── 四元数 (W, X, Y, Z) ──────────────────────────────────",
        f"    {s.quat[0]:+9.5f} {s.quat[1]:+9.5f} {s.quat[2]:+9.5f} {s.quat[3]:+9.5f}"
        f"     |q| = {math.sqrt(sum(v * v for v in s.quat)):.5f}",
        "",
        "── 其他 ─────────────────────────────────────────────────",
        f"    温度 {s.temp} °C    时间戳 {s.ts} ms    pps_sync_ms {s.pps_sync_ms}"
        f"    气压 {s.prs:.3f} Pa",
        f"    磁力 ({s.mag[0]:.3f}, {s.mag[1]:.3f}, {s.mag[2]:.3f}) uT"
        f"   [HI12M0 为 6-DoF，无磁力计，恒为 0]",
    ]
    if args.verify:
        qr, qp, qy = quat_to_euler_zxy(s.quat)
        dev = max(abs(qr - s.roll), abs(qp - s.pitch), abs(qy - s.yaw))
        L.append(f"    [verify] 四元数反算 euler 与上报值最大偏差 {dev:.6f} °")
    L += ["", "    按 Ctrl-C 退出"]
    return "\n".join(L)


def render_line(s, args):
    a, au, g, gu = _units(s, args)
    line = (f"ts={s.ts:<10d} "
            f"acc=({a[0]:+.5f},{a[1]:+.5f},{a[2]:+.5f}){au} "
            f"gyr=({g[0]:+.4f},{g[1]:+.4f},{g[2]:+.4f}){gu} "
            f"rpy=({s.roll:+.3f},{s.pitch:+.3f},{s.yaw:+.3f})deg "
            f"quat=({s.quat[0]:+.4f},{s.quat[1]:+.4f},"
            f"{s.quat[2]:+.4f},{s.quat[3]:+.4f}) temp={s.temp}C")
    if args.verify:
        qr, qp, qy = quat_to_euler_zxy(s.quat)
        dev = max(abs(qr - s.roll), abs(qp - s.pitch), abs(qy - s.yaw))
        line += f" verify_dmax={dev:.6f}deg"
    return line


# ---------------------------------------------------------------- 主流程
def main():
    ap = argparse.ArgumentParser(
        description="HI12M0-MI1-000 (HiPNUC HI12 系列) 实时数据监控",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__.split("用法：")[-1])
    ap.add_argument("--port", default=None, help="串口设备，默认自动探测")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率，默认 115200")
    ap.add_argument("--hz", type=float, default=20.0, help="屏幕刷新率 Hz，默认 20")
    ap.add_argument("--csv", default=None, help="把每个样本追加写入 CSV 文件")
    ap.add_argument("--raw", action="store_true", help="打印每帧原始字节（hex）")
    ap.add_argument("--verify", action="store_true", help="每帧用四元数交叉校验 euler")
    ap.add_argument("--no-crc", action="store_true", help="关闭 CRC 校验（默认开启）")
    ap.add_argument("--duration", type=float, default=None, help="运行多少秒后自动退出")
    ap.add_argument("--accel-unit", choices=("g", "ms2", "mg"), default="g")
    ap.add_argument("--gyro-unit", choices=("deg", "rad"), default="deg",
                    help="陀螺单位，默认 deg/s（HiPNUC 手册明确标注）")
    args = ap.parse_args()

    if not args.port:
        args.port = autodetect_port()
        if not args.port:
            sys.exit("没找到串口设备。请确认 IMU 已插好，或用 --port 指定。")

    if not os.path.exists(args.port):
        sys.exit(f"串口 {args.port} 不存在。当前 ttyUSB: {glob.glob('/dev/ttyUSB*')}")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"打开 {args.port} 失败：{e}\n"
                 f"提示：确认当前用户在 dialout 组（groups | grep dialout）")

    csv_f = None
    if args.csv:
        new = not os.path.exists(args.csv) or os.path.getsize(args.csv) == 0
        csv_f = open(args.csv, "a", buffering=1)
        if new:
            csv_f.write("host_time,ts_ms,pps_sync_ms,temp_c,prs_pa,"
                        "acc_x_g,acc_y_g,acc_z_g,gyr_x_dps,gyr_y_dps,gyr_z_dps,"
                        "mag_x_uT,mag_y_uT,mag_z_uT,roll_deg,pitch_deg,yaw_deg,"
                        "qw,qx,qy,qz\n")

    parser = FrameParser(check_crc=not args.no_crc)
    stats = {"frames": 0, "hz": 0.0, "bad": 0, "bad_crc": 0, "gaps": 0}
    tty = sys.stdout.isatty()
    t0 = time.time()
    last_draw = 0.0
    last_ts = None
    win_t, win_n = t0, 0
    latest = None

    sys.stderr.write(f"已连接 {args.port} @ {args.baud}，等待 0x91 数据帧…\n")
    try:
        while True:
            now = time.time()
            if args.duration and (now - t0) >= args.duration:
                break

            chunk = ser.read(max(1, ser.in_waiting))
            if chunk:
                for frame in parser.feed(chunk):
                    s = ImuSample(frame)
                    latest = s
                    stats["frames"] += 1
                    win_n += 1

                    if last_ts is not None and s.ts - last_ts > 11:
                        stats["gaps"] += 1
                    last_ts = s.ts

                    if args.raw:
                        print(frame.hex(" "))
                    if csv_f:
                        csv_f.write(
                            f"{time.time():.3f},{s.ts},{s.pps_sync_ms},{s.temp},"
                            f"{s.prs:.6f},"
                            f"{s.acc[0]:.6f},{s.acc[1]:.6f},{s.acc[2]:.6f},"
                            f"{s.gyr[0]:.6f},{s.gyr[1]:.6f},{s.gyr[2]:.6f},"
                            f"{s.mag[0]:.6f},{s.mag[1]:.6f},{s.mag[2]:.6f},"
                            f"{s.roll:.6f},{s.pitch:.6f},{s.yaw:.6f},"
                            f"{s.quat[0]:.7f},{s.quat[1]:.7f},"
                            f"{s.quat[2]:.7f},{s.quat[3]:.7f}\n")

            stats["bad"] = parser.bad
            stats["bad_crc"] = parser.bad_crc

            if now - win_t >= 1.0:                      # 1 秒滑动窗帧率
                stats["hz"] = win_n / (now - win_t)
                win_t, win_n = now, 0

            if latest is not None and (now - last_draw) >= 1.0 / args.hz:
                last_draw = now
                if args.raw:
                    sys.stderr.write(f"\r帧率 {stats['hz']:6.1f} Hz  "
                                     f"已收 {stats['frames']} 帧  "
                                     f"CRC错误 {stats['bad_crc']}")
                    sys.stderr.flush()
                elif tty:
                    sys.stdout.write("\033[H\033[J"
                                     + render(latest, args, stats, now - t0) + "\n")
                    sys.stdout.flush()
                else:
                    print(render_line(latest, args), flush=True)

    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if csv_f:
            csv_f.close()
        dt = time.time() - t0
        print(f"\n已停止。共接收 {stats['frames']} 帧，耗时 {dt:.1f}s，"
              f"平均 {stats['frames'] / dt if dt else 0:.1f} Hz，"
              f"CRC错误 {parser.bad_crc}，解析错误 {parser.bad}，"
              f"时间戳丢帧 {stats['gaps']}。"
              + (f" CSV 已保存到 {args.csv}" if args.csv else ""))


if __name__ == "__main__":
    main()
