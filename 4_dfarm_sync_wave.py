#!/usr/bin/env python3
"""
SO-100 Sync Wave Demo — GroupSyncWrite sine-wave motion on all joints.
Requires: pip install feetech-servo-sdk

Each joint oscillates with a configurable amplitude and phase offset,
demonstrating how to drive all 6 servos in a single bus transaction per tick.

Press Ctrl+C to stop and release torque.

支持校准文件（dfarm_calibration.json）：波形以各关节校准零点为中心，
并自动钳在 [range_min, range_max] 内摆动，避免撞到机械限位。

Usage: python 4_so100_sync_wave.py [port] [--cal dfarm_calibration.json]
"""

import json
import math
import sys
import time
from pathlib import Path

import scservo_sdk as scs

PORT     = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL = 0

MOTORS = {
    "shoulder_pan":  1,
    "shoulder_lift": 2,
    "elbow_flex":    3,
    "wrist_flex":    4,
    "wrist_roll":    5,
    "gripper":       6,
}

ADDR = {"torque": 40, "accel": 41, "goal": 42, "lock": 55, "pos": 56,
        "P": 21, "D": 22, "I": 23}

RES    = 4096
CENTER = 2048
FREQ   = 50   # control loop Hz

# Wave parameters per joint: (amplitude_deg, phase_offset_rad, centre_deg)
WAVE = {
    "shoulder_pan":   (20.0, 0.0,              0.0),
    "shoulder_lift":  (15.0, math.pi / 3,      10.0),
    "elbow_flex":     (20.0, 2 * math.pi / 3, -10.0),
    "wrist_flex":     (15.0, math.pi,           0.0),
    "wrist_roll":     (25.0, 4 * math.pi / 3,  0.0),
    "gripper":        (30.0, 5 * math.pi / 3,  0.0),
}
WAVE_FREQ = 0.4   # oscillation frequency in Hz

DEFAULT_CAL_FILE = "dfarm_calibration.json"


def load_calibration(cal_path: str | None) -> dict | None:
    """加载校准文件，返回 None 表示退回原始行为（CENTER 为零点，全程钳位）。"""
    path = Path(cal_path) if cal_path else Path(DEFAULT_CAL_FILE)
    if not path.exists():
        return None
    cal = json.loads(path.read_text())
    print(f"已加载校准文件: {path}")
    return cal


def deg2raw(d: float, motor: str, cal: dict | None) -> int:
    """语义角度 → raw。有校准时以 homing_raw 为零点并钳在 [range_min, range_max]。"""
    if cal and motor in cal:
        c = cal[motor]
        raw = int(c["homing_raw"] + d * RES / 360.0)
        return max(c["range_min"], min(c["range_max"], raw))
    return max(0, min(RES - 1, int(CENTER + d * RES / 360.0)))


def raw2deg(r: int, motor: str, cal: dict | None) -> float:
    if cal and motor in cal:
        return (r - cal[motor]["homing_raw"]) * 360.0 / RES
    return (r - CENTER) * 360.0 / RES

def read_pos(pkt, port, mid):
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else None


def main():
    args = sys.argv[1:]
    port_name = PORT
    cal_path = None
    i = 0
    while i < len(args):
        if args[i] == "--cal" and i + 1 < len(args):
            cal_path = args[i + 1]; i += 2
        elif not args[i].startswith("--"):
            port_name = args[i]; i += 1
        else:
            i += 1

    cal = load_calibration(cal_path)
    if cal is None:
        print("未找到校准文件，使用原始角度模式（CENTER=2048 为零点，全程钳位）。")
    print(f"SO-100 Sync Wave — {port_name}")

    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: cannot open {port_name}"); sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    # Verify all motors
    for name, mid in MOTORS.items():
        if read_pos(pkt, port, mid) is None:
            print(f"ERROR: '{name}' (id={mid}) not responding")
            port.closePort(); sys.exit(1)

    # Configure PID + enable torque
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"],      16)
        pkt.write1ByteTxRx(port, mid, ADDR["D"],      32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"],       0)
        pkt.write1ByteTxRx(port, mid, ADDR["accel"], 150)

    # Set goal = current before enabling torque
    for name, mid in MOTORS.items():
        raw = read_pos(pkt, port, mid) or CENTER
        pkt.write2ByteTxRx(port, mid, ADDR["goal"], raw)
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   1)

    # Build GroupSyncWrite for goal position (2 bytes)
    group_write = scs.GroupSyncWrite(port, pkt, ADDR["goal"], 2)

    # 平滑过渡：从当前姿态线性插值到波形起点（phase=0），避免上电瞬间窜动
    start_raw = {name: (read_pos(pkt, port, mid) or CENTER) for name, mid in MOTORS.items()}
    wave_start = {}
    for name in MOTORS:
        amp, offset, centre = WAVE[name]
        wave_start[name] = deg2raw(centre + amp * math.sin(offset), name, cal)

    ramp_steps = int(1.5 * FREQ)  # 1.5s 过渡
    for step in range(1, ramp_steps + 1):
        t0 = time.perf_counter()
        a = step / ramp_steps
        group_write.clearParam()
        for name, mid in MOTORS.items():
            raw = int(start_raw[name] + a * (wave_start[name] - start_raw[name]))
            group_write.addParam(mid, [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])
        group_write.txPacket()
        sl = 1.0 / FREQ - (time.perf_counter() - t0)
        if sl > 0:
            time.sleep(sl)

    print("Running wave... Ctrl+C to stop.\n")
    print(f"{'Joint':<15} {'Target':>8}")
    print("-" * 26)

    dt      = 1.0 / FREQ
    t_start = time.perf_counter()

    try:
        while True:
            t0 = time.perf_counter()
            elapsed = t0 - t_start
            phase   = 2 * math.pi * WAVE_FREQ * elapsed

            group_write.clearParam()
            for name, mid in MOTORS.items():
                amp, offset, centre = WAVE[name]
                deg = centre + amp * math.sin(phase + offset)
                raw = deg2raw(deg, name, cal)
                param = [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)]
                group_write.addParam(mid, param)

            group_write.txPacket()

            # status line
            sys.stdout.write(f"\033[{len(MOTORS)}A")
            for name, mid in MOTORS.items():
                amp, offset, centre = WAVE[name]
                deg = centre + amp * math.sin(phase + offset)
                sys.stdout.write(f"{name:<15} {deg:>+7.1f}°\n")
            sys.stdout.flush()

            sl = dt - (time.perf_counter() - t0)
            if sl > 0:
                time.sleep(sl)

    except KeyboardInterrupt:
        pass
    finally:
        group_write.clearParam()
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        port.closePort()
        print("\n\nDone.")


if __name__ == "__main__":
    main()
