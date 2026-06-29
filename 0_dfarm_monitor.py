#!/usr/bin/env python3
"""
SO-100 real-time joint angle monitor — no lerobot dependency.
Requires: pip install feetech-servo-sdk

Continuously reads and displays all 6 joint positions at ~10Hz.
Torque is never enabled — arm can be moved freely by hand.
Press Ctrl+C to exit.

Usage:
  python 0_dfarm_monitor.py [port] [--cal dfarm_calibration.json]
"""

import json
import sys
import time
from pathlib import Path

import scservo_sdk as scs

PORT = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL_VERSION = 0

MOTORS = {
    "shoulder_pan":  1,
    "shoulder_lift": 2,
    "elbow_flex":    3,
    "wrist_flex":    4,
    "wrist_roll":    5,
    "gripper":       6,
}

ADDR_PRESENT_POSITION = 56
RESOLUTION = 4096
CENTER = 2048


def load_calibration(cal_path: str | None) -> dict | None:
    path = Path(cal_path) if cal_path else Path("dfarm_calibration.json")
    if not path.exists():
        return None
    with open(path) as f:
        return json.load(f)


def raw_to_deg(raw: int, motor: str, cal: dict | None) -> float:
    if cal and motor in cal:
        return (raw - cal[motor]["homing_raw"]) * 360.0 / RESOLUTION
    return (raw - CENTER) * 360.0 / RESOLUTION


def main():
    args = sys.argv[1:]
    port_name = PORT
    cal_path = None
    i = 0
    while i < len(args):
        if args[i] == "--cal" and i + 1 < len(args):
            cal_path = args[i + 1]
            i += 2
        elif not args[i].startswith("--"):
            port_name = args[i]
            i += 1
        else:
            i += 1

    cal = load_calibration(cal_path)
    deg_label = "校准角度" if cal else "角度(raw中心)"

    print(f"SO-100 Joint Monitor — {port_name} @ {BAUDRATE} baud")
    if cal:
        print("已加载校准文件，显示相对于校准零点的角度。")
    else:
        print("未找到校准文件，显示相对于 raw=2048 的角度。")
        print("提示：使用 --cal dfarm_calibration.json 加载校准文件。")

    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: Cannot open port {port_name}")
        sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL_VERSION)

    for name, mid in MOTORS.items():
        pos, comm, _ = pkt.read2ByteTxRx(port, mid, ADDR_PRESENT_POSITION)
        if comm != scs.COMM_SUCCESS:
            print(f"  ERROR: '{name}' (id={mid}) not responding!")
            port.closePort()
            sys.exit(1)

    print(f"\n{'关节':<15} {'Raw':>6} {deg_label:>12}")
    print("-" * 38)

    try:
        while True:
            sys.stdout.write(f"\033[{len(MOTORS)}A")
            for name, mid in MOTORS.items():
                pos, comm, _ = pkt.read2ByteTxRx(port, mid, ADDR_PRESENT_POSITION)
                if comm == scs.COMM_SUCCESS:
                    deg = raw_to_deg(pos, name, cal)
                    sys.stdout.write(f"{name:<15} {pos:>6} {deg:>+11.1f}°\n")
                else:
                    sys.stdout.write(f"{name:<15} {'ERR':>6} {'---':>12}\n")
            sys.stdout.flush()
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\nDone.")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
