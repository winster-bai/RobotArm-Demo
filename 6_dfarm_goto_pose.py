#!/usr/bin/env python3
"""
SO-100 Named Pose Controller — smooth interpolated motion between preset poses.
Requires: pip install feetech-servo-sdk

支持校准文件（dfarm_calibration.json），校准后角度 0° 对应各关节的物理零点，
换机械臂只需重新运行 6_dfarm_calibrate.py，无需修改姿态定义。

Commands:
  list             show all available poses
  go <name>        move to named pose (smooth interpolation)
  go <name> <sec>  move with custom duration in seconds
  save <name>      save current joint positions as a new pose (in calibrated degrees)
  pos              print current joint positions
  q / Ctrl+C       quit

Usage:
  python 5_dfarm_goto_pose.py [port] [--cal dfarm_calibration.json]
"""

import json
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
FREQ   = 50   # interpolation Hz

DEFAULT_CAL_FILE = "dfarm_calibration.json"


# ── 校准辅助 ──────────────────────────────────────────────────────────────────

def load_calibration(cal_path: str | None) -> dict | None:
    """加载校准文件，返回 None 表示不使用校准（退回旧行为）。"""
    path = Path(cal_path) if cal_path else Path(DEFAULT_CAL_FILE)
    if not path.exists():
        return None
    with open(path) as f:
        cal = json.load(f)
    print(f"已加载校准文件: {path}")
    return cal


def cal_deg2raw(deg: float, motor: str, cal: dict | None) -> int:
    """
    将"语义角度"（相对于校准零点的度数）转换为 raw 值。
    无校准时退回原始行为（CENTER 为零点）。
    """
    if cal and motor in cal:
        c = cal[motor]
        raw = int(c["homing_raw"] + deg * RES / 360.0)
        return max(c["range_min"], min(c["range_max"], raw))
    # 无校准：原始行为
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360.0)))


def cal_raw2deg(raw: int, motor: str, cal: dict | None) -> float:
    """将 raw 值转换回语义角度。"""
    if cal and motor in cal:
        return (raw - cal[motor]["homing_raw"]) * 360.0 / RES
    return (raw - CENTER) * 360.0 / RES

# ── Preset poses (degrees, relative to calibrated homing=0°) ─────────────────
# gripper 约定（基于 dfarm_calibration.json）：homing_raw 在全开端，
# 因此 gripper≈0° 表示完全张开，角度越大越闭合，约 323° 完全闭合。
POSES: dict[str, dict[str, float]] = {
    "home": {
        "shoulder_pan":   0.0,
        "shoulder_lift":  0.0,
        "elbow_flex":     0.0,
        "wrist_flex":     0.0,
        "wrist_roll":     0.0,
        "gripper":        0.0,
    },
    "zero": {
        "shoulder_pan":   0.0,
        "shoulder_lift": -104.0,
        "elbow_flex":     85.0,
        "wrist_flex":     65.0,
        "wrist_roll":     0.0,
        "gripper":        1.0,
    },
    "ready": {
        "shoulder_pan":   0.0,
        "shoulder_lift": -45.0,
        "elbow_flex":     9.0,
        "wrist_flex":     93.0,
        "wrist_roll":     0.0,
        "gripper":        0.0,
    },
}


def read_pos(pkt, port, mid) -> int:
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else CENTER


def read_all_cal_deg(pkt, port, cal: dict | None) -> dict[str, float]:
    """读取所有关节当前位置，返回校准后的语义角度。"""
    return {name: cal_raw2deg(read_pos(pkt, port, mid), name, cal) for name, mid in MOTORS.items()}


def goto_pose(pkt, port, group_write, target_deg: dict[str, float], duration: float, cal: dict | None):
    """从当前位置线性插值到目标姿态（语义角度），duration 秒内完成。"""
    start_deg = read_all_cal_deg(pkt, port, cal)
    steps = max(1, int(duration * FREQ))
    dt = 1.0 / FREQ

    for step in range(1, steps + 1):
        t0 = time.perf_counter()
        alpha = step / steps

        group_write.clearParam()
        for name, mid in MOTORS.items():
            tgt = target_deg.get(name, start_deg[name])
            deg = start_deg[name] + alpha * (tgt - start_deg[name])
            raw = cal_deg2raw(deg, name, cal)
            group_write.addParam(mid, [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])
        group_write.txPacket()

        bar_w = 30
        filled = int(alpha * bar_w)
        bar = "█" * filled + "░" * (bar_w - filled)
        sys.stdout.write(f"\r  [{bar}] {alpha*100:>5.1f}%  step {step}/{steps}")
        sys.stdout.flush()

        sl = dt - (time.perf_counter() - t0)
        if sl > 0:
            time.sleep(sl)

    print()


def print_positions(pkt, port, cal: dict | None):
    label = "校准角度" if cal else "角度(raw中心)"
    print(f"\n{'关节':<15} {label:>12}")
    print("-" * 30)
    for name, mid in MOTORS.items():
        deg = cal_raw2deg(read_pos(pkt, port, mid), name, cal)
        print(f"{name:<15} {deg:>+11.1f}°")
    print()


def main():
    # 解析参数：[port] [--cal <file>]
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
    if cal is None:
        print("未找到校准文件，使用原始角度模式（CENTER=2048 为零点）。")
        print("提示：运行 python 6_dfarm_calibrate.py 生成校准文件。\n")

    print(f"SO-100 Named Pose Controller — {port_name}\n")

    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: cannot open {port_name}"); sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    for name, mid in MOTORS.items():
        if read_pos(pkt, port, mid) is None:
            print(f"ERROR: '{name}' (id={mid}) not responding")
            port.closePort(); sys.exit(1)

    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"],      16)
        pkt.write1ByteTxRx(port, mid, ADDR["D"],      32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"],       0)
        pkt.write1ByteTxRx(port, mid, ADDR["accel"], 150)
        raw = read_pos(pkt, port, mid)
        pkt.write2ByteTxRx(port, mid, ADDR["goal"], raw)
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   1)

    group_write = scs.GroupSyncWrite(port, pkt, ADDR["goal"], 2)

    print("Commands: list  |  go <pose> [sec]  |  save <name>  |  pos  |  q")
    print(f"Poses: {', '.join(POSES)}\n")

    try:
        while True:
            try:
                line = input(">> ").strip()
            except EOFError:
                break

            parts = line.split()
            if not parts:
                continue
            cmd = parts[0].lower()

            if cmd == 'q':
                break

            elif cmd == 'list':
                print(f"\n{'Pose':<15} " + "  ".join(f"{n[:6]:>7}" for n in MOTORS))
                print("-" * (16 + 9 * len(MOTORS)))
                for pname, angles in POSES.items():
                    vals = "  ".join(f"{angles.get(n, 0.0):>+6.1f}°" for n in MOTORS)
                    print(f"{pname:<15} {vals}")
                print()

            elif cmd == 'go' and len(parts) >= 2:
                pname = parts[1]
                duration = float(parts[2]) if len(parts) >= 3 else 2.0
                if pname not in POSES:
                    print(f"  Unknown pose '{pname}'. Try: {', '.join(POSES)}")
                else:
                    print(f"  Moving to '{pname}' over {duration:.1f}s ...")
                    goto_pose(pkt, port, group_write, POSES[pname], duration, cal)
                    print(f"  Reached '{pname}'.")

            elif cmd == 'save' and len(parts) == 2:
                pname = parts[1]
                # 保存为校准语义角度，换机械臂后仍然有意义
                POSES[pname] = read_all_cal_deg(pkt, port, cal)
                print(f"  Saved '{pname}': { {k: f'{v:+.1f}°' for k, v in POSES[pname].items()} }")

            elif cmd == 'pos':
                print_positions(pkt, port, cal)

            else:
                print("  Unknown command. Try: list  go <pose> [sec]  save <name>  pos  q")

    except KeyboardInterrupt:
        pass
    finally:
        group_write.clearParam()
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        port.closePort()
        print("\nDone.")


if __name__ == "__main__":
    main()
