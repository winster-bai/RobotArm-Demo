#!/usr/bin/env python3
"""
SO-100 机械臂校准工具 — 仿照 lerobot 校准流程。
Requires: pip install feetech-servo-sdk

流程：
  1. 释放扭矩，手动把机械臂摆到"中位参考姿态"（通常是竖直向上）
  2. 按 Enter 记录各关节的 homing_raw（零点原始值）
  3. 手动把每个关节拨到运动范围两端，程序实时记录 min/max
  4. 按 Enter 完成，保存 dfarm_calibration.json

校准文件格式：
  {
    "shoulder_pan":  {"id": 1, "homing_raw": 2048, "range_min": 512,  "range_max": 3584},
    "shoulder_lift": {"id": 2, "homing_raw": 2100, "range_min": 1024, "range_max": 3072},
    ...
  }

使用校准后，角度 0° 对应 homing_raw，正负角度按 4096步/360° 换算，
并自动限幅在 [range_min, range_max] 内。

Usage:
  python 6_dfarm_calibrate.py [port] [--output dfarm_calibration.json]
"""

import json
import sys
import time
from pathlib import Path

import scservo_sdk as scs

PORT = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL = 0

MOTORS: dict[str, int] = {
    "shoulder_pan":  1,
    "shoulder_lift": 2,
    "elbow_flex":    3,
    "wrist_flex":    4,
    "wrist_roll":    5,
    "gripper":       6,
}

# wrist_roll 可以整圈旋转，不需要记录范围，直接用全范围
FULL_RANGE_MOTORS = {"wrist_roll"}

ADDR = {"torque": 40, "lock": 55, "pos": 56}
RES = 4096


def read_pos(pkt, port, mid: int) -> int | None:
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else None


def read_all(pkt, port) -> dict[str, int]:
    result = {}
    for name, mid in MOTORS.items():
        v = read_pos(pkt, port, mid)
        result[name] = v if v is not None else 2048
    return result


def release_torque(pkt, port):
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"], 0)


def print_positions(positions: dict[str, int], mins: dict[str, int], maxes: dict[str, int]):
    """打印实时位置表，可被覆写。"""
    print(f"\n{'关节':<15} {'当前(raw)':>10} {'最小':>8} {'最大':>8}")
    print("-" * 45)
    for name in MOTORS:
        pos = positions.get(name, 0)
        lo = mins.get(name, pos)
        hi = maxes.get(name, pos)
        print(f"{name:<15} {pos:>10} {lo:>8} {hi:>8}")


def step1_homing(pkt, port) -> dict[str, int]:
    """步骤1：记录零点 raw 值。"""
    print("\n" + "=" * 55)
    print("步骤 1 / 2 — 设置零点（Homing）")
    print("=" * 55)
    print("扭矩已释放，请手动把机械臂摆到你的'中位参考姿态'")
    print("（例如：竖直向上、或你定义的 home 姿态）")
    print()
    input("摆好后按 Enter 记录零点...")

    homing = read_all(pkt, port)
    print("\n已记录零点：")
    for name, raw in homing.items():
        print(f"  {name:<15} raw={raw}")
    return homing


def step2_ranges(pkt, port) -> tuple[dict[str, int], dict[str, int]]:
    """步骤2：实时记录各关节运动范围。"""
    print("\n" + "=" * 55)
    print("步骤 2 / 2 — 记录运动范围")
    print("=" * 55)
    print(f"请手动把每个关节（除 {FULL_RANGE_MOTORS} 外）拨到两端极限。")
    print("程序实时记录 min/max，完成后按 Enter。\n")

    start = read_all(pkt, port)
    mins = start.copy()
    maxes = start.copy()

    # 非阻塞 Enter 检测
    import select

    print("正在记录... 按 Enter 停止")
    while True:
        positions = read_all(pkt, port)
        for name in MOTORS:
            if name in FULL_RANGE_MOTORS:
                continue
            mins[name] = min(positions[name], mins[name])
            maxes[name] = max(positions[name], maxes[name])

        # 覆写输出
        sys.stdout.write(f"\033[{len(MOTORS) + 3}A")
        print_positions(positions, mins, maxes)
        sys.stdout.flush()

        # 检查 Enter（非阻塞）
        if select.select([sys.stdin], [], [], 0.1)[0]:
            sys.stdin.readline()
            break

    # wrist_roll 全范围
    for name in FULL_RANGE_MOTORS:
        mins[name] = 0
        maxes[name] = RES - 1

    return mins, maxes


def save_calibration(
    homing: dict[str, int],
    mins: dict[str, int],
    maxes: dict[str, int],
    output_path: Path,
):
    calibration = {}
    for name, mid in MOTORS.items():
        calibration[name] = {
            "id": mid,
            "homing_raw": homing[name],
            "range_min": mins[name],
            "range_max": maxes[name],
        }

    output_path.write_text(json.dumps(calibration, indent=2, ensure_ascii=False))
    print(f"\n校准数据已保存到: {output_path}")
    print("\n校准摘要：")
    print(f"{'关节':<15} {'零点(raw)':>10} {'范围 min':>10} {'范围 max':>10} {'范围(°)':>10}")
    print("-" * 60)
    for name, cal in calibration.items():
        span_deg = (cal["range_max"] - cal["range_min"]) * 360.0 / RES
        print(
            f"{name:<15} {cal['homing_raw']:>10} {cal['range_min']:>10} "
            f"{cal['range_max']:>10} {span_deg:>9.1f}°"
        )


def main():
    args = sys.argv[1:]
    port_name = PORT
    output_path = Path("dfarm_calibration.json")

    for i, arg in enumerate(args):
        if arg == "--output" and i + 1 < len(args):
            output_path = Path(args[i + 1])
        elif not arg.startswith("--"):
            port_name = arg

    print(f"SO-100 校准工具 — {port_name}")

    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: 无法打开串口 {port_name}")
        sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    # 验证所有电机在线
    print("检查电机连接...")
    for name, mid in MOTORS.items():
        v = read_pos(pkt, port, mid)
        if v is None:
            print(f"ERROR: '{name}' (id={mid}) 无响应，请检查连接")
            port.closePort()
            sys.exit(1)
    print("所有电机在线 ✓")

    release_torque(pkt, port)

    try:
        homing = step1_homing(pkt, port)
        # 打印空行占位，供 step2 覆写
        print("\n" * (len(MOTORS) + 3))
        mins, maxes = step2_ranges(pkt, port)
        save_calibration(homing, mins, maxes, output_path)

    except KeyboardInterrupt:
        print("\n\n校准已取消。")
    finally:
        release_torque(pkt, port)
        port.closePort()
        print("完成。")


if __name__ == "__main__":
    main()
