#!/usr/bin/env python3
"""
Pinocchio IK 三点循环运动：Ready → Pos1 → Ready → Pos2 → Ready → Pos1 → ...
直接复用 7_dfarm_ik_xyz.py 里的 SO101Kinematics 类。
Requires: pip install pin feetech-servo-sdk

Usage:
  python move_3points.py [port] [--cal ../dfarm_calibration.json] [--urdf ../so101_new_calib.urdf]
"""
import json, sys, time
from pathlib import Path
import numpy as np
import scservo_sdk as scs

# 直接导入 7_dfarm_ik_xyz.py 里的运动学类，保证 IK 逻辑完全一致
sys.path.insert(0, str(Path(__file__).parent.parent))
from importlib import util as _ilu
_spec = _ilu.spec_from_file_location("dfarm_ik", Path(__file__).parent.parent / "7_dfarm_ik_xyz.py")
_mod  = _ilu.module_from_spec(_spec); _spec.loader.exec_module(_mod)
SO101Kinematics = _mod.SO101Kinematics
load_cal  = _mod.load_cal
deg2raw   = _mod.deg2raw
raw2deg   = _mod.raw2deg

READY = (0.1907, -0.0115, 0.1915, "Ready")
POS1  = (0.1860, -0.2017, 0.0070, "Pos1")
POS2  = (0.2820,  0.0218, 0.0069, "Pos2")
# 循环序列：Ready → Pos1 → Ready → Pos2 → Ready → Pos1 → ...
SEQUENCE = [READY, POS1, READY, POS2]

PORT     = "/dev/ttyACM0"
URDF_DEF = str(Path(__file__).parent.parent / "so101_new_calib.urdf")
MOTORS   = {"shoulder_pan":1,"shoulder_lift":2,"elbow_flex":3,"wrist_flex":4,"wrist_roll":5,"gripper":6}
ARM_J    = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
ADDR     = {"torque":40,"accel":41,"goal":42,"lock":55,"pos":56,"P":21,"D":22,"I":23}
RES, CENTER, FREQ = 4096, 2048, 50
MOVE_DUR = 3.0   # 每段运动时间（秒）

# ── 校准 + 电机 ────────────────────────────────────────────────────────────────

# load_cal / deg2raw / raw2deg 已从 7_dfarm_ik_xyz 导入

def load_cal(p):
    f = Path(p); return json.loads(f.read_text()) if f.exists() else None

def d2r(deg, m, cal):
    if cal and m in cal:
        c = cal[m]; return max(c["range_min"], min(c["range_max"], int(c["homing_raw"] + deg * RES / 360)))
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360)))

def r2d(raw, m, cal):
    if cal and m in cal: return (raw - cal[m]["homing_raw"]) * 360 / RES
    return (raw - CENTER) * 360 / RES

def open_arm(port_name, cal):
    port = scs.PortHandler(port_name); port.openPort(); port.setBaudRate(1_000_000)
    pkt  = scs.PacketHandler(0); gw = scs.GroupSyncWrite(port, pkt, ADDR["goal"], 2)
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0); pkt.write1ByteTxRx(port, mid, ADDR["lock"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"], 16);     pkt.write1ByteTxRx(port, mid, ADDR["D"], 32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"], 0);      pkt.write1ByteTxRx(port, mid, ADDR["accel"], 150)
        raw, _, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
        pkt.write2ByteTxRx(port, mid, ADDR["goal"], raw)
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1); pkt.write1ByteTxRx(port, mid, ADDR["lock"], 1)
    return port, pkt, gw

def read_all(pkt, port, cal):
    return {n: r2d((lambda v,c,_: v if c==scs.COMM_SUCCESS else CENTER)(*pkt.read2ByteTxRx(port, mid, ADDR["pos"])), n, cal)
            for n, mid in MOTORS.items()}

def goto(pkt, port, gw, cal, tgt, dur=2.0):
    start = read_all(pkt, port, cal); g0 = start.get("gripper", 50)
    steps = max(1, int(dur * FREQ))
    for i in range(1, steps + 1):
        t0 = time.perf_counter(); a = i / steps; gw.clearParam()
        for n in ARM_J:
            v = d2r(start[n] + a * (tgt.get(n, start[n]) - start[n]), n, cal)
            gw.addParam(MOTORS[n], [scs.SCS_LOBYTE(v), scs.SCS_HIBYTE(v)])
        gr = d2r(g0 + a * (tgt.get("gripper", 50) - g0), "gripper", cal)
        gw.addParam(MOTORS["gripper"], [scs.SCS_LOBYTE(gr), scs.SCS_HIBYTE(gr)])
        gw.txPacket()
        sl = 1 / FREQ - (time.perf_counter() - t0)
        if sl > 0: time.sleep(sl)

# ── 主程序 ────────────────────────────────────────────────────────────────────

def main():
    args = sys.argv[1:]; port_name = PORT; cal_path = "../dfarm_calibration.json"; urdf = URDF_DEF
    i = 0
    while i < len(args):
        if args[i] == "--cal"  and i+1 < len(args): cal_path = args[i+1]; i += 2
        elif args[i] == "--urdf" and i+1 < len(args): urdf = args[i+1]; i += 2
        elif not args[i].startswith("--"): port_name = args[i]; i += 1
        else: i += 1

    cal = load_cal(Path(cal_path))
    print(f"三点循环  port={port_name}  urdf={Path(urdf).name}  cal={'ok' if cal else 'none'}")
    print(f"序列: Ready → Pos1 → Ready → Pos2 → Ready → Pos1 → ...")
    print("按 Ctrl+C 停止\n")

    try: solver = SO101Kinematics(urdf)
    except ImportError: print("ERROR: pip install pin"); sys.exit(1)

    port, pkt, gw = open_arm(port_name, cal)
    try:
        # 预热：对每个唯一目标点先从当前位置求一次解，缓存关节角
        # 后续每次求同一个点都用上次成功的解作为初始猜测，避免局部最优
        print("预热 IK 初始猜测...")
        cur = read_all(pkt, port, cal)
        ik_cache: dict[str, dict] = {}
        import math
        seen = {}
        for x, y, z, label in SEQUENCE:
            key = label
            if key not in seen:
                seen[key] = True
                sol = solver.inverse_kinematics(cur, (x, y, z))
                fx, fy, fz = solver.get_ee_xyz(sol)
                err = math.sqrt((fx-x)**2+(fy-y)**2+(fz-z)**2)*1000
                ik_cache[key] = sol
                print(f"  {label}: IK_err={err:.1f}mm  "
                      f"lift={sol['shoulder_lift']:+.1f}° ef={sol['elbow_flex']:+.1f}°")
                cur = sol  # 用本次解作为下一个点的初始猜测
        print()

        idx = 0
        while True:
            x, y, z, label = SEQUENCE[idx % len(SEQUENCE)]
            # 用缓存的上次解作为初始猜测（热启动）
            warm_start = ik_cache.get(label, read_all(pkt, port, cal))
            t0  = time.perf_counter()
            sol = solver.inverse_kinematics(warm_start, (x, y, z))
            ms  = (time.perf_counter() - t0) * 1000
            sol["gripper"] = read_all(pkt, port, cal).get("gripper", 50.0)
            # 更新缓存
            ik_cache[label] = sol
            fx, fy, fz = solver.get_ee_xyz(sol)
            err_mm = math.sqrt((fx-x)**2+(fy-y)**2+(fz-z)**2)*1000
            print(f"  → {label:<6} ({x:+.4f}, {y:+.4f}, {z:+.4f})  "
                  f"solve={ms:.1f}ms  IK_err={err_mm:.1f}mm  "
                  f"pan={sol['shoulder_pan']:+.1f}° lift={sol['shoulder_lift']:+.1f}° ef={sol['elbow_flex']:+.1f}°")
            goto(pkt, port, gw, cal, sol, dur=MOVE_DUR)
            time.sleep(0.5)
            idx += 1
    except KeyboardInterrupt:
        print("\n停止。")
    finally:
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"], 0)
        port.closePort()

if __name__ == "__main__": main()
