#!/usr/bin/env python3
"""
往复运动测试 — 方法1：2-Link Planar IK（近似3D）
Requires: pip install feetech-servo-sdk

策略：
  pan  = atan2(y, x)           底座转到目标方向
  r    = sqrt(x²+y²)           水平伸展量
  用 (r, z) 作为 2D IK 输入，近似实现 3D 定位

机械臂在 P1→P2→P3→P4→P1 之间循环，按 Enter 退出。

Usage:
  python move_2link.py [port] [--cal ../dfarm_calibration.json]
"""
import json, math, sys, time
from pathlib import Path
import scservo_sdk as scs

# ── 目标点 ────────────────────────────────────────────────────────────────────
WAYPOINTS = [
    (0.1981, -0.0122, 0.1583, "P1"),
    (0.2425, -0.0084, 0.0724, "P2"),
    (0.2063, -0.1909, 0.0203, "P3"),
    (0.1408,  0.0022, 0.3721, "P4"),
]

# ── 硬件 ──────────────────────────────────────────────────────────────────────
PORT    = "/dev/ttyACM0"
MOTORS  = {"shoulder_pan":1,"shoulder_lift":2,"elbow_flex":3,"wrist_flex":4,"wrist_roll":5,"gripper":6}
ARM_J   = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
ADDR    = {"torque":40,"accel":41,"goal":42,"lock":55,"pos":56,"P":21,"D":22,"I":23}
RES, CENTER, FREQ = 4096, 2048, 50
MOVE_DUR = 3.0  # 每段运动时间（秒）

# ── 2-link IK ─────────────────────────────────────────────────────────────────
L1 = 0.1159; L2 = 0.1350
T1 = math.atan2(0.028, 0.11257)
T2 = math.atan2(0.0052, 0.1349) + T1

def ik(x, y, z):
    pan = math.degrees(math.atan2(y, x))
    r   = math.hypot(x, y)
    px, py = r, z
    d = math.hypot(px, py)
    d = max(abs(L1-L2)*1.01, min((L1+L2)*0.99, d))
    if math.hypot(px,py) > 0 and d != math.hypot(px,py):
        s = d/math.hypot(px,py); px*=s; py*=s
    c2 = -(d**2-L1**2-L2**2)/(2*L1*L2); c2 = max(-1,min(1,c2))
    t2 = math.pi - math.acos(c2)
    t1 = math.atan2(py,px) + math.atan2(L2*math.sin(t2), L1+L2*math.cos(t2))
    sl = 90.0 - math.degrees(max(-0.1, min(3.45, t1+T1)))
    ef = math.degrees(max(-0.2, min(math.pi, t2+T2))) - 90.0
    wf = -sl - ef + 90.0
    return {"shoulder_pan":pan,"shoulder_lift":sl,"elbow_flex":ef,
            "wrist_flex":wf,"wrist_roll":0.0,"gripper":50.0}

# ── 校准 ──────────────────────────────────────────────────────────────────────
def load_cal(p):
    f = Path(p); return json.loads(f.read_text()) if f.exists() else None

def d2r(deg, m, cal):
    if cal and m in cal:
        c=cal[m]; return max(c["range_min"],min(c["range_max"],int(c["homing_raw"]+deg*RES/360)))
    return max(0,min(RES-1,int(CENTER+deg*RES/360)))

def r2d(raw, m, cal):
    if cal and m in cal: return (raw-cal[m]["homing_raw"])*360/RES
    return (raw-CENTER)*360/RES

# ── 电机 ──────────────────────────────────────────────────────────────────────
def open_arm(port_name, cal):
    port = scs.PortHandler(port_name)
    port.openPort(); port.setBaudRate(1_000_000)
    pkt = scs.PacketHandler(0)
    gw  = scs.GroupSyncWrite(port, pkt, ADDR["goal"], 2)
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port,mid,ADDR["torque"],0); pkt.write1ByteTxRx(port,mid,ADDR["lock"],0)
        pkt.write1ByteTxRx(port,mid,ADDR["P"],16);     pkt.write1ByteTxRx(port,mid,ADDR["D"],32)
        pkt.write1ByteTxRx(port,mid,ADDR["I"],0);      pkt.write1ByteTxRx(port,mid,ADDR["accel"],150)
        raw,_,_ = pkt.read2ByteTxRx(port,mid,ADDR["pos"])
        pkt.write2ByteTxRx(port,mid,ADDR["goal"],raw)
        pkt.write1ByteTxRx(port,mid,ADDR["torque"],1); pkt.write1ByteTxRx(port,mid,ADDR["lock"],1)
    return port, pkt, gw

def read_all(pkt, port, cal):
    return {n: r2d((lambda v,c,_: v if c==scs.COMM_SUCCESS else CENTER)(*pkt.read2ByteTxRx(port,mid,ADDR["pos"])),n,cal) for n,mid in MOTORS.items()}

def goto(pkt, port, gw, cal, tgt, dur=2.0):
    start=read_all(pkt,port,cal); g0=start.get("gripper",50)
    steps=max(1,int(dur*FREQ))
    for i in range(1,steps+1):
        t0=time.perf_counter(); a=i/steps
        gw.clearParam()
        for n in ARM_J:
            raw=d2r(start[n]+a*(tgt.get(n,start[n])-start[n]),n,cal)
            gw.addParam(MOTORS[n],[scs.SCS_LOBYTE(raw),scs.SCS_HIBYTE(raw)])
        gr=d2r(g0+a*(tgt.get("gripper",50)-g0),"gripper",cal)
        gw.addParam(MOTORS["gripper"],[scs.SCS_LOBYTE(gr),scs.SCS_HIBYTE(gr)])
        gw.txPacket()
        sl=1/FREQ-(time.perf_counter()-t0)
        if sl>0: time.sleep(sl)

# ── 主程序 ────────────────────────────────────────────────────────────────────
def main():
    args=sys.argv[1:]; port_name=PORT; cal_path="../dfarm_calibration.json"
    i=0
    while i<len(args):
        if args[i]=="--cal" and i+1<len(args): cal_path=args[i+1];i+=2
        elif not args[i].startswith("--"): port_name=args[i];i+=1
        else: i+=1

    cal = load_cal(cal_path)
    print(f"2-Link Planar IK 往复运动  port={port_name}  cal={'ok' if cal else 'none'}")
    print("按 Ctrl+C 停止\n")

    port, pkt, gw = open_arm(port_name, cal)
    try:
        idx = 0
        while True:
            x, y, z, label = WAYPOINTS[idx % len(WAYPOINTS)]
            sol = ik(x, y, z)
            print(f"  → {label}  ({x:+.4f}, {y:+.4f}, {z:+.4f})  "
                  f"pan={sol['shoulder_pan']:+.1f}° lift={sol['shoulder_lift']:+.1f}° ef={sol['elbow_flex']:+.1f}°")
            goto(pkt, port, gw, cal, sol, dur=MOVE_DUR)
            time.sleep(0.5)
            idx += 1
    except KeyboardInterrupt:
        print("\n停止。")
    finally:
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port,mid,ADDR["torque"],0)
            pkt.write1ByteTxRx(port,mid,ADDR["lock"],0)
        port.closePort()

if __name__=="__main__": main()
