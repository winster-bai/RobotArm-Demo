#!/usr/bin/env python3
"""
往复运动测试 — 方法3：IKPy
Requires: pip install ikpy feetech-servo-sdk

Usage:
  python move_ikpy.py [port] [--cal ../dfarm_calibration.json] [--urdf ../so101_new_calib.urdf]
"""
import json, math, sys, time
from pathlib import Path
import numpy as np
import scservo_sdk as scs

WAYPOINTS = [
    (0.1981, -0.0122, 0.1583, "P1"),
    (0.2425, -0.0084, 0.0724, "P2"),
    (0.2063, -0.1909, 0.0203, "P3"),
    (0.1408,  0.0022, 0.3721, "P4"),
]

PORT     = "/dev/ttyACM0"
URDF_DEF = str(Path(__file__).parent.parent / "so101_new_calib.urdf")
MOTORS   = {"shoulder_pan":1,"shoulder_lift":2,"elbow_flex":3,"wrist_flex":4,"wrist_roll":5,"gripper":6}
ARM_J    = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
ADDR     = {"torque":40,"accel":41,"goal":42,"lock":55,"pos":56,"P":21,"D":22,"I":23}
RES, CENTER, FREQ = 4096, 2048, 50
MOVE_DUR = 3.0

# ── IKPy ──────────────────────────────────────────────────────────────────────
class IKPyIK:
    def __init__(self, urdf):
        from ikpy.chain import Chain
        # SO-100 URDF 有 7 个 links，index 1-5 是5个臂关节
        self.chain = Chain.from_urdf_file(
            urdf,
            active_links_mask=[False, True, True, True, True, True, False],
        )
        # 关节索引（按 URDF 中的顺序对应 ARM_J）
        self._idx = {n: i+1 for i, n in enumerate(ARM_J)}

    def solve(self, cur_deg, tx, ty, tz):
        n  = len(self.chain.links)
        q0 = np.zeros(n)
        for name, idx in self._idx.items():
            q0[idx] = math.radians(cur_deg.get(name, 0.0))
        q_sol = self.chain.inverse_kinematics(target_position=[tx, ty, tz], initial_position=q0)
        result = {name: math.degrees(q_sol[idx]) for name, idx in self._idx.items()}
        result["gripper"] = 50.0
        return result

# ── 校准 + 电机 ────────────────────────────────────────────────────────────────
def load_cal(p):
    f=Path(p); return json.loads(f.read_text()) if f.exists() else None

def d2r(deg,m,cal):
    if cal and m in cal:
        c=cal[m]; return max(c["range_min"],min(c["range_max"],int(c["homing_raw"]+deg*RES/360)))
    return max(0,min(RES-1,int(CENTER+deg*RES/360)))

def r2d(raw,m,cal):
    if cal and m in cal: return (raw-cal[m]["homing_raw"])*360/RES
    return (raw-CENTER)*360/RES

def open_arm(port_name,cal):
    port=scs.PortHandler(port_name); port.openPort(); port.setBaudRate(1_000_000)
    pkt=scs.PacketHandler(0); gw=scs.GroupSyncWrite(port,pkt,ADDR["goal"],2)
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port,mid,ADDR["torque"],0); pkt.write1ByteTxRx(port,mid,ADDR["lock"],0)
        pkt.write1ByteTxRx(port,mid,ADDR["P"],16);     pkt.write1ByteTxRx(port,mid,ADDR["D"],32)
        pkt.write1ByteTxRx(port,mid,ADDR["I"],0);      pkt.write1ByteTxRx(port,mid,ADDR["accel"],150)
        raw,_,_=pkt.read2ByteTxRx(port,mid,ADDR["pos"])
        pkt.write2ByteTxRx(port,mid,ADDR["goal"],raw)
        pkt.write1ByteTxRx(port,mid,ADDR["torque"],1); pkt.write1ByteTxRx(port,mid,ADDR["lock"],1)
    return port,pkt,gw

def read_all(pkt,port,cal):
    return {n:r2d((lambda v,c,_:v if c==scs.COMM_SUCCESS else CENTER)(*pkt.read2ByteTxRx(port,mid,ADDR["pos"])),n,cal) for n,mid in MOTORS.items()}

def goto(pkt,port,gw,cal,tgt,dur=2.0):
    start=read_all(pkt,port,cal); g0=start.get("gripper",50)
    steps=max(1,int(dur*FREQ))
    for i in range(1,steps+1):
        t0=time.perf_counter(); a=i/steps; gw.clearParam()
        for n in ARM_J:
            v=d2r(start[n]+a*(tgt.get(n,start[n])-start[n]),n,cal)
            gw.addParam(MOTORS[n],[scs.SCS_LOBYTE(v),scs.SCS_HIBYTE(v)])
        gr=d2r(g0+a*(tgt.get("gripper",50)-g0),"gripper",cal)
        gw.addParam(MOTORS["gripper"],[scs.SCS_LOBYTE(gr),scs.SCS_HIBYTE(gr)])
        gw.txPacket()
        sl=1/FREQ-(time.perf_counter()-t0)
        if sl>0: time.sleep(sl)

def main():
    args=sys.argv[1:]; port_name=PORT; cal_path="../dfarm_calibration.json"; urdf=URDF_DEF
    i=0
    while i<len(args):
        if args[i]=="--cal" and i+1<len(args): cal_path=args[i+1];i+=2
        elif args[i]=="--urdf" and i+1<len(args): urdf=args[i+1];i+=2
        elif not args[i].startswith("--"): port_name=args[i];i+=1
        else: i+=1

    cal=load_cal(cal_path)
    print(f"IKPy 往复运动  port={port_name}  urdf={Path(urdf).name}")
    try: solver=IKPyIK(urdf)
    except ImportError: print("ERROR: pip install ikpy"); sys.exit(1)
    print("按 Ctrl+C 停止\n")

    port,pkt,gw=open_arm(port_name,cal)
    try:
        idx=0
        while True:
            x,y,z,label=WAYPOINTS[idx%len(WAYPOINTS)]
            cur=read_all(pkt,port,cal)
            t0=time.perf_counter(); sol=solver.solve(cur,x,y,z); ms=(time.perf_counter()-t0)*1000
            print(f"  → {label}  ({x:+.4f},{y:+.4f},{z:+.4f})  solve={ms:.1f}ms  "
                  f"pan={sol['shoulder_pan']:+.1f}° lift={sol['shoulder_lift']:+.1f}°")
            goto(pkt,port,gw,cal,sol,dur=MOVE_DUR)
            time.sleep(0.5); idx+=1
    except KeyboardInterrupt: print("\n停止。")
    finally:
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port,mid,ADDR["torque"],0); pkt.write1ByteTxRx(port,mid,ADDR["lock"],0)
        port.closePort()

if __name__=="__main__": main()
