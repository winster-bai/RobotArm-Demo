#!/usr/bin/env python3
"""
往复运动测试 — 方法2：Placo IK
Requires: pip install placo feetech-servo-sdk

Usage:
  python move_placo.py [port] [--cal ../dfarm_calibration.json] [--urdf ../so101_new_calib.urdf]
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

# ── Placo IK ──────────────────────────────────────────────────────────────────
class PlacoIK:
    def __init__(self, urdf):
        import placo
        self.robot  = placo.RobotWrapper(urdf)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.task   = self.solver.add_frame_task("gripper_frame_link", np.eye(4))

    def solve(self, cur_deg, tx, ty, tz, n_iters=100, tol_mm=5.0):
        import math as _m
        T_target = np.eye(4)
        T_target[0,3], T_target[1,3], T_target[2,3] = tx, ty, tz
        self.task.T_world_frame = T_target
        self.task.configure("gripper_frame_link", "soft", 1.0, 0.01)

        cur = dict(cur_deg)
        for _ in range(n_iters):
            for n in ARM_J:
                self.robot.set_joint(n, _m.radians(cur.get(n, 0.0)))
            self.robot.update_kinematics()
            self.solver.solve(True)
            self.robot.update_kinematics()
            cur = {n: _m.degrees(self.robot.get_joint(n)) for n in ARM_J}
            T_now = self.robot.get_T_world_frame("gripper_frame_link")
            err = _m.sqrt(sum((T_now[i,3]-[tx,ty,tz][i])**2 for i in range(3)))*1000
            if err < tol_mm:
                break

        cur["gripper"] = 50.0
        return cur

# ── 校准 + 电机（同 move_2link.py）──────────────────────────────────────────
def load_cal(p):
    f=Path(p); return json.loads(f.read_text()) if f.exists() else None

def d2r(deg,m,cal):
    if cal and m in cal:
        c=cal[m]; return max(c["range_min"],min(c["range_max"],int(c["homing_raw"]+deg*RES/360)))
    return max(0,min(RES-1,int(CENTER+deg*RES/360)))

def r2d(raw,m,cal):
    if cal and m in cal: return (raw-cal[m]["homing_raw"])*360/RES
    return (raw-CENTER)*360/RES

def open_arm(port_name, cal):
    port=scs.PortHandler(port_name); port.openPort(); port.setBaudRate(1_000_000)
    pkt=scs.PacketHandler(0); gw=scs.GroupSyncWrite(port,pkt,ADDR["goal"],2)
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port,mid,ADDR["torque"],0); pkt.write1ByteTxRx(port,mid,ADDR["lock"],0)
        pkt.write1ByteTxRx(port,mid,ADDR["P"],16);     pkt.write1ByteTxRx(port,mid,ADDR["D"],32)
        pkt.write1ByteTxRx(port,mid,ADDR["I"],0);      pkt.write1ByteTxRx(port,mid,ADDR["accel"],150)
        raw,_,_=pkt.read2ByteTxRx(port,mid,ADDR["pos"])
        pkt.write2ByteTxRx(port,mid,ADDR["goal"],raw)
        pkt.write1ByteTxRx(port,mid,ADDR["torque"],1); pkt.write1ByteTxRx(port,mid,ADDR["lock"],1)
    return port, pkt, gw

def read_all(pkt,port,cal):
    return {n:r2d((lambda v,c,_:v if c==scs.COMM_SUCCESS else CENTER)(*pkt.read2ByteTxRx(port,mid,ADDR["pos"])),n,cal) for n,mid in MOTORS.items()}

def goto(pkt,port,gw,cal,tgt,dur=2.0):
    start=read_all(pkt,port,cal); g0=start.get("gripper",50)
    steps=max(1,int(dur*FREQ))
    for i in range(1,steps+1):
        t0=time.perf_counter(); a=i/steps; gw.clearParam()
        for n in ARM_J:
            gw.addParam(MOTORS[n],[scs.SCS_LOBYTE(d2r(start[n]+a*(tgt.get(n,start[n])-start[n]),n,cal)),
                                    scs.SCS_HIBYTE(d2r(start[n]+a*(tgt.get(n,start[n])-start[n]),n,cal))])
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
    print(f"Placo IK 往复运动  port={port_name}  urdf={Path(urdf).name}")
    try: solver=PlacoIK(urdf)
    except ImportError: print("ERROR: pip install placo"); sys.exit(1)
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
