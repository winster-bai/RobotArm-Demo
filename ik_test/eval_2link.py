#!/usr/bin/env python3
"""
IK 评估 — 方法1：2-Link Planar IK（扩展版）
Requires: pip install feetech-servo-sdk

局限性说明：
  2-link planar 本质是 2D IK（只解 x, y 平面）。
  为近似处理 3D 目标点 (x, y, z)，采用以下策略：
    - shoulder_pan  = atan2(y, x)，将目标投影到手臂平面
    - 平面内伸展量 r = sqrt(x²+y²)，作为 2D IK 的水平坐标
    - z 轴通过调整 wrist_flex 近似补偿（pitch_sum 不固定，允许变化）
    - 这是一种近似，FK 验证会暴露其与 URDF 真实几何的偏差

  本脚本同时用 pinocchio FK 验证实际到达位置，以公平比较误差。

Usage:
  python eval_2link.py [port] [--cal dfarm_calibration.json] [--urdf ../so101_new_calib.urdf]
"""

from __future__ import annotations
import json, math, sys, time
from pathlib import Path
import numpy as np
import scservo_sdk as scs

# 加载共享评估目标
sys.path.insert(0, str(Path(__file__).parent))
from eval_targets import EVAL_TARGETS, INIT_JOINTS

# ── 常量 ──────────────────────────────────────────────────────────────────────
PORT    = "/dev/ttyACM0"
FREQ    = 50
MOTORS  = {"shoulder_pan":1,"shoulder_lift":2,"elbow_flex":3,"wrist_flex":4,"wrist_roll":5,"gripper":6}
ARM_J   = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
ADDR    = {"torque":40,"accel":41,"goal":42,"lock":55,"pos":56,"P":21,"D":22,"I":23}
RES, CENTER = 4096, 2048

# 2-link 参数
L1 = 0.1159
L2 = 0.1350
THETA1_OFF = math.atan2(0.028, 0.11257)
THETA2_OFF = math.atan2(0.0052, 0.1349) + THETA1_OFF

# ── 校准 ──────────────────────────────────────────────────────────────────────
def load_cal(p): return json.loads(Path(p).read_text()) if Path(p).exists() else None
def d2r(d,m,c):
    if c and m in c: cc=c[m]; return max(cc["range_min"],min(cc["range_max"],int(cc["homing_raw"]+d*RES/360)))
    return max(0,min(RES-1,int(CENTER+d*RES/360)))
def r2d(r,m,c):
    if c and m in c: return (r-c[m]["homing_raw"])*360/RES
    return (r-CENTER)*360/RES

# ── 2-link IK（扩展到 3D 近似）────────────────────────────────────────────────
def ik_2link(x: float, y: float, z: float) -> dict[str, float]:
    """
    近似 3D IK：
      pan  = atan2(y, x)
      r    = sqrt(x²+y²)  在平面内的伸展
      高度 z 通过调整 shoulder_lift 的垂直分量近似
      实际上将 (r, z) 作为 2D IK 的 (x, y)
    """
    pan = math.degrees(math.atan2(y, x))
    r   = math.hypot(x, y)
    # 将 (r, z) 作为平面 IK 输入（r=水平，z=垂直高度）
    px, py = r, z
    dist = math.hypot(px, py)
    dist = max(abs(L1-L2)*1.01, min((L1+L2)*0.99, dist))
    if math.hypot(px,py) > 0 and dist != math.hypot(px,py):
        s = dist/math.hypot(px,py); px*=s; py*=s
    c2 = -(dist**2-L1**2-L2**2)/(2*L1*L2)
    c2 = max(-1.0,min(1.0,c2))
    t2 = math.pi - math.acos(c2)
    t1 = math.atan2(py,px) + math.atan2(L2*math.sin(t2), L1+L2*math.cos(t2))
    j2 = max(-0.1, min(3.45, t1+THETA1_OFF))
    j3 = max(-0.2, min(math.pi, t2+THETA2_OFF))
    sl = 90.0 - math.degrees(j2)
    ef = math.degrees(j3) - 90.0
    # wrist_flex：保持夹爪大致水平（固定补偿）
    wf = -sl - ef + 90.0
    return {"shoulder_pan":pan,"shoulder_lift":sl,"elbow_flex":ef,
            "wrist_flex":wf,"wrist_roll":0.0,"gripper":50.0}

# ── FK 验证（用 pinocchio）────────────────────────────────────────────────────
def build_pin_fk(urdf_path: str):
    try:
        import pinocchio as pin
        model = pin.buildModelFromUrdf(urdf_path)
        data  = model.createData()
        fid   = model.getFrameId("gripper_frame_link")
        def fk(joints_deg: dict) -> np.ndarray:
            q = np.zeros(model.nq)
            for i,j in enumerate(ARM_J):
                q[i] = math.radians(joints_deg.get(j,0.0))
            pin.forwardKinematics(model,data,q)
            pin.updateFramePlacements(model,data)
            return data.oMf[fid].translation.copy()
        return fk
    except ImportError:
        return None

# ── 电机 I/O ──────────────────────────────────────────────────────────────────
class Arm:
    def __init__(self, port_name, cal):
        self.cal = cal
        self.port = scs.PortHandler(port_name)
        self.port.openPort(); self.port.setBaudRate(1_000_000)
        self.pkt = scs.PacketHandler(0)
        self.gw  = scs.GroupSyncWrite(self.port, self.pkt, ADDR["goal"], 2)
        for mid in MOTORS.values():
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["torque"],0)
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["lock"],0)
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["P"],16)
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["D"],32)
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["I"],0)
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["accel"],150)
            raw,_,_ = self.pkt.read2ByteTxRx(self.port,mid,ADDR["pos"])
            self.pkt.write2ByteTxRx(self.port,mid,ADDR["goal"],raw)
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["torque"],1)
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["lock"],1)

    def read(self):
        return {n: r2d((lambda v,c,_:v if c==scs.COMM_SUCCESS else CENTER)(*self.pkt.read2ByteTxRx(self.port,mid,ADDR["pos"])),n,self.cal) for n,mid in MOTORS.items()}

    def goto(self, tgt, dur=2.0):
        start = self.read(); g0 = start.get("gripper",50)
        steps = max(1,int(dur*FREQ))
        for i in range(1,steps+1):
            t0=time.perf_counter(); a=i/steps
            self.gw.clearParam()
            for n in ARM_J:
                raw=d2r(start[n]+a*(tgt.get(n,start[n])-start[n]),n,self.cal)
                self.gw.addParam(MOTORS[n],[scs.SCS_LOBYTE(raw),scs.SCS_HIBYTE(raw)])
            gr=d2r(g0+a*(tgt.get("gripper",50)-g0),"gripper",self.cal)
            self.gw.addParam(MOTORS["gripper"],[scs.SCS_LOBYTE(gr),scs.SCS_HIBYTE(gr)])
            self.gw.txPacket()
            sl=1/FREQ-(time.perf_counter()-t0)
            if sl>0: time.sleep(sl)

    def close(self):
        for mid in MOTORS.values():
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["torque"],0)
            self.pkt.write1ByteTxRx(self.port,mid,ADDR["lock"],0)
        self.port.closePort()

# ── 评估 ──────────────────────────────────────────────────────────────────────
def evaluate(arm: Arm, fk_fn, move: bool):
    results = []
    for tx,ty,tz,label in EVAL_TARGETS:
        # 回到初始姿态
        arm.goto(INIT_JOINTS, dur=2.0); time.sleep(0.5)

        # 求解计时
        t0 = time.perf_counter()
        sol = ik_2link(tx, ty, tz)
        solve_ms = (time.perf_counter()-t0)*1000

        # FK 验证误差（纯计算，不移动机械臂）
        if fk_fn:
            ee = fk_fn(sol)
            err_mm = np.linalg.norm(ee - np.array([tx,ty,tz])) * 1000
        else:
            err_mm = float("nan")

        # 实际移动
        if move:
            arm.goto(sol, dur=2.0)
            time.sleep(0.8)
            actual = arm.read()
            if fk_fn:
                ee_actual = fk_fn(actual)
                real_err_mm = np.linalg.norm(ee_actual - np.array([tx,ty,tz])) * 1000
            else:
                real_err_mm = float("nan")
        else:
            real_err_mm = float("nan")

        results.append({"label":label,"target":(tx,ty,tz),
                         "solve_ms":solve_ms,"fk_err_mm":err_mm,"real_err_mm":real_err_mm})
        print(f"  {label:<16} solve={solve_ms:.3f}ms  FK_err={err_mm:.1f}mm  real_err={real_err_mm:.1f}mm")

    return results

def print_summary(results):
    print("\n── 汇总 ────────────────────────────────────")
    fk_errs  = [r["fk_err_mm"] for r in results if not math.isnan(r["fk_err_mm"])]
    real_errs= [r["real_err_mm"] for r in results if not math.isnan(r["real_err_mm"])]
    times    = [r["solve_ms"]  for r in results]
    if fk_errs:  print(f"  FK误差     avg={sum(fk_errs)/len(fk_errs):.1f}mm  max={max(fk_errs):.1f}mm")
    if real_errs:print(f"  实际误差   avg={sum(real_errs)/len(real_errs):.1f}mm  max={max(real_errs):.1f}mm")
    print(f"  求解时间   avg={sum(times)/len(times):.3f}ms  max={max(times):.3f}ms")

def main():
    args = sys.argv[1:]; port=PORT; cal_path="dfarm_calibration.json"; urdf_path=str(Path(__file__).parent.parent/"so101_new_calib.urdf"); move=True
    i=0
    while i<len(args):
        if args[i]=="--cal" and i+1<len(args): cal_path=args[i+1];i+=2
        elif args[i]=="--urdf" and i+1<len(args): urdf_path=args[i+1];i+=2
        elif args[i]=="--no-move": move=False;i+=1
        elif not args[i].startswith("--"): port=args[i];i+=1
        else: i+=1

    cal = load_cal(cal_path)
    fk_fn = build_pin_fk(urdf_path)
    if fk_fn is None:
        print("警告: pinocchio 未安装，FK 验证将跳过（误差显示 nan）")
    if not move:
        print("--no-move 模式：只计算 IK，不驱动电机")
        results = []
        for tx,ty,tz,label in EVAL_TARGETS:
            t0=time.perf_counter(); sol=ik_2link(tx,ty,tz); ms=(time.perf_counter()-t0)*1000
            err=np.linalg.norm(fk_fn(sol)-np.array([tx,ty,tz]))*1000 if fk_fn else float("nan")
            results.append({"label":label,"target":(tx,ty,tz),"solve_ms":ms,"fk_err_mm":err,"real_err_mm":float("nan")})
            print(f"  {label:<16} solve={ms:.3f}ms  FK_err={err:.1f}mm")
        print_summary(results); return

    arm = Arm(port, cal)
    print(f"\n2-Link Planar IK 评估 — {port}\n{'目标':<16} {'求解时间':>12} {'FK误差':>10} {'实际误差':>10}")
    print("-"*52)
    try:
        results = evaluate(arm, fk_fn, move)
        print_summary(results)
    finally:
        arm.close()

if __name__=="__main__": main()
