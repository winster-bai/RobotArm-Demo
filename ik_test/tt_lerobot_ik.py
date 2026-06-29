#!/usr/bin/env python3
"""
lerobot 官方 placo IK 逻辑测试 — 完全按照 robot_kinematic_processor.py 实现。
Requires: pip install placo feetech-servo-sdk

关键点：
  - IK 输入是完整 6D 位姿（4×4 矩阵），不是纯 xyz
  - 旋转部分从当前末端 FK 读取，保持不变，只替换位置
  - 每步只调用一次 solver.solve(True)，外部循环推进收敛
  - 用当前关节角作为初始猜测（initial_guess_current_joints=True）

在四个目标点之间往复，每步打印 IK 解和 FK 验证误差。

Usage:
  python tt_lerobot_ik.py [port] [--cal ../dfarm_calibration.json] [--urdf ../so101_new_calib.urdf]
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
N_IK_ITERS = 10   # 每次移动前的 IK 收敛迭代次数


class LeRobotPlaco:
    """
    完全按照 src/lerobot/model/kinematics.py + robot_kinematic_processor.py 实现。
    """
    def __init__(self, urdf: str):
        import placo
        self.robot  = placo.RobotWrapper(urdf)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        self.tip_frame = self.solver.add_frame_task("gripper_frame_link", np.eye(4))
        self._joint_names = ARM_J

    def fk(self, joint_deg: list[float]) -> np.ndarray:
        """正运动学，返回 4×4 变换矩阵。"""
        for i, name in enumerate(self._joint_names):
            self.robot.set_joint(name, math.radians(joint_deg[i]))
        self.robot.update_kinematics()
        return self.robot.get_T_world_frame("gripper_frame_link")

    def ik(self, q_curr_deg: list[float], t_des: np.ndarray,
           position_weight: float = 1.0, orientation_weight: float = 0.01) -> list[float]:
        """
        单步 IK（完全复制 kinematics.py）。
        q_curr_deg: 当前关节角（度），长度 = len(ARM_J)
        t_des: 目标 4×4 变换矩阵
        返回：目标关节角（度）
        """
        # 设置当前关节角为初始猜测
        for i, name in enumerate(self._joint_names):
            self.robot.set_joint(name, math.radians(q_curr_deg[i]))

        # 设置目标位姿
        self.tip_frame.T_world_frame = t_des
        self.tip_frame.configure("gripper_frame_link", "soft", position_weight, orientation_weight)

        # 单次求解（lerobot 官方也只调一次）
        self.solver.solve(True)
        self.robot.update_kinematics()

        return [math.degrees(self.robot.get_joint(n)) for n in self._joint_names]

    def ik_converge(self, q_curr_deg: list[float], target_xyz: tuple,
                    n_iters: int = 10, tol_mm: float = 2.0) -> tuple[list[float], float]:
        """
        外部收敛循环：反复调用单步 IK，每次用最新解作为初始猜测。
        关键：旋转部分从当前末端 FK 获取，只替换位置。
        这是 robot_kinematic_processor 的使用模式。
        """
        tx, ty, tz = target_xyz

        # 构造目标 4×4：旋转来自当前末端 FK，位置替换为目标
        T_cur = self.fk(q_curr_deg)
        t_des = T_cur.copy()
        t_des[0, 3] = tx
        t_des[1, 3] = ty
        t_des[2, 3] = tz

        q = list(q_curr_deg)
        for _ in range(n_iters):
            q = self.ik(q, t_des)

        # FK 验证误差
        T_result = self.fk(q)
        err_mm = math.sqrt((T_result[0,3]-tx)**2 + (T_result[1,3]-ty)**2 + (T_result[2,3]-tz)**2) * 1000
        return q, err_mm


# ── 校准 + 电机（与其他脚本相同）──────────────────────────────────────────────
def load_cal(p):
    f=Path(p); return json.loads(f.read_text()) if f.exists() else None

def d2r(deg, m, cal):
    if cal and m in cal:
        c=cal[m]; return max(c["range_min"],min(c["range_max"],int(c["homing_raw"]+deg*RES/360)))
    return max(0,min(RES-1,int(CENTER+deg*RES/360)))

def r2d(raw, m, cal):
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
    return port,pkt,gw

def read_deg(pkt, port, cal):
    return {n: r2d((lambda v,c,_: v if c==scs.COMM_SUCCESS else CENTER)(*pkt.read2ByteTxRx(port,mid,ADDR["pos"])),n,cal)
            for n,mid in MOTORS.items()}

def goto(pkt, port, gw, cal, tgt: dict, dur=2.0):
    start=read_deg(pkt,port,cal); g0=start.get("gripper",50)
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
    print(f"LeRobot Placo IK 测试  port={port_name}  urdf={Path(urdf).name}")
    print(f"IK 迭代次数: {N_IK_ITERS}  （可在脚本顶部修改 N_IK_ITERS）")
    try:
        solver=LeRobotPlaco(urdf)
    except ImportError:
        print("ERROR: pip install placo"); sys.exit(1)
    print("按 Ctrl+C 停止\n")

    port,pkt,gw=open_arm(port_name,cal)
    try:
        idx=0
        while True:
            x,y,z,label=WAYPOINTS[idx%len(WAYPOINTS)]
            cur=read_deg(pkt,port,cal)
            q_cur=[cur.get(n,0.0) for n in ARM_J]

            t0=time.perf_counter()
            q_sol, err_mm=solver.ik_converge(q_cur,(x,y,z),n_iters=N_IK_ITERS)
            ms=(time.perf_counter()-t0)*1000

            tgt={n:q_sol[i] for i,n in enumerate(ARM_J)}
            tgt["gripper"]=50.0

            print(f"  → {label}  ({x:+.4f},{y:+.4f},{z:+.4f})  "
                  f"solve={ms:.1f}ms  FK_err={err_mm:.1f}mm  "
                  f"pan={tgt['shoulder_pan']:+.1f}° lift={tgt['shoulder_lift']:+.1f}° "
                  f"ef={tgt['elbow_flex']:+.1f}°")

            goto(pkt,port,gw,cal,tgt,dur=MOVE_DUR)
            time.sleep(0.5); idx+=1
    except KeyboardInterrupt:
        print("\n停止。")
    finally:
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port,mid,ADDR["torque"],0); pkt.write1ByteTxRx(port,mid,ADDR["lock"],0)
        port.closePort()


if __name__=="__main__": main()
