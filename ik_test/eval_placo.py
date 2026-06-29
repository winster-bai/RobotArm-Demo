#!/usr/bin/env python3
"""
IK 评估 — 方法2：Placo IK
Requires: pip install placo feetech-servo-sdk

Usage:
  python eval_placo.py [port] [--cal ../dfarm_calibration.json] [--urdf ../so101_new_calib.urdf]
  python eval_placo.py --no-move   # 只计算，不驱动电机
"""
from __future__ import annotations
import math, sys, time
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from eval_targets import EVAL_TARGETS, INIT_JOINTS
from arm_driver import ArmDriver, load_cal, build_pin_fk, run_eval, print_summary, ARM_J

PORT     = "/dev/ttyACM0"
URDF_DEF = str(Path(__file__).parent.parent / "so101_new_calib.urdf")


class PlacoIK:
    def __init__(self, urdf_path: str):
        import placo
        self.placo  = placo
        self.robot  = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)
        # 与 9_dfarm_ik_xyz.py 完全一致的已验证写法
        self.tip_task = self.solver.add_frame_task("gripper_frame_link", np.eye(4))

    def solve(self, current_deg: dict, tx: float, ty: float, tz: float) -> dict[str, float]:
        for name in ARM_J:
            self.robot.set_joint(name, math.radians(current_deg.get(name, 0.0)))
        self.robot.update_kinematics()

        # 保持当前姿态旋转，只替换位置目标
        T = self.robot.get_T_world_frame("gripper_frame_link").copy()
        T[0, 3], T[1, 3], T[2, 3] = tx, ty, tz
        self.tip_task.T_world_frame = T
        # orientation_weight=0：只约束位置，不约束姿态
        self.tip_task.configure("gripper_frame_link", "soft", 1.0, 0.0)

        for _ in range(100):
            self.solver.solve(True)
        self.robot.update_kinematics()

        return {name: math.degrees(self.robot.get_joint(name)) for name in ARM_J}

    def fk(self, joints_deg: dict) -> np.ndarray:
        """用 Placo 自身 FK 验证（与求解器坐标系一致）。"""
        for name in ARM_J:
            self.robot.set_joint(name, math.radians(joints_deg.get(name, 0.0)))
        self.robot.update_kinematics()
        T = self.robot.get_T_world_frame("gripper_frame_link")
        return np.array([T[0, 3], T[1, 3], T[2, 3]])


def main():
    args = sys.argv[1:]; port=PORT; cal_path="../dfarm_calibration.json"; urdf=URDF_DEF; move=True
    i=0
    while i<len(args):
        if args[i]=="--cal" and i+1<len(args): cal_path=args[i+1];i+=2
        elif args[i]=="--urdf" and i+1<len(args): urdf=args[i+1];i+=2
        elif args[i]=="--no-move": move=False;i+=1
        elif not args[i].startswith("--"): port=args[i];i+=1
        else: i+=1

    print(f"Placo IK 评估  urdf={Path(urdf).name}  move={move}")
    try:
        solver = PlacoIK(urdf)
    except ImportError as e:
        print(f"ERROR: {e}"); sys.exit(1)

    cal   = load_cal(cal_path)
    fk_fn = build_pin_fk(urdf)
    if fk_fn is None:
        print("警告: pinocchio 未安装，FK 验证将跳过")

    # 闭包：每次求解前读取当前关节作为初始猜测
    arm = ArmDriver(port, cal) if move else None

    def ik_fn(tx, ty, tz):
        cur = arm.read() if arm else INIT_JOINTS.copy()
        sol = solver.solve(cur, tx, ty, tz)
        sol["gripper"] = 50.0
        return sol

    print(f"\n{'':2}{'目标':<16} {'求解时间':>12} {'FK误差':>10} {'实际误差':>10}")
    print("-" * 54)
    try:
        results = run_eval(arm, ik_fn, fk_fn, EVAL_TARGETS, INIT_JOINTS, move)
        print_summary("Placo", results)
    finally:
        if arm: arm.close()


if __name__ == "__main__": main()
