#!/usr/bin/env python3
"""
IK 评估 — 方法3：IKPy
Requires: pip install ikpy feetech-servo-sdk

IKPy 从 URDF 自动提取 DH 参数，使用 BFGS 数值优化求解 IK。
active_links_mask 标记哪些连杆参与求解（跳过 base 和 gripper）。

Usage:
  python eval_ikpy.py [port] [--cal ../dfarm_calibration.json] [--urdf ../so101_new_calib.urdf]
  python eval_ikpy.py --no-move
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


class IKPyIK:
    def __init__(self, urdf_path: str):
        from ikpy.chain import Chain
        # 先不指定 mask，让 IKPy 自动读取所有 joint
        full_chain = Chain.from_urdf_file(urdf_path)
        n = len(full_chain.links)
        link_names = [lk.name for lk in full_chain.links]
        print(f"  IKPy all links ({n}): {link_names}")

        # 按顺序将 ARM_J 映射到 chain 中非 fixed 的 revolute 关节（按序对应）
        active_indices = [i for i, lk in enumerate(full_chain.links)
                          if lk.joint_type != "fixed" and i > 0]
        print(f"  IKPy revolute indices: {active_indices}")

        # 按 ARM_J 顺序逐一对应
        self._joint_idx = {}
        for j, name in enumerate(ARM_J):
            if j < len(active_indices):
                self._joint_idx[name] = active_indices[j]

        mask = [False] * n
        for idx in self._joint_idx.values():
            mask[idx] = True

        self.chain = Chain.from_urdf_file(urdf_path, active_links_mask=mask)
        print(f"  IKPy joint_map={self._joint_idx}")

    def solve(self, current_deg: dict, tx: float, ty: float, tz: float) -> dict[str, float]:
        n = len(self.chain.links)
        q0 = np.zeros(n)
        for name, idx in self._joint_idx.items():
            if idx < n:
                q0[idx] = math.radians(current_deg.get(name, 0.0))

        q_sol = self.chain.inverse_kinematics(
            target_position=[tx, ty, tz],
            initial_position=q0,
        )

        result = {}
        for name, idx in self._joint_idx.items():
            if idx < n:
                result[name] = math.degrees(q_sol[idx])
        result["gripper"] = 50.0

        # 调试：打印第一次的解以确认映射正确
        if not hasattr(self, "_debug_printed"):
            self._debug_printed = True
            print(f"  IKPy first sol (deg): { {k: f'{v:.1f}' for k,v in result.items() if k != 'gripper'} }")
        return result

    def fk(self, joints_deg: dict) -> np.ndarray:
        n = len(self.chain.links)
        q = np.zeros(n)
        for name, idx in self._joint_idx.items():
            if idx < n:
                q[idx] = math.radians(joints_deg.get(name, 0.0))
        T = self.chain.forward_kinematics(q)
        return T[:3, 3]


def main():
    args = sys.argv[1:]; port=PORT; cal_path="../dfarm_calibration.json"; urdf=URDF_DEF; move=True
    i=0
    while i<len(args):
        if args[i]=="--cal" and i+1<len(args): cal_path=args[i+1];i+=2
        elif args[i]=="--urdf" and i+1<len(args): urdf=args[i+1];i+=2
        elif args[i]=="--no-move": move=False;i+=1
        elif not args[i].startswith("--"): port=args[i];i+=1
        else: i+=1

    print(f"IKPy 评估  urdf={Path(urdf).name}  move={move}")
    try:
        solver = IKPyIK(urdf)
    except ImportError as e:
        print(f"ERROR: {e}"); sys.exit(1)

    cal   = load_cal(cal_path)
    fk_fn = build_pin_fk(urdf)  # pinocchio FK 作为统一验证基准
    if fk_fn is None:
        print("警告: pinocchio 未安装，改用 IKPy 内置 FK 验证（注意：自验误差将为 0，不反映真实精度）")
        fk_fn = solver.fk

    arm = ArmDriver(port, cal) if move else None

    def ik_fn(tx, ty, tz):
        cur = arm.read() if arm else INIT_JOINTS.copy()
        return solver.solve(cur, tx, ty, tz)

    print(f"\n{'':2}{'目标':<16} {'求解时间':>12} {'FK误差':>10} {'实际误差':>10}")
    print("-" * 54)
    try:
        results = run_eval(arm, ik_fn, fk_fn, EVAL_TARGETS, INIT_JOINTS, move)
        print_summary("IKPy", results)
    finally:
        if arm: arm.close()


if __name__ == "__main__": main()
