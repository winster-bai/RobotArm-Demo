#!/usr/bin/env python3
"""
IK 评估 — 方法4：Pinocchio（阻尼最小二乘雅可比迭代）
Requires: pip install pin feetech-servo-sdk

与 soarm-control 使用同一套 IK 实现，作为精度和速度的参考基准。

Usage:
  python eval_pinocchio.py [port] [--cal ../dfarm_calibration.json] [--urdf ../so101_new_calib.urdf]
  python eval_pinocchio.py --no-move
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

# IK 超参数
MAX_ITERS  = 300
TOL        = 1e-3   # 收敛阈值（米）
STEP_SIZE  = 0.6
DAMPING    = 1e-4


class PinocchioIK:
    def __init__(self, urdf_path: str):
        import pinocchio as pin
        self.pin    = pin
        self.model  = pin.buildModelFromUrdf(urdf_path)
        self.data   = self.model.createData()
        self.fid    = self.model.getFrameId("gripper_frame_link")

    def _obs_to_q(self, joints_deg: dict) -> np.ndarray:
        q = np.zeros(self.model.nq)
        for i, name in enumerate(ARM_J):
            q[i] = math.radians(joints_deg.get(name, 0.0))
        return np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

    def _fk_pos(self, q: np.ndarray) -> np.ndarray:
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.fid].translation.copy()

    def solve(self, current_deg: dict, tx: float, ty: float, tz: float,
              max_iters: int = MAX_ITERS, tol: float = TOL,
              step: float = STEP_SIZE, damp: float = DAMPING) -> tuple[dict, float, int]:
        """
        返回 (关节角度字典, 最终误差m, 迭代次数)
        """
        target = np.array([tx, ty, tz])
        q = self._obs_to_q(current_deg)
        fixed_gripper = q[5] if len(q) > 5 else 0.0

        for it in range(max_iters):
            cur_xyz = self._fk_pos(q)
            err_vec = target - cur_xyz
            err_norm = float(np.linalg.norm(err_vec))
            if err_norm < tol:
                break

            J = self.pin.computeFrameJacobian(
                self.model, self.data, q, self.fid,
                self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )[:3, :5]

            dq_arm = step * J.T @ np.linalg.solve(J @ J.T + damp * np.eye(3), err_vec)
            dq = np.zeros(self.model.nv)
            dq[:5] = dq_arm
            q = self.pin.integrate(self.model, q, dq)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
            if len(q) > 5: q[5] = fixed_gripper

        final_xyz  = self._fk_pos(q)
        final_err  = float(np.linalg.norm(target - final_xyz))
        result = {name: math.degrees(q[i]) for i, name in enumerate(ARM_J)}
        result["gripper"] = 50.0
        return result, final_err, it + 1


def main():
    args = sys.argv[1:]; port=PORT; cal_path="../dfarm_calibration.json"; urdf=URDF_DEF; move=True
    i=0
    while i<len(args):
        if args[i]=="--cal" and i+1<len(args): cal_path=args[i+1];i+=2
        elif args[i]=="--urdf" and i+1<len(args): urdf=args[i+1];i+=2
        elif args[i]=="--no-move": move=False;i+=1
        elif not args[i].startswith("--"): port=args[i];i+=1
        else: i+=1

    print(f"Pinocchio IK 评估  urdf={Path(urdf).name}  move={move}")
    try:
        solver = PinocchioIK(urdf)
    except ImportError as e:
        print(f"ERROR: {e}"); sys.exit(1)

    cal   = load_cal(cal_path)
    fk_fn = build_pin_fk(urdf)  # 同一个模型，FK 验证与求解器一致

    arm = ArmDriver(port, cal) if move else None

    iters_log = []  # 记录每次迭代次数

    def ik_fn(tx, ty, tz):
        cur = arm.read() if arm else INIT_JOINTS.copy()
        sol, final_err, iters = solver.solve(cur, tx, ty, tz)
        iters_log.append(iters)
        if final_err > TOL:
            print(f"    ⚠ 未收敛: final_err={final_err*1000:.1f}mm  iters={iters}")
        return sol

    print(f"\n{'':2}{'目标':<16} {'求解时间':>12} {'FK误差':>10} {'实际误差':>10}")
    print("-" * 54)
    try:
        results = run_eval(arm, ik_fn, fk_fn, EVAL_TARGETS, INIT_JOINTS, move)
        print_summary("Pinocchio", results)
        if iters_log:
            print(f"  平均迭代次数 {sum(iters_log)/len(iters_log):.1f}  最多 {max(iters_log)} 次")
    finally:
        if arm: arm.close()


if __name__ == "__main__": main()
