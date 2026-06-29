#!/usr/bin/env python3
"""
SO-101 末端执行器 XYZ 位置控制 — 基于 Pinocchio IK + URDF。
依赖: pip install pin feetech-servo-sdk

通过解算完整逆运动学，将末端执行器移动到指定 (x, y, z) 坐标。
坐标系为 URDF base_link 坐标系（单位：米）。

用法:
  python 9_dfarm_ik_xyz.py [port] [--urdf so101_new_calib.urdf] [--cal dfarm_calibration.json]

交互命令:
  xyz <x> <y> <z>          移动到指定坐标（米）
  xyz <x> <y> <z> <sec>    指定运动时间
  fk                        打印当前末端执行器位姿
  pos                       打印当前关节角度
  free                      解锁扭矩
  q                         退出
"""

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import scservo_sdk as scs

# ── 默认参数 ──────────────────────────────────────────────────────────────────

PORT         = "/dev/ttyACM0"
BAUDRATE     = 1_000_000
PROTOCOL     = 0
FREQ         = 50
DEFAULT_URDF = Path(__file__).parent / "so101_new_calib.urdf"
DEFAULT_CAL  = Path(__file__).parent / "dfarm_calibration.json"

MOTORS = {
    "shoulder_pan":  1,
    "shoulder_lift": 2,
    "elbow_flex":    3,
    "wrist_flex":    4,
    "wrist_roll":    5,
    "gripper":       6,
}
# IK 只求解这5个关节，gripper 单独保留
IK_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

ADDR = {"torque": 40, "accel": 41, "goal": 42, "lock": 55, "pos": 56,
        "P": 21, "D": 22, "I": 23}
RES, CENTER = 4096, 2048


# ── 校准辅助 ──────────────────────────────────────────────────────────────────

def load_cal(path: Path) -> dict | None:
    if path.exists():
        cal = json.loads(path.read_text())
        print(f"已加载校准文件: {path}")
        return cal
    print("未找到校准文件，使用原始角度模式。")
    return None


def deg2raw(deg: float, motor: str, cal: dict | None) -> int:
    if cal and motor in cal:
        c = cal[motor]
        return max(c["range_min"], min(c["range_max"], int(c["homing_raw"] + deg * RES / 360.0)))
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360.0)))


def raw2deg(raw: int, motor: str, cal: dict | None) -> float:
    if cal and motor in cal:
        return (raw - cal[motor]["homing_raw"]) * 360.0 / RES
    return (raw - CENTER) * 360.0 / RES


# ── 运动学 ────────────────────────────────────────────────────────────────────

class SO101Kinematics:
    """基于 Pinocchio 的 SO-101 正逆运动学求解器（阻尼最小二乘雅可比迭代）。"""

    def __init__(self, urdf_path: str):
        try:
            import pinocchio as pin
        except ImportError:
            raise ImportError("需要安装 pinocchio: pip install pin")

        self.pin   = pin
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.fid   = self.model.getFrameId("gripper_frame_link")

    def _deg_to_q(self, joint_deg: dict[str, float]) -> np.ndarray:
        q = np.zeros(self.model.nq)
        for i, name in enumerate(IK_JOINTS):
            q[i] = math.radians(joint_deg.get(name, 0.0))
        return np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

    def _fk_pos(self, q: np.ndarray) -> np.ndarray:
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.fid].translation.copy()

    def forward_kinematics(self, joint_deg: dict[str, float]) -> np.ndarray:
        """返回末端 xyz 位置（米）。"""
        return self._fk_pos(self._deg_to_q(joint_deg))

    def inverse_kinematics(
        self,
        current_deg: dict[str, float],
        target_xyz: tuple[float, float, float],
        max_iters: int = 300,
        tol: float = 1e-3,
        step: float = 0.6,
        damp: float = 1e-4,
    ) -> dict[str, float]:
        """
        纯 3D 位置 IK（阻尼最小二乘雅可比迭代）。
        只约束末端位置，姿态完全自由，精度高、收敛稳定。
        """
        target = np.array(target_xyz)
        q = self._deg_to_q(current_deg)
        g = q[5] if self.model.nq > 5 else 0.0

        for _ in range(max_iters):
            err = target - self._fk_pos(q)
            if np.linalg.norm(err) < tol:
                break
            J = self.pin.computeFrameJacobian(
                self.model, self.data, q, self.fid,
                self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )[:3, :5]
            dq = np.zeros(self.model.nv)
            dq[:5] = step * J.T @ np.linalg.solve(J @ J.T + damp * np.eye(3), err)
            q = self.pin.integrate(self.model, q, dq)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
            if self.model.nq > 5:
                q[5] = g

        return {name: math.degrees(q[i]) for i, name in enumerate(IK_JOINTS)}

    def get_ee_xyz(self, joint_deg: dict[str, float]) -> tuple[float, float, float]:
        """返回当前末端 xyz 位置（米）。"""
        pos = self.forward_kinematics(joint_deg)
        return float(pos[0]), float(pos[1]), float(pos[2])


# ── 电机 I/O ──────────────────────────────────────────────────────────────────

class MotorIO:
    def __init__(self, port_name: str, cal: dict | None):
        self.cal  = cal
        self.port = scs.PortHandler(port_name)
        if not self.port.openPort():
            raise RuntimeError(f"无法打开串口 {port_name}")
        self.port.setBaudRate(BAUDRATE)
        self.pkt = scs.PacketHandler(PROTOCOL)
        self.gw  = scs.GroupSyncWrite(self.port, self.pkt, ADDR["goal"], 2)

        # 初始化所有电机
        for mid in MOTORS.values():
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["torque"], 0)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["lock"],   0)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["P"],      16)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["D"],      32)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["I"],       0)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["accel"], 150)
            raw = self._read_raw(mid)
            self.pkt.write2ByteTxRx(self.port, mid, ADDR["goal"], raw)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["torque"], 1)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["lock"],   1)

    def set_torque(self, enable: bool):
        val = 1 if enable else 0
        for mid in MOTORS.values():
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["torque"], val)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["lock"],   val)

    def close(self):
        self.set_torque(False)
        self.port.closePort()

    def _read_raw(self, mid: int) -> int:
        v, c, _ = self.pkt.read2ByteTxRx(self.port, mid, ADDR["pos"])
        return v if c == scs.COMM_SUCCESS else CENTER

    def read_all_deg(self) -> dict[str, float]:
        return {name: raw2deg(self._read_raw(mid), name, self.cal) for name, mid in MOTORS.items()}

    def send_all(self, joint_deg: dict[str, float]):
        self.gw.clearParam()
        for name, mid in MOTORS.items():
            raw = deg2raw(joint_deg.get(name, 0.0), name, self.cal)
            self.gw.addParam(mid, [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])
        self.gw.txPacket()

    def goto(self, target_deg: dict[str, float], duration: float = 2.0):
        """线性插值运动到目标关节角度。"""
        start = self.read_all_deg()
        steps = max(1, int(duration * FREQ))
        for i in range(1, steps + 1):
            t0    = time.perf_counter()
            alpha = i / steps
            interp = {
                name: start[name] + alpha * (target_deg.get(name, start[name]) - start[name])
                for name in MOTORS
            }
            self.send_all(interp)
            sl = 1.0 / FREQ - (time.perf_counter() - t0)
            if sl > 0:
                time.sleep(sl)


# ── 主程序 ────────────────────────────────────────────────────────────────────

def main():
    args      = sys.argv[1:]
    port_name = PORT
    urdf_path = DEFAULT_URDF
    cal_path  = DEFAULT_CAL
    i = 0
    while i < len(args):
        if args[i] == "--urdf" and i + 1 < len(args):
            urdf_path = Path(args[i + 1]); i += 2
        elif args[i] == "--cal" and i + 1 < len(args):
            cal_path = Path(args[i + 1]); i += 2
        elif not args[i].startswith("--"):
            port_name = args[i]; i += 1
        else:
            i += 1

    if not urdf_path.exists():
        print(f"ERROR: URDF 文件不存在: {urdf_path}")
        sys.exit(1)

    print(f"SO-101 IK XYZ 控制 — {port_name}")
    print(f"URDF: {urdf_path}")

    cal = load_cal(cal_path)
    kin = SO101Kinematics(str(urdf_path))
    mot = MotorIO(port_name, cal)

    print("\n命令:")
    print("  xyz <x> <y> <z> [sec]   移动到目标位置（米），默认2秒")
    print("  fk                       显示当前末端执行器 xyz")
    print("  pos                      显示当前关节角度")
    print("  free                     解锁扭矩，自由移动模式（持续打印 FK）")
    print("  lock                     重新上扭矩")
    print("  q                        退出")
    print()

    # 打印初始末端位置
    cur = mot.read_all_deg()
    x, y, z = kin.get_ee_xyz(cur)
    print(f"当前末端位置: x={x:.4f}  y={y:.4f}  z={z:.4f} (m)")
    print()

    try:
        while True:
            try:
                line = input(">> ").strip()
            except EOFError:
                break

            parts = line.split()
            if not parts:
                continue
            cmd = parts[0].lower()

            if cmd == "q":
                break

            elif cmd == "free":
                mot.set_torque(False)
                print("  扭矩已解锁，自由移动模式。按 Enter 停止并重新上扭矩...")
                try:
                    while True:
                        cur = mot.read_all_deg()
                        x, y, z = kin.get_ee_xyz(cur)
                        sys.stdout.write(f"\r  FK  x={x:+.4f}  y={y:+.4f}  z={z:+.4f} (m)   ")
                        sys.stdout.flush()
                        # 非阻塞检测 Enter：用 select 监听 stdin
                        import select
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            sys.stdin.readline()
                            break
                except KeyboardInterrupt:
                    pass
                finally:
                    mot.set_torque(True)
                    print("\n  扭矩已恢复。")

            elif cmd == "lock":
                mot.set_torque(True)
                print("  扭矩已上锁。")

            elif cmd == "fk":
                cur = mot.read_all_deg()
                x, y, z = kin.get_ee_xyz(cur)
                print(f"末端位置: x={x:.4f}  y={y:.4f}  z={z:.4f} (m)")

            elif cmd == "pos":
                cur = mot.read_all_deg()
                print(f"\n{'关节':<16} {'角度':>8}")
                print("-" * 26)
                for name in MOTORS:
                    print(f"  {name:<14} {cur[name]:>+7.2f}°")
                print()

            elif cmd == "xyz" and len(parts) >= 4:
                try:
                    tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
                    duration   = float(parts[4]) if len(parts) >= 5 else 2.0
                except ValueError:
                    print("  格式错误，示例: xyz 0.15 0.00 0.20 2.0")
                    continue

                cur = mot.read_all_deg()
                print(f"  求解 IK: ({tx:.4f}, {ty:.4f}, {tz:.4f}) ...")

                try:
                    target_ik = kin.inverse_kinematics(cur, (tx, ty, tz))
                except Exception as e:
                    print(f"  IK 求解失败: {e}")
                    continue

                # 保留 gripper 当前位置不变
                target_ik["gripper"] = cur["gripper"]

                # 验证 FK 误差
                fx, fy, fz = kin.get_ee_xyz(target_ik)
                err = math.sqrt((fx - tx)**2 + (fy - ty)**2 + (fz - tz)**2)
                print(f"  IK 结果 FK 验证: ({fx:.4f}, {fy:.4f}, {fz:.4f})  误差={err*1000:.2f} mm")

                if err > 0.05:
                    print(f"  警告：误差超过 50mm，目标可能超出工作空间，仍然执行。")

                print(f"  运动中... ({duration:.1f}s)")
                mot.goto(target_ik, duration)

                # 实际到达位置
                actual = mot.read_all_deg()
                ax, ay, az = kin.get_ee_xyz(actual)
                print(f"  到达位置: x={ax:.4f}  y={ay:.4f}  z={az:.4f} (m)")

            else:
                print("  未知命令。输入 xyz <x> <y> <z>、fk、pos 或 q")

    except KeyboardInterrupt:
        pass
    finally:
        mot.close()
        print("\nDone.")


if __name__ == "__main__":
    main()
