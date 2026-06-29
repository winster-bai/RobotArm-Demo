"""
公共电机驱动模块 — 四个评估脚本共用，避免重复。
"""
from __future__ import annotations
import json, math, time
from pathlib import Path
import numpy as np
import scservo_sdk as scs

MOTORS  = {"shoulder_pan":1,"shoulder_lift":2,"elbow_flex":3,"wrist_flex":4,"wrist_roll":5,"gripper":6}
ARM_J   = ["shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll"]
ADDR    = {"torque":40,"accel":41,"goal":42,"lock":55,"pos":56,"P":21,"D":22,"I":23}
RES, CENTER, FREQ = 4096, 2048, 50


def load_cal(path: str) -> dict | None:
    p = Path(path)
    return json.loads(p.read_text()) if p.exists() else None


def deg2raw(deg: float, motor: str, cal: dict | None) -> int:
    if cal and motor in cal:
        c = cal[motor]
        return max(c["range_min"], min(c["range_max"], int(c["homing_raw"] + deg * RES / 360)))
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360)))


def raw2deg(raw: int, motor: str, cal: dict | None) -> float:
    if cal and motor in cal:
        return (raw - cal[motor]["homing_raw"]) * 360.0 / RES
    return (raw - CENTER) * 360.0 / RES


class ArmDriver:
    def __init__(self, port_name: str, cal: dict | None):
        self.cal = cal
        self.port = scs.PortHandler(port_name)
        if not self.port.openPort():
            raise RuntimeError(f"无法打开串口 {port_name}")
        self.port.setBaudRate(1_000_000)
        self.pkt = scs.PacketHandler(0)
        self.gw  = scs.GroupSyncWrite(self.port, self.pkt, ADDR["goal"], 2)
        for mid in MOTORS.values():
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["torque"], 0)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["lock"],   0)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["P"],      16)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["D"],      32)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["I"],       0)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["accel"], 150)
            raw, _, _ = self.pkt.read2ByteTxRx(self.port, mid, ADDR["pos"])
            self.pkt.write2ByteTxRx(self.port, mid, ADDR["goal"], raw)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["torque"], 1)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["lock"],   1)

    def read(self) -> dict[str, float]:
        result = {}
        for name, mid in MOTORS.items():
            v, c, _ = self.pkt.read2ByteTxRx(self.port, mid, ADDR["pos"])
            result[name] = raw2deg(v if c == scs.COMM_SUCCESS else CENTER, name, self.cal)
        return result

    def goto(self, target: dict[str, float], duration: float = 2.0):
        start = self.read()
        g0    = start.get("gripper", 50.0)
        steps = max(1, int(duration * FREQ))
        for i in range(1, steps + 1):
            t0 = time.perf_counter()
            a  = i / steps
            self.gw.clearParam()
            for name in ARM_J:
                raw = deg2raw(start[name] + a * (target.get(name, start[name]) - start[name]), name, self.cal)
                self.gw.addParam(MOTORS[name], [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])
            gr = deg2raw(g0 + a * (target.get("gripper", 50.0) - g0), "gripper", self.cal)
            self.gw.addParam(MOTORS["gripper"], [scs.SCS_LOBYTE(gr), scs.SCS_HIBYTE(gr)])
            self.gw.txPacket()
            sl = 1.0 / FREQ - (time.perf_counter() - t0)
            if sl > 0: time.sleep(sl)

    def close(self):
        for mid in MOTORS.values():
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["torque"], 0)
            self.pkt.write1ByteTxRx(self.port, mid, ADDR["lock"],   0)
        self.port.closePort()


def build_pin_fk(urdf_path: str):
    """构建 pinocchio FK 函数，用于所有脚本的误差验证。返回 None 如果未安装。"""
    try:
        import pinocchio as pin
        model = pin.buildModelFromUrdf(urdf_path)
        data  = model.createData()
        fid   = model.getFrameId("gripper_frame_link")
        def fk(joints_deg: dict) -> np.ndarray:
            q = np.zeros(model.nq)
            for i, j in enumerate(ARM_J):
                q[i] = math.radians(joints_deg.get(j, 0.0))
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            return data.oMf[fid].translation.copy()
        return fk
    except ImportError:
        return None


def run_eval(arm: ArmDriver | None, ik_fn, fk_fn, targets: list, init_joints: dict,
             move: bool = True) -> list[dict]:
    """
    通用评估循环，四个脚本共用。
    ik_fn(x,y,z) -> dict[str,float]  关节角度解
    fk_fn(joints) -> np.ndarray      末端 xyz（pinocchio）
    """
    results = []
    for tx, ty, tz, label in targets:
        if arm and move:
            arm.goto(init_joints, duration=2.0)
            time.sleep(0.5)

        # IK 求解计时
        t0 = time.perf_counter()
        try:
            sol = ik_fn(tx, ty, tz)
            ok  = True
        except Exception as e:
            sol = init_joints.copy()
            ok  = False
            print(f"  {label}: IK 失败 — {e}")
        solve_ms = (time.perf_counter() - t0) * 1000

        # FK 验证误差
        fk_err = float("nan")
        if fk_fn and ok:
            ee = fk_fn(sol)
            fk_err = float(np.linalg.norm(ee - np.array([tx, ty, tz])) * 1000)

        # 实际移动 + 真实误差
        real_err = float("nan")
        if arm and move and ok:
            arm.goto(sol, duration=2.0)
            time.sleep(0.8)
            actual = arm.read()
            if fk_fn:
                ee_actual = fk_fn(actual)
                real_err  = float(np.linalg.norm(ee_actual - np.array([tx, ty, tz])) * 1000)

        results.append({
            "label": label, "target": (tx, ty, tz), "ok": ok,
            "solve_ms": solve_ms, "fk_err_mm": fk_err, "real_err_mm": real_err,
        })
        status = "✓" if ok else "✗"
        print(f"  {status} {label:<16} solve={solve_ms:7.3f}ms  "
              f"FK_err={fk_err:6.1f}mm  real_err={real_err:6.1f}mm")
    return results


def print_summary(method: str, results: list[dict]):
    import math as _math
    print(f"\n── {method} 汇总 {'─'*(40-len(method))}")
    ok_n     = sum(1 for r in results if r["ok"])
    fk_errs  = [r["fk_err_mm"]  for r in results if not _math.isnan(r["fk_err_mm"])]
    real_errs= [r["real_err_mm"] for r in results if not _math.isnan(r["real_err_mm"])]
    times    = [r["solve_ms"]   for r in results]
    print(f"  成功率     {ok_n}/{len(results)}")
    print(f"  求解时间   avg={sum(times)/len(times):.3f}ms  max={max(times):.3f}ms")
    if fk_errs:
        print(f"  FK误差     avg={sum(fk_errs)/len(fk_errs):.1f}mm  max={max(fk_errs):.1f}mm")
    if real_errs:
        print(f"  实际误差   avg={sum(real_errs)/len(real_errs):.1f}mm  max={max(real_errs):.1f}mm")
