#!/usr/bin/env python3
"""
SO-100 YOLO 视觉伺服夹取（Pinocchio IK 下降版）
Requires: pip install feetech-servo-sdk ultralytics opencv-python pin

与 8_dfarm_yolo_pick.py 的区别：
  下降阶段用 Pinocchio IK 垂直下降：
    1. IBVS 对准后，用 pinocchio FK 获取当前末端 xyz
    2. 保持 xy 不变，只将 z 降低到桌面高度（PICK_Z）
    3. 用 pinocchio IK 求解下降关节角 → 真正垂直下降

Usage:
  python 8_dfarm_yolo_pick_ik.py [port] \
      [--cal dfarm_calibration.json] \
      [--urdf so101_new_calib.urdf] \
      [--weights best.pt] \
      [--camera 0] [--conf 0.25] [--target bottle]
"""

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import scservo_sdk as scs
from ultralytics import YOLO

# ── 硬件配置 ──────────────────────────────────────────────────────────────────

PORT     = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL = 0

MOTORS = {
    "shoulder_pan":  1, "shoulder_lift": 2, "elbow_flex": 3,
    "wrist_flex":    4, "wrist_roll":    5, "gripper":    6,
}
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ADDR = {"torque": 40, "accel": 41, "goal": 42, "lock": 55, "pos": 56,
        "P": 21, "D": 22, "I": 23}
RES    = 4096
CENTER = 2048
FREQ   = 50

DEFAULT_CAL_FILE  = "dfarm_calibration.json"
DEFAULT_URDF_FILE = "so101_new_calib.urdf"

# ── 视觉伺服参数 ───────────────────────────────────────────────────────────────

IMG_W, IMG_H   = 640, 480
TARGET_CX      = 320
TARGET_CY      = 240
DEAD_ZONE_PX   = 30.0
LOCK_MAX_JUMP  = 150
KP_LIFT        = 0.10
KP_PAN         = 0.08
MIN_STEP_DEG   = 3.0
GRIPPER_OFFSET_PAN = 10.0   # 单边夹爪偏移（度）

# ── IK 下降参数 ────────────────────────────────────────────────────────────────

PICK_Z           = 0.005   # 抓取高度（米），根据实际桌面标定
DESCEND_DURATION = 2.0     # 下降用时（秒）
LIFT_Z_DELTA     = 0.12    # 抬起后的 z 增量（米），抬起高度

# ── 关键姿态 ───────────────────────────────────────────────────────────────────

POSE_HOME = {
    "shoulder_pan": 0.0, "shoulder_lift": -104.0, "elbow_flex": 85.0,
    "wrist_flex":  65.0, "wrist_roll":    0.0,
}
POSE_OVERHEAD = {
    "shoulder_pan": 0.0, "shoulder_lift": -45.0, "elbow_flex": 9.0,
    "wrist_flex":  93.0, "wrist_roll":    0.0,
}
PLACE_PAN_DEG = 60.0
GRIPPER_OPEN  = 70.0
GRIPPER_CLOSE = 0.0

# ── Pinocchio IK/FK ────────────────────────────────────────────────────────────

class SO101IK:
    """Pinocchio 正逆运动学，复用 7_dfarm_ik_xyz.py 的相同逻辑。"""

    def __init__(self, urdf_path: str):
        try:
            import pinocchio as pin
        except ImportError:
            raise ImportError("需要安装 pinocchio: pip install pin")
        self.pin   = pin
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.fid   = self.model.getFrameId("gripper_frame_link")

    def _deg2q(self, joints: dict) -> np.ndarray:
        q = np.zeros(self.model.nq)
        for i, n in enumerate(ARM_JOINTS):
            q[i] = math.radians(joints.get(n, 0.0))
        return np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

    def _fk_pos(self, q: np.ndarray) -> np.ndarray:
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.fid].translation.copy()

    def get_ee_xyz(self, joints: dict) -> tuple[float, float, float]:
        pos = self._fk_pos(self._deg2q(joints))
        return float(pos[0]), float(pos[1]), float(pos[2])

    def solve(self, current: dict, tx: float, ty: float, tz: float,
              max_iters: int = 300, tol: float = 1e-3,
              step: float = 0.6, damp: float = 1e-4) -> tuple[dict, float]:
        target = np.array([tx, ty, tz])
        q = self._deg2q(current)
        g = q[5] if self.model.nq > 5 else 0.0
        for _ in range(max_iters):
            err = target - self._fk_pos(q)
            if np.linalg.norm(err) < tol:
                break
            J = self.pin.computeFrameJacobian(
                self.model, self.data, q, self.fid,
                self.pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)[:3, :5]
            dq = np.zeros(self.model.nv)
            dq[:5] = step * J.T @ np.linalg.solve(J @ J.T + damp * np.eye(3), err)
            q = self.pin.integrate(self.model, q, dq)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
            if self.model.nq > 5:
                q[5] = g
        result  = {n: math.degrees(q[i]) for i, n in enumerate(ARM_JOINTS)}
        err_mm  = float(np.linalg.norm(target - self._fk_pos(q)) * 1000)
        return result, err_mm


# ── 校准辅助 ───────────────────────────────────────────────────────────────────

def load_calibration(cal_path: str | None) -> dict | None:
    path = Path(cal_path) if cal_path else Path(DEFAULT_CAL_FILE)
    if not path.exists():
        return None
    with open(path) as f:
        cal = json.load(f)
    print(f"已加载校准文件: {path}")
    return cal


def cal_deg2raw(deg: float, motor: str, cal: dict | None) -> int:
    if cal and motor in cal:
        c = cal[motor]
        return max(c["range_min"], min(c["range_max"], int(c["homing_raw"] + deg * RES / 360.0)))
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360.0)))


def cal_raw2deg(raw: int, motor: str, cal: dict | None) -> float:
    if cal and motor in cal:
        return (raw - cal[motor]["homing_raw"]) * 360.0 / RES
    return (raw - CENTER) * 360.0 / RES


def gripper_pct2raw(pct: float, cal: dict | None) -> int:
    pct = max(0.0, min(100.0, pct))
    if cal and "gripper" in cal:
        c = cal["gripper"]
        return int(c["range_min"] + pct / 100.0 * (c["range_max"] - c["range_min"]))
    return int(pct / 100.0 * (RES - 1))


# ── 电机 I/O ───────────────────────────────────────────────────────────────────

def read_pos(pkt, port, mid: int) -> int:
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else CENTER


def read_all_cal_deg(pkt, port, cal: dict | None) -> dict[str, float]:
    return {name: cal_raw2deg(read_pos(pkt, port, mid), name, cal)
            for name, mid in MOTORS.items() if name != "gripper"}


def goto_joints(pkt, port, gw, target_deg: dict, gripper_pct: float | None,
                duration: float, cal: dict | None):
    start        = read_all_cal_deg(pkt, port, cal)
    cur_grip_raw = read_pos(pkt, port, MOTORS["gripper"])
    tgt_grip_raw = gripper_pct2raw(gripper_pct, cal) if gripper_pct is not None else cur_grip_raw
    steps = max(1, int(duration * FREQ))
    dt    = 1.0 / FREQ
    for step in range(1, steps + 1):
        t0    = time.perf_counter()
        alpha = step / steps
        gw.clearParam()
        for name in ARM_JOINTS:
            deg = start[name] + alpha * (target_deg.get(name, start[name]) - start[name])
            raw = cal_deg2raw(deg, name, cal)
            gw.addParam(MOTORS[name], [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])
        g_raw = int(cur_grip_raw + alpha * (tgt_grip_raw - cur_grip_raw))
        gw.addParam(MOTORS["gripper"], [scs.SCS_LOBYTE(g_raw), scs.SCS_HIBYTE(g_raw)])
        gw.txPacket()
        sl = dt - (time.perf_counter() - t0)
        if sl > 0:
            time.sleep(sl)

# ── YOLO 检测线程（与原版完全相同）───────────────────────────────────────────

class DetectionThread:
    def __init__(self, weights, source, conf, target_label, show):
        self.lock = threading.Lock()
        self.objects: list[dict] = []
        self.frame   = None
        self.ready   = False
        self.error   = None
        self._stop   = threading.Event()
        self._weights     = weights
        self._source      = source
        self._conf        = conf
        self._target_label = target_label

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def stop(self):
        self._stop.set()

    def _run(self):
        try:
            model = YOLO(self._weights)
            cap   = cv2.VideoCapture(self._source)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
            if not cap.isOpened():
                raise RuntimeError(f"无法打开相机 source={self._source}")
            self.ready = True
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05); continue
                results = model.predict(frame, conf=self._conf, verbose=False)
                objs = []
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        label = model.names[int(box.cls[0])]
                        if self._target_label and label != self._target_label:
                            continue
                        objs.append({
                            "label": label, "conf": float(box.conf[0]),
                            "cx": int((x1 + x2) / 2), "cy": int((y1 + y2) / 2),
                            "bbox": (int(x1), int(y1), int(x2), int(y2)),
                        })
                vis = frame.copy()
                for o in objs:
                    x1, y1, x2, y2 = o["bbox"]
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(vis, (o["cx"], o["cy"]), 4, (0, 0, 255), -1)
                cv2.drawMarker(vis, (TARGET_CX, TARGET_CY), (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
                with self.lock:
                    self.objects = objs
                    self.frame   = vis
            cap.release()
        except Exception as e:
            self.error = str(e); self.ready = False

    def get_target(self, last_xy):
        with self.lock:
            objs = list(self.objects)
        if not objs:
            return None
        if last_xy is None:
            best = max(objs, key=lambda o: o["conf"])
        else:
            lx, ly = last_xy
            best = min(objs, key=lambda o: (o["cx"] - lx)**2 + (o["cy"] - ly)**2)
            if math.hypot(best["cx"] - lx, best["cy"] - ly) > LOCK_MAX_JUMP:
                return None
        return best["cx"], best["cy"]


# ── IBVS 视觉伺服（与原版相同）────────────────────────────────────────────────

def visual_servo_ibvs(pkt, port, gw, det, cal):
    MAX_ATTEMPTS = 10
    print("\n[IBVS] 自动对准中...")
    cur  = read_all_cal_deg(pkt, port, cal)
    roll = POSE_OVERHEAD["wrist_roll"]
    no_det = 0

    for attempt in range(1, MAX_ATTEMPTS + 1):
        time.sleep(0.4)
        det_xy = None
        deadline = time.time() + 3.0
        while time.time() < deadline:
            det_xy = det.get_target(None)
            if det_xy: break
            time.sleep(0.05)

        if det_xy is None:
            no_det += 1
            if no_det >= 3:
                print("  连续3次未检测到目标，中止。"); return None
            continue
        no_det = 0

        cx, cy = det_xy
        dx, dy = cx - TARGET_CX, cy - TARGET_CY
        err    = math.hypot(dx, dy)
        print(f"  [{attempt}/{MAX_ATTEMPTS}] err={err:.0f}px", end="")

        if err < DEAD_ZONE_PX:
            print("  ✓")
            return {"shoulder_pan": cur["shoulder_pan"], "shoulder_lift": cur["shoulder_lift"],
                    "elbow_flex": cur["elbow_flex"], "wrist_flex": cur["wrist_flex"], "wrist_roll": roll}

        raw_lift   = -dy * KP_LIFT
        raw_pan    =  dx * KP_PAN
        delta_lift = math.copysign(max(abs(raw_lift), MIN_STEP_DEG), raw_lift) if 0 < abs(raw_lift) < MIN_STEP_DEG else raw_lift
        delta_pan  = math.copysign(max(abs(raw_pan),  MIN_STEP_DEG), raw_pan)  if 0 < abs(raw_pan)  < MIN_STEP_DEG else raw_pan
        print(f"  Δlift={delta_lift:+.1f}° Δpan={delta_pan:+.1f}°")
        goto_joints(pkt, port, gw, {
            "shoulder_pan":  cur["shoulder_pan"]  + delta_pan,
            "shoulder_lift": cur["shoulder_lift"] + delta_lift,
            "elbow_flex":    cur["elbow_flex"],
            "wrist_flex":    cur["wrist_flex"]    - delta_lift,
            "wrist_roll":    roll,
        }, GRIPPER_OPEN, 1.2, cal)
        cur = read_all_cal_deg(pkt, port, cal)

    print(f"  已达最大尝试次数({MAX_ATTEMPTS})。")
    return {"shoulder_pan": cur["shoulder_pan"], "shoulder_lift": cur["shoulder_lift"],
            "elbow_flex": cur["elbow_flex"], "wrist_flex": cur["wrist_flex"], "wrist_roll": roll}


# ── 抓取流程 ───────────────────────────────────────────────────────────────────

def run_pick(pkt, port, gw, det, cal, kin: SO101IK):
    def wait_enter(prompt):
        try:
            val = input(f"\n{prompt} [Enter继续 / q中止] ").strip().lower()
            if val == "q": raise KeyboardInterrupt
        except EOFError:
            pass

    # Step 1: 移动到俯视姿态
    print("\n=== Step 1  移动到俯视姿态（夹爪打开）===")
    wait_enter("准备移动到俯视姿态")
    goto_joints(pkt, port, gw, POSE_OVERHEAD, GRIPPER_OPEN, 2.0, cal)
    time.sleep(0.8)

    # Step 2: wrist_flex 预调整 + IBVS 对准
    print("\n=== Step 2  IBVS 视觉伺服对准 ===")
    wait_enter("准备开始对准")
    wrist_adjust = -7.0
    time.sleep(0.3)
    xy0 = det.get_target(None)
    if xy0:
        dist = math.hypot(xy0[0] - TARGET_CX, xy0[1] - TARGET_CY)
        wrist_adjust = (-28 if dist > 250 else -24 if dist > 200 else
                        -20 if dist > 150 else -15 if dist > 100 else
                        -10 if dist > 70  else  -5 if dist > 40  else -7.0)
        pre = dict(POSE_OVERHEAD)
        pre["wrist_flex"] += wrist_adjust
        print(f"  wrist_flex 预调整 {wrist_adjust:+.0f}°")
        goto_joints(pkt, port, gw, pre, GRIPPER_OPEN, 0.8, cal)
        time.sleep(0.3)

    aligned = visual_servo_ibvs(pkt, port, gw, det, cal)
    if aligned is None:
        goto_joints(pkt, port, gw, POSE_HOME, GRIPPER_OPEN, 2.0, cal)
        return

    # Step 3: IK 垂直下降夹取
    print("\n=== Step 3  IK 垂直下降夹取 ===")
    wait_enter("已对准，准备垂直下降")

    # 读取对准后的末端 xyz
    cur_joints = dict(aligned)
    cur_joints["gripper"] = GRIPPER_OPEN
    ee_x, ee_y, ee_z = kin.get_ee_xyz(aligned)
    print(f"  对准末端: x={ee_x:.4f}  y={ee_y:.4f}  z={ee_z:.4f}")

    # 单边夹爪偏移
    pick_pan = aligned["shoulder_pan"] - GRIPPER_OFFSET_PAN
    aligned_offset = dict(aligned)
    aligned_offset["shoulder_pan"] = pick_pan

    # 用 IK 计算垂直下降目标（保持 xy，z 降到桌面）
    descend_sol, err_mm = kin.solve(aligned_offset, ee_x, ee_y, PICK_Z)
    descend_sol["wrist_flex"] = aligned["wrist_flex"] + wrist_adjust
    descend_sol["wrist_roll"] = aligned["wrist_roll"]
    print(f"  IK 下降目标: z={PICK_Z:.4f}m  IK_err={err_mm:.1f}mm")

    goto_joints(pkt, port, gw, descend_sol, GRIPPER_OPEN, DESCEND_DURATION, cal)
    time.sleep(0.3)
    goto_joints(pkt, port, gw, descend_sol, GRIPPER_CLOSE, 0.8, cal)
    time.sleep(0.5)

    # Step 4: IK 垂直抬起 + 平移放置
    print("\n=== Step 4  抬起 + 放置 ===")
    wait_enter("已夹紧，准备抬起")

    # 抬起：z 增加 LIFT_Z_DELTA
    lift_sol, _ = kin.solve(descend_sol, ee_x, ee_y, PICK_Z + LIFT_Z_DELTA)
    lift_sol["wrist_flex"] = aligned["wrist_flex"] + wrist_adjust
    lift_sol["wrist_roll"] = aligned["wrist_roll"]
    goto_joints(pkt, port, gw, lift_sol, GRIPPER_CLOSE, DESCEND_DURATION, cal)
    time.sleep(0.3)

    # 收臂到 overhead 高度再平移
    retract = dict(POSE_OVERHEAD)
    retract["shoulder_pan"] = pick_pan
    goto_joints(pkt, port, gw, retract, GRIPPER_CLOSE, 1.5, cal)
    place = dict(retract)
    place["shoulder_pan"] = PLACE_PAN_DEG
    goto_joints(pkt, port, gw, place, GRIPPER_CLOSE, 2.0, cal)

    # Step 5: 松开 + 回 home
    print("\n=== Step 5  松开 + 回 home ===")
    wait_enter("已到放置点，准备松开")
    goto_joints(pkt, port, gw, place, GRIPPER_OPEN, 0.6, cal)
    time.sleep(0.3)
    goto_joints(pkt, port, gw, POSE_HOME, GRIPPER_OPEN, 2.5, cal)
    print("\n抓取完成 ✓")


# ── 主入口 ─────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(description="SO-100 YOLO Pick (Pinocchio IK)")
    p.add_argument("port",     nargs="?", default=PORT)
    p.add_argument("--cal",    default=None)
    p.add_argument("--urdf",   default=DEFAULT_URDF_FILE)
    p.add_argument("--weights",default="best.pt")
    p.add_argument("--camera", type=int, default=0)
    p.add_argument("--conf",   type=float, default=0.25)
    p.add_argument("--target", default=None)
    p.add_argument("--show",   action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    cal  = load_calibration(args.cal)
    if cal is None:
        print("⚠  未找到校准文件，建议先运行: python 1_dfarm_calibrate.py\n")

    # 加载 Pinocchio IK
    urdf_path = Path(args.urdf)
    if not urdf_path.is_absolute():
        urdf_path = Path(__file__).parent / urdf_path
    print(f"URDF: {urdf_path}")
    kin = SO101IK(str(urdf_path))

    # 串口初始化
    print(f"SO-100 YOLO Pick IK — {args.port}")
    port = scs.PortHandler(args.port)
    if not port.openPort():
        print(f"ERROR: 无法打开 {args.port}"); sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"],      16)
        pkt.write1ByteTxRx(port, mid, ADDR["D"],      32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"],       0)
        pkt.write1ByteTxRx(port, mid, ADDR["accel"], 100)
        raw = read_pos(pkt, port, mid)
        pkt.write2ByteTxRx(port, mid, ADDR["goal"], raw)
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   1)

    gw = scs.GroupSyncWrite(port, pkt, ADDR["goal"], 2)

    # YOLO 检测
    weights_path = Path(args.weights)
    if not weights_path.is_absolute():
        local = Path(__file__).parent / weights_path
        if local.exists():
            weights_path = local
    det = DetectionThread(str(weights_path), args.camera, args.conf, args.target, args.show)
    det.start()

    print("等待相机就绪...", end="", flush=True)
    deadline = time.time() + 10.0
    while not det.ready and time.time() < deadline:
        if det.error:
            print(f"\nERROR: {det.error}"); det.stop(); port.closePort(); sys.exit(1)
        time.sleep(0.2); print(".", end="", flush=True)
    if not det.ready:
        print("\nERROR: 相机启动超时"); det.stop(); port.closePort(); sys.exit(1)
    print(" ✓")

    # 显示窗口
    stop_show = threading.Event()
    def show_loop():
        while not stop_show.is_set():
            with det.lock:
                frame = None if det.frame is None else det.frame.copy()
            if frame is not None:
                cv2.imshow("dfarm_yolo_pick_ik", frame)
            if cv2.waitKey(30) & 0xFF == 27:
                stop_show.set()
        cv2.destroyAllWindows()
    threading.Thread(target=show_loop, daemon=True).start()

    try:
        print("\n摄像头窗口已开启。Enter=开始抓取，q=退出")
        while True:
            try:
                cmd = input("\n>> [Enter=pick, q=quit] ").strip().lower()
            except EOFError:
                break
            if cmd == "q": break
            try:
                run_pick(pkt, port, gw, det, cal, kin)
            except KeyboardInterrupt:
                print("\n流程已中止。")
    except KeyboardInterrupt:
        pass
    finally:
        stop_show.set()
        det.stop()
        cv2.destroyAllWindows()
        gw.clearParam()
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        port.closePort()
        print("Done.")


if __name__ == "__main__":
    main()
