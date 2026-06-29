#!/usr/bin/env python3
"""
SO-100 YOLO 视觉伺服夹取示例 — 不依赖 lerobot，仅用 feetech-servo-sdk + ultralytics + opencv。
Requires: pip install feetech-servo-sdk ultralytics opencv-python

流程：
  1. 移动到俯视姿态（overhead），让相机看向工作区
  2. 启动 YOLO 实时检测线程
  3. 视觉伺服循环：读取目标在画面中的中心 (cx, cy)，
     用 P 控制调整 shoulder_pan(左右) 和 shoulder_lift(前后)，
     直到误差进入死区
  4. 闭合夹爪夹取
  5. 抬起，移动到放置点，松开夹爪
  6. 返回 home 姿态

姿态全部用"校准后语义角度"定义，换机械臂后只需重新跑 6_dfarm_calibrate.py。

Usage:
  python 7_dfarm_yolo_pick.py [port] \
      [--cal dfarm_calibration.json] \
      [--weights ../yolo11x.pt] \
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

# ── 硬件配置 ─────────────────────────────────────────────────────────────────

PORT     = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL = 0

MOTORS = {
    "shoulder_pan":  1,
    "shoulder_lift": 2,
    "elbow_flex":    3,
    "wrist_flex":    4,
    "wrist_roll":    5,
    "gripper":       6,
}

ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]

ADDR = {"torque": 40, "accel": 41, "goal": 42, "lock": 55, "pos": 56,
        "P": 21, "D": 22, "I": 23}

RES    = 4096
CENTER = 2048
FREQ   = 50

DEFAULT_CAL_FILE = "dfarm_calibration.json"

# ── 相机 / 视觉伺服参数 ─────────────────────────────────────────────────────

IMG_W, IMG_H   = 640, 480
TARGET_CX      = 320      # 夹爪正下方在画面中的 x（需标定，先用画面中心）
TARGET_CY      = 240      # 夹爪正下方在画面中的 y（需标定，先用画面中心）
DEAD_ZONE_PX   = 30.0     # 误差小于该值视为对准（像素）
LOCK_MAX_JUMP  = 150      # 跨帧目标跳变阈值（像素）

# IBVS 增益（deg/px），需在实际机械臂上微调
# 增大 → 移动量更大；减小 → 更保守
KP_LIFT = 0.10   # cy 误差 → shoulder_lift 增量
KP_PAN  = 0.08   # cx 误差 → shoulder_pan  增量
MIN_STEP_DEG = 3.0   # 最小移动步长（度），防止接近死区时步长过小陷入震荡

# 单边夹爪偏移：对准后 pan 往左偏移，让开口侧对准物体
# 正值=向左偏，根据夹爪开口方向和物体大小实测调整
GRIPPER_OFFSET_PAN = 10.0   # 度

# 垂直下降参数（固定抓取姿态，工作台高度固定时最可靠）
# 对准后 shoulder_lift 和 elbow_flex 直接覆盖为这两个值完成下降
POSE_PICK = {
    "shoulder_lift": 20.0,    # 实测调整：手臂压到桌面物体的角度
    "elbow_flex":    -5.0,    # 实测调整
}
DESCEND_DURATION = 2.0        # 下压用时（秒）

# ── IK / FK（来自 2_dfarm_ee_control.py，使用原始角度系统 raw=2048为零点）────

L1 = 0.1159
L2 = 0.1350
THETA1_OFF = math.atan2(0.028, 0.11257)
THETA2_OFF = math.atan2(0.0052, 0.1349) + THETA1_OFF

# IK/FK 使用原始角度（raw=2048为零点），与校准角度的偏移量
# 原始角度 = 校准角度 + (homing_raw - 2048) * 360 / 4096
# 这个偏移在运行时从校准文件读取
_RAW_CENTER = 2048
_RAW_RES    = 4096


def _cal2raw_deg(cal_deg: float, motor: str, cal: dict | None) -> float:
    """校准角度 → 原始角度（raw=2048为零点）。"""
    if cal and motor in cal:
        offset = (cal[motor]["homing_raw"] - _RAW_CENTER) * 360.0 / _RAW_RES
        return cal_deg + offset
    return cal_deg


def _raw2cal_deg(raw_deg: float, motor: str, cal: dict | None) -> float:
    """原始角度 → 校准角度。"""
    if cal and motor in cal:
        offset = (cal[motor]["homing_raw"] - _RAW_CENTER) * 360.0 / _RAW_RES
        return raw_deg - offset
    return raw_deg


def ik(x: float, y: float) -> tuple[float, float]:
    """2-link 平面 IK，返回原始角度系统的 (shoulder_lift_deg, elbow_flex_deg)。"""
    r = math.hypot(x, y)
    r = max(abs(L1 - L2) * 1.01, min((L1 + L2) * 0.99, r))
    if r != math.hypot(x, y) and math.hypot(x, y) > 0:
        s = r / math.hypot(x, y)
        x *= s
        y *= s
    c2 = -(r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    t2 = math.pi - math.acos(c2)
    t1 = math.atan2(y, x) + math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))
    j2 = max(-0.1, min(3.45, t1 + THETA1_OFF))
    j3 = max(-0.2, min(math.pi, t2 + THETA2_OFF))
    return 90.0 - math.degrees(j2), math.degrees(j3) - 90.0


def fk(sl_raw_deg: float, ef_raw_deg: float) -> tuple[float, float]:
    """FK，输入原始角度，返回末端 (x, y) 单位米。"""
    t1 = math.radians(90.0 - sl_raw_deg) - THETA1_OFF
    t2 = math.radians(ef_raw_deg + 90.0) - THETA2_OFF
    return (L1 * math.cos(t1) + L2 * math.cos(t1 + t2),
            L1 * math.sin(t1) + L2 * math.sin(t1 + t2))

# ── 关键姿态（语义角度，相对于校准零点） ────────────────────────────────────

POSE_HOME = {
    "shoulder_pan":   0.0,
    "shoulder_lift": -104.0,
    "elbow_flex":     85.0,
    "wrist_flex":     65.0,
    "wrist_roll":     0.0,
    "gripper":        1.0,
}

# 俯视姿态：夹爪面朝下方观察工作区
POSE_OVERHEAD = {
    "shoulder_pan":   0.0,
    "shoulder_lift": -45.0,
    "elbow_flex":     9.0,
    "wrist_flex":     93.0,
    "wrist_roll":     0.0,
    "gripper":        70.0,
}

# 放置点 pan 角度
PLACE_PAN_DEG = 60.0

GRIPPER_OPEN  = 70.0   # 夹爪张开（百分比）
GRIPPER_CLOSE = 0.0    # 夹爪闭合夹取（百分比）


# ── 校准辅助 ─────────────────────────────────────────────────────────────────

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
        raw = int(c["homing_raw"] + deg * RES / 360.0)
        return max(c["range_min"], min(c["range_max"], raw))
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360.0)))


def cal_raw2deg(raw: int, motor: str, cal: dict | None) -> float:
    if cal and motor in cal:
        return (raw - cal[motor]["homing_raw"]) * 360.0 / RES
    return (raw - CENTER) * 360.0 / RES


def gripper_pct2raw(pct: float, cal: dict | None) -> int:
    """夹爪用 0~100 百分比，0=张开，100=闭合，按校准范围线性映射。"""
    pct = max(0.0, min(100.0, pct))
    if cal and "gripper" in cal:
        c = cal["gripper"]
        return int(c["range_min"] + pct / 100.0 * (c["range_max"] - c["range_min"]))
    return int(pct / 100.0 * (RES - 1))


# ── 电机 I/O ────────────────────────────────────────────────────────────────

def read_pos(pkt, port, mid: int) -> int:
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else CENTER


def read_all_cal_deg(pkt, port, cal: dict | None) -> dict[str, float]:
    return {name: cal_raw2deg(read_pos(pkt, port, mid), name, cal)
            for name, mid in MOTORS.items() if name != "gripper"}


def goto_joints(pkt, port, group_write, target_deg: dict[str, float],
                gripper_pct: float | None, duration: float, cal: dict | None):
    """从当前关节角度线性插值到目标，duration 秒内完成。"""
    start = read_all_cal_deg(pkt, port, cal)
    cur_grip_raw = read_pos(pkt, port, MOTORS["gripper"])
    tgt_grip_raw = gripper_pct2raw(gripper_pct, cal) if gripper_pct is not None else cur_grip_raw

    steps = max(1, int(duration * FREQ))
    dt = 1.0 / FREQ
    for step in range(1, steps + 1):
        t0 = time.perf_counter()
        alpha = step / steps
        group_write.clearParam()
        for name in ARM_JOINTS:
            mid = MOTORS[name]
            tgt = target_deg.get(name, start[name])
            deg = start[name] + alpha * (tgt - start[name])
            raw = cal_deg2raw(deg, name, cal)
            group_write.addParam(mid, [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])
        # gripper
        g_raw = int(cur_grip_raw + alpha * (tgt_grip_raw - cur_grip_raw))
        group_write.addParam(MOTORS["gripper"],
                             [scs.SCS_LOBYTE(g_raw), scs.SCS_HIBYTE(g_raw)])
        group_write.txPacket()
        sl = dt - (time.perf_counter() - t0)
        if sl > 0:
            time.sleep(sl)


def send_joints(pkt, port, group_write, joints: dict[str, float],
                gripper_pct: float, cal: dict | None):
    """单次广播写入所有关节 + 夹爪（不做插值，配合视觉伺服循环使用）。"""
    group_write.clearParam()
    for name in ARM_JOINTS:
        raw = cal_deg2raw(joints[name], name, cal)
        group_write.addParam(MOTORS[name], [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])
    g_raw = gripper_pct2raw(gripper_pct, cal)
    group_write.addParam(MOTORS["gripper"], [scs.SCS_LOBYTE(g_raw), scs.SCS_HIBYTE(g_raw)])
    group_write.txPacket()


# ── YOLO 检测线程 ───────────────────────────────────────────────────────────

class DetectionThread:
    """后台线程持续从相机抓帧 + YOLO 推理，主线程随时取最新检测结果。"""

    def __init__(self, weights: str, source: int, conf: float,
                 target_label: str | None, show: bool):
        self.weights = weights
        self.source = source
        self.conf = conf
        self.target_label = target_label
        self.show = show  # 保留参数兼容性，窗口现在始终开启
        self.lock = threading.Lock()
        self.objects: list[dict] = []
        self.frame: np.ndarray | None = None
        self.ready = False
        self.error: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self):
        try:
            model = YOLO(self.weights)
            cap = cv2.VideoCapture(self.source)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
            if not cap.isOpened():
                raise RuntimeError(f"无法打开相机 source={self.source}")
            self.ready = True
            while not self._stop.is_set():
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue
                results = model.predict(frame, conf=self.conf, verbose=False)
                objs = []
                for r in results:
                    for box in r.boxes:
                        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                        cls_id = int(box.cls[0])
                        label = model.names[cls_id]
                        if self.target_label and label != self.target_label:
                            continue
                        objs.append({
                            "label": label,
                            "conf": float(box.conf[0]),
                            "cx": int((x1 + x2) / 2),
                            "cy": int((y1 + y2) / 2),
                            "bbox": (int(x1), int(y1), int(x2), int(y2)),
                        })
                with self.lock:
                    self.objects = objs
                    # 始终生成带标注的 frame，供窗口显示
                    vis = frame.copy()
                    for o in objs:
                        x1, y1, x2, y2 = o["bbox"]
                        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(vis, (o["cx"], o["cy"]), 4, (0, 0, 255), -1)
                        cv2.putText(vis, f"{o['label']} {o['conf']:.2f}",
                                    (x1, max(0, y1 - 6)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                    cv2.drawMarker(vis, (TARGET_CX, TARGET_CY),
                                   (255, 0, 255), cv2.MARKER_CROSS, 20, 2)
                    # 画面中心（白色小圆）
                    cv2.circle(vis, (IMG_W // 2, IMG_H // 2), 5, (255, 255, 255), 1)
                    self.frame = vis
            cap.release()
        except Exception as exc:
            self.error = str(exc)
            self.ready = False

    def get_target(self, last_xy: tuple[int, int] | None) -> tuple[int, int] | None:
        with self.lock:
            objs = list(self.objects)
        if not objs:
            return None
        if last_xy is None:
            best = max(objs, key=lambda o: o["conf"])
        else:
            lx, ly = last_xy
            best = min(objs, key=lambda o: (o["cx"] - lx) ** 2 + (o["cy"] - ly) ** 2)
            d = math.hypot(best["cx"] - lx, best["cy"] - ly)
            if d > LOCK_MAX_JUMP:
                return None
        return best["cx"], best["cy"]


# ── 视觉伺服 + 抓取流程 ─────────────────────────────────────────────────────

def print_detections(det: DetectionThread):
    """打印当前帧的 YOLO 检测结果。"""
    with det.lock:
        objs = list(det.objects)
    if not objs:
        print("  [检测] 当前帧无目标")
    else:
        print(f"  [检测] 共 {len(objs)} 个目标:")
        for o in objs:
            print(f"    {o['label']}  conf={o['conf']:.2f}  center=({o['cx']}, {o['cy']})")


def wait_enter(prompt: str):
    """打印提示并等待用户按 Enter，输入 q 则抛出 KeyboardInterrupt 中止流程。"""
    try:
        val = input(f"\n{prompt} [Enter继续 / q中止] ").strip().lower()
        if val == "q":
            raise KeyboardInterrupt
    except EOFError:
        pass

def visual_servo_ibvs(pkt, port, group_write, det: DetectionThread, cal: dict | None) -> dict | None:
    """
    IBVS 视觉伺服：自动循环，每次将像素误差全量映射到关节角直接移动，
    直到目标进入死区或超过最大尝试次数。

    坐标约定（相机朝正下方）：
      cy < TARGET_CY → 目标在画面上方 → shoulder_lift 增大
      cy > TARGET_CY → 目标在画面下方 → shoulder_lift 减小
      cx < TARGET_CX → 目标在画面左侧 → shoulder_pan 减小
      cx > TARGET_CX → 目标在画面右侧 → shoulder_pan 增大

    wrist_flex 等量反向补偿 shoulder_lift，保持摄像头垂直朝下。
    如方向反了，把 KP_LIFT / KP_PAN 改为负值。
    """
    MAX_ATTEMPTS = 10   # 最多尝试次数，防止无限循环

    print("\n[IBVS视觉伺服] 自动对准中...")
    print(f"  KP_LIFT={KP_LIFT} deg/px  KP_PAN={KP_PAN} deg/px  死区={DEAD_ZONE_PX}px")

    cur = read_all_cal_deg(pkt, port, cal)
    roll = POSE_OVERHEAD["wrist_roll"]
    no_detect_count = 0

    for attempt in range(1, MAX_ATTEMPTS + 1):
        time.sleep(0.4)  # 等待机械臂稳定 + 相机刷新

        # 等待检测结果，最多 3 秒
        det_xy = None
        deadline = time.time() + 3.0
        while time.time() < deadline:
            det_xy = det.get_target(None)
            if det_xy is not None:
                break
            time.sleep(0.05)

        if det_xy is None:
            no_detect_count += 1
            print(f"  [尝试{attempt}] 未检测到目标（连续{no_detect_count}次）")
            if no_detect_count >= 3:
                print("  连续3次未检测到目标，中止。")
                return None
            continue
        no_detect_count = 0

        cx, cy = det_xy
        dx = cx - TARGET_CX
        dy = cy - TARGET_CY
        err = math.hypot(dx, dy)
        print(f"  [尝试{attempt}/{MAX_ATTEMPTS}] 目标=({cx},{cy})  dx={dx:+d}  dy={dy:+d}  err={err:.0f}px", end="")

        if err < DEAD_ZONE_PX:
            print("  ✓ 对准")
            return {
                "shoulder_pan":  cur["shoulder_pan"],
                "shoulder_lift": cur["shoulder_lift"],
                "elbow_flex":    cur["elbow_flex"],
                "wrist_flex":    cur["wrist_flex"],
                "wrist_roll":    roll,
            }

        # 全量映射，一步到位
        # 最小步长只在该轴误差本身很小时才生效，避免大误差轴被干扰
        raw_lift = -dy * KP_LIFT
        raw_pan  =  dx * KP_PAN
        delta_lift = math.copysign(max(abs(raw_lift), MIN_STEP_DEG), raw_lift) if 0 < abs(raw_lift) < MIN_STEP_DEG else raw_lift
        delta_pan  = math.copysign(max(abs(raw_pan),  MIN_STEP_DEG), raw_pan)  if 0 < abs(raw_pan)  < MIN_STEP_DEG else raw_pan
        new_lift = cur["shoulder_lift"] + delta_lift
        new_pan  = cur["shoulder_pan"]  + delta_pan
        new_wf   = cur["wrist_flex"]    - delta_lift

        print(f"  → Δlift={delta_lift:+.1f}°  Δpan={delta_pan:+.1f}°")
        goto_joints(pkt, port, group_write, {
            "shoulder_pan":  new_pan,
            "shoulder_lift": new_lift,
            "elbow_flex":    cur["elbow_flex"],
            "wrist_flex":    new_wf,
            "wrist_roll":    roll,
        }, GRIPPER_OPEN, duration=1.2, cal=cal)
        cur = read_all_cal_deg(pkt, port, cal)

    print(f"  已达最大尝试次数({MAX_ATTEMPTS})，当前误差可能仍超出死区。")
    # 超次数后仍返回当前位置，让用户决定是否继续下降
    return {
        "shoulder_pan":  cur["shoulder_pan"],
        "shoulder_lift": cur["shoulder_lift"],
        "elbow_flex":    cur["elbow_flex"],
        "wrist_flex":    cur["wrist_flex"],
        "wrist_roll":    roll,
    }


def run_pick(pkt, port, group_write, det: DetectionThread, cal: dict | None):
    print("\n=== Step 1/5  移动到俯视姿态（夹爪打开）===")
    print_detections(det)
    wait_enter("准备移动到俯视姿态")
    goto_joints(pkt, port, group_write, POSE_OVERHEAD, GRIPPER_OPEN, duration=2.0, cal=cal)
    time.sleep(0.8)

    print("\n=== Step 2/5  IBVS视觉伺服（自动对准）===")
    print_detections(det)
    wait_enter("准备开始IBVS视觉伺服")

    # 根据目标距画面中心的距离预调整 wrist_flex，补偿不同距离下的视角偏差
    wrist_adjust = -7.0  # 默认值（目标在死区附近）
    time.sleep(0.3)
    initial_det = det.get_target(None)
    if initial_det is not None:
        icx, icy = initial_det
        dist = math.hypot(icx - TARGET_CX, icy - TARGET_CY)
        if dist > 250:
            wrist_adjust = -28.0
        elif dist > 200:
            wrist_adjust = -24.0
        elif dist > 150:
            wrist_adjust = -20.0
        elif dist > 100:
            wrist_adjust = -15.0
        elif dist > 70:
            wrist_adjust = -10.0
        elif dist > 40:
            wrist_adjust = -5.0
        else:
            wrist_adjust = -7.0
        pre = dict(POSE_OVERHEAD)
        pre["wrist_flex"] = POSE_OVERHEAD["wrist_flex"] + wrist_adjust
        print(f"  目标距中心 {dist:.0f}px → wrist_flex 预调整 {wrist_adjust:+.0f}°")
        goto_joints(pkt, port, group_write, pre, GRIPPER_OPEN, duration=0.8, cal=cal)
        time.sleep(0.3)

    aligned = visual_servo_ibvs(pkt, port, group_write, det, cal)
    if aligned is None:
        print("放弃抓取，回到 home。")
        goto_joints(pkt, port, group_write, POSE_HOME, GRIPPER_OPEN, duration=2.0, cal=cal)
        return

    print("\n=== Step 3/5  垂直下降夹取 ===")
    print_detections(det)
    wait_enter("已水平对准，准备垂直下降")

    # 单边夹爪偏移：pan 向左偏移，让开口侧对准物体中心
    pick_pan = aligned["shoulder_pan"] - GRIPPER_OFFSET_PAN
    print(f"  夹爪偏移: pan {aligned['shoulder_pan']:.1f}° → {pick_pan:.1f}°（偏移 -{GRIPPER_OFFSET_PAN}°）")

    # 保持 pan 不变，直接将 shoulder_lift 和 elbow_flex 覆盖为固定抓取角度
    # wrist_flex = aligned 值 + 预调整量，保持下降时夹爪朝向正确
    descend = {
        "shoulder_pan":  pick_pan,
        "shoulder_lift": POSE_PICK["shoulder_lift"],
        "elbow_flex":    POSE_PICK["elbow_flex"],
        "wrist_flex":    aligned["wrist_flex"] + wrist_adjust,
        "wrist_roll":    aligned["wrist_roll"],
    }
    print(f"  抓取姿态: lift={POSE_PICK['shoulder_lift']}°  ef={POSE_PICK['elbow_flex']}°  wf={descend['wrist_flex']:.1f}°")

    # 夹爪已在 overhead 时打开，直接下降
    goto_joints(pkt, port, group_write, descend, GRIPPER_OPEN, duration=DESCEND_DURATION, cal=cal)
    time.sleep(0.3)
    # 夹取
    goto_joints(pkt, port, group_write, descend, GRIPPER_CLOSE, duration=0.8, cal=cal)
    time.sleep(0.5)

    print("\n=== Step 4/5  抬起 + 平移到放置点 ===")
    wait_enter("已夹紧，准备抬起")
    # 先垂直抬起回到对准姿态（夹爪保持闭合）
    goto_joints(pkt, port, group_write, aligned, GRIPPER_CLOSE, duration=DESCEND_DURATION, cal=cal)
    time.sleep(0.3)
    # 再收臂到 overhead 高度
    lift = dict(aligned)
    lift["shoulder_lift"] = POSE_OVERHEAD["shoulder_lift"]
    lift["elbow_flex"]    = POSE_OVERHEAD["elbow_flex"]
    lift["wrist_flex"]    = POSE_OVERHEAD["wrist_flex"]
    goto_joints(pkt, port, group_write, lift, GRIPPER_CLOSE, duration=1.5, cal=cal)
    # 平移到放置点
    place = dict(lift)
    place["shoulder_pan"] = PLACE_PAN_DEG
    goto_joints(pkt, port, group_write, place, GRIPPER_CLOSE, duration=2.0, cal=cal)

    print("\n=== Step 5/5  松开 + 回 home ===")
    wait_enter("已到放置点，准备松开")
    goto_joints(pkt, port, group_write, place, GRIPPER_OPEN, duration=0.6, cal=cal)
    time.sleep(0.3)
    goto_joints(pkt, port, group_write, POSE_HOME, GRIPPER_OPEN, duration=2.5, cal=cal)
    print("\n抓取完成 ✓")


# ── 主入口 ───────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SO-100 YOLO 视觉伺服夹取示例")
    p.add_argument("port", nargs="?", default=PORT, help=f"串口 (默认 {PORT})")
    p.add_argument("--cal", default=None, help=f"校准文件 (默认 {DEFAULT_CAL_FILE})")
    p.add_argument("--weights", default="best.pt",
                   help="YOLO 权重路径，默认同目录的 best.pt")
    p.add_argument("--camera", type=int, default=0, help="相机索引 (默认 0)")
    p.add_argument("--conf", type=float, default=0.25, help="YOLO 置信度阈值")
    p.add_argument("--target", default=None,
                   help="只锁定指定类别 (如 'bottle')，不指定则取置信度最高的目标")
    p.add_argument("--show", action="store_true", help="显示带检测框的实时画面")
    return p.parse_args()


def main():
    args = parse_args()

    cal = load_calibration(args.cal)
    if cal is None:
        print("⚠️  未找到校准文件，使用 CENTER=2048 作为零点。")
        print("   建议先运行: python 6_dfarm_calibrate.py\n")

    # ── 打开串口 + 配置电机 ────────────────────────────────────────────
    print(f"SO-100 YOLO Pick — {args.port}")
    port = scs.PortHandler(args.port)
    if not port.openPort():
        print(f"ERROR: 无法打开 {args.port}")
        sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    for name, mid in MOTORS.items():
        if read_pos(pkt, port, mid) is None:
            print(f"ERROR: '{name}' (id={mid}) 无响应")
            port.closePort()
            sys.exit(1)

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

    group_write = scs.GroupSyncWrite(port, pkt, ADDR["goal"], 2)

    # ── 启动 YOLO ───────────────────────────────────────────────────────
    weights_path = Path(args.weights)
    if not weights_path.is_absolute():
        # 优先从脚本同目录查找
        local = Path(__file__).resolve().parent / weights_path
        if local.exists():
            weights_path = local
    print(f"YOLO 权重: {weights_path}")
    det = DetectionThread(
        weights=str(weights_path),
        source=args.camera,
        conf=args.conf,
        target_label=args.target,
        show=args.show,
    )
    det.start()

    # 等待相机就绪
    print("等待相机就绪...", end="", flush=True)
    deadline = time.time() + 10.0
    while not det.ready and time.time() < deadline:
        if det.error:
            print(f"\nERROR: {det.error}")
            det.stop()
            port.closePort()
            sys.exit(1)
        time.sleep(0.2)
        print(".", end="", flush=True)
    if not det.ready:
        print("\nERROR: 相机/YOLO 启动超时")
        det.stop()
        port.closePort()
        sys.exit(1)
    print(" ✓")

    # ── 始终开启摄像头显示窗口 ──────────────────────────────────────────
    stop_show = threading.Event()

    def show_loop():
        while not stop_show.is_set():
            with det.lock:
                frame = None if det.frame is None else det.frame.copy()
            if frame is not None:
                cv2.imshow("dfarm_yolo_pick", frame)
            if cv2.waitKey(30) & 0xFF == 27:
                stop_show.set()
        cv2.destroyAllWindows()

    show_thread = threading.Thread(target=show_loop, daemon=True)
    show_thread.start()

    try:
        print("\n摄像头窗口已开启。按 Enter 启动抓取流程，q + Enter 退出，窗口内 ESC 关闭画面。")
        while True:
            try:
                cmd = input("\n>> [Enter=pick, q=quit] ").strip().lower()
            except EOFError:
                break
            if cmd == "q":
                break
            try:
                run_pick(pkt, port, group_write, det, cal)
            except KeyboardInterrupt:
                print("\n流程已中止，回到主菜单。")

    except KeyboardInterrupt:
        print("\n中断。")
    finally:
        stop_show.set()
        det.stop()
        cv2.destroyAllWindows()
        group_write.clearParam()
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        port.closePort()
        print("Done.")


if __name__ == "__main__":
    main()
