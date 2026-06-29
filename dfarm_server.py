#!/usr/bin/env python3
"""
DFarm 机械臂控制 API 服务 — 独立版，仅依赖 feetech-servo-sdk + flask。
Requires: pip install feetech-servo-sdk flask

Usage:
  python dfarm_server.py [port] [--cal dfarm_calibration.json] [--api-port 8001]

Endpoints:
  GET  /joints                          查询当前关节角度
  POST /move/joints  {"angles":[...]}   按6个角度运动
  POST /move/pose    {"name":"home"}    移动到命名姿态
"""

import atexit
import json
import sys
import threading
import time
from pathlib import Path

import scservo_sdk as scs
from flask import Flask, jsonify, request

PORT     = "/dev/ttyACM0"
BAUDRATE = 1_000_000
FREQ     = 50

MOTORS = {
    "shoulder_pan":  1, "shoulder_lift": 2, "elbow_flex": 3,
    "wrist_flex":    4, "wrist_roll":    5, "gripper":    6,
}
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ADDR = {"torque": 40, "accel": 41, "goal": 42, "lock": 55, "pos": 56, "P": 21, "D": 22, "I": 23}
RES, CENTER = 4096, 2048

POSES = {
    "home":     {"shoulder_pan": 0, "shoulder_lift": -104, "elbow_flex": 85,  "wrist_flex": 65, "wrist_roll": 0, "gripper": 1},
    "overhead": {"shoulder_pan": 0, "shoulder_lift": -45,  "elbow_flex": 9,   "wrist_flex": 93, "wrist_roll": 0, "gripper": 160},
}
# gripper 约定（基于 dfarm_calibration.json）：homing 在全开端，
# 0°≈完全张开，角度越大越闭合，约 323° 完全闭合（160°≈半闭，适合抓取）。


def _load_cal(path):
    p = Path(path)
    return json.loads(p.read_text()) if p.exists() else None

def _deg2raw(deg, motor, cal):
    if cal and motor in cal:
        c = cal[motor]
        return max(c["range_min"], min(c["range_max"], int(c["homing_raw"] + deg * RES / 360)))
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360)))

def _raw2deg(raw, motor, cal):
    if cal and motor in cal:
        return (raw - cal[motor]["homing_raw"]) * 360 / RES
    return (raw - CENTER) * 360 / RES


class ArmController:
    def __init__(self, port_name, cal):
        self.cal  = cal
        self.lock = threading.Lock()
        self._port = scs.PortHandler(port_name)
        if not self._port.openPort():
            raise RuntimeError(f"无法打开串口 {port_name}")
        self._port.setBaudRate(BAUDRATE)
        self._pkt = scs.PacketHandler(0)
        self._gw  = scs.GroupSyncWrite(self._port, self._pkt, ADDR["goal"], 2)
        for mid in MOTORS.values():
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["torque"], 0)
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["lock"],   0)
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["P"],      16)
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["D"],      32)
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["I"],       0)
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["accel"], 150)
            raw = self._read_raw(mid)
            self._pkt.write2ByteTxRx(self._port, mid, ADDR["goal"], raw)
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["torque"], 1)
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["lock"],   1)

    def close(self):
        for mid in MOTORS.values():
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["torque"], 0)
            self._pkt.write1ByteTxRx(self._port, mid, ADDR["lock"],   0)
        self._port.closePort()

    def _read_raw(self, mid):
        v, c, _ = self._pkt.read2ByteTxRx(self._port, mid, ADDR["pos"])
        return v if c == scs.COMM_SUCCESS else CENTER

    def read_all(self):
        return {name: _raw2deg(self._read_raw(mid), name, self.cal) for name, mid in MOTORS.items()}

    def _send(self, joints, gripper):
        self._gw.clearParam()
        for name in ARM_JOINTS:
            raw = _deg2raw(joints[name], name, self.cal)
            self._gw.addParam(MOTORS[name], [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)])
        g = _deg2raw(gripper, "gripper", self.cal)
        self._gw.addParam(MOTORS["gripper"], [scs.SCS_LOBYTE(g), scs.SCS_HIBYTE(g)])
        self._gw.txPacket()

    def goto(self, target, gripper, duration=2.0):
        start = self.read_all()
        g0    = start.get("gripper", 50)
        steps = max(1, int(duration * FREQ))
        for i in range(1, steps + 1):
            t0 = time.perf_counter()
            a  = i / steps
            j  = {n: start[n] + a * (target.get(n, start[n]) - start[n]) for n in ARM_JOINTS}
            self._send(j, g0 + a * (gripper - g0))
            sl = 1 / FREQ - (time.perf_counter() - t0)
            if sl > 0: time.sleep(sl)


def create_app(arm: ArmController) -> Flask:
    app = Flask(__name__)

    @app.get("/joints")
    def joints():
        with arm.lock:
            return jsonify(arm.read_all())

    @app.post("/move/joints")
    def move_joints():
        body    = request.get_json(force=True) or {}
        angles  = body.get("angles", [])
        if len(angles) != 6:
            return jsonify({"error": "需要6个角度值"}), 400
        target  = {j: float(angles[i]) for i, j in enumerate(ARM_JOINTS)}
        gripper = float(angles[5])
        duration = float(body.get("duration", 2.0))
        threading.Thread(target=lambda: arm.goto(target, gripper, duration), daemon=True).start()
        return jsonify({"ok": True})

    @app.post("/move/pose")
    def move_pose():
        body = request.get_json(force=True) or {}
        name = body.get("name", "home")
        if name not in POSES:
            return jsonify({"error": f"未知姿态 {name}，可用: {list(POSES)}"}), 400
        pose    = POSES[name]
        gripper = pose.get("gripper", 50)
        duration = float(body.get("duration", 2.0))
        threading.Thread(target=lambda: arm.goto(pose, gripper, duration), daemon=True).start()
        return jsonify({"ok": True, "pose": name})

    @app.errorhandler(Exception)
    def err(e):
        return jsonify({"error": str(e)}), 400

    return app


def main():
    args      = sys.argv[1:]
    port_name = PORT
    cal_path  = "dfarm_calibration.json"
    api_port  = 8001
    i = 0
    while i < len(args):
        if args[i] == "--cal" and i + 1 < len(args):
            cal_path = args[i + 1]; i += 2
        elif args[i] == "--api-port" and i + 1 < len(args):
            api_port = int(args[i + 1]); i += 2
        elif not args[i].startswith("--"):
            port_name = args[i]; i += 1
        else:
            i += 1

    cal = _load_cal(cal_path)
    print(f"DFarm Server — {port_name}  cal={'已加载' if cal else '未找到'}  api=http://127.0.0.1:{api_port}")

    arm = ArmController(port_name, cal)
    atexit.register(arm.close)

    app = create_app(arm)
    app.run(host="127.0.0.1", port=api_port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
