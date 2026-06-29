#!/usr/bin/env python3
"""
SO-100 end-effector keyboard control. No lerobot dependency.
Requires: pip install feetech-servo-sdk pynput

Starts from current position (no initial movement).
Uses 2-link planar IK for cartesian control.
50Hz P-control loop, pynput for smooth held-key detection.

Controls:
  W/S - forward / backward (x)
  E/D - up / down (y)
  Q/A - left / right (yaw)
  R/F - pitch up / down
  T/G - roll CW / CCW
  Y/H - gripper close / open
  ESC - quit

Usage: python so100_ee_control.py [port]
"""

import math
import sys
import time
from queue import Queue

import scservo_sdk as scs
from pynput import keyboard

# ─── Config ──────────────────────────────────────────────────────────────────

PORT = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL = 0

MOTORS = {
    "shoulder_pan": 1, "shoulder_lift": 2, "elbow_flex": 3,
    "wrist_flex": 4, "wrist_roll": 5, "gripper": 6,
}

ADDR = {"torque": 40, "accel": 41, "goal": 42, "lock": 55, "pos": 56,
        "P": 21, "D": 22, "I": 23}

RES = 4096
CENTER = 2048
FREQ = 50
KP = 0.5

# Kinematics
L1 = 0.1159
L2 = 0.1350
THETA1_OFF = math.atan2(0.028, 0.11257)
THETA2_OFF = math.atan2(0.0052, 0.1349) + THETA1_OFF

# Reachable annulus (workspace) for the 2-link planar arm
R_MIN = abs(L1 - L2) * 1.01
R_MAX = (L1 + L2) * 0.99

# EE step sizes per tick
XY_STEP = 0.002
YAW_STEP = 1.0
PITCH_STEP = 1.0
ROLL_STEP = 1.5
GRIP_STEP = 2.0


# ─── Keyboard ────────────────────────────────────────────────────────────────

class KB:
    def __init__(self):
        self._q = Queue()
        self._held = {}
        self.quit = False
        self._l = keyboard.Listener(on_press=self._p, on_release=self._r)

    def start(self): self._l.start()
    def stop(self): self._l.stop()

    def _p(self, k):
        try:
            if k.char: self._q.put((k.char, True))
        except AttributeError: pass

    def _r(self, k):
        if k == keyboard.Key.esc: self.quit = True; return
        try:
            if k.char: self._q.put((k.char, False))
        except AttributeError: pass

    def pressed(self) -> set[str]:
        while not self._q.empty():
            c, down = self._q.get_nowait()
            if down: self._held[c] = True
            else: self._held.pop(c, None)
        return set(self._held)


# ─── IK / FK ─────────────────────────────────────────────────────────────────

def clamp_workspace(x: float, y: float) -> tuple[float, float]:
    """Clamp (x, y) into the reachable annulus so ee state can't wind up."""
    r = math.hypot(x, y)
    if r < 1e-9:
        return R_MIN, 0.0
    r_c = max(R_MIN, min(R_MAX, r))
    if r_c != r:
        s = r_c / r
        return x * s, y * s
    return x, y


def ik(x: float, y: float) -> tuple[float, float]:
    """Returns (shoulder_lift_deg, elbow_flex_deg) in SO-100 convention."""
    r = math.hypot(x, y)
    c2 = -(r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    c2 = max(-1.0, min(1.0, c2))
    t2 = math.pi - math.acos(c2)
    t1 = math.atan2(y, x) + math.atan2(L2 * math.sin(t2), L1 + L2 * math.cos(t2))

    j2 = max(-0.1, min(3.45, t1 + THETA1_OFF))
    j3 = max(-0.2, min(math.pi, t2 + THETA2_OFF))
    return 90.0 - math.degrees(j2), math.degrees(j3) - 90.0


def fk(sl_deg: float, ef_deg: float) -> tuple[float, float]:
    """Returns (x, y) from joint angles."""
    t1 = math.radians(90.0 - sl_deg) - THETA1_OFF
    t2 = math.radians(ef_deg + 90.0) - THETA2_OFF
    return (L1 * math.cos(t1) + L2 * math.cos(t1 + t2),
            L1 * math.sin(t1) + L2 * math.sin(t1 + t2))


# ─── Motor I/O ───────────────────────────────────────────────────────────────

def deg2raw(d): return max(0, min(RES - 1, int(CENTER + d * RES / 360.0)))
def raw2deg(r): return (r - CENTER) * 360.0 / RES
def pct2raw(p): return int(max(0.0, min(100.0, p)) / 100.0 * (RES - 1))

def read_pos(pkt, port, mid):
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else None

def write_pos(pkt, port, mid, raw):
    pkt.write2ByteTxRx(port, mid, ADDR["goal"], max(0, min(RES - 1, raw)))


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else PORT
    print(f"SO-100 EE Control — {port_name}")

    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: cannot open {port_name}"); sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    # Check motors
    for name, mid in MOTORS.items():
        if read_pos(pkt, port, mid) is None:
            print(f"ERROR: '{name}' (id={mid}) not responding"); port.closePort(); sys.exit(1)

    # Configure PID (torque off required)
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"], 16)
        pkt.write1ByteTxRx(port, mid, ADDR["D"], 32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["accel"], 254)

    # Read current positions, set Goal = current, enable torque (no movement)
    cur_deg = {}
    for name, mid in MOTORS.items():
        raw = read_pos(pkt, port, mid) or CENTER
        cur_deg[name] = raw2deg(raw)
        write_pos(pkt, port, mid, raw)
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"], 1)

    # Derive EE state from current joint angles
    ee_x, ee_y = fk(cur_deg["shoulder_lift"], cur_deg["elbow_flex"])
    ee_yaw = cur_deg["shoulder_pan"]
    ee_pitch = cur_deg["wrist_flex"] + cur_deg["shoulder_lift"] + cur_deg["elbow_flex"]
    ee_roll = cur_deg["wrist_roll"]
    ee_grip = 50.0

    print("Ready. Controls: W/S=fwd/back E/D=up/down Q/A=left/right")
    print("  R/F=pitch T/G=roll Y/H=gripper ESC=quit")

    kb = KB()
    kb.start()
    dt = 1.0 / FREQ

    try:
        while not kb.quit:
            t0 = time.perf_counter()
            p = kb.pressed()

            if 'w' in p: ee_x += XY_STEP
            if 's' in p: ee_x -= XY_STEP
            if 'e' in p: ee_y += XY_STEP
            if 'd' in p: ee_y -= XY_STEP
            if 'q' in p: ee_yaw -= YAW_STEP
            if 'a' in p: ee_yaw += YAW_STEP
            if 'r' in p: ee_pitch += PITCH_STEP
            if 'f' in p: ee_pitch -= PITCH_STEP
            if 't' in p: ee_roll -= ROLL_STEP
            if 'g' in p: ee_roll += ROLL_STEP
            if 'y' in p: ee_grip = max(0.0, ee_grip - GRIP_STEP)
            if 'h' in p: ee_grip = min(100.0, ee_grip + GRIP_STEP)

            # Clamp target into reachable workspace and write back (anti-windup)
            ee_x, ee_y = clamp_workspace(ee_x, ee_y)

            # IK
            sl, ef = ik(ee_x, ee_y)
            wf = -sl - ef + ee_pitch
            targets = {
                "shoulder_pan": ee_yaw, "shoulder_lift": sl,
                "elbow_flex": ef, "wrist_flex": wf, "wrist_roll": ee_roll,
            }

            # P-control
            for name, mid in MOTORS.items():
                if name == "gripper":
                    tgt = raw2deg(pct2raw(ee_grip))
                else:
                    tgt = targets[name]
                cur_deg[name] += KP * (tgt - cur_deg[name])
                write_pos(pkt, port, mid, deg2raw(cur_deg[name]))

            sl_time = dt - (time.perf_counter() - t0)
            if sl_time > 0: time.sleep(sl_time)

    except KeyboardInterrupt:
        pass
    finally:
        kb.stop()
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"], 0)
        port.closePort()
        print("\nDone.")


if __name__ == "__main__":
    main()
