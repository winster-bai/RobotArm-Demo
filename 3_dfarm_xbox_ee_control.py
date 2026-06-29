#!/usr/bin/env python3
"""
SO-100 end-effector Xbox controller control. No lerobot dependency.
Requires: pip install feetech-servo-sdk pygame

Starts from current position (no initial movement).
Uses 2-link planar IK for cartesian control.
50Hz P-control loop.

Controls:
  Left stick X/Y  - forward/backward, left/right (ee_x, ee_yaw)
  RT / LT         - up / down (ee_y)
  A / Y           - gripper close / open
  X / B           - wrist roll CCW / CW
  Start           - quit

Usage: python so100_xbox_ee_control.py [port]
"""

import math
import sys
import time

import pygame
import scservo_sdk as scs

# ─── Config ──────────────────────────────────────────────────────────────────

PORT     = "/dev/ttyACM1"
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

ADDR = {
    "torque": 40, "accel": 41, "goal": 42,
    "lock":   55, "pos":   56,
    "P": 21, "D": 22, "I": 23,
}

RES    = 4096
CENTER = 2048
FREQ   = 50
KP     = 0.5

# Kinematics
L1          = 0.1159
L2          = 0.1350
THETA1_OFF  = math.atan2(0.028,  0.11257)
THETA2_OFF  = math.atan2(0.0052, 0.1349) + THETA1_OFF

# Reachable annulus (workspace) for the 2-link planar arm
R_MIN = abs(L1 - L2) * 1.01
R_MAX = (L1 + L2) * 0.99

# Step sizes per tick (at max stick deflection)
XY_STEP    = 0.003   # metres
YAW_STEP   = 1.5     # degrees
ROLL_STEP  = 2.0     # degrees
GRIP_STEP  = 2.5     # percent

# Stick dead-zone (normalised -1..1)
DEADZONE = 0.08

# Xbox button indices (pygame / XInput mapping)
BTN_A     = 0
BTN_B     = 1
BTN_X     = 2
BTN_Y     = 3
BTN_START = 7

# Axis indices
AXIS_LX   = 0   # left stick horizontal  (-1=left,  +1=right)
AXIS_LY   = 1   # left stick vertical    (-1=up,    +1=down)  ← pygame inverts Y
AXIS_LT   = 4   # left trigger           (-1=released, +1=full)
AXIS_RT   = 5   # right trigger          (-1=released, +1=full)


# ─── Helpers ─────────────────────────────────────────────────────────────────

def deadzone(v: float, dz: float = DEADZONE) -> float:
    """Apply dead-zone and rescale to full range."""
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def trigger_value(raw: float) -> float:
    """Convert trigger axis (-1..+1) to 0..1."""
    return (raw + 1.0) / 2.0


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
    return (
        L1 * math.cos(t1) + L2 * math.cos(t1 + t2),
        L1 * math.sin(t1) + L2 * math.sin(t1 + t2),
    )


# ─── Motor I/O ───────────────────────────────────────────────────────────────

def deg2raw(d: float) -> int:
    return max(0, min(RES - 1, int(CENTER + d * RES / 360.0)))

def raw2deg(r: int) -> float:
    return (r - CENTER) * 360.0 / RES

def pct2raw(p: float) -> int:
    return int(max(0.0, min(100.0, p)) / 100.0 * (RES - 1))

def read_pos(pkt, port, mid):
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else None

def write_pos(pkt, port, mid, raw: int):
    pkt.write2ByteTxRx(port, mid, ADDR["goal"], max(0, min(RES - 1, raw)))


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else PORT

    # ── pygame / joystick init ──
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("ERROR: no joystick/gamepad detected"); sys.exit(1)
    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Gamepad: {js.get_name()}")

    # ── serial port ──
    print(f"SO-100 Xbox EE Control — {port_name}")
    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: cannot open {port_name}"); sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    # Check motors
    for name, mid in MOTORS.items():
        if read_pos(pkt, port, mid) is None:
            print(f"ERROR: '{name}' (id={mid}) not responding")
            port.closePort(); sys.exit(1)

    # Configure PID (torque off required)
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"],      16)
        pkt.write1ByteTxRx(port, mid, ADDR["D"],      32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"],       0)
        pkt.write1ByteTxRx(port, mid, ADDR["accel"], 254)

    # Read current positions, set Goal = current, enable torque (no movement)
    cur_deg: dict[str, float] = {}
    for name, mid in MOTORS.items():
        raw = read_pos(pkt, port, mid) or CENTER
        cur_deg[name] = raw2deg(raw)
        write_pos(pkt, port, mid, raw)
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   1)

    # Derive EE state from current joint angles
    ee_x, ee_y = fk(cur_deg["shoulder_lift"], cur_deg["elbow_flex"])
    ee_yaw   = cur_deg["shoulder_pan"]
    ee_pitch = cur_deg["wrist_flex"] + cur_deg["shoulder_lift"] + cur_deg["elbow_flex"]
    ee_roll  = cur_deg["wrist_roll"]
    ee_grip  = 50.0

    print("Ready.")
    print("  Left stick    — forward/back (X) + left/right yaw")
    print("  RT            — up   |  LT — down")
    print("  A / Y         — gripper close / open")
    print("  X / B         — roll CCW / CW")
    print("  Start         — quit")

    dt   = 1.0 / FREQ
    quit_flag = False

    try:
        while not quit_flag:
            t0 = time.perf_counter()

            # pump pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    quit_flag = True
                elif event.type == pygame.JOYBUTTONDOWN:
                    if event.button == BTN_START:
                        quit_flag = True

            if quit_flag:
                break

            # ── read axes ──
            lx = deadzone(js.get_axis(AXIS_LX))   # left/right → yaw
            ly = deadzone(js.get_axis(AXIS_LY))   # fwd/back   (pygame: +1 = down)
            lt = trigger_value(js.get_axis(AXIS_LT))
            rt = trigger_value(js.get_axis(AXIS_RT))

            # ── read buttons ──
            btn_a = js.get_button(BTN_A)
            btn_y = js.get_button(BTN_Y)
            btn_x = js.get_button(BTN_X)
            btn_b = js.get_button(BTN_B)

            # ── update EE state ──
            ee_x   -= ly * XY_STEP          # stick up   → forward
            ee_y   += (rt - lt) * XY_STEP   # RT up, LT down
            ee_yaw += lx * YAW_STEP         # stick right → yaw right

            if btn_x: ee_roll -= ROLL_STEP
            if btn_b: ee_roll += ROLL_STEP
            if btn_a: ee_grip  = max(0.0,   ee_grip - GRIP_STEP)
            if btn_y: ee_grip  = min(100.0, ee_grip + GRIP_STEP)

            # Clamp target into reachable workspace and write back (anti-windup)
            ee_x, ee_y = clamp_workspace(ee_x, ee_y)

            # ── IK ──
            sl, ef = ik(ee_x, ee_y)
            wf = -sl - ef + ee_pitch
            targets = {
                "shoulder_pan":  ee_yaw,
                "shoulder_lift": sl,
                "elbow_flex":    ef,
                "wrist_flex":    wf,
                "wrist_roll":    ee_roll,
            }

            # ── P-control → write ──
            for name, mid in MOTORS.items():
                tgt = raw2deg(pct2raw(ee_grip)) if name == "gripper" else targets[name]
                cur_deg[name] += KP * (tgt - cur_deg[name])
                write_pos(pkt, port, mid, deg2raw(cur_deg[name]))

            sl_time = dt - (time.perf_counter() - t0)
            if sl_time > 0:
                time.sleep(sl_time)

    except KeyboardInterrupt:
        pass
    finally:
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        port.closePort()
        pygame.quit()
        print("\nDone.")


if __name__ == "__main__":
    main()
