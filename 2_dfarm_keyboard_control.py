#!/usr/bin/env python3
"""
SO-100 keyboard joint control. No lerobot dependency.
Requires: pip install feetech-servo-sdk pynput

Starts from current position (no initial movement).
50Hz P-control loop, pynput for smooth held-key detection.

Controls:
  Q/A - shoulder_pan  -/+
  W/S - shoulder_lift -/+
  E/D - elbow_flex    -/+
  R/F - wrist_flex    -/+
  T/G - wrist_roll    -/+
  Y/H - gripper       -/+
  0   - center all joints
  ESC - quit

Usage: python so100_keyboard_control.py [port]
"""

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
KP = 0.6
STEP = 4  # raw steps per tick (~0.35°)

KEY_MAP = {
    'q': ("shoulder_pan", -1), 'a': ("shoulder_pan", +1),
    'w': ("shoulder_lift", -1), 's': ("shoulder_lift", +1),
    'e': ("elbow_flex", -1), 'd': ("elbow_flex", +1),
    'r': ("wrist_flex", -1), 'f': ("wrist_flex", +1),
    't': ("wrist_roll", -1), 'g': ("wrist_roll", +1),
    'y': ("gripper", -1), 'h': ("gripper", +1),
}


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


# ─── Motor I/O ───────────────────────────────────────────────────────────────

def read_pos(pkt, port, mid):
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else None

def write_pos(pkt, port, mid, raw):
    pkt.write2ByteTxRx(port, mid, ADDR["goal"], max(0, min(RES - 1, raw)))


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else PORT
    print(f"SO-100 Joint Control — {port_name}")

    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: cannot open {port_name}"); sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    # Check motors exist
    for name, mid in MOTORS.items():
        if read_pos(pkt, port, mid) is None:
            print(f"ERROR: '{name}' (id={mid}) not responding"); port.closePort(); sys.exit(1)

    # Configure PID (requires torque off)
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"], 16)
        pkt.write1ByteTxRx(port, mid, ADDR["D"], 32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["accel"], 254)

    # Read current positions, then enable torque with Goal = current (no movement)
    targets = {}
    current = {}
    for name, mid in MOTORS.items():
        pos = float(read_pos(pkt, port, mid) or CENTER)
        targets[name] = pos
        current[name] = pos
        write_pos(pkt, port, mid, int(pos))  # set goal before torque on
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"], 1)

    print("Ready. Controls: Q/A W/S E/D R/F T/G Y/H | 0=center | ESC=quit")

    kb = KB()
    kb.start()
    dt = 1.0 / FREQ

    try:
        while not kb.quit:
            t0 = time.perf_counter()
            pressed = kb.pressed()

            if '0' in pressed:
                for n in MOTORS: targets[n] = float(CENTER)

            for ch in pressed:
                if ch in KEY_MAP:
                    name, d = KEY_MAP[ch]
                    targets[name] = max(0.0, min(RES - 1.0, targets[name] + d * STEP))

            for name, mid in MOTORS.items():
                current[name] += KP * (targets[name] - current[name])
                write_pos(pkt, port, mid, int(current[name]))

            sl = dt - (time.perf_counter() - t0)
            if sl > 0: time.sleep(sl)
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
