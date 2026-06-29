#!/usr/bin/env python3
"""
SO-100 Interactive Torque Toggle — enable/disable torque per joint.
Requires: pip install feetech-servo-sdk

Useful for manually posing the arm (limp mode) then locking it in place.

Commands:
  1-6        toggle torque on/off for that joint
  a          enable  ALL joints
  r          release ALL joints (free-move / limp mode)
  p          print current positions
  q / Ctrl+C quit (releases all torque before exit)

Usage: python 5_so100_torque_toggle.py [port]
"""

import sys
import scservo_sdk as scs

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

ADDR = {"torque": 40, "lock": 55, "pos": 56,
        "P": 21, "D": 22, "I": 23, "accel": 41}

RES    = 4096
CENTER = 2048

ID_TO_NAME = {v: k for k, v in MOTORS.items()}


def raw2deg(r: int) -> float:
    return (r - CENTER) * 360.0 / RES

def read_pos(pkt, port, mid):
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else None

def set_torque(pkt, port, mid: int, enable: bool):
    val = 1 if enable else 0
    pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
    pkt.write1ByteTxRx(port, mid, ADDR["torque"], val)
    pkt.write1ByteTxRx(port, mid, ADDR["lock"],   val)  # re-lock when enabling

def print_status(pkt, port, torque_state: dict):
    print(f"\n{'ID':<4} {'Joint':<15} {'Torque':<8} {'Raw':>6} {'Degrees':>9}")
    print("-" * 46)
    for name, mid in MOTORS.items():
        raw = read_pos(pkt, port, mid)
        deg_str = f"{raw2deg(raw):+.1f}°" if raw is not None else "ERR"
        raw_str = str(raw) if raw is not None else "ERR"
        state   = "ON " if torque_state[mid] else "off"
        print(f"{mid:<4} {name:<15} {state:<8} {raw_str:>6} {deg_str:>9}")
    print()


def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else PORT
    print(f"SO-100 Torque Toggle — {port_name}\n")

    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: cannot open {port_name}"); sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    # Verify motors
    for name, mid in MOTORS.items():
        if read_pos(pkt, port, mid) is None:
            print(f"ERROR: '{name}' (id={mid}) not responding")
            port.closePort(); sys.exit(1)

    # Init: configure PID, start with torque OFF (limp / free-move)
    for mid in MOTORS.values():
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"],      16)
        pkt.write1ByteTxRx(port, mid, ADDR["D"],      32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"],       0)
        pkt.write1ByteTxRx(port, mid, ADDR["accel"], 200)

    torque_state = {mid: False for mid in MOTORS.values()}

    print("Commands: [1-6] toggle joint  [a] all ON  [r] all OFF  [p] positions  [q] quit")
    print_status(pkt, port, torque_state)

    try:
        while True:
            try:
                cmd = input(">> ").strip().lower()
            except EOFError:
                break

            if cmd == 'q':
                break

            elif cmd == 'a':
                for name, mid in MOTORS.items():
                    # write current pos as goal before enabling (no jump)
                    raw = read_pos(pkt, port, mid) or CENTER
                    pkt.write2ByteTxRx(port, mid, 42, raw)
                    set_torque(pkt, port, mid, True)
                    torque_state[mid] = True
                print("All torques ENABLED.")
                print_status(pkt, port, torque_state)

            elif cmd == 'r':
                for mid in MOTORS.values():
                    set_torque(pkt, port, mid, False)
                    torque_state[mid] = False
                print("All torques RELEASED — arm is free to move.")
                print_status(pkt, port, torque_state)

            elif cmd == 'p':
                print_status(pkt, port, torque_state)

            elif cmd in ('1', '2', '3', '4', '5', '6'):
                mid  = int(cmd)
                name = ID_TO_NAME[mid]
                new_state = not torque_state[mid]
                if new_state:
                    raw = read_pos(pkt, port, mid) or CENTER
                    pkt.write2ByteTxRx(port, mid, 42, raw)
                set_torque(pkt, port, mid, new_state)
                torque_state[mid] = new_state
                state_str = "ENABLED" if new_state else "RELEASED"
                print(f"  {name} (id={mid}) torque {state_str}.")
                print_status(pkt, port, torque_state)

            else:
                print("  Unknown command.")

    except KeyboardInterrupt:
        pass
    finally:
        for mid in MOTORS.values():
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        port.closePort()
        print("\nAll torques released. Done.")


if __name__ == "__main__":
    main()
