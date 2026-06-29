#!/usr/bin/env python3
"""
SO-100 Record & Replay — capture a joint trajectory then loop it back.
Requires: pip install feetech-servo-sdk

Workflow:
  1. Release all torques → manually pose the arm
  2. Press Enter to start recording (samples at ~20 Hz via GroupSyncRead)
  3. Press Enter again to stop recording
  4. Arm replays the trajectory in a loop via GroupSyncWrite
  5. Press Ctrl+C to stop replay and release torque

Usage: python 6_so100_record_replay.py [port]
"""

import sys
import time
import threading

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

ADDR = {"torque": 40, "accel": 41, "goal": 42, "lock": 55, "pos": 56,
        "P": 21, "D": 22, "I": 23}

RES        = 4096
CENTER     = 2048
RECORD_HZ  = 20
REPLAY_HZ  = 20


def raw2deg(r: int) -> float:
    return (r - CENTER) * 360.0 / RES

def read_pos(pkt, port, mid):
    v, c, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    return v if c == scs.COMM_SUCCESS else CENTER


def record_loop(pkt, port, motor_ids, stop_event, frames):
    """Background thread: sample all joints via GroupSyncRead."""
    group_read = scs.GroupSyncRead(port, pkt, ADDR["pos"], 2)
    for mid in motor_ids:
        group_read.addParam(mid)

    dt = 1.0 / RECORD_HZ
    while not stop_event.is_set():
        t0 = time.perf_counter()
        group_read.txRxPacket()
        frame = {}
        for mid in motor_ids:
            if group_read.isAvailable(mid, ADDR["pos"], 2):
                frame[mid] = group_read.getData(mid, ADDR["pos"], 2)
            else:
                frame[mid] = CENTER
        frames.append(frame)
        sl = dt - (time.perf_counter() - t0)
        if sl > 0:
            time.sleep(sl)

    group_read.clearParam()


def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else PORT
    print(f"SO-100 Record & Replay — {port_name}\n")

    port = scs.PortHandler(port_name)
    if not port.openPort():
        print(f"ERROR: cannot open {port_name}"); sys.exit(1)
    port.setBaudRate(BAUDRATE)
    pkt = scs.PacketHandler(PROTOCOL)

    motor_ids = list(MOTORS.values())

    # Verify
    for name, mid in MOTORS.items():
        if read_pos(pkt, port, mid) is None:
            print(f"ERROR: '{name}' (id={mid}) not responding")
            port.closePort(); sys.exit(1)

    # Configure PID
    for mid in motor_ids:
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        pkt.write1ByteTxRx(port, mid, ADDR["P"],      16)
        pkt.write1ByteTxRx(port, mid, ADDR["D"],      32)
        pkt.write1ByteTxRx(port, mid, ADDR["I"],       0)
        pkt.write1ByteTxRx(port, mid, ADDR["accel"], 200)

    # ── Phase 1: free-move for posing ──
    print("Phase 1: Torque RELEASED — pose the arm freely.")
    for mid in motor_ids:
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)

    input("Press Enter to START recording...\n")

    # ── Phase 2: record ──
    frames     = []
    stop_event = threading.Event()
    rec_thread = threading.Thread(target=record_loop,
                                  args=(pkt, port, motor_ids, stop_event, frames),
                                  daemon=True)
    print("Recording... Press Enter to STOP.")
    rec_thread.start()
    input()
    stop_event.set()
    rec_thread.join()

    n = len(frames)
    duration = n / RECORD_HZ
    print(f"Recorded {n} frames ({duration:.1f}s)\n")

    if n == 0:
        print("No frames captured. Exiting.")
        port.closePort(); sys.exit(0)

    # ── Phase 3: enable torque for replay ──
    for mid in motor_ids:
        raw = read_pos(pkt, port, mid)
        pkt.write2ByteTxRx(port, mid, ADDR["goal"], raw)
        pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1)
        pkt.write1ByteTxRx(port, mid, ADDR["lock"],   1)

    group_write = scs.GroupSyncWrite(port, pkt, ADDR["goal"], 2)
    dt = 1.0 / REPLAY_HZ

    print("Replaying trajectory in loop... Ctrl+C to stop.\n")
    print(f"{'Frame':>6}  " + "  ".join(f"{n[:6]:>7}" for n in MOTORS))
    print("-" * (8 + 9 * len(MOTORS)))

    try:
        loop = 0
        while True:
            loop += 1
            for i, frame in enumerate(frames):
                t0 = time.perf_counter()

                group_write.clearParam()
                for mid in motor_ids:
                    raw   = frame[mid]
                    param = [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)]
                    group_write.addParam(mid, param)
                group_write.txPacket()

                # status
                degs = "  ".join(f"{raw2deg(frame[mid]):>+6.1f}°" for mid in motor_ids)
                sys.stdout.write(f"\r[{loop}] {i+1:>4}/{n}  {degs}")
                sys.stdout.flush()

                sl = dt - (time.perf_counter() - t0)
                if sl > 0:
                    time.sleep(sl)

    except KeyboardInterrupt:
        pass
    finally:
        group_write.clearParam()
        for mid in motor_ids:
            pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
            pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
        port.closePort()
        print("\n\nDone.")


if __name__ == "__main__":
    main()
