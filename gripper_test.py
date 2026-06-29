import scservo_sdk as scs

# ─── Config ──────────────────────────────────────────────────────────────────
PORT = "/dev/ttyACM0"
BAUDRATE = 1_000_000
PROTOCOL = 0
MID = 6  # gripper

ADDR_TORQUE = 40
ADDR_LOCK = 55
ADDR_MIN_ANGLE = 9   # Min Angle Limit (2 bytes, EEPROM)
ADDR_MAX_ANGLE = 11  # Max Angle Limit (2 bytes, EEPROM)

NEW_MIN = 0
NEW_MAX = 4095

port = scs.PortHandler(PORT)
if not port.openPort():
    raise SystemExit(f"ERROR: cannot open {PORT}")
port.setBaudRate(BAUDRATE)
pkt = scs.PacketHandler(PROTOCOL)

# Read current limits
lo, _, _ = pkt.read2ByteTxRx(port, MID, ADDR_MIN_ANGLE)
hi, _, _ = pkt.read2ByteTxRx(port, MID, ADDR_MAX_ANGLE)
print(f"before: angle limit = {lo} .. {hi}")

# Unlock EEPROM (torque must be off to write EEPROM)
pkt.write1ByteTxRx(port, MID, ADDR_TORQUE, 0)
pkt.write1ByteTxRx(port, MID, ADDR_LOCK, 0)

# Widen angle limits to full range
pkt.write2ByteTxRx(port, MID, ADDR_MIN_ANGLE, NEW_MIN)
pkt.write2ByteTxRx(port, MID, ADDR_MAX_ANGLE, NEW_MAX)

# Lock EEPROM again
pkt.write1ByteTxRx(port, MID, ADDR_LOCK, 1)

# Verify
lo, _, _ = pkt.read2ByteTxRx(port, MID, ADDR_MIN_ANGLE)
hi, _, _ = pkt.read2ByteTxRx(port, MID, ADDR_MAX_ANGLE)
print(f"after:  angle limit = {lo} .. {hi}")

port.closePort()
