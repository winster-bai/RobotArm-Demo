# scservo_sdk API 参考手册

基于 `feetech-servo-sdk`（`scservo_sdk`）的飞特舵机控制 API，适用于 SO-100 机械臂。

---

## 目录

1. [初始化与连接](#1-初始化与连接)
2. [寄存器地址表](#2-寄存器地址表)
3. [扭矩控制](#3-扭矩控制)
4. [位置控制](#4-位置控制)
5. [读取当前位置](#5-读取当前位置)
6. [PID 与加速度配置](#6-pid-与加速度配置)
7. [锁存寄存器（Lock）](#7-锁存寄存器lock)
8. [Ping 检测](#8-ping-检测)
9. [群组同步写（Sync Write）](#9-群组同步写sync-write)
10. [群组同步读（Sync Read）](#10-群组同步读sync-read)
11. [角度与原始值换算](#11-角度与原始值换算)
12. [完整示例：控制单个舵机到指定角度](#12-完整示例控制单个舵机到指定角度)
13. [完整示例：上电 → 运动 → 释放扭矩](#13-完整示例上电--运动--释放扭矩)

---

## 1. 初始化与连接

```python
import scservo_sdk as scs

PORT     = "/dev/ttyACM0"   # 串口设备
BAUDRATE = 1_000_000        # 波特率（飞特默认 1Mbps）
PROTOCOL = 0                # 0 = 小端序（SCS 系列默认）

port = scs.PortHandler(PORT)
port.openPort()             # 返回 True 表示成功
port.setBaudRate(BAUDRATE)

pkt = scs.PacketHandler(PROTOCOL)  # 创建数据包处理器
```

关闭连接：

```python
port.closePort()
```

---

## 2. 寄存器地址表

SO-100 / SCS 系列舵机控制表（常用地址）：

| 地址 | 名称               | 字节数  | 说明                         |
|------|-------------------|--------|-----------------------------|
| 21   | P_Coefficient     | 1      | P 增益                      |
| 22   | D_Coefficient     | 1      | D 增益                      |
| 23   | I_Coefficient     | 1      | I 增益                      |
| 40   | Torque_Enable     | 1      | 1=使能扭矩，0=释放扭矩         |
| 41   | Acceleration      | 1      | 加速度（0~254）              |
| 42   | Goal_Position     | 2      | 目标位置（0~4095）            |
| 55   | Lock              | 1      | 1=锁定EEPROM，0=解锁         |
| 56   | Present_Position  | 2      | 当前位置（只读）              |

```python
ADDR = {
    "P":      21,
    "D":      22,
    "I":      23,
    "torque": 40,
    "accel":  41,
    "goal":   42,
    "lock":   55,
    "pos":    56,
}
```

---

## 3. 扭矩控制

### 使能扭矩（锁定舵机）

```python
motor_id = 1
pkt.write1ByteTxRx(port, motor_id, ADDR["torque"], 1)
```

### 释放扭矩（舵机可自由转动）

```python
pkt.write1ByteTxRx(port, motor_id, ADDR["torque"], 0)
```

> 修改 PID、加速度等参数前必须先释放扭矩。

---

## 4. 位置控制

### 写入目标位置（原始值 0~4095）

```python
# 将舵机 1 移动到原始值 2048（中位，对应 0°）
goal_raw = 2048
pkt.write2ByteTxRx(port, motor_id, ADDR["goal"], goal_raw)
```

### 按角度控制（需换算，见第 11 节）

```python
CENTER = 2048
RES    = 4096

def deg2raw(deg: float) -> int:
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360.0)))

# 控制舵机 3 转到 +45°
pkt.write2ByteTxRx(port, 3, ADDR["goal"], deg2raw(45.0))
```

---

## 5. 读取当前位置

### 读取原始位置值

```python
raw, comm_result, error = pkt.read2ByteTxRx(port, motor_id, ADDR["pos"])

if comm_result == scs.COMM_SUCCESS:
    print(f"原始值: {raw}")
else:
    print("读取失败:", pkt.getTxRxResult(comm_result))
```

### 读取角度值

```python
def raw2deg(raw: int) -> float:
    return (raw - CENTER) * 360.0 / RES

raw, comm_result, _ = pkt.read2ByteTxRx(port, motor_id, ADDR["pos"])
if comm_result == scs.COMM_SUCCESS:
    print(f"当前角度: {raw2deg(raw):.1f}°")
```

---

## 6. PID 与加速度配置

修改前需先释放扭矩（`Torque_Enable = 0`）并解锁（`Lock = 0`）。

```python
mid = 1

# 解锁 + 释放扭矩
pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)

# 设置 PID
pkt.write1ByteTxRx(port, mid, ADDR["P"], 16)   # P 增益
pkt.write1ByteTxRx(port, mid, ADDR["D"], 32)   # D 增益
pkt.write1ByteTxRx(port, mid, ADDR["I"],  0)   # I 增益

# 设置加速度（254 = 最大）
pkt.write1ByteTxRx(port, mid, ADDR["accel"], 100)
```

---

## 7. 锁存寄存器（Lock）

Lock 寄存器保护 EEPROM 区域（低地址参数）不被意外修改。

```python
# 解锁（允许写入配置参数）
pkt.write1ByteTxRx(port, motor_id, ADDR["lock"], 0)

# 锁定（写入配置后恢复保护）
pkt.write1ByteTxRx(port, motor_id, ADDR["lock"], 1)
```

---

## 8. Ping 检测

```python
model_number, comm_result, error = pkt.ping(port, motor_id)

if comm_result == scs.COMM_SUCCESS:
    print(f"舵机 {motor_id} 在线，型号: {model_number}")
else:
    print(f"舵机 {motor_id} 无响应")
```

---

## 9. 群组同步写（Sync Write）

一次广播命令同时控制多个舵机，延迟最低。

```python
group_write = scs.GroupSyncWrite(port, pkt, ADDR["goal"], 2)  # 2字节

targets = {1: 2048, 2: 1800, 3: 2200}  # motor_id -> raw_position

for mid, raw in targets.items():
    param = [scs.SCS_LOBYTE(raw), scs.SCS_HIBYTE(raw)]
    group_write.addParam(mid, param)

group_write.txPacket()
group_write.clearParam()
```

---

## 10. 群组同步读（Sync Read）

一次请求读取多个舵机的同一寄存器。

```python
motor_ids = [1, 2, 3, 4, 5, 6]
group_read = scs.GroupSyncRead(port, pkt, ADDR["pos"], 2)  # 2字节

for mid in motor_ids:
    group_read.addParam(mid)

comm_result = group_read.txRxPacket()

for mid in motor_ids:
    if group_read.isAvailable(mid, ADDR["pos"], 2):
        raw = group_read.getData(mid, ADDR["pos"], 2)
        print(f"舵机 {mid}: {raw2deg(raw):.1f}°")

group_read.clearParam()
```

---

## 11. 角度与原始值换算

SO-100 舵机分辨率 4096 步 / 360°，中位值 2048 对应 0°。

```python
CENTER = 2048
RES    = 4096

def deg2raw(deg: float) -> int:
    """角度 → 原始值，自动限幅"""
    return max(0, min(RES - 1, int(CENTER + deg * RES / 360.0)))

def raw2deg(raw: int) -> float:
    """原始值 → 角度"""
    return (raw - CENTER) * 360.0 / RES

def pct2raw(pct: float) -> int:
    """百分比（0~100）→ 原始值，用于夹爪"""
    return int(max(0.0, min(100.0, pct)) / 100.0 * (RES - 1))
```

| 角度  | 原始值 |
|-------|--------|
| -180° | 0      |
| 0°    | 2048   |
| +180° | 4095   |

---

## 12. 完整示例：控制单个舵机到指定角度

```python
import scservo_sdk as scs

PORT, BAUDRATE, PROTOCOL = "/dev/ttyACM0", 1_000_000, 0
CENTER, RES = 2048, 4096
ADDR_TORQUE, ADDR_GOAL, ADDR_POS = 40, 42, 56

def deg2raw(deg): return max(0, min(RES - 1, int(CENTER + deg * RES / 360.0)))
def raw2deg(raw): return (raw - CENTER) * 360.0 / RES

port = scs.PortHandler(PORT)
port.openPort()
port.setBaudRate(BAUDRATE)
pkt = scs.PacketHandler(PROTOCOL)

motor_id = 3  # elbow_flex

# 使能扭矩
pkt.write1ByteTxRx(port, motor_id, ADDR_TORQUE, 1)

# 移动到 +45°
pkt.write2ByteTxRx(port, motor_id, ADDR_GOAL, deg2raw(45.0))

import time; time.sleep(1.5)

# 读取当前位置
raw, result, _ = pkt.read2ByteTxRx(port, motor_id, ADDR_POS)
if result == scs.COMM_SUCCESS:
    print(f"当前位置: {raw2deg(raw):.1f}°")

# 释放扭矩
pkt.write1ByteTxRx(port, motor_id, ADDR_TORQUE, 0)
port.closePort()
```

---

## 13. 完整示例：上电 → 运动 → 释放扭矩

模拟 SO-100 六轴机械臂的标准启动流程：

```python
import time
import scservo_sdk as scs

PORT, BAUDRATE, PROTOCOL = "/dev/ttyACM1", 1_000_000, 0
CENTER, RES = 2048, 4096

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

def deg2raw(deg): return max(0, min(RES - 1, int(CENTER + deg * RES / 360.0)))
def raw2deg(raw): return (raw - CENTER) * 360.0 / RES

port = scs.PortHandler(PORT)
port.openPort()
port.setBaudRate(BAUDRATE)
pkt = scs.PacketHandler(PROTOCOL)

# ── 步骤 1：配置 PID（需先释放扭矩）──
for mid in MOTORS.values():
    pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
    pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)
    pkt.write1ByteTxRx(port, mid, ADDR["P"],      16)
    pkt.write1ByteTxRx(port, mid, ADDR["D"],      32)
    pkt.write1ByteTxRx(port, mid, ADDR["I"],       0)
    pkt.write1ByteTxRx(port, mid, ADDR["accel"], 100)

# ── 步骤 2：读取当前位置，Goal = 当前值，再使能扭矩（防止突然运动）──
for name, mid in MOTORS.items():
    raw, result, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    if result != scs.COMM_SUCCESS:
        raise RuntimeError(f"舵机 {name}(id={mid}) 无响应")
    pkt.write2ByteTxRx(port, mid, ADDR["goal"], raw)   # 先写当前位置
    pkt.write1ByteTxRx(port, mid, ADDR["torque"], 1)   # 再使能扭矩
    pkt.write1ByteTxRx(port, mid, ADDR["lock"],   1)

print("所有舵机已上电，当前位置：")
for name, mid in MOTORS.items():
    raw, _, _ = pkt.read2ByteTxRx(port, mid, ADDR["pos"])
    print(f"  {name:<15} {raw2deg(raw):+.1f}°")

# ── 步骤 3：移动到目标角度 ──
target_angles = {
    "shoulder_pan":   0.0,
    "shoulder_lift": 20.0,
    "elbow_flex":   -30.0,
    "wrist_flex":    10.0,
    "wrist_roll":     0.0,
    "gripper":        0.0,
}

for name, deg in target_angles.items():
    mid = MOTORS[name]
    pkt.write2ByteTxRx(port, mid, ADDR["goal"], deg2raw(deg))

time.sleep(2.0)  # 等待运动完成

# ── 步骤 4：释放所有扭矩 ──
for mid in MOTORS.values():
    pkt.write1ByteTxRx(port, mid, ADDR["torque"], 0)
    pkt.write1ByteTxRx(port, mid, ADDR["lock"],   0)

port.closePort()
print("完成，扭矩已释放。")
```

---

## 通信结果码

| 常量                | 值  | 说明               |
|---------------------|-----|--------------------|
| `COMM_SUCCESS`      | 0   | 通信成功           |
| `COMM_PORT_BUSY`    | -1  | 端口占用           |
| `COMM_TX_FAIL`      | -2  | 发送失败           |
| `COMM_RX_FAIL`      | -3  | 接收失败           |
| `COMM_RX_TIMEOUT`   | -6  | 接收超时           |
| `COMM_RX_CORRUPT`   | -7  | 数据包校验错误     |

```python
# 统一错误检查模式
result, error = pkt.write1ByteTxRx(port, mid, addr, value)
if result != scs.COMM_SUCCESS:
    print(pkt.getTxRxResult(result))
if error:
    print(pkt.getRxPacketError(error))
```
