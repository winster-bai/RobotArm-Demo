# df_arm — DF Robot 机械臂控制脚本

基于 `feetech-servo-sdk` 的最小化 SO-100 机械臂控制工具集，无需 lerobot 依赖。

## 安装依赖

```bash
pip install -r requirements.txt
```

或单独安装：

```bash
pip install feetech-servo-sdk          # 所有脚本必需
pip install pynput                     # 键盘控制脚本需要
pip install pygame                     # Xbox 手柄脚本需要
pip install pin                        # Pinocchio IK 脚本需要
pip install ultralytics opencv-python  # YOLO 视觉夹取脚本需要
pip install ollama                     # VLM 摄像头分析脚本需要
```

> VLM 脚本需要本地运行 [Ollama](https://ollama.com) 并拉取模型：
> ```bash
> ollama pull qwen2.5vl:7b
> ```

## 硬件说明

SO-100 使用 6 个 Feetech STS3215 舵机，菊花链连接：

| ID | 关节名称 | 说明 |
|----|----------|------|
| 1 | shoulder_pan | 底座旋转 |
| 2 | shoulder_lift | 肩部俯仰 |
| 3 | elbow_flex | 肘部弯曲 |
| 4 | wrist_flex | 腕部俯仰 |
| 5 | wrist_roll | 腕部旋转 |
| 6 | gripper | 夹爪 |

- 波特率：1,000,000 bps
- 编码器分辨率：4096（12-bit），原始中位值 2048 对应未校准的 0°
- 默认串口：`/dev/ttyACM0`（可通过命令行参数覆盖）

### 关于夹爪行程（重要）

STS3215 在 EEPROM 中有角度限位寄存器（Min/Max Angle Limit，地址 9 / 11）。lerobot 框架标定时可能把夹爪写成了一个较窄的子区间，导致夹爪行程受限（实测仅约 130°）。若更换了夹爪机械结构需要更大行程，可用 `tt.py` 将舵机 6 的角度限位放开到全程 `0~4095`：

```bash
python tt.py
```

放开后再用 `1_dfarm_calibrate.py` 重新标定，把新夹爪的全开 / 全合两端记录进 `dfarm_calibration.json`。当前约定：夹爪 `homing_raw` 设在全开端，**0° ≈ 完全张开，角度越大越闭合**。

---

## 脚本列表

脚本按使用顺序编号，建议首次使用时从 `0_` 开始依次了解。

---

### `0_dfarm_monitor.py` — 实时关节监视器

实时读取并显示所有 6 个关节的原始值和角度，约 10Hz 刷新。无需扭矩使能，只读模式，适合上电后第一步确认状态。

```bash
python 0_dfarm_monitor.py [port] [--cal dfarm_calibration.json]
```

---

### `0_dfarm_torque_toggle.py` — 交互式扭矩开关

逐关节或批量控制扭矩使能/释放，适合手动摆姿势后锁定，也用于校准前的准备。

```bash
python 0_dfarm_torque_toggle.py [port]
```

| 命令 | 说明 |
|------|------|
| `1`~`6` | 切换对应关节扭矩 |
| `a` | 使能所有关节 |
| `r` | 释放所有关节（自由移动模式） |
| `p` | 打印当前位置 |
| `q` | 退出 |

---

### `1_dfarm_calibrate.py` — 关节校准

对每个关节进行零点标定，生成 `dfarm_calibration.json`。**建议首次使用时运行**，后续所有脚本都支持 `--cal` 加载校准文件，使用语义角度（相对物理零点）替代原始角度。

```bash
python 1_dfarm_calibrate.py [port]
```

流程：按提示逐关节手动移到零位，按 Enter 确认，完成后自动保存 `dfarm_calibration.json`。

---

### `2_dfarm_keyboard_control.py` — 键盘关节空间控制

直接控制每个关节角度，50Hz P 控制平滑插值。适合熟悉各关节方向和运动范围。

```bash
python 2_dfarm_keyboard_control.py [port]
```

| 按键 | 关节 | 方向 |
|------|------|------|
| Q / A | shoulder_pan | − / + |
| W / S | shoulder_lift | − / + |
| E / D | elbow_flex | − / + |
| R / F | wrist_flex | − / + |
| T / G | wrist_roll | − / + |
| Y / H | gripper | − / + |
| `0` | 全部关节 | 回中 |
| ESC | — | 退出 |

---

### `3_dfarm_ee_control.py` — 键盘末端笛卡尔控制

通过 2-link 平面 IK 实现末端夹爪的笛卡尔空间控制，50Hz P 控制。

```bash
python 3_dfarm_ee_control.py [port]
```

| 按键 | 功能 |
|------|------|
| W / S | 末端前进 / 后退 |
| E / D | 末端上升 / 下降 |
| Q / A | 底座左转 / 右转（yaw） |
| R / F | 腕部俯仰 上 / 下 |
| T / G | 腕部横滚 CW / CCW |
| Y / H | 夹爪关闭 / 打开 |
| ESC | 退出 |

> 末端目标点会自动钳位在 2-link 可达圆环 `[R_MIN, R_MAX]` 内并写回状态，避免到达工作空间边界后继续按键产生“死区”（积分饱和）。

---

### `3_dfarm_xbox_ee_control.py` — Xbox 手柄末端控制

用 Xbox 手柄控制末端笛卡尔空间，50Hz P 控制。

```bash
python 3_dfarm_xbox_ee_control.py [port]
```

| 输入 | 功能 |
|------|------|
| 左摇杆 Y | 末端前进 / 后退 |
| 左摇杆 X | 底座左右偏航 |
| RT | 末端上升 |
| LT | 末端下降 |
| A | 夹爪关闭 |
| Y | 夹爪打开 |
| X | 腕部横滚 CCW |
| B | 腕部横滚 CW |
| Start | 退出 |

> 摇杆方向不对时，修改脚本顶部的 `AXIS_*` 常量。

---

### `4_dfarm_sync_wave.py` — GroupSyncWrite 正弦波演示

所有关节同时做正弦波运动，演示 GroupSyncWrite 单次广播驱动多舵机。

```bash
python 4_dfarm_sync_wave.py [port] [--cal dfarm_calibration.json]
```

每个关节有独立的振幅、相位偏移和中心角度，可在脚本顶部的 `WAVE` 字典中修改。加载校准文件后，波形以各关节校准零点为中心，并自动钳在 `[range_min, range_max]` 内，避免越界撞机械限位；启动时有 1.5 秒平滑过渡到波形起点，避免上电窜动。Ctrl+C 停止并释放扭矩。

---

### `5_dfarm_record_replay.py` — 轨迹录制与回放

手动摆姿势录制关节轨迹，然后循环回放。适合快速创建演示动作。

```bash
python 5_dfarm_record_replay.py [port]
```

流程：
1. 启动后自动释放扭矩，手动摆好姿势
2. 按 Enter 开始录制（20Hz 采样）
3. 再按 Enter 停止录制
4. 自动使能扭矩，循环回放
5. Ctrl+C 停止并释放扭矩

---

### `6_dfarm_goto_pose.py` — 命名姿态控制器

预设多个命名姿态，输入姿态名平滑插值运动过去。支持校准文件，换机械臂后只需重新校准，姿态定义不变。

```bash
python 6_dfarm_goto_pose.py [port] [--cal dfarm_calibration.json]
```

| 命令 | 说明 |
|------|------|
| `list` | 列出所有预设姿态及角度 |
| `go <name>` | 平滑移动到指定姿态（默认 2 秒） |
| `go <name> <sec>` | 指定运动时长（秒） |
| `save <name>` | 将当前位置保存为新姿态 |
| `pos` | 打印当前各关节角度 |
| `q` | 退出 |

内置姿态：`home` / `zero` / `ready`

---

### `7_dfarm_ik_xyz.py` — Pinocchio IK 三维末端控制

基于 Pinocchio 的精确 3D 逆运动学控制，支持输入 xyz 坐标（URDF base_link 坐标系，单位米）移动末端。

```bash
python 7_dfarm_ik_xyz.py [port] [--urdf so101_new_calib.urdf] [--cal dfarm_calibration.json]
```

| 命令 | 说明 |
|------|------|
| `xyz <x> <y> <z>` | 移动到目标坐标（米），默认 2 秒 |
| `xyz <x> <y> <z> <sec>` | 指定运动时长 |
| `fk` | 显示当前末端 xyz 位置 |
| `pos` | 显示当前关节角度 |
| `free` | 解锁扭矩，实时打印末端位置（用于采集目标点） |
| `lock` | 重新上扭矩 |
| `q` | 退出 |

> 采集目标点：`free` 模式下手动摆到目标位置，终端实时显示当前 xyz，记录后 Enter 恢复扭矩。

---

### `8_dfarm_yolo_pick.py` — YOLO 视觉伺服夹取

基于 IBVS（Image-Based Visual Servoing）的俯视抓取脚本，用 YOLO 检测目标后自动对准并夹取。

```bash
python 8_dfarm_yolo_pick.py [port] \
    [--cal dfarm_calibration.json] \
    [--weights best.pt] \
    [--camera 0] [--conf 0.25] \
    [--target bottle]
```

| 参数 | 说明 |
|------|------|
| `--cal` | 校准文件路径 |
| `--weights` | YOLO 权重，默认同目录 `best.pt` |
| `--camera` | OpenCV 相机索引（默认 0） |
| `--conf` | YOLO 置信度阈值（默认 0.25） |
| `--target` | 仅锁定指定类别，不指定则取置信度最高目标 |

完整抓取流程：
1. 移动到俯视姿态（夹爪打开）
2. IBVS 自动对准目标
3. 单边夹爪偏移 + 下降到固定抓取姿态 + 闭合夹爪
4. 抬起 + 平移到放置点
5. 松开 + 回 home

关键调参项（脚本顶部）：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `TARGET_CX / TARGET_CY` | 320 / 240 | 夹爪在画面中的实际投影位置（需标定） |
| `DEAD_ZONE_PX` | 40 | 对准死区（像素） |
| `KP_LIFT` | 0.10 | 前后方向增益（deg/px），方向反了改为负值 |
| `KP_PAN` | 0.08 | 左右方向增益（deg/px） |
| `GRIPPER_OFFSET_PAN` | 5.0 | 单边夹爪 pan 偏移量（度） |
| `POSE_PICK` | lift=20°, ef=-5° | 固定抓取姿态，需实测标定 |

---

### `8_dfarm_yolo_pick_ik.py` — YOLO 视觉伺服夹取（IK 垂直下降版）

与 `8_dfarm_yolo_pick.py` 流程相同，区别在下降阶段：用 Pinocchio IK 实现真正的垂直下降——IBVS 对准后用 FK 获取当前末端 xyz，保持 xy 不变只把 z 降到桌面高度（`PICK_Z`），再用 IK 求解下降关节角。相比固定抓取姿态更适合不同高度的目标。

```bash
python 8_dfarm_yolo_pick_ik.py [port] \
    [--cal dfarm_calibration.json] \
    [--urdf so101_new_calib.urdf] \
    [--weights best.pt] \
    [--camera 0] [--conf 0.25] \
    [--target bottle]
```

需额外安装 Pinocchio：`pip install pin`。其余参数与 `8_dfarm_yolo_pick.py` 一致。

---

### `9_dfarm_vlm_watch.py` — VLM 视觉语言模型控制

用户输入自然语言指令，结合摄像头画面和机械臂当前状态，由本地 Ollama 视觉模型决策并通过 `dfarm_server.py` 执行动作。

```bash
# 先启动机械臂 API 服务（另一终端）
python dfarm_server.py [port] [--cal dfarm_calibration.json]

# 再启动 VLM 控制
python 9_dfarm_vlm_watch.py [--camera 0] [--model qwen2.5vl:7b] [--api http://127.0.0.1:8001]
```

启动后在终端输入指令（如"移动到俯视姿态"、"把夹爪打开"），模型结合画面和当前关节状态返回 JSON 动作并自动执行。

前置条件：
```bash
ollama pull qwen2.5vl:7b
pip install ollama requests
```

---

## 辅助文件

- `gripper_test.py` — 舵机角度限位工具：读取并放开舵机 6（夹爪）的 EEPROM 角度限位到全程 `0~4095`
- `dfarm_server.py` — 轻量 Flask API 服务，为 `9_dfarm_vlm_watch.py` 提供 HTTP 控制接口
- `dfarm_calibration.json` — 由 `1_dfarm_calibrate.py` 生成的校准文件
- `so101_new_calib.urdf` — SO-101 机械臂 URDF 模型，供 `7_dfarm_ik_xyz.py` 与 `8_dfarm_yolo_pick_ik.py` 使用
- `best.pt` — YOLO 权重文件，供 `8_dfarm_yolo_pick*.py` 使用
- `ik_test/` — IK 方法对比测试脚本（2-link / IKPy / Pinocchio / Placo 往复运动评估）

---

## 控制原理

所有实时控制脚本均采用相同的软件架构：

- 固定 50Hz 控制循环，每个 tick 都发送指令
- 维护 `target`（目标角度）和 `current`（当前发送值）两个状态
- 每 tick 用 P 控制 `current += KP * (target - current)` 平滑逼近目标
- 启动时先读取当前位置作为初始目标，避免上电跳动
- 退出时统一释放所有扭矩

电机端 PID 默认配置：P=16，D=32，I=0，加速度按场景设置（150~254）。
