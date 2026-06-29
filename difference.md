# 2-Link Planar IK vs Placo IK — 区别与适用场景

## 概述

本项目中使用了两种逆运动学（IK）方法：
- **2-link planar IK**：用于 `2_dfarm_ee_control.py`、`7_dfarm_yolo_pick.py`、`dfarm_api.py`
- **Placo IK**：用于 `9_dfarm_ik_xyz.py`

两者解决的是同一个问题——给定末端目标位置，求出各关节角度——但在建模方式、求解精度、适用场景上有本质区别。

---

## 2-Link Planar IK

### 原理

将 SO-100 的肩部（`shoulder_lift`）和肘部（`elbow_flex`）两个关节抽象成一个**平面二连杆系统**，只在一个垂直平面内求解。

```
底座（固定）
  │
  L1（shoulder_lift）
  │
  L2（elbow_flex）
  │
  末端
```

用解析解直接求解：

```python
L1 = 0.1159  # 大臂长度（米）
L2 = 0.1350  # 小臂长度（米）

# 余弦定理求 elbow angle
c2 = -(r² - L1² - L2²) / (2·L1·L2)
t2 = π - arccos(c2)

# 几何关系求 shoulder angle
t1 = atan2(y, x) + atan2(L2·sin(t2), L1 + L2·cos(t2))
```

`shoulder_pan` 控制底座旋转（左右），`wrist_flex` 通过 `pitch_sum` 约束保持末端朝向不变，均独立于 IK 计算。

### 特点

| 特性 | 说明 |
|---|---|
| 计算维度 | 2D 平面（x, y） |
| 求解方式 | 解析解，直接计算无迭代 |
| 速度 | 极快，微秒级 |
| 依赖 | 仅 Python 标准库（math） |
| 精度 | 在模型假设范围内精确，但忽略了连杆偏置和旋转轴偏移 |
| 自由度 | 仅解 shoulder_lift + elbow_flex，其他关节独立控制 |
| 姿态控制 | 不支持（只控制位置，朝向通过 pitch_sum 补偿近似） |

### 局限性

1. **忽略 Z 轴**：只能在一个垂直平面内移动，无法控制真实的三维空间位置
2. **忽略连杆偏置**：`THETA1_OFF` / `THETA2_OFF` 是经验修正量，不是从 URDF 精确提取的
3. **不处理奇异点**：接近全伸展或全收缩时数值不稳定
4. **wrist_flex 补偿是近似的**：`pitch_sum = sl + ef + wf` 假设了线性关系，在大角度变化时有误差

---

## Placo IK

### 原理

Placo 是一个基于 URDF 的通用机器人运动学库，使用**带阻尼最小二乘的迭代雅可比矩阵法**（Damped Least Squares / Levenberg-Marquardt）求解。

```
加载 URDF → 建立完整刚体运动学树
    ↓
正运动学（FK）：从当前关节角度计算末端位姿 T_world_frame
    ↓
计算雅可比矩阵 J（6×5）
    ↓
阻尼最小二乘：dq = α · Jᵀ · (J·Jᵀ + λI)⁻¹ · error
    ↓
更新关节角度，迭代直到收敛
```

每次迭代调用 100 次：

```python
for _ in range(100):
    self.solver.solve(True)
```

### 特点

| 特性 | 说明 |
|---|---|
| 计算维度 | 完整 3D 空间（x, y, z） |
| 求解方式 | 数值迭代，雅可比矩阵法 |
| 速度 | 较慢，毫秒到数十毫秒级 |
| 依赖 | `placo`（需额外安装），URDF 文件 |
| 精度 | 基于 URDF 的精确几何模型，误差在毫米级 |
| 自由度 | 求解所有 5 个臂关节（shoulder_pan 也参与） |
| 姿态控制 | 支持（可设置 orientation_weight 约束末端朝向） |

### 局限性

1. **需要 URDF 文件**：必须有精确的机器人描述文件
2. **迭代可能不收敛**：目标超出工作空间时迭代发散
3. **初始猜测依赖**：以当前关节角为初始值，局部最优解可能不理想
4. **安装依赖较重**：`placo` 包本身有较多 C++ 依赖

---

## 核心差异对比

| 对比项 | 2-Link Planar IK | Placo IK |
|---|---|---|
| 空间维度 | 2D（x, y 平面） | 3D（x, y, z） |
| 求解方法 | 解析解 | 数值迭代（雅可比法） |
| 计算速度 | 极快（<1μs） | 较慢（~10ms） |
| 精度 | 近似（忽略偏置和 Z 轴） | 高（基于 URDF 几何） |
| 姿态控制 | 不支持 | 支持（可选） |
| 依赖 | 无 | placo + URDF |
| 奇异点处理 | 手动限幅 | 阻尼矩阵自动处理 |
| 实时控制 | 适合（50Hz+ 无压力） | 较吃力（需优化） |

---

## 适用场景

### 推荐使用 2-Link Planar IK 的场景

**俯视平面抓取**（当前 7_dfarm_yolo_pick.py 的场景）

机械臂在固定高度俯视工作台，只需在水平平面内移动末端位置。此时 `shoulder_lift` + `elbow_flex` 控制前后距离，`shoulder_pan` 控制左右，`wrist_flex` 保持摄像头垂直朝下，Z 轴由固定的抓取姿态（`POSE_PICK`）决定。2-link planar IK 计算量极小，完全满足 50Hz 实时控制要求。

适合的具体任务：
- 俯视摄像头 + 视觉伺服（IBVS）
- 固定桌面高度的抓取和放置
- 键盘/手柄末端控制
- 需要高频率（>30Hz）实时响应的场景

**不适合**：
- 需要精确控制 Z 轴高度的任务
- 需要控制末端姿态（朝向）的任务
- 复杂空间轨迹

---

### 推荐使用 Placo IK 的场景

**三维空间精确定位**（9_dfarm_ik_xyz.py 的场景）

需要指定真实三维坐标 `(x, y, z)` 让末端到达目标位置，且需要较高精度。URDF 描述了机械臂的精确几何结构，Placo 能正确处理连杆偏置、旋转轴偏移等实际因素。

适合的具体任务：
- 需要 Z 轴精确控制（不同高度夹取）
- 需要末端姿态约束（如保持水平、保持某个角度）
- 复杂空间轨迹规划
- 对精度要求高的装配类任务
- 与其他系统配合（如点云、深度相机给出 3D 坐标）

**不适合**：
- 高频实时控制（>30Hz）
- 嵌入式或资源受限环境
- 不需要 Z 轴控制的简单平面任务（用 2-link 就够了）

---

## 实际选择建议

```
任务需要控制 Z 轴或末端姿态？
    是 → Placo IK（9_dfarm_ik_xyz.py）
    否 →
        需要实时高频控制（>30Hz）？
            是 → 2-Link Planar IK（ee_control / yolo_pick）
            否 → 两者都可以，Placo 更精确
```

对于本项目的主要用例——**俯视 YOLO 视觉伺服抓取**——2-link planar IK 是更合适的选择：计算开销几乎为零，实时性强，且俯视平面抓取本质上是一个 2D 问题，不需要 Z 轴 IK。

Placo IK 更适合未来拓展的场景，比如：从深度相机获取物体 3D 位置后直接移动到该坐标，或者需要机械臂在不同高度层之间精确运动的任务。

---

## 其他适合机械臂夹取的 IK 方法

### IKPy

**简介**：纯 Python 的开源 IK 库，基于 DH 参数（Denavit-Hartenberg）建模，支持从 URDF 自动提取参数。使用数值迭代（BFGS 优化）或解析法求解。

```python
from ikpy.chain import Chain
chain = Chain.from_urdf_file("so101_new_calib.urdf", active_links_mask=[...])
target = np.array([0.15, 0.0, 0.20])
angles = chain.inverse_kinematics(target)
```

| 特性 | 说明 |
|---|---|
| 安装 | `pip install ikpy`，纯 Python，无 C++ 依赖 |
| 建模 | DH 参数，支持从 URDF 读取 |
| 求解方式 | 数值优化（BFGS），可选解析解 |
| 速度 | 中等（~5-20ms / 次） |
| 精度 | 高（毫米级，取决于收敛） |
| 姿态控制 | 支持（6-DOF 位置+姿态） |
| 奇异点 | 优化器自动处理，但可能陷入局部极值 |

**适合场景**：
- 不想依赖重型 C++ 库，需要纯 Python 环境
- 需要比 2-link planar 更高精度，但不想装 placo
- 快速原型验证 3D IK 逻辑
- 对安装复杂度敏感的部署环境

**不适合**：实时高频控制（>20Hz），收敛速度不稳定。

---

### Pinocchio（soarm-control 使用的方案）

**简介**：C++ 编写、Python 绑定的高性能刚体动力学和运动学库，是工业级机器人控制的标准工具之一。soarm-control 项目使用 Pinocchio 配合雅可比矩阵法求解 IK。

```python
import pinocchio as pin
model = pin.buildModelFromUrdf("so101_new_calib.urdf")
data  = model.createData()
# 雅可比矩阵迭代
jacobian = pin.computeFrameJacobian(model, data, q, frame_id, ...)
dq = step_size * J.T @ np.linalg.solve(J @ J.T + damping * I, error)
```

| 特性 | 说明 |
|---|---|
| 安装 | `pip install pin`（有预编译包），需 C++ 运行时 |
| 建模 | 完整 URDF，支持动力学（质量、惯量） |
| 求解方式 | 阻尼最小二乘雅可比迭代（同 Placo） |
| 速度 | 快（C++ 核心，~1-5ms） |
| 精度 | 高（与 Placo 同级） |
| 姿态控制 | 支持（完整 SE(3) 约束） |
| 额外能力 | 动力学、碰撞检测、轨迹优化 |

**适合场景**：
- 对性能要求高，需要接近实时的 3D IK（10-20Hz 可行）
- 需要扩展到动力学控制（力控、阻抗控制）
- 已有 lerobot / soarm 生态的项目（soarm-control 已集成）
- 长期项目，需要稳定的工业级库支撑

**不适合**：安装复杂，轻量化场景下依赖过重。

---

### Scipy 优化（minimize）

**简介**：用 `scipy.optimize.minimize` 将 IK 转化为数值优化问题，最小化末端位置误差。不需要专用 IK 库，只需 scipy 和一个 FK 函数。

```python
from scipy.optimize import minimize

def cost(q):
    ee = forward_kinematics(q)          # 自己实现 FK
    return np.linalg.norm(ee - target)  # 位置误差

result = minimize(cost, q0, method="SLSQP",
                  bounds=[(-np.pi, np.pi)] * n_joints)
```

| 特性 | 说明 |
|---|---|
| 安装 | `pip install scipy`，无额外依赖 |
| 建模 | 自定义 FK 函数，灵活但需手写 |
| 求解方式 | 通用数值优化（SLSQP、L-BFGS-B 等） |
| 速度 | 慢（~50-200ms），不适合实时 |
| 精度 | 高（取决于 FK 精度和优化器收敛） |
| 姿态控制 | 可扩展（加入姿态误差项到 cost） |

**适合场景**：
- 快速验证想法，不想引入专用库
- 离线轨迹规划（不需要实时）
- FK 模型已经有但没有 URDF 的情况
- 教学/研究场景，理解 IK 优化本质

**不适合**：实时控制，对速度有要求的任何场景。

---

## 完整横向对比

| 方法 | 维度 | 速度 | 精度 | 安装难度 | 姿态控制 | 实时适用 |
|---|---|---|---|---|---|---|
| **2-Link Planar** | 2D | ★★★★★ | ★★★ | 无依赖 | ✗ | ✅ 50Hz+ |
| **IKPy** | 3D | ★★★ | ★★★★ | 简单 | ✅ | ⚠️ ~20Hz |
| **Placo** | 3D | ★★★★ | ★★★★★ | 中等 | ✅ | ⚠️ ~20Hz |
| **Pinocchio** | 3D | ★★★★★ | ★★★★★ | 较难 | ✅ | ✅ ~30Hz |
| **Scipy minimize** | 3D | ★ | ★★★★ | 简单 | ✅ | ✗ 离线 |

---

## 对 SO-100 夹取任务的最终建议

| 任务类型 | 推荐方案 | 理由 |
|---|---|---|
| 俯视平面视觉伺服（IBVS） | 2-Link Planar | 够用、无依赖、50Hz 无压力 |
| 固定高度桌面抓取 | 2-Link Planar | 同上 |
| 多高度精确定位 | Placo / IKPy | 需要 Z 轴控制 |
| 末端姿态约束（如保持水平） | Pinocchio / Placo | 需要 SE(3) 约束 |
| 离线轨迹规划 | Scipy / IKPy | 不需要实时，优先精度 |
| 与 lerobot 生态集成 | Pinocchio | soarm-control 已验证 |
| 快速原型 / 无重依赖 | IKPy | 纯 Python，安装简单 |

---

## 实测结论（SO-100 四点往复运动测试）

测试方法：用 `9_dfarm_ik_xyz.py`（Placo FK）采集四个实际点位的 xyz 坐标，分别用四种 IK 方法驱动机械臂到达这四个点，主观评估到达精度。

| 方法 | 实测精度 | 备注 |
|---|---|---|
| Pinocchio | ✅ 完全符合 | 推荐，精度最高 |
| IKPy | ✓ 基本可用 | 夹爪和部分关节有轻微偏差 |
| 2-Link Planar | ✗ 误差极大 | 符合预期，本质不支持真 3D |
| Placo | ✗ 完全偏离 | soft 约束配置下 IK 精度极差 |

**Placo 问题分析**：`9_dfarm_ik_xyz.py` 中 Placo 的 IK 实现使用 `frame_task` + `"soft"` 约束，`orientation_weight=0.0` 在 Placo 里并不等同于"不约束姿态"，求解器仍然受姿态约束影响，导致位置精度极差（误差 ~300mm）。这是 Placo 的 API 行为与预期不符，并非 URDF 问题。

**最终推荐**：对 SO-100 需要精确 3D 位置控制的场景，使用 **Pinocchio**（`9_dfarm_ik_xyz.py` 已内置）。
