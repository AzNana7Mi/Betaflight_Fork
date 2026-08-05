   # Betaflight ALTHOLD（定高模式）实现原理分析报告

> 分析基于: `Betaflight_Fork` 飞控代码库

---

## 一、整体架构概览

ALTHOLD 定高模式是一个**多层闭环控制系统**，从传感器数据采集到最终电机输出，分为以下层次：

```
┌─────────────────────────────────────────────────────────────┐
│  传感器层                                                    │
│  气压计(Baro) / GPS / 测距仪(Rangefinder) / 加速度计(IMU)     │
└──────────────────────┬──────────────────────────────────────┘
                       │ 原始数据
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  位置估计器 (Position Estimator)                              │
│  2态 Kalman 滤波器 [position, velocity]                        │
│  - 预测: IMU加速度积分                                        │
│  - 修正: 气压计/GPS/测距仪观测                                 │
│  - 输出: 融合后的高度(cm) 和 垂直速度(cm/s)                     │
└──────────────────────┬──────────────────────────────────────┘
                       │ 融合高度 & 垂直速度
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  ALTHOLD 模式管理层 (alt_hold_multirotor.c)                   │
│  - 模式切换 & 目标高度管理                                     │
│  - 油门摇杆死区判断 & 目标高度调节                               │
│  - 悬停油门捕获                                              │
└──────────────────────┬──────────────────────────────────────┘
                       │ 目标高度, 目标速度
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  级联 PID 控制器 (autopilot_multirotor.c)                     │
│  外环(高度): P+I → 垂直速度指令                                │
│  内环(速度): P+I → 油门偏置                                    │
│  + 前馈(Feedforward) + 倾斜补偿                               │
└──────────────────────┬──────────────────────────────────────┘
                       │ 油门输出 [0.0, 1.0]
                       ▼
┌─────────────────────────────────────────────────────────────┐
│  混控器 (Mixer)                                              │
│  替代飞手油门，与其他通道混合输出到电机                           │

---

## 二、传感器数据融合：Kalman 滤波器

### 2.1 核心算法

使用 **2 状态 Kalman 滤波器**（文件: `position_filter.h`, `position_estimator.c`）：

- 状态向量: `[position(cm), velocity(cm/s)]`
- 预测步骤: 使用 IMU 加速度（去除重力、旋转到地球坐标系）进行死推算
- 更新步骤: 使用外部传感器（气压计/GPS/测距仪）观测值修正

### 2.2 仅用气压计（Baro）可以实现定高吗？

**可以，而且这是最常见的配置。** 关键代码在 `position_estimator.c`:

- **预测依赖加速度计**: Kalman 滤波器的预测步骤使用 IMU 的 Z 轴加速度积分，提供高频的姿态响应
- **修正依赖气压计**: 气压计提供绝对高度观测，以较低频率修正漂移
- **互补特性**: 加速度计短期精度高但会漂移，气压计长期稳定但有噪声和延迟 → Kalman 滤波器将两者最优融合

气压计测量噪声参数: `R_BARO_ALT = 2500.0 cm²`（位置方差），相对于 GPS 的 `R_GPS_ALT_BASE = 40000.0 cm²` 更精确。

**仅用气压计定高的精度取决于:**
1. 气压计本身的分辨率和噪声（现代 BMP280/BMP388/DPS310 可达 ±10cm 相对精度）
2. 温度漂移补偿（barometer.c 中有温度读取和校准流程）
3. 桨盘效应和机身气压扰动（需要海绵遮光罩、避开气流）
4. IMU 加速度精度

> **结论**: 仅气压计可以实现实用的定高，但精度受限于气压计质量和物理安装。配合 Kalman 滤波器融合加速度数据后，短时间的定高精度可达 **±20-50cm**。

### 2.3 多传感器融合优先级

配置项 `altitude_source` 决定高度源策略：

| 值 | 含义 |
|---|---|
| `ALTITUDE_SOURCE_DEFAULT` (0) | 所有可用传感器都参与融合 |
| `ALTITUDE_SOURCE_BARO_ONLY` (1) | 仅气压计 |
| `ALTITUDE_SOURCE_GPS_ONLY` (2) | 仅 GPS |
| `ALTITUDE_SOURCE_RANGEFINDER_PREFER` (3) | 优先测距仪，辅以气压计 |
| `ALTITUDE_SOURCE_RANGEFINDER_ONLY` (4) | 仅测距仪 |

当启用测距仪时，其测量噪声极低 (`R_RANGEFINDER_ALT = 100 cm²`)，在低空时可提供厘米级精度。

### 2.4 传感器偏移校准

`barometer.c` 在上电后进行自动校准（多次采样取平均），建立 `baroGroundAltitude` 基准。`position_estimator.c` 中还有跨传感器偏移校准（cross-calibration），用非漂移源（如 GPS）修正气压计长期漂移。

└─────────────────────────────────────────────────────────────┘
```

---

## 三、ALTHOLD 模式管理（alt_hold_multirotor.c）

### 3.1 模式激活与退出

```
模式开关 (BOXALTHOLD) 激活时:
  ├── altHoldReset(): 捕获当前高度为目标高度
  ├── autopilotCaptureHoverThrottleForAltHold(): 捕获当前油门为悬停油门
  └── altHold.isActive = true

模式退出时:
  ├── resetAltitudeControl(): 重置 PID 积分项
  ├── autopilotClearAltHoldHoverThrottle(): 清除捕获的悬停油门
  └── altHold.isActive = false
```

### 3.2 悬停油门管理

代码: `autopilotGetEffectiveHoverThrottlePwm()` 按优先级选择:
1. 手动配置的 `hoverThrottle` CLI 参数 (非零时)
2. ALTHOLD 激活时自动捕获的油门值
3. 默认值 `AP_HOVER_THROTTLE_DEFAULT = 1275` (PWM us)

自动捕获逻辑: `autopilotCaptureHoverThrottleForAltHold()` 在模式进入时读取 `rcCommand[THROTTLE]` 并钳位在 [1100, 1700] 范围内。这要求飞行员在进入定高前已处于悬停状态。

### 3.3 油门摇杆目标高度调节

`altHoldUpdateTargetAltitude()` 实现:

```
飞手油门在 hoverPwm 附近 deadband 内 → 保持当前目标高度
飞手推油门超过上死区 → 目标高度以 maxVelocity 速率上升
飞手拉油门低于下死区 → 目标高度以 maxVelocity 速率下降
```

**死区是不对称的** (更人性化):
- 下边界: `hoverPwm - deadband% * (hoverPwm - PWM_RANGE_MIN)`
- 上边界: `hoverPwm + deadband% * (PWM_RANGE_MAX - hoverPwm)`

**约束**: 目标高度不会与当前实际高度相差超过 1 秒的 `maxVelocity` 行程，防止 P/I 响应过激。

Failsafe 模式下使用更激进的下降速率（高空 10x 加速下降）。



---

## 四、级联 PID 控制器（autopilot_multirotor.c） — 核心控制算法

### 4.1 控制器架构

采用 **外环（位置）+ 内环（速度）的级联 PID** 结构：

```
目标高度 ──→[外环 P+I]──→ 速度指令 ──→[内环 P+I]──→ 油门偏置
                ↑                        ↑
              高度误差                 速度误差
                ↑                        ↑
           实际高度(KF)             实际速度(KF)
```

### 4.2 关键 PID 参数映射

| CLI 参数 | 默认值 | 作用 | 内部缩放 |
|----------|--------|------|----------|
| `altitudeP` | 15 | 外环高度 P（高度误差→速度指令） | ×0.04 → (cm/s)/cm |
| `altitudeI` | 15 | **分裂为两路**: 外环 I (高度→速度偏置) + 内环 I (速度→油门) | 外环 ×0.0008, 内环 ×0.00012 |
| `altitudeD` | 15 | 内环速度 P（D 在此作速度 P 使用） | ×0.005 → throttle/(cm/s) |
| `altitudeF` | 15 | 前馈增益（目标速度直接到油门） | ×0.005, 30 = 100% feedforward |

### 4.3 外环：高度→速度 (P+I)

```c
velCmd = targetVelocity * altitudeFfKfNorm          // Feedforward
       + altitudeError * altitudeOuterKp             // P: 高度误差→速度
       + altitudePosIntegral;                        // I: 累积高度误差→速度偏置

// 积分分离: 当高度误差 > 200cm 时，I 增益降至 10%
const float itermRelax = (fabsf(altitudeErrorCm) < 200.0f) ? 1.0f : 0.1f;

// 速度指令限幅: [-1500, 1500] cm/s (默认) 或自定义 velLimitCmS
velCmd = constrainf(velCmd, -velMax, velMax);
```

### 4.4 内环：速度→油门 (P+I)

```c
velErr = velCmd - vz;                               // 速度误差
altitudeVelIntegral += velErr * altitudeInnerKi * dt; // I 积分
innerP = velErr * altitudeInnerKp * dBoost;          // P (带速度过大时的增强)

throttleOffset = innerP + altitudeVelIntegral + hoverOffset;
```

**D-Boost 机制**: 当 |垂直速度| > 500 cm/s 时，内环 P 增益动态增强（最大 3x），在高动态飞行时提供更强的阻尼。

### 4.5 倾斜补偿（Tilt Compensation）— 打倾斜摇杆时的自动高度保持

**核心代码** (`autopilot_multirotor.c:321-322`):

```c
const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
throttleOffset *= tiltMultiplier;
```

#### 工作原理

当飞行器倾斜时，电机产生的总推力 T 分解为：
- **垂直分量**: T × cos(θ) — 用于维持高度
- **水平分量**: T × sin(θ) — 用于水平运动

在没有倾斜补偿的情况下，如果 ALTHOLD 只输出悬停油门，倾斜角 θ 增大时垂直推力分量减少，飞机会**掉高**。倾斜补偿通过在油门输出上乘以 `1/cos(θ)`，确保垂直方向的推力保持恒定：

```
补偿后垂直推力 = (T × 1/cosθ) × cosθ = T  ← 与水平时相同
```

#### 补偿范围与限制

| 倾角 θ | cos(θ) | tiltMultiplier (1/cosθ) | 油门增幅 |
|--------|--------|------------------------|----------|
| 0° | 1.000 | 1.00x | 0% |
| 10° | 0.985 | 1.02x | +2% |
| 20° | 0.940 | 1.06x | +6% |
| 30° | 0.866 | 1.15x | +15% |
| 45° | 0.707 | 1.41x | +41% |
| 60° | 0.500 | **2.00x** (钳位) | +100% |
| 70° | 0.342 | **2.00x** (钳位) | +100% |

**关键钳位**: `fmaxf(cosθ, 0.5)` 意味着当倾角超过 **60°** 时，补偿不再线性增加，始终以 2x 为上限。这是为了防止极端倾角下油门过度补偿导致飞丢或震荡。

#### 在整个控制回路中的位置

倾斜补偿位于**级联 PID 内环输出之后、油门限幅之前**：

```
高度误差 → [外环 P+I] → 速度指令 → [内环 P+I] → 油门偏置
                                                      │
                                         ┌─ throttleOffset (PID 输出)
                                         │
                                    [倾斜补偿 × 1/cosθ]
                                         │
                                    [PWM范围限幅 + 归一化]
                                         │
                                    最终油门输出 [0,1]
```

这意味着：
1. **PID 本身不需要感知倾角** — 外环和内环在"垂直方向"的坐标系中工作（Kalman 估计的高度和垂直速度已经是 ENU 坐标系）
2. **倾斜补偿是纯前馈** — 它独立于 PID，不经过 PID 回路，直接基于当前倾角计算补偿倍数
3. **补偿是即时的** — 只要 IMU 能测到倾角变化，补偿立即生效，不需要等待高度误差产生

#### 实际表现

**有倾斜补偿时的行为：**
- 在 ALTHOLD 模式下打俯仰/横滚摇杆使飞机倾斜 → 油门自动增加以维持高度
- 小角度倾斜（<30°）：高度保持很好，几乎感觉不到掉高
- 中等角度倾斜（30°-45°）：有轻微波动但总体能维持
- 大角度倾斜（>60°）：补偿饱和，可能掉高

**补偿的局限性（会掉高的情况）：**

1. **动态响应延迟**: 虽然补偿是前馈的，但电机转速变化、气流扰动等因素导致实际推力建立需要时间。突然大角度倾斜时，在几十毫秒内仍可能掉高几厘米。

2. **60° 钳位**: 倾角超过 60° 后补偿不再增加，且此时 `throttleMax` 限幅也可能起作用。如果 `throttleMax = 1900`，悬停油门 1275，可用范围仅 625us，2x 补偿相当于需要 1275 + (1275-1000)×2 = 1825us，尚在范围内；但如果飞机较重（悬停油门 > 1450），则可能撞到上限。

3. **纯几何模型假设**: `1/cosθ` 假设推力与油门呈线性关系且推力方向完全沿机体 Z 轴。实际中：
   - 桨盘入流角变化会改变推力-油门关系
   - 机身气动阻力（尤其是大倾角时的垂向阻力）未被建模
   - 电池电压下降导致相同油门下推力降低

4. **水平运动引发的附加效应**:
   - 快速水平飞行时，机身气动外形可能产生额外的升力或下沉力
   - 地面效应在低空时也会改变推力特性

#### 如果补偿失效或不足，如何从代码层面改进（不改代码的分析）

> 以下为理论分析，现有代码不做修改。

**(a) 增加前馈+反馈混合补偿：**
当前是纯前馈 `1/cosθ`。可以在此基础上前置一个基于 `d(cosθ)/dt` 的微分量，预测倾角变化趋势提前补偿：
```
tiltFF = 1/cosθ + Kv * d(1/cosθ)/dt   // Kv 为速度前馈系数
```
这可以缓解大角度突变时的瞬时掉高。

**(b) 放宽 60° 钳位，改用渐变衰减：**
当前 `fmaxf(cosθ, 0.5)` 在 60° 处硬截断。可改为在 45°-70° 之间使用平滑衰减函数（如 sigmoid 或平方根衰减），让补偿更渐进：
```
effectiveCos = cosθ > 0.342 ? cosθ : 0.342 + 0.3*(cosθ/0.342)
```

**(c) 增加推力-油门非线性映射：**
在倾斜补偿前先对油门做推力线性化（如使用推力曲线表或 sqrt 映射），确保 `throttleOffset × 1/cosθ` 确实产生期望的推力增量，而非简单假设线性关系。

**(d) 增加垂直加速度前馈：**
Kalman 滤波器已经估计了垂直速度 `vz`，可以进一步计算垂直加速度 `az`。将 `az` 反馈到油门中作为加速度前馈（类似 D 的增强版），在掉高刚刚开始时（加速度向下）就增加油门，而不是等高度误差产生后再响应。

**(e) 调参层面的折中方案（无需改代码）：**
- 增大 `altitudeD`（内环速度 P）：更快的速度误差响应，在开始掉高时更迅速补油
- 增大 `altitudeF`：强化目标速度到油门的前馈路径
- 适当提高 `altitudeP`：更激进的高度误差响应
- 确保 `throttleMax` 足够高（如 1950-2000）：给补偿留出余量
- 确保 `hover_throttle` 准确：补偿基准正确才能发挥作用


### 4.6 油门输出限幅

```c
// 原始油门: PWM us 范围
newThrottle = PWM_RANGE_MIN + throttleOffset;
newThrottle = constrainf(newThrottle, throttleMin, throttleMax);

// 归一化到 [0, 1] 供混控器使用
newThrottle = scaleRangef(newThrottle, mincheck, PWM_RANGE_MAX, 0.0f, 1.0f);
throttleOut = constrainf(newThrottle, 0.0f, 1.0f);
```

---

## 五、系统调度与混控集成

- ALTHOLD 任务频率: **100 Hz** (`ALTHOLD_TASK_RATE_HZ`)
- 任务启用条件: 需要有气压计、测距仪或 GPS 任一传感器可用
- 任务优先级: `TASK_PRIORITY_LOW`
- 混控器中 (`mixer.c`): 当 `ALT_HOLD_MODE` 激活时，油门由 `getAutopilotThrottle()` 提供，替代飞手油门输入

---

## 六、纯气压计定高的可行性分析

### 6.1 可以，但有局限

| 条件 | 影响 |
|------|------|
| **气压计硬件质量** | BMP280 ~±100cm, BMP388 ~±25cm, DPS310 ~±5cm 相对精度 |
| **海绵遮光罩** | 必须！减小桨盘气流和光照引起的压力波动 |
| **温度补偿** | 气压计受温度影响大，需良好的温度补偿 |
| **IMU 加速度质量** | Kalman 预测依赖加速度积分，振动过大会降低精度 |
| **飞行高度范围** | 气压计在低空（<2m）受地面效应影响，高空精度更好 |

### 6.2 纯气压计定高的典型表现

- **悬停保持**: ±30-80cm 范围（取决于气压计和安装）
- **上升下降**: 加速度计提供高频响应，气压计纠正低频漂移
- **温度变化**: 飞行中电池/电调发热可能引起几米的漂移（需传感器偏移校准缓解）
- **风速变化**: 气压变化会直接影响高度读数

### 6.3 提升纯气压计定高精度的建议

1. **物理防护**: 使用海绵遮光罩，避免阳光直射和气流
2. **安装位置**: 远离桨盘下沉气流和电调热量
3. **选择高质量气压计**: DPS310 > BMP388 > BMP280


---

## 七、参数调优指南

### 7.1 关键参数速查表

| CLI 参数 | 默认值 | 范围 | 作用 |
|----------|--------|------|------|
| `autopilot_altitude_p` | 15 | 0-200 | 高度误差响应强度 |
| `autopilot_altitude_i` | 15 | 0-200 | 稳态误差消除（慢） |
| `autopilot_altitude_d` | 15 | 0-200 | 垂直速度阻尼 |
| `autopilot_altitude_f` | 15 | 0-200 | 前馈/爬升速率响应 |
| `autopilot_hover_throttle` | 1275 | 0-2000 | 悬停油门 (0=自动捕获) |
| `autopilot_throttle_min` | 1100 | 1000-2000 | 自动油门下限 |
| `autopilot_throttle_max` | 1900 | 1000-2000 | 自动油门上限 |
| `alt_hold_climb_rate` | 50 | 0-255 | 最大爬升速率 (×10 = cm/s, 50=5m/s) |
| `alt_hold_deadband` | 20 | 0-100 | 油门死区百分比 |
| `position_altitude_source` | 0 | 0-4 | 高度传感器源 |
| `position_altitude_prefer_baro` | 100 | 0-100 | 气压计权重 (÷100) |
| `position_altitude_lpf` | 300 | 0-1000 | 高度低通滤波截止频率 (÷100 Hz) |
| `position_altitude_d_lpf` | 300 | 0-1000 | 垂直速度低通滤波截止频率 (÷100 Hz) |

### 7.2 调参流程

#### 第一步：确定悬停油门

```
1. 在 ACRO/ANGLE 模式下手动悬停
2. 观察 OSD 中油门值 (RC 通道)
3. 设置 autopilot_hover_throttle 为该值
   或者: 设为 0，让飞控在 ALTHOLD 激活时自动捕获
```

#### 第二步：基础 PID 调整

```
症状: 高度上下来回震荡（超调）
→ 减小 altitudeP (每步 -3~5)

症状: 保持高度但缓慢漂移（稳态误差）
→ 增大 altitudeI (每步 +3~5)

症状: 油门响应迟钝、反应慢
→ 增大 altitudeP

症状: 垂直方向来回抖动（速度环震荡）
→ 减小 altitudeD (每步 -3~5)
→ 或增大 position_altitude_d_lpf 到 500

症状: 油门变化过于剧烈（高频振荡）
→ 减小 altitudeD
→ 增大 position_altitude_d_lpf
```

#### 第三步：爬升速率调整

```
alt_hold_climb_rate = 50  # 默认 5 m/s
- 降低: 使目标高度调节更平滑 (如 30 = 3 m/s)
- 提高: 快速爬升/下降 (如 80 = 8 m/s)

alt_hold_deadband = 20  # 死区 20%
- 增大: 减少误触发 (如 30)
- 减小: 更灵敏的目标调节 (如 10)
```

#### 第四步：传感器滤波调优

```
position_altitude_lpf = 300  # 高度低通 3Hz
- 如果高度读数噪声大: 降低到 200 或 150
- 如果高度响应慢: 提高到 500

position_altitude_d_lpf = 300  # 速度低通 3Hz
- 如果速度抖动: 降低到 150-200
- 如果需要快速响应: 提高到 400-500

position_altitude_prefer_baro = 100  # 气压计权重 100%
- 纯气压计: 保持 100
- 有 GPS 辅助: 可降低到 50-80
```

### 7.3 常见问题诊断

| 现象 | 可能原因 | 解决方案 |
|------|---------|----------|
| 进入ALTHOLD后猛升/猛降 | 悬停油门不准 | 手动设定 `hover_throttle` 或确保进入时稳定悬停 |
| 高度缓慢漂移 | I 增益不足或气压计漂移 | 增大 `altitudeI`; 检查气压计遮光 |
| 高度周期性震荡 | P 过大或 D 不足 | 减小 `altitudeP`; 增大 `altitudeD` |
| 油门快速抖动 | D 过大或速度噪声 | 减小 `altitudeD`; 增大 `altitude_d_lpf` |
| 倾斜时掉高 | 倾斜补偿不足或饱和 | 见 [4.5 节倾斜补偿详解](#45-倾斜补偿tilt-compensation-打倾斜摇杆时的自动高度保持)；确保 `throttle_max` 充裕，调高 `altitudeD` |
| 低空定高不准 | 地面效应+桨盘气流 | 考虑加装测距仪或光流传感器 |

### 7.4 针对纯气压计的优化配置建议

```
# 保守参数（稳定性优先）
set autopilot_altitude_p = 12
set autopilot_altitude_i = 18
set autopilot_altitude_d = 10
set autopilot_altitude_f = 10

# 滤波增强（纯气压计噪声较大）
set position_altitude_lpf = 200

---

## 八、代码文件索引

| 文件 | 作用 |
|------|------|
| `src/main/flight/alt_hold_multirotor.c` | 多旋翼 ALTHOLD 模式管理（状态切换、目标管理） |
| `src/main/flight/alt_hold_multirotor.h` | ALTHOLD 接口声明 |
| `src/main/flight/autopilot_multirotor.c` | 级联 PID 控制器 + 悬停油门管理 |
| `src/main/flight/autopilot_multirotor.h` | 自动驾驶接口声明 |
| `src/main/flight/position_estimator.c` | Kalman 滤波器传感器融合 |
| `src/main/flight/position_estimator.h` | 融合估计接口 |
| `src/main/flight/position_filter.h` | 2状态 Kalman 滤波器实现 |
| `src/main/flight/position.c` | 高度/速度的低通滤波和对外接口 |
| `src/main/flight/position.h` | `getAltitudeCmControl()` 等接口 |
| `src/main/pg/alt_hold_multirotor.c` | ALTHOLD 参数组（climbRate, deadband） |
| `src/main/pg/alt_hold_multirotor.h` | ALTHOLD 参数结构体 |
| `src/main/pg/autopilot.h` | 自动驾驶PID参数结构体 |
| `src/main/pg/autopilot.c` | PID 默认值 |
| `src/main/sensors/barometer.c` | 气压计驱动与高度计算 |
| `src/main/flight/mixer.c` | 混控器（ALT_HOLD时替代飞手油门） |
| `src/main/fc/tasks.c` | 任务调度（100Hz ALTHOLD 任务） |

---

## 九、总结

1. **纯气压计可以实现定高**，但精度受限于气压计本身质量和安装工艺，典型悬停保持精度约 ±30-80cm。

2. **Kalman 滤波器是关键**，它将加速度计的高频响应与气压计的长期稳定性融合，兼顾了响应速度和稳态精度。

3. **级联 PID 结构**（外环高度→速度，内环速度→油门）提供了良好的解耦控制，各参数职责清晰：
   - **altitudeP**: 响应速度（高度误差→速度指令）
   - **altitudeI**: 消除稳态误差（同时作用于外环和内环）
   - **altitudeD**: 速度阻尼（在此作为内环速度 P 使用）
   - **altitudeF**: 前馈（摇杆/导航目标速度直接映射到油门）

4. **有倾斜补偿！** 打倾斜摇杆时，飞控通过 `1/cosθ` 前馈公式自动增加油门来保持高度。补偿在 <30° 倾角时效果良好，30°-60° 有轻微波动，>60° 时补偿饱和（2x 钳位）可能掉高。详见 [第 4.5 节](#45-倾斜补偿tilt-compensation-打倾斜摇杆时的自动高度保持)。

5. **提升定高精度的三个方向**:
   - **物理**: 高质量气压计 + 海绵遮光罩 + 远离气流/热源
   - **滤波**: 调整 `position_altitude_lpf` 和 `position_altitude_d_lpf` 抑制噪声
   - **PID**: 正确设定悬停油门 + 适度调低 P/调高 I + 使用较小的 climb_rate

6. **终极方案**: 如需厘米级精度（如自动降落），建议加装激光/超声波测距仪，在低空时启用 `ALTITUDE_SOURCE_RANGEFINDER_PREFER`。

---

> 报告生成日期: 2026-07-30 | 分析代码: Betaflight_Fork

