# Hover Throttle Auto-Learn 实现方案

> 目标：在 Betaflight 飞控上实现类似 ArduPilot 的悬停油门自动学习功能，适用于**纯气压计+IMU**（无GPS/测距仪）的低成本方案。

---

## 一、现有定高架构回顾

### 1.1 传感器融合层 (`position_estimator.c`)

- **Kalman滤波器**（2态: `[position, velocity]`）在 Z 轴进行传感器融合
- **预测步骤**: 使用IMU线性加速度（ENU坐标系、去重力）作为控制输入，100Hz
- **校正步骤**: 气压计(baro)测量值更新，`R_BARO_ALT = 2500 cm²`
- **交叉校准**: 用不漂移的传感器（GPS/测距仪，若有）校准漂移传感器（气压计）的偏移

### 1.2 高度估计层 (`position.c`)

- `calculateEstimatedAltitude()` 运行在 100Hz
- `controlAltitudeCm` = 原始Kalman估计（给控制器用，低延迟）
- `displayAltitudeCm` = PT2低通滤波后（给OSD显示用）

### 1.3 定高逻辑层 (`alt_hold_multirotor.c`)

- ALT_HOLD模式激活时：记录当前高度为目标高度，捕获当前油门为悬停基线
- 摇杆在死区外时调整目标高度（以悬停油门为中心）
- 退出时重置PID积分和悬停油门捕获值

### 1.4 串级PID控制 (`autopilot_multirotor.c::altitudeControl()`)

```
外环（位置→速度）:
  velCmd = targetVel * FF_gain + altError * outerKp + outerIntegral

内环（速度→油门）:
  throttleOut = (velErr * innerKp + innerIntegral) + hoverOffset
  hoverOffset = hoverThrottlePwm - PWM_RANGE_MIN  ← 悬停油门前馈
  throttleOut *= 1/cos(tilt)  ← 倾斜补偿
```

### 1.5 悬停油门获取优先级（当前）

```
cfg->hoverThrottle（用户手动设置，!=0） > 
  altHoldCapturedHoverPwm（进入定高时一次性捕获） > 
  AP_HOVER_THROTTLE_DEFAULT（硬编码 1275）
```

**问题**: 只有一次性捕获，没有持续学习。电池电压变化/载重变化/气压计漂移都无法自适应。

---

## 二、修改文件清单

| # | 文件 | 修改内容 |
|---|------|---------|
| 1 | `src/main/pg/autopilot.h` | 添加 `hoverThrottleLearn` 和 `hoverThrottleLearnRate` 结构体字段 |
| 2 | `src/main/pg/autopilot.c` | 添加两字段默认值，升 PG 版本号 |
| 3 | `src/main/fc/parameter_names.h` | 添加两个CLI参数名宏定义 |
| 4 | `src/main/cli/settings.c` | 注册两个CLI参数 |
| 5 | `src/main/flight/autopilot_multirotor.c` | **核心**：学习算法实现 |
| 6 | `src/main/flight/autopilot_multirotor.h` | 导出新函数声明 |
| 7 | `src/main/flight/alt_hold_multirotor.c` | 集成保存调用到退出逻辑 |
| 8 | `src/main/blackbox/blackbox.c` | 记录新参数到日志头 |

---

## 三、详细修改内容

### 3.1 `src/main/pg/autopilot.h` — 结构体扩展

在 `autopilotConfig_t` 的结尾 `}` 之前加入：

```c
    // Hover throttle auto-learning
    // 启用后，在ALT_HOLD模式下自动学习悬停油门值
    // 学习条件: 摇杆在死区内、飞机姿态<25°、垂直速度<50cm/s
    uint8_t hoverThrottleLearn;      // 0=禁用(默认), 1=启用
    // 学习速率 ×100
    // 例: 5 = 0.05/s, 约20秒时间常数
    // 例: 2 = 0.02/s, 约50秒时间常数
    uint8_t hoverThrottleLearnRate;  // 默认5 (范围1-50)
```

### 3.2 `src/main/pg/autopilot.c` — 默认值 & 版本号

```c
// 修改 PG_REGISTER 版本号: 7 → 8
PG_REGISTER_WITH_RESET_TEMPLATE(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 8);

// 在 PG_RESET_TEMPLATE 的闭括号 ) 前加入:
    .hoverThrottleLearn = 0,           // 默认关闭
    .hoverThrottleLearnRate = 5,       // 默认学习速率 0.05/s (20秒时间常数)
```

### 3.3 `src/main/fc/parameter_names.h` — 参数名

在 `PARAM_NAME_AP_GEOFENCE_ACTION` 之后加入：

```c
#define PARAM_NAME_AP_HOVER_THROTTLE_LEARN "ap_hover_throttle_learn"
#define PARAM_NAME_AP_HOVER_THROTTLE_LEARN_RATE "ap_hover_throttle_learn_rate"
```

### 3.4 `src/main/cli/settings.c` — 参数注册

在 `PARAM_NAME_AP_GEOFENCE_ACTION` 行之后、`#endif // ENABLE_FLIGHT_PLAN` 之前加入：

```c
    // Hover throttle auto-learning
    { PARAM_NAME_AP_HOVER_THROTTLE_LEARN,      VAR_UINT8  | MASTER_VALUE | MODE_LOOKUP, .config.lookup = { TABLE_OFF_ON }, PG_AUTOPILOT, offsetof(autopilotConfig_t, hoverThrottleLearn) },
    { PARAM_NAME_AP_HOVER_THROTTLE_LEARN_RATE, VAR_UINT8  | MASTER_VALUE, .config.minmaxUnsigned = { 1, 50 },     PG_AUTOPILOT, offsetof(autopilotConfig_t, hoverThrottleLearnRate) },
```

### 3.5 `src/main/flight/autopilot_multirotor.c` — ⭐核心实现

#### 3.5.1 新增学习状态结构体（约第106行 `altHoldCapturedHoverPwm` 旁边）

```c
// ──── 悬停油门自动学习状态 ────
typedef struct {
    float    smoothedHoverPwm;    // 一阶低通平滑后的悬停油门 PWM 值
    uint32_t stableTimeAccumMs;   // 累计稳定时间（毫秒）
    bool     isLearningActive;    // 当前是否正在学习
    bool     wasLearned;          // 本次上电/ARM 以来是否学到有效值
} hoverLearnState_t;

static hoverLearnState_t hoverLearn;
```

#### 3.5.2 学习初始化函数（在 `autopilotInit()` 附近）

```c
static void hoverThrottleLearnInit(void)
{
    hoverLearn.smoothedHoverPwm = (float)AP_HOVER_THROTTLE_DEFAULT;
    hoverLearn.stableTimeAccumMs = 0;
    hoverLearn.isLearningActive = false;
    hoverLearn.wasLearned = false;
}
```

#### 3.5.3 核心学习更新函数

```c
/**
 * 悬停油门自动学习更新
 *
 * 设计思路:
 * ┌─────────────────────────────────────────────────────────────┐
 * │ 稳定性检查 (三个条件同时满足):                               │
 * │   ① |垂直速度| < 50 cm/s  (Kalman融合速度，容忍短期漂移)      │
 * │   ② cos(tilt) > 0.9       (倾斜角 < ~25°)                   │
 * │   ③ 摇杆在死区内          (基于已学hover值和死区配置)        │
 * │                                                             │
 * │ 学习机制:                                                   │
 * │   连续稳定 ≥1秒 → 启动一阶低通                               │
 * │   smoothed = smoothed*(1-α) + throttleOut*α                 │
 * │   α = learnRate/100 * dt_s   (默认: 0.05*0.01 = 0.0005/s)  │
 * │   钳位: [1100, 1700] PWM                                    │
 * └─────────────────────────────────────────────────────────────┘
 */
static void hoverThrottleLearnUpdate(float throttlePwm, float dt_s)
{
    const autopilotConfig_t *cfg = autopilotConfig();

    // ====== 功能开关检查 ======
    if (!cfg->hoverThrottleLearn) {
        hoverLearn.isLearningActive = false;
        hoverLearn.stableTimeAccumMs = 0;
        return;
    }

    // 用户手动设置了 hoverThrottle → 不学习（用户值优先）
    if (cfg->hoverThrottle != 0) {
        hoverLearn.isLearningActive = false;
        hoverLearn.stableTimeAccumMs = 0;
        return;
    }

    // ====== 重新激活时初始化基准值 ======
    if (!hoverLearn.isLearningActive && hoverLearn.stableTimeAccumMs == 0) {
        hoverLearn.smoothedHoverPwm = (float)autopilotGetEffectiveHoverThrottlePwm();
    }

    // ====== 稳定性条件判断 ======
    bool stable = true;

    // 条件1: 垂直速度 < 50 cm/s
    // 注: 纯气压计+IMU下，Kalman速度可能在几秒内漂移，但对于学习窗口(数秒)足够
    if (fabsf(getAltitudeDerivativeControl()) > 50.0f) {
        stable = false;
    }

    // 条件2: 倾斜角 < ~25° (cos > 0.9)
    if (getCosTiltAngle() < 0.9f) {
        stable = false;
    }

    // 条件3: 摇杆在死区内（仅AltHold模式下检查）
    if (isAltHoldActive()) {
        const float deadband = altHoldConfig()->deadband / 100.0f;
        if (altHoldConfig()->deadband != 0) {
            const float rcThrottle = rcCommand[THROTTLE];
            const float hoverPwm = hoverLearn.smoothedHoverPwm;
            const float lowThreshold  = hoverPwm - deadband * (hoverPwm - PWM_RANGE_MIN);
            const float highThreshold = hoverPwm + deadband * (PWM_RANGE_MAX - hoverPwm);
            if (rcThrottle < lowThreshold || rcThrottle > highThreshold) {
                stable = false;
            }
        }
    }

    // ====== 学习状态机 ======
    if (stable) {
        hoverLearn.stableTimeAccumMs += (uint32_t)(dt_s * 1000.0f);

        // 稳定持续 ≥1秒 → 开始学习
        if (hoverLearn.stableTimeAccumMs >= 1000) {
            hoverLearn.isLearningActive = true;

            // 一阶低通: smoothed = smoothed*(1-α) + new*α
            const float alpha = cfg->hoverThrottleLearnRate * 0.01f * dt_s;
            const float clampedAlpha = constrainf(alpha, 0.0f, 0.1f);  // 防步长过大
            hoverLearn.smoothedHoverPwm = hoverLearn.smoothedHoverPwm * (1.0f - clampedAlpha)
                                        + throttlePwm * clampedAlpha;

            // 钳位到合理范围
            hoverLearn.smoothedHoverPwm = constrainf(hoverLearn.smoothedHoverPwm,
                                                     (float)AP_HOVER_THROTTLE_CAPTURE_MIN,
                                                     (float)AP_HOVER_THROTTLE_CAPTURE_MAX);
            hoverLearn.wasLearned = true;
        }
    } else {
        // 不稳定 → 重置累计时间（但保留已学习值）
        hoverLearn.stableTimeAccumMs = 0;
        hoverLearn.isLearningActive = false;
    }
}
```

#### 3.5.4 修改 `autopilotGetEffectiveHoverThrottlePwm()` — 插入学习值优先级

```c
uint16_t autopilotGetEffectiveHoverThrottlePwm(void)
{
    // 优先级1: 用户手动设置
    const uint16_t cfgHover = autopilotConfig()->hoverThrottle;
    if (cfgHover != 0) {
        return cfgHover;
    }

    // 优先级2: 自动学习值（本航次已学到）
    if (autopilotConfig()->hoverThrottleLearn && hoverLearn.wasLearned) {
        return (uint16_t)lrintf(hoverLearn.smoothedHoverPwm);
    }

    // 优先级3: 进入定高时一次性捕获（原有逻辑）
    if (altHoldCapturedHoverPwm != 0) {
        return altHoldCapturedHoverPwm;
    }

    // 优先级4: 硬编码默认值
    return AP_HOVER_THROTTLE_DEFAULT;
}
```

#### 3.5.5 修改 `autopilotCaptureHoverThrottleForAltHold()` — 学习接管时跳过捕获

```c
void autopilotCaptureHoverThrottleForAltHold(void)
{
    if (autopilotConfig()->hoverThrottle != 0) {
        altHoldCapturedHoverPwm = 0;
        return;
    }

    // 如果启用学习且本航次已学到有效值 → 继续使用学习值，不覆盖
    if (autopilotConfig()->hoverThrottleLearn && hoverLearn.wasLearned) {
        altHoldCapturedHoverPwm = 0;
        return;
    }

    altHoldCapturedHoverPwm = (uint16_t)lrintf(constrainf(rcCommand[THROTTLE],
        (float)AP_HOVER_THROTTLE_CAPTURE_MIN, (float)AP_HOVER_THROTTLE_CAPTURE_MAX));
}
```

#### 3.5.6 修改 `altitudeControl()` — 在函数末尾加入学习调用

在 `altitudeControl()` 函数的末尾，`DEBUG_SET(DEBUG_AUTOPILOT_ALTITUDE, 7, ...)` 之后加入：

```c
    // ──── 悬停油门自动学习 ────
    // 在每个 control cycle 运行学习更新
    // 使用限幅后、缩放到[0,1]之前的 PWM 值作为学习输入
    {
        float learnPwm = PWM_RANGE_MIN + throttleOffset;  // 还原未缩放的 PWM
        learnPwm = constrainf(learnPwm, (float)AP_HOVER_THROTTLE_CAPTURE_MIN,
                                        (float)AP_HOVER_THROTTLE_CAPTURE_MAX);
        hoverThrottleLearnUpdate(learnPwm, taskIntervalS);
    }
```

#### 3.5.7 修改 `autopilotInit()` — 加入初始化

```c
void autopilotInit(void)
{
    // ... 现有代码保持不变 ...

    hoverThrottleLearnInit();  // ← 新增
}
```

#### 3.5.8 新增保存和查询函数

```c
/**
 * 将学习到的悬停油门保存到 EEPROM
 *
 * 调用时机: ALT_HOLD 退出时 / ARM 解除时
 * 保存条件: 
 *   - 学习确实完成(wasLearned)
 *   - 新值与原值差异 > 10 PWM（防止频繁写入）
 *   - 当前未手动设置 hoverThrottle
 */
void autopilotSaveLearnedHoverThrottle(void)
{
    if (!hoverLearn.wasLearned) {
        return;
    }

    const uint16_t learned = (uint16_t)lrintf(hoverLearn.smoothedHoverPwm);
    const uint16_t current = autopilotConfig()->hoverThrottle;

    // 仅当显著变化（>10 PWM）且用户未手动设置时才保存
    if (abs((int)learned - (int)current) > 10 && current == 0) {
        autopilotConfigMutable()->hoverThrottle = learned;
        saveConfigAndNotify();  // 写入 Flash + 蜂鸣确认
    }
}

/** 查询学习状态（供 OSD / 调试使用） */
bool autopilotIsHoverThrottleLearning(void)
{
    return hoverLearn.isLearningActive;
}

/** 获取学习到的悬停油门值（供 OSD 显示） */
uint16_t autopilotGetLearnedHoverThrottlePwm(void)
{
    if (!hoverLearn.wasLearned) {
        return 0;
    }
    return (uint16_t)lrintf(hoverLearn.smoothedHoverPwm);
}
```

### 3.6 `src/main/flight/autopilot_multirotor.h` — 导出声明

在现有导出之后加入：

```c
void autopilotSaveLearnedHoverThrottle(void);
bool autopilotIsHoverThrottleLearning(void);
uint16_t autopilotGetLearnedHoverThrottlePwm(void);
```

### 3.7 `src/main/flight/alt_hold_multirotor.c` — 退出时保存

修改 `altHoldProcessTransitions()` 的退出分支：

```c
    } else {
        if (altHold.isActive) {
            resetAltitudeControl();  // Reset throttle output when exiting altitude hold
            autopilotSaveLearnedHoverThrottle();  // ← 新增: 自动保存学习值
            autopilotClearAltHoldHoverThrottle();
        }
        altHold.isActive = false;
    }
```

### 3.8 `src/main/blackbox/blackbox.c` — 日志头

在 autopilot 参数记录区，`PARAM_NAME_AP_MAX_ANGLE` 行之后加入：

```c
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_AP_HOVER_THROTTLE_LEARN, "%d",       autopilotConfig()->hoverThrottleLearn);
        BLACKBOX_PRINT_HEADER_LINE(PARAM_NAME_AP_HOVER_THROTTLE_LEARN_RATE, "%d",  autopilotConfig()->hoverThrottleLearnRate);
```

---

## 四、用户使用指南

### 4.1 CLI 配置

```
# 启用悬停油门自动学习
set ap_hover_throttle_learn = ON

# 设置学习速率（默认5，建议2-10）
#  2 = 慢速学习（~50秒时间常数），稳定性好
#  5 = 中等速度（~20秒时间常数），推荐
# 10 = 快速学习（~10秒时间常数），响应快但容易受扰动影响
set ap_hover_throttle_learn_rate = 5

# 确保手动悬停油门清零（让学习接管）
set ap_hover_throttle = 0

# 保存
save
```

### 4.2 使用流程

```
1. ARM → 推油门起飞
2. 切 ALT_HOLD 模式 → 推杆到悬停油门位置
3. 保持摇杆在死区内 → 飞机稳定悬停
4. 1秒后自动开始学习（学习中无蜂鸣）
5. 持续悬停 10-30 秒 → 学习收敛完成
6. 切出 ALT_HOLD → 蜂鸣 "哔" 一声 → 值已保存到Flash
7. 下次ARM自动使用学到的悬停油门
```

### 4.3 OSD 建议（后续优化）

可在 OSD 中显示：
- `ap_hover_throttle` 当前值（PWM）
- 学习状态指示器（L = Learning, S = Stable）
- 已学习值预览

### 4.4 Blackbox 调试

使用 `DEBUG_AUTOPILOT_ALTITUDE` 模式：
- `debug[0]`: 最终 throttle PWM（现有）
- `debug[1]`: 学习状态 (0=未学习, >0=学习中的smoothed值)

---

## 五、数据流图

```
┌─ CLI 配置 ───────────────────────────────────────────────────────┐
│ set ap_hover_throttle_learn = ON                                  │
│ set ap_hover_throttle_learn_rate = 5                              │
│ set ap_hover_throttle = 0    (清零手动值）                        │
│ save                                                              │
└──────────────────────────────────────────────────────────────────┘
         │
         ▼  (ARM / 飞行中)
┌─ ALT_HOLD 进入 ──────────────────────────────────────────────────┐
│ autopilotCaptureHoverThrottleForAltHold()                         │
│   ┌─ 已学到有效值 → 跳过捕获，直接使用学习值                      │
│   └─ 首次飞行 → 捕获当前油门作为初始值                            │
│                              │                                    │
│   hoverOffset = learnedPwm - PWM_RANGE_MIN                        │
│            ┌─────────────────┘                                    │
│            ▼                                                      │
│ ┌─ altitudeControl() 每 10ms ──────────────────────────────────┐ │
│ │ 串级PID → throttleOut                                         │ │
│ │   velCmd = ff + P*altErr + I∫altErr                           │ │
│ │   throttle = P*velErr + I∫velErr + hoverOffset                │ │
│ │   throttle *= 1/cos(tilt)    ← 倾斜补偿                       │ │
│ │                                                                │ │
│ │ 学习检查 (hoverThrottleLearnUpdate):                           │ │
│ │   ├─ |vz| < 50 cm/s ?                                         │ │
│ │   ├─ cos(tilt) > 0.9 ?                                        │ │
│ │   └─ stick in deadband ?                                      │ │
│ │       ↓ 全部满足                                               │ │
│ │   stableTimeAccum >= 1000ms ?  → 启动学习                      │ │
│ │       ↓                                                        │ │
│ │   smoothedPwm = smoothedPwm*(1-α) + throttlePwm*α              │ │
│ └────────────────────────────────────────────────────────────────┘ │
│                                                                   │
│ ┌─ ALT_HOLD 退出 ──────────────────────────────────────────────┐ │
│ │ autopilotSaveLearnedHoverThrottle()                            │ │
│ │   → cfg->hoverThrottle = learnedPwm                            │ │
│ │   → saveConfigAndNotify()  → Flash EEPROM 写入 + 蜂鸣确认     │ │
│ └────────────────────────────────────────────────────────────────┘ │
└───────────────────────────────────────────────────────────────────┘
         │
         ▼  (下次 ARM)
┌─ autopilotGetEffectiveHoverThrottlePwm() ─────────────────────────┐
│ 优先级: cfg->hoverThrottle > learned > captured > default(1275)   │
└──────────────────────────────────────────────────────────────────┘
```

---

## 六、参数说明

| CLI参数 | 类型 | 默认 | 范围 | 说明 |
|---------|------|------|------|------|
| `ap_hover_throttle_learn` | uint8 | 0(OFF) | OFF/ON | 启用悬停油门自动学习 |
| `ap_hover_throttle_learn_rate` | uint8 | 5 | 1-50 | 学习速率×100，越大越快 |

### 学习速率效果对比

| learn_rate | α/s | 时间常数 | 适用场景 |
|------------|-----|---------|---------|
| 2 | 0.02 | ~50s | 平稳飞行，抗扰动好 |
| 5 | 0.05 | ~20s | 推荐默认值 |
| 10 | 0.10 | ~10s | 快速适应，但容易受阵风影响 |
| 20 | 0.20 | ~5s | 激进学习，用于调试 |

---

## 七、PG兼容性

PG 版本号从 `7` 升级到 `8`。升级后旧配置自动填充新字段为默认值（0, 5）。

---

## 八、与 ArduPilot MOT_HOVER_LEARN 的对比

| 特性 | ArduPilot | 本方案 |
|------|-----------|--------|
| 学习激活 | 需要GPS 3D Fix | **仅需气压计+IMU** |
| 速度反馈 | GPS速度 | **Kalman融合速度**（容忍短期漂移） |
| 学习速率 | 固定 | **CLI可调** (1-50) |
| 保存机制 | 自动存EEPROM | **退出定高时自动存** |
| 电池电压补偿 | 有 | 暂未实现（可后续添加） |
| 学习条件 | 多条件组合 | 简易三条件（vz, tilt, stick） |

---

## 九、潜在风险与缓解

| 风险 | 等级 | 缓解措施 |
|------|------|---------|
| 气压计速度估计漂移 | 中 | 学习窗口短（数秒），外环I积分补偿长期漂移 |
| 阵风下学习到错误值 | 低 | 1秒稳定缓冲 + 缓慢学习速率 |
| 电池电压变化导致相同PWM推力不同 | 中 | 后续可添加电压补偿因子 |
| EEPROM写入过于频繁 | 低 | 差异>10 PWM才写入，退出定高时单次写入 |
| 切换电池后学习值不准 | 中 | 每次飞行可重新学习，或后续加电压补偿 |

---

## 十、后续优化方向

1. **电池电压补偿**: `smoothedPwm *= (referenceVoltage / currentVoltage)`
2. **OSD显示**: 显示实时学习状态 + 已学习值
3. **多电池学习**: 按电池电压分段存储多个hover值
4. **快速学习模式**: disarm时学习值差异大时，下次arm使用更快的学习速率
5. **故障保护**: 学习值超出安全范围(±30%)时回退到默认值
