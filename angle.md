# ANGLE 模式下打倾斜杆掉高度的原因分析与代码级解决方案

> 分析对象: `Betaflight_Fork` (RP_F405_BT) 代码库 | 本文只做分析，**不改动任何源代码**

---

## 一、现象

在 **ANGLE（自稳）模式** 下，油门杆不动，打俯仰/横滚杆让飞机倾斜飞行，飞机会明显**掉高度**。
姿态本身能稳住（姿态环工作正常），但高度不保。这不是飞机故障，而是**控制架构 + 物理本质**决定的。

---

## 二、根本原因（物理）

### 2.1 推力向量分解

多旋翼的推力 **T** 始终沿机体 Z 轴方向。机体倾斜角 θ（横滚与俯仰的合成倾角）时：

```
垂直升力分量 L = T × cos(θ)      ← 负责"扛住重力"
水平分量 H     = T × sin(θ)      ← 产生水平加速度/速度
```

悬停时 θ = 0，需要 `T₀ = m·g`。
倾斜 θ 后要保持同样的垂直升力，需要：

```
T(θ) = T₀ / cos(θ)
```

### 2.2 倾角越大，掉得越厉害

| 倾角 θ | cos(θ) | 垂直升力损失 1−cosθ | 需要补的油门 1/cosθ − 1 |
|--------|--------|---------------------|--------------------------|
| 10°    | 0.985  | 1.5%                | +1.5%                    |
| 20°    | 0.940  | 6.0%                | +6.4%                    |
| 30°    | 0.866  | **13.4%**           | +15.5%                   |
| 45°    | 0.707  | **29.3%**           | +41.4%                   |
| 60°    | 0.500  | **50.0%**           | +100%（2 倍）             |

所以在默认 `angle_limit = 60°` 的 ANGLE 模式（`src/main/flight/pid.c:142`）下，**油门不动的话，倾 45° 直接损失近 1/3 升力**，必然下沉。

### 2.3 还有两个次生因素

1. **水平加速的"能耗"**：倾斜后水平分量做功使机体加速，这部分的能量最终也要从"高度势能"里出，瞬时掉高比稳态公式更明显。
2. **ANGLE 模式姿态保持很"积极"**：`pidLevel()`（`src/main/flight/pid.c:560-643`）用角度误差 × `angle_gain` + 前馈生成角速度指令，杆回中后飞机会被强制改平。倾斜越深、改平越猛，瞬时升力损失越大。

### 2.4 关键结论

**ANGLE 模式只控制姿态（横滚/俯仰/偏航），完全不控制高度/油门**——油门 100% 由飞手摇杆决定（`mixer.c:227`）。这是"设计如此"，不是 bug。

---

## 三、代码层面现状（为什么没有任何补偿在起作用）

### 3.1 ANGLE 模式的油门链路

```
rcCommand[THROTTLE] (飞手摇杆 1000~2000us)
        │
        ▼
mixer.c:227   throttle = rcCommand[THROTTLE] - PWM_RANGE_MIN + throttleAngleCorrection
        │
        ▼
mixer.c:274   throttle = constrainf(throttle / PWM_RANGE, 0, 1)   ← 归一化到 0~1
        │
        ▼
mixer.c:718-740   油门限幅 / 油门boost / TPA ...
        │
        ▼
mixer.c:765-779   与 roll/pitch/yaw PID 混控相加 → 电机输出
```

即：**纯 ANGLE 模式没有任何"按倾角补油门"的环节**，唯一相关的是下面这个 `throttleAngleCorrection`。

### 3.2 内置补偿 `thr_corr_value`：默认是关的，且是加法式

Betaflight 内置了一个倾角油门补偿：

- 计算：`src/main/flight/imu.c:729-744` `calculateThrottleAngleCorrection()`
- 生效条件：`imu.c:762-766` —— 必须 `throttle_correction_value > 0` **且** 处于 ANGLE/HORIZON 模式 **且** 已解锁
- **默认值 `thr_corr_value = 0`（关闭）**：`src/main/fc/core.c:182`
- 公式是 **加法式 + sin 曲线**，不是几何正确的乘法式 1/cosθ：

```c
// imu.c:729-744 (简化)
int angle = acos(getCosTiltAngle()) * throttleAngleScale;   // 倾角 → 0..900 刻度
if (angle > 900) angle = 900;
return throttleAngleValue * sin(angle / (900 * π/2));       // 最大 = thr_corr_value
```

`thr_corr_value` 单位是"us 内的千分之一"，即 `thr_corr_value=10` 只补约 **1%** 油门（范围 0~150 → 0~15%）。对 45° 需要 +41% 的需求来说，**杯水车薪**。CLI 参数见 `src/main/cli/settings.c:967-968`。

### 3.3 本 fork 的乘法式 1/cosθ 补偿：只在 ALT_HOLD / GPS救援里

本 fork 移植的定高自动飞控里**有正确的乘法式补偿**：

```c
// src/main/flight/autopilot_multirotor.c:321
const float tiltMultiplier = 1.0f / fmaxf(getCosTiltAngle(), 0.5f);
throttleOffset *= tiltMultiplier;
```

但它在 **`altitudeControl()` 内部**，而 `altitudeControl()` 只在 `ALT_HOLD_MODE`（`alt_hold_multirotor.c:168`）或 GPS 救援（`gps_rescue_multirotor.c:228`）时被调用。混控器也只在 `ALT_HOLD_MODE` 时才用自动飞控的油门替代飞手油门：

```c
// src/main/flight/mixer.c:798-803
if (FLIGHT_MODE(ALT_HOLD_MODE)) {
    throttle = getAutopilotThrottle();   // 替代飞手油门，含 1/cosθ 补偿
}
```

**所以：不开 ALT_HOLD 的纯 ANGLE 模式，永远走不到这段代码，1/cosθ 补偿完全不生效。**

### 3.4 总结现状

| 补偿方式 | 位置 | 是否生效于 ANGLE 模式 | 强度 |
|---------|------|----------------------|------|
| `thr_corr_value` 加法补偿 | `imu.c` → `mixer.c:227` | ✅ 生效（但默认 value=0 关闭） | 加法式、sin 曲线，最大 15%，几何不准确 |
| 乘法式 1/cosθ 补偿 | `autopilot_multirotor.c:321` | ❌ 只在 ALT_HOLD/GPS救援 | 几何正确，但 ANGLE 模式够不着 |

---

## 四、代码级解决方案

### 方案 A：零代码 —— 用配置缓解

1. **ANGLE + ALT_HOLD 组合**（推荐，最省事）
   本 fork 的 ALT_HOLD 自带 `1/cosθ` 补偿 + 气压计闭环（Kalman 高度估计），倾斜不掉高。开定高后摇杆调节目标高度即可。

2. **启用内置 `thr_corr`**（弱，仅小角度可用）
   ```
   set thr_corr_value = 20        # 最大补 ~2% 油门（太小）
   set thr_corr_angle = 450       # 45° 时补偿达到最大
   ```
   注意它是加法式的，45° 时需要 +41% 它只能给 ~2%，基本只适合 15° 以内的微倾。

3. **习惯层面**：ANGLE 模式倾斜飞行时主动推一点油门。任何纯姿态模式都无法避免这个物理问题。

### 方案 B：给 ANGLE 模式加乘法式 1/cosθ 补偿（推荐，代码层）

思路：把 ALT_HOLD 里那套 `1/cosθ` 前馈移植到 ANGLE 模式的油门链路上，**只对摇杆油门中"偏移量"部分做补偿**（与 `autopilot_multirotor.c:321` 的做法一致，不动 `PWM_RANGE_MIN` 基座，避免低油门下过度补偿）。

**修改位置：`src/main/flight/mixer.c`**，在油门从 `rcCommand` 计算出来、归一化之前（`mixer.c:227` 附近，`#ifdef USE_ACC` 内），追加：

```c
// —— 伪代码，仅示意，未写入源码 ——
if (FLIGHT_MODE(ANGLE_MODE | HORIZON_MODE) && ARMING_FLAG(ARMED)) {
    const float cosTilt = fmaxf(getCosTiltAngle(), 0.5f);   // 0.5 钳位 = 最大 2x 补偿（同 autopilot_multirotor.c:321）
    if (cosTilt > 0.0f) {
        // 建议做成可调增益 k（0~100%），默认 60~80%，避免补偿过猛撞电机上限
        const float k = 0.8f;   // 未来可做成 CLI 参数 angle_tilt_comp
        throttle = PWM_RANGE_MIN
                 + (throttle - PWM_RANGE_MIN) * (1.0f + k * (1.0f / cosTilt - 1.0f));
    }
}
```

要点：

- `getCosTiltAngle()`（`imu.c:789-792`）返回 `rMat.m[2][2]`，**正好是合成倾角（横滚+俯仰联合）的余弦**，无需自行合成两轴。
- 用 `fmaxf(cosTilt, 0.5f)` 与 ALT_HOLD 保持一致的 60° / 2x 钳位。
- 建议加可调增益：`1 + k·(1/cosθ − 1)`，k 默认 0.6~0.8，留 20~40% 给飞手/姿态环余量。
- 放在归一化（`mixer.c:274`）之前，单位统一为 us 偏移量。

**可选增强：倾角变化率前馈**

`1/cosθ` 是稳态前馈，杆动瞬间（倾角快速增大）有滞后。可叠加 `Kv·d(1/cosθ)/dt` 项预测掉高趋势（见 `althold.md` §4.5 同款思路）。

### 方案 C：闭环垂直速度反馈（最稳，本质就是 ALT_HOLD）

前馈无法覆盖所有情况（阵风、减速刹车、大倾角饱和）。最可靠的工程做法是**闭环**：

```
垂直加速度(IMU) + 气压计 → Kalman 高度/垂速估计 (position_estimator.c)
        │
        ▼
垂速误差 → 油门修正（内环 PI，autopilot 已有 altitudeControl()）
```

本 fork **已经具备全部闭环零件**（`alt_hold_multirotor.c` + `autopilot_multirotor.c` + `position_estimator.c`），ANGLE + ALT_HOLD 组合即可获得"自稳 + 定高"体验，不必重新发明。

---

## 五、补偿的物理极限与注意点（无论哪种方案都要知道）

1. **电机上限是硬天花板**。悬停油门越低、余量越大，补偿越有效。若悬停油门 >70%（重机/高 KV 不足），45° 补偿 +41% 大概率直接撞 `throttle_max` 饱和，仍会掉高。
2. **大倾角 + 减速刹车**：前馈只解决"倾斜本身的升力损失"，制动时额外需要的推力前馈补不了——这也是为什么最终仍建议闭环（方案 C）。
3. **60° 钳位**：与 ALT_HOLD 一致，>60° 后补偿不再增大，此时只能靠减倾角。
4. **不要在低油门/怠速时全额补偿**：把补偿只作用在 `throttle − PWM_RANGE_MIN` 偏移量上（方案 B 写法），避免油门杆在底部时反而被抬升。
5. **与 `thr_corr_value` 的关系**：二者可叠加但无必要，新补偿启用后建议把 `thr_corr_value` 设回 0。
6. **`getCosTiltAngle()` 的符号问题**：翻过来（倒挂）时 `cos < 0`，必须用 `fmaxf(..., 0.5f)` 或判断 `<= 0` 跳过（参考 `imu.c:736` 的 `<= 0.015f` 保护）。

---

## 六、结论

1. **掉高的本质**：ANGLE 模式只控姿态不控油门；倾斜 θ 后垂直升力 = T·cosθ，油门不动必然下沉。45° 损失约 29% 升力，这是物理定律。
2. **代码里没有任何补偿在纯 ANGLE 模式下生效**：内置 `thr_corr` 默认关闭且是弱加法式；正确的乘法式 `1/cosθ` 只存在于 ALT_HOLD 的 `altitudeControl()`（`autopilot_multirotor.c:321`），ANGLE 模式不经过它。
3. **代码级解法**（三选一或组合）：
   - **方案 B**：在 `mixer.c` 油门链路给 ANGLE/HORIZON 加乘法式 `1/cosθ` 前馈（带 0.5 钳位 + 可调增益 k），代码量约 8 行，直接解决"倾斜掉高"。
   - **方案 A**：直接 ANGLE + ALT_HOLD，复用 fork 已有的闭环定高，不写代码。
   - **方案 C**：正式引入垂直速度闭环，是根治方案，但工程量最大（已有基础）。
4. 无论何种补偿，**电机推力余量是最终限制**；重机/高悬停油门时小倾角补偿有效、大倾角必然饱和。

---

> 本文仅分析，未修改任何源代码。代码行号以当前工作区 `Betaflight_Fork` 为准。
