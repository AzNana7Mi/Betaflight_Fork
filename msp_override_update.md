# MSP Override 上位机失联后回退遥控器控制 —— 考察报告

> 考察对象：Betaflight Fork
> 考察日期：2026-08-14
> 报告性质：代码分析 / 差异考察（未修改任何代码）

---

## 0. 结论摘要

1. **现象**：在 2026.6.1 版本中，`BOXMSPOVERRIDE` 模式激活、且设置了 `msp_override_channels_mask` 之后，如果上位机（companion / 地面站）停止发送 `MSP_SET_RAW_RC` 超过 **300 ms**，被屏蔽（override）的通道会自动**直接按遥控器（接收机）通道值输出**，即"上位机停发 → 直接跟随遥控器打杆"。
2. **根因**：2026.6.1 合入了提交 `61e775052`（PR #15217）"RX MSP Override: require fresh MSP RC data before substituting channels"，在 `src/main/rx/msp.c` 中引入了 **300 ms 数据新鲜度窗口** `RX_MSP_RC_FRAME_FRESH_MS` 和判定函数 `rxMspIsRcChannelFresh()`，并在 `rxMspOverrideReadRawRc()` 的 override 条件中新增了 `&& rxMspIsRcChannelFresh(chan)`。一旦超过窗口未收到新帧，条件不成立，函数返回实时接收机采样值。
3. **当前分支（master/experimental，`e5fd7e76e`）无此改动**：只要 `BOXMSPOVERRIDE` 模式激活，被屏蔽通道就持续使用 `mspFrame[]` 静态缓冲区里的**最后一帧 MSP 值**（保持最后一帧），上位机失联不会回退遥控器。
4. **需要修改的位置**（详见第 7 章）：
   - `src/main/rx/msp_override.c` 的 `rxMspOverrideReadRawRc()`（核心）；
   - `src/main/rx/msp.c` 的新鲜度逻辑（`rxMspIsRcChannelFresh()`、`RX_MSP_RC_FRAME_FRESH_MS`）；
   - `src/main/rx/rx.c` 的 `updateRxSignalReceived()`（仅"无 RC 链路自治"场景需要）；
   - `src/main/pg/rx.h` / `src/main/pg/rx.c` / `src/main/cli/settings.c`（若把失联动作做成可配置项）；
   - 单元测试 `src/test/unit/rx_msp_override_unittest.cc`。

---

## 1. 考察环境与版本

| 项目 | 内容 |
| --- | --- |
| 工作目录 | `/home/azure/Betaflight_Fork` |
| 2026.6.1 版本 | `HEAD = 6dbc4218f`（`Bump version to 2026.6.1`，`2026.6-maintenance` 线） |
| 当前版本（切换前） | `master = experimental = e5fd7e76e` |
| 关键差异提交 | `61e775052` RX MSP Override: require fresh MSP RC data before substituting channels (#15217)，2026-06-27 合入 |

版本确认命令：

```bash
git rev-parse HEAD              # 6dbc4218f  (2026.6.1)
git log --oneline -1 master     # e5fd7e76e  (切换前的当前版本)
git diff HEAD experimental -- src/main/rx/msp.c src/main/rx/msp_override.c src/main/rx/msp.h
```

> 说明：`src/main/rx/rx.c` 在两者之间还有一个无关的 scaleRange 重构（`scaleRangefApply` 预初始化 vs 运行时 `scaleRangef`），数学等价，与本问题无关，本文不展开。

---

## 2. 功能背景（MSP Override 是什么）

MSP Override 允许上位机通过 MSP 协议直接注入部分（或全部）RC 通道，用于远程 / 自治控制场景。相关配置与开关：

- **`msp_override_channels_mask`**（`src/main/pg/rx.h:63`，CLI 项 `src/main/cli/settings.c:883`）：位掩码，置位的通道将被 MSP 值替换；
- **`BOXMSPOVERRIDE` 模式**（`src/main/fc/rc_modes.h:81`，permanentId=50，`src/main/msp/msp_box.c:99`）：该模式的激活开关（可由遥控器 AUX 或 MSP 切换）；
- **`msp_override_failsafe`**（`src/main/pg/rx.h:64`，CLI 项 `src/main/cli/settings.c:884`）：为 `1` 时允许"仅靠 MSP 流维持信号有效"（无额外 RC 链路的自治场景）；为 `0` 时始终要求额外 RC 链路。
- 功能在 `src/main/target/common_pre.h:411` 通过 `USE_RX_MSP_OVERRIDE` 全局使能。

配置校验：`src/main/config/config.c:532-541`（掩码为 0 时移除 `BOXMSPOVERRIDE` 模式激活条件；模式使用的 AUX 通道若未被掩码覆盖则从掩码中剔除）。

---

## 3. 代码链路分析（数据流）

```
上位机 (ground station / companion)
   │  MSP_SET_RAW_RC (id=200)
   ▼
src/main/msp/msp.c:2887-2902   MSP 命令处理：解析 channelCount 个 u16 通道值
   │  rxMspFrameReceive(frame, channelCount)
   ▼
src/main/rx/msp.c:61-77        rxMspFrameReceive()
   │  - 写 mspFrame[] 静态缓冲区（未覆盖的通道尾部清零）
   │  - 记录 lastRxMspRcFrameMs / lastRxMspRcFrameChannelCount
   │  - 置 rxMspFrameDone / rxMspOverrideFrameDone / rxMspRcFrameEverReceived
   ▼
src/main/rx/rx.c:722-746       readRxChannelsApplyRanges()（每 PID 循环）
   │  若 msp_override_channels_mask ≠ 0 → rxMspOverrideReadRawRc()  (rx.c:730-733)
   ▼
src/main/rx/msp_override.c:30-43  rxMspOverrideReadRawRc()
   │  if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && override && rxMspIsRcChannelFresh(chan))
   │        → 返回 MSP 值 (constrainf(mspFrame[chan], rx_min_usec, rx_max_usec))
   │  else → 返回 rcReadRawFn() 的实时接收机值
   ▼
rcRaw[] → detectAndApplySignalLossBehaviour() (rx.c:748+) → rcData[] → 飞控/混控
```

另有**信号有效性与 failsafe** 链路：

```
src/main/rx/rx.c:634-642  updateRxSignalReceived()
   BOXMSPOVERRIDE 激活 && 掩码非零 && msp_override_failsafe=1
      且 rxMspOverrideFrameStatus() 返回 RX_FRAME_COMPLETE（说明刚收到过 MSP 帧）
      → rxSignalReceived = true（MSP 流被当作"信号有效"）
```

---

## 4. 2026.6.1 引入的关键改动（#15217 全貌）

`git show 61e775052` 涉及 5 个文件：`src/main/rx/msp.c`、`src/main/rx/msp.h`、`src/main/rx/msp_override.c`、`src/test/Makefile`、`src/test/unit/rx_msp_override_unittest.cc`。核心改动如下。

### 4.1 `src/main/rx/msp.c` 新增"新鲜度"机制

```c
// L36-41  新增静态状态量
static uint16_t mspFrame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
static bool rxMspFrameDone = false;
static bool rxMspOverrideFrameDone = false;
static bool rxMspRcFrameEverReceived = false;        // 是否收到过任何 MSP RC 帧
static timeMs_t lastRxMspRcFrameMs = 0;              // 最近一帧到达时刻
static uint8_t lastRxMspRcFrameChannelCount = 0;     // 最近一帧的通道数

// L50     300 ms 窗口（约 5Hz MSP 速率的余量）
#define RX_MSP_RC_FRAME_FRESH_MS 300

// L61-77  rxMspFrameReceive()：每收到一帧就刷新上述状态
// L79-88  判定函数（新增）
bool rxMspIsRcChannelFresh(uint8_t chan)
{
    if (!rxMspRcFrameEverReceived) return false;                  // 从未收到帧
    if (chan >= lastRxMspRcFrameChannelCount) return false;       // 该帧未覆盖此通道
    return (millis() - lastRxMspRcFrameMs) <= RX_MSP_RC_FRAME_FRESH_MS;  // 300ms 内
}
```

### 4.2 `src/main/rx/msp_override.c` override 条件收紧

```c
// 当前版本 (e5fd7e76e)：只要模式激活 + 掩码置位就替换
if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && override) { return overrideSample; }

// 2026.6.1 (HEAD)：还必须"通道数据新鲜"
if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && override && rxMspIsRcChannelFresh(chan)) {
    return overrideSample;
} else {
    return rxSample;    // ← 上位机失联 > 300ms 后走这里：回退遥控器实时值
}
```

---

## 5. 行为差异对比

前提：`BOXMSPOVERRIDE` 已激活、`msp_override_channels_mask` 覆盖 ROLL/PITCH/YAW/AUX、接收机（遥控器）链路正常。

| 场景 | 当前版本 (e5fd7e76e) | 2026.6.1 |
| --- | --- | --- |
| 上位机正常发帧（如 10~20Hz） | 屏蔽通道用 MSP 实时值 | 屏蔽通道用 MSP 实时值（**一致**） |
| 上位机停发 ≤ 300ms | 保持最后一帧 MSP 值 | 保持最后一帧 MSP 值（仍在窗口内） |
| **上位机停发 > 300ms** | **保持最后一帧 MSP 值（不回退）** | **回退到接收机实时值 → 遥控器控制**（本报告考察的现象） |
| 上位机从未发过帧 | `mspFrame[]` 全 0 → 被 `constrainf` 钳到 `rx_min_usec`（历史 BUG，即 #15217 要修的问题） | 视为不新鲜 → 回退接收机值（安全） |
| 只发 AETR 短帧但掩码含 AUX | AUX 读 `mspFrame[]` 清零尾部 → 钳到 `rx_min_usec`（历史 BUG） | AUX 视为不新鲜 → 回退接收机值（安全） |
| 无 RC 链路（自治）、上位机停发、`msp_override_failsafe=1` | 通道保持最后一帧 MSP 值；信号方面 `rxMspOverrideFrameStatus()` 为 PENDING → `rxSignalReceived` 变 false → 仍会进 failsafe | 通道回退接收机值；信号同样变 false → 同样进 failsafe |

**结论**：2026.6.1 与当前版本的行为差异**完全由 300ms 新鲜度窗口引起**。该窗口同时是"修复钳位 BUG"和"产生回退遥控器行为"的同一把双刃剑。

> 补充说明（2026.6.1 的"直接按遥控器通道"机制细节）：该回退是**逐通道、逐控制循环实时判断**的，没有"锁存/一次性切换"逻辑——
> - 停发 >300ms 后，`rxMspIsRcChannelFresh(chan)` 返回 false，`rxMspOverrideReadRawRc()` 的 else 分支把 `rcReadRawFn()`（即 CRSF/SBUS/SPI 等接收机协议驱动的实时通道值）直接作为该通道的采样值，并经 `readRxChannelsApplyRanges()` 正常做范围校准后进入 `rcRaw[]/rcData[]`；
> - 上位机一旦恢复发送（哪怕只发一帧 `MSP_SET_RAW_RC` 覆盖该通道），`rxMspFrameReceive()` 立刻刷新时间戳，下一循环即切回 MSP 值——切换是"新鲜就 MSP、不新鲜就遥控器"的持续仲裁，而非切换一次后保持。


---

## 6. 与失控保护（failsafe）的交互

- `rx.c:634-642`：`msp_override_failsafe=1` 时，MSP 帧到达后 `rxMspOverrideFrameStatus()` 返回一次 `RX_FRAME_COMPLETE`（随后清标志），借此把 `rxSignalReceived` 置 true 并延长 150ms 的 `needRxSignalBefore`。**上位机停发后，约 150ms 内无 MSP 帧，`rxSignalReceived` 变为 false**。
- 若同时有正常 RC 链路 → `rxSignalReceived` 仍被接收机帧维持为 true，**不会触发 failsafe**，只是"通道值回退遥控器"。这正是用户观察到的现象场景。
- 若没有 RC 链路（纯自治）+ `msp_override_failsafe=1` → 上位机停发后 `rxSignalReceived=false`，在 failsafe_delay 之后进入 failsafe Stage 2，`detectAndApplySignalLossBehaviour()`（rx.c:748+）会进一步用 failsafe 值覆盖通道。**因此"保持最后一帧"若要完整生效，自治场景下还需同步处理该信号链路**（见第 7.2 节）。

---

## 7. 需要修改的地方（修改方案分析）

以下按"改动从小到大"给出四个方案。**推荐方案 B（或 B+C 组合）**。所有改动点均给出精确位置。

### 方案 A：完全回退（撤销 #15217）

- 改动点：`src/main/rx/msp_override.c:38` 去掉 `&& rxMspIsRcChannelFresh(chan)`；`src/main/rx/msp.c` 删掉新增状态量/函数；`src/main/rx/msp.h` 删声明。
- 效果：恢复"保持最后一帧"，但**重新引入** #15217 修复的两个 BUG：
  1. 上位机从未发帧 / 提前停发时，屏蔽通道被钳位到 `rx_min_usec`（摇杆到底）；
  2. AETR 短帧导致被掩码覆盖的 AUX 通道读 0 钳位。
- **不推荐**。除非产品上完全可控上位机的发送行为。

### 方案 B（推荐）：stale 时"保持最后一帧"，从未收到时仍回退（HOLD 语义）

在保留新鲜度机制的前提下，把"stale 的返回值"从"回退遥控器"改为"保持最后一帧 MSP 值"。

1. **`src/main/rx/msp.c`**：
   - 把 `rxMspIsRcChannelFresh()`（L79-88）中的 300ms 时间条件拆出，新增/暴露一个"是否收到过该通道有效数据"的判定，例如：
     ```c
     bool rxMspHasRcChannelData(uint8_t chan)
     {
         if (!rxMspRcFrameEverReceived) return false;
         if (chan >= lastRxMspRcFrameChannelCount) return false;
         return true;   // 收到过且该帧覆盖该通道；不含时间窗口
     }
     ```
     或在 `rxMspIsRcChannelFresh()` 内把 300ms 窗口参数化。
   - 在 `src/main/rx/msp.h` 增加声明。
2. **`src/main/rx/msp_override.c` `rxMspOverrideReadRawRc()`（L30-43）**，把 L38 的逻辑改为：
   ```c
   if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && override && rxMspHasRcChannelData(chan)) {
       return overrideSample;   // fresh → 实时 MSP；stale → 保持最后一帧 mspFrame 值
   } else {
       return rxSample;         // 从未收到该通道数据 → 回退遥控器（保留 #15217 安全性）
   }
   ```
   注意 `overrideSample = constrainf(rxMspReadRawRC(...))` 已经直接读 `mspFrame[]`，stale 时返回的正是"最后一帧"，无需额外保存。
- 效果：上位机停发后，**被屏蔽通道保持最后一帧 MSP 命令**（不回退遥控器）；同时"从未发帧 / 部分帧"的钳位 BUG 依然被挡住。
- 风险：油门通道若在最后一帧为满油门，失联后会持续保持 → 见第 8 节；建议对失联时间引入二级超时（如 2~5s 后切到安全动作），或仅对 RPY/AUX 生效、油门按策略处理。

### 方案 C（推荐与 B 组合）：把"失联动作"做成可配置项

新增 CLI 参数（沿用 `msp_override_failsafe` 的 PG/CLI 模式），例如 `msp_override_stale_action`：

- 改动点：
  1. `src/main/pg/rx.h:64` 附近：`rxConfig_t` 增加字段，如 `uint8_t msp_override_stale_action;`（0=回退遥控器，1=保持最后一帧 HOLD，2=立即进入 failsafe / 安全动作）；
  2. `src/main/pg/rx.c:116` 附近：设置默认值（按产品需要，例如 HOLD）；
  3. `src/main/cli/settings.c:883-884` 附近：注册 CLI 项（`VAR_UINT8 | MASTER_VALUE | MODE_LOOKUP`，LOOKUP 表 `TABLE_OFF_ON` 扩展为三态）；
  4. 可选 `src/main/config/config.c:532-541` 附近：取值范围校验；
  5. `src/main/rx/msp_override.c`：按该字段分支（方案 B 的 HOLD 或现有 2026.6.1 的 RC 回退）。
- 好处：不改动上游默认行为也能满足本 fork 需求，且同一固件可适配不同产品线。

### 方案 D：仅把 300ms 窗口参数化/调大

- 改动点：`src/main/rx/msp.c:50` 的 `RX_MSP_RC_FRAME_FRESH_MS` 改为配置项。
- 局限：只能推迟回退，无法做到"失联即保持"；若上位机长时间无信号，仍会回退遥控器。**建议仅作为辅助选项**。

### 7.2 自治场景（无 RC 链路）需要同步修改的信号链路

如果目标场景是"纯上位机控制、无遥控器链路"（`msp_override_failsafe=1`），仅改通道值还不够，因为上位机停发后 `rxSignalReceived` 会变 false 并最终触发 failsafe：

- `src/main/rx/msp.c:101-111` `rxMspOverrideFrameStatus()`：可在"新鲜度窗口内（或 HOLD 有效期内）"返回 `RX_FRAME_COMPLETE`，把"MSP 流仍在控制"的信号语义维持住；
- `src/main/rx/rx.c:634-642`：相应地在 HOLD 期间维持 `rxSignalReceived=true`；
- 注意：这会推迟甚至禁用 failsafe，需权衡安全性（上位机失联但电机持续输出的风险）。

### 7.3 配套测试改动

- `src/test/unit/rx_msp_override_unittest.cc`：现有用例覆盖"新鲜/不新鲜"判定（第 46-89 行）。若实现方案 B/C，需新增：
  - stale 时 `rxMspOverrideReadRawRc()` 返回最后一帧 MSP 值（HOLD）；
  - 从未收到帧时仍回退 `rxSample`；
  - 配置项不同取值的行为分支。
- `src/test/Makefile:367-372`：`rx_msp_override_unittest` target 已存在，无需新增。
- 运行：`make test`（本仓库 unittest 为宿主 x86 构建）。

---

## 8. 修改风险评估

| 风险 | 说明 | 缓解 |
| --- | --- | --- |
| 油门保持 | HOLD 语义下上位机失联，油门停留在最后一帧；若为满油门，飞机会持续输出动力 | 对油门/主控制通道做 HOLD 超时（如 1~5s）后转安全动作；或由方案 C 配置 |
| 延迟 failsafe | 若同时修改 7.2 的信号链路，failsafe 触发条件被推迟，可能掩盖上位机异常 | 分离"通道值策略"与"failsafe 策略"两个开关，failsafe 时间窗保持独立可调 |
| 与上游升级冲突 | 后续 merge upstream 的 msp 相关改动时，`rxMspIsRcChannelFresh` 与新增函数会产生冲突 | 改动集中、注释完整；尽量通过配置项兼容上游默认行为 |
| 短帧/部分通道 | 若上位机只发 AETR，掩码中的 AUX 在方案 B 下会因"通道数据从未覆盖"而回退遥控器——行为正确但与 HOLD 直觉不同 | 文档注明"仅覆盖帧内包含的通道"；必要时按帧内通道数补齐帧长 |

---

## 9. 验证方法

1. 单元测试：`src/test/unit/rx_msp_override_unittest.cc` 覆盖新鲜度与 HOLD/回退分支；`make test` 通过。
2. 板级验证（配合上位机模拟失联）：
   - 正常发帧 → 摇杆不动、屏蔽通道跟随 MSP 值；
   - 停发 > 300ms → 期望（方案 B）：屏蔽通道保持最后一帧 MSP 值；遥控器对屏蔽通道无效；
   - 停发 > HOLD 超时（若实现）→ 按配置进入安全动作/failsafe；
   - 从未发帧直接激活模式 → 屏蔽通道回退遥控器且不出现钳位（回归验证 #15217 修复不丢）。
3. 观察 DEBUG 通道 `DEBUG_RX_SIGNAL_LOSS`（rx.c:606,645）与 CLI `status` 中的信号/失控状态，确认与预期一致。

---

## 10. 同步方案：如何把 experimental 分支的 MSP Override 行为同步到 2026.6.1

### 10.1 同步的本质（先澄清版本关系）

- 已核实：`experimental`（`e5fd7e76e`）的三个 msp 文件（`src/main/rx/msp.c`、`msp.h`、`msp_override.c`）与 #15217 改动前（`61e775052~1`）**完全一致**（`git diff 61e775052~1 experimental -- src/main/rx/...` 输出为空）。
- 因此"experimental 的 MSP Override 行为" = **无 300ms 新鲜度窗口、模式激活即持续使用最后一帧 MSP 值**。
- 需要澄清：官方 2026.6.0-rc1~rc3、2026.6.1 版本线**均已包含** #15217（`git tag --contains 61e775052` 结果）。experimental 实际基于 #15217 合入**之前**的上游 master 快照，并非官方 2026.6.0 固件的行为。所以这里的"同步"= 把 experimental 的**旧行为**移植到 2026.6.1 代码基。
- 目标：在 2026.6.1（HEAD=`6dbc4218f`）上实现"上位机停发后，屏蔽通道保持最后一帧 MSP 值（不回退遥控器）"。

### 10.2 feature 差异清单（experimental → HEAD）

| 文件 | 差异内容 | 是否 feature 相关 |
| --- | --- | --- |
| `src/main/rx/msp.c` | 新增 `rxMspRcFrameEverReceived` / `lastRxMspRcFrameMs` / `lastRxMspRcFrameChannelCount` 三个静态变量、`RX_MSP_RC_FRAME_FRESH_MS=300` 宏、`rxMspIsRcChannelFresh()` 函数，及 `rxMspFrameReceive()` 内 3 行时间戳/计数更新 | **是** |
| `src/main/rx/msp.h` | 新增 `rxMspIsRcChannelFresh()` 声明 | **是** |
| `src/main/rx/msp_override.c` | L38 条件增加 `&& rxMspIsRcChannelFresh(chan)` | **是** |
| `src/main/rx/rx.c` | 仅 scaleRange 预初始化重构（数学等价）；L634-642 / L730-733 的 override 调用与 failsafe 逻辑两分支一致 | 否 |
| `pg/rx.h`、`pg/rx.c`、`cli/settings.c`、`config/config.c`、`msp/msp.c`、`msp/msp_box.c` | `msp_override_*` 相关代码两分支**完全一致**（已 diff 验证，无任何差异输出） | 否 |

**结论：feature 相关的代码差异只有 3 个文件**。其余文件的差异与 MSP Override 无关，同步时不要做整树合并。

### 10.3 三种同步方式

**方式一：`git revert 61e775052`（最直接，已验证可干净应用）**

```bash
git revert --no-commit 61e775052   # 建议先审查，确认后 git commit
```

- 已在 HEAD 上 dry-run 验证：`src/main/rx/msp.c`、`msp.h`、`msp_override.c`、`src/test/Makefile` 自动合并成功，`src/test/unit/rx_msp_override_unittest.cc` 被删除，**无冲突**。
- 效果：完全回到 experimental 行为（保持最后一帧，无 300ms 回退）。
- 代价：重新引入 #15217 修复的两个 BUG——上位机从未发帧 → 屏蔽通道钳位到 `rx_min_usec`（摇杆到底）；AETR 短帧 → 被掩码覆盖的 AUX 读 0 钳位。**仅适用于能完全保证上位机发送行为的产品**。
- 注意：revert 会留下"撤销 #15217"的历史记录；若后续上游再合入该功能的变体，会产生 merge 冲突。

**方式二（推荐）：第 7 章方案 B（HOLD 语义，等价行为 + 保留防护）**

- 只改 2 处：`src/main/rx/msp.c` 新增"是否收到过该通道数据"的判定（不含 300ms 时间条件），`src/main/rx/msp_override.c` L38 在 stale 时返回最后一帧 `overrideSample`、从未收到时回退 `rxSample`。
- 行为与 experimental **在正常场景完全等价**（发帧后停发 → 保持最后一帧），边缘场景（从未发帧/短帧）更安全。

**方式三：文件级覆盖（不推荐）**

```bash
git checkout experimental -- src/main/rx/msp.c src/main/rx/msp.h src/main/rx/msp_override.c
```

- 效果等同方式一，但会把 experimental 上的其他私有改动一并带入，且历史不清晰。仅当 experimental 后续还有未上游化的私有改动时才考虑。

### 10.4 需要修改底层驱动吗？—— 不需要

明确结论：**同步该功能不需要修改任何设备底层驱动。**

理由（基于全仓代码检索）：

1. **涉及文件全部在应用/协议层**：`grep -rln 'USE_RX_MSP_OVERRIDE\|msp_override' src/main/` 命中的 10 个文件全部位于 `pg/`（配置）、`cli/`、`config/`、`msp/`、`rx/`、`target/` 层，**没有任何 `drivers/` 目录文件**（UART/SPI/I2C/DMA/定时器/IO/外部存储等驱动均不涉及）。
2. **数据路径不触碰硬件**：上位机字节由既有 serial/MSP 基础设施接收 → `src/main/msp/msp.c:2887-2902` 的 `MSP_SET_RAW_RC` 命令处理器调用 `rxMspFrameReceive()` 写入 `mspFrame[]` 静态数组 → `src/main/rx/msp_override.c` 做纯逻辑"通道选择"（返回 MSP 值还是接收机值）。全程没有寄存器、中断、DMA 操作。
3. **`USE_RX_MSP_OVERRIDE` 是编译宏，不是驱动**：`src/main/target/common_pre.h:411` 全局使能，`common_post.h:453-454` 仅是"未启用 `USE_RX_MSP` 时自动取消"的保护逻辑；两者都是构建配置，不需要按设备修改。
4. **接收机侧复用现有协议驱动**：`rcReadRawFn`（CRSF/SBUS/SPI 等）在 2026.6.1 已具备；MSP Override 只是在它们的结果之上做"是否替换"的上层决策。

唯一需要做的"设备相关"检查（注意是**检查**，不是**修改**）：

- 确认目标固件构建时 `USE_RX_MSP_OVERRIDE` 宏生效（默认全局使能，仅当某 target 自定义 `#undef` 时才需要关注）；
- 确认 CLI 能读写 `msp_override_channels_mask` / `msp_override_failsafe`（PG 参数，存 eeprom，不涉及底层驱动）。

### 10.5 同步后的验证

- 单元测试：`make test`（`rx_msp_override_unittest` target；方式一 revert 后该测试被删除，属预期）。
- 构建：对实际使用目标执行 `make TARGET=<目标>`，确认编译通过。
- 板级验证（同第 9 章清单）：
  - 正常发帧 → 屏蔽通道跟随 MSP 值；
  - 停发 > 300ms → 屏蔽通道**保持最后一帧**，遥控器对屏蔽通道无效；
  - 从未发帧直接激活模式 → 回退遥控器且不出现钳位（仅方式二有此保障）；
  - 无 RC 链路自治场景 → 按第 7.2 节评估 failsafe 行为。
- CLI：`get msp_override_channels_mask` / `get msp_override_failsafe` 正常读写。

### 10.6 注意事项

- **官方 2026.6.0 / 2026.6.1 都带 300ms 窗口**：如果需求被描述为"官方 2026.6.0 行为"，那它与 6.1 一致（都有窗口），仍需按方式二（或第 7 章方案 C）自行改造，才能恢复 experimental 的"保持最后一帧"行为。
- 建议把"失联动作"做成配置项（第 7 章方案 C），避免后续 merge 上游 master 时反复冲突。
- 若用 `git revert`，请在提交信息/代码注释中标明本地策略（如 `// 本地策略：MSP override 失联保持最后一帧`），防止团队后续同步上游时被再次合入窗口逻辑。

### 10.7 反向同步：把 2026.6.1 的"300ms 新鲜度 + 失联回退遥控器"特性移植到 experimental 分支

使用场景：在 experimental 分支做相关开发、暂不能升级版本，但想先用上 6.1 的这个新特性。

**前置验证结论（本报告已实测）**：

1. experimental（`e5fd7e76e`）的 `msp.c / msp.h / msp_override.c` 与 #15217 改动前（`61e775052~1`）完全一致 → 该提交可直接移植，不需要做任何适配改写；
2. 特性所需基础设施在 experimental **均已具备**：`USE_RX_MSP_OVERRIDE`（`common_pre.h:404`）、`BOXMSPOVERRIDE`（`msp_box.c:99/362`）、`timeMs_t`（`common/time.h:33`）、`millis()`（`drivers/time.h:32`，且 `rx/msp.c` 已 include `drivers/time.h`）；
3. 在 experimental 基座上 dry-run：`git cherry-pick --no-commit 61e775052` **零冲突**（3 个源文件直接应用，`src/test/Makefile` 自动合并，新增单元测试文件）；
4. 移植后在 experimental 基座实际编译并运行 `rx_msp_override_unittest`：**编译通过、测试 PASS**（`MspOverrideTest.ChannelFreshness` OK）。

**操作方法**：

```bash
# 方式 A（推荐）：直接 cherry-pick 上游提交 61e775052
git checkout experimental
git cherry-pick 61e775052
# 复查：git log -1 为该提交；
#       src/main/rx/msp.c 含 300ms 新鲜度逻辑；
#       src/main/rx/msp_override.c L38 条件含 rxMspIsRcChannelFresh(chan)
```

方式 B（不想要单元测试时只移植功能代码）：按 10.2 节差异清单手工改 3 个源文件——
`msp.c` 新增 3 个静态变量 + `RX_MSP_RC_FRAME_FRESH_MS` 宏 + `rxMspIsRcChannelFresh()` 函数 + `rxMspFrameReceive()` 内 3 行时间戳/计数更新；`msp.h` 加函数声明；`msp_override.c` L38 条件追加 `&& rxMspIsRcChannelFresh(chan)`。

**移植后注意事项**：

- 窗口可调：`RX_MSP_RC_FRAME_FRESH_MS`（`msp.c:50`）默认 300ms，按需直接改宏；
- 移植后即获得 6.1 完整行为：上位机停发 >300ms → 被屏蔽通道**直接按遥控器通道值输出**；恢复发帧（下一循环）立即切回 MSP 值；
- 该提交只涉及 5 个文件（3 源 + test/Makefile + 新增测试），**不会带入 experimental 不需要的其他 6.1 改动**（如 rx.c 的 scaleRange 重构）；
- 单元测试：`make test_rx_msp_override_unittest`；若本机 clang 缺 `libBlocksRuntime`，需安装该库或用空 stub 库注入 `LIBRARY_PATH`/`LD_LIBRARY_PATH`；
- 若 experimental 后续对上述文件有更多本地改动导致 cherry-pick 冲突，按 10.2 节差异清单手工解决即可。

---
## 11. 附录：关键文件与行号索引（以 HEAD=6dbc4218f 为准）

| 文件 | 位置 | 内容 |
| --- | --- | --- |
| `src/main/rx/msp_override.c` | L30-43 | `rxMspOverrideReadRawRc()` —— **核心修改点（L38）** |
| `src/main/rx/msp.c` | L36-50 | 静态状态量 + 300ms 窗口 |
| `src/main/rx/msp.c` | L61-77 | `rxMspFrameReceive()` |
| `src/main/rx/msp.c` | L79-88 | `rxMspIsRcChannelFresh()` |
| `src/main/rx/msp.c` | L101-111 | `rxMspOverrideFrameStatus()` |
| `src/main/rx/msp.h` | L26-29 | 函数声明 |
| `src/main/rx/rx.c` | L634-642 | `msp_override_failsafe` 信号维持逻辑 |
| `src/main/rx/rx.c` | L722-746 | `readRxChannelsApplyRanges()`（L730-733 调用 override） |
| `src/main/rx/rx.c` | L748+ | `detectAndApplySignalLossBehaviour()` |
| `src/main/msp/msp.c` | L2887-2902 | `MSP_SET_RAW_RC` 处理 → `rxMspFrameReceive()` |
| `src/main/pg/rx.h` | L63-64 | `msp_override_channels_mask` / `msp_override_failsafe` |
| `src/main/pg/rx.c` | L116 | 默认值 `.msp_override_channels_mask = 0` |
| `src/main/cli/settings.c` | L883-884 | CLI 项注册 |
| `src/main/config/config.c` | L532-541 | 掩码/模式激活条件校验 |
| `src/main/msp/msp_box.c` | L99, L364-368 | `BOXMSPOVERRIDE` 模式注册 |
| `src/main/target/common_pre.h` | L411 | `USE_RX_MSP_OVERRIDE` 使能 |
| `src/test/unit/rx_msp_override_unittest.cc` | 全文 | 新鲜度判定测试 |
| `src/test/Makefile` | L367-372 | unittest target |

### 历史提交时间线

```
aa5066e44  Add MSP override mode                       (功能引入)
24e7dabed  Ensure MSP channel data is valid (#13352)   (数据合法性)
a0c0e191e  Do not go into failsafe ... (#13380)        (msp_override_failsafe)
61e775052  Require fresh MSP RC data ... (#15217)      (300ms 新鲜度 → 本报告根因)
```

---

## 12. 反向同步逐文件修改清单（6.1 → experimental，完整代码/位置/注释）

> 说明：以下行号均以 `git show experimental:<file>`（experimental=`e5fd7e76e`）为准；代码与注释逐字取自提交 `61e775052`（#15217），可原样照抄。已实测：按此清单修改后，在 experimental 基座上编译并通过 `rx_msp_override_unittest`。

### 12.1 `src/main/rx/msp.c`（共 3 处改动，+26 行）

**改动点 1：新增 3 个静态状态变量 + 300ms 窗口宏（含上游注释）**

- 位置：第 38 行 `static bool rxMspOverrideFrameDone = false;` 之后插入
- 代码：

```c
static bool rxMspRcFrameEverReceived = false;        // 是否收到过任何 MSP RC 帧
static timeMs_t lastRxMspRcFrameMs = 0;              // 最近一帧到达时刻（ms）
static uint8_t lastRxMspRcFrameChannelCount = 0;     // 最近一帧包含的通道数

// Substitute MSP RC values into the override path only if a frame containing
// the channel arrived within this window. Without this guard:
//  - empty mspFrame[] (companion never sent, or stopped) would clamp masked
//    channels to rx_min_usec the instant BOXMSPOVERRIDE activated;
//  - a short MSP_SET_RAW_RC frame (e.g. AETR-only) would still leave any
//    masked AUX channel reading from the zero-filled tail of mspFrame[].
// 300 ms gives honest support for ~5 Hz MSP RC with timing margin.
#define RX_MSP_RC_FRAME_FRESH_MS 300
```

**改动点 2：`rxMspFrameReceive()` 内更新新鲜度状态**

- 位置：第 61 行 `rxMspOverrideFrameDone = true;` 之后追加 3 行
- 代码：

```c
    rxMspFrameDone = true;
    rxMspOverrideFrameDone = true;
    lastRxMspRcFrameMs = millis();
    lastRxMspRcFrameChannelCount = channelCount;
    rxMspRcFrameEverReceived = true;
```

**改动点 3：新增 `rxMspIsRcChannelFresh()` 函数**

- 位置：`rxMspFrameReceive()` 结束（第 62 行 `}`）之后、第 64 行 `static uint8_t rxMspFrameStatus(...)` 之前插入
- 代码：

```c
bool rxMspIsRcChannelFresh(uint8_t chan)
{
    if (!rxMspRcFrameEverReceived) {
        return false;
    }
    if (chan >= lastRxMspRcFrameChannelCount) {
        return false;
    }
    return (millis() - lastRxMspRcFrameMs) <= RX_MSP_RC_FRAME_FRESH_MS;
}
```

### 12.2 `src/main/rx/msp.h`（+1 行）

- 位置：第 28 行 `uint8_t rxMspOverrideFrameStatus(void);` 之后追加
- 代码：

```c
bool rxMspIsRcChannelFresh(uint8_t chan);
```

### 12.3 `src/main/rx/msp_override.c`（改 1 行，核心行为改动）

- 位置：第 38 行 `rxMspOverrideReadRawRc()` 内的判断条件
- 代码（改前 / 改后）：

```c
// 改前：只要模式激活 + 掩码置位就替换
    if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && override) {
// 改后：还要求该通道最近有新鲜 MSP 数据；否则回退到接收机（遥控器）实时值
    if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && override && rxMspIsRcChannelFresh(chan)) {
```

### 12.4 `src/test/Makefile`（+8 行）

- 位置：第 301 行 `rx_ranges_unittest_SRC := \` 之前插入（`rx_ibus_unittest` 定义块之后）
- 代码：

```make
rx_msp_override_unittest_SRC := \
		$(USER_DIR)/rx/msp.c

rx_msp_override_unittest_DEFINES := \
		USE_RX_MSP= \
		USE_RX_MSP_OVERRIDE=

```

### 12.5 新增文件 `src/test/unit/rx_msp_override_unittest.cc`（89 行，全文件）

- 位置：新建文件，路径 `src/test/unit/rx_msp_override_unittest.cc`
- 完整内容：

```cpp
/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>

extern "C" {
    #include "platform.h"

    #include "pg/rx.h"
    #include "rx/rx.h"
    #include "rx/msp.h"
}

#include "gtest/gtest.h"

static uint32_t simulationMillis = 0;

extern "C" {
    uint32_t millis(void) { return simulationMillis; }
    uint32_t micros(void) { return simulationMillis * 1000; }
}

// rxMspIsRcChannelFresh returns true only when the most recent MSP_SET_RAW_RC
// frame both (a) included the requested channel and (b) arrived within the
// freshness window. Without these guards an empty or short MSP frame would
// allow rxMspOverrideReadRawRc() to clamp masked channels to rx_min_usec.
TEST(MspOverrideTest, ChannelFreshness)
{
    // Before any frame is received, every channel is stale.
    simulationMillis = 1000;
    EXPECT_FALSE(rxMspIsRcChannelFresh(0));
    EXPECT_FALSE(rxMspIsRcChannelFresh(3));
    EXPECT_FALSE(rxMspIsRcChannelFresh(8));

    // A 4-channel (AETR-only) frame makes channels 0..3 fresh; AUX channels
    // beyond the frame's channelCount remain stale even though the frame
    // itself is current.
    uint16_t frame4[MAX_SUPPORTED_RC_CHANNEL_COUNT] = {1500, 1500, 1500, 1000};
    rxMspFrameReceive(frame4, 4);
    EXPECT_TRUE(rxMspIsRcChannelFresh(0));
    EXPECT_TRUE(rxMspIsRcChannelFresh(3));
    EXPECT_FALSE(rxMspIsRcChannelFresh(4));
    EXPECT_FALSE(rxMspIsRcChannelFresh(8));

    // Inside the freshness window: still fresh.
    simulationMillis = 1000 + 250;
    EXPECT_TRUE(rxMspIsRcChannelFresh(0));
    EXPECT_FALSE(rxMspIsRcChannelFresh(4));

    // A 6-channel frame extends freshness to AUX2 (channel 5).
    simulationMillis = 1500;
    uint16_t frame6[MAX_SUPPORTED_RC_CHANNEL_COUNT] = {1500, 1500, 1500, 1000, 2000, 1750};
    rxMspFrameReceive(frame6, 6);
    EXPECT_TRUE(rxMspIsRcChannelFresh(0));
    EXPECT_TRUE(rxMspIsRcChannelFresh(5));
    EXPECT_FALSE(rxMspIsRcChannelFresh(6));

    // Past the freshness window every channel becomes stale, regardless of
    // whether earlier frames had covered it.
    simulationMillis = 1500 + 301;
    EXPECT_FALSE(rxMspIsRcChannelFresh(0));
    EXPECT_FALSE(rxMspIsRcChannelFresh(5));

    // A subsequent shorter frame shrinks coverage: a previously-fresh AUX
    // channel must drop back to stale even while channels 0..3 stay fresh.
    simulationMillis = 2000;
    rxMspFrameReceive(frame4, 4);
    EXPECT_TRUE(rxMspIsRcChannelFresh(3));
    EXPECT_FALSE(rxMspIsRcChannelFresh(5));
}
```

### 12.6 修改后验证

```bash
# 1) 单元测试（experimental 基座，已实测 PASS）
make test_rx_msp_override_unittest
#    若本机 clang 缺 libBlocksRuntime：安装该库，
#    或用空 stub 库注入 LIBRARY_PATH / LD_LIBRARY_PATH 后重新链接/运行

# 2) 目标固件编译（用实际使用的 target）
make TARGET=<你的目标>          # 或 make -j4 <你的目标>

# 3) 板级行为验证
#    - 上位机正常发帧：屏蔽通道跟随 MSP 值；
#    - 上位机停发 >300ms：屏蔽通道直接按遥控器通道值输出；
#    - 恢复发帧：下一循环立即切回 MSP 值。
```





