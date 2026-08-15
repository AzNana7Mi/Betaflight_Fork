# Betaflight OSD 系统实现原理分析报告

> 分析基于: `Betaflight_Fork` 飞控代码库（本报告只做代码分析，未修改任何代码）
> 报告日期：2026-08-15

---

## 0. 结论摘要

1. **OSD（屏上显示）是一个"状态机 + 任务切片"驱动的文本渲染系统**：调度器每帧唤醒一次 `TASK_OSD`（默认 12 Hz），由 `osdUpdateCheck` 判断是否需要刷新，`osdUpdate` 分多个状态把一帧的绘制工作切成多次任务执行（防止单次任务占用过长）。
2. **所有显示目标通过 `displayPort_t` 抽象层（vtable）统一访问**，OSD 逻辑本身与具体硬件无关。具体实现有：`MAX7456`（模拟图传、板载芯片）、`MSP DisplayPort`（数字图传及外部 MSP OSD 器件）、`FB_OSD`（framebuffer 像素级 OSD）、`FRSKYOSD/OLED/CRSF/HOTT/SRXL`（遥控器/外设屏幕，非视频叠加）。
3. **模拟图传（PAL/NTSC）OSD 的经典方案是 MAX7456 芯片**：它是一颗专用的"视频叠加字符发生器"，FC 通过 SPI 只写入"字符索引"（文字/符号编码），芯片自己把字符网格叠加进模拟视频信号。**必须有这颗芯片**（或同功能器件）。
4. **数字图传（DJI / HDZero / Walksnail 等）不需要 MAX7456**：FC 通过 UART 用 **MSP DisplayPort** 协议把"字符+坐标"推送给眼镜/VTX，由眼镜端 GPU/渲染引擎完成叠加。
5. **MSP OSD 的本质**：FC 把自己当作 MSP 客户端，把每一次显示原语（清屏、写字符串、画屏、心跳等）编码成 `MSP_DISPLAYPORT`(182) 报文主动推送到串口，对端（眼镜/图传/MSP 芯片）负责渲染。**FC 自身不渲染任何像素。**
6. **MSP OSD 可以用于模拟图传**：MSP DisplayPort 协议与视频制式无关。只要 UART 对端是一个"能接收 MSP DisplayPort 并自行叠加到模拟视频"的设备（例如代码中显式支持的 **Airbot Theia OSD** 这类 MSP 芯片，见 `USE_MSP_DISPLAYPORT_FONT` / `OSD_FLAGS_OSD_HARDWARE_AIRBOT_THEIA_OSD`），就能在无 MAX7456 的情况下通过 UART 给模拟图传提供 MSP OSD。**普通"裸"模拟图传（无 OSD 处理器）做不到**——它只透传视频信号。

---

## 1. 代码文件地图

| 文件 | 职责 |
| --- | --- |
| `src/main/osd/osd.c` | OSD 核心：初始化、状态机、统计页、armed 屏、配置文件 |
| `src/main/osd/osd.h` | 元素枚举 `osd_items_e`、统计枚举 `osd_stats_e`、`osdConfig_t` 等 |
| `src/main/osd/osd_elements.c/.h` | 元素渲染表、活跃元素管理、单元素绘制、背景/前景分层 |
| `src/main/osd/osd_warnings.c/.h` | OSD 告警元素（电池/信号/失联等闪烁提示） |
| `src/main/osd/osd_custom_text.c/.h` | 自定义文本消息（CraftName 叠加 LQ/RSSI/告警等） |
| `src/main/drivers/display.h/.c` | `displayPort_t` 抽象层与通用包装函数 |
| `src/main/io/displayport_max7456.c` | MAX7456 的 displayPort 适配 |
| `src/main/drivers/max7456.c/.h` | MAX7456 芯片驱动（SPI、寄存器、NVM 字库、DMA） |
| `src/main/io/displayport_msp.c/.h` | MSP DisplayPort 适配（把显示原语打包成 MSP 报文） |
| `src/main/io/displayport_fb_osd.c` | Framebuffer OSD 适配（像素级渲染，RP2350/PICO 目标） |
| `src/main/io/displayport_frsky_osd.c`、`frsky_osd.c` | FrSky 遥控器屏幕 OSD |
| `src/main/io/displayport_oled.c` / `displayport_crsf.c` / `displayport_hott.c` / `displayport_srxl.c` | OLED / CRSF / HoTT / SRXL 屏幕适配 |
| `src/main/fc/init.c` | OSD 初始化、displayPort 设备选择 |
| `src/main/fc/tasks.c` | `TASK_OSD` 调度任务注册 |
| `src/main/config/config.c` | 校验配置并定位 MSP DisplayPort 所在串口 |
| `src/main/msp/msp.c` | MSP 服务端：OSD 配置读写、字体上传、canvas 设置 |
| `src/main/msp/msp_serial.c` | `mspSerialPush()`：把 MSP 报文编码推送到串口 |
| `src/main/pg/vcd.c`、`pg/displayport_profiles.c`、`pg/max7456.c` | 视频系统/显示端口参数组（PG） |


---

## 2. 初始化调用路径

入口在 `src/main/fc/init.c` 的 `init()`（`// CMS, display devices and OSD` 段）：

```
init()
 ├─ mspInit(); mspSerialInit();          // MSP 栈先就绪（MSP DisplayPort 依赖它）
 ├─ cmsInit();                            // 字符菜单系统（与 OSD 共用 displayPort）
 │
 └─ if (featureIsEnabled(FEATURE_OSD)) {
      ├─ 设备选择:
      │    if (vcdProfile()->video_system == VIDEO_SYSTEM_HD)
      │        device = OSD_DISPLAYPORT_DEVICE_MSP;      // 数字图传强制走 MSP
      │    else
      │        device = osdConfig()->displayPortDevice;  // 默认 AUTO
      │
      │    switch (device) {  // AUTO 时的 fallback 顺序:
      │      FRSKYOSD → frskyOsdDisplayPortInit()
      │      MAX7456  → max7456DisplayPortInit()   // SPI 探测芯片
      │      FBOSD    → fbOsdDisplayPortInit()
      │      MSP      → displayPortMspInit()
      │      NONE     → ...
      │    }
      │
      ├─ osdInit(osdDisplayPort, device);
      │    ├─ cmsDisplayPortRegister(osdDisplayPort);     // 与 CMS 共用同一屏幕
      │    ├─ 同步 canvas_cols/rows、把越界元素拉回画布内
      │    └─ （未真正初始化显示内容，见状态机 OSD_STATE_INIT）
      │
      └─ if (device == NONE) featureDisableImmediate(FEATURE_OSD);
    }
```

**MSP DisplayPort 用的串口是怎么确定的？**

- 目标固件定义 `MSP_DISPLAYPORT_UART`（如 `AIRBOTSUPERF4V2` 的 `SERIAL_PORT_USART8`）时，`src/main/io/serial.c` 会把该串口的 `functionMask` 设为 `FUNCTION_VTX_MSP | FUNCTION_MSP`。
- 否则（通用方式）由配置器在串口配置里同时勾选 `VTX MSP` 与 `MSP` 功能，`src/main/config/config.c` 的 `validateAndFixConfig()` 扫描第一个同时带 `FUNCTION_VTX_MSP | FUNCTION_MSP` 的串口并调用 `displayPortMspSetSerial()` 记录下来。
- 该串口在 MSP 处理循环里是"显示专用"的：`msp_serial.c` 中对这个口不再评估非 MSP 数据（防误入 CLI 等）。

**`video_system`（PAL/NTSC/HD/AUTO）来自哪里？** `pg/vcd.c` 的 `vcdProfile`。HD 模式下 OSD 画布默认 53x20（`OSD_HD_COLS/ROWS`），SD 模式 30 列 x 16 行（PAL）/13 行（NTSC）。

---

## 3. 运行期调用路径（核心）

### 3.1 调度器入口

`src/main/fc/tasks.c`：

```c
[TASK_OSD] = DEFINE_TASK("OSD", NULL,
                         osdUpdateCheck,   // check 函数
                         osdUpdate,        // task 函数
                         TASK_PERIOD_HZ(OSD_FRAMERATE_DEFAULT_HZ),  // 默认 12 Hz
                         TASK_PRIORITY_LOW);
```

实际刷新间隔由参数 `osd_framerate_hz` 决定：`OSD_UPDATE_INTERVAL_US = 1000000 / osdConfig()->framerate_hz`。

### 3.2 `osdUpdateCheck`（每帧被调度器调用）

```
osdUpdateCheck(currentTimeUs)
 ├─ if (osdState == IDLE && 到达刷新时刻)
 │    osdState = CHECK; 并计算下一次刷新时间（避免追帧洪峰）
 ├─ if (CHECK 或 TRANSFER 状态 && displayIsTransferInProgress)
 │    返回 false            // DMA/传输未完成，先不跑任务
 └─ return (osdState != IDLE)   // 有活干才让调度器执行 osdUpdate
```

### 3.3 `osdUpdate` 状态机（一次帧刷新 = 一个状态环）

```
INIT ──displayCheckReady──▶ COMMIT
CHECK ──等待DMA──▶ UPDATE_HEARTBEAT ──displayHeartbeat──▶ PROCESS_STATS1
PROCESS_STATS1 ──(需要刷新统计?)──▶ REFRESH_STATS ──▶ PROCESS_STATS2
PROCESS_STATS2 ──▶ PROCESS_STATS3 ──(CMS占用则跳过)──▶ UPDATE_ALARMS
UPDATE_ALARMS ──▶ UPDATE_CANVAS ──▶ DRAW_ELEMENT ──(逐个元素)──▶ [DISPLAY_ELEMENT]
    ──▶ [REFRESH_PREARM(解锁前规格页)]──▶ COMMIT ──▶ TRANSFER ──▶ IDLE
```

各状态职责：

| 状态 | 做什么 |
| --- | --- |
| `INIT` | `displayCheckReady()`（MAX7456 可延迟探测/重扫）；`osdCompleteInitialization()` 画 logo、版本号、CMS 帮助、初始化图层、`osdAnalyzeActiveElements()` |
| `CHECK` | 若 `displayIsTransferInProgress()`（MAX7456 DMA 进行中）则原地等待 |
| `UPDATE_HEARTBEAT` | `displayHeartbeat()`：MAX7456 掉线重初始化；MSP 则推送 `MSP_DP_HEARTBEAT`（防止对端释放显示/提示断开） |
| `PROCESS_STATS1` | 检测解锁/上锁：解锁→`osdShowArmed()`（画 "ARMED" + 可选 logo，停留 logo_on_arming_duration）；上锁后 60s→开启统计页；累计飞行时间；触发统计刷新则进入 `REFRESH_STATS` |
| `REFRESH_STATS` | `osdRefreshStats()`：四阶段 `INITIAL_CLEAR_SCREEN → COUNT_STATS → CLEAR_SCREEN → RENDER_STATS`，一次调用只画一行统计（时间切片） |
| `PROCESS_STATS2/3` | 读取 ESC 综合遥测、计算 G 力极值 |
| `UPDATE_ALARMS` | `osdUpdateAlarms()`：按 RSSI/信号质量/电压/高度/温度等阈值设置各元素的 `blinkBits`（闪烁标志） |
| `UPDATE_CANVAS` | `BOXOSD` 拨杆隐藏 OSD；支持背景层的设备先 `displayLayerCopy`（前景=背景）再画动态部分；不支持背景层的清屏；GPS 传感器状态变化→重新 `osdAnalyzeActiveElements()`；`osdSyncBlink()` 同步闪烁相位 |
| `DRAW_ELEMENT` | `osdDrawNextActiveElement()` **每次只画一个元素**（时间切片的关键）；记录该元素耗时（指数衰减的最大值），用 `schedulerSetNextStateTime()` 让调度器尽快再来 |
| `DISPLAY_ELEMENT` | `osdDisplayActiveElement()` 把刚缓冲好的字符串真正写入 displayPort（背景/前景分两次） |
| `REFRESH_PREARM` | 未解锁且开了 `osd_show_spec_prearm` 时逐行画规格页（RPM/极数/混控/油门/电机/电池/版本） |
| `COMMIT` | `displayCommitTransaction()`（MAX7456 双缓冲切换提交；MSP 为 no-op） |
| `TRANSFER` | `displayDrawScreen()`：MAX7456 启动 DMA 只传输"有变化"的字符；MSP 推送 `MSP_DP_DRAW_SCREEN`；结束后回 `IDLE` |

**时间切片设计**：一帧画面被拆到多次 `osdUpdate` 调用中完成，配合 `schedulerSetNextStateTime()` + `schedulerIgnoreTaskExecTime()/schedulerIgnoreTaskExecRate()`，避免 OSD 低优先级任务长时间霸占 CPU（对 SPI 接收等实时任务友好）。

---

## 4. 元素渲染机制（osd_elements.c）

- **元素清单**：`osd_items_e`（`osd.h`，如 `OSD_MAIN_BATT_VOLTAGE`、`OSD_ARTIFICIAL_HORIZON`、`OSD_WARNINGS`、`OSD_SYS_*` 等）+ 绘制函数映射表 `osdElementDrawFunction[]`。
- **活跃元素分析**：`osdAnalyzeActiveElements()`（OSD 配置改变/解锁状态/GPS 传感器变化时调用）→ `osdAddActiveElements()` 依据"可见性掩码 + 硬件/功能是否可用"（如 GPS 元素需要 GPS）构建 `activeOsdElementArray[]`。
- **单元素绘制**：`osdDrawSingleElement()` 格式化数据到 `activeElement.buff`（缓冲区），并通过 `displayWrite / displayWriteChar / displaySys / displayExtended` 输出。一个元素可以被拆成：
  - **背景（静态部分）**：支持背景层的设备（MAX7456）一次性画到背景层，此后每帧只 `layerCopy` 再画动态部分，减少刷新量；
  - **前景（动态部分）**：每帧重画。
- **闪烁**：`osdSyncBlink()` 计算 `blinkState`，`osdUpdateAlarms()` 维护每个元素的 `blinkBits`；元素把 `DISPLAYPORT_BLINK` 合进 attr。
- **告警**：`osd_warnings.c` 提供 `OSD_WARNINGS` 元素（解锁警告、电池电压、失联、核心温度等）。
- **统计页**：上锁后显示 `--- STATS ---` 及 `osdStatsDisplayOrder[]` 定义的各项统计（最大速度、最低电压、飞行时间等）。

---

## 5. displayPort 抽象层

`src/main/drivers/display.h` 定义 `displayPort_t`（行/列/位置/属性 + vtable 指针）与 `displayPortVTable_t`：


---

## 6. OSD 的实现方式详解

### 6.1 模拟图传 + MAX7456（经典方案）

- **硬件**：板载 MAX7456（SPI 从机，最大 10 MHz），FC 与摄像头/VTX 组成视频环路（Camera → MAX7456 → VTX）。
- **原理**：MAX7456 内部有 480（PAL）/390（NTSC）个"字符槽"的显存和一个 NVM 字库。FC 通过 SPI 往显存写入**字符索引**（0x00~0xFF，含符号表 `osd_symbols.h` 里的 `SYM_*`），芯片在视频消隐期把字符网格"叠加"进模拟视频。字符级叠加由芯片硬件完成，FC 几乎零开销。
- **代码路径**：`displayport_max7456.c` → `max7456Write(x,y,text)` → 写影子缓冲 → `max7456DrawScreen()` DMA 只刷变化字符。
- **特性**：PAL/NTSC 自动识别（STAT 寄存器）、VM0/VM1 控制亮度/反转/背景灰、双图层（前景/背景）双缓冲、NVM 字库（`MSP_OSD_CHAR_WRITE` → `max7456WriteNvm` 从配置器上传字库）。
- **结论：需要 MAX7456 芯片**（或其他等效 OSD 叠加芯片）。

### 6.2 模拟图传 + FB_OSD（framebuffer OSD）

- `displayport_fb_osd.c` + `drivers/fb_osd_impl.h`：FC 内置视频输出能力（当前为 RP2350 / PICO 平台实现，见提交 `73d0224ee`）。
- 不再靠外部芯片叠加字符，而是 FC 自己维护像素级 framebuffer，把字符按字体**像素化**渲染后交给视频编码输出。支持图层、背景缓存、`drawOsdItem` 缓存（人工地平线等可直接按元素缓存）。
- 同样用于模拟/自输出视频，**不依赖 MAX7456**。

### 6.3 数字图传 + MSP DisplayPort（DJI / HDZero / Walksnail）

- FC 的 UART（`MSP_DISPLAYPORT_UART` 或勾选 `VTX MSP` 的串口）接数字图传/VTX。`video_system = HD` 时初始化强制选 `OSD_DISPLAYPORT_DEVICE_MSP`。
- 画布 53x20（默认），可被眼镜通过 `MSP_SET_OSD_CANVAS`（MSP_OSD_CANVAS=189）动态告知实际分辨率（代码中收到后自动切 HD、写 EEPROM 并复位生效）。
- FC 把 OSD 文本/坐标推给眼镜，**眼镜端渲染**。无需 MAX7456。`displaySys()`（`MSP_DP_SYS`）还支持把"眼镜电压/比特率/延迟/距离/信号强度/DVR"等系统元素交给眼镜自己绘制。

### 6.4 MSP OSD 原理（重点）

**角色**：FC = 显示主控（MSP 客户端，主动推送）；对端 = 显示执行器（眼镜/VTX/独立 OSD 芯片，负责实际渲染）。

**报文格式**：`mspSerialPush(displayPortSerial, MSP_DISPLAYPORT /*182*/, subcmd..., len, MSP_DIRECTION_REPLY, MSP_V1)`，即一条条 **MSP v1 报文**，命令号 `MSP_DISPLAYPORT = 182`，载荷第一个字节为子命令（`io/displayport_msp.h`）：

| 子命令 | 值 | 含义 |
| --- | --- | --- |
| `MSP_DP_HEARTBEAT` | 0 | 心跳（防止对端释放显示/显示"已断开"） |
| `MSP_DP_RELEASE` | 1 | 释放显示 |
| `MSP_DP_CLEAR_SCREEN` | 2 | 清屏 |
| `MSP_DP_WRITE_STRING` | 3 | 在 (col,row) 写字符串（载荷：row, col, attr[字体bank|闪烁], text≤30） |
| `MSP_DP_DRAW_SCREEN` | 4 | 触发一屏刷新 |
| `MSP_DP_OPTIONS` | 5 | 预留（ArduPilot/INAV） |
| `MSP_DP_SYS` | 6 | 绘制系统元素（眼镜电压等） |
| `MSP_DP_FONTCHAR_WRITE` | 7 | 通过 MSP 上传字库字符（`USE_MSP_DISPLAYPORT_FONT`，写完后 delay(80ms)） |

**每个 OSD 显示原语**（清屏、写字符串、写字符、画屏、心跳）→ `displayport_msp.c` vtable 的对应函数 → 打包成一条 `MSP_DISPLAYPORT` 报文 → `mspSerialPush` 编码写入串口 TX。**FC 不渲染像素，对端渲染**。

**对端如何"发现"FC？** FC 每帧推心跳/清屏/绘制；配置器通过 `MSP_OSD_CONFIG`(84) 上报 `OSD_FLAGS_OSD_MSP_DEVICE`（+`OSD_FLAGS_OSD_HARDWARE_AIRBOT_THEIA_OSD` 当开启字体上传）来表示当前是 MSP 设备。

### 6.5 其它屏幕（非视频叠加）

`FRSKYOSD`（FrSky 遥控器屏幕）、`OLED`（外接 OLED 仪表）、`CRSF/HOTT/SRXL`（遥控器/外设屏协议）。它们只是"另一类 displayPort"，不走视频信号。

```
grab / release / clearScreen / drawScreen / screenSize
writeString / writeChar / writeSys / writeFontCharacter
isTransferInProgress / heartbeat / redraw / isSynced / txBytesFree
layerSupported / layerSelect / layerCopy
checkReady / beginTransaction / commitTransaction / setBackgroundType ...
```

`display.c` 提供 `displayWrite()` 等包装函数。**OSD 逻辑只调用这些包装函数，完全不感知底层是 SPI 芯片还是串口**——这就是"一套 OSD 代码适配所有屏幕"的关键。

设备类型枚举（`displayPortDeviceType_e`）：`MAX7456 / OLED / MSP / FRSKYOSD / CRSF / HOTT / SRXL / FBOSD`。

`displaySupportsOsdSymbols()` 判定 MAX7456 / MSP / FRSKYOSD 支持 OSD 符号表（字库图形）。


---

## 7. 关键问答

### Q1：模拟图传的 OSD 有哪几种实现？
1. **板载 MAX7456 芯片**（SPI 字符叠加）——最主流，FC 只写字符索引。
2. **FB_OSD**（FC 自输出视频、像素级 framebuffer 渲染）。
3. **外部 MSP OSD 设备**（UART 连支持 MSP DisplayPort 的模拟 OSD 芯片/图传，如 Airbot Theia 类）。

### Q2：数字图传的 OSD 怎么实现？需要 MAX7456 吗？
不需要。数字图传（DJI O3、HDZero、Walksnail）在眼镜/VTX 端有独立渲染引擎，FC 只通过 UART 发送 MSP DisplayPort 文本指令，**渲染完全在眼镜端**。所以数字图传的 OSD 甚至能做得比 MAX7456 更精细（更大的画布、系统元素等）。

### Q3：MSP 是怎么实现 OSD 的？
见 6.4：FC 把"清屏/写字符串/画屏/心跳/字体"等原语封装成 `MSP_DISPLAYPORT`(182) 报文，主动推送到配置为 `FUNCTION_VTX_MSP|FUNCTION_MSP` 的串口；对端设备接收并渲染。FC 是"发指令"方，不是"画图"方。

### Q4：MSP OSD 可以用于模拟图传吗？
**可以**。MSP DisplayPort 只描述"在 (x,y) 写哪个字符/字符串"，跟视频是 PAL/NTSC 还是 HD 无关。只要对端硬件把收到的字符渲染成视频叠加层并混入模拟视频即可。代码层面 `displayPortMspInit()` 在非 HD（`video_system != HD`）时也会按 PAL/NTSC 计算行数（16/13 行 + row_adjust），说明 **MSP DisplayPort 原生支持 SD 画布**。实际产品如 Airbot Theia OSD（`OSD_FLAGS_OSD_HARDWARE_AIRBOT_THEIA_OSD`、`USE_MSP_DISPLAYPORT_FONT`）就是走 MSP 的模拟 OSD 方案。

### Q5：没有 MAX7456，通过 UART 能给模拟图传发 MSP OSD 吗？
**能，但前提是 UART 对端必须是"支持 MSP DisplayPort 的 OSD 器件/图传"**：

```
FC 的 UART(TX)  ──MSP_DISPLAYPORT 报文──▶  支持 MSP OSD 的模拟图传/OSD芯片
                                                     │ 自行叠加字符
                                                     ▼
                                    Camera ─▶ [OSD芯片] ─▶ VTX/模拟视频
```

- 配置方法：把该串口功能设为 `VTX MSP` + `MSP`（或目标固件直接定义 `MSP_DISPLAYPORT_UART`），CLI 设 `display_port_device = MSP`，视频制式设为 PAL/NTSC。
- **注意**：普通模拟图传（只做射频发射、没有 OSD 处理器）**不行**——它没有能力接收 MSP 报文并叠加视频。此时唯一的"无 MAX7456 模拟 OSD"路径就是 FB_OSD（FC 自己渲染并输出视频）。

---

## 8. 相关关键配置项

| CLI 参数 | 说明 |
| --- | --- |
| `display_port_device` | `NONE/AUTO/MAX7456/MSP/FRSKYOSD/FBOSD`（AUTO 依次探测） |
| `video_system` | `AUTO/PAL/NTSC/HD`（HD 强制走 MSP DisplayPort） |
| `osd_framerate_hz` | OSD 刷新率（默认 12 Hz，范围 1–60） |
| `osd_canvas_width/height` | HD 画布（默认 53x20） |
| `displayport_msp_col_adjust/row_adjust` | MSP 显示偏移校正 |
| `displayport_msp_fonts` | MSP 显示的 4 个字库 bank 选择（对应 severity 级别） |
| `displayport_msp_use_device_blink` | 是否用设备本地闪烁 |
| 目标定义 `MSP_DISPLAYPORT_UART` | 指定 MSP DisplayPort 专用串口 |

---

## 9. 调用路径速查（一次完整 OSD 帧）

```
调度器 TaskLoop
 └─ TASK_OSD（12Hz，LOW）
     ├─ osdUpdateCheck()          → 到点/空闲判断，DMA 忙则让位
     └─ osdUpdate()               → 状态机（跨多次调用切完一帧）
         ├─ UPDATE_HEARTBEAT      → displayHeartbeat()   [MSP: 推 HEARTBEAT]
         ├─ PROCESS_STATS1..3     → arm/disarm、飞行时间、G力、ESC 遥测
         ├─ UPDATE_ALARMS         → osdUpdateAlarms() → blinkBits
         ├─ UPDATE_CANVAS         → layerCopy / clearScreen / osdSyncBlink
         ├─ DRAW_ELEMENT ×N       → osdDrawNextActiveElement()
         │                          └─ osdDrawSingleElement()
         │                             └─ osdElementDrawFunction[item]()
         │                                └─ displayWrite/WriteChar/Sys
         │                                   ├─ MAX7456: max7456Write() 写影子缓冲
         │                                   └─ MSP: mspSerialPush(MSP_DISPLAYPORT, WRITE_STRING...)
         ├─ DISPLAY_ELEMENT       → osdDisplayActiveElement() 落盘字符串
         ├─ COMMIT                → displayCommitTransaction()
         └─ TRANSFER              → displayDrawScreen()
                                    ├─ MAX7456: DMA 刷变化字符 → 视频叠加
                                    └─ MSP: 推 DRAW_SCREEN → 对端渲染
```

---

## 10. 参考依据（本报告涉及的关键源码位置）

- `src/main/fc/tasks.c:433` TASK_OSD 注册
- `src/main/osd/osd.c:534` osdInit；`osd.c:1365` osdUpdateCheck；`osd.c:1397` osdUpdate；`osd.c:1339-1358` 状态机枚举
- `src/main/fc/init.c:933-1008` OSD 初始化与设备选择（AUTO fallback 顺序）
- `src/main/osd/osd_elements.c:2356` osdDrawNextActiveElement；`osd_elements.c:2482` 背景层
- `src/main/drivers/display.h` vtable；`drivers/display.c:101` displayWrite
- `src/main/io/displayport_msp.c` MSP DisplayPort 全实现；`io/displayport_msp.h` 子命令
- `src/main/io/displayport_max7456.c`；`drivers/max7456.c` 芯片驱动
- `src/main/io/displayport_fb_osd.c`；`drivers/fb_osd_impl.h`
- `src/main/msp/msp_serial.c:646` mspSerialPush；`src/main/config/config.c:566-578` 串口定位
- `src/main/msp/msp.c:964-1045` MSP_OSD_CONFIG（MSP 设备标志）；`msp.c:4694-4710` MSP_SET_OSD_CANVAS（HD VTX 自报分辨率）
- `src/main/msp/msp_protocol.h:226` `#define MSP_DISPLAYPORT 182`
- `src/config/configs/AIRB/AIRBOTSUPERF4V2/config.h` `USE_MSP_DISPLAYPORT_FONT` + `MSP_DISPLAYPORT_UART`


---

## 11. 专题分析：MSP OSD 在地面站不开启对应 UART 时也能识别并输出

> 现象：某 BF 飞控产品（源码疑似被改动过）使用 MSP OSD 时，即使在地面站/配置器的"端口"页没有勾选该 UART 的 VTX MSP/MSP 功能，OSD 仍能被识别并输出。

### 11.1 原版 Betaflight 的正常要求

MSP DisplayPort 必须有一条"挂载"串口，链路如下：

```
config.c validateAndFixConfig()
  └─ 扫描所有端口: functionMask 同时含 FUNCTION_VTX_MSP|FUNCTION_MSP 的第一个端口
       └─ displayPortMspSetSerial(该端口)          // 保存到 static displayPortSerial
io/displayport_msp.c
  └─ 每次显示原语 → mspSerialPush(displayPortSerial, MSP_DISPLAYPORT(182), ...)
msp_serial.c mspSerialPush()
  └─ 只推送到 identifier == displayPortSerial 的端口；SERIAL_PORT_NONE 时不推送
```

**结论**：原版固件中，"输出"到眼镜/VTX 的 MSP OSD 帧**必须**依赖该端口被勾选 `VTX MSP` + `MSP`（EEPROM 里存了 functionMask）。

### 11.2 "识别"和"输出"是两回事

- **识别（MSP_OSD_CONFIG 上报为 MSP 设备）**：`init.c` 在 `video_system == HD` 或 `display_port_device = MSP`/AUTO 且无 MAX7456 时就把 `osdDisplayPortDevice` 设为 MSP。这个判断**不依赖任何 UART 配置**，USB 上就能查询到——所以"不开启 UART 也能识别"对原版固件也是成立的。
- **输出（实际推送 OSD 帧）**：才需要 `displayPortSerial` 有效。

### 11.3 为什么"不开启 UART 也能输出"？几种可能

| 情形 | 机制 | 是否需改源码 |
| --- | --- | --- |
| A. 出厂默认配置 | 目标固件定义了 `MSP_DISPLAYPORT_UART`，`pgResetFn_serialConfig`（serial.c:361）在**配置复位**时把该端口 functionMask 强制为 `FUNCTION_VTX_MSP|FUNCTION_MSP`。若用户从未在配置器里保存过端口配置（或EEPROM无效/被重置），该端口本来就"开着的" | **不需要**改源码 |
| B. 启动时强制 | 固件在 `serialInit()` 或 `validateAndFixConfig()` 每次开机**无条件**重写该端口的 functionMask（不再等 EEPROM） | 需要小改（如把 serial.c:361 移到开机路径） |
| C. 推送不按端口过滤 | `mspSerialPush` 被改成 `SERIAL_PORT_ALL`，或 `displayPortMspSetSerial` 被硬编码到某个 UART | 需要改 msp_serial.c / displayport_msp.c / config.c |
| D. 目标本身无此端口 | 完全靠 `MSP_DISPLAYPORT_UART` 直连（如 AIRBOTSUPERF4V2 的 USART8） | 目标配置里定义即可 |

**注意**：原版机制下，只要用户在配置器里"保存"过一次端口设置（把 VTX MSP 勾掉），EEPROM 覆盖默认值后，**情形 A 失效**。若此时 OSD 仍然输出，则基本可以断定是 **B 或 C**——即该产品固件在开机/推送环节做了强制处理。

### 11.4 如何验证 / 定位改动

1. **实验法**：在配置器端口页把可能相关的 UART 的 `VTX MSP` 全部取消、保存并重启 → 若 OSD 仍正常输出，说明固件在开机时强制覆盖了 functionMask（情形 B/C）。
2. **观察法**：连接配置器查看 OSD 页显示设备类型。若显示 "MSP" 且 `displayport_msp_*` 参数可调，说明 `displayPortDevice` 已是 MSP。
3. **对照 diff 法**：把产品固件源码与官方 BF 对照这几个文件：
   - `src/main/io/serial.c`（`#ifdef MSP_DISPLAYPORT_UART` 强制段是否被挪到开机路径）
   - `src/main/config/config.c`（`validateAndFixConfig` 是否改为"强制写 functionMask"）
   - `src/main/msp/msp_serial.c`（`mspSerialPush` 是否放宽端口过滤）
   - `src/main/io/displayport_msp.c`（`displayPortMspSetSerial`/`displayPortMspInit` 是否硬编码端口）
   - 目标配置（是否定义 `MSP_DISPLAYPORT_UART`）
4. **本仓库结论**：本 fork（含 RP_F405_BT 相关提交）在 serial.c / config.c / msp_serial.c / displayport_msp.c 上**与上游一致**，没有任何强制覆盖端口的改动。若你的产品基于本仓库而出现上述现象，说明产品固件相对本仓库另有修改（或该目标本身定义了 `MSP_DISPLAYPORT_UART`）。

---

## 12. MAX7456 主板硬件接线

### 12.1 芯片在板上的三组信号

| 组 | 信号 | 说明 |
| --- | --- | --- |
| SPI 控制 | SCK / MOSI(SDIN) / MISO(SDOUT) / CS | 接到 MCU 的某个 SPI 总线，FC 通过 SPI 写"字符索引"、读状态、刷字库 |
| 视频环 | VIN（摄像头视频进）、VOUT（视频出→VTX） | **关键**：模拟摄像头必须先进 MAX7456，MAX7456 把字符叠加后再输出给 VTX |
| 电源/复位 | VCC(3.3V) / GND / RESET#（可省） | 3.3V 供电；有些板把 RESET 与 MCU 的 IO 相连做硬复位 |

**视频链路**：

```
模拟摄像头 ──CVBS──▶ [MAX7456 VIN] ──(字符叠加)── [VOUT] ──CVBS──▶ 模拟图传VTX ──射频──▶ 眼镜
```

这就是"摄像头视频线进飞控板、VTX 视频线与摄像头不直连"的标准原因——**FC 板故意把视频"环"进 OSD 芯片再环出去**。

### 12.2 目标固件里的引脚配置（示例）

```c
// src/config/configs/FPVM/BETAFLIGHTF4/config.h
#define USE_MAX7456
#define MAX7456_SPI_CS_PIN   PB12     // 片选
#define MAX7456_SPI_INSTANCE SPI2     // 挂在 SPI2 上
// SPI2_SCK=PB13  SPI2_SDI(MISO)=PB14  SPI2_SDO(MOSI)=PB15

// src/config/configs/FPVM/XRACERF4/config.h
#define MAX7456_SPI_CS_PIN   PA15
#define MAX7456_SPI_INSTANCE SPI3
```

运行时参数（`src/main/pg/max7456.c`）：`max7456Config`（csTag、spiDevice、clockConfig 半/额定/双倍速、preInitOPU）。

### 12.3 常见问题

- **必须走 SPI，不能走 UART**：MAX7456 是 SPI 从机，不支持 MSP。如果 FC 没有引出 MAX7456 的 SPI 引脚（飞线到 MCU 的 SPI），就无法外接 MAX7456；外接"MINIMOSD"之类模块走的是串口 MWOSD 协议（另一套方案，非本驱动）。
- **视频同步**：VIN 需给有效的 PAL/NTSC CVBS 信号；芯片用 STAT 寄存器检测制式（`max7456ReInit` 读取 VIN 状态），无信号时会退化为默认 PAL。
- **供电与纹波**：3.3V 供电需干净；视频走线要短、远离开关电源（避免画面抖动）。
- **为何"看着没有芯片"**：老式 TQFP-48 封装很大；但 MAX7456 也有 TSSOP-28 等小封装，且很多 AIO 板把 OSD 芯片藏在屏蔽罩/板背面。若板上视频环（Camera→FC→VTX）成立却没有 MAX7456，参见下一节（MSP OSD 芯片 / FB_OSD 等替代方案）。

---

## 13. 专题分析：板上没有 MAX7456，但摄像头视频线接入飞控板、VTX 未与摄像头直连

> 现象：飞控板上看不出大封装的 MAX7456，但摄像头视频线确实进了飞控板；实测 VTX 的视频输入线没有与摄像头直接相连。

### 13.1 结论：视频环 ≠ 一定有 MAX7456，OSD 可以由"小型 MSP OSD 芯片"完成

**摄像头 → 飞控板 → VTX 的"视频环"结构，本质上只说明板上存在一个"视频叠加/处理节点"**，并不限定是 MAX7456。既然板上没有大封装 MAX7456，最可能的替代实现是：

```
模拟摄像头 ──CVBS──▶ [小型 MSP OSD 芯片 (VIN)] ──(MCU字符叠加)── [VOUT] ──CVBS──▶ 模拟图传VTX
                          ▲
                          │ 串口(MSP_DISPLAYPORT 报文)
                          │
                    FC 主控 STM32 (UART 引脚直连芯片)
```

**这正是 MSP DisplayPort 的典型模拟应用**：
- 芯片体积远小于 TQFP-48 的 MAX7456（常为 SSOP/QFN/SOP 小封装，容易被当作"稳压/电源"器件忽略）；
- 它内部有"视频同步分离 + 字符点阵 + 视频混合"电路，接收 FC 通过 UART 发来的 `MSP_DISPLAYPORT` 文本指令，自行把字符叠加进模拟视频；
- 代码层面的对应物：`USE_MSP_DISPLAYPORT_FONT`（MSP 上传字库，提交 #14391）、`OSD_FLAGS_OSD_HARDWARE_AIRBOT_THEIA_OSD`（MSP OSD 器件标志）。这类产品即"Airbot Theia OSD"及同类 MSP 芯片。

### 13.2 与前面现象的闭环

- 为什么"地面站不开启 UART 也能识别并输出 OSD"？因为该 MSP OSD 芯片在**硬件上已固定接到某个 UART**（目标固件 `MSP_DISPLAYPORT_UART` 或固件强制），配置器端口页是否勾选不影响这条物理链路；而"识别为 MSP"本来就不依赖端口配置（见第 11 节）。
- 为什么"没有 MAX7456"？因为它根本不基于 MAX7456——字符渲染在 MSP OSD 芯片内部完成，不需要大封装 SPI 芯片。

### 13.3 如何进一步确认（按优先级）

1. **用眼睛找小芯片**：沿摄像头 VIN 焊盘和 VTX VOUT 焊盘之间找小封装 IC（SSOP/SOP/QFN，常见 8~28 脚），并确认其附近有 2~4 根走线连到 MCU（UART TX/RX + 电源）。
2. **看地面站 OSD 页设备类型**：若显示 "MSP" 且能正常预览/输出，基本坐实是 MSP OSD 芯片。
3. **CLI 验证**：`get osd_framerate_hz` / `get display_port_device`，或 `status` 看 `OSD: MSP`。
4. **示波器量 UART**：OSD 工作时可看到该 UART TX 上有 `$M.>` 头的 MSP 帧（MSP_DISPLAYPORT 182）。
5. **对照目标配置**：查产品固件目标是否定义 `MSP_DISPLAYPORT_UART` / `USE_MSP_DISPLAYPORT_FONT`（对应仓库 `src/config/configs/AIRB/AIRBOTSUPERF4V2/config.h` 的写法）。
6. **少数情况（可排除）**：MAX7456 的 QFN/TSSOP 小封装版藏在屏蔽罩或板背面——若实测 VTX 视频与摄像头不直连且 OSD 存在，可用金属屏蔽罩下方/板背是否有 30 列字符网格 OSD 来判断；也可看 `status` 里 OSD 设备是 MAX7456 还是 MSP。

### 13.4 对本仓库的说明

本 fork 的 `FB_OSD`（framebuffer OSD，RP2350/PICO 目标）是另一种"无 MAX7456 也能输出视频+OSD"的实现，但它在 FC 内部做像素级渲染，通常用于自输出视频的板子；若你的板是常见 STM32F4 AIO（摄像头+模拟VTX），最合理的就是**第 13.1 节的 MSP OSD 芯片方案**。

