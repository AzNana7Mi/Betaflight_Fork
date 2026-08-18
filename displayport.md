# MSP DisplayPort 协议实现分析报告

> 分析基于: `Betaflight_Fork` 飞控代码库（本报告只做代码分析，未修改任何代码）
> 报告日期：2026-08-15
> 适用对象：想要实现"接收飞控 OSD 并叠加到视频"的上位机/显示设备开发者

---

## 0. 结论摘要

1. **MSP DisplayPort 是一套"字符网格（Grid）绘制"协议**：FC 作为显示主控（master），通过 UART 把"在某行某列写一串字符"的指令用标准 **MSP v1 报文**推送给显示设备（眼镜/图传/上位机/OSD 芯片）；**FC 不发送任何像素数据**，渲染完全由对端完成。
2. **协议核心 = 1 条 MSP 命令 + 8 个子命令**：命令号 `MSP_DISPLAYPORT = 182`，子命令有 HEARTBEAT(0) / RELEASE(1) / CLEAR_SCREEN(2) / WRITE_STRING(3) / DRAW_SCREEN(4) / OPTIONS(5,预留) / SYS(6) / FONTCHAR_WRITE(7)。
3. **每帧 OSD 的推送节奏固定**：`HEARTBEAT → (CLEAR_SCREEN) → N×WRITE_STRING → DRAW_SCREEN`，随调度器 `TASK_OSD` 以 `osd_framerate_hz`（默认 12 Hz）重复。
4. **对端（上位机）必须实现的指令**：解析 MSP v1 帧（cmd=182）＋子命令 **HEARTBEAT / CLEAR_SCREEN / WRITE_STRING / DRAW_SCREEN** 四个即可完成基础 OSD；若要支持闪烁需处理 attr 的 BLINK 位，要支持自定义字库需实现 **FONTCHAR_WRITE(7)**，要告诉 FC 你的画布尺寸需主动发送 **MSP_SET_OSD_CANVAS(188)**。
5. **画布尺寸**：SD 视频默认 30 列 × 16 行（PAL）/ 13 行（NTSC）；HD 默认 53×20，且可由显示设备通过 `MSP_SET_OSD_CANVAS` 上报实际行列数（FC 收到后自动切 HD 并重启）。

---

## 1. 协议角色与数据流

```
                    MSP v1 帧 (cmd=182)
 FC 主控 STM32 ────────────────────────────▶  显示设备(上位机/眼镜/VTX/OSD芯片)
 (displayport_msp.c 打包)                        (解析 + 渲染字符网格到视频)
        ▲                                              │
        └──────── MSP_SET_OSD_CANVAS(188) ◀────────────┘
        设备自报画布尺寸(cols, rows)，触发 FC 切 HD
```

- **FC = master**：主动推送。OSD 任务（`TASK_OSD`）每帧调用 `displayWrite/displayClearScreen/displayDrawScreen` 等 vtable 接口，`displayport_msp.c` 把它们一条条封装成 `MSP_DISPLAYPORT` 报文，经 `mspSerialPush()` 发到"挂载"串口。
- **显示设备 = renderer**：接收、维护字符网格、把字符渲染进自己处理的视频流。不做任何 FC 交互也能工作（单向流），只有"上报画布/改配置"时才需要反向发指令。
- **链路**：FC 侧串口需带 `FUNCTION_VTX_MSP | FUNCTION_MSP`（目标 `MSP_DISPLAYPORT_UART` 或配置器勾选），FC 把该串口记到 `displayPortSerial`，只往这个口推。

---

## 2. MSP v1 帧封装（FC 推送方向）

FC 用 `mspSerialPush(port, cmd, payload, len, MSP_DIRECTION_REPLY, MSP_V1)` 编码（`msp_serial.c mspSerialEncode`）：

```
┌──────┬──────┬──────┬──────┬──────┬────────────────────────┬──────────┐
│ '$'  │ 'M'  │ '>'  │ size │ cmd  │ payload                │ checksum │
│ 0x24 │ 0x4D │ 0x3E │ 1B   │ 0xB6 │ 子命令 + 参数          │ 1B       │
└──────┴──────┴──────┴──────┴──────┴────────────────────────┴──────────┘
```

- 魔数 `$M` + 方向字节：FC→设备固定 `>`（0x3E；若 FC 内部出错会变 `!`，正常不会）；
- `size` = payload 字节数（WRITE_STRING 最多 30+4=34，不会触发 JUMBO 头）；
- `cmd` = `MSP_DISPLAYPORT` = 182 = 0xB6；
- `checksum` = `size XOR cmd XOR payload[0] XOR payload[1] ...`（MSP v1 校验）；
- 这是**标准 MSP v1**，任何 MSP 库（或手写解析器）都能解析。

---

## 3. 命令与子命令总表

子命令定义在 `src/main/io/displayport_msp.h`：

| 子命令 | 值 | 名称 | 载荷格式 |
| --- | --- | --- | --- |
| `MSP_DP_HEARTBEAT` | 0 | 心跳 | `[0]` |
| `MSP_DP_RELEASE` | 1 | 释放显示 | `[1]` |
| `MSP_DP_CLEAR_SCREEN` | 2 | 清屏 | `[2]` |
| `MSP_DP_WRITE_STRING` | 3 | 写字符串 | `[3][row][col][attr][text...]`（text≤30） |
| `MSP_DP_DRAW_SCREEN` | 4 | 触发整屏绘制 | `[4]` |
| `MSP_DP_OPTIONS` | 5 | 预留（ArduPilot/INAV） | 不使用 |
| `MSP_DP_SYS` | 6 | 系统元素 | `[6][row][col][systemElement]` |
| `MSP_DP_FONTCHAR_WRITE` | 7 | 写单个字库字符 | `[7][addr_lo][addr_hi][attr=0][64B]`（`USE_MSP_DISPLAYPORT_FONT` 时启用） |

注意：WRITE_STRING / SYS 的载荷顺序是 **先 row 后 col**（与常见 col,row 相反）。

---

## 4. 关键子命令详解

### 4.1 WRITE_STRING(3) —— 最核心的指令

```
[3][row][col][attr][文本字节...]
```

- `row`/`col`：字符网格坐标（左上角为 0,0），范围受画布约束（SD: 30×16/13，HD: 53×20 等）；
- `attr` 编码（bit7 必须为 0，bit5-2 未用）：

| 位 | 掩码 | 含义 |
| --- | --- | --- |
| bit7 | `DISPLAYPORT_MSP_ATTR_VERSION` | 格式指示，V1/V2 必须为 0 |
| bit6 | `DISPLAYPORT_MSP_ATTR_BLINK` | 设备本地闪烁 |
| bit1-0 | `DISPLAYPORT_MSP_ATTR_FONT` | 字库 bank（0~3，256 字符/库） |

- **attr 的生成**：FC 把"严重级别"（`displayPortSeverity_e`：NORMAL=0/INFO=1/WARNING=2/CRITICAL=3）映射到字库 bank，映射表由 CLI 参数 `displayport_msp_fonts`（`fontSelection[severity]`）配置，**默认 bank = severity**（`displayport_profiles.c`）；
- 若元素在闪烁，FC 会置上 `DISPLAYPORT_BLINK`，从而在 attr 里加 bit6（仅当 `displayport_msp_use_device_blink=ON` 时，否则 FC 自己用空格交替实现闪烁）；
- 字符串最多 30 字节（`MSP_OSD_MAX_STRING_LENGTH`），超长被截断；`writeChar` 就是写 1 字符的 WRITE_STRING。

### 4.2 HEARTBEAT(0)

- FC 每帧 OSD 开始都会推一条；两个作用：防止"MW OSD"软件释放显示、防止从设备显示"已断开"。
- **上位机应把"超过 N 秒没有 HEARTBEAT"当作连接断开**（建议 1~2 秒超时），可清屏或显示"disconnected"。

### 4.3 CLEAR_SCREEN(2) / DRAW_SCREEN(4)

- `CLEAR_SCREEN`：把整张字符网格清为空白（0x20）；
- `DRAW_SCREEN`：表示"这一帧的指令到此为止，请提交渲染"。显示设备应在收到它时把已更新的网格合成到当前视频帧（对逐行视频可放到场消隐期）。
- 典型帧：`HEARTBEAT → CLEAR_SCREEN → WRITE_STRING×N → DRAW_SCREEN`，12 Hz 重复。

### 4.4 SYS(6) —— 系统元素（可选）

```
[6][row][col][systemElement]
```

元素枚举（`display.h DISPLAYPORT_SYS_*`）：GOGGLE_VOLTAGE=0、VTX_VOLTAGE=1、BITRATE=2、DELAY=3、DISTANCE=4、LQ=5、GOGGLE_DVR=6、VTX_DVR=7、WARNINGS=8、VTX_TEMP=9、FAN_SPEED=10。**含义是"这个位置交给设备自己画某个系统状态"**（如比特率/延迟只能由眼镜自己测）。上位机若无法提供对应数据，忽略即可。

### 4.5 FONTCHAR_WRITE(7) —— 字库上传（可选）

```
[7][addr_lo][addr_hi][0][64字节字符点阵]
```

- `addr` = 16 位字符地址（低字节在前；当前实现只用到低 8 位）；
- 点阵格式：`OSD_CHAR_BYTES=64` 字节/字符，其中前 `12列×18行×2bit/像素 ÷ 8 = 54` 字节为有效像素（2bit/像素，4 级灰度），后 10 字节预留；
- FC 每写一个字符后固定延时 80ms（`displayport_msp.c`），所以字库上传很慢（256 字符 ≈ 20s+）；
- 只有编译了 `USE_MSP_DISPLAYPORT_FONT`（如 AIRBOTSUPERF4V2 目标）且配置器上传字库时才会出现。

---

## 5. 一帧 OSD 的典型字节流（抓包视角）

以 SD/PAL 下显示电池电压 `12.6V` 于 (0,0) 为例（实际为多个元素）：

```
$M> 03 B6 00 6E            <- HEARTBEAT(0): size=3, cmd=182, payload=[0], cs
$M> 03 B6 02 6D            <- CLEAR_SCREEN(2)
$M> 10 B6 03 00 00 00 31 32 2E 36 56 0A   <- WRITE_STRING(3): row=0,col=0,attr=0,text="12.6V"
$M> 03 B6 04 6D            <- DRAW_SCREEN(4)
```

（校验和按 `size XOR cmd XOR payload...` 计算，示例字节仅供结构示意。）


---

## 6. 反向通道：上位机 → FC 的指令

| 命令号 | 名称 | 方向/载荷 | 用途 |
| --- | --- | --- | --- |
| 188 | `MSP_SET_OSD_CANVAS` | 设备→FC：`[cols][rows]` | **上报你的画布尺寸**。FC 收到后：设 `canvas_cols/rows`、强制 `video_system=HD`、`display_port_device=MSP`，写 EEPROM 并重启 |
| 84 | `MSP_OSD_CONFIG` | 设备→FC 查询 | 读 OSD 配置/设备类型（响应含 `osdFlags`，bit6=`OSD_FLAGS_OSD_MSP_DEVICE`，bit7=Theia；后续还有 video_system、units、alarms、各元素 item_pos、统计/告警使能、canvas 等） |
| 189 | `MSP_OSD_CANVAS` | 设备→FC 查询 | 读当前画布 `[cols][rows]` |
| 87 | `MSP_OSD_CHAR_WRITE` | 配置器→FC | 配置器上传字库给 FC，FC 再转发为 FONTCHAR_WRITE(7) 推给显示设备 |
| 86 | `MSP_OSD_CHAR_READ` | 配置器→FC | 读回字库字符 |

---

## 7. 上位机实现清单（直接回答：该实现哪些指令？）

### 7.1 必选（能显示 OSD 的最小集）

1. **MSP v1 帧解析器**：同步 `$M`，方向 `>`（FC→设备），读 size/cmd，cmd==182 时解析 payload，做 XOR 校验。建议同时兼容 `$M!`（出错）。
2. **子命令处理**：
   - `HEARTBEAT(0)` → 刷新保活时间戳；
   - `CLEAR_SCREEN(2)` → 字符网格全部清为空白（0x20）；
   - `WRITE_STRING(3)` → 把文本按 attr 写入 `grid[row][col]`（处理越界截断）；
   - `DRAW_SCREEN(4)` → 把整张网格（或脏矩形）合成到视频帧并输出。
3. **字符网格模型**：分配 `rows×cols` 的"字符码+attr"缓冲；内置至少一套 12×18 字体（可先用与 BF 默认字库一致的位图；不要求 100% 符号一致，但空格/数字/字母必须正确）。尺寸取 `cols=30`、`rows` 按视频制式 PAL=16/NTSC=13；若走 HD 模式则等 FC 推送后用收到的坐标系，并在连接时用 MSP_SET_OSD_CANVAS 上报真实尺寸。
4. **帧节奏**：每收到 DRAW_SCREEN 渲染一次即可，FC 已按 framerate_hz 节流。

### 7.2 建议（体验更完整）

5. **HEARTBEAT 超时**：1~2s 无心跳 → 清屏/显示"disconnected"，恢复后自动重新显示；
6. **attr 支持**：bit6（BLINK）→ 本地按 ~1Hz 交替显示该字符/空格；bit0-1（FONT bank）→ 多套 256 字符字库切换；
7. **反向上报**：连接时发送 `MSP_SET_OSD_CANVAS(188)` `[cols][rows]`（若你希望 FC 使用 HD 画布；若你的视频是 SD 则跳过，保持 PAL/NTSC）；
8. 对同一条串口上其它 MSP 命令（该口同时带 `FUNCTION_MSP`，FC 也会响应查询）按 cmd 号忽略，只处理 182。

### 7.3 可选（按需）

9. `SYS(6)`：能提供对应数据（如你测到的链路比特率/延迟）就在指定坐标自绘；不能提供就忽略；
10. `FONTCHAR_WRITE(7)`：支持接收 FC 转发的字库数据（需要 `USE_MSP_DISPLAYPORT_FONT`），实现自定义字体/符号；
11. `RELEASE(1)`：收到后清理显示资源/回到待机画面；
12. 参与 OSD 配置：查询 `MSP_OSD_CONFIG(84)`/`MSP_OSD_CANVAS(189)`，甚至用 `MSP_SET_OSD_CONFIG` 调整元素位置（等价于半个配置器）。

### 7.4 实现要点 / 易错点

- **坐标顺序**：payload 里是 `[row][col]`，不是 `[col][row]`；
- **字符串长度上限 30**：但不要假设，按 size 字段解析；
- **字符码**：`0x20` 空格；`0x7F~0xFF` 是 OSD 符号（`osd_symbols.h` 的 `SYM_*`），没有字库前至少保证 ASCII 可读；
- **渲染时机**：DRAW_SCREEN 前不要输出半帧；视频叠加要做在垂直消隐期或双缓冲，避免撕裂；
- **多路兼容**：同一 UART 上可能同时有 FC 的其它 MSP 响应，按 cmd==182 过滤。

---

## 8. FC 侧实现参考（代码地图）

| 环节 | 位置 |
| --- | --- |
| 子命令枚举 / attr 位定义 | `src/main/io/displayport_msp.h` |
| vtable 实现（打包推送） | `src/main/io/displayport_msp.c`（`writeString:111`、`writeSys:135`、`heartbeat:65`、`writeFontCharacter:180`） |
| MSP 帧编码 | `src/main/msp/msp_serial.c:325` `mspSerialEncode`；`mspSerialPush:646` |
| 命令号 | `src/main/msp/msp_protocol.h:226` `#define MSP_DISPLAYPORT 182` |
| 画布/HD 切换 | `src/main/msp/msp.c` `MSP_SET_OSD_CANVAS`（收到后切 HD+MSP 并重启） |
| OSD 设备上报 | `src/main/msp/msp.c:964-1045` `MSP_OSD_CONFIG`（`OSD_FLAGS_OSD_MSP_DEVICE` 等） |
| 串口挂载 | `src/main/config/config.c:566-578`；`src/main/io/serial.c:361`（`MSP_DISPLAYPORT_UART`） |
| 字库参数 | `src/main/pg/displayport_profiles.c`（`fontSelection[severity]=severity` 默认） |
| 字符尺寸 | `src/main/drivers/osd.h`（12×18，2bit/px，`OSD_CHAR_BYTES=64`，可见 54B） |

