# Betaflight 多任务调度与实时性分析报告（STM32F4）

> 分析基于: `Betaflight_Fork` 飞控代码库（本报告只做代码分析，未修改任何代码）
> 报告日期：2026-08-15

---

## 0. 结论摘要

1. **Betaflight 是"单核裸机 + 协作式调度"**：没有 RTOS、没有线程抢占。`main()` 里只有一个死循环 `run()`，每圈调用一次 `scheduler()`，调度器在**当前任务**之间做选择，任何时刻 CPU 只属于一个任务。
2. **实时性靠"陀螺仪任务优先 + 忙等对齐时间边界 + 中断优先级分层"三条腿保证**：
   - `TASK_GYRO / FILTER / PID` 标记为 `TASK_PRIORITY_REALTIME`，调度器到时间边界后**先无条件执行**这三个任务（PID 控制环，F4 上默认 8 kHz），其它任务只能"插空"；
   - 调度器通过 DWT 周期计数器**忙等（busy-wait）**到 gyro 采样时刻，并用陀螺仪 EXTI 中断计数自适应校正周期；
   - 中断按 NVIC 抢占优先级分层：gyro EXTI 和 SPI/I2C DMA 用最高抢占级 `(0,0)`，UART 用 `(1,x)`，确保数据采集不受串口阻塞。
3. **UART / SPI / I2C 都是"中断/DMA 收数据 + 主循环任务处理数据"**：中断服务函数只做"搬字节到环形缓冲 / 推进 DMA 段 / 置标志"，真正解析和处理（MSP 帧解析、传感器融合、OSD 渲染）全部发生在调度器里的低优先级任务中，从而把 ISR 耗时压到最小、不影响陀螺仪时序。
4. **"处理器占用"是统计出来的**：`taskSystemLoad` 按"任务总执行时间/实际流逝时间"给出 `CPU_LOAD_PCT`；任务执行时间用**移动平均 + 峰值衰减估计**（`anticipatedExecutionTime`），调度器据此判断"剩余时间够不够跑这个任务"，不够就推迟到下一轮，宁可让任务晚跑也不打断 gyro 环。

---

## 1. 整体架构

```
main()                      (src/main/main.c)
 └─ systemInit()            时钟/看门狗/NVIC
 ├─ initPhase1()            配置加载(EEPROM)、外设初始化、陀螺仪/混控
 ├─ initPhase2()            串口/定时器/电机/调度器初始化
 ├─ initPhase3()            OSD/CMS/MSP/遥测初始化
 └─ run()                   while(1){ scheduler(); }   ← 唯一的"主循环"
                              (src/main/main.c:141)
```

调度器初始化时把所有任务按**静态优先级降序**插入任务队列（`queueAdd` 按 `staticPriority` 从大到小排序，`TASK_PRIORITY_REALTIME=-1` 排最前）。

---

## 2. 调度器核心算法（src/main/scheduler/scheduler.c）

### 2.1 实时（realtime）任务路径 —— 陀螺仪环的"专属时间槽"

```c
if (gyroEnabled) {
    nextTargetCycles = lastTargetCycles + desiredPeriodCycles;   // 下次 gyro 采样时刻
    schedLoopRemainingCycles = nextTargetCycles - nowCycles;

    // 任务严重超时(整周期丢失)则整周期跳变恢复
    if (schedLoopRemainingCycles < -desiredPeriodCycles)
        nextTargetCycles += desiredPeriodCycles * (1 + ...);

    // 接近时间边界时忙等对齐
    if (schedLoopRemainingCycles < schedLoopStartCycles) {
        while (schedLoopRemainingCycles > 0) {   // DWT 忙等 <2us 级别
            ... 读周期计数器 ...
        }
        schedulerExecuteTask(getTask(TASK_GYRO), ...);    // ① 采陀螺仪
        if (gyroFilterReady())  schedulerExecuteTask(getTask(TASK_FILTER), ...); // ② 滤波
        if (pidLoopReady())     schedulerExecuteTask(getTask(TASK_PID), ...);    // ③ PID+电机输出
        rxFrameCheck(...);            // 顺带查 RX 帧(ELRS 等)与失联保护
    }
}
```

- **F4 默认 `TASK_GYROPID_DESIRED_PERIOD = 125us`（8 kHz）**，即 GYRO/FILTER/PID 三个任务合成一个 8 kHz 控制环；
- `taskGyroSample` → `gyroUpdate()`（SPI 读原始角速度）；`taskFiltering` → `gyroFiltering()`；`taskMainPidLoop` → `subTaskPidController()` + `subTaskMotorUpdate()`（DSHOT 输出）。
- 忙等窗口 `schedLoopStartCycles`（1~12us 自适应）保证在**时间边界前尽量贴近**再执行，减少 jitter。
- 与陀螺仪 EXTI 锁定：统计 `GYRO_RATE_COUNT(25000)` 次中断平均周期 → 自适应 `desiredPeriodCycles`；再用 `GYRO_LOCK_COUNT(50)` 次累计 skew 校正 `lastTargetCycles`，把调度器锁在真实陀螺仪采样率上。

### 2.2 非实时任务路径 —— "最老优先" + "时间预算"双准则

```c
// 更新每个任务的 dynamicPriority = 1 + staticPriority * 任务年龄(过了几个周期)
for (task = queueFirst(); task; task = queueNext()) {
    if (task->staticPriority == TASK_PRIORITY_REALTIME) continue; // 实时任务不参与
    if (task->checkFunc) {
        // 事件驱动: checkFunc 返回 true 才置高优先级
        if (task->checkFunc(now)) { task->dynamicPriority = 1 + staticPriority; }
    } else {
        // 时间驱动: 年龄 = (now - lastExecutedAt) / desiredPeriod
        task->dynamicPriority = 1 + staticPriority * 年龄;
    }
    if (task->dynamicPriority > 当前最高) {
        预计耗时 = anticipatedExecutionTime >> 7;
        if (预计耗时 < 剩余时间 或 任务已老化/是SERIAL任务) {
            选中该任务;   // 时间预算内才允许跑
        }
    }
}
```

关键点：
- **事件驱动任务**（如 `TASK_OSD` 的 `osdUpdateCheck`、`TASK_RX` 的 check）：check 函数负责"是否有活"，有活才提升动态优先级；
- **时间预算**：`taskRequiredTimeCycles + taskGuardCycles < schedLoopRemainingCycles` 才允许执行，防止长任务吃掉下一拍 gyro 时间；任务跑得越久，`taskGuardCycles`（3~6us 自适应）越大，越不容易被放行；
- **老化豁免**：长时间跑不上来的任务（`taskAgePeriods > 1`）或 OSD/RX 超过 `rxRelaxDeterminism / osdRelaxDeterminism` 次被跳过时，把它的估计耗时缩小（×0.9）直到能插空执行——防止饿死；
- **`TASK_SERIAL` 特殊放行**：即使时间不够也执行，保证 MSP/CLI 串口不卡死。

### 2.3 任务耗时统计与 CPU 负载

- `schedulerExecuteTask()` 记录 `movingSumExecutionTime10thUs`，`anticipatedExecutionTime` 按 `TASK_EXEC_TIME_SHIFT(7)` 指数衰减的峰值估计；
- `taskSystemLoad()`：`CPU% = 100 × taskTotalExecutionTime / deltaTime`；
- `USE_LATE_TASK_STATISTICS` 编译选项下可输出每周期 gyro 时序的均值/方差、迟到任务统计（`DEBUG_SCHEDULER_DETERMINISM`）。



---

## 3. 任务清单与优先级（src/main/fc/tasks.c）

| 任务 | 优先级 | 周期 | 说明 |
| --- | --- | --- | --- |
| GYRO / FILTER / PID | REALTIME(-1) | 125us (8kHz, F4) | 采样→滤波→PID→电机输出，独占时间槽 |
| SYSTEM | MAX(255) | 100Hz | 系统状态机 |
| SERIAL | HIGH | 循环 | MSP/CLI/遥测串口收发（放行执行） |
| RX | MEDIUM_HIGH | 1ms | RC 数据解析（check 事件驱动） |
| OSD | LOW | 12Hz | 见 osd.md（check 事件驱动 + 时间切片） |
| TELEMETRY / GPS / BARO / ALTITUDE / BATTERY / VTXCTRL 等 | LOW~MEDIUM | 各自周期 | 传感器与外围处理 |

调度器保证：**8 kHz 控制环在任何情况下优先**；其余任务按"越久没跑越优先"的加权轮询插空运行，且受时间预算约束。

---

## 4. UART 收发与中断（以 STM32F4 为例）

### 4.1 驱动分层

- 通用层：`src/main/drivers/serial.c` / `serial_uart.c`（环形缓冲、vtable）
- F4 平台层：`src/platform/STM32/serial_uart_stm32f4xx.c`（寄存器操作、中断）

### 4.2 中断向量

`serial_uart.c` 为每个串口生成 `USARTx_IRQHandler` → `uartIrqHandler(uartPort)`：

```c
void uartIrqHandler(uartPort_t *s)
{
    // RX：逐字节模式（无 DMA 时），读 DR 入环形缓冲；有 rxCallback 则直接回调
    if (!s->rxDMAResource && RXNE) {
        if (s->port.rxCallback) s->port.rxCallback(USARTx->DR, data);
        else rxBuffer[rxBufferHead++] = USARTx->DR;
    }
    // TX：无 DMA 时 TXE 中断把 txBuffer 逐字节发出去，空则关 TXE 中断
    if (!s->txDMAResource && TXE) { ...USART_SendData(...)... }
    // 传输完成(TC) / 溢出(ORE) / 空闲(IDLE) 处理
}
```

### 4.3 两种收发模式

| 模式 | 触发 | 数据流 |
| --- | --- | --- |
| RX 中断（默认） | RXNE 每字节一次中断 | DR → 环形缓冲 → `TASK_SERIAL` 里 MSP/CLI 解析 |
| RX DMA（`USE_UARTx_RX_DMA`） | DMA 装满/半满/空闲 | 内存 ← DMA ← DR，`uartTotalRxBytesWaiting` 用 `DMA_GetCurrDataCounter` 计算 |
| TX 中断 | TXE 每字节 | 环形缓冲 → DR（每字节一次中断，CPU 开销大） |
| TX DMA（`USE_UARTx_TX_DMA`） | DMA TC 中断 | 环形缓冲 → DMA → DR；`uartDmaIrqHandler()` 在 TC 时继续搬下一段 |

### 4.4 中断优先级（NVIC）

见 `src/main/drivers/nvic.h`（F4 为 4bit 抢占 + 4bit 子优先级）：

```
NVIC_PRIO_SERIALUARTx_TXDMA   (1,0)   ← 串口发送 DMA 中最高
NVIC_PRIO_SERIALUARTx_RXDMA   (1,1)
NVIC_PRIO_SERIALUARTx         (1,2)   ← 逐字节 UART 中断
```

- UART 优先级 **低于** gyro EXTI `(0,0)` 和 SPI DMA `(0,0)`——串口再忙也不会打断陀螺仪采集；
- 串口 ISR 只做"搬字节"，不做协议解析，所以 ISR 时间极短。

---

## 5. SPI 收发与中断（F4 平台）

### 5.1 驱动分层

- `src/main/drivers/bus_spi.c`（总线/设备抽象、段链表）、`bus_spi_impl.h`
- `src/platform/STM32/bus_spi_ll.c`（LL 库寄存器级 DMA 启动）

### 5.2 传输模型（DMA 为主）

```
主循环任务（如 TASK_GYRO 读陀螺仪 / MAX7456 刷屏 / 外置 FLASH）
   └─ busRawTransfer / spiSequenceStart(dev)
        └─ spiInternalStartDMA(dev)：配置 TX/RX 双 DMA，RX 无数据时写入 dummyRxByte
             └─ RX DMA 传输完成中断(NVIC_PRIO_SPI_DMA=(0,0))
                  └─ dmaSetHandler 注册的回调 → spiProcessSegmentsDMA(dev)
                       ├─ 逐个 segment 推进（CS 拉低→下一段 DMA→CS 拉高）
                       └─ 全部完成 → 置 BUS_SPI_FREE → 任务侧 poll 到完成继续
```

- 阈值 `SPI_DMA_THRESHOLD=8`：≥8 字节用 DMA，小命令用寄存器轮询；
- 也提供**纯轮询模式** `spiProcessSegmentsPolled()`（不开 DMA 时），主循环里同步完成；
- SPI 优先级 `(0,0)` 与 gyro EXTI 同级最高——保证陀螺仪数据在 8kHz 环内"读得到、读得完"。

### 5.3 与主循环的关系

- 陀螺仪：`taskGyroSample → gyroUpdate()` 在实时时间槽内发起 SPI 读（DMA/轮询），拿到数据马上滤波+PID；
- MAX7456：`TASK_OSD` 的 `TRANSFER` 状态发起 `max7456DrawScreen()` DMA，`displayIsTransferInProgress()` 判断没传完就让 OSD 任务原地等待，不打扰 gyro 环；
- 外置 FLASH/黑匣子：`TASK_FLASHFS` 等低优先级任务按段异步写。

---

## 6. I2C 收发与中断（F4 平台）

### 6.1 驱动分层

- `src/main/drivers/bus_i2c.c / bus_i2c_busdev.c`（总线设备抽象、`i2cBus*` 接口）
- `src/platform/STM32/bus_i2c_stm32f4xx.c`（F4 事件/错误中断状态机）

### 6.2 非阻塞状态机（事件中断驱动）

```
任务侧: i2cRead/i2cWrite → 填 state(addr/reg/bytes/读写指针) → 置 busy=1 → 触发 START → 开 EVT/ERR 中断 → 返回
ISR 侧: I2Cx_EV_IRQHandler → i2c_ev_handler() 状态机:
        START 已发→送从机地址; 地址ACK→发子地址(reg); 发完→数据字节; 收完→STOP
        I2Cx_ER_IRQHandler → i2c_er_handler(): 错误→清标志、重初始化、置 error
任务侧: i2cWait() 轮询 state->busy 直到完成(超时 10ms 判硬件故障) 或 i2cBusy() 查询
```

- I2C 优先级 `NVIC_PRIO_I2C_EV/ER = (0,0)`（最高，与 gyro EXTI 同级），但**总线上有数据时中断很短**，状态机每次只处理一个事件；
- 常见传感器（气压计、罗盘、OLED）都是"发请求→轮询 busy→用数据"，主循环不被 I2C 阻塞超过几个字节的时间。

---

## 7. 实时性如何保证（重点）

| 手段 | 机制 | 代码位置 |
| --- | --- | --- |
| ① 控制环独占时间槽 | REALTIME 任务绕过队列，忙等对齐后强制执行 | `scheduler.c` `scheduler()` |
| ② 时间预算 | 预计耗时+guard < 剩余时间 才放行其它任务 | `scheduler.c` `scheduler()` |
| ③ 自适应守护 | `schedLoopStartCycles`(1~12us)、`taskGuardCycles`(3~6us) 动态调整 | `scheduler.h` 宏 |
| ④ 陀螺仪锁频 | EXTI 计数自适应周期 + skew 校正 | `scheduler.c` gyro 段 |
| ⑤ ISR 只搬数据 | UART/SPI/I2C ISR 最小化，协议解析都在任务里 | 各驱动 |
| ⑥ 中断优先级分层 | gyro EXTI/SPI DMA/I2C=(0,0) > UART=(1,x) > 其余 | `nvic.h` |
| ⑦ 任务时间切片 | OSD 一帧拆多次执行；`schedulerSetNextStateTime` 立即续跑 | `osd.c` 状态机 |
| ⑧ 可配置让步 | `rxRelaxDeterminism` / `osdRelaxDeterminism` 让 RX/OSD 更容易插空 | `pg/scheduler` |

**核心矛盾**：8kHz 控制环要求每个 125us 时间窗内必须完成"读陀螺仪→滤波→PID→输出"，其余一切（串口、OSD、GPS、遥测）都只能利用**剩余时间**。Betaflight 的选择是"软实时 + 严格保控制环"：
- 控制环保证硬实时（忙等对齐 + 最高中断优先级保护）；
- 其余任务软实时，靠"年龄老化 + 时间预算"尽力而为，宁可丢一帧 OSD 也不丢一拍 PID。

---

## 8. 中断与主循环的交互模型（一图流）

```
                 ┌──────────────────── 中断域（ISR，越短越好）────────────────────┐
  陀螺仪 EXTI ──► │ (0,0) 置 gyroSyncEXTI/计数，调度器据此锁频                    │
  SPI RX DMA ──► │ (0,0) 搬完 segment，回调 spiProcessSegmentsDMA 推进 CS/下一段  │
  I2C EVT/ERR ─► │ (0,0) 事件状态机推进一字节/一事件                              │
  USART IRQ ───► │ (1,2) RXNE→环形缓冲 / TXE→发一字节                             │
  USART TX DMA ► │ (1,0) TC→继续搬下一段                                          │
                 └───────────────────────────┬────────────────────────────────────┘
                                             │ 标志/环形缓冲/状态
                                             ▼
                 ┌──────────────────── 任务域（调度器，时间分片）─────────────────┐
  scheduler() ──► │ 1. 到 gyro 边界 → 强制跑 GYRO→FILTER→PID（8kHz）             │
                 │ 2. 剩余时间预算内 → 跑最"老"的任务：SERIAL(解析MSP/CLI)        │
                 │    / RX(解析RC帧) / OSD(画一个元素) / GPS / TELEMETRY / ...    │
                 └──────────────────────────────────────────────────────────────┘
```

---

## 9. 关键源码位置

- `src/main/main.c:141` `run()` 主循环
- `src/main/scheduler/scheduler.c` `scheduler()`、`schedulerExecuteTask()`、陀螺仪锁频
- `src/main/scheduler/scheduler.h` 优先级枚举、`TASK_PERIOD_HZ`、宏
- `src/main/fc/tasks.c:384-386` GYRO/FILTER/PID 实时任务；`tasks.c:433` TASK_OSD
- `src/main/fc/core.c:1369-1426` `taskGyroSample`/`taskFiltering`/`taskMainPidLoop`
- `src/main/drivers/serial_uart.c:575-645` UART IRQ 向量；`uartIrqHandler`（F4 在 `src/platform/STM32/serial_uart_stm32f4xx.c:300`）
- `src/main/drivers/nvic.h` 全部 NVIC 优先级
- `src/main/drivers/bus_spi.c:503/600` `spiProcessSegmentsDMA`/`spiProcessSegmentsPolled`；`src/platform/STM32/bus_spi_ll.c:439` `spiInternalStartDMA`
- `src/platform/STM32/dma_stm32f4xx.c:60-75` DMA1/2 各 Stream IRQ；`dmaSetHandler`(99) 注册回调
- `src/platform/STM32/bus_i2c_stm32f4xx.c:129-159` I2C 事件/错误中断；`i2cWriteBuffer`(169)/`i2cWait`(223) 状态机
- `src/platform/STM32/include/platform/platform.h:410` `TASK_GYROPID_DESIRED_PERIOD 125`（8kHz）

