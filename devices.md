# Betaflight 飞控程序外设接口实现分析报告 (devices.md)

> 分析对象:本仓库 `src/platform/` 下 **STM32 / ESP32 / CH32** 三个平台的驱动实现。
> CH32 与 ESP32 均为**非官方移植分支**(CH32 移植者为 Temperslee,ESP32 为社区 bring-up)。
> 本报告只做代码分析,不修改任何代码。

---

## 0. 架构总览:Betaflight 的"驱动抽象框架"

三个平台共用同一套上层抽象接口(`src/main/drivers/`),平台差异完全被隔离在
`src/platform/<PLATFORM>/` 目录中。上层代码通过以下约定访问底层:

- **能力宏(Trait Macro)**:`platform.h` 中定义 `UART_TRAIT_AF_PIN`、`SPI_TRAIT_AF_PIN`、
  `I2C_TRAIT_AF_PIN`、`DMA_TRAIT_MUX`、`USE_DMA_SPEC`、`USE_DSHOT_BITBANG` 等,
  编译期决定驱动行为。
- **硬件描述表**:每个外设导出一个 `xxxHardware[]` 常量数组(如 `uartHardware[]`、
  `i2cHardware[]`、`spiHardware[]`、`timerDefinitions[]`),由 target 的 `#define` 裁剪。
- **实现函数签名**:平台目录负责实现特定函数(如 `uartReconfigure()`、`spiSequenceStart()`、
  `pwmDshotMotorHardwareConfig()`),上层代码直接调用。
- **共用代码**:CH32 大量复用 `src/platform/common/stm32/` 中的公共文件
  (`serial_uart_hw.c`、`bus_spi_hw.c`、`dshot_dpwm.c`、`pwm_output_dshot_shared.c` 等),
  仅通过 `USE_CHBSP_DRIVER` 宏切换寄存器层 API(标准外设库)。

### 三平台 SDK/寄存器层对比

| | STM32 | CH32H415 | ESP32-S3 |
|---|---|---|---|
| 寄存器层 | STM32 LL / HAL / 标准外设库(F4) | WCH 标准外设库 `ch32h417_*.c`(仿 STM32 SPL) | ESP-IDF `hal/xxx_ll.h` 寄存器宏 |
| 架构 | ARM Cortex-M4/M7/M33/M55/N6 | RISC-V `rv32imafc` | Xtensa LX7 |
| 编译工具链 | arm-none-eabi-gcc | `riscv-wch-elf`(WCH 工具链) | `xtensa-esp32s3-elf` |
| 构建文件 | `STM32*.mk`(按系列) | `CH32H4.mk` | `ESP32S3.mk` |
| DMA | DMA1/DMA2 流(Stream)+ DMAMUX | DMA1/DMA2 通道(Ch)+ DMAMUX | GDMA + lldesc 链表 |
| 定时器 | 通用定时器框架(LL/StdPeriph) | 通用定时器框架(StdPeriph) | **无通用定时器**(空桩),用 LEDC/RMT |

---

## 1. UART

### 1.1 STM32 — `serial_uart_ll.c`(主力 LL 实现)

- **驱动模型**:中断 + 可选 DMA(接收循环 DMA,发送单次 DMA)。
- **核心函数** `uartReconfigure()`:
  - 先关中断清 `CR1/CR3`,再 `LL_USART_Disable/DeInit`,避免 `UE=0` 时 TC 恒置位导致中断风暴。
  - 用 `LL_USART_InitTypeDef` 设置波特率/数据位(奇偶校验时 9bit)/停止位/流控。
  - `usartConfigurePinInversion()` 利用 LL 的 **TX/RX 引脚电平反相**能力实现 SERIAL_INVERTED(SBUS 等)。
  - `UART_TRAIT_PINSWAP` 时通过 `LL_USART_SetTXRXSwap()` 做 TX/RX 交换。
- **接收路径**:
  - 有 DMA(循环模式)时由 DMA 填充环形缓冲,`IDLE` 中断驱动 `idleCallback`。
  - 无 DMA 时走 `RXNE` 中断,逐个字节写入环形缓冲。
  - 错误标志 PE/FE/NE/ORE 逐个清除(ORE 会同时做 RX 数据刷新)。
- **发送路径**:
  - 有 DMA 时 `uartTryStartTxDMA()` 维护 `txBufferHead/Tail` 分片下发,
    DMA TC 中断调用 `handleUsartTxDma()` 续发;`uartDmaIrqHandler()` 处理 TCIF/TEIF。
  - 无 DMA 时用 `TXE` 中断逐字节 `TDR` 发送,发空后关 TXEIE。
  - 发送完成(TC)后 `uartTxMonitor()` 把 TX 引脚切回输入上拉,用于 `checkUsartTxOutput()` 半双工/外部反相器监控。
- **其他**:`STM32G4` 特判——G4 DMA 完成后不会自动清 EN,需手动
  `xLL_EX_DMA_DisableResource()` 以便下次 `IS_DMA_ENABLED` 判断。
- 另有 F4 的 `serial_uart_stdperiph.c`、各系列专用 `serial_uart_stm32{xx}.c`(H5/N6 的 I3C/LPUART 等)。


### 1.2 CH32H415 — `serial_uart_ch32h41x.c` + `serial_uart_ch32bsp.c`

- **移植方式**:把 STM32 的 `serial_uart_stm32f4xx.c`/`serial_uart_ll.c` 结构照搬到 WCH 标准外设库
  (API 名与 STM32 SPL 一致:`USART_Init`、`USART_SendData`、`USART_GetFlagStatus` 等),
  寄存器访问用 `USARTx->DATAR/STATR`。
- **硬件表 `uartHardware[]`**:定义了 USART1~USART8,每个都列出多组 RX/TX 引脚映射
  (如 USART1 的 PA10/PB7/PB15/PD12)并标注 `GPIO_AFx`;每个口带 `rxDMAMuxId/txDMAMuxId`
  (DMAMUX 请求号,如 `DMAMUX_DMAREQ_ID_USART1_RX`)与 `rxDMAResource/txDMAResource`。
- **`uartReconfigure()`**:
  - `USART_Cmd(DISABLE)` → `USART_Init()` 配置 → `USART_HalfDuplexCmd()` 支持 SERIAL_BIDIR(半双工)
    → `uartConfigureExternalPinInversion()`(**CH32 没有内部反相**,需外部反相器,故用共享层的 `inverter.c`)。
  - RX DMA 用 **循环模式**,`DMA_InitStructure.DMA_Mode = DMA_Mode_Circular`,外设地址为 `DATAR`;
    DMA 就绪后开 RXNE 中断做错误恢复。
- **中断处理 `uartIrqHandler()`**(`FAST_IRQ_HANDLER` = `FAST_CODE`,RISC-V 快速中断):
  - `RXNE`:无 DMA 时入环;`TC` 后 `uartTxMonitor()`;`TXE` 时逐字节 `USART_SendData()`。
  - `ORE` 通过读 `STATR`+`DATAR` 清除;`IDLE` 触发 `idleCallback`。
- **DMA TX**:`uartTryStartTxDMA()` 直接写 `((DMA_ARCH_TYPE*)s->txDMAResource)->MADDR` 与
  `xDMA_SetCurrDataCounter()`,用 `xDMA_GetCurrDataCounter()` 防 F4 式误判 TC。
- **注释遗留**:文件末尾 `//TODO: ADD UART 6\7\8 HERE!`,UART8 仍借用 USART8 引脚定义,
  说明该移植对高编号 UART 仍有 TODO 痕迹。

### 1.3 ESP32 — `serial_uart_esp32.c`

- **驱动模型**:纯 FIFO + 中断,**无 DMA**(`uartTryStartTxDMA()` 是空函数)。
- **硬件表 `uartHardware[]`**:UART0/1/2 三个控制器,`reg` 用平台自定义的
  `esp32UartDev0/1/2` 指针占位,运行时通过 `uartGetPortNum()` 解析回真实 `uart_dev_t*`。
- **引脚配置**:利用 **GPIO Matrix** 把任意引脚连接到 UART 信号
  (`esp_rom_gpio_connect_out_signal(pin, U0TXD_OUT_IDX, ...)` / `connect_in_signal`),
  对应 `UART_TRAIT_AF_PORT=1` 能力宏。
- **波特率/格式**:`uart_ll_set_baudrate(hw, baud, ESP32_APB_CLK_FREQ(80MHz))`,
  固定 8N1;`serialUART()` 里 `UNUSED(options)` —— **不支持奇偶校验/停止位/反相选项**。
- **接收**:RX FIFO 阈值=1 字节触发中断 + 空闲超时中断(RX_TOUT_THRESHOLD=10 位周期),
  保证低延迟。中断通过 `esp32IntrRoute/Register/Enable`(ROM `ets_isr_attach`)注册。
- **发送**:`uartEnableTxInterrupt()` 用 `UART_INTR_TXFIFO_EMPTY` 中断逐 FIFO 发送。
- **与其他平台的差异**:没有 IDLE 中断回调/无 DMA,`idleCallback` 相关路径不生效;
  依赖上层 `serial.c` 的软件缓冲。

---

## 2. I2C

### 2.1 STM32 — `bus_i2c_ll.c`(LL,中断式状态机)+ `bus_i2c_stm32f4xx.c`(老平台)

- **驱动模型**:EV/ER 双中断 + 状态机,支持读写缓冲区切分、总线错误恢复。
- `i2cEVIRQHandler()`:处理 NACK、TXIS(发数据)、RXNE(收数据)、TC(完成/重启)。
- `i2cRecoverFromISRError()`:全关中断 → 清 STOPF/TXIS/TXE → `CR2=0` →
  **PE 周期(Disable→NOP 延时→Enable)清除粘滞的 BUSY 位**。
- `i2cBusy()`:支持超时(超过 `I2C_TIMEOUT_US`)强制恢复,先关 NVIC 中断防止 ISR 竞争。
- 中断使能用**单次原子 CR1 写**(`__disable_irq` 包住),防止与 ISR 的 RMW 竞争导致残留中断。
- `bus_i2c_stm32f4xx.c` 是经典 SPL 事件中断实现;`bus_i2c_i3c.c` 用于新系列 H5/N6 的 I3C。


### 2.2 CH32H415 — `bus_i2c_ch32h41x.c`

- **移植方式**:直接沿用 STM32 F4 的 SPL 式 I2C 中断状态机(`I2C_EVT_IRQHandler` 逻辑逐段移植),
  `ev_state` 处理 EVT 中断、地址发送/子地址发送/数据收发/STOP;`er_state` 处理 ERR。
- **硬件表 `i2cHardware[]`**:I2C1~I2C4,引脚带 AF(如 I2C1:PB6/PB8 SCL、PB7/PB9 SDA,AF4),
  `ev_irq/er_irq` 为 `I2Cx_EV_IRQn/I2Cx_ER_IRQn`。
- **`i2cInit()`**:`i2cUnstick()` 先解死总线 → `I2C_Init()` 配置时钟速率 → `I2C_Cmd` 使能 →
  `I2C_StretchClockCmd` 开时钟拉伸 → `NVIC_SetPriority/EnableIRQ` 注册 EV/ER 中断
  (原 `NVIC_Init` 调用被注释掉,改为直接 NVIC API)。
- **引脚模式**:`IOCFG_I2C_PU`(开漏+内部上拉,可省外部上拉)与 `IOCFG_I2C` 两种可选。
- 中断使能时机与 STM32 F4 版一致——"由第一次请求使能",总线空闲时关闭 EVT/ERR 防 BTF 假中断。

### 2.3 ESP32 — `bus_i2c_esp32.c`

- **驱动模型**:**命令寄存器(CMD)式主控**,非中断状态机。直接用 `i2c_ll` 把一次传输编码成
  START → 写地址 → (RESTART → 写子地址) → 读 N 字节(末字节 NACK) → STOP 的硬件命令序列。
- `i2cReadBuffer()`:逐个填充 `i2c_ll_master_write_cmd_reg(hw, cmd, idx)`,
  `i2c_ll_update()` + `i2c_ll_start_trans()` 启动,`i2cWaitCmdDone()` 轮询完成(带超时)。
- `i2cBusy()`:`i2c_ll_is_bus_busy()` 直接查硬件忙位,无软件状态机。
- **总线恢复** `i2cRecover()`:9 个时钟脉冲解死锁。
- 时钟源:ESP32-S3 用 XTAL 40MHz,原版 ESP32 用 APB 80MHz(条件编译区分)。
- 引脚经 GPIO Matrix(无 AF 概念),支持 `i2cPinConfigure()` 重映射与上拉配置。
- **限制**:单次传输长度受硬件命令槽限制;无 DMA;错误计数为软件计数器。

---

## 3. SPI

### 3.1 STM32 — `bus_spi_ll.c`(LL)

- **轮询传输**:`spiInternalReadWriteBufPolled()` 按 FIFO 阈值/16 位字优化;`spiInternalReadWriteBuf()` 逐字节。
- **DMA 传输**:`spiInternalInitStream()` 填 `LL_DMA_InitTypeDef`,RX/TX 用 `dummyTxByte(0xff)/dummyRxByte` 补位;
  H7/F7 对 TX 做 `SCB_CleanDCache_by_Addr`、RX 做 `CleanInvalidateDCache`(**Cache 一致性处理**)。
  `spiInternalStartDMA()` 用 RX 的 TC 中断作为传输完成通知。
- **DMA 安全性检查**(`spiSequenceStart`):逐 segment 检查
  - H7/F7:DTCM 不可被 DMA 访问、RX 缓冲必须 Cache 对齐(否则回退轮询);
  - G4:CCM RAM 不可被 DMA 访问;
  - 只有多段 / 长度 ≥ `SPI_DMA_THRESHOLD(8)` / 保持 CS 时才走 DMA,否则轮询。
- **时钟**:`spiDivisorToBRbits()` 考虑 F7 的 APB1/APB2 差异,`defaultInit` 为 8bit/主模式/NSS 软控/CPOL 高/CPHA 2 边沿。
- `STM32H5` 使用新的 `SPI_CFG1_RXDMAEN/TXDMAEN` + `LL_SPI_SetTransferSize`(HAL2 风格);
  `STM32N6` 的 GPDMA 尚未支持 SPI DMA(TODO)。

### 3.2 CH32H415 — `bus_spi_ch32h41x.c`

- **移植方式**:SPL 风格。`defaultInit` 与 STM32 F4 版几乎一致,`spiDivisorToBRbits()` 用
  `ffs(divisor)-2 << 3` 写 `SPI_CTLR1` 的 BR 位。
- **DMA**:`spiInternalResetDescriptors()` 初始化 `DMA_InitTypeDef`;`spiInternalStartDMA()` 配置
  流寄存器(通道号映射自 DMAMUX)并开 `SPI_I2S_DMAReq_Tx`。
- **`spiInternalStopDMA()`**:有 RX 时双流都停并清标志;无 RX 时轮询 `BSY` 后排空 `DATAR`。
- **`spiSequenceStart()`**:切换分频/极相(CPOL/CPHA 由 `leadingEdge` 决定),段长度 ≥ 阈值或
  多段或保持 CS 时走 DMA,否则 `spiProcessSegmentsPolled()` —— 与 STM32 决策逻辑一致。
- **注意**:未发现 H7 式 Cache 处理(CH32H415 无 Cache/DMA 内存限制,`IS_CCM` 宏被注释掉)。

### 3.3 ESP32 — `bus_spi_esp32.c`

- **GPIO Matrix**:SCK/MOSI/MISO 全部经矩阵路由(SPI2=FSPI、SPI3=HSPI 信号索引)。
- **轮询传输**:`spiInternalReadWriteBufPolled()` 每次最多 `SPI_MAX_TRANSFER_SIZE(64)` 字节,
  写 `spi_ll_write_buffer()` 硬件缓冲(无 FIFO,直接写寄存器)。
- **DMA 传输**(GDMA):`spiInternalStartDMA()` 使用 **linked-list descriptor(lldesc)**:
  - TX/RX 各一个 `lldesc_t`,`owner=1` 交给 DMA 硬件;无数据缓冲时用 `dummyTxBuf(0xFF)/dummyRxBuf`。
  - 通过 `gdma_ll_tx_set_desc_addr()` 装载描述符,`gdma_ll_rx_start()` 启动;
  - 用 **RX SUC_EOF 中断**(`gdma_ll_rx_enable_interrupt(..., GDMA_LL_EVENT_RX_SUC_EOF)`)作为传输完成通知,
    ISR 里拉高 CS、停 DMA、调 `spiIrqHandler()`。
- **`spiInitBusDMA()`**:为每个 SPI 设备分配一个空闲 GDMA 通道做 TX、一个做 RX,
  `gdma_ll_*_connect_to_periph()` 关联触发源,开 data/descriptor burst,注册 RX 完成 handler。
- 分频用 `spiCalculateDivider()` 按 APB 80MHz 整数除法,`spi_ll_master_set_clock()` 应用。

---

## 4. DMA

### 4.1 STM32 — `dma_stm32{xx}.c`(按系列)

- 以 H7 为例:`dmaDescriptors[]` 定义 **DMA1/DMA2 各 8 条流**,`DEFINE_DMA_CHANNEL(DMA1, 0, 0)` 记录
  中断号偏移;`DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_ST0_HANDLER)` 批量生成 16 个流中断。
- `dmaSetHandler()` 用 `HAL_NVIC_SetPriority(基/子优先级)+ EnableIRQ`。
- `dmaGetDataLength()` 用 `LL_EX_DMA_GetDataLength()`。
- 各系列差异:CM4(F4)是 8 流无 DMAMUX;G4/H7/C5 有 **DMAMUX**(`dmaMuxEnable()` 经 `DMA_MuxChannelConfig`);
  N6 是 GPDMA(API 不同,部分外设驱动仍是 TODO)。
- `dma_reqmap_mcu.c` 提供 `dmaGetChannelSpecByTimer()` 等按外设查 DMAMUX 请求号的映射表。

### 4.2 CH32H415 — `dma_ch32h41x.c`

- **结构**:仿 STM32 F4 —— `dmaDescriptors[]` 定义 **DMA1/DMA2 各 8 个通道**
  (`DEFINE_DMA_CHANNEL(DMA1, 1, 0)` 等),8 个通道共享 4 个中断向量(DMA1_CH1..CH8_HANDLER)。
- **RISC-V 快速中断**:所有 DMA 中断用 `__FAST_INTERRUPT` + `DEFINE_DMA_IRQ_HANDLER`,
  注释提示需硬件压栈特性 "WCH-Interrupt-fast"。
- `dmaMuxEnable()` 调 `DMA_MuxChannelConfig(index, dmaMuxId)`(DMAMUX 通道配置),与 STM32 G4/H7 思路一致。
- `dmaGetDataLength()` 返回 `DMA_GetCurrDataCounter()` 当前剩余计数。
- 中断优先级用 `NVIC_SetPriority(irq, priority)`(单值,无基/子拆分)。

### 4.3 ESP32 — `dma_esp32.c`

- **GDMA 通道池**:ESP32-S3 共 **5 条 GDMA 通道**(`DEFINE_DMA_CHANNEL(DMA_CH0_HANDLER)`...)。
- **动态分配**:`dmaGetFreeIdentifier()` 顺序下发通道;`dmaSetHandler()` 从
  `gdmaCpuIntrPool[]` 动态领取 CPU 中断线,`cpuIntrToChannel[]` 反查映射。
- **ISR 分发**:公共 `gdmaRxIsrDispatch()` 清 RX SUC_EOF 中断后调用 `descriptor->irqHandlerCallback`。
- **受限**:`dmaGetDataLength()` 返回 0(桩);`dmaEnable()` 为空;`dmaMuxEnable` 概念不存在
  (GDMA 用 `gdma_ll_*_connect_to_periph` 绑定外设触发源,见 SPI 节)。
- 初始化 `esp32DmaInit()` 使能总线时钟并复位。


---

## 5. Timer(通用定时器框架)

### 5.1 STM32 — `timer_hal2.c` / `timer_hal.c` / `timer_stdperiph.c`

- **框架**:`timerConfig[]` 保存每定时器的 update/edge/overflow 回调链;
  `lookupTimerIndex()` 用地址移位 switch 快速把 `TIMx` 指针映射到索引;
  `USED_TIMERS` 位图(由 target 定义)决定编译哪些定时器。
- 提供 `timerInit()`、`timerConfigureTimeBase()`、`timerChInit()`、`timerChConfigIC/OC()`、
  `timerChCaptureCompareEnable/Disable()`、`timerGetPeriod()` 等完整 API,支持输入捕获(飞控需要
  的 PPM/PWM 输入)、输出比较、PWM 模式、强制溢出等。
- `timer_hal2.c` 是为带 HAL2(Cube 2.0)的新系列(C5 等)写的 LL 变体,`timerHandle_t` 只存 Instance 指针。
- NVIC 用 `HAL_NVIC_SetPriority(NVIC_PRIO_TIMER)` 统一管理。

### 5.2 CH32H415 — `timer_ch32h41x.c` + `timer_ch32bsp.c`

- **框架**与 STM32 逐行对应:`timerConfig[]`、`usedTimers[]`、`lookupTimerIndex()`(TIM1..TIM20)。
- `timerDefinitions[]`:TIM1~TIM12,含 RCC 门控(`RCC_HB1/HB2`)与 `inputIrq`
  (TIM1/TIM8 用 `TIMx_CC_IRQn`,其余用 `TIMx_IRQn`)。
- `fullTimerHardware[]`(`USE_TIMER_MGMT` 开启):TIM1/8(含互补 N 通道)、TIM2~TIM12 的完整引脚表,
  覆盖 PE9/PA8/PB13 等多组复用,满足 8 电机 + 舵机 + 双向 DShot 需求。
- `timerClockFromInstance()` 直接返回 `HCLKClock`(所有定时器同一时钟,未区分 APB1/APB2)。
- 对外提供的 `timerOCInit/timerChCCR/timerDmaSource` 等经 WCH SPL(`TIM_OCInit` 等)实现。

### 5.3 ESP32 — `timer_esp32.c`(**空桩**)

- **关键结论**:ESP32 移植**没有通用定时器框架**。文件头注释明确:
  "The ESP32 uses LEDC for PWM and RMT for DShot/LED, not general-purpose timers."
- `fullTimerHardware[1] = {{0}}` 空表;`timerInit()` NOOP;
  `timerGetByTagAndIndex()/timerGetTIMNumber()` 等全部返回 NULL/-1。
- 因此 target.h 里 `#undef USE_SERIALRX_SPEKTRUM`(绑定需要定时器)、
  `#undef USE_DSHOT_BITBANG`、`#undef USE_DSHOT_TELEMETRY`、`#undef USE_OSD_HD` 等——
  凡是依赖通用定时器/高精度时序的功能在 ESP32 上被关闭。

---

## 6. PWM 输出(电机 / 舵机 / 蜂鸣器)

### 6.1 STM32 — `pwm_output_hw.c`(标准 PWM)+ 共享 `dshot_dpwm.c`

- **标准 PWM**:`pwmOutputConfig()` → `timerReconfigureTimeBase()` + `pwmOCConfig()`(PWM1 模式) +
  `TIM_CtrlPWMOutputs()`(高级定时器主输出使能);`pwmWriteChannel()` 直接写 `CCR`。
- 电机初始化计算 `pulseScale/pulseOffset` 把 0..1 值映射到 CCR(BRUSHED 时用整个周期)。
- 舵机:`servoDevInit()` 50Hz(1MHz/速率)定周期,写 `servoCenterPulse` 初值。
- 蜂鸣器:`common/stm32/pwm_output_beeper.c`。

### 6.2 CH32H415 — `pwm_output_ch32.c`

- **与 STM32 同构**,API 为 SPL:`TIM_OCInitStructure` → `timerOCInit()` +
  `timerOCPreloadConfig()` + `TIM_CtrlPWMOutputs()` + `TIM_Cmd()`。
- 支持 `TIMER_OUTPUT_N_CHANNEL` 互补通道与极性反转。
- 电机标准 PWM 流程与 STM32 一致(连续更新模式 `useContinuousUpdate`、强制溢出同 TIM 分组)。
- 舵机 50Hz 初始化与 STM32 相同逻辑。

### 6.3 ESP32 — `pwm_motor_esp32.c` / `pwm_servo_esp32.c` / `pwm_beeper_esp32.c`

- **全部基于 LEDC 外设**(不使用通用定时器):
  - 电机:`motorPwmDevInit()` 配置 LEDC timer0,16bit 分辨率(高速率自动降到 14bit),
    `ledc_ll_set_clock_divider()`(Q18.4 格式分频)驱动 80MHz APB;
    每电机一个 LEDC 通道(0-3),`ledc_ll_bind_channel_timer()` 绑定到 timer0,
    GPIO 输出经矩阵连到 `LEDC_LS_SIG_OUT0_IDX + ch`。
    `pwmMotorWrite()` 用 `pulseScale/pulseOffset`(基于占空比计数)写 `ledc_ll_set_duty_int_part`。
  - 舵机:`servoDevInit()` 用 **timer1、50Hz、16bit**,通道 4-7(与电机分时复用 LEDC 定时器),
    初值 `servoCenterPulse`。
  - 蜂鸣器:`beeperPwmInit()` 用 **timer2、10bit 分辨率**,通道 7,50% 占空比方波,
    `pwmWriteBeeper()` 开关。
- 所有 PWM 输出**没有 DMA、没有中断**——纯寄存器写入,无电机遥测(telemetry 相关 vtable 是桩)。


---

## 7. DShot 电机协议

### 7.1 STM32 — `pwm_output_dshot.c`(定时器+DMA 传统方式)+ `dshot_bitbang_ll.c`(Bitbang)

- **传统 DMA 方式**(`pwm_output_dshot.c`):
  - `pwmDshotMotorHardwareConfig()`:每个电机通道配 OC(PWM1)+ DMA,把预编码的 16 位帧
    (由 `loadDmaBufferDshot()` 生成 `MOTOR_BIT_1/BIT_0` 脉宽表)用 DMA 以定时器速率搬进 CCR。
  - `motor_DMA_IRQHandler()`:TC 后停 DMA 与 `TIM_DMACmd`;`USE_DSHOT_TELEMETRY` 时
    `pwmDshotSetDirectionInput()` 切输入并用 DMA 采集 GCR 回传边沿,解码 `decodeTelemetryPacket()`。
  - **DSHOT_DMAR(多电机突发)**:`useBurstDshot` 时用 `dmaTimUPRef` + `dshotBurstDmaBuffer`,
    一次 DMA 更新一个定时器上所有 CCR(`TIM_DMAConfig(TIM_DMABase_CCR1, 4Transfers)`)。
  - 帧同步:`pwmCompleteDshotMotorUpdate()` 重置 `ARR` 并把所有电机的 DMA 请求同时使能。
- **Bitbang 方式**(`dshot_bitbang.c` + `dshot_bitbang_ll.c`):
  - 用 DMA 直接把**整段 GPIO 时序**(16 帧 × 3 状态 set/hold/reset)写进 `BSRR`,
    一个 GPIO 端口上所有电机共享一次 DMA,定时器只做节奏(pacer)。
  - `bbTimerHardware[]` 用 TIM1/TIM8 的 4 通道作 pacer;双向 DShot 时
    `bbSwitchToInput()` 重配 `ARR` 为输入过采样频率,再用 DMA 从 `IDR` 采样回传。
  - LL 版 `dshot_bitbang_ll.c` 与 CH32 的 stdperiph 版逻辑一致。

### 7.2 CH32H415 — `pwm_output_dshot.c`(SPL 版)+ `dshot_bitbang.c` + `dshot_bitbang_stdperiph.c`

- **传统 DMA 方式**:与 STM32 `pwm_output_dshot.c` 逐函数对应,但全部换成 SPL
  (`TIM_DMACmd`、`DMA_MuxChannelConfig`、`TIM_DMAConfig`)。也支持 DMAR 突发
  (`DMA_MemoryDataSize_Word` + `DMA_PeripheralDataSize_HalfWord` 写入 CCR)。
- 重要差异:`pwmDshotSetDirectionOutput()` 里先手动清 `DMA_CFGR1_EN`(不是 `xDMA_DeInit`),
  **telemetry 使用同一 DMA 通道方向切换**(与 STM32 相同方案)。
- **Bitbang**:`dshot_bitbang.c` 是主框架,`dshot_bitbang_stdperiph.c` 提供 GPIO/DMA/TIM 的
  SPL 底层(`bbGpioSetup` 直接操作 `GPIO_CFGLR/BSHR/INDR`、`DMA_PeripheralBaseAddr=(uint32_t)&bbPort->gpio->BSHR`)。
- **寄存器细节**:`bbLoadDMARegs()/bbSaveDMARegs()`(USE_DMA_REGISTER_CACHE)在中断里恢复 DMA 寄存器
  以减延迟。pacer 选择 TIM1 或 TIM8(由 `useDshotBitbangedTimer` 配置)。
- target.h 目前 `#undef USE_DSHOT_DMAR`,而 `USE_DSHOT_BITBAND` 开启,telemetry 相关宏被注释待测。

### 7.3 ESP32 — `dshot_esp32.c`(RMT 方式)

- **完全不同的实现路径**:用 **RMT(遥控发射外设)** 按位定时直接产生 DShot 波形。
- 定时表:20MHz(80/4)tick 下 DShot600/300/150 三套 `t1h/t1l/t0h/t0l`
  (600kbps:高 25 tick、低 8 tick 等,按位周期线性缩放)。
- `dshotEncodePacket()` 把 16 位帧编码成 `rmt_symbol_word_t` 符号数组(位序 MSB first),
  末位 0 值作为结束标记;`dshotWriteInt()` 把符号表逐字写入 RMT 通道内存
  (`RMT_MEM_BASE + ch*48*sizeof(item)`,非 FIFO 直存模式)。
- 初始化:`rmt_ll_enable_mem_access_nonfifo()`、`rmt_ll_tx_fix_idle_level(低)`、关闭载波/循环,
  GPIO 经矩阵连 `RMT_SIG_OUT0_IDX + ch`。
- **无遥测、无 Bitbang**:`dshotDecodeTelemetry()/dshotTelemetryWait()` 恒返回 true(桩),
  target 已 `#undef USE_DSHOT_TELEMETRY / USE_DSHOT_BITBANG`。最多 4 电机。

---

## 8. ADC

### 8.1 STM32 — `adc_stm32{xx}.c`(按系列)

- **DMA 驱动、连续扫描转换**:`adcInit()` 配置规则通道序列,`adcConfigureDmaChannel()` 用
  循环 DMA 把结果持续写入 `adcConversionBuffer[]`;`adcGetChannelValues()` 拷贝最新值。
- 支持内部通道(Vrefint/温度)、外部 VBAT/电流/RSSI;H7 等带 Cache 平台有 DMA 缓冲对齐处理。
- 各系列采样时间/时钟分频不同(`adc_stm32f4xx.c`、`adc_stm32h7xx.c` 等)。

### 8.2 CH32H415 — `adc_ch32h41x.c`

- **同 STM32 F4 模型**:`adcHardware[]` 定义 ADC1/ADC2,`adcTagMap[]` 把引脚映射到
  `ADC_Channel_x`(PA0~PA7/PB0~PB1/PC0~PC4 + Vrefint/温度)。`adcConversionBuffer` 用
  `volatile DMA_DATA` 32 对齐。
- `adcInitDevice()`:连续转换(`ADC_ContinuousConvMode=ENABLE`)、`ADC_RegularChannelConfig()`
  编排通道序列,`ADC_DMACmd` 使能 DMA,`ADC_SoftwareStartConvCmd` 启动。
- 校准代码被注释(`ADC_ResetCalibration`...),用 `ADC_LowPowerModeCmd` 替代。
- 受 `USE_DMA_SPEC` 影响,`dmaSpec->ref` 提供 DMA 资源并 `dmaMuxEnable()`。

### 8.3 ESP32 — `adc_esp32.c`(单次轮询)

- **无 DMA、无连续转换**:`adcReadChannel()` 每次 `adc_oneshot_ll_start()` + 轮询
  `adc_oneshot_ll_raw_check_valid()`(超时 10000 次迭代),12bit,衰减 12dB(0-3.3V)。
- 通道映射:ESP32-S3 GPIO1-10 → CH0-9(线性);原版 ESP32 GPIO36-39/32-35(非线性)。
- `adcGetValue()` 每次调用都做一次新转换(注释:"ADC runs at task rate ~10Hz, no DMA needed")。
- 内部参考:Vrefint 硬编码 1100mV;温度传感器**未实现**,返回 25°C。


---

## 9. USB(VCP / MSC)

| | STM32 | CH32H415 | ESP32-S3 |
|---|---|---|---|
| VCP | `serial_usb_vcp.c`(ST USB CDC 驱动,`CDC_Receive_DATA` 等)+ `usb_msc_hal.c/f4xx.c`(MSC) | `serial_usb_vcp_ch32h4.c` + `usb_msc_ch32h41x.c`(WCH USBHS 中间件,`usbd_cdc_acm.c`) | `serial_usb_vcp_esp32.c`(**USB-Serial-JTAG**,`usb_serial_jtag_ll`) |
| 连接检测 | USB 枚举/CtrlLineState 回调 | USB 枚举 | **SOF 帧轮询**(3ms 未见 SOF 判离线) |
| MSC | 支持(黑盒盘) | target 里 `#undef USE_USB_MSC`(默认关) | `#undef USE_USB_MSC`(关) |

- ESP32 的 VCP 是 USB-Serial-JTAG 硬件串口(CDC 兼容),波特率设置函数为空
  (JTAG 串口固定速率),`usbVcpRxBytesAvailable()` 轮询 FIFO。

---

## 10. LED strip(WS2811)

### STM32 — `light_ws2811strip_hal*.c / stdperiph.c`
- 定时器 PWM + DMA 直接搬 `ledStripDMABuffer[]`(每个 LED 的 0/1 脉宽序列)进 CCR;
  TC 中断里 `ws2811LedDataTransferInProgress=false`,支持单色循环刷新。

### CH32H415 — `light_ws2811strip_ch32h41x.c`
- 与 STM32 stdperiph 版同构:800kHz 时基、`BIT_COMPARE_1=period*2/3`、DMA 从内存写 CCR、
  `USE_WS2811_SINGLE_COLOUR` 时循环模式;`#warning` 提示必须 USE_DMA_SPEC 才能拿到 DMAMUX 请求号。

### ESP32 — `light_ws2811strip_esp32.c`
- **RMT**:按 WS2812 时序把每字节编码成符号(400ns/850ns/800ns/450ns),写入 `ws2812RmtBuffer[]`
  再逐字刷入 RMT 通道内存(软件刷新,非 DMA 链表)。通道号在 DShot 之后分配
  (`ws2812RmtChannel = dshotMotorCount`,S3 上 TX 通道共 4 个)。

---

## 11. 其他外设

### EXTI(外部中断)
| STM32 `exti.c` | CH32 `exti_ch32.c` | ESP32 `exti_esp32.c` |
|---|---|---|
| 标准 NVIC + EXTI 线配置 | SPL 版:`EXTI_Init()` + **2 个分组中断**(EXTI7_0 / EXTI15_8),`__FAST_INTERRUPT` | **单个共享 GPIO ISR**:读 `gpio_ll_get_intr_status`(0-31 与 high 32-48 两段),`ctz` 分发到每引脚回调 |

### Camera control(相机控制)
| STM32 `camera_control_stm32.c` | CH32 `camera_control_ch32.c` | ESP32 |
|---|---|---|
| 硬件 PWM 模式 | 硬件 PWM + **软件 PWM**(`TIM6_IRQHandler` 拉高、`TIM7_IRQHandler` 拉低,双定时器同步) | 无(未移植) |

### 系统时间源
| STM32 | CH32 | ESP32 |
|---|---|---|
| DWT 周期计数 + SysTick(`common/stm32/system.c`) | 同 STM32(共用 `common/stm32/system.c`) | **ESP32-S3 SYSTIMER(16MHz 52bit)**;原版 ESP32 用 Xtensa `CCOUNT` 累加器 |

### 其他 ESP32 桩/占位
- `persistent.c`、`config_flash.c`(分区存储)、`debug_esp32.c` 等。
- `serial_usb_vcp_esp32.c` 的 baud/ctrl-line 回调为空。
- `timer_esp32.c` 全桩(见第 5 节)。
- 无 transponder、无 audio、无 octo/quadspi(flash 芯片功能在 target.h 被关)。

---

## 12. 三平台实现路线对比总结

| 外设 | STM32(成熟/官方) | CH32H415(移植,仿 F4 SPL) | ESP32(移植,bring-up) |
|---|---|---|---|
| **UART** | LL/SPL + 中断 + DMA(循环 RX) | SPL + 中断 + DMA(循环 RX),引脚 AF 表齐全 | FIFO + 中断,**无 DMA**,GPIO 矩阵,仅 8N1 |
| **I2C** | LL 中断状态机 + 超时恢复 | SPL 中断状态机(照搬 F4) | 命令寄存器式主控,轮询完成 |
| **SPI** | LL + DMA + Cache 一致性 + DTCM/CCM 检查 | SPL + DMA(无 Cache 问题) | LL + **GDMA 链表描述符** |
| **DMA** | 流式 DMA1/2 + DMAMUX(按系列) | 通道式 DMA1/2 + DMAMUX(`DMA_MuxChannelConfig`) | GDMA 5 通道 + 动态中断池分配 |
| **Timer** | 完整通用定时器框架 | 完整框架(照搬 STM32) | **空桩**(无定时器) |
| **PWM** | 定时器 OC + CCR | 定时器 OC + CCR(SPL) | **LEDC** |
| **DShot** | 定时器+DMA / Bitbang(LL) | 定时器+DMA(SPL)/ Bitbang(stdperiph) | **RMT** 符号定时 |
| **ADC** | DMA 循环连续转换 | DMA 循环连续转换 | 单次轮询(无 DMA) |
| **USB VCP** | ST USB CDC | WCH USBHS 中间件 | USB-Serial-JTAG |
| **WS2811** | 定时器+DMA | 定时器+DMA(SPL) | RMT 软件编码 |
| **Camera ctrl** | 硬件 PWM | 硬件 + 软件 PWM(TIM6/7) | 未实现 |
| **EXTI** | 标准 NVIC | SPL + 2 分组中断 | 共享 GPIO ISR 分发 |

### 各平台移植成熟度评估(基于代码痕迹)

- **STM32**:最成熟。官方维护,多系列(CM4/7/G4/H5/H7/N6/C5)LL/HAL/SPL 三套并存,完整覆盖
  DMA、Cache、遥测、DMAR、I3C、octo/quadspi 等高级特性。
- **CH32H415**:移植程度**非常高**,几乎逐函数复刻 STM32 F4 的实现并复用 `common/stm32/` 公共层,
  寄存器名仅做机械替换(WCH SPL 与 STM32 SPL 同名)。亮点:8 路 UART 全引脚表、完整定时器表、
  Bitbang DShot。遗留:TODO 注释(如 UART8 借用)、校准代码注释、`USE_DSHOT_DMAR` 关闭待测。
- **ESP32**:**基础设施级 bring-up**,尽量复用上层但底层换用 ESP-IDF LL + 专属外设(LEDC/RMT/GDMA/
  USB-JTAG)。大量功能因无定时器/无 DMA 而关闭(telemetry、bitbang、Spektrum bind、flash 芯片等),
  多处桩实现(telemetry wait、dmaGetDataLength 等)。目标文件里用 `#undef` 显式声明未支持特性。

---

## 附:分析涉及的关键文件清单

```
src/platform/STM32/serial_uart_ll.c          src/platform/CH32/serial_uart_ch32bsp.c / _ch32h41x.c
src/platform/STM32/bus_i2c_ll.c              src/platform/CH32/bus_i2c_ch32h41x.c
src/platform/STM32/bus_spi_ll.c              src/platform/CH32/bus_spi_ch32h41x.c
src/platform/STM32/dma_stm32h7xx.c           src/platform/CH32/dma_ch32h41x.c
src/platform/STM32/timer_hal2.c              src/platform/CH32/timer_ch32bsp.c / timer_ch32h41x.c
src/platform/STM32/pwm_output_dshot.c        src/platform/CH32/pwm_output_dshot.c
src/platform/STM32/dshot_bitbang_ll.c        src/platform/CH32/dshot_bitbang.c / dshot_bitbang_stdperiph.c
src/platform/STM32/pwm_output_hw.c           src/platform/CH32/pwm_output_ch32.c
src/platform/common/stm32/dshot_dpwm.c       src/platform/common/stm32/pwm_output_dshot_shared.c
src/platform/ESP32/serial_uart_esp32.c       src/platform/ESP32/bus_i2c_esp32.c
src/platform/ESP32/bus_spi_esp32.c           src/platform/ESP32/dma_esp32.c
src/platform/ESP32/timer_esp32.c             src/platform/ESP32/dshot_esp32.c
src/platform/ESP32/pwm_motor_esp32.c         src/platform/ESP32/pwm_servo_esp32.c
src/platform/ESP32/adc_esp32.c               src/platform/ESP32/light_ws2811strip_esp32.c
src/platform/ESP32/exti_esp32.c              src/platform/ESP32/interrupt_esp32.c
src/platform/CH32/mk/CH32H4.mk              src/platform/ESP32/mk/ESP32S3.mk
src/platform/CH32/include/platform/platform.h src/platform/ESP32/target/ESP32S3/target.h
```

---

## 13. 重点比对:CH32 vs STM32 —— 哪些地方不同,能否直接搬?

> 方法:对 `src/platform/CH32/*.c` 与 STM32 对应文件逐对做行级比对
> (`difflib.SequenceMatcher`,剥离 license 头后计算相似度),再提取标识符差异归纳差异类型。

### 13.1 相似度量化结果

| CH32 文件 | 对比的 STM32 文件 | 行相似度 | 判定 |
|---|---|---|---|
| `light_ws2811strip_ch32h41x.c` | `light_ws2811strip_stdperiph.c` | **90.9%** | 可直接搬 |
| `dshot_bitbang.c` | `dshot_bitbang.c` | **90.7%** | 可直接搬 |
| `pwm_output_ch32.c` | `pwm_output_hw.c` | **88.4%** | 可直接搬 |
| `timer_ch32bsp.c` | `timer_stdperiph.c` | **84.4%** | 可直接搬 |
| `bus_spi_ch32h41x.c` | `bus_spi_stdperiph.c` | **80.3%** | 大部分直接搬 |
| `bus_i2c_ch32h41x.c` | `bus_i2c_stm32f4xx.c` | **73.7%** | 大部分可搬,改 GPIO 宏/寄存器名 |
| `pwm_output_dshot.c` | `pwm_output_dshot.c` | **71.7%** | 大部分可搬 |
| `camera_control_ch32.c` | `camera_control_stm32.c` | **64.4%** | 可搬,软 PWM 部分重写 |
| `dshot_bitbang_stdperiph.c` | `dshot_bitbang_stdperiph.c` | **57.4%** | 框架可搬,GPIO/DMA 底层重写 |
| `dma_ch32h41x.c` | `dma_stm32f4xx.c` | **54.1%** | 结构差异大(stream 变 channel) |
| `serial_uart_ch32bsp.c` | `serial_uart_stdperiph.c` | **51.2%** | 逻辑可搬,DMA 段需重写 |
| `exti_ch32.c` | `exti.c` | **44.6%** | STM32 新版 exti 是寄存器版,需重写 |
| `serial_uart_ch32h41x.c` | `serial_uart_stm32f4xx.c` | **32.4%** | 硬件表,必须重写 |
| `io_ch32.c` | `io_stm32.c` | **21.9%** | GPIO 底层,必须重写 |
| `adc_ch32h41x.c` | `adc_stm32f4xx.c` | **16.1%** | ADC_Common 结构不同,必须重写 |
| `timer_ch32h41x.c` | `timer_stm32f4xx.c` | **15.4%** | 硬件表(引脚定义),必须重写 |

> 注:`*_ch32h41x.c` / `*_stm32f4xx.c` 这类文件名带芯片后缀的是**引脚硬件表**
> (uart/timer 的引脚 + AF + IRQ 映射),相似度低是正常的——芯片引脚不同本就该重写,
> 不代表移植难度高。

### 13.2 差异的 6 类本质(按"改动成本"从低到高)

**(1) 寄存器字段名机械替换(最常见、最便宜)**
WCH 标准外设库与 STM32 SPL API 同名,但寄存器/位段命名不同,一一对应:

| STM32 | CH32(WCH) |
|---|---|
| `CR1` / `CR2` | `CTLR1` / `CTLR2` |
| `SR` | `STATR` |
| `DR` | `DATAR` |
| `ARR` | `ATRLR` |
| `CCR1/2/3/4` | `CH1CVR/CH2CVR/CH3CVR/CH4CVR` |
| `DIER` | `DMAINTENR` |
| `EGR` / `SR`(TIM) | `SWEVGR` / `INTFR` |
| `GPIO_BSRR` | `GPIO_BSHR` |
| `GPIO_MODER` | `GPIO_CFGLR/CFGHR` |
| `DMA_CNDTR` | 通过 `xDMA_GetCurrDataCounter()` 访问 |
| `DMA_MemoryBaseAddr`(Init 结构) | 直接写寄存器 `MADDR` |
| `SPI_CR1_BR_Pos` | `SPI_CTLR1_BR_Pos` |
| I2C `SR1/SR2` 位段(BTF/ADDR/AF...) | `STAR1/STAR2` 同名位段 |

这类差异可用**查找-替换**批量完成,属于"搬过去后机械替换"的范畴。


**(2) GPIO 配置宏体系不同(中等成本)**
- STM32 SPL:`IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP)`
- CH32:`IO_CONFIG(DIR_OUT, GPIO_MODE_OUT_AF_PP, GPIO_SPEED_VERY_HIGH, GPIO_PULL_UP)`
  —— **参数含义/顺序完全不同**(第 1 参数是方向 vs 模式),不能整体替换;
  但 CH32 `platform.h` 已定义全套 `IOCFG_*` 快捷宏(`IOCFG_AF_PP/IOCFG_IPU/IOCFG_I2C_PU` 等),
  移植时把 `IO_CONFIG(...)` 替换成对应 `IOCFG_*` 即可。
- GPIO AF 编号不同:STM32 `GPIO_AF9_I2C2` / `GPIO_AF4_USART1` 与 CH32 的裸数字
  `GPIO_AF4/GPIO_AF7/GPIO_AF14` 形式不同,硬件表里必须按 CH32H415 数据手册逐个核对。

**(3) 中断 / 优先级 API 差异(低成本)**
- STM32 旧 SPL:`NVIC_Init(&NVIC_InitStructure)`(基/子优先级结构体);
  STM32 新 LL/HAL:`HAL_NVIC_SetPriority(irq, base, sub)` + `HAL_NVIC_EnableIRQ()`。
- CH32:直接 `NVIC_SetPriority(irq, 单一优先级)` + `NVIC_EnableIRQ(irq)`(CV 风格),
  优先级分组宏为 `NVIC_PRIORITY_GROUPING`(platform.h 已适配)。
- 中断处理函数要加 **`__FAST_INTERRUPT`**(RISC-V WCH 快速中断属性,硬件自动压栈),
  这是 STM32 完全没有的概念;ISR 里被调函数需 `FAST_CODE`/`FAST_IRQ_HANDLER` 修饰(已定义)。
- DMA 中断 handler 命名:`DEFINE_DMA_IRQ_HANDLER(1, 0, DMA1_ST0_HANDLER)`(stream)
  → `DEFINE_DMA_IRQ_HANDLER(1, 1, DMA1_CH1_HANDLER)`(channel),宏参数含义不同。

**(4) DMA 硬件结构差异(中高成本)**
- STM32 F4:8 条 **Stream**,每条 Stream 下有 8 个 Channel(共享向量);
- CH32H415:8 条独立 **Channel**(每通道独立中断),且带 **DMAMUX**
  (需要 `dmaMuxEnable()` + `DMA_MuxChannelConfig()`——这点更像 STM32 G4/H7,不像 F4);
- `DMA_InitTypeDef` 字段不同:STM32 F4 有 `DMA_FIFOMode/DMA_FIFOThreshold/DMA_MemoryBurst` 等,
  CH32 没有 FIFO 概念;`DMA_DIR` 常量名也不同(`MemoryToPeripheral` vs `PeripheralDST`);
- 因此 `dma_reqmap_mcu.c`(外设→DMAMUX 请求号映射表)必须按 CH32 的
  `DMAMUX_DMAREQ_ID_*` 重写(见 CH32 `dma_reqmap_mcu.c` 中 `REQMAP_DIR` 宏)。

**(5) 时钟树抽象(低成本)**
- STM32:`RCC_AHB1PeriphClockCmd(...)`、`RCC_APB1PeriphClockCmd(...)` 按总线直呼;
- CH32:统一抽象为 `RCC_ClockCmd(rccPeriphTag_t, ENABLE)` + tag 宏
  `RCC_HB1(USART1)/RCC_HB2(TIM1)/RCC_HB(DMA1)`(在 `rcc_ch32.c` 中 switch 解析到
  `RCC->HB1PCENR/HB2PCENR/HBPCENR`,复位走 `*PRSTR` 寄存器)。
- 移植时把 `RCC_xxxPeriphClockCmd(x, ENABLE)` 换成 `RCC_ClockCmd(RCC_xx(x), ENABLE)`。

**(6) 内存属性 / Cache(低成本,反而更简单)**
- STM32 F4/F7/H7 驱动里大量 `IS_DTCM()/IS_CCM()` 检查 + `SCB_CleanDCache_by_Addr()`:
  CH32H415 **无 Cache、无 DTCM/CCM 限制**,这些分支全部删掉即可
  (现有 CH32 代码正是这么做的,`IS_CCM` 宏被整行注释)。
- 对齐要求保留:`DMA_DATA / DMA_RAM` 仍定义为 32 字节对齐。


### 13.3 结论:可以直接搬的比例与条件

**总体判断:CH32 与 STM32(F4/SPL 系)的驱动有 70%~90% 可以直接搬。**
原因:WCH 标准外设库本身就是 STM32 SPL 的"克隆"(同名 API、同结构体、同流程),
当前 CH32 移植代码绝大多数文件就是 STM32 对应文件的**逐行复刻**,差异集中在
"寄存器字段名 + GPIO 宏 + 中断注册 + 硬件表"四类。

| 可直接搬(>80%) | 需针对性修改(60~80%) | 框架可搬、底层重写(<60%) |
|---|---|---|
| WS2811 LED | I2C(改 GPIO 宏/寄存器名) | DMA(stream→channel+DMAMUX) |
| DShot bitbang 主框架 | DShot DMA(寄存器名/DMA 方向常量) | UART bsp 驱动(DMA 段重写) |
| 标准 PWM 输出 | Camera control | exti(改寄存器版/分组中断) |
| 通用定时器框架 | | ADC(ADC_Common 结构不同) |
| SPI | | GPIO/IO 层 |
| | | 所有引脚硬件表 |

**"直接搬"的正确姿势(建议流程):**
1. 以 **STM32 F4/StdPeriph 版本**为蓝本(不要用最新的 LL 版,CH32 是 SPL 系);
2. 复制文件后批量做 13.2 中(1)(3)(5)的机械替换(寄存器名 / NVIC API / RCC);
3. 把 `IO_CONFIG(...)` 换成 CH32 `IOCFG_*` 快捷宏(2);
4. 硬件表(引脚+AF+IRQ+DMAMUX 请求号)按 CH32H415 数据手册重写(不可搬);
5. DMA 相关照 G4/H7 的 DMAMUX 思路改(4);
6. 删掉所有 Cache/DTCM/CCM 分支(6);
7. 编译后对照 CH32 现有实现(`serial_uart_ch32bsp.c`、`bus_spi_ch32h41x.c` 等)核对。

### 13.4 特别提醒:common/stm32 公共层是"零成本复用"的典范
- CH32 直接复用 `src/platform/common/stm32/` 里大量公共文件
  (`serial_uart_hw.c`、`bus_spi_hw.c`、`bus_spi_pinconfig.c`、`dshot_dpwm.c`、
  `pwm_output_dshot_shared.c`、`dshot_bitbang_shared.c`、`system.c`、`config_flash.c` 等),
  通过 `USE_CHBSP_DRIVER` 宏在公共层里做**最小切换**——例如 `bus_spi_hw.c` 中:
  ```c
  #if defined(USE_ATBSP_DRIVER) || defined(USE_CHBSP_DRIVER)
  dmaMuxEnable(dmaTxIdentifier, dmaTxChannelSpec->dmaMuxId);   // CH32 需手动配 DMAMUX
  #endif
  ```
  这证明:**同一份 STM32 公共驱动,只要在关键 API 处插入平台条件编译,就能同时服务多平台**,
  这正是 CH32 移植能保持高层逻辑零改动的原因。
- 反过来说,将来想搬**新功能**(如新传感器、新外设),只要该驱动在 STM32 F4 上存在,
  大概率可以在 1~2 天内以"复制 + 13.2 的机械替换 + 重写硬件表"的方式落到 CH32。

---

## 附 2:本章对比分析使用的命令(可复现)

```bash
# 1) 相似度(剥离 license 后逐行比对)
python3 -c "
import difflib,os
def rd(p): return open(p,encoding='utf-8').read().splitlines()
def sl(l):
    i=0
    while i<len(l):
        s=l[i].strip()
        if s.startswith('/*') or s.startswith('*') or s=='': i+=1
        else: break
    return l[i:]
c=sl(rd('src/platform/CH32/bus_spi_ch32h41x.c'))
s=sl(rd('src/platform/STM32/bus_spi_stdperiph.c'))
sm=difflib.SequenceMatcher(None,c,s,autojunk=False)
same=sum(b.size for b in sm.get_matching_blocks())
print('%.1f%%' % (100.0*same/max(len(c),len(s))))
"

# 2) 提取两文件各自的独有标识符(函数/寄存器/宏名),定位"非机械差异"
#    见 13.2 的 6 类差异归纳
```


