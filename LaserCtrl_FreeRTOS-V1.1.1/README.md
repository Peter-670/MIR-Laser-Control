# LaserCtrl_FreeRTOS V1.1.1

> 基于 STM32F103RCT6 + FreeRTOS 的三通道激光驱动器固件，支持 UART 指令控制、温度安全门控、ADC 实时反馈。

---

## 目录

- [项目概述](#项目概述)
- [系统架构设计](#系统架构设计)
- [硬件配置详解](#硬件配置详解)
- [软件配置详解](#软件配置详解)
- [核心算法与逻辑](#核心算法与逻辑)
- [通信协议](#通信协议)
- [编译与部署](#编译与部署)
- [使用说明](#使用说明)
- [自定义修改指南](#自定义修改指南)
- [文件结构说明](#文件结构说明)
- [实用附录](#实用附录)

---

## 项目概述

### 一句话说清

通过 UART 发送单字符 `'0'`-`'9'` 命令，控制 3 路激光器的输出功率，具备温度传感器安全门控和 ADC 电压/电流实时反馈。

### 核心功能

| 功能 | 说明 |
|------|------|
| 3 通道激光控制 | Laser1/2 由 STM32 内部 DAC 驱动，Laser3 由外部 MCP4725 I2C DAC 驱动 |
| 10 档命令调节 | 每个通道支持大步进/小步进双向调节 |
| 温度安全门控 | 三级级联温度保护：Laser2 依赖 Laser1 温度正常，Laser3 依赖全部三路正常 |
| ADC 实时反馈 | 4 通道扫描模式采集 DAC 输出电压/电流，每次命令后回传计算结果 |
| RTOS 任务架构 | 6 个 FreeRTOS 任务分工协作，队列+信号量+互斥锁解耦 |
| 安全启动序列 | 上电后所有 DAC 输出归零，温度检测通过才释放全局就绪标志 |

### 技术栈

| 层级 | 技术 |
|------|------|
| MCU | STM32F103RCT6 (Cortex-M3, 256KB Flash, 48KB SRAM) |
| RTOS | FreeRTOS Kernel V10.3.1 (CMSIS-RTOS V2 API) |
| HAL | STM32Cube FW_F1 V1.8.7 |
| 工具链 | Keil MDK-ARM V5.32 / ARMCC V5.06 update 7 |
| 代码生成 | STM32CubeMX 6.17.0 |

---

## 系统架构设计

### 整体结构

```
                    ┌──────────────────────────────────────┐
                    │             USART2 (115200)           │
                    │   TX ──────────────────────> PC端     │
                    │   RX <────────────────────── PC端     │
                    └──────────┬───────────────┬───────────┘
                               │               │
                    ┌──────────▼──┐     ┌──────▼──────────┐
                    │ UartRxTask  │     │   UartTxTask     │
                    │ (轮询接收)   │     │  (日志转发)      │
                    └──────┬──────┘     └──────▲──────────┘
                           │ uartCmdQueue       │ uartTxQueue
                    ┌──────▼────────────────────┴──────────┐
                    │           CommandTask                │
                    │  (命令解析 → 门控校验 → DAC输出      │
                    │   → ADC采样 → 计算 → 日志)           │
                    └──┬──────────┬──────────┬────────────┘
                       │          │          │
              adcReqSem│  i2cMutex│  laserStateMutex
                    ┌──▼──┐  ┌───▼───┐  ┌──▼───────────┐
                    │AdcTask│ │MCP4725│  │TempMonitorTask│
                    │(DMA)  │ │(I2C)  │  │(200ms周期)    │
                    └──┬───▲┘ └───────┘  └──────────────┘
                       │   │
                  DMA1_Ch1 └── adcCpltSem (DMA完成回调)
                  
              ┌────────────────────────┐
              │     defaultTask        │
              │  (安全初始化 → 释放     │
              │   APP_FLAG_READY →      │
              │   心跳 8s)              │
              └────────────────────────┘
```

### 设计思路

**为什么用 6 个任务而不是一个大循环？**

1. **UART 收发分离** — 接收和发送互不阻塞。发送队列满时不影响接收，反之亦然。
2. **命令处理独立** — CommandTask 有最高优先级（AboveNormal），收到命令立即响应，不被日志输出阻塞。
3. **ADC 采样剥离** — ADC 涉及 DMA 启停和信号量等待，独立成 AdcTask 避免 CommandTask 被长时间占用。
4. **温度监控解耦** — 每 200ms 独立扫描，与命令处理互不干扰，通过 `laserStateMutex` 保护共享状态。
5. **安全初始化独立** — defaultTask 最先运行，完成硬件安全初始化后释放 `APP_FLAG_READY`，其余 5 个任务全部阻塞等待此标志。
6. **日志异步输出** — 所有 Log_Printf 只是入队，UartTxTask 从队列取消息发出，不拖慢调用者。

**关键设计决策：**

- **TIM4 代替 SysTick 做 HAL 时基** — FreeRTOS 占用 SysTick 后，HAL 需要独立的 1ms 时基源。TIM4 挂在 APB1（36MHz→72MHz），不受 SysTick 干扰。
- **heap_4.c** — 支持内存碎片合并，比 heap_1/2/3 更灵活，适配动态创建 RTOS 对象。
- **PendSV 优先级 15** — 最低中断优先级，确保 RTOS 上下文切换不抢占任何硬件中断。
- **DMA 完成回调只发信号量** — `HAL_ADC_ConvCpltCallback` 只做 `osSemaphoreRelease(adcCpltSem)`，数据处理全在 AdcTask 任务上下文完成，ISR 极短。

---

## 硬件配置详解

### MCU 引脚分配

| 引脚 | 功能 | 模式 | 用途 |
|------|------|------|------|
| PA0 | ADC1_IN0 | Analog | Laser1 反馈电压采样 |
| PA1 | ADC1_IN1 | Analog | Laser2 反馈电压采样 |
| PA2 | USART2_TX | AF Push-Pull | 调试串口发送 (115200) |
| PA3 | USART2_RX | Input | 调试串口接收 |
| PA4 | DAC_OUT1 | Analog | Laser1 控制电压输出 |
| PA5 | DAC_OUT2 | Analog | Laser2 控制电压输出 |
| PA6 | ADC1_IN6 | Analog | Laser3 反馈电压采样 (V) |
| PA7 | ADC1_IN7 | Analog | Laser3 反馈电流采样 (I) |
| PA13 | SWDIO | SW Debug | 调试接口（保留） |
| PA14 | SWCLK | SW Debug | 调试接口（保留） |
| PA15 | GPIO_Output | PP + Pull-down | Laser2 使能控制 |
| PB6 | I2C1_SCL | AF Open-Drain | MCP4725 DAC 时钟 |
| PB7 | I2C1_SDA | AF Open-Drain | MCP4725 DAC 数据 |
| PB12 | GPIO_Output | PP + Pull-down | Laser1 使能控制 |
| PB13 | GPIO_Input | Pull-down | 温度传感器 1（Laser1） |
| PB14 | GPIO_Input | Pull-down | 温度传感器 2（Laser2） |
| PB15 | GPIO_Input | Pull-down | 预留 / NC |
| PC4 | GPIO_Output | PP + Pull-down | Laser3 使能控制 |
| PD0 | OSC_IN | HSE | 8 MHz 外部晶振 |
| PD1 | OSC_OUT | HSE | 8 MHz 外部晶振 |

### 时钟树

```
    HSE 8MHz ──> PLL (×9) ──> SYSCLK 72MHz
                                 │
                    ┌────────────┼────────────┐
                    │            │            │
                  HCLK 72MHz   APB1 36MHz   APB2 72MHz
                  (AHB 总线)   (低速外设)   (高速外设)
                    │            │            │
                    │         TIM4 (×2)    ADC1 (/6)
                    │         72MHz        12MHz
                    │
                 GPIO / DMA1
```

### 外设参数

| 外设 | 参数 | 说明 |
|------|------|------|
| **ADC1** | 4 通道扫描 + DMA 循环 | CH0→CH1→CH6→CH7，采样时间 55.5 cycles |
| **DAC** | 双通道 OUT1/OUT2 | 12 位右对齐，无触发，输出缓冲开启 |
| **I2C1** | 100 kHz 标准模式 | 7 位地址，连接 MCP4725 (0x62) |
| **USART2** | 115200-8-N-1 | 异步收发，无流控 |
| **DMA1_CH1** | 外设→内存，半字对齐 | 循环模式，配合 ADC 连续扫描 |
| **TIM4** | 1 kHz HAL 时基 | 代替 SysTick，预分频自动计算 |

### 激光通道硬件表

| 属性 | Laser1 | Laser2 | Laser3 |
|------|--------|--------|--------|
| DAC 方式 | STM32 DAC OUT1 | STM32 DAC OUT2 | MCP4725 I2C DAC |
| DAC 分辨率 | 12-bit (0-4095) | 12-bit (0-4095) | 12-bit (0-4095) |
| 软件最大 DAC | 3103 | 3103 | 1861 |
| 控制引脚 | PA4 | PA5 | PB6/PB7 (I2C) |
| 使能引脚 | PB12 | PA15 | PC4 |
| 反馈 ADC | CH0 (PA0) | CH1 (PA1) | CH6-V / CH7-I (PA6/PA7) |
| 温度传感器 | PB13 | PB14 | 软件固定就绪 |

---

## 软件配置详解

### FreeRTOS 核心参数

| 参数 | 值 | 说明 |
|------|------|------|
| `configUSE_PREEMPTION` | 1 | 抢占式调度 |
| `configTICK_RATE_HZ` | 1000 | 1ms 系统节拍 |
| `configMAX_PRIORITIES` | 56 | 可用优先级数 |
| `configTOTAL_HEAP_SIZE` | 20000 | 堆大小 20KB |
| `configMINIMAL_STACK_SIZE` | 128 | 最小任务栈（字） |
| `configUSE_MUTEXES` | 1 | 互斥锁（含优先级继承） |
| `configCHECK_FOR_STACK_OVERFLOW` | 2 | 栈溢出检测（方法2） |
| `configUSE_MALLOC_FAILED_HOOK` | 1 | 内存分配失败回调 |
| `USE_FreeRTOS_HEAP_4` | heap_4.c | 最佳匹配+碎片合并 |

### 任务配置

| 任务 | CMSIS 优先级 | 栈大小 | 职责 |
|------|-------------|--------|------|
| `defaultTask` | Low | 1024 B | 安全初始化 + 8s 心跳 |
| `UartRxTask` | Normal | 1024 B | 轮询 USART2 接收 |
| `CommandTask` | AboveNormal | 2048 B | 命令处理（最高优先级） |
| `TempMonitorTask` | Normal | 1024 B | 200ms 温度扫描 |
| `UartTxTask` | Low | 2048 B | 异步日志输出 |
| `AdcTask` | Normal | 1024 B | ADC DMA 采样协调 |

### RTOS 同步对象

| 对象 | 类型 | 数量/容量 | 用途 |
|------|------|----------|------|
| `uartCmdQueue` | 消息队列 | 16 × uint8_t | UartRxTask → CommandTask |
| `uartTxQueue` | 消息队列 | 12 × UartLogMsg_t (160B) | 全局 → UartTxTask |
| `i2cMutex` | 互斥锁 | 1 | 串行化 I2C 总线访问 |
| `laserStateMutex` | 互斥锁 | 1 | 保护 `g_laser` 结构体 |
| `adcDataMutex` | 互斥锁 | 1 | 保护 `g_adc` ADC 结果 |
| `adcReqSem` | 二值信号量 | 1 | CommandTask 触发 ADC 采样 |
| `adcDoneSem` | 二值信号量 | 1 | AdcTask 通知采样完成 |
| `adcCpltSem` | 二值信号量 | 1 | DMA ISR 通知 DMA 完成 |
| `appFlags` | 事件标志 | 1 bit | 全局就绪标志 `APP_FLAG_READY` |

### 全局状态结构

```c
// 激光全局状态（受 laserStateMutex 保护）
typedef struct {
    uint32_t dac1;        // Laser1 当前 DAC 值 (0-3103)
    uint32_t dac2;        // Laser2 当前 DAC 值 (0-3103)
    uint16_t dac3;        // Laser3 当前 DAC 值 (0-1861)
    uint8_t  temp1_ready; // 温度传感器 1 状态
    uint8_t  temp2_ready; // 温度传感器 2 状态
    uint8_t  temp3_ready; // 温度传感器 3 状态（固定 1）
    uint8_t  en1;         // Laser1 使能状态
    uint8_t  en2;         // Laser2 使能状态
    uint8_t  en3;         // Laser3 使能状态
} LaserState_t;

// ADC 采样快照（受 adcDataMutex 保护）
typedef struct {
    uint16_t raw[4];      // CH0, CH1, CH6, CH7 原始值
} AdcSnapshot_t;
```

---

## 核心算法与逻辑

### 1. 安全启动序列

```
上电 → HAL_Init → 时钟配置 → 外设初始化 → MX_FREERTOS_Init（创建所有RTOS对象）
  → osKernelStart → 调度器启动
  → defaultTask (最高优先级启动任务) 执行 Hardware_InitSafe():
      ① ADC1 校准
      ② DAC CH1/CH2 启动，输出设为 0
      ③ MCP4725 输出设为 0（I2C 互斥锁保护）
      ④ 读取温度传感器 GPIO，更新使能状态
      ⑤ 设置 APP_FLAG_READY 事件标志
  → 其余 5 个任务被唤醒，系统就绪
```

### 2. 温度级联门控

```
Laser1 使能条件: temp1_ready
Laser2 使能条件: temp1_ready AND temp2_ready
Laser3 使能条件: temp1_ready AND temp2_ready AND temp3_ready

命令过滤:
  '0'-'3' (Laser1 命令): 要求 temp1_ready
  '4'-'7' (Laser2 命令): 要求 temp1_ready AND temp2_ready
  '8'-'9' (Laser3 命令): 要求全部三路就绪
```

门控逻辑确保：下游激光器永远不能在上游温度异常时独立工作，形成硬件安全链。

### 3. DAC 饱和加法

```c
#define SAT_ADD_U32(v, step, hi) (((v) >= ((hi) - (step))) ? (hi) : ((v) + (step)))
```

防止加法溢出：若当前值加步进超过上限，直接返回上限值，不翻转。

### 4. Laser3 ADC 双量程映射

Laser3 有两路 ADC 反馈（电压 + 电流），各自使用独立的线性映射：

```
电压通道 (CH6): ADC [0, 3.26V] → 实际 [2.0V, 3.3V]
电流通道 (CH7): ADC [0, 2.71V] → 实际 [0.8A, 10.0A]

公式: 实际值 = MIN + (ADC电压 - ADC_MIN) × (MAX - MIN) / (ADC_MAX - ADC_MIN)
```

### 5. 命令处理流程

```
UartRxTask 收到字符 '0'-'9'
  → 入队 uartCmdQueue
  → CommandTask 取出
  → 获取 laserStateMutex
  → Laser_CommandAllowedLocked() 校验温度门控
  → 若不允许：Log_Printf 拒绝原因，释放锁返回
  → 若允许：更新 g_laser.dacX，释放锁
  → 写入对应 DAC 硬件
  → 延时等待稳定（Laser1/2: 100ms, Laser3: 200ms）
  → Adc_RequestAndWait() 触发 AdcTask 采样
  → 读取 ADC 快照
  → 计算输入/输出 电压/电流
  → Log_Printf 输出结果
```

### 6. ADC 采样信号量握手

```
CommandTask                          AdcTask                    DMA ISR
    │                                   │                          │
    ├─ osSemaphoreRelease(adcReqSem) ──►│                          │
    │                                   ├─ HAL_ADC_Start_DMA()     │
    │                                   │                          │
    │                                   ├─ osSemaphoreAcquire      │
    │                                   │  (adcCpltSem, 300ms)     │
    │                                   │                          │
    │                                   │               ADC完成 ──►│
    │                                   │          HAL_ADC_ConvCplt│
    │                                   │          Callback()      │
    │                                   │     osSemaphoreRelease()◄│
    │                                   │     (adcCpltSem)         │
    │                                   │                          │
    │                                   ├─ HAL_ADC_Stop_DMA()      │
    │                                   ├─ memcpy g_adc            │
    │                                   ├─ osSemaphoreRelease      │
    │                                   │  (adcDoneSem) ──────────►│
    │◄── osSemaphoreAcquire 返回 ──────┤                          │
    │                                   │                          │
    ├─ 读取 g_adc 快照                  │                          │
```

---

## 通信协议

### 物理层

| 参数 | 值 |
|------|------|
| 接口 | USART2 (PA2-TX, PA3-RX) |
| 波特率 | 115200 |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验 | 无 |
| 流控 | 无 |
| 电平 | 3.3V TTL（需外接 USB-TTL 模块连接 PC） |

### 命令协议

**命令格式：** 单个 ASCII 字符 `'0'` 到 `'9'`，无需回车换行。

| 命令 | 激光通道 | 方向 | 步进类型 | 步进量 | DAC 上限 |
|------|---------|------|---------|--------|---------|
| `0` | Laser1 | - | 大步 | 31 | 0（下限） |
| `1` | Laser1 | + | 大步 | 31 | 3103 |
| `2` | Laser1 | - | 小步 | 3 | 0（下限） |
| `3` | Laser1 | + | 小步 | 3 | 3103 |
| `4` | Laser2 | + | 大步 | 31 | 3103 |
| `5` | Laser2 | - | 大步 | 31 | 0（下限） |
| `6` | Laser2 | + | 小步 | 3 | 3103 |
| `7` | Laser2 | - | 小步 | 3 | 0（下限） |
| `8` | Laser3 | + | — | 20 | 1861 |
| `9` | Laser3 | - | — | 20 | 0（下限） |

### 命令响应格式

**正常响应：**
```
Laser1: DAC=1023, InI=0.824A, InV=0.824V, OutI=0.815A, OutV=0.815V
Laser2: DAC=2046, InI=1.648A, InV=1.648V, OutI=1.640A, OutV=1.640V
Laser3: DAC=500, InI=1.548A, InV=0.403V, OutI=2.150A, OutV=2.850V
```

**拒绝响应（温度未就绪）：**
```
Reject: Laser1 temp not ready
Reject: Laser1/Laser2 not ready
Reject: Laser1/Laser2/Laser3 not ready
```

**系统消息：**
```
Boot done V3: USART2 OK
RTOS multi-task init done
Heartbeat: rx/cmd/temp/adc/tx running
RX cmd=1
```

---

## 编译与部署

### 前置条件

1. **Keil MDK-ARM V5.32+** 安装在 Windows 环境
2. **Keil.STM32F1xx_DFP.2.3.0** 设备支持包（Keil Pack Installer 在线安装）
3. **STM32Cube FW_F1 V1.8.7**（若需重新生成代码，用 CubeMX 6.17.0+ 打开 `.ioc`）
4. **ST-Link / J-Link** 调试器 + SWD 连接线（PA13-SWDIO, PA14-SWCLK, GND, 3.3V）

### 编译步骤

1. 双击 `MDK-ARM\LaserCtrl_FreeRTOS.uvprojx` 打开 Keil 工程
2. 确认 Project → Options → Device 中 MCU 为 `STM32F103RC`
3. 点击 **Build (F7)** 编译
4. 确认输出窗口显示 `0 Error(s), 0 Warning(s)`
5. 产物在 `MDK-ARM\LaserCtrl_FreeRTOS\` 下：
   - `LaserCtrl_FreeRTOS.axf` — 调试文件
   - `LaserCtrl_FreeRTOS.hex` — 烧录文件
   - `LaserCtrl_FreeRTOS.map` — 内存映射

### 烧录步骤

1. 连接 ST-Link 到目标板 SWD 接口
2. Keil → Flash → Download (F8)
3. 或者用 STM32CubeProgrammer 加载 `.hex` 文件烧录

### 注意事项

- **JTAG 已禁用**，仅保留 SWD 调试接口（`__HAL_AFIO_REMAP_SWJ_NOJTAG()`）
- 外部必须接 **8 MHz HSE 晶振**，否则时钟配置失败会进入 `Error_Handler()`
- 首次上电请观察串口输出 `Boot done V3: USART2 OK` 确认系统启动成功
- `keilkill.bat` 可清理编译产物和中间文件

---

## 使用说明

### 典型操作流程

1. 用 USB-TTL 模块连接 PA2(TX)、PA3(RX)、GND 到 PC
2. 打开串口助手，配置 **115200-8-N-1**
3. 目标板上电，观察串口输出：
   ```
   Boot done V3: USART2 OK
   System init done, waiting cmd
   RTOS multi-task init done
   ```
4. 确认温度传感器连接正常后，发送数字命令调节激光功率
5. 每发送一个命令，系统返回当前通道的 DAC 值、输入/输出 电压/电流

### 命令速查

| 目的 | 按键 |
|------|------|
| Laser1 快速增/减 | `1` / `0` |
| Laser1 微调增/减 | `3` / `2` |
| Laser2 快速增/减 | `4` / `5` |
| Laser2 微调增/减 | `6` / `7` |
| Laser3 增/减 | `8` / `9` |

### 常见问题排查

| 现象 | 可能原因 | 解决 |
|------|---------|------|
| 上电无串口输出 | 波特率不匹配 | 确认串口助手 115200-8-N-1 |
| 发送命令无响应 | 命令字节非法 | 只支持 `'0'`-`'9'`，不接回车 |
| 提示 temp not ready | 温度传感器未连接/异常 | 检查 PB13（L1）/PB14（L2）电平 |
| 串口乱码 | 晶振未起振 | 检查 8MHz HSE 晶振焊接 |
| `Error: ADC sample timeout` | DMA 中断被阻塞 | 检查其他 ISR 是否耗时过长 |
| Laser3 无输出 | MCP4725 地址不对 | 确认芯片地址为 0x62（A0 引脚接 GND） |

---

## 自定义修改指南

### 修改 DAC 最大输出

在 [freertos.c](Core/Src/freertos.c) 中修改宏定义：

```c
#define LASER12_DAC_MAX    3103U   // Laser1/2 最大 DAC 值（0-4095）
#define LASER3_DAC_MAX     1861U   // Laser3 最大 DAC 值（0-4095）
```

### 修改调节步进

```c
#define LASER1_BIG_STEP    31U     // Laser1 大步进量
#define LASER1_SMALL_STEP  3U      // Laser1 小步进量
#define LASER2_BIG_STEP    31U     // Laser2 大步进量
#define LASER2_SMALL_STEP  3U      // Laser2 小步进量
#define LASER3_STEP        20U     // Laser3 步进量
```

### 修改稳定延时

```c
#define LASER3_SETTLE_DELAY_MS  200U  // Laser3 DAC 稳定等待时间
```

Laser1/2 的延时代码在 `Laser_ProcessCommand()` 中硬编码为 `osDelay(100U)`，可直接修改。

### 修改 Laser3 ADC 校准参数

```c
#define LASER3_VOLTAGE_MIN      2.0f    // 实际最小电压
#define LASER3_VOLTAGE_MAX      3.3f    // 实际最大电压
#define LASER3_ADC_VOLTAGE_MIN  0.0f    // ADC 对应最小电压
#define LASER3_ADC_VOLTAGE_MAX  3.26f   // ADC 对应最大电压

#define LASER3_CURRENT_MIN      0.8f    // 实际最小电流 (A)
#define LASER3_CURRENT_MAX      10.0f   // 实际最大电流 (A)
#define LASER3_ADC_CURRENT_MIN  0.0f    // ADC 对应最小电压
#define LASER3_ADC_CURRENT_MAX  2.71f   // ADC 对应最大电压
```

### 修改 UART 波特率

在 [main.c](Core/Src/main.c) 的 `SystemClock_Config()` 或 [usart.c](Core/Src/usart.c) 中修改 `huart2.Init.BaudRate`。若通过 CubeMX 生成，在 `.ioc` 中修改 Pinout → USART2 → Baud Rate。

### 添加新的激光通道

1. 在 CubeMX 中配置新的 DAC/ADC/GPIO 引脚
2. 在 `freertos.c` 的 `LaserState_t` 中添加 `dac4`、`en4` 等字段
3. 在 `Laser_ProcessCommand()` 的 `switch` 中添加新命令处理分支
4. 在 `Laser_CommandAllowedLocked()` 中添加温度门控条件
5. 在 `Laser_UpdateEnableLocked()` 中添加使能引脚控制
6. 在 `AdcSnapshot_t` 中扩展 `raw[]` 数组（如需新 ADC 通道）

### 修改 FreeRTOS 堆大小

在 [FreeRTOSConfig.h](Core/Inc/FreeRTOSConfig.h) 中：

```c
#define configTOTAL_HEAP_SIZE  ((size_t)20000)   // 改为你需要的大小
```

### 修改任务栈大小

在 [freertos.c](Core/Src/freertos.c) 的各任务 `osThreadAttr_t` 中修改 `.stack_size`，例如：

```c
const osThreadAttr_t CommandTask_attributes = {
  .name = "CommandTask",
  .stack_size = 1024 * 4,   // 从 512*4 增大到 1024*4
  .priority = (osPriority_t) osPriorityAboveNormal,
};
```

---

## 文件结构说明

```
项目根目录/
│
├── README.md                          ← 本文档
├── LaserCtrl_FreeRTOS.ioc             ← CubeMX 项目配置（引脚/时钟/外设/RTOS）
├── .mxproject                         ← CubeMX 项目元数据（文件列表等）
├── keilkill.bat                       ← 编译产物清理脚本
│
├── Core/                              ← ★ 应用层源代码（手写 + CubeMX 生成）
│   ├── Inc/                           ←   头文件
│   │   ├── main.h                     ←     公共定义、HAL 引用
│   │   ├── FreeRTOSConfig.h           ←     FreeRTOS 内核配置
│   │   ├── adc.h / dac.h / dma.h      ←     外设模块头文件
│   │   ├── gpio.h / i2c.h / usart.h   ←
│   │   ├── mcp4725.h                  ←     MCP4725 驱动接口
│   │   ├── stm32f1xx_hal_conf.h       ←     HAL 模块开关
│   │   └── stm32f1xx_it.h             ←     中断服务函数声明
│   │
│   └── Src/                           ←   源文件
│       ├── main.c                     ←     入口 (main + SystemClock_Config)
│       ├── freertos.c                 ←     ★ 核心应用（6个任务 + 同步对象 + 业务逻辑）
│       ├── mcp4725.c                  ←     MCP4725 I2C DAC 驱动
│       ├── adc.c / dac.c / dma.c      ←     外设初始化
│       ├── gpio.c / i2c.c / usart.c   ←
│       ├── stm32f1xx_it.c             ←     中断服务函数
│       ├── stm32f1xx_hal_msp.c        ←     HAL MSP（引脚下配置/PendSV/JTAG禁用）
│       ├── stm32f1xx_hal_timebase_tim.c ←   TIM4 代替 SysTick 的 HAL 时基
│       └── system_stm32f1xx.c         ←     CMSIS SystemInit/SystemCoreClockUpdate
│
├── Drivers/                           ← STM32Cube SDK（不在备份范围内）
│   ├── CMSIS/                         ←   CMSIS Core + Device
│   └── STM32F1xx_HAL_Driver/          ←   STM32F1 HAL 库
│
├── Middlewares/                        ← FreeRTOS 中间件（不在备份范围内）
│   └── Third_Party/FreeRTOS/          ←   FreeRTOS V10.3.1 + CMSIS-RTOS V2 wrapper
│
├── MDK-ARM/                           ← Keil MDK 工程
│   ├── LaserCtrl_FreeRTOS.uvprojx     ←   Keil 工程文件 ★
│   ├── LaserCtrl_FreeRTOS.uvoptx      ←   Keil 工程选项 ★
│   ├── startup_stm32f103xe.s          ←   启动文件（向量表+栈堆初始化） ★
│   ├── LaserCtrl_FreeRTOS/            ←   编译输出（.o .hex .axf .map，不备份）
│   ├── DebugConfig/                   ←   调试配置（不备份）
│   └── RTE/                           ←   运行时环境（不备份）
│
├── FreeRTOS_项目总结与面试讲解.md      ← 项目技术总结文档
└── FreeRTOS_项目总结与面试讲解.pdf     ← 同上 PDF 版
```

> ★ 标记的文件为备份保留文件。Drivers/ 和 Middlewares/ 属于 STM32Cube SDK 内容，可通过 CubeMX 自动生成或从 ST 官网下载，不在备份范围内。

---

## 实用附录

### A. 命令速查表

| 按键 | 通道 | 动作 | 步进 |
|------|------|------|------|
| `0` | L1 | - 大步 | 31 |
| `1` | L1 | + 大步 | 31 |
| `2` | L1 | - 小步 | 3 |
| `3` | L1 | + 小步 | 3 |
| `4` | L2 | + 大步 | 31 |
| `5` | L2 | - 大步 | 31 |
| `6` | L2 | + 小步 | 3 |
| `7` | L2 | - 小步 | 3 |
| `8` | L3 | + | 20 |
| `9` | L3 | - | 20 |

### B. DAC 值换算公式

```
Laser1/2:
  输出电压 Vout = DAC值 × 3.3V / 4096
  输出电流 Iout = Vout / 2.5Ω

Laser3:
  输入电压 Vin  = DAC值 × 3.3V / 4096
  输入电流 Iin  = Vin × 6.13 + 0.8A
  输出电压 Vout = 2.0V + (ADC_CH6_raw × 3.3V/4096 - 0V) × (3.3V - 2.0V) / (3.26V - 0V)
  输出电流 Iout = 0.8A + (ADC_CH7_raw × 3.3V/4096 - 0V) × (10.0A - 0.8A) / (2.71V - 0V)
```

### C. DAC 常用值速查

| DAC 值 | 输出电压 (V) | 输出电流 (A) | 说明 |
|--------|-------------|-------------|------|
| 0 | 0.000 | 0.000 | 关闭 |
| 500 | 0.403 | 0.161 | 低功率 |
| 1000 | 0.806 | 0.322 | |
| 1500 | 1.209 | 0.484 | |
| 2000 | 1.611 | 0.645 | 中等功率 |
| 2500 | 2.014 | 0.806 | |
| 3000 | 2.417 | 0.967 | |
| 3103 | 2.500 | 1.000 | **Laser1/2 上限** |
| 1861 | 1.499 | 0.600 | **Laser3 上限** |

### D. 温度传感器逻辑电平

| GPIO 状态 | 含义 |
|-----------|------|
| 高电平 (1) | 温度正常，对应通道使能 |
| 低电平 (0) | 温度异常，对应通道禁用 |

传感器默认为下拉（Pull-down），未连接时为低电平 = 异常状态（fail-safe）。

### E. MCP4725 参考

| 属性 | 值 |
|------|------|
| 型号 | MCP4725A0T-E/CH (或兼容型号) |
| 分辨率 | 12-bit |
| I2C 地址 | 0x62 (7-bit), 0xC0 (写), 0xC1 (读) |
| 写模式 | Fast Write (命令字节 0x40) |
| 最大电压 | VDD（通常 3.3V） |
| A0 引脚 | 接 GND（地址 0x62） |

### F. 中断优先级分配

| 中断 | 优先级 (组) | 说明 |
|------|-----------|------|
| PendSV | 15,0 | FreeRTOS 上下文切换（最低） |
| TIM4 | 由 HAL_InitTick 决定 | HAL 1ms 时基 |
| DMA1_Channel1 | 5,0 | ADC DMA 完成中断 |

---

*文档基于 LaserCtrl_FreeRTOS V1.1.1，最后更新 2026-05-01*
