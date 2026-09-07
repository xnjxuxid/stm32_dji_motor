# 如果用 CubeMX 配置本工程（对照表）

> 本仓库**不包含 .ioc 文件**——外设初始化是手写的（与任务一/任务二工程同一做法），
> 好处是不用重新生成 CubeMX 工程、不会被 CubeMX 覆盖自己写的代码。
> 本文档说明：**如果要在 CubeMX 里配出完全等价的工程，该怎么点**。
> 手写代码位置 → CubeMX 配置项 一一对应。

---

## 1. 芯片与工程

| 项 | 设置 |
|---|---|
| MCU | **STM32F405RGT6**（LQFP64，1024KB Flash / 192KB RAM） |
| Toolchain | MDK-ARM V5 |
| 代码生成 | 勾选 "Generate peripheral initialization as a pair of .c/.h files" |

## 2. SYS（调试口）

| 项 | 设置 | 说明 |
|---|---|---|
| Debug | **Serial Wire** | PA13=SWDIO、PA14=SWCLK |
| Timebase Source | TIM1（因为用了 FreeRTOS，SysTick 要留给 RTOS） | 对应 `stm32f4xx_hal_timebase_tim.c` |

## 3. RCC / 时钟树

| 项 | 设置 |
|---|---|
| High Speed Clock (HSE) | Crystal/Ceramic Resonator（8MHz） |
| PLL Source | HSE |
| PLLM / PLLN / PLLP | 8 / 336 / 2 |
| SYSCLK | **168 MHz** |
| APB1 (PCLK1) | **42 MHz**（CAN 挂在这里） |
| APB2 (PCLK2) | 84 MHz |

> 手写位置：`Src/main.c` 的 `SystemClock_Config()`
> 注意：手写版做了 **HSE 失败自动回退 HSI 16MHz**（同样配到 168MHz），
> 板子没有 8M 晶振也能跑，CubeMX 版本没有这个回退。

## 4. CAN1

| 项 | 设置 |
|---|---|
| 引脚 | **PA11 = CAN1_RX，PA12 = CAN1_TX**（AF9） |
| Mode | Normal（自检时可改 Loopback） |
| Prescaler | **3** |
| Time Seg1 (BS1) | **10 TQ** |
| Time Seg2 (BS2) | **3 TQ** |
| SJW | 1 TQ |
| 波特率 | 42MHz / 3 / 14 = **1 Mbps**（GM6020 规定） |
| NVIC | CAN1 RX0 interrupt **使能**，抢占优先级 **5** |
| 过滤器 | 手写（CubeMX 不生成）：**掩码模式**，ID=0x200，Mask=0x7F0，收 0x200~0x20F |

> 手写位置：`Src/can.c` 的 `MX_CAN1_Init()`
> 区别：手写版的分频由 `HAL_RCC_GetPCLK1Freq()` **自动计算**，换时钟配置也不会错。

## 5. UART5（VOFA / 调参命令）

| 项 | 设置 |
|---|---|
| 引脚 | **PC12 = UART5_TX，PD2 = UART5_RX**（AF8） |
| Baud | **115200**，8N1 |
| DMA RX | **DMA1 Stream0，Channel 4，Circular**，优先级 Low |
| DMA TX | **DMA1 Stream7，Channel 4，Normal** |
| NVIC | UART5 全局中断 **使能**，优先级 5 |

> 手写位置：`Src/usart.c`（串口+引脚+NVIC）、`Src/dma.c`（DMA 句柄）、`Src/uart_vofa.c`（空闲中断+DMA 接收）

## 6. FreeRTOS

| 项 | 设置 |
|---|---|
| Interface | **CMSIS_V2** |
| configUSE_TASK_NOTIFICATIONS | Enabled |
| 任务 | 代码里用 `xTaskCreate` 创建（motor / log / cmd），CubeMX 可不建任务 |
| Heap | heap_4 |

> 手写位置：`Src/freertos.c`（`MX_FREERTOS_Init` 里调用 `App_Tasks_Create()`）、`Src/app_tasks.c`

## 7. 不需要配置的外设

TIM1（时基由 HAL 自动用）、USART1、SPI、I2C —— 本阶段都没用到。

## 8. 生成后还要做的事（CubeMX 版本）

1. `usart.c` 里加 `fputc` 重定向（或用 `syscalls.c`）
2. `stm32f4xx_it.c` 的 `UART5_IRQHandler` 里调用 `UART_RxIdleCallback(&huart5)`
3. **不要**重复定义 `SVC_Handler` / `PendSV_Handler` / `SysTick_Handler`（FreeRTOS 已提供，重复会报 multiply defined）

---

## 结论

现在的工程**不需要 CubeMX**，直接打开 `MDK-ARM/gimbal.uvprojx` 编译下载即可。
这份对照表的用途是：① 验收讲解时说明"每个配置对应什么"；② 想换成 CubeMX 管理时照着配。
