#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#include "stm32f4xx_hal.h"

/* ============================================================================
 * 阶段二：云台板 STM32F405RGT6 + GM6020（CAN 1Mbps）
 * ----------------------------------------------------------------------------
 *  引脚（来自原理图）
 *    CAN1  : PA11 = RX , PA12 = TX      （板载已带 CAN 收发器）
 *    UART5 : PC12 = TX , PD2 = RX       （接 USB-TTL 给 VOFA / 调参命令）
 *    SWD   : PA13 / PA14
 *  时钟
 *    SYSCLK = 168 MHz，APB1(PCLK1) = 42 MHz，APB2 = 84 MHz
 *    优先 HSE 8 MHz；若无外部晶振自动回退 HSI 16 MHz（两者同为 168 MHz）
 *  注意
 *    CAN 分频由 PCLK1 自动计算（不写死），换时钟配置也不用改 CAN 代码
 * ==========================================================================*/

/* =========================== 阶段三新增 =========================== */

/* ---------- 遥控器协议选择（二选一，改这里即可，不用换代码） ----------
 *  0 = iBus   ：USART6_RX(PC7)，115200 8N1 **正逻辑**，J8 座
 *               → 适用：**富斯 iA10B / iA6B 的 i-BUS 口**（i6X 遥控器只能出这个）
 *  1 = DBUS/SBUS：USART2_RX(PA3)，100000 8E2 **反相**（板上 SS8050 已反相）
 *               → 适用：**DJI DR16 接收机（配 DT7 遥控器）**或能出 SBUS 的接收机
 *               → 接板上 P1 座（丝印"下层DBUS，上层舵机"）
 *
 *  ⚠️ 重要事实：富斯 iA10B **只能输出 iBus / PPM / PWM**，
 *     发不出 DBUS，也发不出 SBUS（i6X 菜单里没有 SBUS 选项）；
 *     DBUS 是 DJI DR16 的私有协议，且 DR16 只能配 DJI DT7 遥控器。
 *     所以用 iA10B 时必须选 0（iBus）。 */
#ifndef RC_PROTOCOL
#define RC_PROTOCOL             (1)     /* 0 = iBus(PC7/J8座), 1 = DBUS/SBUS(PA3/P1座) */
#endif

#if (RC_PROTOCOL == 0)
/* ---------- 遥控器：FlySky iBus（iA10B 的 i-BUS 口） ----------
 * 为什么用 iBus：115200 / 8N1 / **正逻辑**（不用反相器、不用奇偶校验），
 *   是所有遥控协议里最简单的一根线方案。
 * 为什么用 USART6(PC7)：
 *   原理图确认 P1 座（RC+Servo）的 DBUS 经 Q1 SS8050 **反相**后接 PA3，
 *   反相后的信号不能收 iBus；板上的 **J8 座（GHS1.25 4P）** 直接是
 *   USART6_RX(PC7) + USART6_TX(PC6) + 5V + GND，无反相且带 3.6V 保护二极管。
 * 帧格式：0x20 0x40 头 + 14 通道×2 字节(小端, 1000~2000us) + 2 字节校验和 = 32 字节 */
#define RC_UART_HANDLE      huart6
#define RC_UART             (&RC_UART_HANDLE)
#define RC_UART_INSTANCE    USART6
#define RC_BAUDRATE         (115200u)
#else
#define RC_UART_HANDLE      huart2
#define RC_UART             (&RC_UART_HANDLE)
#define RC_UART_INSTANCE    USART2
#define RC_BAUDRATE         (100000u)
#endif

/* 遥控保护：超过该时间没收到新帧 → 立即停输出、清 PID 积分（电机"没力"） */
#define RC_LINK_TIMEOUT_MS  (100u)

extern UART_HandleTypeDef huart6;   /* iBus 用（PC7）  */
extern UART_HandleTypeDef huart2;   /* DBUS/SBUS 用（PA3，经 SS8050 反相） */

/* ---------- BMI088（SPI1，原理图确认是 SPI 不是 I2C） ----------
 * SPI1 : PA5 = SCK, PA6 = MISO, PA7 = MOSI
 * CS   : CS_Accel = PC4 , CS_Gyro = PA4（软件片选）
 * INT  : INT_Accel = PB0 , INT_Gyro = PC5（本阶段未用） */
extern SPI_HandleTypeDef  hspi1;

#define BMI088_SPI              (&hspi1)
#define BMI088_CS_ACC_PORT      GPIOC
#define BMI088_CS_ACC_PIN       GPIO_PIN_4
#define BMI088_CS_GYRO_PORT     GPIOA
#define BMI088_CS_GYRO_PIN      GPIO_PIN_4

/* 量程与灵敏度（初始化里配成 ±6g / ±1000dps） */
#define BMI088_ACC_RANGE_G      (6.0f)
#define BMI088_ACC_LSB_PER_G    (5460.0f)    /* ±6g   */
#define BMI088_GYRO_LSB_PER_DPS (32.8f)      /* ±1000dps */

/* ---------- 串口（VOFA + 调参命令） ---------- */
#define VOFA_UART_HANDLE        huart5
#define VOFA_UART               (&VOFA_UART_HANDLE)
#define VOFA_UART_INSTANCE      UART5

extern CAN_HandleTypeDef  hcan1;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef  hdma_uart5_rx;
extern DMA_HandleTypeDef  hdma_uart5_tx;

/* ---------- 时钟源（🔒 锁芯片问题的关键开关） ----------
 * 0 = HSI 内部 16MHz → PLL 到 168MHz（**不依赖任何外部晶振**，绝不会超频）
 * 1 = HSE 外部晶振 → 168MHz（**必须确认板上晶振真的是 8MHz**）
 *
 * ⚠️ 为什么默认改成 HSI：
 *    如果板子实际晶振是 12MHz/25MHz，而代码按 8MHz 算 PLL（HSE 仍能起振、
 *    HAL 不报错），VCO 会被配到 504MHz（上限 432MHz）→ 芯片跑飞 →
 *    调试口失联，表现就是烧完再连报 "Invalid ROM Table"（看起来像锁芯片）。
 *    先用 HSI 跑通，确认板子正常后，再根据晶振丝印改回 HSE。 */
#ifndef CLOCK_USE_HSE
#define CLOCK_USE_HSE           (1)
#endif

/* 板上晶振实际频率（Hz）——来自原理图：位号 X1，标注 "25M/3225"
 * ⚠️ 必须是 25MHz：若错填 8MHz，PLLM 会算成 8 → VCO = 25/8×336 = 1050MHz
 *    （上限 432MHz）→ 芯片跑飞 → 调试口失联（Invalid ROM Table） */
#ifndef BOARD_HSE_HZ
#define BOARD_HSE_HZ            (25000000u)
#endif

/* ---------- CAN ---------- */
#define CAN_BAUD_HZ             (1000000u)   /* GM6020 规定 1 Mbps */
#define CAN_TQ_TOTAL            (14u)        /* 1(SYNC) + BS1(10) + BS2(3) */

/* 自检用：置 1 时 CAN 工作在环回模式（自发自收，不接电机也能验证软件） */
#ifndef CAN_LOOPBACK_TEST
#define CAN_LOOPBACK_TEST       (0)
#endif

/* ---------- GM6020 ---------- */
#define MOTOR_ID                (1u)                    /* 拨码设置的 ID（1~7） */
#define MOTOR_FEEDBACK_ID       (0x204u + MOTOR_ID)     /* 反馈标识符 0x205 */
#define CTRL_ID_1_4             (0x1FFu)                /* 电压控制 ID1~4 */
#define CTRL_ID_5_7             (0x2FFu)                /* 电压控制 ID5~7 */
#define GM6020_ANGLE_MAX_RAW    (8192.0f)               /* 机械角度满量程 */
#define GM6020_VOLT_MAX         (25000.0f)              /* 电压给定上限 */
#define GM6020_CUR_MAX_RAW      (16384.0f)              /* 电流满量程 */
#define GM6020_CUR_MAX_A        (3.0f)                  /* 对应 ±3 A */

/* 安全限幅（上电默认很小，命令 lv 可改；不要一上来就满电压） */
#define MOTOR_VOLT_LIMIT_DEFAULT (6000.0f)

/* 反馈超时保护：超过该时间没收到新反馈 → 输出 0 并清积分 */
#define MOTOR_RX_TIMEOUT_MS      (200u)

/* ---------- 队列 ---------- */
#define CAN_QUEUE_LENGTH        (8u)
#define UART_QUEUE_LENGTH       (3u)
#define UART_RX_BUF_SIZE        (128u)
#define UART_TX_BUF_SIZE        (256u)

/* ---------- 任务周期（vTaskDelayUntil 绝对周期） ---------- */
#define CTRL_TASK_PERIOD_MS     (1u)    /* 控制环：与电机 1kHz 反馈同频 */
#define LOG_TASK_PERIOD_MS      (5u)    /* 曲线打印：5ms（VOFA 时间戳可验收） */
#define CTRL_TASK_STACK         (512u)
#define LOG_TASK_STACK          (512u)
#define CMD_TASK_STACK          (512u)
#define CTRL_TASK_PRIO          (4)
#define LOG_TASK_PRIO           (2)
#define CMD_TASK_PRIO           (3)

/* ---------- 速度目标上限（GM6020 最高 320rpm，任务书 1000rpm 做不到） ---------- */
#define MOTOR_SPEED_LIMIT_RPM   (250.0f)

#endif /* __BOARD_CONFIG_H */
