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

/* ---------- 串口（VOFA + 调参命令） ---------- */
#define VOFA_UART_HANDLE        huart5
#define VOFA_UART               (&VOFA_UART_HANDLE)
#define VOFA_UART_INSTANCE      UART5

extern CAN_HandleTypeDef  hcan1;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef  hdma_uart5_rx;
extern DMA_HandleTypeDef  hdma_uart5_tx;

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
