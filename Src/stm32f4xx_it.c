/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   中断服务程序
  *   CAN1_RX0 : 电机反馈帧（HAL_CAN_IRQHandler 内部会调用接收回调）
  *   UART5    : 空闲中断打包不定长命令 + HAL 处理
  *   DMA1_S0/7: UART5 RX/TX
  *   SysTick  : 交给 FreeRTOS（HAL 时基由 TIM1 提供，见 hal_timebase_tim.c）
  ******************************************************************************
  */
#include "main.h"
#include "board_config.h"
#include "can.h"
#include "usart.h"
#include "dma.h"
#include "uart_vofa.h"
#include "rc_dr16.h"
#include "cmsis_os2.h"

void NMI_Handler(void)          { for (;;) { } }
void HardFault_Handler(void)    { for (;;) { } }
void MemManage_Handler(void)    { for (;;) { } }
void BusFault_Handler(void)     { for (;;) { } }
void UsageFault_Handler(void)   { for (;;) { } }
void DebugMon_Handler(void)     { }

/* 注意：SVC_Handler / PendSV_Handler 由 FreeRTOS 的 port.c 提供，
 *       SysTick_Handler 由 CMSIS-RTOS2 的 cmsis_os2.c 提供，
 *       这里绝对不要重复定义，否则链接报 multiply defined。 */

void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
    extern TIM_HandleTypeDef htim1;
    HAL_TIM_IRQHandler(&htim1);
}

void UART5_IRQHandler(void)
{
    UART_RxIdleCallback(&huart5);      /* 空闲中断：整行命令打包入队 */
    HAL_UART_IRQHandler(&huart5);
}

void DMA1_Stream0_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart5_rx);
}

void DMA1_Stream7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_uart5_tx);
}

void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);   /* 遥控器逐字节接收（RxCplt 回调在 rc_dr16.c） */
}
