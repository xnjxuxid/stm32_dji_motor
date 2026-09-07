#ifndef __UART_VOFA_H
#define __UART_VOFA_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "board_config.h"

/* 空闲中断打包好的一帧串口数据（不定长） */
typedef struct
{
    uint16_t len;
    uint8_t  buf[UART_RX_BUF_SIZE];
} UartRxPacket_t;

void          UART_VOFA_Init(void);
QueueHandle_t UART_GetQueue(void);

/* 由 USART1_IRQHandler 在检测到空闲中断时调用 */
void          UART_RxIdleCallback(UART_HandleTypeDef *huart);

/* 任务中调用：通过 DMA 把数据发回 VOFA */
void          UART_SendData_DMA(uint8_t *data, uint16_t len);

#endif /* __UART_VOFA_H */
