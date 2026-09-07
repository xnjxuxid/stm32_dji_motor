#ifndef __CAN_H
#define __CAN_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

/* CAN 接收报文（ISR 里只做这个结构体入队，解析交给任务） */
typedef struct
{
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanRxPacket_t;

void MX_CAN1_Init(void);
QueueHandle_t CAN_GetQueue(void);
uint16_t CAN_GetPrescaler(void);   /* 实际分频值（用于打印真实波特率） */
void CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan);

#endif /* __CAN_H */
