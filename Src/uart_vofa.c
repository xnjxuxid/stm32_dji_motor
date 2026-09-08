#include <string.h>
#include "uart_vofa.h"

static QueueHandle_t     s_uartQueue = NULL;
static SemaphoreHandle_t s_txMutex   = NULL;

/* DMA 环形接收缓冲（空闲中断时取走已收长度） */
static uint8_t s_dmaRxBuf[UART_RX_BUF_SIZE];
static uint8_t s_dmaTxBuf[UART_TX_BUF_SIZE];

void UART_VOFA_Init(void)
{
    s_uartQueue = xQueueCreate(UART_QUEUE_LENGTH, sizeof(UartRxPacket_t));
    configASSERT(s_uartQueue != NULL);

    s_txMutex = xSemaphoreCreateMutex();
    configASSERT(s_txMutex != NULL);

    /* 打开串口空闲中断，并启动 DMA 接收（不定长）
     * 串口句柄统一用 VOFA_UART 宏，换串口时只改 board_config.h */
    __HAL_UART_CLEAR_IDLEFLAG(VOFA_UART);
    __HAL_UART_ENABLE_IT(VOFA_UART, UART_IT_IDLE);
    HAL_UART_Receive_DMA(VOFA_UART, s_dmaRxBuf, UART_RX_BUF_SIZE);
}

QueueHandle_t UART_GetQueue(void)
{
    return s_uartQueue;
}

/**
  * @brief  串口空闲中断回调：只打包 + 入队，不做数据处理
  * @note   运行在 ISR 上下文
  */
void UART_RxIdleCallback(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t   len;
    UartRxPacket_t pkt;

    if (huart->Instance != VOFA_UART_INSTANCE)
    {
        return;
    }

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == RESET)
    {
        return;
    }
    __HAL_UART_CLEAR_IDLEFLAG(huart);

    /* 先停 DMA 才能读出剩余计数值 */
    HAL_UART_DMAStop(huart);
    len = UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

    if ((len > 0U) && (s_uartQueue != NULL))
    {
        pkt.len = (len > UART_RX_BUF_SIZE) ? UART_RX_BUF_SIZE : len;
        memcpy(pkt.buf, s_dmaRxBuf, pkt.len);

        (void)xQueueSendFromISR(s_uartQueue, &pkt, &xHigherPriorityTaskWoken);
    }

    /* 立刻重启 DMA 接收，避免丢帧 */
    HAL_UART_Receive_DMA(huart, s_dmaRxBuf, UART_RX_BUF_SIZE);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
  * @brief  任务中调用：串口 + DMA 发送（互斥量保护，多任务共用一个串口）
  */
void UART_SendData_DMA(uint8_t *data, uint16_t len)
{
    if (s_txMutex == NULL)
    {
        return;
    }

    xSemaphoreTake(s_txMutex, portMAX_DELAY);

    /* 等待上一次 DMA 发送完成（TX 完成中断会把 gState 置回 READY）。
     * 加超时保护：万一 gState 卡在 BUSY（DMA 中断丢失等异常），
     * 强制中止发送恢复 READY，保证曲线流不会永久卡死。 */
    {
        TickType_t t0 = xTaskGetTickCount();
        while (VOFA_UART->gState != HAL_UART_STATE_READY)
        {
            if ((xTaskGetTickCount() - t0) > pdMS_TO_TICKS(20U))
            {
                HAL_UART_AbortTransmit(VOFA_UART);   /* 强制恢复 READY */
                break;
            }
            vTaskDelay(1);
        }
    }

    if (len > UART_TX_BUF_SIZE)
    {
        len = UART_TX_BUF_SIZE;
    }
    memcpy(s_dmaTxBuf, data, len);

    HAL_UART_Transmit_DMA(VOFA_UART, s_dmaTxBuf, len);

    xSemaphoreGive(s_txMutex);
}
