/**
  ******************************************************************************
  * @file    dma.c
  * @brief   DMA 句柄：UART5_RX = DMA1_Stream0(CH4) 循环；UART5_TX = DMA1_Stream7(CH4)
  ******************************************************************************
  */
#include "dma.h"
#include "main.h"

DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_uart5_tx;

void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* ---------------- UART5 RX：循环模式，配合串口空闲中断收不定长 ---------------- */
    hdma_uart5_rx.Instance                 = DMA1_Stream0;
    hdma_uart5_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_uart5_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode                = DMA_CIRCULAR;
    hdma_uart5_rx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_uart5_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---------------- UART5 TX：普通模式（曲线/日志发送） ---------------- */
    hdma_uart5_tx.Instance                 = DMA1_Stream7;
    hdma_uart5_tx.Init.Channel             = DMA_CHANNEL_4;
    hdma_uart5_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    hdma_uart5_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_uart5_tx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_uart5_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_uart5_tx.Init.Mode                = DMA_NORMAL;
    hdma_uart5_tx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_uart5_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart5_tx) != HAL_OK)
    {
        Error_Handler();
    }
}
