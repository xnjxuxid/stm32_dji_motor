/**
  ******************************************************************************
  * @file    usart.c
  * @brief   UART5 初始化：PC12 = TX, PD2 = RX（接 USB-TTL → VOFA / 调参命令）
  *          115200 8N1；RX = DMA1_Stream0(CH4) 循环，TX = DMA1_Stream7(CH4) 普通
  ******************************************************************************
  */
#include "usart.h"
#include "main.h"
#include "board_config.h"
#include <stdio.h>

UART_HandleTypeDef huart5;

void MX_UART5_Init(void)
{
    huart5.Instance          = UART5;
    huart5.Init.BaudRate     = 115200;
    huart5.Init.WordLength   = UART_WORDLENGTH_8B;
    huart5.Init.StopBits     = UART_STOPBITS_1;
    huart5.Init.Parity       = UART_PARITY_NONE;
    huart5.Init.Mode         = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == UART5)
    {
        __HAL_RCC_UART5_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /** PC12 = UART5_TX (AF8) */
        GPIO_InitStruct.Pin       = GPIO_PIN_12;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /** PD2 = UART5_RX (AF8) */
        GPIO_InitStruct.Pin       = GPIO_PIN_2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* 挂载 DMA 句柄（句柄本体在 dma.c 中创建） */
        __HAL_LINKDMA(huart, hdmarx, hdma_uart5_rx);
        __HAL_LINKDMA(huart, hdmatx, hdma_uart5_tx);

        HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
        /* ⚠️ UART5 的 DMA 收发中断必须使能：
         *  - DMA1_Stream7(TX) 的 TC 中断负责把 gState 从 BUSY_TX 恢复成 READY，
         *    不使能的话 JustFloat 只能发出第一帧，之后全部堵死
         *  - DMA1_Stream0(RX) 配合 IDLE 中断收不定长命令 */
        HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
        HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    }
#if (RC_PROTOCOL == 1)
    else if (huart->Instance == USART2)
    {
        /* ---- DBUS/SBUS：USART2_RX = PA3（P1 座，经 Q1 SS8050 反相） ---- */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_3;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
#else
    else if (huart->Instance == USART6)
    {
        /* ---- 遥控器 iBus：USART6_RX = PC7（板上 J8 座，无反相、带 3.6V 保护） ---- */
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART6_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
#endif
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        __HAL_RCC_UART5_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
        HAL_NVIC_DisableIRQ(UART5_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Stream0_IRQn);
        HAL_NVIC_DisableIRQ(DMA1_Stream7_IRQn);
    }
}

/* printf 重定向到 UART5（MicroLIB + fputc） */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart5, (uint8_t *)&ch, 1, 1000);
    return ch;
}
