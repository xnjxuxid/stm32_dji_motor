/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO：CAN1 / UART5 的引脚在各外设的 MspInit 中配置，这里只留统一入口
  ******************************************************************************
  */
#include "gpio.h"

void MX_GPIO_Init(void)
{
    /* CAN1(PA11/PA12)、UART5(PC12/PD2) 的引脚配置分别在 can.c / usart.c 完成 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
}
