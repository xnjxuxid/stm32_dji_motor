/**
  ******************************************************************************
  * @file    spi.c
  * @brief   SPI1：PA5=SCK, PA6=MISO, PA7=MOSI（接 BMI088 / ICM42688）
  *          片选用普通 GPIO：CS_Accel = PC4，CS_Gyro = PA4（硬件 NSS 不用）
  *
  *  时钟：SPI1 挂 APB2 = 84 MHz，分频 32 → 2.625 MHz（BMI088 上限 10MHz）
  *  模式：CPOL=0, CPHA=0（BMI088 支持 Mode0 / Mode3）
  ******************************************************************************
  */
#include "spi.h"
#include "main.h"
#include "board_config.h"

SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /** SPI1 GPIO: PA5=SCK, PA6=MISO, PA7=MOSI (AF5) */
    GPIO_InitStruct.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 软件片选：推挽输出，默认拉高（不选中） */
    GPIO_InitStruct.Pin   = BMI088_CS_GYRO_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(BMI088_CS_GYRO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = BMI088_CS_ACC_PIN;
    HAL_GPIO_Init(BMI088_CS_ACC_PORT, &GPIO_InitStruct);

    HAL_GPIO_WritePin(BMI088_CS_GYRO_PORT, BMI088_CS_GYRO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMI088_CS_ACC_PORT,  BMI088_CS_ACC_PIN,  GPIO_PIN_SET);

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;   /* 84MHz/32 = 2.6MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial     = 10;

    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
}
