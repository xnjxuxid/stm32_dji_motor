/**
  ******************************************************************************
  * @file    bmi088.h
  * @brief   BMI088 驱动（SPI 接口，原理图确认：非 I2C）
  *
  *  加速度计：CHIP_ID = 0x1E（寄存器 0x00）
  *  陀螺仪  ：CHIP_ID = 0x0F（寄存器 0x00）
  *  ⚠️ SPI 读差异：加速度计读需要**一个 dummy 字节**，陀螺不需要（硬件特性）
  *
  *  自检（任务书要求）：
  *   1) CHIP_ID 校验（上电）
  *   2) 陀螺自检：写 GYRO_SELF_TEST(0x3C)=0x01，等待完成后读结果位
  *   3) 加速度自检：静止时三轴矢量和应 ≈ 1g（数据合理性自检）
  ******************************************************************************
  */
#ifndef __BMI088_H
#define __BMI088_H

#include <stdint.h>

#define BMI088_ACC_CHIP_ID    (0x1Eu)
#define BMI088_GYRO_CHIP_ID   (0x0Fu)

typedef struct
{
    int16_t accRaw[3];      /* 原始值 */
    int16_t gyroRaw[3];
    float   accG[3];        /* g      */
    float   gyroDps[3];     /* °/s    */
    float   tempC;          /* ℃（加速度计内置温度计） */

    uint8_t accId;
    uint8_t gyroId;
    uint8_t selfTestOk;     /* 1 = 全部自检通过 */
    uint8_t online;
    uint32_t rxCount;
} BMI088_t;

void BMI088_Init(BMI088_t *dev);
int  BMI088_SelfTest(BMI088_t *dev);      /* 0 = 通过；打印自检过程 */
void BMI088_Read(BMI088_t *dev);          /* 读六轴 + 温度 */

#endif /* __BMI088_H */
