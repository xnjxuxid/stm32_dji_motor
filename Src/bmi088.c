/**
  ******************************************************************************
  * @file    bmi088.c
  * @brief   BMI088（SPI）驱动实现：初始化 / 自检 / 六轴读取
  ******************************************************************************
  */
#include "bmi088.h"
#include "board_config.h"
#include "spi.h"
#include <stdio.h>
#include <math.h>

/* ---------- 寄存器 ---------- */
#define ACC_CHIP_ID       0x00u
#define ACC_ERR_REG       0x02u
#define ACC_X_LSB         0x12u
#define ACC_TEMP_MSB      0x22u
#define ACC_CONF          0x40u
#define ACC_RANGE         0x41u
#define ACC_PWR_CONF      0x7Cu
#define ACC_PWR_CTRL      0x7Du
#define ACC_SOFTRESET     0x7Eu

#define GYRO_CHIP_ID      0x00u
#define GYRO_X_LSB        0x02u
#define GYRO_RANGE        0x0Fu
#define GYRO_BANDWIDTH    0x10u
#define GYRO_LPM1         0x11u
#define GYRO_SOFTRESET    0x14u
#define GYRO_SELF_TEST    0x3Cu

/* ---------- 底层 SPI 收发 ---------- */
static void CS_AccLow(void)  { HAL_GPIO_WritePin(BMI088_CS_ACC_PORT,  BMI088_CS_ACC_PIN,  GPIO_PIN_RESET); }
static void CS_AccHigh(void) { HAL_GPIO_WritePin(BMI088_CS_ACC_PORT,  BMI088_CS_ACC_PIN,  GPIO_PIN_SET);   }
static void CS_GyroLow(void) { HAL_GPIO_WritePin(BMI088_CS_GYRO_PORT, BMI088_CS_GYRO_PIN, GPIO_PIN_RESET); }
static void CS_GyroHigh(void){ HAL_GPIO_WritePin(BMI088_CS_GYRO_PORT, BMI088_CS_GYRO_PIN, GPIO_PIN_SET);   }

static void WriteReg(uint8_t cs, uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    (void)cs;
    tx[0] = (uint8_t)(reg & 0x7Fu);   /* 写：最高位 0 */
    tx[1] = val;

    if (cs == 0U) { CS_AccLow(); } else { CS_GyroLow(); }
    HAL_SPI_Transmit(BMI088_SPI, tx, 2, 10);
    if (cs == 0U) { CS_AccHigh(); } else { CS_GyroHigh(); }
}

/* 读：加速度计需要先发一个 dummy 字节再读（BMI088 硬件要求） */
static uint8_t ReadReg(uint8_t cs, uint8_t reg)
{
    uint8_t tx = (uint8_t)(reg | 0x80u);   /* 读：最高位 1 */
    uint8_t rx = 0;

    if (cs == 0U) { CS_AccLow(); } else { CS_GyroLow(); }
    HAL_SPI_Transmit(BMI088_SPI, &tx, 1, 10);
    if (cs == 0U)
    {
        uint8_t dummy = 0x00u;
        HAL_SPI_Transmit(BMI088_SPI, &dummy, 1, 10);   /* 加速度计 dummy 字节 */
    }
    HAL_SPI_Receive(BMI088_SPI, &rx, 1, 10);
    if (cs == 0U) { CS_AccHigh(); } else { CS_GyroHigh(); }
    return rx;
}

static void ReadMulti(uint8_t cs, uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx = (uint8_t)(reg | 0x80u);

    if (cs == 0U) { CS_AccLow(); } else { CS_GyroLow(); }
    HAL_SPI_Transmit(BMI088_SPI, &tx, 1, 10);
    if (cs == 0U)
    {
        uint8_t dummy = 0x00u;
        HAL_SPI_Transmit(BMI088_SPI, &dummy, 1, 10);
    }
    HAL_SPI_Receive(BMI088_SPI, buf, len, 10);
    if (cs == 0U) { CS_AccHigh(); } else { CS_GyroHigh(); }
}

#define ACC_SEL  0U
#define GYRO_SEL 1U

/* ---------- API ---------- */
void BMI088_Init(BMI088_t *dev)
{
    dev->accId = dev->gyroId = 0;
    dev->selfTestOk = 0;
    dev->online = 0;
    dev->rxCount = 0;

    /* 加速度计：上电 → 等待 → 配置 */
    WriteReg(ACC_SEL, ACC_SOFTRESET, 0xB6u);
    HAL_Delay(30);
    WriteReg(ACC_SEL, ACC_PWR_CONF, 0x00u);     /* active */
    HAL_Delay(5);
    WriteReg(ACC_SEL, ACC_PWR_CTRL, 0x04u);     /* 打开加速度计 */
    HAL_Delay(50);
    WriteReg(ACC_SEL, ACC_CONF,  0x0Au);        /* ODR 1600Hz, 正常带宽 */
    HAL_Delay(5);
    WriteReg(ACC_SEL, ACC_RANGE, 0x01u);        /* ±6g */
    HAL_Delay(5);

    /* 陀螺：退出低功耗 → 量程 ±1000dps → 带宽 */
    WriteReg(GYRO_SEL, GYRO_SOFTRESET, 0xB6u);
    HAL_Delay(30);
    WriteReg(GYRO_SEL, GYRO_LPM1,      0x00u);  /* normal mode */
    HAL_Delay(30);
    WriteReg(GYRO_SEL, GYRO_RANGE,     0x01u);  /* ±1000 dps */
    HAL_Delay(5);
    WriteReg(GYRO_SEL, GYRO_BANDWIDTH, 0x07u);  /* ODR 1000Hz, 116Hz 滤波 */
    HAL_Delay(10);

    dev->accId  = ReadReg(ACC_SEL,  ACC_CHIP_ID);
    dev->gyroId = ReadReg(GYRO_SEL, GYRO_CHIP_ID);

    dev->online = ((dev->accId == BMI088_ACC_CHIP_ID) &&
                   (dev->gyroId == BMI088_GYRO_CHIP_ID)) ? 1U : 0U;
}

int BMI088_SelfTest(BMI088_t *dev)
{
    int ok = 1;

    printf("BMI088 self-test:\r\n");

    /* 1) CHIP_ID */
    printf("  acc  CHIP_ID = 0x%02X (expect 0x1E) : %s\r\n",
           dev->accId, dev->accId == BMI088_ACC_CHIP_ID ? "OK" : "FAIL");
    printf("  gyro CHIP_ID = 0x%02X (expect 0x0F) : %s\r\n",
           dev->gyroId, dev->gyroId == BMI088_GYRO_CHIP_ID ? "OK" : "FAIL");
    if (dev->accId != BMI088_ACC_CHIP_ID)  { ok = 0; }
    if (dev->gyroId != BMI088_GYRO_CHIP_ID){ ok = 0; }

    /* 2) 陀螺自检（寄存器流程） */
    WriteReg(GYRO_SEL, GYRO_SELF_TEST, 0x01u);
    HAL_Delay(50);
    {
        uint8_t st = ReadReg(GYRO_SEL, GYRO_SELF_TEST);
        uint8_t pass = (uint8_t)((st & 0x02u) >> 1);
        printf("  gyro self-test reg = 0x%02X : %s\r\n", st, pass ? "OK" : "FAIL");
        if (pass == 0U) { ok = 0; }
    }

    /* 3) 加速度数据合理性自检：静止时矢量模长应 ≈ 1g */
    HAL_Delay(20);
    BMI088_Read(dev);
    {
        float mag = sqrtf(dev->accG[0] * dev->accG[0] +
                          dev->accG[1] * dev->accG[1] +
                          dev->accG[2] * dev->accG[2]);
        uint8_t pass = ((mag > 0.8f) && (mag < 1.25f)) ? 1U : 0U;
        printf("  acc magnitude = %.3f g (expect ~1.0, keep still) : %s\r\n",
               mag, pass ? "OK" : "FAIL");
        if (pass == 0U) { ok = 0; }
    }

    dev->selfTestOk = (uint8_t)(ok ? 1U : 0U);
    printf("BMI088 self-test %s\r\n", ok ? "PASSED" : "FAILED");
    return ok ? 0 : -1;
}

void BMI088_Read(BMI088_t *dev)
{
    uint8_t b[8];

    /* 加速度 0x12 起 6 字节（小端） */
    ReadMulti(ACC_SEL, ACC_X_LSB, b, 6);
    dev->accRaw[0] = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    dev->accRaw[1] = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
    dev->accRaw[2] = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));

    /* 温度：0x22/0x23，11 位补码，分辨率 0.125℃/LSB，偏移 23℃ */
    ReadMulti(ACC_SEL, ACC_TEMP_MSB, b, 2);
    {
        int16_t t = (int16_t)(((uint16_t)b[0] << 3) | ((uint16_t)b[1] >> 5));
        if (t > 1023) { t -= 2048; }
        dev->tempC = (float)t * 0.125f + 23.0f;
    }

    /* 陀螺 0x02 起 6 字节 */
    ReadMulti(GYRO_SEL, GYRO_X_LSB, b, 6);
    dev->gyroRaw[0] = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
    dev->gyroRaw[1] = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
    dev->gyroRaw[2] = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));

    for (uint8_t i = 0U; i < 3U; i++)
    {
        dev->accG[i]    = (float)dev->accRaw[i]  / BMI088_ACC_LSB_PER_G;
        dev->gyroDps[i] = (float)dev->gyroRaw[i] / BMI088_GYRO_LSB_PER_DPS;
    }

    dev->rxCount++;
}
