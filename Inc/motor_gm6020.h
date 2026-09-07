/**
  ******************************************************************************
  * @file    motor_gm6020.h
  * @brief   GM6020 电机对象：反馈解析、连续（多圈）角度、电压指令发送
  *
  *  反馈帧（1kHz）：标识符 0x204 + ID，8 字节
  *    [0][1] 机械角度 0~8191（大端）
  *    [2][3] 转速 int16，单位 rpm（正 = CCW）
  *    [4][5] 实际转矩电流 int16（±16384 ↔ ±3A）
  *    [6]    温度 ℃
  *  控制帧：0x1FF（ID1~4）/ 0x2FF（ID5~7），电压给定 int16 ±25000（大端）
  ******************************************************************************
  */
#ifndef __MOTOR_GM6020_H
#define __MOTOR_GM6020_H

#include <stdint.h>

typedef struct
{
    uint8_t  id;            /* 1~7 */
    uint16_t angleRaw;      /* 0~8191        */
    int16_t  speedRpm;      /* rpm           */
    int16_t  currentRaw;    /* ±16384        */
    uint8_t  tempC;         /* ℃             */

    float    angleDeg;      /* 单圈角度 0~360°      */
    float    angleCont;     /* 连续角度（度，可多圈）*/
    float    speed;         /* rpm                  */
    float    currentA;      /* A                    */

    uint16_t lastRaw;       /* 上一次角度原始值（跨圈检测） */
    uint8_t  rawValid;
    uint32_t lastRxMs;      /* 最后一次收到反馈的系统时间   */
    uint32_t rxCount;
    uint8_t  online;
} GM6020_t;

void   GM6020_Init(GM6020_t *m, uint8_t id);
void   GM6020_Update(GM6020_t *m, const uint8_t *data8);
void   GM6020_ZeroAngle(GM6020_t *m);
uint8_t GM6020_IsOnline(GM6020_t *m, uint32_t timeoutMs);
void   GM6020_SendVoltage(uint8_t id, float volt);
void   GM6020_SendStop(uint8_t id);

#endif /* __MOTOR_GM6020_H */
