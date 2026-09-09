/**
  ******************************************************************************
  * @file    motor_gm6020.c
  ******************************************************************************
  */
#include "motor_gm6020.h"
#include "board_config.h"
#include "can.h"
#include <math.h>

extern CAN_HandleTypeDef hcan1;

/* ---------------- 反馈解析 ---------------- */

void GM6020_Init(GM6020_t *m, uint8_t id)
{
    m->id        = id;
    m->angleRaw  = 0;
    m->speedRpm  = 0;
    m->currentRaw= 0;
    m->tempC     = 0;
    m->angleDeg  = 0.0f;
    m->angleCont = 0.0f;
    m->speedRaw  = 0.0f;
    m->speed     = 0.0f;
    m->speedFilt = 0.30f;    /* 1ms 周期下截止约 57Hz；想更平滑改小，想更快改大 */
    m->currentA  = 0.0f;
    m->lastRaw   = 0;
    m->rawValid  = 0;
    m->lastRxMs  = 0;
    m->rxCount   = 0;
    m->online    = 0;
}

/**
  * @brief  解析一帧反馈，并把单圈角度累加成"连续角度"（支持 720° 等多圈）
  *
  *  跨圈处理：机械角度只有 0~8191，转过零点会从 8191 跳到 0。
  *  相邻两帧的差值绝对值 > 半圈(4096) 时判定为跨越零点，做 ±8192 修正。
  */
void GM6020_Update(GM6020_t *m, const uint8_t *d)
{
    int16_t delta;

    m->angleRaw   = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
    m->speedRpm   = (int16_t)(((uint16_t)d[2] << 8) | d[3]);
    m->currentRaw = (int16_t)(((uint16_t)d[4] << 8) | d[5]);
    m->tempC      = d[6];

    m->angleDeg  = (float)m->angleRaw * 360.0f / GM6020_ANGLE_MAX_RAW;
    m->speedRaw  = (float)m->speedRpm;
    /* 一阶低通：滤掉转速反馈的量化噪声/高频抖动，能明显抬高可用的 kp 上限 */
    m->speed    += (m->speedRaw - m->speed) * m->speedFilt;
    m->currentA  = (float)m->currentRaw * GM6020_CUR_MAX_A / GM6020_CUR_MAX_RAW;

    if (m->rawValid)
    {
        delta = (int16_t)m->angleRaw - (int16_t)m->lastRaw;
        if (delta >  4096) { delta -= 8192; }
        if (delta < -4096) { delta += 8192; }
        m->angleCont += (float)delta * 360.0f / GM6020_ANGLE_MAX_RAW;
    }
    else
    {
        m->rawValid = 1;               /* 第一帧只建立基准，不累加 */
    }
    m->lastRaw = m->angleRaw;

    m->lastRxMs = HAL_GetTick();
    m->rxCount++;
    m->online   = 1;
}

void GM6020_ZeroAngle(GM6020_t *m)
{
    m->angleCont = 0.0f;              /* 把当前位置设为角度零点 */
}

uint8_t GM6020_IsOnline(GM6020_t *m, uint32_t timeoutMs)
{
    if ((HAL_GetTick() - m->lastRxMs) > timeoutMs)
    {
        m->online = 0;
        return 0;
    }
    return 1;
}

/* ---------------- 控制指令 ---------------- */

static uint8_t GM6020_CAN_Send(uint32_t stdId, uint8_t *data)
{
    CAN_TxHeaderTypeDef hdr;
    uint32_t            mailbox;

    hdr.StdId = stdId;
    hdr.IDE   = CAN_ID_STD;
    hdr.RTR   = CAN_RTR_DATA;
    hdr.DLC   = 8;

    /* 三个发送邮箱都忙时返回 0（1ms 周期下极少发生，忽略这一次即可） */
    if (HAL_CAN_AddTxMessage(&hcan1, &hdr, data, &mailbox) != HAL_OK)
    {
        return 0;
    }
    return 1;
}

void GM6020_SendVoltage(uint8_t id, float volt)
{
    uint8_t  tx[8];
    uint32_t stdId;
    uint8_t  slot;
    int16_t  v;

    if ((id < 1U) || (id > 7U)) { return; }

    if (id <= 4U)
    {
        stdId = CTRL_ID_1_4;
        slot  = (uint8_t)((id - 1U) * 2U);
    }
    else
    {
        stdId = CTRL_ID_5_7;
        slot  = (uint8_t)((id - 5U) * 2U);
    }

    if (volt >  GM6020_VOLT_MAX) { volt =  GM6020_VOLT_MAX; }
    if (volt < -GM6020_VOLT_MAX) { volt = -GM6020_VOLT_MAX; }
    v = (int16_t)volt;

    for (uint8_t i = 0U; i < 8U; i++) { tx[i] = 0U; }

    tx[slot]     = (uint8_t)((uint16_t)v >> 8);   /* 大端：高 8 位在前 */
    tx[slot + 1] = (uint8_t)((uint16_t)v & 0xFFU);

    (void)GM6020_CAN_Send(stdId, tx);
}

void GM6020_SendStop(uint8_t id)
{
    GM6020_SendVoltage(id, 0.0f);
}
