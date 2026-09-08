/**
  ******************************************************************************
  * @file    rc_ibus.h
  * @brief   FlySky iBus 接收（iA10B 的 i-BUS 口）
  *
  *  协议：115200 / 8N1 / **正逻辑**（无需反相器）
  *  帧  ：0x20 0x40 | 14 通道 × 2 字节(小端, 1000~2000us) | 校验和 2 字节 = 32 字节
  *        校验和 = 0xFFFF - (前 30 字节之和)
  *  接口：USART6_RX = PC7（板上 J8 座，无反相、带 3.6V 保护）
  *  接收：逐字节中断 + 帧头同步 + 帧超时重组
  *  保护：超过 RC_LINK_TIMEOUT_MS 无新帧 → linked = 0（调用方立即停机）
  ******************************************************************************
  */
#ifndef __RC_IBUS_H
#define __RC_IBUS_H

#include <stdint.h>

typedef struct
{
    uint16_t ch[14];        /* 通道值 1000~2000us（中位 1500） */
    uint8_t  linked;
    uint32_t frameCount;
    uint32_t lastFrameMs;
} RC_t;

extern RC_t g_rc;

void MX_USART6_UART_Init(void);
void RC_Init(void);
uint8_t RC_IsLinked(void);
float  RC_Norm(uint8_t idx);         /* 1000~2000 → -1.0 ~ +1.0 */

#endif /* __RC_IBUS_H */
