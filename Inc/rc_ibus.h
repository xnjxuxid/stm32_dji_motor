/**
  ******************************************************************************
  * @file    rc_ibus.h
  * @brief   遥控接收（协议可选，见 board_config.h 的 RC_PROTOCOL）
  *
  *  RC_PROTOCOL = 0  iBus  ：USART6_RX(PC7) 115200 8N1 正逻辑，J8 座
  *                           帧：0x20 0x40 + 14 通道×2B + 校验和 = 32 字节
  *  RC_PROTOCOL = 1  DBUS/SBUS：USART2_RX(PA3) 100000 8E2 反相（板上已反相），P1 座
  *                           DBUS 18 字节 / SBUS 25 字节（0x0F 头、0x00 尾）自动识别
  *
  *  两种协议都：逐字节中断接收 + 帧超时重同步 + 校验通过才更新通道。
  *  保护：超过 RC_LINK_TIMEOUT_MS 无新帧 → linked = 0（调用方立即停机）。
  ******************************************************************************
  */
#ifndef __RC_IBUS_H
#define __RC_IBUS_H

#include <stdint.h>

typedef struct
{
    uint16_t ch[16];        /* iBus 1000~2000us；DBUS 364~1684（中位 1024） */
    uint8_t  s1;            /* DBUS 拨杆（iBus 模式下无效） */
    uint8_t  s2;
    uint8_t  failsafe;      /* SBUS 帧里的失控标志（接收机进入失控保护） */
    uint8_t  linked;
    uint32_t frameCount;
    uint32_t lastFrameMs;
} RC_t;

extern RC_t g_rc;

void    MX_RC_UART_Init(void);
void    RC_Init(void);
uint8_t RC_IsLinked(void);
float   RC_Norm(uint8_t idx);        /* 归一化到 -1.0 ~ +1.0 */

#endif /* __RC_IBUS_H */
