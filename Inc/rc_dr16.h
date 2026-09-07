/**
  ******************************************************************************
  * @file    rc_dr16.h
  * @brief   遥控接收：DJI DR16（DBUS，18 字节）与 SBUS（25 字节）自动识别
  *
  *  物理层：USART1_RX = PA10，100000 bps，8 数据位 + 偶校验 + 2 停止位，
  *          **电平反相**（DR16 / SBUS 都是反相的，必须开 USART 的 RXINV）
  *  帧格式：DBUS 18 字节（4 通道摇杆 + 2 个拨杆 + 鼠标键盘）
  *          SBUS 25 字节（0x0F 头、16 通道 × 11 位、flag、0x00 尾）
  *  接收：DMA 循环 + 串口空闲中断（IDLE）→ 一帧结束即解析
  *  保护：超过 RC_LINK_TIMEOUT_MS 没新帧 → linked = 0（调用方立即停机）
  ******************************************************************************
  */
#ifndef __RC_DR16_H
#define __RC_DR16_H

#include <stdint.h>

typedef struct
{
    uint16_t ch[16];        /* 通道原始值（DBUS 364~1684 / SBUS 172~1811） */
    uint8_t  s1;            /* 拨杆 S1：1上 3中 2下（DBUS） */
    uint8_t  s2;
    uint8_t  linked;        /* 1 = 遥控器在线 */
    uint32_t frameCount;
    uint32_t lastFrameMs;
} RC_t;

extern RC_t g_rc;

void MX_USART1_UART_Init(void);
void RC_Init(void);                        /* 开 IDLE 中断 + 启动 DMA */
void RC_IdleCallback(void);                /* 由 USART1_IRQHandler 调用 */
uint8_t RC_IsLinked(void);                 /* 0 = 失联（必须立即停机） */
float  RC_Norm(uint8_t idx);               /* 通道归一化到 -1.0 ~ +1.0 */

#endif /* __RC_DR16_H */
