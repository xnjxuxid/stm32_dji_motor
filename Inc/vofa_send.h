#ifndef __VOFA_SEND_H
#define __VOFA_SEND_H

#include <stdint.h>

/* ============================================================================
 * VOFA+ JustFloat 协议发送
 * ----------------------------------------------------------------------------
 * JustFloat 帧格式：N 个 float（小端 4 字节）+ 帧尾 0x00 0x00 0x80 0x7F
 * VOFA+ 上位机选"JustFloat"协议，即可把每个 float 画成一条实时曲线。
 * 优势：二进制定长帧，115200 波特率下 9 通道只需约 3.5ms < 5ms 周期；
 *       文本打印一行要 3~5ms 且不可靠，所以周期性数据用 JustFloat。
 * ============================================================================*/

/* 发送 n 通道 JustFloat 帧（内部走 UART DMA，与其它任务共用互斥量） */
void VOFA_SendJustFloat(const float *channels, int n);

#endif /* __VOFA_SEND_H */
