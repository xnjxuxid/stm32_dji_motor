/**
  ******************************************************************************
  * @file    vofa_send.c
  * @brief   VOFA+ JustFloat 帧打包与发送
  *
  *  帧结构：ch[0]..ch[n-1] 各 4 字节小端 float，尾帧 0x00 0x00 0x80 0x7F
  *  发送：复用 uart_vofa.c 的 UART_SendData_DMA()（内部互斥量保护，
  *        与任务一保留的文本回传任务不冲突）
  ******************************************************************************
  */
#include <string.h>
#include "vofa_send.h"
#include "uart_vofa.h"

void VOFA_SendJustFloat(const float *channels, int n)
{
    static uint8_t frame[9 * 4 + 4];        /* 最大 9 通道 + 4 字节帧尾 */
    int i;

    if ((channels == 0) || (n <= 0) || (n > 9))
    {
        return;
    }

    for (i = 0; i < n; i++)
    {
        /* float 直接按内存布局拷贝（Cortex-M4 小端，float32 即 VOFA 需要的格式） */
        memcpy(&frame[i * 4], &channels[i], 4);
    }
    /* JustFloat 帧尾 */
    frame[n * 4 + 0] = 0x00u;
    frame[n * 4 + 1] = 0x00u;
    frame[n * 4 + 2] = 0x80u;
    frame[n * 4 + 3] = 0x7Fu;

    UART_SendData_DMA(frame, (uint16_t)(n * 4 + 4));
}
