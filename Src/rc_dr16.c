/**
  ******************************************************************************
  * @file    rc_dr16.c
  * @brief   遥控接收：DJI DR16（DBUS，18 字节）与 SBUS（25 字节）自动识别
  *
  *  硬件（原理图确认）：板上 DBUS 座经 SS8050 三极管反相后接 USART2_RX(PA3)，
  *  电平反相由硬件完成。
  *  物理层：100000 bps，8 数据位 + 偶校验 + 2 停止位。
  *  接收：RXNE 逐字节中断 + 帧间超时(>2ms)判定帧边界 → 收满即解析。
  *  保护：超过 RC_LINK_TIMEOUT_MS 没新帧 → linked = 0（调用方立即停机）。
  ******************************************************************************
  */
#include "rc_dr16.h"
#include "board_config.h"
#include "main.h"
#include <string.h>

UART_HandleTypeDef huart2;
RC_t               g_rc;

#define RC_RX_BUF_SIZE  (32u)
#define RC_FRAME_GAP_MS (2u)      /* 帧内字节间隔 <2ms，超过则视为新一帧 */

static uint8_t  s_byte;
static uint8_t  s_rxBuf[RC_RX_BUF_SIZE];
static uint8_t  s_rxLen = 0;
static uint32_t s_lastByteMs = 0;

/* ---------------- 串口初始化 ---------------- */
void MX_USART1_UART_Init(void)   /* 函数名与 main.c 调用保持一致，实际是 USART2 */
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = RC_BAUDRATE;          /* 100000 */
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;   /* + 偶校验 = 9 位帧 */
    huart2.Init.StopBits     = UART_STOPBITS_2;
    huart2.Init.Parity       = UART_PARITY_EVEN;
    huart2.Init.Mode         = UART_MODE_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }

    (void)HAL_UART_Receive_IT(&huart2, &s_byte, 1);
}

/* ---------------- 解析 ---------------- */
static void ParseDBUS(const uint8_t *b)
{
    g_rc.ch[0] = (uint16_t)(((uint16_t)b[0]        | ((uint16_t)b[1] << 8)) & 0x07FFu);
    g_rc.ch[1] = (uint16_t)(((uint16_t)(b[1] >> 3) | ((uint16_t)b[2] << 5)) & 0x07FFu);
    g_rc.ch[2] = (uint16_t)(((uint16_t)(b[2] >> 6) | ((uint16_t)b[3] << 2) |
                             ((uint16_t)b[4] << 10)) & 0x07FFu);
    g_rc.ch[3] = (uint16_t)(((uint16_t)(b[4] >> 1) | ((uint16_t)b[5] << 7)) & 0x07FFu);
    g_rc.s1    = (uint8_t)((b[5] >> 4) & 0x03u);
    g_rc.s2    = (uint8_t)((b[5] >> 6) & 0x03u);
}

static void ParseSBUS(const uint8_t *b)
{
    const uint8_t *p = &b[1];
    g_rc.ch[0]  = (uint16_t)(((uint16_t)p[0]        | ((uint16_t)p[1] << 8)) & 0x07FFu);
    g_rc.ch[1]  = (uint16_t)(((uint16_t)(p[1] >> 3) | ((uint16_t)p[2] << 5)) & 0x07FFu);
    g_rc.ch[2]  = (uint16_t)(((uint16_t)(p[2] >> 6) | ((uint16_t)p[3] << 2) |
                              ((uint16_t)p[4] << 10)) & 0x07FFu);
    g_rc.ch[3]  = (uint16_t)(((uint16_t)(p[4] >> 1) | ((uint16_t)p[5] << 7)) & 0x07FFu);
    g_rc.ch[4]  = (uint16_t)(((uint16_t)(p[5] >> 6) | ((uint16_t)p[6] << 2) |
                              ((uint16_t)p[7] << 10)) & 0x07FFu);
    g_rc.ch[5]  = (uint16_t)(((uint16_t)(p[7] >> 1) | ((uint16_t)p[8] << 7)) & 0x07FFu);
    g_rc.ch[6]  = (uint16_t)(((uint16_t)(p[8] >> 6) | ((uint16_t)p[9] << 2) |
                              ((uint16_t)p[10] << 10)) & 0x07FFu);
    g_rc.ch[7]  = (uint16_t)(((uint16_t)(p[10] >> 1)| ((uint16_t)p[11] << 7)) & 0x07FFu);
}

static void RC_FeedByte(uint8_t byte)
{
    uint32_t now = HAL_GetTick();

    /* 帧间超时：字节间隔超过 2ms 视为新一帧，重新对齐 */
    if ((s_rxLen > 0U) && ((now - s_lastByteMs) >= RC_FRAME_GAP_MS))
    {
        s_rxLen = 0;
    }
    s_lastByteMs = now;

    if (s_rxLen >= RC_RX_BUF_SIZE)
    {
        s_rxLen = 0;
        return;
    }
    s_rxBuf[s_rxLen++] = byte;

    if (s_rxLen == 18U)
    {
        /* DBUS：18 字节，无固定帧头（DR16） */
        ParseDBUS(s_rxBuf);
        g_rc.linked = 1U;
        g_rc.frameCount++;
        g_rc.lastFrameMs = now;
        s_rxLen = 0;
    }
    else if (s_rxLen == 25U)
    {
        /* SBUS：帧头 0x0F，帧尾 0x00 */
        if ((s_rxBuf[0] == 0x0FU) && (s_rxBuf[24] == 0x00U))
        {
            ParseSBUS(s_rxBuf);
            g_rc.linked = 1U;
            g_rc.frameCount++;
            g_rc.lastFrameMs = now;
        }
        s_rxLen = 0;
    }
}

/**
  * @brief HAL 逐字节接收完成回调（ISR 上下文）：喂字节 + 续接接收
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        RC_FeedByte(s_byte);
        (void)HAL_UART_Receive_IT(&huart2, &s_byte, 1);
    }
}

/* ---------------- API ---------------- */
void RC_Init(void)
{
    memset(&g_rc, 0, sizeof(g_rc));
    s_rxLen = 0;
    s_lastByteMs = 0;
}

uint8_t RC_IsLinked(void)
{
    if ((HAL_GetTick() - g_rc.lastFrameMs) > RC_LINK_TIMEOUT_MS)
    {
        g_rc.linked = 0U;
        return 0U;
    }
    return g_rc.linked;
}

/* 通道归一化：中值 1024(DBUS) / 992(SBUS)，范围约 ±660 → -1.0 ~ +1.0 */
float RC_Norm(uint8_t idx)
{
    float v;
    if (idx >= 16U) { return 0.0f; }
    v = (float)g_rc.ch[idx] - 1024.0f;
    v /= 660.0f;
    if (v >  1.0f) { v =  1.0f; }
    if (v < -1.0f) { v = -1.0f; }
    return v;
}
