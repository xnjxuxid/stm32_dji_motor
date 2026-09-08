/**
  ******************************************************************************
  * @file    rc_ibus.c
  * @brief   FlySky iBus 接收实现（USART6 / PC7）
  ******************************************************************************
  */
#include "rc_ibus.h"
#include "board_config.h"
#include "main.h"
#include <string.h>

UART_HandleTypeDef huart6;
RC_t               g_rc;

#define IBUS_FRAME_LEN  (32u)
#define IBUS_FRAME_GAP  (3u)      /* 帧内字节间隔 >3ms 视为新帧，重新同步 */

static uint8_t  s_byte;
static uint8_t  s_buf[IBUS_FRAME_LEN];
static uint8_t  s_len = 0;
static uint32_t s_lastByteMs = 0;

void MX_USART6_UART_Init(void)
{
    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = RC_BAUDRATE;          /* 115200 */
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;     /* iBus：无校验 */
    huart6.Init.Mode         = UART_MODE_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }

    (void)HAL_UART_Receive_IT(&huart6, &s_byte, 1);
}

static void FeedByte(uint8_t b)
{
    uint32_t now = HAL_GetTick();

    /* 帧间超时 → 重新同步 */
    if ((s_len > 0U) && ((now - s_lastByteMs) >= IBUS_FRAME_GAP))
    {
        s_len = 0;
    }
    s_lastByteMs = now;

    /* 帧头同步：0x20 0x40 */
    if (s_len == 0U)
    {
        if (b != 0x20u) { return; }
    }
    else if (s_len == 1U)
    {
        if (b != 0x40u) { s_len = 0U; return; }
    }

    s_buf[s_len++] = b;

    if (s_len >= IBUS_FRAME_LEN)
    {
        uint16_t sum = 0;
        uint16_t chk;
        uint8_t  i;

        s_len = 0;

        for (i = 0U; i < (IBUS_FRAME_LEN - 2U); i++)
        {
            sum = (uint16_t)(sum + s_buf[i]);
        }
        chk = (uint16_t)(((uint16_t)s_buf[31] << 8) | s_buf[30]);

        if (chk == (uint16_t)(0xFFFFu - sum))
        {
            for (i = 0U; i < 14U; i++)
            {
                g_rc.ch[i] = (uint16_t)(((uint16_t)s_buf[2 + i * 2]) |
                                        ((uint16_t)s_buf[3 + i * 2] << 8));
            }
            g_rc.linked      = 1U;
            g_rc.frameCount++;
            g_rc.lastFrameMs = now;
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        FeedByte(s_byte);
        (void)HAL_UART_Receive_IT(&huart6, &s_byte, 1);
    }
}

void RC_Init(void)
{
    memset(&g_rc, 0, sizeof(g_rc));
    s_len = 0;
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

/* 1000~2000us（中位 1500）→ -1.0 ~ +1.0 */
float RC_Norm(uint8_t idx)
{
    float v;
    if (idx >= 14U) { return 0.0f; }
    v = ((float)g_rc.ch[idx] - 1500.0f) / 500.0f;
    if (v >  1.0f) { v =  1.0f; }
    if (v < -1.0f) { v = -1.0f; }
    return v;
}
