/**
  ******************************************************************************
  * @file    rc_ibus.c
  * @brief   遥控接收实现（iBus / DBUS-SBUS 二选一，由 RC_PROTOCOL 决定）
  ******************************************************************************
  */
#include "rc_ibus.h"
#include "board_config.h"
#include "main.h"
#include <string.h>

RC_t g_rc;

#define RC_FRAME_GAP_MS   (3u)      /* 帧内字节间隔超过它 → 重新同步 */

static uint8_t  s_byte;
static uint8_t  s_len = 0;
static uint32_t s_lastByteMs = 0;

#if (RC_PROTOCOL == 0)
/* ============================ iBus（USART6 / PC7） ============================ */
UART_HandleTypeDef huart6;

#define IBUS_FRAME_LEN  (32u)
static uint8_t s_buf[IBUS_FRAME_LEN];

void MX_RC_UART_Init(void)
{
    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = RC_BAUDRATE;          /* 115200 */
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK) { Error_Handler(); }
    (void)HAL_UART_Receive_IT(&huart6, &s_byte, 1);
}

static void FeedByte(uint8_t b)
{
    uint32_t now = HAL_GetTick();

    /* iBus：帧头 0x20 0x40 逐字节同步（不依赖超时，帧间隔很短也能对齐） */
    if (s_len == 0U)
    {
        if (b != 0x20u) { return; }
    }
    else if (s_len == 1U)
    {
        if (b != 0x40u)
        {
            /* 0x20 后面不是 0x40：这个 0x20 可能是数据，重新从当前字节判断 */
            s_len = 0U;
            FeedByte(b);
            return;
        }
    }

    s_buf[s_len++] = b;
    s_lastByteMs = now;

    if (s_len >= IBUS_FRAME_LEN)
    {
        uint16_t sum = 0;
        uint8_t  i;
        s_len = 0;
        for (i = 0U; i < (IBUS_FRAME_LEN - 2U); i++) { sum = (uint16_t)(sum + s_buf[i]); }
        if (((uint16_t)(((uint16_t)s_buf[31] << 8) | s_buf[30])) == (uint16_t)(0xFFFFu - sum))
        {
            for (i = 0U; i < 14U; i++)
            {
                g_rc.ch[i] = (uint16_t)(((uint16_t)s_buf[2 + i * 2]) |
                                        ((uint16_t)s_buf[3 + i * 2] << 8));
            }
            g_rc.linked = 1U; g_rc.frameCount++; g_rc.lastFrameMs = now;
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

float RC_Norm(uint8_t idx)
{
    float v;
    if (idx >= 14U) { return 0.0f; }
    v = ((float)g_rc.ch[idx] - 1500.0f) / 500.0f;     /* iBus 中位 1500 */
    if (v >  1.0f) { v =  1.0f; }
    if (v < -1.0f) { v = -1.0f; }
    return v;
}

#else
/* ========================= DBUS / SBUS（USART2 / PA3） ======================== */
UART_HandleTypeDef huart2;

#define RC_RX_BUF_SIZE  (32u)
static uint8_t s_buf[RC_RX_BUF_SIZE];

void MX_RC_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = RC_BAUDRATE;          /* 100000 */
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;   /* + 偶校验 = 9 位帧 */
    huart2.Init.StopBits     = UART_STOPBITS_2;
    huart2.Init.Parity       = UART_PARITY_EVEN;
    huart2.Init.Mode         = UART_MODE_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    /* 反相由板上 SS8050 三极管完成（原理图：P1 座 DBUS → Q1 → PA3） */
    if (HAL_UART_Init(&huart2) != HAL_OK) { Error_Handler(); }
    (void)HAL_UART_Receive_IT(&huart2, &s_byte, 1);
}

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
    g_rc.ch[0] = (uint16_t)(((uint16_t)p[0]        | ((uint16_t)p[1] << 8)) & 0x07FFu);
    g_rc.ch[1] = (uint16_t)(((uint16_t)(p[1] >> 3) | ((uint16_t)p[2] << 5)) & 0x07FFu);
    g_rc.ch[2] = (uint16_t)(((uint16_t)(p[2] >> 6) | ((uint16_t)p[3] << 2) |
                             ((uint16_t)p[4] << 10)) & 0x07FFu);
    g_rc.ch[3] = (uint16_t)(((uint16_t)(p[4] >> 1) | ((uint16_t)p[5] << 7)) & 0x07FFu);
    g_rc.ch[4] = (uint16_t)(((uint16_t)(p[5] >> 6) | ((uint16_t)p[6] << 2) |
                             ((uint16_t)p[7] << 10)) & 0x07FFu);
    g_rc.ch[5] = (uint16_t)(((uint16_t)(p[7] >> 1) | ((uint16_t)p[8] << 7)) & 0x07FFu);
}

static void FeedByte(uint8_t b)
{
    /* SBUS/DBUS：SBUS 有帧头 0x0F + 帧尾 0x00，直接按帧头同步
     * （SBUS 帧率高、帧间隔可能小于超时阈值，靠超时对齐会错位）；
     * DBUS（DR16）无帧头，仍靠帧间超时对齐。 */
#if 1
    if (s_len == 0U)
    {
        if (b != 0x0Fu) { return; }              /* 等帧头 */
    }
    s_buf[s_len++] = b;
    s_lastByteMs = HAL_GetTick();

    if (s_len >= 25U)
    {
        if ((s_buf[0] == 0x0Fu) && (s_buf[24] == 0x00U))
        {
            /* SBUS flags 在 byte[23]：bit2 = 丢帧，bit3 = 失控保护 */
            uint8_t flags = s_buf[23];
            g_rc.failsafe = (uint8_t)((flags & 0x08U) ? 1U : 0U);

            if ((flags & 0x0CU) == 0U)          /* 正常帧才更新通道 */
            {
                ParseSBUS(s_buf);
                g_rc.linked = 1U; g_rc.frameCount++; g_rc.lastFrameMs = s_lastByteMs;
            }
        }
        s_len = 0;
        (void)b;
    }
#else
    uint32_t now = HAL_GetTick();

    if ((s_len > 0U) && ((now - s_lastByteMs) >= RC_FRAME_GAP_MS)) { s_len = 0; }
    s_lastByteMs = now;

    if (s_len >= RC_RX_BUF_SIZE) { s_len = 0; return; }
    s_buf[s_len++] = b;

    if (s_len == 18U)                       /* DBUS（DR16） */
    {
        ParseDBUS(s_buf);
        g_rc.linked = 1U; g_rc.frameCount++; g_rc.lastFrameMs = now;
        s_len = 0;
    }
    else if (s_len == 25U)                  /* SBUS */
    {
        if ((s_buf[0] == 0x0Fu) && (s_buf[24] == 0x00U))
        {
            /* SBUS flags 在 byte[23]：bit2 = 丢帧，bit3 = 失控保护 */
            uint8_t flags = s_buf[23];
            g_rc.failsafe = (uint8_t)((flags & 0x08U) ? 1U : 0U);

            if ((flags & 0x0CU) == 0U)          /* 正常帧才更新通道 */
            {
                ParseSBUS(s_buf);
                g_rc.linked = 1U; g_rc.frameCount++; g_rc.lastFrameMs = now;
            }
        }
        s_len = 0;
    }
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        FeedByte(s_byte);
        (void)HAL_UART_Receive_IT(&huart2, &s_byte, 1);
    }
}

float RC_Norm(uint8_t idx)
{
    float v;
    if (idx >= 16U) { return 0.0f; }
    v = ((float)g_rc.ch[idx] - 992.0f) / 820.0f;    /* SBUS：172~1811，中位 992 */
    if (v >  1.0f) { v =  1.0f; }
    if (v < -1.0f) { v = -1.0f; }
    return v;
}
#endif

/* ============================== 公共部分 ============================== */
void RC_Init(void)
{
    memset(&g_rc, 0, sizeof(g_rc));
    s_len = 0;
    (void)ParseDBUS;    /* DR16(DBUS) 模式使用；SBUS 模式下保留备用 */
}

uint8_t RC_IsLinked(void)
{
    /* SBUS 接收机进入失控保护 → 立即判定失联（比超时更快） */
    if (g_rc.failsafe != 0U)
    {
        g_rc.linked = 0U;
        return 0U;
    }
    if ((HAL_GetTick() - g_rc.lastFrameMs) > RC_LINK_TIMEOUT_MS)
    {
        g_rc.linked = 0U;
        return 0U;
    }
    return g_rc.linked;
}
