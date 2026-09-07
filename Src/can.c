/**
  ******************************************************************************
  * @file    can.c
  * @brief   CAN1 初始化：PA11=RX / PA12=TX，1 Mbps（GM6020 规定波特率）
  *
  *  波特率不写死：由 PCLK1 自动算分频
  *    TQ 总数 = 1(SYNC) + BS1(10) + BS2(3) = 14
  *    Prescaler = PCLK1 / (1MHz × 14) = 42MHz / 14M = 3
  *    采样点 = (1+10)/14 ≈ 78.6%（工程推荐 75%~80%）
  ******************************************************************************
  */
#include "can.h"
#include "main.h"
#include "board_config.h"

CAN_HandleTypeDef hcan1;

static QueueHandle_t s_canQueue = NULL;
static uint16_t     s_prescaler = 0;

void MX_CAN1_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStruct = {0};
    CAN_FilterTypeDef  filter;

    /* ---------------- 时钟与引脚 ---------------- */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /** CAN1 GPIO: PA11 = RX, PA12 = TX (AF9) */
    GPIO_InitStruct.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ---------------- 波特率：由实际 PCLK1 计算 ---------------- */
    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    uint32_t presc = pclk1 / ((uint32_t)CAN_BAUD_HZ * CAN_TQ_TOTAL);
    if (presc == 0U) { presc = 1U; }
    if (presc > 1024U) { presc = 1024U; }
    s_prescaler = (uint16_t)presc;

    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = s_prescaler;
    hcan1.Init.Mode                 = (CAN_LOOPBACK_TEST) ? CAN_MODE_LOOPBACK
                                                          : CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_10TQ;
    hcan1.Init.TimeSeg2             = CAN_BS2_3TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = ENABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;    /* 总线忙时自动重发，指令不丢 */
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---------------- 过滤器：掩码模式一次收下 0x200~0x20F ---------------- */
    filter.FilterBank           = 0;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = (uint16_t)(0x200U << 5);   /* 标准 ID 放高 16 位 */
    filter.FilterIdLow          = 0x0000U;
    filter.FilterMaskIdHigh     = (uint16_t)(0x7F0U << 5);   /* 高 7 位必须匹配 */
    filter.FilterMaskIdLow      = 0x0000U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }

    /* ---------------- 启动 + 开接收中断 ---------------- */
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    /* NVIC：优先级 >= 5（FreeRTOS 中 ISR 内可调用 FromISR API） */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    /* 接收队列（调度器启动前创建，合法） */
    if (s_canQueue == NULL)
    {
        s_canQueue = xQueueCreate(CAN_QUEUE_LENGTH, sizeof(CanRxPacket_t));
    }
}

QueueHandle_t CAN_GetQueue(void)
{
    return s_canQueue;
}

uint16_t CAN_GetPrescaler(void)
{
    return s_prescaler;
}

/**
  * @brief  FIFO0 报文挂起回调（ISR 上下文）：只打包入队，不解析
  */
void CAN_RxFifo0Callback(CAN_HandleTypeDef *hcan)
{
    BaseType_t        xHigherPriorityTaskWoken = pdFALSE;
    CAN_RxHeaderTypeDef hdr;
    CanRxPacket_t     pkt;

    if (s_canQueue == NULL) { return; }

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, pkt.data) == HAL_OK)
        {
            pkt.id  = (hdr.IDE == CAN_ID_STD) ? hdr.StdId : hdr.ExtId;
            pkt.dlc = hdr.DLC;
            (void)xQueueSendFromISR(s_canQueue, &pkt, &xHigherPriorityTaskWoken);
        }
        else
        {
            break;
        }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
