/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @brief   阶段二：云台板 F405 + GM6020（CAN 1Mbps）+ FreeRTOS
  *
  *  数据流：
  *    CAN 中断 --(队列)--> Task_MotorCtrl(1ms) --双环 PID--> CAN 电压指令
  *    串口空闲中断 --(队列)--> Task_Cmd（调参命令）
  *    Task_Log(5ms) --> VOFA（文本 / JustFloat 曲线）
  ******************************************************************************
  */	
/* USER CODE END Header */
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes */
#include <stdio.h>
#include "board_config.h"
#include "uart_vofa.h"
#include "app_tasks.h"

void SystemClock_Config(void);
void MX_FREERTOS_Init(void);

#if CLOCK_USE_HSE
static uint32_t s_clockSrc = 1;  /* 1 = HSE, 2 = HSI(回退)；仅 HSE 模式下会打印 */
#endif

int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();          /* 必须先于 UART5：DMA 句柄要挂到 huart */
    MX_CAN1_Init();         /* CAN1 PA11/PA12，1 Mbps（分频自动计算） */
    MX_UART5_Init();        /* UART5 PC12/PD2，115200                 */

    printf("\r\n==== GM6020 CAN demo (F405 + FreeRTOS) ====\r\n");
#if CLOCK_USE_HSE
    printf("clock source : HSE %lu Hz\r\n", (unsigned long)BOARD_HSE_HZ);
#else
    printf("clock source : HSI 16MHz (internal, crystal-independent)\r\n");
#endif
    printf("SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(),
           (unsigned long)HAL_RCC_GetHCLKFreq(),
           (unsigned long)HAL_RCC_GetPCLK1Freq(),
           (unsigned long)HAL_RCC_GetPCLK2Freq());
    printf("CAN1: %s baud=%lu bps (prescaler=%u, TQ=%u)\r\n",
           (CAN_LOOPBACK_TEST) ? "LOOPBACK" : "NORMAL",
           (unsigned long)(HAL_RCC_GetPCLK1Freq() /
                           ((uint32_t)CAN_GetPrescaler() * CAN_TQ_TOTAL)),
           (unsigned)CAN_GetPrescaler(), (unsigned)CAN_TQ_TOTAL);
    printf("motor ID=%u, feedback ID=0x%03X, ctrl ID=0x%03X\r\n",
           (unsigned)MOTOR_ID, (unsigned)MOTOR_FEEDBACK_ID,
           (unsigned)((MOTOR_ID <= 4U) ? CTRL_ID_1_4 : CTRL_ID_5_7));
    printf("send 'help' for commands\r\n");

    UART_VOFA_Init();       /* 打开串口空闲中断 + DMA 接收（调参命令） */

    osKernelInitialize();
    MX_FREERTOS_Init();     /* 任务在 freertos.c 中调用 App_Tasks_Create() */
    osKernelStart();

    for (;;) { }
}

/**
  * @brief  系统时钟：优先 HSE 8MHz → 168MHz；没有外部晶振则回退 HSI 16MHz
  *         两条路径都是 SYSCLK=168MHz / PCLK1=42MHz，CAN 分频自动适配
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

#if CLOCK_USE_HSE
    /* 路径 A：外部晶振 BOARD_HSE_HZ → 168MHz（晶振频率必须与丝印一致！） */
    uint32_t hse = BOARD_HSE_HZ;
    uint32_t pllm = hse / 1000000U;         /* 把 HSE 分频到 1MHz 再倍频 */
    if (pllm == 0U) { pllm = 1U; }

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM       = pllm;
    osc.PLL.PLLN       = 336;               /* 1MHz ×336 = 336MHz(VCO) */
    osc.PLL.PLLP       = RCC_PLLP_DIV2;     /* /2 = 168MHz            */
    osc.PLL.PLLQ       = 7;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        HAL_RCC_DeInit();
        s_clockSrc = 2;
        /* 回退 HSI（见下方路径 B 的同一套参数） */
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSEState       = RCC_HSE_OFF;
        osc.HSIState       = RCC_HSI_ON;
        osc.PLL.PLLState   = RCC_PLL_ON;
        osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM       = 16;
        osc.PLL.PLLN       = 336;
        osc.PLL.PLLP       = RCC_PLLP_DIV2;
        osc.PLL.PLLQ       = 7;
        if (HAL_RCC_OscConfig(&osc) != HAL_OK)
        {
            Error_Handler();
        }
    }
#else
    /* 路径 B（默认）：内部 HSI 16MHz → 168MHz
     * 完全不依赖外部晶振 → 不可能因晶振频率不符而超频跑飞/锁死调试口 */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState       = RCC_HSI_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM       = 16;                /* 16MHz / 16 = 1MHz       */
    osc.PLL.PLLN       = 336;               /* ×336 = 336MHz (VCO)     */
    osc.PLL.PLLP       = RCC_PLLP_DIV2;     /* /2 = 168MHz             */
    osc.PLL.PLLQ       = 7;                 /* 336/7 = 48MHz           */
    if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    {
        Error_Handler();
    }
#endif

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;   /* HCLK  = 168MHz */
    clk.APB1CLKDivider = RCC_HCLK_DIV4;     /* PCLK1 = 42MHz  */
    clk.APB2CLKDivider = RCC_HCLK_DIV2;     /* PCLK2 = 84MHz  */

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  TIM1 是 HAL 的 1ms 时基（见 stm32f4xx_hal_timebase_tim.c），
  *         SysTick 留给 FreeRTOS 专用，两者互不干扰
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxFifo0Callback(hcan);
}

/* USER CODE END 4 */

void Error_Handler(void)
{
    __disable_irq();
    for (;;) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file; (void)line;
}
#endif
