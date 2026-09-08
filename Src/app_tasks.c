/**
  ******************************************************************************
  * @file    app_tasks.c
  * @brief   三个 FreeRTOS 任务
  *
  *   Task_MotorCtrl (1ms, vTaskDelayUntil) : 取 CAN 反馈 → 双环 PID → 发电压指令
  *   Task_Log       (5ms, vTaskDelayUntil) : VOFA 打印（文本 / JustFloat 曲线）
  *   Task_Cmd       (事件驱动)             : 串口命令（设目标 / 改 PID 参数）
  *
  *  控制结构（双环）：
  *    目标角度 --[位置环]--> 速度目标(限幅) --[速度环]--> 电压 --> 电机
  ******************************************************************************
  */
#include "app_tasks.h"
#include "board_config.h"
#include "can.h"
#include "stm32f4xx_hal.h"
#include "uart_vofa.h"
#include "vofa_send.h"
#include "rc_ibus.h"
#include "bmi088.h"
#include "spi.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

MotorCtrl_t g_ctrl;
GM6020_t    g_motor;
BMI088_t    g_imu;

/* 诊断：收到了非本电机 ID 的反馈帧（说明总线通了，只是 ID 配错） */
static uint32_t s_rxOtherId = 0;

static void Task_MotorCtrl(void *arg);
static void Task_Log(void *arg);
static void Task_Cmd(void *arg);
static void Task_Rc(void *arg);
static void Task_Imu(void *arg);

/* ---------------- 工具 ---------------- */
static float ClampF(float v, float lo, float hi)
{
    if (v > hi) { return hi; }
    if (v < lo) { return lo; }
    return v;
}

/* 串口控制命令时自动接管：禁用遥控，否则 Task_Rc 每 10ms 会用摇杆值
 * 覆盖串口刚设的目标（表现为电机抽动一下就停） */
static void SerialTakeOver(void)
{
    if (g_ctrl.rcEnabled != 0U)
    {
        g_ctrl.rcEnabled = 0U;
        printf("(remote control auto-disabled by serial command)\r\n");
    }
}

/* ---------------- 创建 ---------------- */
void App_Tasks_Create(void)
{
    GM6020_Init(&g_motor, MOTOR_ID);

    /* PID 初值（保守，之后用串口命令调，不需要重新编译）
     * 速度环：outMax 用大值，实际限幅靠 g_ctrl.voltLimit
     * 位置环：输出是"速度目标"，所以 outMax = 速度上限 */
    PID_Init(&g_ctrl.pidSpeed, 30.0f, 2.0f, 0.0f, 0.001f, GM6020_VOLT_MAX, 15000.0f);
    /* 位置环：Kp 决定"多快冲过去"，Ki 消除末端小误差导致的爬行，Kd 抑制超调 */
    PID_Init(&g_ctrl.pidAngle, 1.5f, 0.02f, 0.03f, 0.001f, MOTOR_SPEED_LIMIT_RPM, 60.0f);

    g_ctrl.mode       = MODE_IDLE;      /* 上电不输出，安全第一 */
    g_ctrl.voltCmd    = 0.0f;
    g_ctrl.speedTarget= 0.0f;
    g_ctrl.angleTarget= 0.0f;
    g_ctrl.speedCmd   = 0.0f;
    g_ctrl.voltLimit  = MOTOR_VOLT_LIMIT_DEFAULT;
    g_ctrl.speedLimit = MOTOR_SPEED_LIMIT_RPM;
    g_ctrl.angleMinSpeed = 15.0f;   /* 最小 15rpm：低于这个值电机推不动（静摩擦） */
    g_ctrl.angleDeadband = 0.5f;    /* 0.5° 以内算到位 */
    g_ctrl.out        = 0.0f;
    g_ctrl.logCurve   = 0U;
    g_ctrl.rcEnabled  = 0U;        /* 默认不允许遥控控制，命令 rc 1 开启 */
    g_ctrl.rcDeadzone = 0.10f;     /* 默认 10%：松手后有残余速度就往上加 */
    g_ctrl.rcSwCh     = 0U;        /* 0 = 不启用安全开关；命令 rsw 5 可指定通道 */

    /* ---- 阶段三：BMI088 上电自检（SPI） ---- */
    BMI088_Init(&g_imu);
    (void)BMI088_SelfTest(&g_imu);

    /* ---- 阶段三：启动遥控接收（USART1 + DMA + 空闲中断） ---- */
    RC_Init();

    xTaskCreate(Task_MotorCtrl, "motor", CTRL_TASK_STACK, NULL, CTRL_TASK_PRIO, NULL);
    xTaskCreate(Task_Log,       "log",   LOG_TASK_STACK,  NULL, LOG_TASK_PRIO,  NULL);
    xTaskCreate(Task_Cmd,       "cmd",   CMD_TASK_STACK,  NULL, CMD_TASK_PRIO,  NULL);
    xTaskCreate(Task_Rc,        "rc",    CMD_TASK_STACK,  NULL, CMD_TASK_PRIO,  NULL);
    xTaskCreate(Task_Imu,       "imu",   LOG_TASK_STACK,  NULL, LOG_TASK_PRIO,  NULL);
}

/* ---------------- 遥控任务（10ms）：解析结果 → 控制电机 + 失联保护 ---------------- */
static void Task_Rc(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    (void)arg;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10U));

        /* ★ 遥控保护（任务书硬要求）：失联 → 立即停止输出 + 清积分，
         *   电机进入"没力"状态（不发电压指令 / 发 0 电压） */
        if (!RC_IsLinked())
        {
            if (g_ctrl.mode != MODE_IDLE)
            {
                printf("[RC] link lost -> motor disabled\r\n");
            }
            g_ctrl.mode = MODE_IDLE;
            g_ctrl.out  = 0.0f;
            PID_Reset(&g_ctrl.pidSpeed);
            PID_Reset(&g_ctrl.pidAngle);
            GM6020_SendStop(MOTOR_ID);
            continue;
        }

        if (!g_ctrl.rcEnabled)
        {
            continue;
        }

        /* 可选安全开关：rsw 指定通道号（1~14），0 = 不用开关。
         * 开关通道 > 1500us（拨杆上位）才允许输出，否则停机。 */
        if (g_ctrl.rcSwCh != 0U)
        {
            /* 用归一化值判断（兼容 iBus 与 DBUS 两种量纲）：>0 才允许输出 */
            if (RC_Norm((uint8_t)(g_ctrl.rcSwCh - 1U)) <= 0.0f)
            {
                g_ctrl.mode = MODE_IDLE;
                g_ctrl.out  = 0.0f;
                PID_Reset(&g_ctrl.pidSpeed);
                PID_Reset(&g_ctrl.pidAngle);
                GM6020_SendStop(MOTOR_ID);
                continue;
            }
        }

        /* 右摇杆上下（CH2 = 索引 1）→ 速度目标（±speedLimit）
         * 死区：摇杆机械回中总有点误差（回不到精确 1024），死区把
         * "接近中位"的小量直接判零，否则松手后会有几 rpm 的残余速度 */
        float raw = RC_Norm(1);
        if (fabsf(raw) < g_ctrl.rcDeadzone) { raw = 0.0f; }
        float cmd = raw * g_ctrl.speedLimit;

        if (g_ctrl.mode != MODE_SPEED)
        {
            g_ctrl.mode = MODE_SPEED;
            PID_Reset(&g_ctrl.pidSpeed);     /* 切模式清积分，防止突变 */
        }
        g_ctrl.speedTarget = cmd;
    }
}

/* ---------------- BMI088 任务（10ms 读取，500ms 打印） ---------------- */
static void Task_Imu(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t   tick     = 0;
    (void)arg;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10U));

        if (g_imu.online)
        {
            BMI088_Read(&g_imu);

            if ((++tick % 50U) == 0U)     /* 500ms 一行 */
            {
                printf("imu: acc[%6.3f %6.3f %6.3f]g gyro[%7.2f %7.2f %7.2f]dps "
                       "temp=%.1fC n=%lu\r\n",
                       g_imu.accG[0], g_imu.accG[1], g_imu.accG[2],
                       g_imu.gyroDps[0], g_imu.gyroDps[1], g_imu.gyroDps[2],
                       g_imu.tempC, (unsigned long)g_imu.rxCount);
            }
        }
        else
        {
            if ((++tick % 100U) == 0U)
            {
                printf("imu: not detected (accId=0x%02X gyroId=0x%02X) - check SPI wiring\r\n",
                       g_imu.accId, g_imu.gyroId);
            }
        }
    }
}

/* ---------------- 控制任务：1ms 绝对周期 ---------------- */
static void Task_MotorCtrl(void *arg)
{
    TickType_t    lastWake = xTaskGetTickCount();
    CanRxPacket_t pkt;
    QueueHandle_t q        = CAN_GetQueue();
    float         out      = 0.0f;
    float         speedCmd = 0.0f;

    (void)arg;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(CTRL_TASK_PERIOD_MS));

        /* 1) 取走所有 CAN 反馈帧（ISR 只入队，这里才解析） */
        if (q != NULL)
        {
            while (xQueueReceive(q, &pkt, 0) == pdPASS)
            {
                if (pkt.id == MOTOR_FEEDBACK_ID)
                {
                    GM6020_Update(&g_motor, pkt.data);
                }
                else if ((pkt.id >= 0x200U) && (pkt.id <= 0x20FU))
                {
                    /* 收到了别的电机的反馈 → ID 配置不对，提示正确的 ID */
                    s_rxOtherId = pkt.id;
                }
            }
        }

        /* 2) 保护：反馈超时 → 立即停止输出并清积分（拔掉 CAN 线也安全） */
        {
            uint8_t wasOnline = g_motor.online;
            if (!GM6020_IsOnline(&g_motor, MOTOR_RX_TIMEOUT_MS))
            {
                if (wasOnline != 0U)   /* 沿触发：只在"由在线变失联"时打一次 */
                {
                    printf("[MOTOR] feedback timeout -> output disabled\r\n");
                }
                g_ctrl.out = 0.0f;
                PID_Reset(&g_ctrl.pidSpeed);
                PID_Reset(&g_ctrl.pidAngle);
                GM6020_SendStop(MOTOR_ID);
                continue;
            }
            if (wasOnline == 0U)
            {
                printf("[MOTOR] feedback OK\r\n");
            }
        }

        /* 3) 模式机 + 双环 */
        switch (g_ctrl.mode)
        {
        case MODE_VOLT:
            out = g_ctrl.voltCmd;
            break;

        case MODE_SPEED:
            out = PID_Calc(&g_ctrl.pidSpeed,
                           g_ctrl.speedTarget, g_motor.speed);
            break;

        case MODE_ANGLE:
            /* 外环：角度误差 → 速度目标（限幅 = 给电机加"最高限速"） */
            speedCmd = PID_Calc(&g_ctrl.pidAngle,
                                g_ctrl.angleTarget, g_motor.angleCont);
            speedCmd = ClampF(speedCmd, -g_ctrl.speedLimit, g_ctrl.speedLimit);

            /* 抗静摩擦：还有误差但速度指令已小于"能推动电机的最小速度"时，
             * 直接给到最小速度，否则最后几度会一直慢慢爬（末端爬行现象） */
            if (fabsf(g_ctrl.pidAngle.err) > g_ctrl.angleDeadband)
            {
                if ((speedCmd < g_ctrl.angleMinSpeed) &&
                    (speedCmd > -g_ctrl.angleMinSpeed))
                {
                    speedCmd = (g_ctrl.pidAngle.err >= 0.0f) ?
                               g_ctrl.angleMinSpeed : -g_ctrl.angleMinSpeed;
                }
            }
            else
            {
                speedCmd = 0.0f;      /* 已到死区内：停住，避免来回抖 */
            }
            g_ctrl.speedCmd = speedCmd;
            /* 内环：速度误差 → 电压 */
            out = PID_Calc(&g_ctrl.pidSpeed, speedCmd, g_motor.speed);
            break;

        default:
            out = 0.0f;
            break;
        }

        /* 4) 输出限幅 + 下发 */
        out = ClampF(out, -g_ctrl.voltLimit, g_ctrl.voltLimit);
        g_ctrl.out = out;
        GM6020_SendVoltage(MOTOR_ID, out);
    }
}

/* ---------------- 日志/曲线任务：5ms 绝对周期 ---------------- */
static void Task_Log(void *arg)
{
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t   tick     = 0;
    float      ch[4];

    (void)arg;

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(LOG_TASK_PERIOD_MS));

        if (g_ctrl.logCurve)
        {
            /* JustFloat 曲线（VOFA 选 JustFloat 协议）：
             * ch0 = 目标, ch1 = 实际, ch2 = 输出, ch3 = 误差 */
            switch (g_ctrl.mode)
            {
            case MODE_SPEED: ch[0] = g_ctrl.speedTarget; ch[1] = g_motor.speed;     break;
            case MODE_ANGLE: ch[0] = g_ctrl.angleTarget; ch[1] = g_motor.angleCont; break;
            default:         ch[0] = g_ctrl.voltCmd;     ch[1] = g_motor.speed;     break;
            }
            ch[2] = g_ctrl.out;
            ch[3] = ch[0] - ch[1];
            VOFA_SendJustFloat(ch, 4);
        }
        else
        {
            /* 文本模式：每 500ms 一行，便于直接看数值 */
            if ((++tick % 100U) == 0U)
            {
                /* CAN 错误状态（二分定位：物理层没通 vs ID 不对） */
                uint32_t esr  = hcan1.Instance->ESR;
                uint8_t  lec  = (uint8_t)((esr >> 4) & 0x07U);
                uint8_t  boff = (uint8_t)((esr >> 2) & 0x01U);
                uint8_t  tec  = (uint8_t)((esr >> 16) & 0xFFU);
                uint8_t  rec  = (uint8_t)((esr >> 24) & 0xFFU);

                printf("mode=%d online=%u rx=%lu ang=%.1f cont=%.1f spd=%.0f "
                       "cur=%.2fA tmp=%uC out=%.0f | can lec=%u tec=%u rec=%u boff=%u\r\n",
                       (int)g_ctrl.mode, (unsigned)g_motor.online,
                       (unsigned long)g_motor.rxCount,
                       g_motor.angleDeg, g_motor.angleCont, g_motor.speed,
                       g_motor.currentA, (unsigned)g_motor.tempC, g_ctrl.out,
                       (unsigned)lec, (unsigned)tec, (unsigned)rec, (unsigned)boff);

                /* 遥控状态行：一眼看出 SBUS/DBUS 有没有数据进来 */
                printf("rc: link=%u fs=%u fc=%lu ch=[%u %u %u %u] sw=%u\r\n",
                       (unsigned)g_rc.linked, (unsigned)g_rc.failsafe,
                       (unsigned long)g_rc.frameCount,
                       (unsigned)g_rc.ch[0], (unsigned)g_rc.ch[1],
                       (unsigned)g_rc.ch[2], (unsigned)g_rc.ch[3],
                       (unsigned)g_ctrl.rcSwCh);

                if (s_rxOtherId != 0U)
                {
                    printf("note: got feedback from ID 0x%03lX -> set MOTOR_ID=%lu "
                           "in board_config.h\r\n",
                           (unsigned long)s_rxOtherId,
                           (unsigned long)(s_rxOtherId - 0x204U));
                }
            }
        }
    }
}

/* ---------------- 命令任务：串口调参（不重新编译） ---------------- */
static void HandleCommand(char *line)
{
    char  *cmd;
    char  *arg;
    float  v;

    cmd = strtok(line, " \r\n\t");
    if (cmd == NULL) { return; }
    arg = strtok(NULL, " \r\n\t");
    v   = (arg != NULL) ? (float)atof(arg) : 0.0f;

    if      (strcmp(cmd, "v")  == 0) {                      /* 直接给电压试转 */
        SerialTakeOver();
        g_ctrl.mode = MODE_VOLT;
        g_ctrl.voltCmd = ClampF(v, -g_ctrl.voltLimit, g_ctrl.voltLimit);
        printf("volt mode: %.0f\r\n", g_ctrl.voltCmd);
    }
    else if (strcmp(cmd, "sp") == 0) {                      /* 速度环目标 */
        SerialTakeOver();
        PID_Reset(&g_ctrl.pidSpeed);
        g_ctrl.mode = MODE_SPEED;
        g_ctrl.speedTarget = ClampF(v, -g_ctrl.speedLimit, g_ctrl.speedLimit);
        printf("speed target: %.1f rpm\r\n", g_ctrl.speedTarget);
    }
    else if (strcmp(cmd, "an") == 0) {                      /* 位置环：相对当前转 v 度 */
        SerialTakeOver();
        PID_Reset(&g_ctrl.pidSpeed);
        PID_Reset(&g_ctrl.pidAngle);
        g_ctrl.mode = MODE_ANGLE;
        g_ctrl.angleTarget = g_motor.angleCont + v;
        printf("angle target: %.1f deg (now %.1f)\r\n",
               g_ctrl.angleTarget, g_motor.angleCont);
    }
    else if (strcmp(cmd, "kp")  == 0) { g_ctrl.pidSpeed.Kp = v; }
    else if (strcmp(cmd, "ki")  == 0) { g_ctrl.pidSpeed.Ki = v; }
    else if (strcmp(cmd, "kd")  == 0) { g_ctrl.pidSpeed.Kd = v; }
    else if (strcmp(cmd, "kp2") == 0) { g_ctrl.pidAngle.Kp = v; }
    else if (strcmp(cmd, "ki2") == 0) { g_ctrl.pidAngle.Ki = v; }
    else if (strcmp(cmd, "kd2") == 0) { g_ctrl.pidAngle.Kd = v; }
    else if (strcmp(cmd, "lv")  == 0) { g_ctrl.voltLimit = ClampF(fabsf(v), 0.0f, GM6020_VOLT_MAX); }
    else if (strcmp(cmd, "ls")  == 0) { g_ctrl.speedLimit = ClampF(fabsf(v), 0.0f, 320.0f); }
    else if (strcmp(cmd, "lmin")== 0) { g_ctrl.angleMinSpeed = ClampF(fabsf(v), 0.0f, 100.0f); }
    else if (strcmp(cmd, "ad")  == 0) { g_ctrl.angleDeadband = ClampF(fabsf(v), 0.0f, 10.0f); }
    else if (strcmp(cmd, "log") == 0)
    {
        g_ctrl.logCurve = (v > 0.0f) ? 1U : 0U;
        if (g_ctrl.logCurve)
        {
            printf("log: JustFloat ON (4ch: target/actual/output/error), "
                   "text paused - set VOFA protocol to JustFloat to see curves\r\n");
        }
        else
        {
            printf("log: text mode ON (500ms per line)\r\n");
        }
    }
    else if (strcmp(cmd, "rc")  == 0)
    {
        g_ctrl.rcEnabled = (v > 0.0f) ? 1U : 0U;
        printf("remote control %s (S1 must be UP to enable output)\r\n",
               g_ctrl.rcEnabled ? "ENABLED" : "disabled");
    }
    else if (strcmp(cmd, "imust")== 0) { (void)BMI088_SelfTest(&g_imu); }
    else if (strcmp(cmd, "dz")   == 0)
    {
        g_ctrl.rcDeadzone = ClampF(fabsf(v), 0.0f, 0.5f);
        printf("rc deadzone = %.0f%%\r\n", g_ctrl.rcDeadzone * 100.0f);
    }
    else if (strcmp(cmd, "rsw")  == 0)
    {
        g_ctrl.rcSwCh = (uint8_t)ClampF(v, 0.0f, 14.0f);
        printf("rc safety switch channel = %u (0 = disabled)\r\n",
               (unsigned)g_ctrl.rcSwCh);
    }
    else if (strcmp(cmd, "zero")== 0) { GM6020_ZeroAngle(&g_motor); printf("angle zeroed\r\n"); }
    else if (strcmp(cmd, "stop")== 0) { g_ctrl.mode = MODE_IDLE; PID_Reset(&g_ctrl.pidSpeed); PID_Reset(&g_ctrl.pidAngle); }
    else if (strcmp(cmd, "help")== 0 || strcmp(cmd, "?") == 0)
    {
        printf("== commands ==\r\n"
               " v <volt>   direct voltage (try small first)\r\n"
               " sp <rpm>   speed loop target (GM6020 max 320rpm)\r\n"
               " an <deg>   angle loop: rotate N degrees (720 ok)\r\n"
               " kp/ki/kd   speed-loop gains | kp2/ki2/kd2 angle-loop gains\r\n"
               " lv <v>     voltage limit   | ls <rpm> inner speed limit\r\n"
               " rc 1|0     remote control on/off (DR16-SBUS, S1 up = enable)\r\n"
               " imust      run BMI088 self-test again\r\n"
               " lmin <rpm> min speed of angle loop (fix slow approach)\r\n"
               " ad <deg>   angle deadband  | zero=reset angle | stop\r\n"
               " log 1|0    curve/text print\r\n");
    }
    else
    {
        printf("unknown: %s (send 'help')\r\n", cmd);
    }
}

static void Task_Cmd(void *arg)
{
    UartRxPacket_t pkt;
    QueueHandle_t  q = UART_GetQueue();

    (void)arg;

    for (;;)
    {
        if ((q != NULL) && (xQueueReceive(q, &pkt, portMAX_DELAY) == pdPASS))
        {
            if (pkt.len >= UART_RX_BUF_SIZE) { pkt.len = UART_RX_BUF_SIZE - 1; }
            pkt.buf[pkt.len] = '\0';
            HandleCommand((char *)pkt.buf);
        }
    }
}
