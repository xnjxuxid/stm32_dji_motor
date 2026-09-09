#ifndef __APP_TASKS_H
#define __APP_TASKS_H

#include "pid.h"
#include "motor_gm6020.h"
#include "bmi088.h"

/* 控制模式：IDLE(不输出) / 直接给电压 / 速度环 / 位置+速度双环 */
typedef enum
{
    MODE_IDLE  = 0,
    MODE_VOLT  = 1,
    MODE_SPEED = 2,
    MODE_ANGLE = 3
} CtrlMode_t;

/* 全局控制对象：Debug 的 Watch 窗口可以直接观察和修改这些量（不重新编译） */
typedef struct
{
    CtrlMode_t mode;
    float voltCmd;        /* 电压模式给定值            */
    float speedTarget;    /* 速度环目标 rpm            */
    float angleTarget;    /* 位置环目标（连续角度，度）*/
    float speedCmd;       /* 位置环输出 = 内环速度目标（调试观察用） */
    float voltLimit;      /* 输出限幅（安全）          */
    float speedLimit;     /* 内环速度限幅（安全）      */
    float angleMinSpeed;  /* 位置环最小速度指令（抗静摩擦，治末端爬行） */
    float angleDeadband;  /* 位置环到达死区（度），进入死区不再补速度 */
    float speedFF;        /* 速度前馈系数（电压/rpm），0 = 关闭前馈 */
    float out;            /* 最终输出电压（限幅后，实际下发） */
    float outRaw;         /* 限幅前的 PID 原始输出（诊断用：raw>>out 说明被 lv 卡住） */
    uint8_t logCurve;     /* 1 = JustFloat 曲线，0 = 文本 */
    uint8_t rcEnabled;    /* 1 = 允许遥控控制电机（命令 rc 1 开启） */
    uint8_t rcSwCh;       /* 安全开关通道号 1~14，0 = 不使用开关 */
    float  rcDeadzone;    /* 遥控死区（归一化 0~0.5）：吸收摇杆回中机械误差 */
    PID_t  pidSpeed;
    PID_t  pidAngle;
} MotorCtrl_t;

extern MotorCtrl_t g_ctrl;
extern GM6020_t    g_motor;
extern BMI088_t    g_imu;

void App_Tasks_Create(void);

#endif /* __APP_TASKS_H */
