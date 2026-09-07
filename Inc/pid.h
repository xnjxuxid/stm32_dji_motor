/**
  ******************************************************************************
  * @file    pid.h
  * @brief   通用 PID 控制器库（模块化 / 可实例化 / 可运行时改参数）
  *
  *  用法：
  *    PID_t pid_speed;                                   // 实例 1：速度环
  *    PID_t pid_angle;                                   // 实例 2：位置环
  *    PID_Init(&pid_speed, 1.0f, 0.1f, 0.0f, 0.001f, 25000, 8000);
  *    out = PID_Calc(&pid_speed, target_rpm, now_rpm);    // 给目标和实际 → 得输出
  *
  *  所有状态都在结构体里 → Debug 的 Watch 窗口能直接看 pOut/iOut/dOut 分解量，
  *  也能在运行时直接改 Kp/Ki/Kd（不重新编译）。
  ******************************************************************************
  */
#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct
{
    /* ---- 参数（可调） ---- */
    float Kp;
    float Ki;
    float Kd;
    float dt;           /* 控制周期（秒），1ms 环 = 0.001f */
    float outMax;       /* 输出限幅      */
    float iMax;         /* 积分限幅（抗积分饱和） */
    float deadband;     /* 误差死区（0 = 不启用） */

    /* ---- 状态（只读，调试看这些） ---- */
    float target;
    float feedback;
    float err;
    float errLast;
    float errSum;
    float pOut;
    float iOut;
    float dOut;
    float out;
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd,
               float dt, float outMax, float iMax);
void  PID_SetParam(PID_t *pid, float kp, float ki, float kd);
void  PID_SetLimit(PID_t *pid, float outMax, float iMax);
void  PID_Reset(PID_t *pid);
float PID_Calc(PID_t *pid, float target, float feedback);

#endif /* __PID_H */
