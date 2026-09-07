/**
  ******************************************************************************
  * @file    pid.c
  * @brief   通用 PID 控制器实现
  *
  *  位置式 PID：
  *      out = Kp·e + Ki·Σe·dt + Kd·(e - e_last)/dt
  *
  *  工程化处理（都是实战必需，缺一个就会出问题）：
  *   1) 输出限幅     —— 防止超出执行机构能力（这里是 ±25000 电压）
  *   2) 积分限幅     —— 抗积分饱和：误差长期存在时积分不会"憋爆"
  *   3) PID_Reset    —— 切模式/触发保护时必须清积分，否则会突然猛冲
  *   4) 死区         —— 消除传感器噪声导致的抖动（可选，默认 0）
  ******************************************************************************
  */
#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float dt, float outMax, float iMax)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = (dt > 0.0f) ? dt : 0.001f;
    pid->outMax = outMax;
    pid->iMax = iMax;
    pid->deadband = 0.0f;
    PID_Reset(pid);
}

void PID_SetParam(PID_t *pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}

void PID_SetLimit(PID_t *pid, float outMax, float iMax)
{
    pid->outMax = outMax;
    pid->iMax = iMax;
}

void PID_Reset(PID_t *pid)
{
    pid->target   = 0.0f;
    pid->feedback = 0.0f;
    pid->err      = 0.0f;
    pid->errLast  = 0.0f;
    pid->errSum   = 0.0f;
    pid->pOut     = 0.0f;
    pid->iOut     = 0.0f;
    pid->dOut     = 0.0f;
    pid->out      = 0.0f;
}

float PID_Calc(PID_t *pid, float target, float feedback)
{
    float err = target - feedback;

    pid->target   = target;
    pid->feedback = feedback;

    if (fabsf(err) < pid->deadband)
    {
        err = 0.0f;
    }
    pid->err = err;

    /* ---- 比例 ---- */
    pid->pOut = pid->Kp * err;

    /* ---- 积分（先累加，再限幅，超限时把累加值拉回，抗饱和） ---- */
    pid->errSum += err * pid->dt;
    pid->iOut    = pid->Ki * pid->errSum;

    if (pid->iOut > pid->iMax)
    {
        pid->iOut = pid->iMax;
        if (pid->Ki > 1e-6f) { pid->errSum = pid->iMax / pid->Ki; }
    }
    else if (pid->iOut < -pid->iMax)
    {
        pid->iOut = -pid->iMax;
        if (pid->Ki > 1e-6f) { pid->errSum = -pid->iMax / pid->Ki; }
    }

    /* ---- 微分 ---- */
    pid->dOut = pid->Kd * (err - pid->errLast) / pid->dt;
    pid->errLast = err;

    /* ---- 合成 + 输出限幅 ---- */
    pid->out = pid->pOut + pid->iOut + pid->dOut;

    if (pid->out > pid->outMax)  { pid->out = pid->outMax; }
    if (pid->out < -pid->outMax) { pid->out = -pid->outMax; }

    return pid->out;
}
