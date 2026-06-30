/**
 * @file    pid.c
 * @brief   位置式PID控制器 — 带抗积分饱和 + 输出限幅
 * @note    浮点运算，10ms 调用周期
 */

#include "pid.h"

/* --------------------------------------------------------------------------
 *  内部工具
 * -------------------------------------------------------------------------- */

/* 数值钳位 */
static float clamp(float value, float lo, float hi)
{
    if (value > hi) return hi;
    if (value < lo) return lo;
    return value;
}

/* --------------------------------------------------------------------------
 *  对外接口
 * -------------------------------------------------------------------------- */

/**
 * @brief  初始化PID句柄
 * @param  kp/ki/kd      比例/积分/微分系数
 * @param  dt            采样周期(s), 如 0.01 = 10ms
 * @param  out_max/min   输出限幅, 防电机过冲
 */
void PID_Init(PID_HandleTypeDef *pid,
              float kp, float ki, float kd, float dt,
              float out_max, float out_min)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->dt = dt;
    pid->out_max = out_max;
    pid->out_min = out_min;
    pid->integral_max = out_max;   /* 积分限幅默认 = 输出限幅 */

    PID_Reset(pid);
}

/* 设定目标值 */
void PID_SetTarget(PID_HandleTypeDef *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/**
 * @brief  一次PID计算
 * @param  feedback     当前测量值
 * @param  dt_override  非0时覆盖默认dt（通常传0用默认）
 * @return 限幅后的输出值
 */
float PID_Compute(PID_HandleTypeDef *pid, float feedback, float dt_override)
{
    float dt = (dt_override > 0.0f) ? dt_override : pid->dt;
    if (dt <= 0.0f) return pid->output;     /* 防除零 */

    pid->feedback = feedback;
    pid->error    = pid->setpoint - feedback;

    /* P — 比例 */
    float p_term = pid->Kp * pid->error;

    /* I — 积分（带限幅防饱和） */
    pid->error_sum += pid->error * dt;
    pid->error_sum  = clamp(pid->error_sum, -pid->integral_max, pid->integral_max);
    float i_term = pid->Ki * pid->error_sum;

    /* D — 微分（基于误差变化率） */
    float d_term = pid->Kd * (pid->error - pid->last_error) / dt;

    /* 求和 → 输出限幅 */
    float raw = p_term + i_term + d_term;
    pid->output = clamp(raw, pid->out_min, pid->out_max);

    /* 保存本次状态 */
    pid->integ      = i_term;
    pid->deriv      = d_term;
    pid->last_error = pid->error;

    return pid->output;
}

/* 清零积分 + 历史误差（刹车/切模式时调用） */
void PID_Reset(PID_HandleTypeDef *pid)
{
    pid->error_sum  = 0.0f;
    pid->last_error = 0.0f;
    pid->output     = 0.0f;
    pid->error      = 0.0f;
    pid->integ      = 0.0f;
    pid->deriv      = 0.0f;
}
