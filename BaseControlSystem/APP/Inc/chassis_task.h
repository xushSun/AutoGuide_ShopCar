/**
 * @file    chassis_task.h
 * @brief   Chassis velocity control loop
 * @note    One SpeedPID per wheel. Called from TIM4 ISR (100 Hz → 10 ms).
 *          API: set target (v, ω) → control loop auto-converts → wheel PID → motor.
 */

#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "kinematics.h"       /* Velocity_t, WheelSpeed_t */

/* ---- wheel speed limit (mm/s, ground speed) ---- */
#define WHEEL_SPEED_MAX_MM_S    500.0f   /**< Max absolute wheel linear speed */

/* ---- PID gains (PID 输出直接 = PWM 占空比) ---- */
#define SPEED_KP    20.0f
#define SPEED_KI    5.0f       /* 降低积分防积分饱和引起振荡 */
#define SPEED_KD    1.5f
#define SPEED_DT    0.01f               /**< 10 ms sample time               */

/* ---- 左轮平衡补偿 (右轮=基准1.0, >1.0左轮加速, <1.0左轮减速) ---- */
#define LEFT_WHEEL_TRIM        1.00f    /**< 左轮目标速度倍率, 补偿右轮偏快 (165vs182mm/s@50%) */

/* ---- 开环占空比估算 (仅 Motor_SetBoth 直驱时参考, PID 路径不使用) ---- */
#define SPEED_TO_DUTY_SCALE   12.0f     /**< duty ≈ mm/s * scale, capped 0~4999 */

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialise both wheel PIDs, link encoder, start motor PWM.
 */
void Chassis_Init(void);

/**
 * @brief  Set target body velocity.
 * @param  v : linear  velocity (mm/s)
 * @param  w : angular velocity (rad/s)
 */
void Chassis_SetTarget(float v, float w);

/**
 * @brief  Emergency stop – brake motors, reset PIDs.
 */
void Chassis_EmergencyStop(void);

/**
 * @brief  Enable / disable PID control loop.
 */
void Chassis_PID_Enable(uint8_t en);

/**
 * @brief  Main 10 ms control iteration.
 * @note   Call from HAL_TIM_PeriodElapsedCallback (TIM4).
 *         ① read encoder wheel speeds
 *         ② compute PID → duty
 *         ③ output to motors
 */
void Chassis_ControlLoop(void);

/**
 * @brief  Get current body velocity (odometry).
 */
Velocity_t Chassis_GetVelocity(void);

/**
 * @brief  Adjust left wheel trim at runtime (default = LEFT_WHEEL_TRIM).
 * @param  factor : >1.0 = left faster, <1.0 = left slower, 1.0 = balanced
 */
void Chassis_SetLeftTrim(float factor);

/* PID 输出占空比 (主循环诊断用, ISR写入) */
extern int16_t g_duty_L;
extern int16_t g_duty_R;

#endif /* __CHASSIS_TASK_H__ */
