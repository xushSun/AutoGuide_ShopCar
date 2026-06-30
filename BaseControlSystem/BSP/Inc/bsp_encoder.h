/**
 * @file    bsp_encoder.h
 * @brief   Hall encoder speed & odometry
 * @note    TIM2 (32-bit, left), TIM3 (16-bit, right) – encoder mode
 *          Call Encoder_Update() from TIM4 ISR (every 10 ms = 100 Hz)
 */

#ifndef __BSP_ENCODER_H__
#define __BSP_ENCODER_H__

#include "main.h"

/* --- Vehicle physical constants (user-tune) -------------------------------- */
#define ENCODER_PPR             11      /**< Pulses per rev (motor shaft)     */
#define WHEEL_DIAMETER_MM       67.5f   /**< Wheel diameter in mm            */
#define GEAR_RATIO              90.0f   /**< Motor : Wheel reduction ratio   */

/* --- Derived (do not edit) ------------------------------------------------- */
#define ENCODER_CPR             (ENCODER_PPR * 4)   /* 4× encoding          */
#define WHEEL_CIRCUMFERENCE_MM  (3.14159265358979f * WHEEL_DIAMETER_MM)

/* 左编码器方向翻转: INA1/INA2反接后电机物理转向反了, 编码器需同步翻 */
#define ENCODER_LEFT_INVERT     1   /**< 1=左编码器取反, 0=不动 */

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Reset encoder counts to zero (run once before control loop).
 */
void Encoder_Init(void);

/**
 * @brief  Sample encoder counters, compute RPM & odometry.
 * @param  dt_s : sample period in seconds (e.g. 0.01f for 10 ms)
 * @note   Call from TIM4_IRQHandler or bsp_callbacks.
 */
void Encoder_Update(float dt_s);

/**
 * @brief  Get left wheel speed in RPM.
 */
float Encoder_GetLeftRPM(void);

/**
 * @brief  Get right wheel speed in RPM.
 */
float Encoder_GetRightRPM(void);

/**
 * @brief  Get left wheel speed in mm/s (linear at ground).
 */
float Encoder_GetLeftSpeed_mm_s(void);

/**
 * @brief  Get right wheel speed in mm/s (linear at ground).
 */
float Encoder_GetRightSpeed_mm_s(void);

/**
 * @brief  Get cumulative left distance in mm (odometry).
 */
float Encoder_GetLeftOdom_mm(void);

/**
 * @brief  Get cumulative right distance in mm (odometry).
 */
float Encoder_GetRightOdom_mm(void);

/**
 * @brief  Reset odometry distances to zero.
 */
void Encoder_ResetOdom(void);

#endif /* __BSP_ENCODER_H__ */
