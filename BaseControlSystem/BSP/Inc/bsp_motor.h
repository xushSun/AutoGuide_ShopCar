/**
 * @file    bsp_motor.h
 * @brief   TB6612 dual H-bridge motor driver
 * @note    Direction via PB12-15 GPIO, speed via TIM1 CH1/CH2 PWM (20 kHz)
 */

#ifndef __BSP_MOTOR_H__
#define __BSP_MOTOR_H__

#include "main.h"

/* PWM resolution: TIM1 ARR = 5000-1, so valid range = 0 .. 4999 */
#define MOTOR_PWM_MAX   4999

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Start PWM output on both channels.
 * @note   Called once after CubeMX init. PWM starts at 0% duty.
 */
void Motor_Init(void);

/**
 * @brief  Set left motor speed.
 * @param  duty : -MOTOR_PWM_MAX .. +MOTOR_PWM_MAX
 *         positive = forward, negative = reverse, 0 = brake
 */
void Motor_SetLeft(int16_t duty);

/**
 * @brief  Set right motor speed.
 */
void Motor_SetRight(int16_t duty);

/**
 * @brief  Convenience: set both motors at once.
 * @note   Slightly faster than two separate calls (single HAL GPIO write).
 */
void Motor_SetBoth(int16_t left_duty, int16_t right_duty);

/**
 * @brief  Brake both motors (coast to stop).
 */
void Motor_Brake(void);

/**
 * @brief  Active brake — short reverse pulse to kill residual momentum,
 *         then hard brake.  Eliminates the coasting at low speed.
 * @param  channel : 0 = both, 1 = left only, 2 = right only
 * @param  strength_pct : reverse pulse strength, 1~100 (recommend 30~50)
 * @param  duration_ms   : reverse pulse duration, 10~200ms (recommend 80)
 */
void Motor_ActiveBrake(uint8_t channel, uint8_t strength_pct, uint16_t duration_ms);

#endif /* __BSP_MOTOR_H__ */
