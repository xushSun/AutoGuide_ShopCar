/**
 * @file    pid.h
 * @brief   Generic PID controller with anti-windup
 * @note    Float-based, supports both positional and incremental modes
 */

#ifndef __PID_H__
#define __PID_H__

#include "main.h"

/* PID handle structure */
typedef struct {
    /* --- user-config --- */
    float Kp;            /**< Proportional gain          */
    float Ki;            /**< Integral gain              */
    float Kd;            /**< Derivative gain            */
    float dt;            /**< Sample time (seconds)      */
    float out_max;       /**< Output upper clamp         */
    float out_min;       /**< Output lower clamp         */
    float integral_max;  /**< Integral term upper clamp  */

    /* --- internal state --- */
    float setpoint;      /**< Target value               */
    float feedback;      /**< Current measured value     */
    float error;         /**< setpoint - feedback        */
    float error_sum;     /**< Integral accumulator       */
    float last_error;    /**< Previous error for D term  */
    float output;        /**< Last computed output       */
    float deriv;         /**< Derivative term (debug)    */
    float integ;         /**< Integral term (debug)      */
} PID_HandleTypeDef;

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Initialize a PID handle.
 * @param  pid   : pointer to handle
 * @param  kp, ki, kd : gains
 * @param  dt    : sample time in seconds (e.g. 0.01 for 10 ms)
 * @param  out_max, out_min : output saturation limits
 */
void PID_Init(PID_HandleTypeDef *pid,
              float kp, float ki, float kd, float dt,
              float out_max, float out_min);

/**
 * @brief  Set the target value.
 */
void PID_SetTarget(PID_HandleTypeDef *pid, float setpoint);

/**
 * @brief  Compute one PID iteration.
 * @param  pid      : handle (must be initialised)
 * @param  feedback : current measured value
 * @return clipped output value
 * @note   caller should supply actual dt if it differs from pid->dt,
 *         otherwise pass 0 to use the stored dt.
 */
float PID_Compute(PID_HandleTypeDef *pid, float feedback, float dt_override);

/**
 * @brief  Reset integral and last-error; call when disabling the loop.
 */
void PID_Reset(PID_HandleTypeDef *pid);

#endif /* __PID_H__ */
