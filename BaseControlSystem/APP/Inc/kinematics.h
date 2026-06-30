/**
 * @file    kinematics.h
 * @brief   Two-wheel differential-drive kinematics
 * @note    Forward:  wheel speeds (mm/s) → body velocity (mm/s, rad/s)
 *          Inverse:   body velocity (mm/s, rad/s) → wheel speeds (mm/s)
 */

#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "main.h"

/* Vehicle geometry (user-tune) */
#define TRACK_WIDTH_MM  150.0f      /**< Distance between two wheels (mm)   */
#define WHEEL_RADIUS_MM 33.75f      /**< Wheel radius (mm) – should match
                                          WHEEL_DIAMETER_MM / 2 in encoder   */

/* Velocity container */
typedef struct {
    float v;        /**< Linear velocity  (mm/s)                            */
    float w;        /**< Angular velocity (rad/s)                           */
} Velocity_t;

/* Wheel speed container */
typedef struct {
    float left;     /**< Left  wheel linear speed (mm/s)                    */
    float right;    /**< Right wheel linear speed (mm/s)                    */
} WheelSpeed_t;

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

/**
 * @brief  Inverse kinematics:  body velocity → wheel speeds.
 * @param  v  : linear  velocity (mm/s)
 * @param  w  : angular velocity (rad/s)
 * @param  out: pointer to result
 */
void Kinematics_Inverse(float v, float w, WheelSpeed_t *out);

/**
 * @brief  Forward kinematics:  wheel speeds → body velocity.
 * @param  in : left & right wheel speeds (mm/s)
 * @param  out: pointer to result
 */
void Kinematics_Forward(const WheelSpeed_t *in, Velocity_t *out);

#endif /* __KINEMATICS_H__ */
