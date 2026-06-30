/**
 * @file    kinematics.c
 * @brief   两轮差速运动学 — 正解/逆解（单位:mm, rad）
 *
 * 模型：
 *   正解：  v = (vL+vR)/2       ω = (vR−vL)/L
 *   逆解：  vL = v − ω·L/2      vR = v + ω·L/2
 */

#include "kinematics.h"

#define HALF_TRACK  (TRACK_WIDTH_MM * 0.5f)   /* 半轮距 L/2 */

/* ==========================================================================
 *  逆解：车体(v,ω) → 左右轮速(mm/s)
 * ========================================================================== */
void Kinematics_Inverse(float v, float w, WheelSpeed_t *out)
{
    float half_wL = w * HALF_TRACK;    /* ω·L/2 */
    out->left  = v - half_wL;
    out->right = v + half_wL;
}

/* ==========================================================================
 *  正解：左右轮速(mm/s) → 车体(v,ω)
 * ========================================================================== */
void Kinematics_Forward(const WheelSpeed_t *in, Velocity_t *out)
{
    out->v = (in->right + in->left) * 0.5f;
    out->w = (in->right - in->left) / TRACK_WIDTH_MM;
}
