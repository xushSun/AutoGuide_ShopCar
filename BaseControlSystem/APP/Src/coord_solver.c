/**
 * @file    coord_solver.c
 * @brief   坐标解算器 — UWB局部 → 世界 2D仿射变换
 *
 * 已知 A/B 两锚点在 UWB 系和世界系中的坐标:
 *   世界: A(8880,0), B(20,0)   mm
 *   UWB:  A(8.88,0), B(0.20,0) m
 *
 * 从 A→B 方向解出 X 比例:
 *   sx = (A_w.x - B_w.x) / ((A_u.x - B_u.x) * 1000) = 8860/8680 ≈ 1.0207
 *   tx = A_w.x - sx * A_u.x * 1000 = 8880 - 1.0207*8880 ≈ -184
 *
 * 验证: B: 1.0207*200 + (-184) = 204 - 184 = 20 ✓
 *       A: 1.0207*8880 + (-184) = 9064 - 184 = 8880 ✓
 *
 * for Y: both AB on y=0, 假设 sy=1.0 (已用C锚验证: 8930/8930=1.0)
 */

#include "coord_solver.h"
#include "bsp_uwb.h"
#include <math.h>
#include <string.h>

static CoordSolver_t solver;

void CoordSolver_Init(void)
{
    memset(&solver, 0, sizeof(solver));
    solver.configured = 0;
}

void CoordSolver_SetAnchors(int16_t ax_mm, int16_t ay_mm,
                            int16_t bx_mm, int16_t by_mm,
                            float a_uwb_x, float a_uwb_y,
                            float b_uwb_x, float b_uwb_y)
{
    solver.anchor_A_world.x_mm = ax_mm;
    solver.anchor_A_world.y_mm = ay_mm;
    solver.anchor_B_world.x_mm = bx_mm;
    solver.anchor_B_world.y_mm = by_mm;
    solver.anchor_A_uwb.x_m    = a_uwb_x;
    solver.anchor_A_uwb.y_m    = a_uwb_y;
    solver.anchor_B_uwb.x_m    = b_uwb_x;
    solver.anchor_B_uwb.y_m    = b_uwb_y;

    /* X轴: A→B方向比例 */
    float duwb_x = (b_uwb_x - a_uwb_x) * 1000.0f;   /* UWB Δx in mm */
    float dw_x   = (float)(bx_mm - ax_mm);           /* World Δx in mm */

    if (fabsf(duwb_x) < 1.0f) {
        solver.sx = 1.0f;
        solver.tx = 0.0f;
    } else {
        solver.sx = dw_x / duwb_x;                   /* 8860/8680 ≈ 1.0207 */
        solver.tx = (float)ax_mm - solver.sx * a_uwb_x * 1000.0f;
    }

    /* Y轴: 比例 (AB都在y=0无法校准, 假设1.0, C锚已验证 8930/8930=1) */
    float duwb_y = (b_uwb_y - a_uwb_y) * 1000.0f;
    float dw_y   = (float)(by_mm - ay_mm);

    if (fabsf(duwb_y) < 1.0f && fabsf(dw_y) < 1.0f) {
        solver.sy = 1.0f;
        solver.ty = 0.0f;
    } else if (fabsf(duwb_y) > 1.0f) {
        solver.sy = dw_y / duwb_y;
        solver.ty = (float)ay_mm - solver.sy * a_uwb_y * 1000.0f;
    } else {
        solver.sy = 1.0f;
        solver.ty = 0.0f;
    }

    solver.configured = 1;
}

void CoordSolver_Transform(float local_x_m, float local_y_m,
                           int32_t *world_x_mm, int32_t *world_y_mm)
{
    if (!solver.configured) {
        *world_x_mm = 0;
        *world_y_mm = 0;
        return;
    }

    float xL_mm = local_x_m * 1000.0f;   /* UWB m → mm */
    float yL_mm = local_y_m * 1000.0f;

    /* 仿射变换: W = s * P + t */
    *world_x_mm = (int32_t)(solver.sx * xL_mm + solver.tx);
    *world_y_mm = (int32_t)(solver.sy * yL_mm + solver.ty);
}

const CoordSolver_t * CoordSolver_GetState(void)
{
    return &solver;
}
