/**
 * @file    coord_solver.c
 * @brief   坐标解算器 — UWB局部 → 世界 完整2D仿射变换 (6参数)
 *
 * 三锚标定: A(8880,0) B(20,0) C(9660,8930) mm
 * 实测UWB: A(8.85,0.00) B(-0.72,1.00) C(8.95,9.35) m
 *
 * 解 6 元线性方程组得:
 *   W_x = ax*U_x + bx*U_y + cx
 *   W_y = ay*U_x + by*U_y + cy
 */

#include "coord_solver.h"
#include <math.h>
#include <string.h>

static CoordSolver_t solver;

/* ── 3x3 矩阵求逆 (辅助函数) ── */
static int mat3_inverse(const float m[9], float inv[9])
{
    float det = m[0]*(m[4]*m[8] - m[5]*m[7])
              - m[1]*(m[3]*m[8] - m[5]*m[6])
              + m[2]*(m[3]*m[7] - m[4]*m[6]);
    if (fabsf(det) < 1e-6f) return 0;
    float id = 1.0f / det;
    inv[0] = (m[4]*m[8] - m[5]*m[7]) * id;
    inv[1] = (m[2]*m[7] - m[1]*m[8]) * id;
    inv[2] = (m[1]*m[5] - m[2]*m[4]) * id;
    inv[3] = (m[5]*m[6] - m[3]*m[8]) * id;
    inv[4] = (m[0]*m[8] - m[2]*m[6]) * id;
    inv[5] = (m[2]*m[3] - m[0]*m[5]) * id;
    inv[6] = (m[3]*m[7] - m[4]*m[6]) * id;
    inv[7] = (m[1]*m[6] - m[0]*m[7]) * id;
    inv[8] = (m[0]*m[4] - m[1]*m[3]) * id;
    return 1;
}

void CoordSolver_Init(void)
{
    memset(&solver, 0, sizeof(solver));
    solver.configured = 0;
}

void CoordSolver_SetAnchors(int16_t ax_mm, int16_t ay_mm,
                            int16_t bx_mm, int16_t by_mm,
                            int16_t cx_mm, int16_t cy_mm,
                            float a_uwb_x, float a_uwb_y,
                            float b_uwb_x, float b_uwb_y,
                            float c_uwb_x, float c_uwb_y)
{
    solver.anchor_A_world.x_mm = ax_mm;
    solver.anchor_A_world.y_mm = ay_mm;
    solver.anchor_B_world.x_mm = bx_mm;
    solver.anchor_B_world.y_mm = by_mm;
    solver.anchor_C_world.x_mm = cx_mm;
    solver.anchor_C_world.y_mm = cy_mm;
    solver.anchor_A_uwb.x_m    = a_uwb_x;
    solver.anchor_A_uwb.y_m    = a_uwb_y;
    solver.anchor_B_uwb.x_m    = b_uwb_x;
    solver.anchor_B_uwb.y_m    = b_uwb_y;
    solver.anchor_C_uwb.x_m    = c_uwb_x;
    solver.anchor_C_uwb.y_m    = c_uwb_y;

    /*
     * 构建方程: M * [a,b,c]^T = [Wx_A, Wx_B, Wx_C]^T
     *   M = | Ux_A  Uy_A  1 |
     *       | Ux_B  Uy_B  1 |
     *       | Ux_C  Uy_C  1 |
     */
    float M[9] = {
        a_uwb_x, a_uwb_y, 1.0f,
        b_uwb_x, b_uwb_y, 1.0f,
        c_uwb_x, c_uwb_y, 1.0f,
    };
    float Minv[9];
    if (!mat3_inverse(M, Minv)) {
        /* 奇异: 退化为无变换 */
        solver.ax = 1000.0f; solver.bx = 0.0f;    solver.cx = 0.0f;
        solver.ay = 0.0f;    solver.by = 1000.0f; solver.cy = 0.0f;
        solver.configured = 1;
        return;
    }

    /* W_x = ax*Ux + bx*Uy + cx */
    solver.ax = Minv[0]*(float)ax_mm + Minv[1]*(float)bx_mm + Minv[2]*(float)cx_mm;
    solver.bx = Minv[3]*(float)ax_mm + Minv[4]*(float)bx_mm + Minv[5]*(float)cx_mm;
    solver.cx = Minv[6]*(float)ax_mm + Minv[7]*(float)bx_mm + Minv[8]*(float)cx_mm;

    /* W_y = ay*Ux + by*Uy + cy */
    solver.ay = Minv[0]*(float)ay_mm + Minv[1]*(float)by_mm + Minv[2]*(float)cy_mm;
    solver.by = Minv[3]*(float)ay_mm + Minv[4]*(float)by_mm + Minv[5]*(float)cy_mm;
    solver.cy = Minv[6]*(float)ay_mm + Minv[7]*(float)by_mm + Minv[8]*(float)cy_mm;

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

    /* 6 参数仿射: 输入 UWB (m), 输出世界 (mm) */
    *world_x_mm = (int32_t)(solver.ax * local_x_m + solver.bx * local_y_m + solver.cx);
    *world_y_mm = (int32_t)(solver.ay * local_x_m + solver.by * local_y_m + solver.cy);
}

const CoordSolver_t * CoordSolver_GetState(void)
{
    return &solver;
}
