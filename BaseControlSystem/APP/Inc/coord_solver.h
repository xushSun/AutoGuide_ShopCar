/**
 * @file    coord_solver.h
 * @brief   坐标解算器 — UWB局部坐标系 → 超市世界坐标系 (完整2D仿射变换)
 *
 * 用 A/B/C 三个锚点的实测 UWB 坐标做 6 参数仿射标定:
 *   W_x = a*U_x + b*U_y + c
 *   W_y = d*U_x + e*U_y + f
 *
 * 可处理 UWB 坐标系与世界坐标系之间的旋转+缩放+平移。
 */

#ifndef __COORD_SOLVER_H__
#define __COORD_SOLVER_H__

#include "main.h"
#include <stdint.h>

/* 世界坐标点 (单位: mm) */
typedef struct {
    int32_t x_mm;
    int32_t y_mm;
} WorldPoint_t;

/* UWB坐标点 (单位: 米, float) */
typedef struct {
    float x_m;
    float y_m;
} UWBPoint_t;

/* 坐标解算器状态 */
typedef struct {
    WorldPoint_t anchor_A_world;
    WorldPoint_t anchor_B_world;
    WorldPoint_t anchor_C_world;
    UWBPoint_t   anchor_A_uwb;
    UWBPoint_t   anchor_B_uwb;
    UWBPoint_t   anchor_C_uwb;
    /* 6 参数仿射: W = M * U + T */
    float ax, bx, cx;   /* W_x = ax*U_x + bx*U_y + cx */
    float ay, by, cy;   /* W_y = ay*U_x + by*U_y + cy */
    uint8_t configured;
} CoordSolver_t;

/* ── Public API ──────────────────────────────── */

void CoordSolver_Init(void);

/**
 * @brief  设定三个锚点 (UWB实测 + 世界), 预计算6参数仿射
 * @param  ax/ay_mm : 锚点A世界坐标 (mm)
 * @param  bx/by_mm : 锚点B世界坐标 (mm)
 * @param  cx/cy_mm : 锚点C世界坐标 (mm)
 * @param  a_uwb_x/y: 锚点A实测UWB坐标 (m)
 * @param  b_uwb_x/y: 锚点B实测UWB坐标 (m)
 * @param  c_uwb_x/y: 锚点C实测UWB坐标 (m)
 */
void CoordSolver_SetAnchors(int16_t ax_mm, int16_t ay_mm,
                            int16_t bx_mm, int16_t by_mm,
                            int16_t cx_mm, int16_t cy_mm,
                            float a_uwb_x, float a_uwb_y,
                            float b_uwb_x, float b_uwb_y,
                            float c_uwb_x, float c_uwb_y);

/**
 * @brief  UWB局部坐标 → 世界坐标 (6参数仿射)
 */
void CoordSolver_Transform(float local_x_m, float local_y_m,
                           int32_t *world_x_mm, int32_t *world_y_mm);

const CoordSolver_t * CoordSolver_GetState(void);

#endif /* __COORD_SOLVER_H__ */
