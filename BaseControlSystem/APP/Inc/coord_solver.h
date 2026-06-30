/**
 * @file    coord_solver.h
 * @brief   坐标解算器 — UWB局部坐标系 → 超市世界坐标系 (仿射变换)
 *
 * 已知两个锚点在UWB系和世界系中的坐标，求解仿射变换参数(sx, sy, tx, ty)。
 * UWB输出(xL, yL)单位米, 世界坐标单位mm。
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
    UWBPoint_t   anchor_A_uwb;
    UWBPoint_t   anchor_B_uwb;
    float sx;                     /* X比例: (dw/dx_uwb) mm/m  */
    float sy;                     /* Y比例: (dw/dy_uwb) mm/m  */
    float tx;                     /* X平移 mm                 */
    float ty;                     /* Y平移 mm                 */
    uint8_t configured;
} CoordSolver_t;

/* ── Public API ──────────────────────────────── */

void CoordSolver_Init(void);

/**
 * @brief  设定锚点对 (UWB + 世界), 预计算仿射参数
 * @param  ax_mm, ay_mm : 锚点A世界坐标 (mm)
 * @param  bx_mm, by_mm : 锚点B世界坐标 (mm)
 * @param  a_uwb_x/y    : 锚点A在UWB局部系中的坐标 (m)
 * @param  b_uwb_x/y    : 锚点B在UWB局部系中的坐标 (m)
 */
void CoordSolver_SetAnchors(int16_t ax_mm, int16_t ay_mm,
                            int16_t bx_mm, int16_t by_mm,
                            float a_uwb_x, float a_uwb_y,
                            float b_uwb_x, float b_uwb_y);

/**
 * @brief  UWB局部坐标 → 世界坐标 (仿射变换)
 */
void CoordSolver_Transform(float local_x_m, float local_y_m,
                           int32_t *world_x_mm, int32_t *world_y_mm);

const CoordSolver_t * CoordSolver_GetState(void);

#endif /* __COORD_SOLVER_H__ */
