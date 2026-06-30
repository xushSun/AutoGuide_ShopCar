/**
 * @file    navi_task.h
 * @brief   导航调度 — 收目标坐标 -> 结合UWB/里程计 -> 算v,w -> 底盘闭环
 *
 * 位姿来源: UWB 提供绝对(x,y)修正, 编码器累加里程计算航向和局部位移
 *
 * 可调参数:
 *   NAVI_ARRIVE_MM       到达判定距离(mm), 进入此范围即停车
 *   NAVI_MAX_SPEED_MM_S  最大线速度
 *   NAVI_MAX_OMEGA       最大角速度(rad/s)
 *   NAVI_K_DIST          距离->速度 P 系数
 *   NAVI_K_ANGLE         角度误差->角速度 P 系数
 */

#ifndef __NAVI_TASK_H__
#define __NAVI_TASK_H__

#include "main.h"

/* 到达判定 (mm) */
#define NAVI_ARRIVE_MM          500.0f   /* 到达判定(mm) */
#define NAVI_SLOW_DIST_MM       2000.0f   /* 2m内线性减速 */

/* 速度限制 */
#define NAVI_MAX_SPEED_MM_S     100.0f     /* 100mm/s 慢速稳定 */
#define NAVI_MAX_OMEGA          0.6f       /* rad/s */

/* 导航 P 系数 */
#define NAVI_K_DIST             1.0f       /* 距离→速度 */
#define NAVI_K_ANGLE            1.0f       /* 角度差->角速度 */
#define NAVI_ANGLE_EMA_ALPHA    0.10f      /* 角度误差EMA α — 强滤波抗UWB抖动 */
#define NAVI_ANGLE_DEADBAND      0.10f     /* rad(5.7°) — 小于此角度w=0, 防颤 */
#define NAVI_SLOW_ANGLE_THRESH  1.5f       /* rad(86°) — 超过此角度才减速 */

/* 位姿状态 */
typedef struct {
    float x_mm;           /* 当前 X (mm)     */
    float y_mm;           /* 当前 Y (mm)     */
    float yaw_rad;        /* 当前航向 (rad)  */
    uint8_t valid;        /* 1=有定位数据    */
} NaviPose_t;

/* ── 调试全局变量 (Keil Watch 直接可见) ── */
extern NaviPose_t pose;                     /* 当前位姿 (世界坐标 mm) */
extern int32_t    navi_target_x_mm;         /* 目标 X (mm)           */
extern int32_t    navi_target_y_mm;         /* 目标 Y (mm)           */
extern uint8_t    navi_target_active;       /* 1=有目标              */
extern uint8_t    navi_arrived;             /* 1=已到达              */
extern float      navi_angle_err_ema;       /* 角度误差 EMA          */

void Navi_Init(void);
void Navi_SetTarget(int32_t x_mm, int32_t y_mm);
void Navi_Stop(void);
void Navi_Update(void);                     /* 每10ms由TIM4回调 */
const NaviPose_t * Navi_GetPose(void);

#endif
