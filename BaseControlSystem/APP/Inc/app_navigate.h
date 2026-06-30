/**
 * @file    app_navigate.h
 * @brief   通用路点跟随导航 — UWB+编码器+陀螺仪五状态机
 *
 * IDLE → PLAN → ALIGN → DRIVE → (回ALIGN) → ARRIVED
 */

#ifndef __APP_NAVIGATE_H__
#define __APP_NAVIGATE_H__

#include "main.h"

/* ════════════════════════════════════════════
 *  速度参数
 * ════════════════════════════════════════════ */
#define NAV_SPEED_MM_S          200.0f     /* 直行速度 mm/s                 */
#define NAV_TURN_SPEED_RAD_S    0.6f       /* 转弯角速度 rad/s              */
#define NAV_SLOW_DIST_MM        800.0f     /* 减速距离 mm                   */
#define NAV_MIN_SPEED_RATIO     0.2f       /* 最低速度比例                  */

/* ════════════════════════════════════════════
 *  UWB 容差 / 滤波
 * ════════════════════════════════════════════ */
#define NAV_UWB_ARRIVE_MM       400.0f     /* 路点到达容差 mm               */
#define NAV_FINAL_ARRIVE_MM     500.0f     /* 最终目标容差 mm               */
#define NAV_UWB_JUMP_MM         400.0f     /* UWB 跳变拒绝阈值 mm           */
#define NAV_UWB_EMA_ALPHA       0.3f       /* UWB EMA 平滑系数              */
#define NAV_UWB_TIMEOUT_MS      1500       /* UWB 丢包超时 ms               */

/* ════════════════════════════════════════════
 *  转弯参数
 * ════════════════════════════════════════════ */
#define NAV_ALIGN_DONE_RAD      0.05f      /* 转弯完成阈值 rad (2.9°)       */
#define NAV_ALIGN_SLOW_RAD      0.30f      /* 转弯减速阈值 rad (17.2°)      */
#define NAV_ALIGN_MIN_RATIO     0.25f      /* 转弯最低速度比例              */

/* ════════════════════════════════════════════
 *  过道判定 (保留, 仅诊断用)
 * ════════════════════════════════════════════ */
#define MAIN_AISLE_X_MIN        8200.0f    /* 主过道 X > 8200               */
#define AISLE1_Y_MAX            3000.0f    /* 过道1: Y 0~3000               */
#define AISLE2_Y_MIN            3500.0f    /* 过道2: Y 3500~5500            */
#define AISLE2_Y_MAX            5500.0f
#define AISLE3_Y_MIN            6200.0f    /* 过道3: Y 6200~9000            */

/* ════════════════════════════════════════════
 *  导航状态机
 * ════════════════════════════════════════════ */
typedef enum {
    NAV_STATE_IDLE = 0,       /* 等待 SetTarget, PID 关                   */
    NAV_STATE_PLAN,           /* Path_Plan 运行中 (一帧完成)               */
    NAV_STATE_ALIGN,          /* 原地转弯, 朝向路点                        */
    NAV_STATE_DRIVE,          /* 直行, 陀螺锁航向, 编码器+UWB 判定到达     */
    NAV_STATE_ARRIVED         /* 刹车, nav_arrived=1                       */
} NavState_t;

/* ════════════════════════════════════════════
 *  位姿
 * ════════════════════════════════════════════ */
typedef struct {
    float    x_mm;
    float    y_mm;
    float    yaw_rad;
    uint8_t  valid;
} NavPose_t;

/* ════════════════════════════════════════════
 *  对外接口
 * ════════════════════════════════════════════ */
void APP_Navigate_Init(void);
void APP_Navigate_Update(void);               /* 主循环 50ms 调用           */
void APP_Navigate_Stop(void);

/**
 * @brief  设定目标坐标, 触发导航 (IDLE/ARRIVED → PLAN)
 * @param  x_mm, y_mm : 世界坐标 (mm)
 */
void Nav_SetTarget(int32_t x_mm, int32_t y_mm);

/* ════════════════════════════════════════════
 *  全局变量 (main.c 诊断 / Keil Watch)
 * ════════════════════════════════════════════ */
extern NavPose_t   nav_pose;
extern NavState_t  nav_state;
extern uint8_t     nav_arrived;
extern int32_t     nav_target_x_mm;
extern int32_t     nav_target_y_mm;

/* LIDAR (main.c 写入, 导航只读) */
extern volatile uint8_t  g_lidar_turn_open;
extern volatile float    g_lidar_left_y;
extern volatile float    g_lidar_right_y;

#endif
