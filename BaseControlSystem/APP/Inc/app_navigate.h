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
#define NAV_MIN_SPEED_RATIO     0.25f       /* 最低速度比例                  */

/* ════════════════════════════════════════════
 *  UWB 容差 / 滤波
 * ════════════════════════════════════════════ */
#define NAV_UWB_ARRIVE_MM       580.0f     /* 路点到达容差 mm               */
#define NAV_FINAL_ARRIVE_MM     350.0f     /* 最终目标容差 mm (UWB)       */
#define NAV_UWB_JUMP_MM         2000.0f    /* UWB 跳变拒绝阈值 mm           */
#define NAV_UWB_EMA_ALPHA       0.3f       /* UWB EMA 平滑系数              */
#define NAV_UWB_TIMEOUT_MS      1500       /* UWB 丢包超时 ms               */

/* ════════════════════════════════════════════
 *  转弯参数
 * ════════════════════════════════════════════ */
#define NAV_ALIGN_DONE_RAD      0.087f     /* 转弯完成阈值 rad (约5°)       */
#define NAV_ALIGN_SLOW_RAD      0.30f      /* 转弯减速阈值 rad (17.2°)      */
#define NAV_ALIGN_MIN_RATIO     0.25f      /* 转弯最低速度比例              */

/* LIDAR辅助避障: 只在DRIVE直行阶段限速/微调, 不接管主导航 */
#define NAV_LIDAR_FRONT_STOP_M  0.42f      /* 前方小于此距离先停住          */
#define NAV_LIDAR_FRONT_SLOW_M  0.70f      /* 前方小于此距离降速            */
#define NAV_LIDAR_SLOW_SPEED    80.0f      /* 避障降速后的最大直线速度 mm/s */
#define NAV_LIDAR_ASSIST_W      0.28f      /* 左右微调角速度 rad/s          */
#define NAV_LIDAR_SIDE_MARGIN_M 0.08f      /* 左右距离差超过此值才微调      */
#define NAV_LIDAR_TIMEOUT_MS    350        /* 雷达帧超时后才认为无障碍      */
#define NAV_LIDAR_SIDE_BLOCK_M  0.36f      /* 单侧小于此距离认为贴近障碍    */
#define NAV_LIDAR_SIDE_CLEAR_M  0.55f      /* 单侧大于此距离认为可偏过去    */
#define NAV_LIDAR_SIDE_HOLD_MS  650        /* 单侧避障至少保持一小段时间    */
#define NAV_LIDAR_TURN_MS       450        /* 正前方堵住时先转出的时间      */
#define NAV_LIDAR_BYPASS_MS     900        /* 绕障低速前进时间              */
#define NAV_LIDAR_RECOVER_MS    500        /* 反向恢复航向时间              */
#define NAV_LIDAR_RAMP_V_STEP   35.0f      /* 每次导航更新最大速度变化 mm/s */
#define NAV_LIDAR_RAMP_W_STEP   0.06f      /* 每次导航更新最大角速度变化    */

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

/* ISR→主循环目标投递 (防数据竞争) */
extern volatile uint8_t  nav_target_pending;
extern volatile int32_t  nav_pending_x;
extern volatile int32_t  nav_pending_y;

/* LIDAR (main.c 写入, 导航只读) */
extern volatile uint8_t  g_lidar_turn_open;
extern volatile float    g_lidar_left_y;
extern volatile float    g_lidar_right_y;
extern volatile float    g_lidar_front_m;
extern volatile float    g_lidar_left_m;
extern volatile float    g_lidar_right_m;
extern volatile uint8_t  g_lidar_assist_state;

#endif
