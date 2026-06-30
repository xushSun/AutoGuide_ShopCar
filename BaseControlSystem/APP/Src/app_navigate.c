/**
 * @file    app_navigate.c
 * @brief   通用路点跟随导航 — UWB+编码器+陀螺仪五状态机
 *
 * IDLE → PLAN → ALIGN → DRIVE → (回 ALIGN) → ARRIVED
 *
 * 定位源:
 *   UWB 绝对 (x,y) + EMA 平滑 + 跳变过滤
 *   陀螺仪 yaw 积分 (GyroHold)
 *   编码器测距 (DRIVE 段)
 *
 * 仅从主循环调用(非 ISR)。
 */

#include "Header.h"
#include "app_navigate.h"
#include "bsp_callbacks.h"
#include "bsp_encoder.h"
#include "coord_solver.h"
#include "chassis_task.h"
#include "bsp_uwb.h"
#include "path_planner.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

/* ════════════════════════════════════════
 *  全局变量
 * ════════════════════════════════════════ */
NavPose_t   nav_pose;
int32_t     nav_target_x_mm = 0;
int32_t     nav_target_y_mm = 0;
uint8_t     nav_arrived;
NavState_t  nav_state;
uint8_t     nav_current_aisle;

volatile uint8_t  g_lidar_turn_open;
volatile float    g_lidar_left_y;
volatile float    g_lidar_right_y;

/* ════════════════════════════════════════
 *  模块内部状态
 * ════════════════════════════════════════ */
static uint8_t  started;
static float    turn_target_yaw;       /* ALIGN: 目标朝向 (rad)    */
static float    seg_target_mm;         /* DRIVE: 本段预计距离 (mm) */
static uint8_t  seg_is_final;          /* DRIVE: 1=最后路点        */
static float    seg_dx, seg_dy;        /* DRIVE 进入时到路点向量   */

/* ════════════════════════════════════════
 *  角度归一化 [-PI, PI)
 * ════════════════════════════════════════ */
static float angle_normalize(float a)
{
    while (a >  M_PI) a -= 2.0f * (float)M_PI;
    while (a < -M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* ════════════════════════════════════════
 *  过道检测 (诊断用)
 * ════════════════════════════════════════ */
static uint8_t detect_aisle(float y_mm)
{
    if (y_mm <= AISLE1_Y_MAX) return 1;
    if (y_mm >= AISLE2_Y_MIN && y_mm <= AISLE2_Y_MAX) return 2;
    if (y_mm >= AISLE3_Y_MIN) return 3;
    return 0;
}

/* ════════════════════════════════════════
 *  位姿更新 (UWB 跳变过滤 + EMA 平滑)
 * ════════════════════════════════════════ */
static void Nav_UpdatePose(void)
{
    const UWB_State_t *u = UWB_GetState();
    static float    wx_ema = 0.0f, wy_ema = 0.0f;
    static uint8_t  ema_init = 0;
    static uint32_t last_valid_ms = 0;
    uint32_t now = HAL_GetTick();

    if (u->data_valid && u->quality >= 20) {
        int32_t wx, wy;
        CoordSolver_Transform(u->x_m, u->y_m, &wx, &wy);
        float fx = (float)wx;
        float fy = (float)wy;

        if (ema_init) {
            /* 跳变检测: 新坐标与 EMA 值差 >400mm → 丢弃 */
            float dx = fx - wx_ema;
            float dy = fy - wy_ema;
            float jump = sqrtf(dx * dx + dy * dy);
            if (jump <= NAV_UWB_JUMP_MM) {
                wx_ema += NAV_UWB_EMA_ALPHA * (fx - wx_ema);
                wy_ema += NAV_UWB_EMA_ALPHA * (fy - wy_ema);
            }
            /* else: 跳变过大, 保持 EMA 值 */
        } else {
            wx_ema = fx;
            wy_ema = fy;
            ema_init = 1;
        }

        nav_pose.x_mm = wx_ema;
        nav_pose.y_mm = wy_ema;
        nav_pose.valid = 1;
        last_valid_ms = now;
    } else if (nav_pose.valid && (uint32_t)(now - last_valid_ms) < NAV_UWB_TIMEOUT_MS) {
        /* 短暂丢包: 保持最后有效值 */
    } else {
        nav_pose.valid = 0;
    }

    nav_pose.yaw_rad  = HLD_get_yaw_rad();
    nav_current_aisle = detect_aisle(nav_pose.y_mm);
}

/* ════════════════════════════════════════
 *  状态切换 (仅做机械初始化, 不递归)
 * ════════════════════════════════════════ */
static void Nav_EnterState(NavState_t st)
{
    nav_state = st;

    switch (st) {

    case NAV_STATE_PLAN:
        /* Path_Plan 在 Update 中执行 (依赖当前位姿) */
        break;

    case NAV_STATE_ALIGN:
        /* 转弯前解除锁航向, 纯开环转 */
        HoldYaw_Release();
        turn_target_yaw = 0.0f;              /* 由 Update 第一帧计算 */
        break;

    case NAV_STATE_DRIVE: {
        /* 直行段初始化: 编码器归零 + 锁航向 + 算距离 */
        const PpWaypoint_t *wp = Path_CurrentWP();
        if (!wp) {
            Nav_EnterState(NAV_STATE_ARRIVED);
            break;
        }
        Encoder_ResetOdom();
        HoldYaw_Lock();
        seg_dx = (float)wp->x_mm - nav_pose.x_mm;
        seg_dy = (float)wp->y_mm - nav_pose.y_mm;
        seg_target_mm = sqrtf(seg_dx * seg_dx + seg_dy * seg_dy);
        seg_is_final  = (Path_GetWPCount() > 0
                     && wp == &Path_GetWPs()[Path_GetWPCount() - 1]) ? 1 : 0;
        break;
    }

    case NAV_STATE_ARRIVED:
        Chassis_SetTarget(0.0f, 0.0f);
        Chassis_PID_Enable(0);
        nav_arrived = 1;
        break;

    default:
        break;
    }
}

/* ════════════════════════════════════════
 *  对外接口
 * ════════════════════════════════════════ */

void APP_Navigate_Init(void)
{
    memset(&nav_pose, 0, sizeof(nav_pose));
    nav_arrived       = 0;
    started           = 0;
    nav_state         = NAV_STATE_IDLE;
    nav_current_aisle = 0;
    seg_target_mm     = 0.0f;
    seg_is_final      = 0;
    seg_dx            = 0.0f;
    seg_dy            = 0.0f;
    turn_target_yaw   = 0.0f;
}

/**
 * @brief  设定目标, 触发导航
 */
void Nav_SetTarget(int32_t x_mm, int32_t y_mm)
{
    nav_target_x_mm  = x_mm;
    nav_target_y_mm  = y_mm;
    nav_arrived      = 0;

    /* 陀螺 yaw 归零: atan2f 算的是世界坐标系角度, yaw 须同步归零 */
    GyroYaw_Reset();

    /* PID 使能 + 进入规划 */
    Chassis_PID_Enable(1);
    Nav_EnterState(NAV_STATE_PLAN);
}

/**
 * @brief  主循环 50ms 调用
 */
void APP_Navigate_Update(void)
{
    Nav_UpdatePose();

    /* 首次调用: 打印提示 */
    if (!started) {
        started = 1;
        printf("NAV: IDLE, waiting target\r\n");
    }

    /* IDLE 或已到达 → 不动作 */
    if (nav_state == NAV_STATE_IDLE || nav_arrived)
        return;

    switch (nav_state) {

    /* ── PLAN: 路径规划 (一帧完成) ── */
    case NAV_STATE_PLAN: {
        int32_t sx = nav_pose.valid ? (int32_t)nav_pose.x_mm : 0;
        int32_t sy = nav_pose.valid ? (int32_t)nav_pose.y_mm : 0;
        uint8_t wc = Path_Plan(sx, sy, nav_target_x_mm, nav_target_y_mm);

        if (wc > 0) {
            printf("NAV: plan ok, %d waypoints\r\n", wc);
            Nav_EnterState(NAV_STATE_ALIGN);
        } else {
            printf("NAV: plan FAIL\r\n");
            Nav_EnterState(NAV_STATE_ARRIVED);
        }
        break;
    }

    /* ── ALIGN: 原地转弯 ── */
    case NAV_STATE_ALIGN: {
        const PpWaypoint_t *wp = Path_CurrentWP();
        if (!wp) {
            Nav_EnterState(NAV_STATE_ARRIVED);
            break;
        }

        /* UWB有效时才更新目标朝向, 丢包时保持上次值 */
        if (nav_pose.valid) {
            float dx = (float)wp->x_mm - nav_pose.x_mm;
            float dy = (float)wp->y_mm - nav_pose.y_mm;
            turn_target_yaw = atan2f(dy, dx);
        }

        float err     = angle_normalize(turn_target_yaw - nav_pose.yaw_rad);
        float abs_err = fabsf(err);

        /* 到位: 进入直行 */
        if (abs_err < NAV_ALIGN_DONE_RAD) {
            Chassis_SetTarget(0.0f, 0.0f);
            Nav_EnterState(NAV_STATE_DRIVE);
            break;
        }

        /* 计算转弯角速度: sign(err) × 0.6 rad/s */
        float w = (err > 0.0f) ? NAV_TURN_SPEED_RAD_S : -NAV_TURN_SPEED_RAD_S;

        /* 接近目标时减速, 最低 25% */
        if (abs_err < NAV_ALIGN_SLOW_RAD) {
            float ratio = abs_err / NAV_ALIGN_SLOW_RAD;
            if (ratio < NAV_ALIGN_MIN_RATIO) ratio = NAV_ALIGN_MIN_RATIO;
            w *= ratio;
        }

        Chassis_SetTarget(0.0f, w);
        break;
    }

    /* ── DRIVE: 直行 + 陀螺锁航向 ── */
    case NAV_STATE_DRIVE: {
        const PpWaypoint_t *wp = Path_CurrentWP();
        if (!wp) {
            Chassis_SetTarget(0.0f, 0.0f);
            Nav_EnterState(NAV_STATE_ARRIVED);
            break;
        }

        float v = NAV_SPEED_MM_S;
        float w = GyroHold_ComputeW();            /* 陀螺仪锁航向修正 */

        float traveled = (Encoder_GetLeftOdom_mm() + Encoder_GetRightOdom_mm()) * 0.5f;
        float remain   = seg_target_mm - traveled;

        /* 接近目标时线性减速 */
        if (remain < NAV_SLOW_DIST_MM && remain > 0.0f) {
            float ratio = remain / NAV_SLOW_DIST_MM;
            if (ratio < NAV_MIN_SPEED_RATIO) ratio = NAV_MIN_SPEED_RATIO;
            v *= ratio;
        }

        /* ── 到达判定 ── */
        uint8_t arr = 0;
        float arrive_tol = seg_is_final ? NAV_FINAL_ARRIVE_MM : NAV_UWB_ARRIVE_MM;

        /* 编码器 ≥ 80% 后启用 UWB 判定 */
        if (traveled >= seg_target_mm * 0.8f) {
            if (nav_pose.valid) {
                float dx_uwb = (float)wp->x_mm - nav_pose.x_mm;
                float dy_uwb = (float)wp->y_mm - nav_pose.y_mm;
                if (sqrtf(dx_uwb * dx_uwb + dy_uwb * dy_uwb) < arrive_tol)
                    arr = 1;
            }
        }

        /* 编码器 120% 兜底: 强制到达防卡死 */
        if (traveled >= seg_target_mm * 1.2f)
            arr = 1;

        if (arr) {
            Chassis_SetTarget(0.0f, 0.0f);
            if (Path_Advance()) {
                /* 还有下一个路点 → 转向 */
                Nav_EnterState(NAV_STATE_ALIGN);
            } else {
                /* 全部到达 */
                Nav_EnterState(NAV_STATE_ARRIVED);
            }
        } else {
            Chassis_SetTarget(v, w);
        }
        break;
    }

    default:
        break;
    }
}

void APP_Navigate_Stop(void)
{
    Chassis_SetTarget(0.0f, 0.0f);
    Chassis_PID_Enable(0);
    nav_arrived = 1;
    nav_state   = NAV_STATE_ARRIVED;
}
