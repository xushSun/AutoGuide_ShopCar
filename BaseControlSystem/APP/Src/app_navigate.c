/**
 * @file    app_navigate.c
 * @brief   通用路点跟随导航 — 陀螺仪+编码器运动控制, UWB仅到达诊断
 *
 * IDLE → PLAN → DRIVE → ALIGN → DRIVE → ... → ARRIVED
 *
 * 定位源:
 *   UWB:      PLAN时用一次, Path_Plan起点; DRIVE到达双重校验
 *   陀螺仪:   yaw 积分 (GyroHold), ALIGN误差计算, DRIVE锁航向
 *   编码器:   DRIVE段里程判定
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

/* ISR→主循环目标投递 (防数据竞争) */
volatile uint8_t  nav_target_pending;
volatile int32_t  nav_pending_x;
volatile int32_t  nav_pending_y;

volatile uint8_t  g_lidar_turn_open;
volatile float    g_lidar_left_y;
volatile float    g_lidar_right_y;
volatile float    g_lidar_front_m = 99.0f;
volatile float    g_lidar_left_m  = 99.0f;
volatile float    g_lidar_right_m = 99.0f;
volatile uint8_t  g_lidar_assist_state;

/* ════════════════════════════════════════
 *  模块内部状态
 * ════════════════════════════════════════ */
static uint8_t  started;
static int32_t  plan_start_x_mm;
static int32_t  plan_start_y_mm;
static int32_t  logic_x_mm;
static int32_t  logic_y_mm;
static uint8_t  logic_pose_valid;

/* PLAN 预计算: 段距离 + 段索引 */
static float    seg_dists[PP_MAX_WP];   /* seg_dists[i] = 第i段距离(mm) */
static uint8_t  seg_count;             /* 总段数 (= 路点数)            */
static uint8_t  seg_index;             /* 当前段序号 (0-based)         */

/* DRIVE: 目标距离 (从 seg_dists 拷贝) */
static float    seg_target_mm;

/* ALIGN: 目标朝向 (rad) + 超时 */
static float    turn_target_yaw;
static uint32_t align_start_ms;
static uint8_t  nav_heading;          /* 0:+X, 1:+Y, 2:-X, 3:-Y */
static uint8_t  nav_target_heading;
static uint8_t  nav_heading_valid;
static uint8_t  align_split_180;
static uint8_t  align_final_heading;

/* ════════════════════════════════════════
 *  角度归一化 [-PI, PI)
 * ════════════════════════════════════════ */
static float angle_normalize(float a)
{
    while (a >  M_PI) a -= 2.0f * (float)M_PI;
    while (a < -M_PI) a += 2.0f * (float)M_PI;
    return a;
}

static uint8_t heading_from_delta(float dx, float dy)
{
    if (fabsf(dx) >= fabsf(dy)) {
        return (dx >= 0.0f) ? 0 : 2;
    }
    return (dy >= 0.0f) ? 1 : 3;
}

static float turn_delta_to_heading(uint8_t from, uint8_t to)
{
    uint8_t diff = (uint8_t)((to + 4 - from) & 0x03);
    if (diff == 0) return 0.0f;
    if (diff == 1) return -0.5f * (float)M_PI;
    if (diff == 2) return (float)M_PI;
    return 0.5f * (float)M_PI;
}

static void Nav_ApplyLidarAssist(float *v, float *w)
{
#if DEBUG_ENABLE_LIDAR
    float front = g_lidar_front_m;
    float left  = g_lidar_left_m;
    float right = g_lidar_right_m;
    uint32_t now = HAL_GetTick();
    static uint8_t avoid_mode = 0;
    static int8_t avoid_dir = 0;          /* +1=right, -1=left */
    static uint32_t avoid_start_ms = 0;
    static uint8_t hold_released_by_lidar = 0;
    static float smooth_v = NAV_SPEED_MM_S;
    static float smooth_w = 0.0f;
    uint8_t assisted = 0;

    g_lidar_assist_state = 0;

    if (avoid_mode != 0) {
        uint32_t elapsed = (uint32_t)(now - avoid_start_ms);
        g_lidar_assist_state = (uint8_t)(avoid_mode + 1);
        assisted = 1;

        HoldYaw_Release();
        hold_released_by_lidar = 1;

        if (avoid_mode == 1) {
            *v = 0.0f;
            *w = (float)avoid_dir * NAV_LIDAR_ASSIST_W;
            if (elapsed >= NAV_LIDAR_TURN_MS || front > NAV_LIDAR_FRONT_SLOW_M) {
                avoid_mode = 2;
                avoid_start_ms = now;
            }
            goto smooth_output;
        }

        if (avoid_mode == 2) {
            *v = NAV_LIDAR_SLOW_SPEED;
            *w = (float)avoid_dir * (NAV_LIDAR_ASSIST_W * 0.55f);
            if (elapsed >= NAV_LIDAR_BYPASS_MS || front < NAV_LIDAR_FRONT_STOP_M) {
                avoid_mode = 3;
                avoid_start_ms = now;
            }
            goto smooth_output;
        }

        if (avoid_mode == 3) {
            *v = NAV_LIDAR_SLOW_SPEED;
            *w = -(float)avoid_dir * (NAV_LIDAR_ASSIST_W * 0.45f);
            if (elapsed >= NAV_LIDAR_RECOVER_MS) {
                avoid_mode = 0;
                avoid_dir = 0;
                HoldYaw_Lock();
                hold_released_by_lidar = 0;
            }
            goto smooth_output;
        }

        if (avoid_mode == 4) {
            *v = NAV_LIDAR_SLOW_SPEED;
            *w = (float)avoid_dir * NAV_LIDAR_ASSIST_W;
            if (elapsed >= NAV_LIDAR_SIDE_HOLD_MS) {
                if ((avoid_dir > 0 && !(left < NAV_LIDAR_SIDE_BLOCK_M && right > NAV_LIDAR_SIDE_CLEAR_M)) ||
                    (avoid_dir < 0 && !(right < NAV_LIDAR_SIDE_BLOCK_M && left > NAV_LIDAR_SIDE_CLEAR_M))) {
                    avoid_mode = 0;
                    avoid_dir = 0;
                    HoldYaw_Lock();
                    hold_released_by_lidar = 0;
                }
            }
            goto smooth_output;
        }
    }

    if (front < NAV_LIDAR_FRONT_STOP_M) {
        HoldYaw_Release();
        hold_released_by_lidar = 1;
        avoid_dir = (right > left) ? 1 : -1;
        avoid_mode = 1;
        avoid_start_ms = now;
        *v = 0.0f;
        *w = (float)avoid_dir * NAV_LIDAR_ASSIST_W;
        g_lidar_assist_state = 2;
        assisted = 1;
        goto smooth_output;
    }

    if (left < NAV_LIDAR_SIDE_BLOCK_M && right > NAV_LIDAR_SIDE_CLEAR_M) {
        HoldYaw_Release();
        hold_released_by_lidar = 1;
        avoid_dir = 1;
        avoid_mode = 4;
        avoid_start_ms = now;
        *v = NAV_LIDAR_SLOW_SPEED;
        *w = NAV_LIDAR_ASSIST_W;
        g_lidar_assist_state = 5;
        assisted = 1;
        goto smooth_output;
    }

    if (right < NAV_LIDAR_SIDE_BLOCK_M && left > NAV_LIDAR_SIDE_CLEAR_M) {
        HoldYaw_Release();
        hold_released_by_lidar = 1;
        avoid_dir = -1;
        avoid_mode = 4;
        avoid_start_ms = now;
        *v = NAV_LIDAR_SLOW_SPEED;
        *w = -NAV_LIDAR_ASSIST_W;
        g_lidar_assist_state = 5;
        assisted = 1;
        goto smooth_output;
    }

    if (front < NAV_LIDAR_FRONT_SLOW_M) {
        if (*v > NAV_LIDAR_SLOW_SPEED) *v = NAV_LIDAR_SLOW_SPEED;
        g_lidar_assist_state = 1;
        assisted = 1;
        if (hold_released_by_lidar) {
            HoldYaw_Lock();
            hold_released_by_lidar = 0;
        }
    } else if (hold_released_by_lidar) {
        HoldYaw_Lock();
        hold_released_by_lidar = 0;
    }

smooth_output:
    if (assisted) {
        float dv = *v - smooth_v;
        float dw = *w - smooth_w;
        if (dv >  NAV_LIDAR_RAMP_V_STEP) dv =  NAV_LIDAR_RAMP_V_STEP;
        if (dv < -NAV_LIDAR_RAMP_V_STEP) dv = -NAV_LIDAR_RAMP_V_STEP;
        if (dw >  NAV_LIDAR_RAMP_W_STEP) dw =  NAV_LIDAR_RAMP_W_STEP;
        if (dw < -NAV_LIDAR_RAMP_W_STEP) dw = -NAV_LIDAR_RAMP_W_STEP;
        smooth_v += dv;
        smooth_w += dw;
        *v = smooth_v;
        *w = smooth_w;
    } else {
        smooth_v = *v;
        smooth_w = *w;
    }
#else
    (void)v;
    (void)w;
#endif
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

static int32_t lane_center_y(uint8_t lane)
{
    if (lane == 1) return 950;
    if (lane == 2) return 4200;
    if (lane == 3) return 7350;
    return 950;
}

static void Nav_GetPlanningStart(int32_t *sx, int32_t *sy)
{
    int32_t x = logic_pose_valid ? logic_x_mm : 1000;
    int32_t y = logic_pose_valid ? logic_y_mm : 950;
    uint8_t lane = nav_pose.valid ? detect_aisle(nav_pose.y_mm) : detect_aisle((float)y);
    uint8_t in_main = 0;

    if (x < 0) x = 0;
    if (x > 9270) x = 9270;

    if (nav_pose.valid && nav_pose.x_mm >= 8500.0f) in_main = 1;
    if (x >= 8500) in_main = 1;

    if (in_main) {
        *sx = 9270;
        *sy = y;
        return;
    }

    if (lane == 0) {
        lane = detect_aisle((float)nav_target_y_mm);
        if (lane == 0) lane = 1;
    }

    *sx = x;
    *sy = lane_center_y(lane);
}

/* ════════════════════════════════════════
 *  UWB EMA 滤波器 (保持运行, 运动控制不读)
 * ════════════════════════════════════════ */
static float    wx_ema, wy_ema;
static uint8_t  ema_init;
static uint32_t last_valid_ms;

void Nav_ResetFilter(void)
{
    wx_ema = 0.0f;
    wy_ema = 0.0f;
    ema_init = 0;
    last_valid_ms = 0;
}

static void Nav_UpdatePose(void)
{
    const UWB_State_t *u = UWB_GetState();
    uint32_t now = HAL_GetTick();

    if (u->data_valid && u->quality >= 20) {
        int32_t wx, wy;
        CoordSolver_Transform(u->x_m, u->y_m, &wx, &wy);
        float fx = (float)wx;
        float fy = (float)wy;

        if (ema_init) {
            float dx = fx - wx_ema;
            float dy = fy - wy_ema;
            float jump = sqrtf(dx * dx + dy * dy);
            if (jump <= NAV_UWB_JUMP_MM) {
                wx_ema += NAV_UWB_EMA_ALPHA * (fx - wx_ema);
                wy_ema += NAV_UWB_EMA_ALPHA * (fy - wy_ema);
            }
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
 *  状态切换
 * ════════════════════════════════════════ */
static void Nav_EnterState(NavState_t st)
{
    nav_state = st;

    switch (st) {

    case NAV_STATE_PLAN:
        /* Path_Plan 在 Update 中执行 (依赖当前 UWB 位姿) */
        break;

    case NAV_STATE_DRIVE:
        /* 直行: 编码器归零 + 锁航向 */
        Chassis_PID_Enable(1);
        Encoder_ResetOdom();
        HoldYaw_Lock();
        seg_target_mm = seg_dists[seg_index];
        if (seg_target_mm < 200.0f) {
            seg_target_mm = 0.0f;
        }
        break;

    case NAV_STATE_ALIGN: {
        /* 原地转弯: 解除锁航向, 纯开环转 */
        Chassis_PID_Enable(1);
        HoldYaw_Release();
        /* 路点间几何算目标朝向 (假设车朝+X yaw=0, Nav_SetTarget 已 GyroYaw_Reset) */
        {
            const PpWaypoint_t *wps = Path_GetWPs();
            float dx, dy;
            if (seg_index == 0) {
                /* 第一段: wp[0] 方向 (假设起点原点) */
                dx = (float)wps[0].x_mm - (float)plan_start_x_mm;
                dy = (float)wps[0].y_mm - (float)plan_start_y_mm;
            } else {
                /* 后续段: 路点间几何方向 */
                dx = (float)wps[seg_index].x_mm - (float)wps[seg_index - 1].x_mm;
                dy = (float)wps[seg_index].y_mm - (float)wps[seg_index - 1].y_mm;
            }
            nav_target_heading = heading_from_delta(dx, dy);
            {
                uint8_t diff = (uint8_t)((nav_target_heading + 4 - nav_heading) & 0x03);
                if (diff == 2) {
                    align_split_180 = 1;
                    align_final_heading = nav_target_heading;
                    nav_target_heading = (uint8_t)((nav_heading + 1) & 0x03);
                } else {
                    align_split_180 = 0;
                    align_final_heading = nav_target_heading;
                }
            }
            turn_target_yaw = turn_delta_to_heading(nav_heading, nav_target_heading);
            GyroYaw_Reset();
            printf("TURN: h=%d->%d dyaw=%.0f split=%d final=%d\r\n",
                   nav_heading, nav_target_heading, turn_target_yaw * 57.29578f,
                   align_split_180, align_final_heading);
        }
        align_start_ms = HAL_GetTick();
        break;
    }

    case NAV_STATE_ARRIVED:
        Chassis_EmergencyStop();
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
    seg_count         = 0;
    seg_index         = 0;
    turn_target_yaw   = 0.0f;
    align_start_ms    = 0;
    nav_heading       = 0;
    nav_target_heading = 0;
    nav_heading_valid = 1;
    align_split_180   = 0;
    align_final_heading = 0;
    logic_x_mm        = 1000;
    logic_y_mm        = 950;
    logic_pose_valid  = 1;
    memset(seg_dists, 0, sizeof(seg_dists));
}

/**
 * @brief  设定目标, 触发导航
 */
void Nav_SetTarget(int32_t x_mm, int32_t y_mm)
{
    nav_target_x_mm  = x_mm;
    nav_target_y_mm  = y_mm;
    nav_arrived      = 0;

    /* 每次下发目标前, 车头按 +X 方向摆放 */
    if (!nav_heading_valid) {
        GyroYaw_Reset();
        nav_heading = 0;
        nav_target_heading = 0;
        nav_heading_valid = 1;
    }
    /* 重置 UWB EMA 滤波器 */
    Nav_ResetFilter();

    /* PID 使能 + 进入规划 */
    Chassis_PID_Enable(1);
    Nav_EnterState(NAV_STATE_PLAN);
}

/**
 * @brief  主循环 50ms 调用
 */
void APP_Navigate_Update(void)
{
    Nav_UpdatePose();   /* 保持 UWB 滤波器运行 (运动控制不读它) */

    /* 首次调用: 打印提示 */
    if (!started) {
        started = 1;
        printf("NAV: IDLE, waiting target\r\n");
    }

    /* IDLE 或已到达 → 不动作 */
    if (nav_state == NAV_STATE_IDLE || nav_arrived)
        return;

    switch (nav_state) {

    /* ── PLAN: 路径规划 (一帧完成) → 直接 DRIVE ── */
    case NAV_STATE_PLAN: {
        int32_t sx, sy;
        Nav_GetPlanningStart(&sx, &sy);
        plan_start_x_mm = sx;
        plan_start_y_mm = sy;
        uint8_t wc = Path_Plan(sx, sy, nav_target_x_mm, nav_target_y_mm);

        if (wc == 0) {
            printf("NAV: plan FAIL\r\n");
            Nav_EnterState(NAV_STATE_ARRIVED);
            break;
        }

        /* 预计算所有段距离 */
        {
            const PpWaypoint_t *wps = Path_GetWPs();
            seg_dists[0] = sqrtf((float)((int32_t)wps[0].x_mm - sx) * (wps[0].x_mm - sx)
                               + (float)((int32_t)wps[0].y_mm - sy) * (wps[0].y_mm - sy));
            for (uint8_t i = 1; i < wc; i++) {
                float dx = (float)wps[i].x_mm - (float)wps[i-1].x_mm;
                float dy = (float)wps[i].y_mm - (float)wps[i-1].y_mm;
                seg_dists[i] = sqrtf(dx * dx + dy * dy);
            }
            seg_count = wc;
            seg_index = 0;
            /* 清零未使用段, 防止 DRIVE 读到垃圾值 */
            for (uint8_t i = wc; i < PP_MAX_WP; i++)
                seg_dists[i] = 0.0f;
        }

        printf("NAV: plan ok, start=(%d,%d), %d segs, 1st=%.0fmm\r\n",
               (int)sx, (int)sy, seg_count, seg_dists[0]);
        {
            printf("SEG: ");
            for (uint8_t i = 0; i < seg_count; i++)
                printf("%.0f ", seg_dists[i]);
            printf("\r\n");
        }
        Nav_EnterState(NAV_STATE_ALIGN);
        break;
    }

    /* ── DRIVE: 直行 — 编码器里程 + 陀螺锁航向 + UWB双重校验 ── */
    case NAV_STATE_DRIVE: {
        if (seg_target_mm <= 0.0f) {
            seg_index++;
            Path_Advance();
            if (seg_index >= seg_count) {
                Nav_EnterState(NAV_STATE_ARRIVED);
            } else {
                Nav_EnterState(NAV_STATE_ALIGN);
            }
            break;
        }

        float v = NAV_SPEED_MM_S;
        float w = 0.0f;  /* 陀螺修正已在 Chassis_ControlLoop 占空比层做 */

        float traveled = (Encoder_GetLeftOdom_mm() + Encoder_GetRightOdom_mm()) * 0.5f;
        float remain   = seg_target_mm - traveled;

        /* 接近目标时线性减速 */
        if (remain < NAV_SLOW_DIST_MM && remain > 0.0f) {
            float ratio = remain / NAV_SLOW_DIST_MM;
            if (ratio < NAV_MIN_SPEED_RATIO) ratio = NAV_MIN_SPEED_RATIO;
            v *= ratio;
        }

        /* 最终目标检查: 任何段中接近最终目标 → 停车 */
        if (nav_pose.valid) {
            float dx_final = (float)nav_target_x_mm - nav_pose.x_mm;
            float dy_final = (float)nav_target_y_mm - nav_pose.y_mm;
            float df = sqrtf(dx_final * dx_final + dy_final * dy_final);
            {
                static uint8_t fc = 0;
                if (++fc >= 20) { fc = 0;
                    printf("FINAL: d=%.0f uwb=(%d,%d) tgt=(%d,%d)\r\n",
                           df, (int)nav_pose.x_mm, (int)nav_pose.y_mm,
                           (int)nav_target_x_mm, (int)nav_target_y_mm);
                }
            }
            if (df < NAV_FINAL_ARRIVE_MM) {
                logic_x_mm = nav_target_x_mm;
                logic_y_mm = nav_target_y_mm;
                logic_pose_valid = 1;
                Chassis_SetTarget(0.0f, 0.0f);
                Nav_EnterState(NAV_STATE_ARRIVED);
                break;
            }
        }

        /* 到达判定: 编码器 + UWB 双重校验 */
        {
            static uint8_t diag_cnt = 0;
            if (++diag_cnt >= 10) {
                diag_cnt = 0;
                printf("DRIVE: trav=%.0f tgt=%.0f seg=%d/%d\r\n",
                       traveled, seg_target_mm, seg_index + 1, seg_count);
            }
        }
        uint8_t arr = 0;
        if (traveled >= seg_target_mm) {
            arr = 1;
        }

        if (arr) {
            const PpWaypoint_t *wp_done = Path_CurrentWP();
            if (wp_done) {
                logic_x_mm = wp_done->x_mm;
                logic_y_mm = wp_done->y_mm;
                logic_pose_valid = 1;
            }
            Chassis_EmergencyStop();
            seg_index++;
            Path_Advance();
            if (seg_index >= seg_count) {
                Nav_EnterState(NAV_STATE_ARRIVED);
            } else {
                Nav_EnterState(NAV_STATE_ALIGN);
            }
        } else {
            Nav_ApplyLidarAssist(&v, &w);
            Chassis_SetTarget(v, w);
        }
        break;
    }

    /* ── ALIGN: 原地转弯 — 纯陀螺仪 yaw, 路点几何算方向 ── */
    case NAV_STATE_ALIGN: {
        float gyro_yaw  = HLD_get_yaw_rad();
        float err       = angle_normalize(turn_target_yaw - gyro_yaw);
        float abs_err   = fabsf(err);

        /* 到位: 进入直行 */
        if (abs_err < NAV_ALIGN_DONE_RAD) {
            Chassis_EmergencyStop();
            nav_heading = nav_target_heading;
            if (align_split_180) {
                Chassis_PID_Enable(1);
                HoldYaw_Release();
                nav_target_heading = align_final_heading;
                turn_target_yaw = turn_delta_to_heading(nav_heading, nav_target_heading);
                align_split_180 = 0;
                GyroYaw_Reset();
                align_start_ms = HAL_GetTick();
                printf("TURN: h=%d->%d dyaw=%.0f split=2 final=%d\r\n",
                       nav_heading, nav_target_heading,
                       turn_target_yaw * 57.29578f, align_final_heading);
                break;
            }
            Nav_EnterState(NAV_STATE_DRIVE);
            break;
        }

        float w = (err > 0.0f) ? NAV_TURN_SPEED_RAD_S : -NAV_TURN_SPEED_RAD_S;

        /* 接近目标时减速 */
        if (abs_err < NAV_ALIGN_SLOW_RAD) {
            float ratio = abs_err / NAV_ALIGN_SLOW_RAD;
            if (ratio < NAV_ALIGN_MIN_RATIO) ratio = NAV_ALIGN_MIN_RATIO;
            w *= ratio;
        }

        Chassis_SetTarget(0.0f, w);

        /* 超时保护: >10s 放弃转弯直接进直行 */
        if ((uint32_t)(HAL_GetTick() - align_start_ms) > 10000) {
            printf("NAV: ALIGN timeout, skip turn\r\n");
            Chassis_EmergencyStop();
            nav_heading = nav_target_heading;
            Nav_EnterState(NAV_STATE_DRIVE);
        }
        break;
    }

    default:
        break;
    }
}

void APP_Navigate_Stop(void)
{
    Chassis_EmergencyStop();
    nav_arrived = 1;
    nav_state   = NAV_STATE_ARRIVED;
}
