/**
 * @file    navi_task.c
 * @brief   导航调度 — 目标坐标 + 位姿 -> 算 v,w -> 底盘速度闭环
 *
 * 数据流:
 *   上位机发 [x,y] -> Navi_SetTarget 存目标
 *   UWB       -> 绝对 (x,y) 主定位源, 无漂移
 *   编码器    -> yaw 朝向累加 (UWB不提供朝向, 必须里程计)
 *              -> x,y 兜底 (UWB掉线时短暂接管)
 *   Navi_Update(每10ms):
 *     计算目标朝向 = atan2(dy, dx)
 *     角度误差    = 朝向 - 当前yaw  (归一化到 ±PI)
 *     距离        = sqrt(dx^2+dy^2)
 *     v = clamp(Kd*dist, 0, MAX_SPEED)
 *     w = clamp(Ka*angle_err, -MAX_OMEGA, +MAX_OMEGA)
 *     -> Chassis_SetTarget(v, w)
 *     到达(距离<阈值) -> 停车
 */

#include "navi_task.h"
#include "debug_config.h"
#include "chassis_task.h"
#include "bsp_callbacks.h"
#include "bsp_encoder.h"
#include "bsp_motor.h"
#include "bsp_uwb.h"
#include "bsp_uart.h"
#include "coord_solver.h"
#include "map_manager.h"
#include "path_planner.h"
#include "bsp_lidar.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* MicroLIB 不定义 M_PI, 手动定义 */
#ifndef M_PI
#define M_PI  3.14159265358979323846f
#endif

/* ==========================================================================
 *  模块状态 (全局 — 调试可见)
 * ========================================================================== */
NaviPose_t pose;                     /* 当前位姿                           */
int32_t    navi_target_x_mm;         /* 目标 X (mm)                        */
int32_t    navi_target_y_mm;         /* 目标 Y (mm)                        */
uint8_t    navi_target_active;       /* 1=有目标, 0=无目标/idle            */
uint8_t    navi_arrived;             /* 1=已到达                           */

/* 10ms 累积, 500ms 上报一次位姿 */
static uint8_t pose_report_tick;

/* 角度误差 EMA — 滤除 UWB 噪声引起的 desired_yaw 高频跳变 */
float navi_angle_err_ema;

/* ==========================================================================
 *  内部工具
 * ========================================================================== */

/* 角度归一化到 [-PI, PI) */
static float norm_angle(float a)
{
    while (a >  M_PI) a -= 2.0f * (float)M_PI;
    while (a < -M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/* UWB 坐标修正: UWB局部坐标 → 世界坐标, 覆盖里程计累积位置 (仅消除漂移, 不影响航向) */
static void uwb_correct(void)
{
    const UWB_State_t *uwb = UWB_GetState();
    static uint32_t last_valid_ms = 0;

    if (uwb->data_valid && uwb->quality >= 20) {
        int32_t wx, wy;
        CoordSolver_Transform(uwb->x_m, uwb->y_m, &wx, &wy);
        pose.x_mm = (float)wx;
        pose.y_mm = (float)wy;
        pose.valid = 1;
        last_valid_ms = HAL_GetTick();
    }

    /* UWB离线>3s → 急停 (编码器兜底累积漂移太大) */
    if (pose.valid && (uint32_t)(HAL_GetTick() - last_valid_ms) > 3000) {
        Chassis_EmergencyStop();
        navi_target_active = 0;
        navi_arrived = 1;
        pose.valid = 0;
    }
}

/* 编码器航迹推算: dt=10ms，x/y 兜底 (yaw 由陀螺仪独立提供) */
static void odom_update(void)
{
    float dt   = 0.01f;
    float vL   = Encoder_GetLeftSpeed_mm_s();
    float vR   = Encoder_GetRightSpeed_mm_s();
    float v    = (vR + vL) * 0.5f;                 /* 车体线速度 mm/s          */
    float ds   = v  * dt;                          /* 本周期移动距离            */

    /* x,y 兜底: UWB有效时会被 uwb_correct() 覆盖，仅在UWB掉线时短暂接管 */
    pose.x_mm += ds * cosf(pose.yaw_rad);
    pose.y_mm += ds * sinf(pose.yaw_rad);
}

/* 陀螺仪 yaw: 直接从 GyroHold 共享变量读取 (GyroHold_Update已积分好) */
static void gyro_yaw_update(void)
{
    pose.yaw_rad = HLD_get_yaw_rad();
}

/* ==========================================================================
 *  对外接口
 * ========================================================================== */

void Navi_Init(void)
{
    memset(&pose, 0, sizeof(pose));
    navi_target_active = 0;
    navi_arrived       = 0;
    pose_report_tick = 0;
    navi_angle_err_ema = 0.0f;
    extern float yaw_angle; extern uint32_t timer;
    yaw_angle = 0.0f;                            /* 陀螺仪 yaw 归零 */
    timer = HAL_GetTick();                       /* dt 基准重校准 */
}

void Navi_SetTarget(int32_t x_mm, int32_t y_mm)
{
    navi_target_x_mm  = x_mm;
    navi_target_y_mm  = y_mm;
    navi_target_active = 1;
    navi_arrived       = 0;
    navi_angle_err_ema = 0.0f;     /* 新目标 — 重置角度EMA */

    /* A* 规划路径 (基于当前UWB位姿) */
    Path_Plan((int32_t)pose.x_mm, (int32_t)pose.y_mm, x_mm, y_mm);

    /* 回传路点至上位机 (0x90帧) */
    {
        uint8_t wc = Path_GetWPCount();
        if (wc > 0) {
            const PpWaypoint_t *wps = Path_GetWPs();
            for (uint8_t i = 0; i < wc && i < PP_MAX_WP; i++) {
                uint8_t pkt[10];
                pkt[0] = i; pkt[1] = wc;
                pkt[2] = (uint8_t)(wps[i].x_mm & 0xFF);
                pkt[3] = (uint8_t)((wps[i].x_mm >> 8) & 0xFF);
                pkt[4] = (uint8_t)((wps[i].x_mm >> 16) & 0xFF);
                pkt[5] = (uint8_t)((wps[i].x_mm >> 24) & 0xFF);
                pkt[6] = (uint8_t)(wps[i].y_mm & 0xFF);
                pkt[7] = (uint8_t)((wps[i].y_mm >> 8) & 0xFF);
                pkt[8] = (uint8_t)((wps[i].y_mm >> 16) & 0xFF);
                pkt[9] = (uint8_t)((wps[i].y_mm >> 24) & 0xFF);
                HostUART_SendPacket(0x90, pkt, 10);
            }
        }
    }
}

/* 设置初始航向 — PID启动前调用, 告诉小车实际朝向 */
void Navi_SetInitialYaw(float yaw_rad)
{
    pose.yaw_rad = yaw_rad;
    navi_angle_err_ema = 0.0f;
}

void Navi_Stop(void)
{
    Chassis_EmergencyStop();
    navi_target_active = 0;
    navi_arrived       = 1;
}

/* 每 10ms 由 TIM4 回调
 * 分两部分: (1) 位姿维护+上报 始终运行; (2) 导航控制 仅 navi_target_active 时运行 */
void Navi_Update(void)
{
    /* ==================== 位姿维护 (始终运行) ==================== */

    /* ① 编码器航迹推算: x/y 兜底 */
    odom_update();

    /* ①B 陀螺仪 yaw: GyroHold 已调 MPU6050_Read_All, 直接读共享变量 */
#if DEBUG_ENABLE_GYRO
    gyro_yaw_update();
#endif

    /* ② UWB 坐标修正: 用绝对坐标覆盖 (x,y), 消漂移 */
    uwb_correct();

    /* ③ 定时上报 2000ms (仅导航启用时输出，降频避免printf阻塞ISR) */
#if DEBUG_ENABLE_NAVI
    pose_report_tick++;
    if (pose_report_tick >= 200) {
        pose_report_tick = 0;
        const UWB_State_t *u = UWB_GetState();
        printf("X=%.2f Y=%.2f Q=%d %s r=%.2f\r\n",
               pose.x_mm * 0.001f, pose.y_mm * 0.001f,
               u->quality,
               pose.valid ? "OK" : "INV",
               u->rmse_m);
    }
#endif

    /* ==================== 导航控制 (仅在有目标时运行) ==================== */
    if (!navi_target_active) return;
    if (navi_arrived)         return;

    /* ④ 获取当前路点 (路径规划已生成) */
    const PpWaypoint_t *wp = Path_CurrentWP();
    if (!wp) {
        /* 路径为空/已走完 → 检查是否到最终目标 */
        float dgx = (float)(navi_target_x_mm) - pose.x_mm;
        float dgy = (float)(navi_target_y_mm) - pose.y_mm;
        if (sqrtf(dgx*dgx + dgy*dgy) < NAVI_ARRIVE_MM) {
            Chassis_EmergencyStop();
            Motor_ActiveBrake(0, 50, 100);            /* 主动刹车锁死 */
            navi_arrived = 1;
        }
        return;
    }

    /* ⑤ 计算到当前路点的向量 */
    float dx = (float)(wp->x_mm) - pose.x_mm;
    float dy = (float)(wp->y_mm) - pose.y_mm;
    float dist = sqrtf(dx * dx + dy * dy);

    /* ⑥ 到达当前路点 → 前进到下一个 */
    if (dist < NAVI_ARRIVE_MM) {
        if (!Path_Advance()) {
            /* 最后一个路点 → 检查最终目标 */
            float dgx = (float)(navi_target_x_mm) - pose.x_mm;
            float dgy = (float)(navi_target_y_mm) - pose.y_mm;
            if (sqrtf(dgx*dgx + dgy*dgy) < NAVI_ARRIVE_MM) {
                Chassis_EmergencyStop();
                Motor_ActiveBrake(0, 50, 100);        /* 主动刹车锁死 */
                navi_arrived = 1;
            }
        }
        /* 立即用新路点重算 */
        wp = Path_CurrentWP();
        if (!wp) return;
        dx = (float)(wp->x_mm) - pose.x_mm;
        dy = (float)(wp->y_mm) - pose.y_mm;
        dist = sqrtf(dx * dx + dy * dy);
    }

    /* ⑦ 朝向 = atan2(dy, dx) → 角度误差 EMA 滤波 (抗UWB噪声) */
    float desired_yaw = atan2f(dy, dx);
    float angle_err   = norm_angle(desired_yaw - pose.yaw_rad);
    navi_angle_err_ema += NAVI_ANGLE_EMA_ALPHA * (angle_err - navi_angle_err_ema);
    float angle_filt  = navi_angle_err_ema;        /* 用滤波后的误差计算 w */

    /* ⑧ P控制: 距离->速度 → 近距离减速防冲过, 角度死区防颤 */
    float v = NAVI_K_DIST * dist;
    if (v > NAVI_MAX_SPEED_MM_S) v = NAVI_MAX_SPEED_MM_S;

    /* 角度死区: |angle_filt|<5.7° → w=0, 直走不颤 */
    float w;
    float abs_filt = (angle_filt > 0.0f) ? angle_filt : -angle_filt;
    if (abs_filt < NAVI_ANGLE_DEADBAND) {
        w = 0.0f;
    } else {
        w = NAVI_K_ANGLE * angle_filt;
    }

    /* 接近路点时减速: 800mm内线性缩放, 到200mm降至最低 */
    if (dist < NAVI_SLOW_DIST_MM) {
        float ratio = (dist - NAVI_ARRIVE_MM) / (NAVI_SLOW_DIST_MM - NAVI_ARRIVE_MM);
        if (ratio < 0.1f) ratio = 0.1f;    /* 最低10%, 不停死 */
        v *= ratio;
    }

    if (w >  NAVI_MAX_OMEGA) w =  NAVI_MAX_OMEGA;
    if (w < -NAVI_MAX_OMEGA) w = -NAVI_MAX_OMEGA;

    /* ⑨ 大角度偏差时减速 (用滤波值判断, 降低误触发) */
    float abs_angle = (angle_filt > 0.0f) ? angle_filt : -angle_filt;
    if (abs_angle > NAVI_SLOW_ANGLE_THRESH) v *= 0.3f;

    /* ⑩ 前向障碍检测 — LIDAR 实时 + 地图静态 双保险 */
    {
        uint8_t obstructed = 0;

        /* ── 地图检测 ── */
        {
            float lookahead = 500.0f;
            int32_t lx = (int32_t)(pose.x_mm + lookahead * cosf(pose.yaw_rad));
            int32_t ly = (int32_t)(pose.y_mm + lookahead * sinf(pose.yaw_rad));
            if (!Map_IsFree(lx, ly)) obstructed = 1;
        }

        /* ── LIDAR 实时检测: 前方 0.15~0.8m, ±0.25m 范围内有点=障碍 ── */
    #if DEBUG_ENABLE_LIDAR
        {
            uint16_t n;
            const LidarPoint_t *pts = Lidar_GetScan(&n);
            if (pts && n > 100) {
                uint16_t danger = 0;
                for (uint16_t i = 0; i < n; i++) {
                    if (!pts[i].valid) continue;
                    if (pts[i].x > 0.15f && pts[i].x < 0.4f &&
                        pts[i].y > -0.25f && pts[i].y < 0.25f) {
                        danger++;
                    }
                }
                if (danger >= 5) obstructed = 1;
            }
        }
    #endif

        if (obstructed) {
            /* 有障碍: 刹停 → 转向目标方向绕过 */
            v = 0.0f;
            float target_yaw = atan2f(dy, dx);
            float escape_err = norm_angle(target_yaw - pose.yaw_rad);
            w = (escape_err > 0.0f) ? NAVI_MAX_OMEGA : -NAVI_MAX_OMEGA;
        }
    }

    Chassis_SetTarget(v, w);
}

const NaviPose_t * Navi_GetPose(void)
{
    return &pose;
}
