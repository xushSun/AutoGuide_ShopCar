/**
 * @file    chassis_task.c
 * @brief   底盘速度闭环 — 10ms周期由TIM4中断驱动
 *
 * 数据流：
 *   目标(v,ω) → 逆运动学 → 左右轮速目标 → PID → PWM占空比 → 电机
 *   编码器 → RPM换算 → 左右轮速反馈 → PID ┘
 */

#include "chassis_task.h"
#include "pid.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "kinematics.h"

/* --------------------------------------------------------------------------
 *  模块内部状态
 * -------------------------------------------------------------------------- */

static PID_HandleTypeDef pid_left;       /* 左轮速度PID                      */
static PID_HandleTypeDef pid_right;      /* 右轮速度PID                      */
static WheelSpeed_t target_wheel;        /* 目标轮速(mm/s) — 由逆运动学算出  */
static WheelSpeed_t actual_wheel;        /* 当前轮速(mm/s) — 编码器反馈      */
static int8_t        control_active;     /* 1=闭环运行, 0=停车               */
static int8_t        pid_enabled;        /* 1=PID开启, 0=关闭 (默认关)       */
static Velocity_t    actual_body;        /* 当前车体速度 (Getter用)          */
static float         left_trim = LEFT_WHEEL_TRIM;  /* 左轮补偿倍率         */
int16_t g_duty_L;                         /* PID左轮输出 (诊断)              */
int16_t g_duty_R;                         /* PID右轮输出 (诊断)              */

/* --------------------------------------------------------------------------
 *  对外接口
 * -------------------------------------------------------------------------- */

/* 初始化：双轮PID + 编码器归零 + 电机PWM启动（占空比0，刹车态） */
void Chassis_Init(void)
{
    PID_Init(&pid_left,  SPEED_KP, SPEED_KI, SPEED_KD, SPEED_DT,
             (float)MOTOR_PWM_MAX, (float)-MOTOR_PWM_MAX);
    PID_Init(&pid_right, SPEED_KP, SPEED_KI, SPEED_KD, SPEED_DT,
             (float)MOTOR_PWM_MAX, (float)-MOTOR_PWM_MAX);

    Encoder_Init();
    Motor_Init();

    control_active = 0;
    pid_enabled    = 0;     /* PID 默认关闭, 开环模式 */
    actual_body.v = 0.0f;
    actual_body.w = 0.0f;
}

/* 设定目标速度，同时激活闭环 */
void Chassis_SetTarget(float v, float w)
{
    Kinematics_Inverse(v, w, &target_wheel);
    target_wheel.left *= left_trim;             /* 左轮补偿 */

    PID_SetTarget(&pid_left,  target_wheel.left);
    PID_SetTarget(&pid_right, target_wheel.right);

    control_active = 1;
}

/* 急停：刹车 + PID积分清零 */
void Chassis_EmergencyStop(void)
{
    Motor_Brake();
    PID_Reset(&pid_left);
    PID_Reset(&pid_right);
    control_active = 0;
    target_wheel.left  = 0.0f;
    target_wheel.right = 0.0f;
}

/* 10ms控制迭代（TIM4中断 → bsp_callbacks → 这里） */
void Chassis_ControlLoop(void)
{
    if (!control_active) return;
    if (!pid_enabled)     return;

    /* ① 读编码器 → 轮速(mm/s) */
    actual_wheel.left  = Encoder_GetLeftSpeed_mm_s();
    actual_wheel.right = Encoder_GetRightSpeed_mm_s();

    /* ② PID计算 → PWM占空比 */
    int16_t duty_left  = (int16_t)PID_Compute(&pid_left,  actual_wheel.left,  0.0f);
    int16_t duty_right = (int16_t)PID_Compute(&pid_right, actual_wheel.right, 0.0f);
    g_duty_L = duty_left;
    g_duty_R = duty_right;

    /* ③ → 电机 */
    Motor_SetBoth(duty_left, duty_right);

    /* ④ 正运动学 → 车体速度（遥测用） */
    Kinematics_Forward(&actual_wheel, &actual_body);
}

/* 获取当前车体速度 */
Velocity_t Chassis_GetVelocity(void)
{
    return actual_body;
}

/* PID 使能开关 */
void Chassis_PID_Enable(uint8_t en)
{
    pid_enabled = (en != 0) ? 1 : 0;
    if (en) control_active = 1;
}

/* 运行时调节左轮平衡 (1.0=平衡, >1.0=左轮加速, <1.0=左轮减速) */
void Chassis_SetLeftTrim(float factor)
{
    if (factor < 0.5f)  factor = 0.5f;
    if (factor > 1.5f)  factor = 1.5f;
    left_trim = factor;
}
