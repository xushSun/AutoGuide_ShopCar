/**
 * @file    bsp_encoder.c
 * @brief   霍尔编码器 — 轮速RPM + 里程累计
 *
 * 硬件: TIM2(左,32位) TIM3(右,16位) 编码器模式, 4倍频
 * 物理: 编码器11PPR × 减速比90:1 × 轮径67.5mm
 * 调用: Encoder_Update(0.01f) 每10ms由TIM4中断触发
 */

#include "bsp_encoder.h"
#include "tim.h"

/* ==========================================================================
 *  模块静态变量
 * ========================================================================== */
static int32_t  raw_last_left;       /* 上周期TIM2->CNT (32位)              */
static uint16_t raw_last_right;      /* 上周期TIM3->CNT (16位)              */

static float rpm_left;               /* 左轮转速 RPM                       */
static float rpm_right;              /* 右轮转速 RPM                       */
float speed_left_mm_s;        /* 左轮线速度 mm/s                    */
float speed_right_mm_s;       /* 右轮线速度 mm/s                    */
static float odom_left_mm;           /* 左轮累计行程 mm                    */
static float odom_right_mm;          /* 右轮累计行程 mm                    */

/* 预计算系数（Init时算好，Update里乘就行） */
static float pulses_to_rev;          /* 1 / (PPR×4)                        */
static float rev_to_dist_mm;         /* 轮周长 / 减速比                     */
static float pulses_to_dist_mm;      /* 每脉冲对应 mm  (=rev_to_dist/pulses) */

/* ==========================================================================
 *  对外接口
 * ========================================================================== */

/* 初始化：启动编码器定时器、读当前CNT归零基准，预计算转换系数 */
void Encoder_Init(void)
{
    /* 启动编码器模式计数 (CubeMX Init只配置, 不启动!) */
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    /* 启动后CNT可能非零，以此为基准 */
    raw_last_left  = (int32_t)TIM2->CNT;
    raw_last_right = (uint16_t)TIM3->CNT;

    rpm_left  = 0.0f;   rpm_right = 0.0f;
    speed_left_mm_s  = 0.0f;   speed_right_mm_s = 0.0f;
    odom_left_mm  = 0.0f;      odom_right_mm = 0.0f;

    pulses_to_rev  = 1.0f / (float)ENCODER_CPR;        /* 1 / (PPR×4)    */
    rev_to_dist_mm = WHEEL_CIRCUMFERENCE_MM / GEAR_RATIO;
    pulses_to_dist_mm = rev_to_dist_mm * pulses_to_rev;
}

/**
 * @brief  读编码器计数值，算出RPM、线速度、里程
 * @param  dt_s  采样周期(秒), 如 0.01f
 * @note   16位计数器溢出用int16差值自动处理
 */
void Encoder_Update(float dt_s)
{
    if (dt_s <= 0.0f) return;

    /* ① 读当前计数值 */
    int32_t  cnt_left  = (int32_t)TIM2->CNT;
    uint16_t cnt_right = (uint16_t)TIM3->CNT;

    /* ② 差值（32位自然翻转，16位需手动处理） */
    int32_t delta_left  = cnt_left - raw_last_left;
    int32_t delta_right = (int32_t)((int16_t)(cnt_right - raw_last_right));

    raw_last_left  = cnt_left;
    raw_last_right = cnt_right;

    /* ③ 脉冲数 → 圈数 */
    float rev_left  = (float)delta_left  * pulses_to_rev;
    float rev_right = (float)delta_right * pulses_to_rev;

    /* ④ RPM = 圈/采样周期 × 60 */
    rpm_left  = rev_left  / dt_s * 60.0f;
    rpm_right = rev_right / dt_s * 60.0f;

    /* ⑤ 线速度 mm/s = 圈数×每圈路程 / 采样周期 */
    speed_left_mm_s  = rev_left  * rev_to_dist_mm / dt_s;
    speed_right_mm_s = rev_right * rev_to_dist_mm / dt_s;

#if ENCODER_LEFT_INVERT
    speed_left_mm_s  = -speed_left_mm_s;
    rpm_left         = -rpm_left;
#endif

    /* ⑥ 里程累加 */
#if ENCODER_LEFT_INVERT
    odom_left_mm  -= (float)delta_left  * pulses_to_dist_mm;
#else
    odom_left_mm  += (float)delta_left  * pulses_to_dist_mm;
#endif
    odom_right_mm += (float)delta_right * pulses_to_dist_mm;
}

/* ---- getter ---- */
float Encoder_GetLeftRPM(void)          { return rpm_left; }
float Encoder_GetRightRPM(void)         { return rpm_right; }
float Encoder_GetLeftSpeed_mm_s(void)   { return speed_left_mm_s; }
float Encoder_GetRightSpeed_mm_s(void)  { return speed_right_mm_s; }
float Encoder_GetLeftOdom_mm(void)      { return odom_left_mm; }
float Encoder_GetRightOdom_mm(void)     { return odom_right_mm; }

void Encoder_ResetOdom(void)
{
    odom_left_mm  = 0.0f;
    odom_right_mm = 0.0f;
}
