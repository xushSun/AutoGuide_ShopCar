/**
 * @file    bsp_motor.c
 * @brief   TB6612双H桥电机驱动 — 方向GPIO + 速度PWM
 *
 * TB6612 控制真值（单路）：
 *   INA1/INA2 = 00 → 刹车    01 → 反转    10 → 正转    11 → 刹车
 *   PWM: TIM1 CH1(PA8)左轮, CH2(PA9)右轮, 20kHz, 0~4999
 */

#include "bsp_motor.h"
#include "tim.h"

/* --------------------------------------------------------------------------
 *  内部工具 — 有符号占空比 → 无符号绝对值
 * -------------------------------------------------------------------------- */
static inline uint16_t clamp_duty(int16_t duty)
{
    if (duty > MOTOR_PWM_MAX) return MOTOR_PWM_MAX;
    if (duty < -MOTOR_PWM_MAX) return MOTOR_PWM_MAX;   /* 取绝对值 */
    return (uint16_t)(duty < 0 ? -duty : duty);
}

/* --------------------------------------------------------------------------
 *  对外接口
 * -------------------------------------------------------------------------- */

/* 启动两路PWM（占空比0%） */
void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   /* PA8 — 左轮 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   /* PA9 — 右轮 */
}

/* 左轮速度：正=前进，负=后退，0=刹车
 * 实车左电机物理反接, INA1/INA2 逻辑与右轮相反 */
void Motor_SetLeft(int16_t duty)
{
    uint16_t d = clamp_duty(duty);

    if (duty > 0) {                             /* 正转 → 物理前进 */
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);   /* 反接 */
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_SET);
    } else if (duty < 0) {                      /* 反转 → 物理后退 */
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);   /* 反接 */
    } else {                                    /* 刹车 */
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, d);
}

/* 右轮速度 */
void Motor_SetRight(int16_t duty)
{
    uint16_t d = clamp_duty(duty);

    if (duty > 0) {
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_RESET);
    } else if (duty < 0) {
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_RESET);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, d);
}

/* 双轮同时设置（比两次单独调用少一次GPIO写）
 * 左电机物理反接, INA1/INA2 逻辑与右轮相反 */
void Motor_SetBoth(int16_t left_duty, int16_t right_duty)
{
    uint16_t ld = clamp_duty(left_duty);
    uint16_t rd = clamp_duty(right_duty);

    /* — 左轮方向 (反接: INA1↔INA2) — */
    if (left_duty > 0) {
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);   /* 反接 */
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_SET);
    } else if (left_duty < 0) {
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);   /* 反接 */
    } else {
        HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);
    }

    /* — 右轮方向 — */
    if (right_duty > 0) {
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_RESET);
    } else if (right_duty < 0) {
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_RESET);
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ld);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, rd);
}

/* 四路全刹 + PWM归零 */
void Motor_Brake(void)
{
    HAL_GPIO_WritePin(INA1_GPIO_Port, INA1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INA2_GPIO_Port, INA2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INB1_GPIO_Port, INB1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(INB2_GPIO_Port, INB2_Pin, GPIO_PIN_RESET);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
}

/* 主动刹车: 短脉冲反转 → 抵消残留动量 → 硬刹车锁死
 * 纯短路刹车在低速时反电动势太弱刹不住, 反转脉冲主动顶掉惯性 */
void Motor_ActiveBrake(uint8_t channel, uint8_t strength_pct, uint16_t duration_ms)
{
    if (strength_pct > 100) strength_pct = 100;
    int16_t rev = (int16_t)((uint32_t)MOTOR_PWM_MAX * strength_pct / 100);

    int16_t ld = 0, rd = 0;
    if (channel == 0 || channel == 1) ld = -rev;   /* 左轮短时反转 */
    if (channel == 0 || channel == 2) rd = -rev;   /* 右轮短时反转 */

    Motor_SetBoth(ld, rd);
    HAL_Delay(duration_ms);
    Motor_Brake();
}
