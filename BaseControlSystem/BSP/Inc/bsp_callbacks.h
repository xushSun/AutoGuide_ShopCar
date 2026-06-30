/**
 * @file    bsp_callbacks.h
 * @brief   Custom callback declarations
 * @note    HAL native callbacks (PeriodElapsed, RxCplt etc.) are declared in
 *          HAL headers; their implementations live in bsp_callbacks.c only.
 */

#ifndef __BSP_CALLBACKS_H__
#define __BSP_CALLBACKS_H__

#include "main.h"

/**
 * @brief  Handle USART2 IDLE – detects end of LD06 data frame.
 * @note   Call from USART2_IRQHandler (Core/Src/stm32f4xx_it.c):
 *         if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
 *             __HAL_UART_CLEAR_IDLEFLAG(&huart2);
 *             BSP_UART2_IDLE_Handler();
 *         }
 */
void BSP_UART2_IDLE_Handler(void);

/* 陀螺仪 yaw (rad/deg) — GyroHold_Update 积分, NAVI/直走共用 */
void  Gyro_Enable(void);           /* MPU6050_Init 成功后调用 */
void  GyroHold_Calibrate(void);    /* 上电静止2s采样 Gz 零偏, TIM4启动前调用 */
float HLD_get_yaw_rad(void);
float HLD_get_yaw_deg(void);
void  GyroYaw_Reset(void);
void  HoldYaw_Lock(void);          /* 锁当前yaw为航向参考,清零I项 */
void  HoldYaw_Release(void);       /* 转弯前解除锁,清I项防干扰 */
float GyroHold_ComputeW(void);     /* PI航向修正 w(rad/s), 已取反适配差速 */
char  GyroHold_ReadIMU(void);      /* 主循环调用, I2C读MPU6050 (非ISR安全) */

/* 诊断全局变量 (main.c printf 用) */
extern float g_gyro_yaw_err;       /* 当前航向误差 (rad)     */
extern float g_gyro_corr_w;        /* 当前修正角速度 (rad/s) */

#endif /* __BSP_CALLBACKS_H__ */
