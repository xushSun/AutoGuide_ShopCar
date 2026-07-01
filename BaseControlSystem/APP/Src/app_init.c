/**
 * @file    app_init.c
 * @brief   应用层统一初始化入口 — main.c 只需调用 App_Init()
 */

#include <stdio.h>
#include "app_init.h"
#include "debug_config.h"
#include "chassis_task.h"
#include "app_navigate.h"
#include "bsp_lidar.h"
#include "bsp_uwb.h"
#include "bsp_uart.h"
#include "coord_solver.h"
#include "map_manager.h"
#include "path_planner.h"
#include "tim.h"

#if DEBUG_ENABLE_GYRO
#include "mpu6050.h"
#include "bsp_callbacks.h"
#include "i2c.h"
#endif

/* ==========================================================================
 *  上电初始化 (各模块由 debug_config.h 开关独立控制)
 * ========================================================================== */
void App_Init(void)
{
#if DEBUG_ENABLE_ENCODER || DEBUG_ENABLE_MOTOR || DEBUG_ENABLE_PID
    /* 底盘: 编码器 + 双轮PID + 电机PWM (初速=0, 刹车态) */
    Chassis_Init();
#endif

#if DEBUG_ENABLE_NAVI
    /* 导航层: 初始位姿归零, 等待上位机发目标 */
    APP_Navigate_Init();
#endif

#if DEBUG_ENABLE_COORD_SOLVER
    /* 坐标解算器: 3锚实测UWB坐标 → 世界 6参数仿射
     * 世界: A=(8880,0)  B=(20,0)  C=(9660,8930) mm
     * 实测: A=(8.85,0.00) B=(-0.72,1.00) C=(8.95,9.35) m */
    CoordSolver_Init();
    CoordSolver_SetAnchors(8880, 0, 20, 0, 9660, 8930,
                           8.85f, 0.00f,
                           -0.72f, 1.00f,
                           8.95f, 9.35f);
#endif

#if DEBUG_ENABLE_MAP
    /* 超市地图: 加载硬编码默认地图 (货架/过道/商品) */
    Map_Init();
    Map_LoadDefault();
#endif

#if DEBUG_ENABLE_PATH_PLAN
    /* 路径规划: 路点图 + A* */
    Path_Init();
#endif

#if DEBUG_ENABLE_HOST_UART
    /* 上位机通信: USART6 单字节IT接收, 协议0xA5 */
    HostUART_Init();
#endif

#if DEBUG_ENABLE_UWB
    /* UWB定位: USART1 DMA轮询, MK8000协议 */
    UWB_Init();
#endif

#if DEBUG_ENABLE_LIDAR
    /* LD06激光雷达: USART2 DMA循环接收 + IDLE帧检测 */
    Lidar_Init();
#endif

#if DEBUG_ENABLE_GYRO
    /* MPU6050 陀螺仪: I2C1 PB6/PB7, 主循环读(I2C安全), TIM4 ISR只积分 */
    {
        uint8_t ok = MPU6050_Init(&hi2c1);
        if (ok == 0) {
            Gyro_Enable();
            printf("GYRO: MPU6050 OK\r\n");
            GyroHold_Calibrate();     /* 静止2s采样 Gz 零偏 (TIM4启动前) */
        } else {
            printf("GYRO: MPU6050 FAIL (WHO_AM_I=%d)\r\n", ok);
        }
    }
#endif

    /* 启动100Hz控制定时器 (TIM4 → 10ms周期) */
    HAL_TIM_Base_Start_IT(&htim4);

#if DEBUG_ENABLE_LIDAR
    /* 雷达电机启动 (TIM10 CH1 → PB8, 30kHz 50%) */
    Lidar_MotorOn();
#endif
}
