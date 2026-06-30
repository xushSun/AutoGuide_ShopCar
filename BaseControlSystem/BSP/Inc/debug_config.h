/**
 * @file    debug_config.h
 * @brief   模块级调试开关 — 每个功能模块独立可控
 *
 * 用法:
 *   1 = 启用该模块 (编译进固件并执行)
 *   0 = 禁用该模块 (不初始化、不执行)
 *
 * 示例:
 *   想单独测试UWB: 只开 DEBUG_ENABLE_UWB + DEBUG_ENABLE_LED + DEBUG_ENABLE_HOST_UART
 *   想测试完整导航: 全部置 1
 */

#ifndef __DEBUG_CONFIG_H__
#define __DEBUG_CONFIG_H__

/* ==================================================================
 *  调试开关 (1=启用, 0=禁用)
 * ================================================================== */

#define DEBUG_ENABLE_LED            1
#define DEBUG_ENABLE_UWB            1
#define DEBUG_ENABLE_ENCODER        1
#define DEBUG_ENABLE_MOTOR          1
#define DEBUG_ENABLE_PID            1
#define DEBUG_ENABLE_NAVI           1
#define DEBUG_ENABLE_LIDAR          1
#define DEBUG_ENABLE_HOST_UART      1
#define DEBUG_ENABLE_PATH_PLAN      1
#define DEBUG_ENABLE_COORD_SOLVER   1
#define DEBUG_ENABLE_MAP            1
#define DEBUG_ENABLE_MOTOR_TEST     0
#define DEBUG_ENABLE_GYRO           1


/* 底盘相关: 需要编码器+电机+PID 同时启用才有效 */
#define DEBUG_CHASSIS_READY  (DEBUG_ENABLE_ENCODER && DEBUG_ENABLE_MOTOR && DEBUG_ENABLE_PID)

/* 导航相关: 需要 UWB+坐标解算+底盘+路径规划 */
#define DEBUG_NAVI_READY     (DEBUG_ENABLE_UWB && DEBUG_ENABLE_COORD_SOLVER && DEBUG_CHASSIS_READY && DEBUG_ENABLE_NAVI)
#endif /* __DEBUG_CONFIG_H__ */
