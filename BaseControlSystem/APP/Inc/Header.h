#ifndef _HEADER_H_
#define _HEADER_H_

#include "stm32f4xx_hal.h"

#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "debug_config.h"

/* --- BSP module headers --- */
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_lidar.h"
#include "bsp_uwb.h"
#include "bsp_uart.h"
#include "bsp_callbacks.h"
#include "bsp_motor.h"
#include "mpu6050.h"

/* --- APP module headers --- */
#include "pid.h"
#include "kinematics.h"
#include "chassis_task.h"
#include "coord_solver.h"
#include "map_manager.h"
#include "path_planner.h"
#include "app_init.h"
#include "app_navigate.h"
#include "navi_task.h"
#endif
