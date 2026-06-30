/**
 * @file    bsp_uart.h
 * @brief   上位机通信协议 (USART6, 115200bps)
 *
 * 帧格式: [0xA5] [CMD] [LEN] [载荷...] [XOR校验]
 * 校验 = CMD ^ LEN ^ payload[0] ^ ... ^ payload[N-1]
 *
 * 指令:
 *   0x01 = 目标坐标  载荷8B: x(i32 mm) + y(i32 mm)
 *   0x02 = 急停      载荷0B
 *   0x03 = 请求位姿  载荷0B
 *   0x80 = 回复位姿  载荷10B: x(i32 mm) + y(i32 mm) + yaw(i16 0.01°)
 */

#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "main.h"
#include <stdint.h>

#define HOST_HEADER         0xA5
#define HOST_CMD_TARGET     0x01    /* 目标坐标 x(mm)+y(mm) i32+i32=8B   */
#define HOST_CMD_STOP       0x02    /* 急停                                */
#define HOST_CMD_POSE_REQ   0x03    /* 请求位姿                            */
#define HOST_CMD_OPENLOOP   0x04    /* 开环测试: left% + right% (u8+u8, 2B)*/
#define HOST_CMD_SET_ANCHORS 0x05   /* 设定锚点世界坐标: ax,ay,bx,by (int16*4=8B) */
#define HOST_CMD_ID_POSE    0x80    /* 回复位姿: x+y+yaw 共10B            */
#define HOST_CMD_PATH_WP    0x90    /* 回复路点: [idx u8][total u8][x i32][y i32] = 10B */

#define HOST_RX_BUF_LEN     64
#define HOST_TX_BUF_LEN     128

void HostUART_Init(void);
void HostUART_ParseByte(uint8_t byte);
void HostUART_SendPose(int32_t x_mm, int32_t y_mm, int16_t yaw_001deg);
void HostUART_SendPacket(uint8_t cmd, const uint8_t *payload, uint8_t len);
void HostUART_CheckTimeout(void);  /* 周期性检查ASCII超时 */

#endif
