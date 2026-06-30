/**
 * @file    lidar.h
 * @brief   LD06 DTOF 360° laser radar driver — HAL DMA circular
 */

#ifndef __LIDAR_H
#define __LIDAR_H

#include "main.h"

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

#define LIDAR_DMA_BUF_SIZE      2048
#define LIDAR_ZONE_DANGER_M     0.8f
#define LIDAR_ZONE_WARN_M       2.0f

typedef enum {
    LIDAR_ZONE_CLEAR   = 0,
    LIDAR_ZONE_WARNING = 1,
    LIDAR_ZONE_DANGER  = 2
} lidar_zone_t;

typedef enum {
    LIDAR_DIR_FRONT = 0,
    LIDAR_DIR_FRONT_LEFT,
    LIDAR_DIR_LEFT,
    LIDAR_DIR_FRONT_RIGHT,
    LIDAR_DIR_RIGHT,
    LIDAR_DIR_COUNT
} lidar_dir_t;

typedef struct {
    float        dist_m;      /* min distance in this sector */
    lidar_zone_t zone;
} lidar_sector_t;

typedef struct {
    lidar_sector_t dir[LIDAR_DIR_COUNT];
    uint32_t       pkt_count;
} lidar_state_t;

void   lidar_init(void);
void   lidar_poll(void);
const volatile lidar_state_t *lidar_get_state(void);
void   lidar_dma_buf_reset(void);
void   lidar_idle_handler(void);

extern volatile uint32_t  g_lidar_hdr_cnt;
extern volatile uint32_t  g_lidar_ver_fail;
extern volatile uint32_t  g_lidar_crc_fail;
extern volatile uint32_t  g_lidar_rx_bytes;
extern volatile lidar_state_t g_lidar_state;

#endif
