/**
 * @file    bsp_lidar.h
 * @brief   LD06_LD laser radar driver (UART + DMA + IDLE detection)
 * @note    LD06 protocol: 230400 bps, 8N1, 0.02–12 m, ~4500 pts/s
 */

#ifndef __BSP_LIDAR_H__
#define __BSP_LIDAR_H__

#include "main.h"

/* ---- DMA buffer sizing ---- */
#define LD06_POINTS_MAX         1800    /**< Max points per revolution @10Hz   */
#define LD06_BYTES_PER_POINT    3       /**< LD06 packs 3 bytes per point       */
#define LD06_FRAME_MAX_BYTES    (LD06_POINTS_MAX * LD06_BYTES_PER_POINT)  /* 5400 */
#define LD06_DMA_MARGIN         2048    /**< Guard band for circular DMA        */
#define LD06_DMA_BUF_LEN        (LD06_FRAME_MAX_BYTES + LD06_DMA_MARGIN)   /* ~7448 */

#define LD06_SCAN_POINTS_MAX    LD06_POINTS_MAX

/* ---- point data (always visible) ---- */
#pragma pack(push, 1)
typedef struct {
    float    x;           /**< Cartesian X (m), forward = +X                     */
    float    y;           /**< Cartesian Y (m), left   = +Y                     */
    float    dist;        /**< Polar distance (m)                                */
    float    angle;       /**< Polar angle   (rad)                              */
    uint8_t  intensity;   /**< Signal strength (0–255)                          */
    uint8_t  valid;       /**< 1 = valid point, 0 = filtered out                */
} LidarPoint_t;
#pragma pack(pop)

/* -------------------------------------------------------------------------- */
/*  Public API                                                                */
/* -------------------------------------------------------------------------- */

void Lidar_Init(void);
void Lidar_Poll(void);
void Lidar_MarkDataPending(void);
void Lidar_ParseFrame(const uint8_t *buf, uint16_t len);
const LidarPoint_t * Lidar_GetScan(uint16_t *count);
void Lidar_MotorOn(void);
void Lidar_MotorOff(void);
uint8_t Lidar_IsNewScanReady(void);

/**
 * @brief  Get pointer to the raw DMA circular buffer.
 * @note   Used by bsp_callbacks to compute rx byte count.
 */
uint8_t * Lidar_GetDMABuf(void);

#endif /* __BSP_LIDAR_H__ */
