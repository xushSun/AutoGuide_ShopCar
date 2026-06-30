/**
 * @file    bsp_uwb.h
 * @brief   MK8000 UWB — trilat2D_3A linear least squares (Arduino移植)
 *
 * 锚点: B(0.2,0)东南 A(8.88,0)西南 C(9.66,8.93)西北
 *       UWB局部系: B=(0.2,0), X=B→A方向, Y⊥BA向北
 * 世界系: B=(0,0)原点 A=(8880,0) C=(9660,8930)
 * 高度: 三锚=1.9m 标签=0.52m → dz=1.8m统一
 */

#ifndef __BSP_UWB_H__
#define __BSP_UWB_H__

#include "main.h"

#define UWB_N_ANCHORS           3

/* ── 锚点地址 (实测定标) ── */
#define UWB_ANCHOR_A_ID         0x0002   /* A: 西南角 (8.88,0) — 物理模块 0x0002 */
#define UWB_ANCHOR_B_ID         0x0001   /* B: 东南角 (0.2,0) — 物理模块 0x0001  */
#define UWB_ANCHOR_C_ID         0x0003   /* C: 西北角 (9.66,8.93) — 物理模块 0x0003 */

/* ── 锚点UWB局部坐标 (m) ── */
#define UWB_A_X_M  8.88f
#define UWB_A_Y_M  0.00f
#define UWB_B_X_M  0.20f
#define UWB_B_Y_M  0.00f

/* ── 高度 (三锚等高, 标签0.1m) ── */
#define UWB_TAG_HEIGHT_M        0.86f
#define UWB_ANCHOR_HEIGHT_M     1.90f

/* ── 每锚天线延迟 (运行时可变, 初始值) ── */
extern float uwb_delay_a_m;
extern float uwb_delay_b_m;
extern float uwb_delay_c_m;
void UWB_SetDelay(char anchor, float val);   /* 'A'/'B'/'C' → 设延迟(m) */

/* ── 滤波 ── */
#define UWB_EMA_ALPHA           0.15f

/* ── 门限 ── */
#define UWB_TIMEOUT_MS          500
#define UWB_RSSI_MIN            30
#define UWB_MAX_DIST_M          30.0f

/* 锚点信息 */
typedef struct {
    float    dist_m;
    uint8_t  rssi;
    uint32_t last_update_ms;
} UWB_Anchor_t;

/* 定位结果 */
typedef struct {
    UWB_Anchor_t anchor_A, anchor_B, anchor_C;
    float x_m, y_m;          /* UWB局部坐标(m), B=(0.2,0), X=B→A方向, Y⊥BA向北 */
    uint32_t pkt_count;
    uint8_t  data_valid;
    uint8_t  quality;
    float    rmse_m;
} UWB_State_t;

void UWB_Init(void);
void UWB_Poll(void);
void UWB_RxCallback(uint8_t byte);
const UWB_State_t * UWB_GetState(void);

#endif
