/**
 * @file    path_planner.h
 * @brief   路径规划 — 过道路点图 + A* 搜索
 *
 * 超市地图化为 30 个路点(水平过道 × 垂直过道的交叉点),
 * A* 搜索后输出有序路点序列, 供 Navi_Task 逐点跟随。
 * 内存 <1.5KB, 适配 STM32F411 (128KB SRAM)。
 */

#ifndef __PATH_PLANNER_H__
#define __PATH_PLANNER_H__

#include "main.h"
#include <stdint.h>

/* ── 容量 ── */
#define PP_MAX_NODES    35
#define PP_MAX_WP       40

/* ── 路点 ── */
typedef struct {
    int32_t x_mm;
    int32_t y_mm;
} PpWaypoint_t;

/* ── 路点图节点 ── */
typedef struct {
    int32_t  x_mm, y_mm;       /* 世界坐标(mm) */
    uint8_t  n_edges;          /* 邻接节点数 */
    uint8_t  edges[8];         /* 邻接节点 ID */
    uint16_t edge_cost[8];     /* 边权(mm) */
} PpNode_t;

/* ── Public API ───────────────────────────────── */

/**
 * @brief  初始化路点图 (从 map_manager 过道数据构建)
 */
void Path_Init(void);

/**
 * @brief  A* 规划路径
 * @param  sx,sy : 起点世界坐标(mm)
 * @param  gx,gy : 终点世界坐标(mm)
 * @return 路点数, 0 = 不可达
 */
uint8_t Path_Plan(int32_t sx, int32_t sy, int32_t gx, int32_t gy);

/**
 * @brief  获取当前目标路点
 * @return 当前应去的路点指针, NULL = 已到达/无路径
 */
const PpWaypoint_t * Path_CurrentWP(void);

/**
 * @brief  标记当前路点已到达, 前进到下一个
 * @return 1=还有下一个路点, 0=全部到达
 */
uint8_t Path_Advance(void);

/**
 * @brief  路径是否已完成
 */
uint8_t Path_IsDone(void);

/**
 * @brief  获取规划的路点列表 (调试用)
 */
const PpWaypoint_t * Path_GetWPs(void);
uint8_t Path_GetWPCount(void);
uint8_t Path_GetNodeCount(void);

#endif
