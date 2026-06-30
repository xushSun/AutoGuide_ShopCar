/**
 * @file    map_manager.h
 * @brief   超市地图管理 — 栅格占据地图 + 商品坐标表 + 静态障碍物
 *
 * 地图定义:
 *   - 世界坐标系原点: 超市左下角 (由锚点位置决定)
 *   - X轴: 东(右), Y轴: 北(前)
 *   - 栅格分辨率: MAP_CELL_SIZE_MM mm/格
 *   - 格值: 0=空闲, 1=障碍物, 2=过道, 3=收银区, 4=货架, 5=墙壁
 *
 * 商品表: 上位机通过 CMD 0x06 发商品ID, 本模块二分查找返回世界坐标
 * 导航:   调用 Map_IsFree() 做前向障碍检测 (未来A*用到整个grid)
 *
 * 内存预算 (128KB SRAM):
 *   栅格:  MAP_WIDTH_CELLS * MAP_HEIGHT_CELLS * 1B ≈ 9.5KB
 *   障碍:  MAP_OBSTACLE_MAX * 12B                      ≈ 1.2KB
 *   过道:  MAP_AISLE_MAX * 22B                         ≈ 0.4KB
 *   合计:                                               ≈ 11.1KB
 */

#ifndef __MAP_MANAGER_H__
#define __MAP_MANAGER_H__

#include "main.h"
#include "coord_solver.h"
#include <stdint.h>

/* ════════════════════════════════════════════════════
 *  地图尺寸 (编译时可调, 按实际超市修改)
 * ════════════════════════════════════════════════════ */
#define MAP_CELL_SIZE_MM        100       /* 栅格分辨率 (mm/格)                   */
#define MAP_WIDTH_CELLS         100       /* X方向格数 9660mm ≈ 100格            */
#define MAP_HEIGHT_CELLS        95        /* Y方向格数 8930mm ≈ 95格             */
#define MAP_WIDTH_MM            ((int32_t)MAP_WIDTH_CELLS  * MAP_CELL_SIZE_MM)
#define MAP_HEIGHT_MM           ((int32_t)MAP_HEIGHT_CELLS * MAP_CELL_SIZE_MM)

/* ════════════════════════════════════════════════════
 *  格值定义
 * ════════════════════════════════════════════════════ */
#define MAP_CELL_FREE           0         /* 空闲可通行                           */
#define MAP_CELL_OBSTACLE       1         /* 障碍物 (不可通行)                    */
#define MAP_CELL_AISLE          2         /* 过道 (优先通行)                      */
#define MAP_CELL_CHECKOUT       3         /* 收银区                              */
#define MAP_CELL_SHELF          4         /* 货架 (不可通行)                      */
#define MAP_CELL_WALL           5         /* 墙壁 (不可通行)                      */

/* ════════════════════════════════════════════════════
 *  容量限制
 * ════════════════════════════════════════════════════ */
#define MAP_OBSTACLE_MAX        100       /* 最大障碍物矩形数                     */
#define MAP_AISLE_MAX           20        /* 最大过道数                           */

/* ════════════════════════════════════════════════════
 *  数据结构
 * ════════════════════════════════════════════════════ */

/* 障碍物矩形 (编译时硬编码, 运行时栅格化) */
typedef struct {
    int16_t x1_mm;            /* 左上/起点 X (mm)                    */
    int16_t y1_mm;            /* 左上/起点 Y (mm)                    */
    int16_t x2_mm;            /* 右下/终点 X (mm)                    */
    int16_t y2_mm;            /* 右下/终点 Y (mm)                    */
    uint8_t cell_type;        /* 填充格值 (SHELF, WALL等)            */
    uint8_t reserved[3];      /* 对齐填充                            */
} ObstacleRect_t;

/* 过道定义 (导航提示/A*代价偏置) */
typedef struct {
    char    name[8];          /* 过道名称 (如 "A1", "B2")            */
    int16_t x1_mm, y1_mm;     /* 起点世界坐标 (mm)                   */
    int16_t x2_mm, y2_mm;     /* 终点世界坐标 (mm)                   */
} AisleDef_t;

/* 地图管理器 (全局单例) */
typedef struct {
    uint8_t         grid[MAP_WIDTH_CELLS][MAP_HEIGHT_CELLS]; /* 占据栅格          */
    ObstacleRect_t  obstacles[MAP_OBSTACLE_MAX];/* 障碍物列表                    */
    uint8_t         obstacle_count;            /* 当前障碍物数                   */
    AisleDef_t      aisles[MAP_AISLE_MAX];     /* 过道列表                       */
    uint8_t         aisle_count;               /* 当前过道数                     */
    uint8_t         map_loaded;                /* 1=地图已加载                    */
} MapManager_t;

/* ════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════ */

void Map_Init(void);

/**
 * @brief  加载硬编码默认超市地图
 * @note   包含: 墙壁/货架/过道/商品坐标, 均需按实际超市修改
 */
void Map_LoadDefault(void);

/**
 * @brief  查询某点是否可通行 (0=空闲可走, 1=障碍物)
 */
uint8_t Map_IsObstacle(int32_t x_mm, int32_t y_mm);

/**
 * @brief  查询某点是否空闲 (Map_IsObstacle 取反)
 */
uint8_t Map_IsFree(int32_t x_mm, int32_t y_mm);

/**
 * @brief  获取地图管理器状态 (含所有内部数据)
 */
const MapManager_t * Map_GetState(void);

#endif /* __MAP_MANAGER_H__ */
