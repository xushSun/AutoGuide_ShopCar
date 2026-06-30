/**
 * @file    map_manager.c
 * @brief   超市地图 — 栅格占据地图 + 商品表 + 障碍物 (B原点)
 *
 * 坐标系: B=(0,0) 东南角, +X=西, +Y=北
 * 数据来源: 地图.xlsx (2026-06-20 用户标注)
 *
 * 物理布局 (南→北, 东→西):
 *   ┌─────┬───────────────────────────────────┐
 *   │ 西  │███████ 北货架排 Row1 73cm ████████│ Y=8200~8930
 *   │ 墙  │         过道3 Row2 169cm         │ Y=6510~8200
 *   │(主) │███████ 货架A上 Row3 73cm ████████│ Y=5780~6510
 *   │ 过  │███████ 货架A下 Row4 73cm ████████│ Y=5050~5780
 *   │ 道  │         过道2 Row5 169cm         │ Y=3360~5050
 *   │78cm │███████ 货架B上 Row6 73cm ████████│ Y=2630~3360
 *   │     │███████ 货架B下 Row7 73cm ████████│ Y=1900~2630
 *   │     │         过道1 Row8 190cm         │ Y=0~1900
 *   │     │ A(8880,0)                     B(0,0)
 *   └─────┴───────────────────────────────────┘
 *   9660  8880                                 0 mm
 *   (X西)                                       (X东)
 *
 * 尺寸汇总:
 *   总宽X: 9660mm (780主过道 + 8880货架区)
 *   总高Y: 8930mm
 *   每列宽: 1110mm (111cm), 8列 = 8880mm
 *   过道宽: 1900/1690/1690mm (过道1/2/3)
 *   货架深: 730mm (73cm), 背靠背两组 = 1460mm
 *   锚AB间距: 8880mm (8×1110) ✓
 */

#include "map_manager.h"
#include <string.h>

/* ── 模块单例 ─────────────────────────────────── */
static MapManager_t map;

/* ═════════════════════════════════════════════════════
 *  内部工具
 * ═════════════════════════════════════════════════════ */

static uint8_t world_to_cell(int32_t x_mm, int32_t y_mm,
                             uint16_t *cx, uint16_t *cy)
{
    if (x_mm < 0 || y_mm < 0 ||
        x_mm >= MAP_WIDTH_MM || y_mm >= MAP_HEIGHT_MM) {
        return 0;
    }
    *cx = (uint16_t)((uint32_t)x_mm / MAP_CELL_SIZE_MM);
    *cy = (uint16_t)((uint32_t)y_mm / MAP_CELL_SIZE_MM);
    if (*cx >= MAP_WIDTH_CELLS || *cy >= MAP_HEIGHT_CELLS) return 0;
    return 1;
}

static void rasterize_rect(const ObstacleRect_t *r)
{
    uint16_t cx1, cy1, cx2, cy2;
    if (!world_to_cell(r->x1_mm, r->y1_mm, &cx1, &cy1)) return;
    if (!world_to_cell(r->x2_mm, r->y2_mm, &cx2, &cy2)) return;
    if (cx1 > cx2) { uint16_t t = cx1; cx1 = cx2; cx2 = t; }
    if (cy1 > cy2) { uint16_t t = cy1; cy1 = cy2; cy2 = t; }
    for (uint16_t x = cx1; x <= cx2 && x < MAP_WIDTH_CELLS; x++)
        for (uint16_t y = cy1; y <= cy2 && y < MAP_HEIGHT_CELLS; y++)
            map.grid[x][y] = r->cell_type;
}

/* ═════════════════════════════════════════════════════
 *  对外接口
 * ═════════════════════════════════════════════════════ */

void Map_Init(void)
{
    memset(&map, 0, sizeof(map));
    map.map_loaded = 0;
}

void Map_LoadDefault(void)
{
    uint8_t i;

    /* ── ① 清空栅格 ── */
    memset(map.grid, MAP_CELL_FREE, sizeof(map.grid));

    /* ═══════════════════════════════════════════════════
     *  ② 墙壁 (4面, 厚度100mm=1格)
     * ═══════════════════════════════════════════════════ */
    map.obstacle_count = 0;

    /* 南墙 y=0~100 (Row9锚点行=0, 贴墙) */
    map.obstacles[map.obstacle_count++] = (ObstacleRect_t){
        0, 0, MAP_WIDTH_MM - 1, 100, MAP_CELL_WALL, {0}};

    /* 北墙 y=8830~8930 (Row1北货架上方贴墙) */
    map.obstacles[map.obstacle_count++] = (ObstacleRect_t){
        0, 8830, MAP_WIDTH_MM - 1, 8930, MAP_CELL_WALL, {0}};

    /* 东墙 x=0~100 */
    map.obstacles[map.obstacle_count++] = (ObstacleRect_t){
        0, 0, 100, MAP_HEIGHT_MM - 1, MAP_CELL_WALL, {0}};

    /* 西墙 x=9560~9660 (主过道西侧) */
    map.obstacles[map.obstacle_count++] = (ObstacleRect_t){
        9560, 0, MAP_WIDTH_MM - 1, MAP_HEIGHT_MM - 1, MAP_CELL_WALL, {0}};

    /* ═══════════════════════════════════════════════════
     *  ③ 货架区 (3块 × 通栏 = 3大矩形)
     *
     *  每个货架列: 1110×730mm
     *  8列并排无间隙 (锚间距=8×1110=8880mm证明)
     *
     *  货架块B: Y=1900~3360 (Row6+Row7, 146cm)
     *  货架块A: Y=5050~6510 (Row3+Row4, 146cm)
     *  北货架:  Y=8200~8930 (Row1, 73cm)
     *  所有货架X: 100~8880 (东墙到主过道)
     * ═══════════════════════════════════════════════════ */

    /* 货架块B (南侧, 过道1和过道2之间) */
    map.obstacles[map.obstacle_count++] = (ObstacleRect_t){
        100, 1900, 8880, 3360, MAP_CELL_SHELF, {0}};

    /* 货架块A (中间, 过道2和过道3之间) */
    map.obstacles[map.obstacle_count++] = (ObstacleRect_t){
        100, 5050, 8880, 6510, MAP_CELL_SHELF, {0}};

    /* 北货架 (贴北墙, 过道3上方) */
    map.obstacles[map.obstacle_count++] = (ObstacleRect_t){
        100, 8200, 8880, 8830, MAP_CELL_SHELF, {0}};

    /* ═══════════════════════════════════════════════════
     *  ④ 过道定义 (10条)
     * ═══════════════════════════════════════════════════ */
    map.aisle_count = 0;

    /* 主过道 (垂直, X=8880~9560, 780mm宽) */
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Main", 9270, 600, 9270, 8800};

    /* 过道1 (南, Y=100~1900, 190cm) */
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Aisle1", 500, 950, 8800, 950};

    /* 过道2 (中, Y=3360~5050, 169cm) */
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Aisle2", 500, 4200, 8800, 4200};

    /* 过道3 (北, Y=6510~8200, 169cm) */
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Aisle3", 500, 7350, 8800, 7350};

    /* 列间过道 (8列间可通行, 中心在每列边界) */
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Col01", 1110, 200, 1110, 8800};
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Col02", 2220, 200, 2220, 8800};
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Col03", 3330, 200, 3330, 8800};
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Col04", 4440, 200, 4440, 8800};
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Col05", 5550, 200, 5550, 8800};
    map.aisles[map.aisle_count++] = (AisleDef_t){
        "Col06", 6660, 200, 6660, 8800};

    /* ═══════════════════════════════════════════════════
     *  ⑤ 排序 + 栅格化
     * ═══════════════════════════════════════════════════ */

    for (i = 0; i < map.obstacle_count; i++)
        rasterize_rect(&map.obstacles[i]);

    /* ═══════════════════════════════════════════════════
     *  ⑤ 标记过道区域 (空白区 → AISLE)
     * ═══════════════════════════════════════════════════ */
    /* 主过道 X=89~95 格 */
    for (uint16_t x = 89; x < 96 && x < MAP_WIDTH_CELLS; x++)
        for (uint16_t y = 1; y < MAP_HEIGHT_CELLS - 1; y++)
            if (map.grid[x][y] == MAP_CELL_FREE) map.grid[x][y] = MAP_CELL_AISLE;

    /* 过道1 Y=1~19 格 */
    for (uint16_t x = 1; x < MAP_WIDTH_CELLS - 1; x++)
        for (uint16_t y = 1; y < 19 && y < MAP_HEIGHT_CELLS; y++)
            if (map.grid[x][y] == MAP_CELL_FREE) map.grid[x][y] = MAP_CELL_AISLE;

    /* 过道2 Y=33~50 格 */
    for (uint16_t x = 1; x < MAP_WIDTH_CELLS - 1; x++)
        for (uint16_t y = 33; y < 51 && y < MAP_HEIGHT_CELLS; y++)
            if (map.grid[x][y] == MAP_CELL_FREE) map.grid[x][y] = MAP_CELL_AISLE;

    /* 过道3 Y=65~82 格 */
    for (uint16_t x = 1; x < MAP_WIDTH_CELLS - 1; x++)
        for (uint16_t y = 65; y < 82 && y < MAP_HEIGHT_CELLS; y++)
            if (map.grid[x][y] == MAP_CELL_FREE) map.grid[x][y] = MAP_CELL_AISLE;

    map.map_loaded = 1;
}

/* ═════════════════════════════════════════════════════
 *  栅格查询
 * ═════════════════════════════════════════════════════ */

uint8_t Map_IsObstacle(int32_t x_mm, int32_t y_mm)
{
    uint16_t cx, cy;
    if (!world_to_cell(x_mm, y_mm, &cx, &cy)) return 1;
    uint8_t cell = map.grid[cx][cy];
    return (cell == MAP_CELL_OBSTACLE || cell == MAP_CELL_SHELF ||
            cell == MAP_CELL_WALL);
}

uint8_t Map_IsFree(int32_t x_mm, int32_t y_mm)
{
    return Map_IsObstacle(x_mm, y_mm) ? 0 : 1;
}

const MapManager_t * Map_GetState(void) { return &map; }
