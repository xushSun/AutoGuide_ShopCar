/**
 * @file    path_planner.c
 * @brief   路径规划 — 过道路点图 + A*
 *
 * 路点图: 3层水平过道 × 9列垂直过道 ≈ 29 节点
 * A*: 线性搜索 (40节点无需堆)
 *
 * 边连接规则:
 *   - 同一水平层相邻列: 水平边
 *   - 同一垂直列相邻层: 垂直边 (仅主过道/列过道)
 */

#include "path_planner.h"
#include "map_manager.h"
#include <string.h>

/* ── 路点图 ── */
static PpNode_t   nodes[PP_MAX_NODES];
static uint8_t    node_count;

/* ── A* 内部 ── */
static float      g[PP_MAX_NODES];
static uint8_t    parent[PP_MAX_NODES];
static uint8_t    open_set[PP_MAX_NODES];   /* 1=在open中 */
static uint8_t    closed_set[PP_MAX_NODES];
static uint8_t    start_node, goal_node;

/* ── 路径输出 ── */
static PpWaypoint_t waypoints[PP_MAX_WP];
static uint8_t      wp_count;
static uint8_t      wp_index;   /* 当前路点序号 */

/* ═════════════════════════════════════════════════════
 *  路点图构建
 *
 *  水平层 Y = 985, 4230, 7380  (过道中线, 取自map)
 *  垂直列 X = 600, 1110, 2220, 3330, 4440, 5550, 6660, 8800, 9270
 *
 *  拓扑: 梳子型 — 仅主过道(col8)可南北穿行, 货架列间无缝隙
 *    节点编号:  row×9 + col
 *      row 0: 过道1 (Y=985)
 *      row 1: 过道2 (Y=4230)
 *      row 2: 过道3 (Y=7380)
 * ═════════════════════════════════════════════════════ */

#define ROWS  3
#define COLS  9

static const int32_t row_y[ROWS] = { 985, 4230, 7380 };
static const int32_t col_x[COLS] = { 600, 1110, 2220, 3330, 4440, 5550, 6660, 8800, 9270 };

/* 垂直通道: 仅主过道 col8=9270 可南北穿行，货架列间无缝隙 */
static const uint8_t col_vertical[COLS] = { 0, 0, 0, 0, 0, 0, 0, 0, 1 };

static void add_edge(uint8_t a, uint8_t b, uint16_t cost)
{
    if (nodes[a].n_edges < 8) {
        uint8_t i = nodes[a].n_edges++;
        nodes[a].edges[i] = b;
        nodes[a].edge_cost[i] = cost;
    }
}

static void connect_both(uint8_t a, uint8_t b)
{
    int32_t dx = nodes[a].x_mm - nodes[b].x_mm;
    int32_t dy = nodes[a].y_mm - nodes[b].y_mm;
    uint16_t d = (uint16_t)((dx>0?dx:-dx) + (dy>0?dy:-dy));
    add_edge(a, b, d);
    add_edge(b, a, d);
}

void Path_Init(void)
{
    uint8_t r, c;

    memset(nodes, 0, sizeof(nodes));
    node_count = 0;

    /* ── 创建 3×9=27 个节点 ── */
    for (r = 0; r < ROWS; r++) {
        for (c = 0; c < COLS; c++) {
            uint8_t id = r * COLS + c;
            nodes[id].x_mm = col_x[c];
            nodes[id].y_mm = row_y[r];
            nodes[id].n_edges = 0;
        }
    }
    node_count = ROWS * COLS;   /* 27 */

    /* ── 水平边 (同行相邻列) ── */
    for (r = 0; r < ROWS; r++) {
        for (c = 0; c < COLS - 1; c++) {
            connect_both(r * COLS + c, r * COLS + c + 1);
        }
    }

    /* ── 垂直边 (同列相邻行, 仅垂直通道列) ── */
    for (r = 0; r < ROWS - 1; r++) {
        for (c = 0; c < COLS; c++) {
            if (col_vertical[c]) {
                connect_both(r * COLS + c, (r + 1) * COLS + c);
            }
        }
    }
}

/* ═════════════════════════════════════════════════════
 *  A* 搜索
 * ═════════════════════════════════════════════════════ */

static float heuristic(uint8_t n)
{
    int32_t dx = nodes[n].x_mm - nodes[goal_node].x_mm;
    int32_t dy = nodes[n].y_mm - nodes[goal_node].y_mm;
    if (dx < 0) dx = -dx;
    if (dy < 0) dy = -dy;
    return (float)(dx + dy) * 0.001f;   /* mm → m */
}

static uint8_t pop_min_f(void)
{
    float best = 1e9f;
    uint8_t best_i = 0xFF;
    for (uint8_t i = 0; i < node_count; i++) {
        if (!open_set[i]) continue;
        float f = g[i] + heuristic(i);
        if (f < best) { best = f; best_i = i; }
    }
    if (best_i != 0xFF) open_set[best_i] = 0;
    return best_i;
}

uint8_t Path_Plan(int32_t sx, int32_t sy, int32_t gx, int32_t gy)
{
    if (node_count == 0) return 0;

    uint8_t i;

    /* ── 找最近路点 ── */
    {
        int32_t best_d = 0x7FFFFFFF;
        uint8_t best_s = 0, best_g = 0;
        for (i = 0; i < node_count; i++) {
            int32_t dsx = nodes[i].x_mm - sx; if (dsx<0) dsx=-dsx;
            int32_t dsy = nodes[i].y_mm - sy; if (dsy<0) dsy=-dsy;
            int32_t ds = dsx + dsy;
            if (ds < best_d) { best_d = ds; best_s = i; }
        }
        start_node = best_s;
        best_d = 0x7FFFFFFF;
        for (i = 0; i < node_count; i++) {
            int32_t dgx = nodes[i].x_mm - gx; if (dgx<0) dgx=-dgx;
            int32_t dgy = nodes[i].y_mm - gy; if (dgy<0) dgy=-dgy;
            int32_t dg = dgx + dgy;
            if (dg < best_d) { best_d = dg; best_g = i; }
        }
        goal_node = best_g;
    }

    /* ── A* 初始化 ── */
    for (i = 0; i < node_count; i++) {
        g[i] = 1e9f;
        parent[i] = 0xFF;
        open_set[i] = 0;
        closed_set[i] = 0;
    }

    g[start_node] = 0.0f;
    open_set[start_node] = 1;

    /* ── A* 主循环 ── */
    while (1) {
        uint8_t cur = pop_min_f();
        if (cur == 0xFF) { wp_count = 0; return 0; }   /* 无路, 清零防止读到过期路点 */
        if (cur == goal_node) break;        /* 到达 */

        closed_set[cur] = 1;

        PpNode_t *cn = &nodes[cur];
        for (i = 0; i < cn->n_edges; i++) {
            uint8_t nb = cn->edges[i];
            if (closed_set[nb]) continue;

            float ng = g[cur] + (float)cn->edge_cost[i] * 0.001f;
            if (ng < g[nb]) {
                g[nb] = ng;
                parent[nb] = cur;
                open_set[nb] = 1;
            }
        }
    }

    /* ── 回溯构建路点序列 ── */
    uint8_t rev[PP_MAX_WP];
    i = 0;
    uint8_t cur = goal_node;
    while (cur != 0xFF) {
        rev[i++] = cur;
        cur = parent[cur];
    }

    /* 反转 + 输出 + 删除冗余共线点 */
    wp_count = 0;
    for (uint8_t j = 0; j < i; j++) {
        uint8_t nid = rev[i - 1 - j];
        PpWaypoint_t wp;
        wp.x_mm = nodes[nid].x_mm;
        wp.y_mm = nodes[nid].y_mm;

        /* 去共线: 如果新点和前两个点共线 → 替换中间点 */
        if (wp_count >= 2) {
            int32_t x1=waypoints[wp_count-2].x_mm, y1=waypoints[wp_count-2].y_mm;
            int32_t x2=waypoints[wp_count-1].x_mm, y2=waypoints[wp_count-1].y_mm;
            /* 三点共线 → 替换最后一个 */
            if ((x1==x2 && x2==wp.x_mm) || (y1==y2 && y2==wp.y_mm)) {
                waypoints[wp_count-1] = wp;
                continue;
            }
        }
        waypoints[wp_count++] = wp;
    }

    /* 把终点坐标也加入 */
    waypoints[wp_count].x_mm = gx;
    waypoints[wp_count].y_mm = gy;
    wp_count++;

    wp_index = 0;
    return wp_count;
}

/* ═════════════════════════════════════════════════════
 *  路点导航接口
 * ═════════════════════════════════════════════════════ */

const PpWaypoint_t * Path_CurrentWP(void)
{
    if (wp_index >= wp_count) return NULL;
    return &waypoints[wp_index];
}

uint8_t Path_Advance(void)
{
    wp_index++;
    return (wp_index < wp_count) ? 1 : 0;
}

uint8_t Path_IsDone(void)
{
    return (wp_index >= wp_count) ? 1 : 0;
}

const PpWaypoint_t * Path_GetWPs(void) { return waypoints; }
uint8_t Path_GetWPCount(void)          { return wp_count; }
uint8_t Path_GetNodeCount(void)        { return node_count; }
