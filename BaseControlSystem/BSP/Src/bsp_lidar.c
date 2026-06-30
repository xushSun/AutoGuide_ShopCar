/**
 * @file    bsp_lidar.c
 * @brief   LD06激光雷达驱动 — 协议解析 + 点云双缓冲
 *
 * LD06 子帧结构（每圈~45个子帧, 10Hz）：
 *   [0x54] [VerLen] [Speed×2] [StartAng×2] [N点×3B] [EndAng×2] [TS×2] [CRC8]
 *   VerLen: bit[7:5]=类型(1=点云), bit[4:0]=点数N(≤12)
 *   每点3B: 距离(uint16,mm) + 信号强度(uint8)
 *   坐标转换: 右手系, x=前 y=左, x=dist·cos(θ), y=dist·sin(θ)
 */

#include "bsp_lidar.h"
#include "usart.h"
#include "tim.h"
#include "bsp_callbacks.h"
#include <string.h>
#include <math.h>

/* LD06 协议常量 */
#define LD06_HEADER         0x54     /* 帧头 */
#define LD06_PKT_POINTS_MAX 12       /* 单子帧最多12点 */
#define LD06_ANGLE_SCALE    0.01f    /* 角度 LSB = 0.01° */
#define LD06_DEG_TO_RAD     0.01745329252f

/* DMA 循环接收缓冲（4字节对齐，防DMA未对齐错误） */
static uint8_t  dma_buf[LD06_DMA_BUF_LEN] __attribute__((aligned(4)));

/* 点云双缓冲（写Bank不被干扰，读Bank稳定） */
static LidarPoint_t scan_buf[2][LD06_SCAN_POINTS_MAX];
static uint16_t     scan_count[2];   /* 有效点数 */
static uint8_t      scan_ready[2];   /* 1=新数据可读 */
static uint8_t      active_bank;     /* 当前写入Bank(0/1) */

/* CRC8 查表（LD06专有多项式） */
static const uint8_t crc8_table[256] = {
    0x00,0x4D,0x9A,0xD7,0x79,0x34,0xE3,0xAE,0xF2,0xBF,0x68,0x25,0x8B,0xC6,0x11,0x5C,
    0xA9,0xE4,0x33,0x7E,0xD0,0x9D,0x4A,0x07,0x5B,0x16,0xC1,0x8C,0x22,0x6F,0xB8,0xF5,
    0x1F,0x52,0x85,0xC8,0x66,0x2B,0xFC,0xB1,0xED,0xA0,0x77,0x3A,0x94,0xD9,0x0E,0x43,
    0xB6,0xFB,0x2C,0x61,0xCF,0x82,0x55,0x18,0x44,0x09,0xDE,0x93,0x3D,0x70,0xA7,0xEA,
    0x3E,0x73,0xA4,0xE9,0x47,0x0A,0xDD,0x90,0xCC,0x81,0x56,0x1B,0xB5,0xF8,0x2F,0x62,
    0x97,0xDA,0x0D,0x40,0xEE,0xA3,0x74,0x39,0x65,0x28,0xFF,0xB2,0x1C,0x51,0x86,0xCB,
    0x21,0x6C,0xBB,0xF6,0x58,0x15,0xC2,0x8F,0xD3,0x9E,0x49,0x04,0xAA,0xE7,0x30,0x7D,
    0x88,0xC5,0x12,0x5F,0xF1,0xBC,0x6B,0x26,0x7A,0x37,0xE0,0xAD,0x03,0x4E,0x99,0xD4,
    0x7C,0x31,0xE6,0xAB,0x05,0x48,0x9F,0xD2,0x8E,0xC3,0x14,0x59,0xF7,0xBA,0x6D,0x20,
    0xD5,0x98,0x4F,0x02,0xAC,0xE1,0x36,0x7B,0x27,0x6A,0xBD,0xF0,0x5E,0x13,0xC4,0x89,
    0x63,0x2E,0xF9,0xB4,0x1A,0x57,0x80,0xCD,0x91,0xDC,0x0B,0x46,0xE8,0xA5,0x72,0x3F,
    0xCA,0x87,0x50,0x1D,0xB3,0xFE,0x29,0x64,0x38,0x75,0xA2,0xEF,0x41,0x0C,0xDB,0x96,
    0x42,0x0F,0xD8,0x95,0x3B,0x76,0xA1,0xEC,0xB0,0xFD,0x2A,0x67,0xC9,0x84,0x53,0x1E,
    0xEB,0xA6,0x71,0x3C,0x92,0xDF,0x08,0x45,0x19,0x54,0x83,0xCE,0x60,0x2D,0xFA,0xB7,
    0x5D,0x10,0xC7,0x8A,0x24,0x69,0xBE,0xF3,0xAF,0xE2,0x35,0x78,0xD6,0x9B,0x4C,0x01,
    0xF4,0xB9,0x6E,0x23,0x8D,0xC0,0x17,0x5A,0x06,0x4B,0x9C,0xD1,0x7F,0x32,0xE5,0xA8
};

/* ==========================================================================
 *  内部工具函数
 * ========================================================================== */

/* 查表计算CRC8 */
static uint8_t ld06_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* 小端序 uint16 解包 */
static inline uint16_t u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0]) | ((uint16_t)(p[1]) << 8);
}

/* 极坐标(mm, 0.01°) → 直角坐标(m) — 右手系，x前y左 */
static void polar_to_cart(float dist_mm, uint16_t angle_raw, LidarPoint_t *out)
{
    float ang_deg = (float)angle_raw * LD06_ANGLE_SCALE;
    float ang_rad = ang_deg * LD06_DEG_TO_RAD;

    out->dist  = dist_mm * 0.001f;          /* mm → m                     */
    out->angle = ang_rad;
    out->x     = out->dist * cosf(ang_rad);
    out->y     = out->dist * sinf(ang_rad);
}

/* ==========================================================================
 *  对外接口
 * ========================================================================== */

/* 初始化：清空缓冲 → 使能USART2 IDLE中断 → 启动DMA循环接收 */
void Lidar_Init(void)
{
    memset(dma_buf,  0, sizeof(dma_buf));
    memset(scan_buf, 0, sizeof(scan_buf));

    active_bank   = 0;
    scan_count[0] = 0;  scan_count[1] = 0;
    scan_ready[0] = 0;  scan_ready[1] = 0;

    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);        /* 使能帧空闲中断 */
    HAL_UART_Receive_DMA(&huart2, dma_buf, LD06_DMA_BUF_LEN);  /* 启动DMA */
}

/**
 * @brief  解析DMA缓冲中的LD06子帧（由IDLE中断调用）
 * @note   逐子帧搜索0x54头 → CRC校验 → 极坐标转直角 → 写入active_bank
 *         点数过半时视为一圈，翻转双缓冲。
 */
void Lidar_ParseFrame(const uint8_t *buf, uint16_t len)
{
    uint16_t i = 0;
    uint16_t pkt_points;            /* N = VerLen & 0x1F                     */

    while (i + 1 < len) {

        /* — 搜索0x54帧头 — */
        if (buf[i] != LD06_HEADER) { i++; continue; }
        if (i + 8 > len) break;                        /* 不够最小子帧长度 */

        uint8_t verlen   = buf[i + 1];
        uint8_t pkt_type = (verlen >> 5) & 0x07;       /* bit[7:5]—1=点云 */
        pkt_points = verlen & 0x1F;                    /* bit[4:0]—N≤12   */
        if (pkt_type != 1) { i++; continue; }           /* 只处理点云帧   */

        uint16_t pkt_len = 2 + 2 + 2 + (uint16_t)pkt_points * 3 + 2 + 2 + 1;
        if (i + pkt_len > len) break;

        /* — CRC校验（不含CRC字节本身） — */
        uint8_t crc_calc = ld06_crc8(&buf[i], pkt_len - 1);
        uint8_t crc_rcvd = buf[i + pkt_len - 1];
        if (crc_calc != crc_rcvd) { i++; continue; }    /* 校验失败，跳过1字节重搜 */

        /* — 提取角度起止 — */
        uint16_t start_ang = u16_le(&buf[i + 4]);
        uint16_t end_ang   = u16_le(&buf[i + 6 + (uint16_t)pkt_points * 3]);

        /* — 解析N个点 — */
        uint16_t base = i + 6;                   /* 第一个点的数据字节偏移 */
        for (uint8_t n = 0; n < pkt_points; n++) {
            uint16_t dist_raw  = u16_le(&buf[base + n * 3]);
            uint8_t  intensity = buf[base + n * 3 + 2];

            if (dist_raw == 0 || intensity == 0) continue;   /* 无效点 */

            /* 角度线性插值（子帧起止角之间） */
            float frac = (pkt_points > 1) ? ((float)n / (float)(pkt_points - 1)) : 0.0f;
            uint16_t pt_ang_raw = (uint16_t)((float)start_ang
                                   + (float)((int16_t)(end_ang - start_ang)) * frac);

            uint16_t idx = scan_count[active_bank];
            if (idx >= LD06_SCAN_POINTS_MAX) continue;       /* Bank满，丢弃 */

            polar_to_cart((float)dist_raw, pt_ang_raw, &scan_buf[active_bank][idx]);
            scan_buf[active_bank][idx].intensity = intensity;
            scan_buf[active_bank][idx].valid     = 1;
            scan_count[active_bank] = idx + 1;
        }

        i += pkt_len;                                      /* 跳到下一个子帧 */
    }

    /* — 翻转检测：当前Bank点数过半即视为一圈，交换双缓冲 — */
    if (scan_count[active_bank] >= LD06_SCAN_POINTS_MAX / 2) {
        scan_ready[active_bank] = 1;
        active_bank ^= 1;                                  /* 切换写入Bank */
        scan_count[active_bank] = 0;
        scan_ready[active_bank] = 0;
    }
}

/* 获取最新一帧点云（读取非活跃Bank，消费 ready 标志） */
const LidarPoint_t * Lidar_GetScan(uint16_t *count)
{
    uint8_t read_bank = active_bank ^ 1;
    if (!scan_ready[read_bank]) {
        if (count) *count = 0;
        return NULL;
    }
    scan_ready[read_bank] = 0;   /* 消费标志，避免重复输出同一帧 */
    if (count) *count = scan_count[read_bank];
    return scan_buf[read_bank];
}

/* 启动雷达电机 (TIM10 CH1 → PB8, 30kHz, 50%占空比) */
void Lidar_MotorOn(void)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 1666);   /* 3332/2 */
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

/* 停止雷达电机 */
void Lidar_MotorOff(void)
{
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Stop(&htim10, TIM_CHANNEL_1);
}

/* 非阻塞查询是否有新帧（仅查不消费，GetScan 内消费 flag） */
uint8_t Lidar_IsNewScanReady(void)
{
    uint8_t read_bank = active_bank ^ 1;
    return scan_ready[read_bank];
}

/* 返回DMA缓冲指针（供bsp_callbacks计算接收字节数） */
uint8_t * Lidar_GetDMABuf(void)
{
    return dma_buf;
}
