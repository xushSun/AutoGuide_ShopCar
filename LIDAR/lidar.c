/**
 * @file    lidar.c
 * @brief   LD06 DTOF radar — 5-sector obstacle detection
 */

#include "lidar.h"
#include <string.h>
#include <math.h>

/* ==========================================================================
 *  GLOBAL — DMA buffer
 * ========================================================================== */
uint8_t            g_lidar_dma_buf[LIDAR_DMA_BUF_SIZE] __attribute__((aligned(4)));
volatile uint32_t  g_lidar_dma_rd;

/* ==========================================================================
 *  GLOBAL — packet parser
 * ========================================================================== */
uint8_t            g_lidar_pkt[47];
volatile uint8_t   g_lidar_pkt_idx;
volatile uint32_t  g_lidar_pkt_total;

/* ==========================================================================
 *  GLOBAL — 5-sector state
 * ========================================================================== */
volatile lidar_state_t g_lidar_state;

/* Diagnostic */
volatile uint32_t g_lidar_hdr_cnt  = 0;
volatile uint32_t g_lidar_ver_fail = 0;
volatile uint32_t g_lidar_crc_fail = 0;
volatile uint32_t g_lidar_rx_bytes = 0;

/* ==========================================================================
 *  CRC8 table from LD06 manual
 * ========================================================================== */
static const uint8_t ld06_crc_table[256] = {
    0x00,0x4d,0x9a,0xd7,0x79,0x34,0xe3,0xae,0xf2,0xbf,0x68,0x25,0x8b,0xc6,0x11,0x5c,
    0xa9,0xe4,0x33,0x7e,0xd0,0x9d,0x4a,0x07,0x5b,0x16,0xc1,0x8c,0x22,0x6f,0xb8,0xf5,
    0x1f,0x52,0x85,0xc8,0x66,0x2b,0xfc,0xb1,0xed,0xa0,0x77,0x3a,0x94,0xd9,0x0e,0x43,
    0xb6,0xfb,0x2c,0x61,0xcf,0x82,0x55,0x18,0x44,0x09,0xde,0x93,0x3d,0x70,0xa7,0xea,
    0x3e,0x73,0xa4,0xe9,0x47,0x0a,0xdd,0x90,0xcc,0x81,0x56,0x1b,0xb5,0xf8,0x2f,0x62,
    0x97,0xda,0x0d,0x40,0xee,0xa3,0x74,0x39,0x65,0x28,0xff,0xb2,0x1c,0x51,0x86,0xcb,
    0x21,0x6c,0xbb,0xf6,0x58,0x15,0xc2,0x8f,0xd3,0x9e,0x49,0x04,0xaa,0xe7,0x30,0x7d,
    0x88,0xc5,0x12,0x5f,0xf1,0xbc,0x6b,0x26,0x7a,0x37,0xe0,0xad,0x03,0x4e,0x99,0xd4,
    0x7c,0x31,0xe6,0xab,0x05,0x48,0x9f,0xd2,0x8e,0xc3,0x14,0x59,0xf7,0xba,0x6d,0x20,
    0xd5,0x98,0x4f,0x02,0xac,0xe1,0x36,0x7b,0x27,0x6a,0xbd,0xf0,0x5e,0x13,0xc4,0x89,
    0x63,0x2e,0xf9,0xb4,0x1a,0x57,0x80,0xcd,0x91,0xdc,0x0b,0x46,0xe8,0xa5,0x72,0x3f,
    0xca,0x87,0x50,0x1d,0xb3,0xfe,0x29,0x64,0x38,0x75,0xa2,0xef,0x41,0x0c,0xdb,0x96,
    0x42,0x0f,0xd8,0x95,0x3b,0x76,0xa1,0xec,0xb0,0xfd,0x2a,0x67,0xc9,0x84,0x53,0x1e,
    0xeb,0xa6,0x71,0x3c,0x92,0xdf,0x08,0x45,0x19,0x54,0x83,0xce,0x60,0x2d,0xfa,0xb7,
    0x5d,0x10,0xc7,0x8a,0x24,0x69,0xbe,0xf3,0xaf,0xe2,0x35,0x78,0xd6,0x9b,0x4c,0x01,
    0xf4,0xb9,0x6e,0x23,0x8d,0xc0,0x17,0x5a,0x06,0x4b,0x9c,0xd1,0x7f,0x32,0xe5,0xa8,
};

static uint8_t ld06_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    uint16_t i;
    for (i = 0; i < len; i++)
        crc = ld06_crc_table[(crc ^ data[i]) & 0xFF];
    return crc;
}

/* ==========================================================================
 *  CLASSIFY ANGLE → SECTOR (±15° each)
 * ========================================================================== */
static lidar_dir_t classify_angle(float angle)
{
    /* Front: 345°~360°, 0°~15°  (center 0°) */
    if (angle >= 345.0f || angle < 15.0f)  return LIDAR_DIR_FRONT;
    /* Front-Left: 300°~330° (center 315°) */
    if (angle >= 300.0f && angle <= 330.0f) return LIDAR_DIR_FRONT_LEFT;
    /* Left: 255°~285° (center 270°) */
    if (angle >= 255.0f && angle <= 285.0f) return LIDAR_DIR_LEFT;
    /* Front-Right: 30°~60° (center 45°) */
    if (angle >= 30.0f  && angle <= 60.0f)  return LIDAR_DIR_FRONT_RIGHT;
    /* Right: 75°~105° (center 90°) */
    if (angle >= 75.0f  && angle <= 105.0f) return LIDAR_DIR_RIGHT;
    /* Not in any sector */
    return -1;
}

static void update_sector(lidar_dir_t d, float dist_m)
{
    if (d >= LIDAR_DIR_COUNT) return;
    g_lidar_state.dir[d].dist_m = dist_m;  /* keep latest, no reset */
}

static void calc_zones(void)
{
    uint8_t i;
    for (i = 0; i < LIDAR_DIR_COUNT; i++) {
        float d = g_lidar_state.dir[i].dist_m;
        if      (d < LIDAR_ZONE_DANGER_M) g_lidar_state.dir[i].zone = LIDAR_ZONE_DANGER;
        else if (d < LIDAR_ZONE_WARN_M)   g_lidar_state.dir[i].zone = LIDAR_ZONE_WARNING;
        else                               g_lidar_state.dir[i].zone = LIDAR_ZONE_CLEAR;
    }
}

static void reset_sectors(void)
{
    uint8_t i;
    for (i = 0; i < LIDAR_DIR_COUNT; i++) {
        g_lidar_state.dir[i].dist_m = 999.0f;
        g_lidar_state.dir[i].zone   = LIDAR_ZONE_CLEAR;
    }
}

/* ==========================================================================
 *  PARSE ONE BYTE
 * ========================================================================== */
static void parse_byte(uint8_t b)
{
    g_lidar_rx_bytes++;

    if (g_lidar_pkt_idx == 0) {
        if (b == 0x54) {
            g_lidar_pkt[0] = b;
            g_lidar_pkt_idx = 1;
            g_lidar_hdr_cnt++;
        }
        return;
    }

    g_lidar_pkt[g_lidar_pkt_idx++] = b;

    if (g_lidar_pkt_idx == 2) {
        uint8_t ver = (b >> 5) & 0x07;
        uint8_t n   = b & 0x1F;
        if (ver != 1 || n != 12) {
            g_lidar_ver_fail++;
            g_lidar_pkt_idx = 0;
            return;
        }
    }

    if (g_lidar_pkt_idx >= 47) {
        g_lidar_pkt_idx = 0;

        if (ld06_crc8(g_lidar_pkt, 46) != g_lidar_pkt[46]) {
            g_lidar_crc_fail++;
            return;
        }

        float ang0 = (float)(g_lidar_pkt[4] | (g_lidar_pkt[5] << 8)) * 0.01f;
        float ang1 = (float)(g_lidar_pkt[42] | (g_lidar_pkt[43] << 8)) * 0.01f;

        float step;
        if (ang1 >= ang0)
            step = (ang1 - ang0) / 11.0f;
        else
            step = (ang1 + 360.0f - ang0) / 11.0f;

        for (int i = 0; i < 12; i++) {
            int off = 6 + i * 3;
            uint16_t dist_mm = g_lidar_pkt[off] | (g_lidar_pkt[off + 1] << 8);
            uint8_t  conf    = g_lidar_pkt[off + 2];

            if (dist_mm == 0 || conf < 10 || dist_mm > 12000) continue;

            float angle = ang0 + step * (float)i;
            if (angle >= 360.0f) angle -= 360.0f;

            lidar_dir_t d = classify_angle(angle);
            if (d < LIDAR_DIR_COUNT) {
                update_sector(d, (float)dist_mm * 0.001f);
            }
        }

        g_lidar_pkt_total++;
        g_lidar_state.pkt_count = g_lidar_pkt_total;
    }
}

/* ==========================================================================
 *  API
 * ========================================================================== */

static float    g_last_end_angle = 0.0f;

void lidar_init(void)
{
    g_lidar_pkt_idx    = 0;
    g_lidar_pkt_total  = 0;
    g_lidar_dma_rd     = 0;
    g_lidar_rx_bytes   = 0;
    g_lidar_hdr_cnt    = 0;
    g_lidar_ver_fail   = 0;
    g_lidar_crc_fail   = 0;
    g_last_end_angle   = 0.0f;

    reset_sectors();

    HAL_UART_Receive_DMA(&huart5, g_lidar_dma_buf, LIDAR_DMA_BUF_SIZE);
}

volatile uint32_t g_lidar_poll_cnt = 0;

void lidar_poll(void)
{
    g_lidar_poll_cnt++;

    uint32_t wr = (LIDAR_DMA_BUF_SIZE -
                    __HAL_DMA_GET_COUNTER(&hdma_uart5_rx))
                   & (LIDAR_DMA_BUF_SIZE - 1);

    while (g_lidar_dma_rd != wr) {
        parse_byte(g_lidar_dma_buf[g_lidar_dma_rd]);
        g_lidar_dma_rd = (g_lidar_dma_rd + 1) & (LIDAR_DMA_BUF_SIZE - 1);
    }

    /* One zone-update pass after draining DMA */
    calc_zones();
}

const volatile lidar_state_t *lidar_get_state(void)
{
    return &g_lidar_state;
}

void lidar_dma_buf_reset(void)
{
    g_lidar_dma_rd = 0;
    HAL_UART_AbortReceive(&huart5);
    HAL_UART_Receive_DMA(&huart5, g_lidar_dma_buf, LIDAR_DMA_BUF_SIZE);
}

void lidar_idle_handler(void) {}
