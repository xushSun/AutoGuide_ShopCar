/**
 * @file    uwb.c
 * @brief   MK8000 UWB tag — DMA circular + polling parser
 */

#include "uwb.h"
#include <string.h>
#include <math.h>

#define UWB_DMA_BUF_SIZE   512   /* 8-byte frames, 512B is plenty */
#define UWB_DISTANCE    0.15f  /* distance between anchors (m) */
/* ==========================================================================
 *  DMA buffer
 * ========================================================================== */
uint8_t         uwb_dma_buf[UWB_DMA_BUF_SIZE] __attribute__((aligned(4)));
static volatile uint32_t uwb_dma_rd;

/* ==========================================================================
 *  Frame parser state machine
 * ========================================================================== */
static uint8_t  frame[8];
static uint8_t  frame_idx;
static uint32_t frame_total;

/* ==========================================================================
 *  Global state
 * ========================================================================== */
volatile uwb_state_t g_uwb_state;
volatile uint32_t g_uwb_rx_bytes = 0;
volatile uint32_t g_uwb_hdr_cnt  = 0;
volatile uint8_t  g_uwb_debug_frame[8];  /* last 8-byte frame for debug */

/* ==========================================================================
 *  PARSE ONE BYTE
 * ========================================================================== */
static void uwb_parse_byte(uint8_t b)
{
    /* Wait for header 0xF0 */
    if (frame_idx == 0) {
        if (b == 0xF0) {
            frame[0] = b;
            frame_idx = 1;
            g_uwb_hdr_cnt++;
        }
        return;
    }

    frame[frame_idx++] = b;

    /* Frame complete? (8 bytes) */
    if (frame_idx >= 8) {
        frame_idx = 0;

        /* Save last frame for debug inspection */
        for (int i = 0; i < 8; i++) g_uwb_debug_frame[i] = frame[i];

        /* Validate: length byte, footer, and data length */
        if (frame[1] != 0x05 || frame[7] != 0xAA) return;

        uint16_t addr = frame[2] | ((uint16_t)frame[3] << 8);
        uint16_t dist = frame[4] | ((uint16_t)frame[5] << 8);
        uint8_t  rssi = frame[6];

        if (dist == 0 || dist > 50000) return;  /* 0 or >500m = invalid */

        float dist_m = (float)dist * 0.01f;      /* cm → m */
        uint32_t now = HAL_GetTick();

        if (addr == UWB_ANCHOR_A) {
            g_uwb_state.anchor_A.dist_raw = dist_m;
            g_uwb_state.anchor_A.rssi     = rssi;
            g_uwb_state.anchor_A.last_update_ms = now;

            /* EMA filter + spike rejection */
            {
                float prev = g_uwb_state.anchor_A.dist_m;
                float diff = dist_m - prev;
                if (diff < 0) diff = -diff;
                float k = (diff > 0.5f) ? 0.05f : 0.15f;  /* spike → low weight */
                g_uwb_state.anchor_A.dist_m = prev + k * (dist_m - prev);
            }
        } else if (addr == UWB_ANCHOR_B) {
            g_uwb_state.anchor_B.dist_raw = dist_m;
            g_uwb_state.anchor_B.rssi     = rssi;
            g_uwb_state.anchor_B.last_update_ms = now;

            {
                float prev = g_uwb_state.anchor_B.dist_m;
                float diff = dist_m - prev;
                if (diff < 0) diff = -diff;
                float k = (diff > 0.5f) ? 0.05f : 0.15f;
                g_uwb_state.anchor_B.dist_m = prev + k * (dist_m - prev);
            }
        }

        frame_total++;
        g_uwb_state.pkt_count = frame_total;
    }
}

/* ==========================================================================
 *  API
 * ========================================================================== */

void uwb_init(void)
{
    frame_idx   = 0;
    frame_total = 0;
    uwb_dma_rd  = 0;
    memset((void *)&g_uwb_state, 0, sizeof(g_uwb_state));

    /* Re-init UART4 at 230400 — same path UART5 used successfully */
    huart4.Init.BaudRate = 230400;
    HAL_UART_Init(&huart4);

    /* Kill UART interrupts, keep NVIC off */
    __HAL_UART_DISABLE_IT(&huart4, UART_IT_ERR | UART_IT_RXNE | UART_IT_IDLE);

    /* Start DMA */
    HAL_UART_Receive_DMA(&huart4, uwb_dma_buf, UWB_DMA_BUF_SIZE);
}

volatile uint32_t uwb_poll_cnt = 0;  /* debug: poll call count */

void uwb_poll(void)
{
    uwb_poll_cnt++;

    uint32_t wr = (UWB_DMA_BUF_SIZE -
                   __HAL_DMA_GET_COUNTER(&hdma_uart4_rx))
                  & (UWB_DMA_BUF_SIZE - 1);

    while (uwb_dma_rd != wr) {
        uwb_parse_byte(uwb_dma_buf[uwb_dma_rd]);
        g_uwb_rx_bytes++;
        uwb_dma_rd = (uwb_dma_rd + 1) & (UWB_DMA_BUF_SIZE - 1);
    }

    /* ---- Coordinate calculation: A(0,0), B(d,0) ---- */
    {
        const float d = UWB_DISTANCE;  /* anchor spacing (m) */
        float rA = g_uwb_state.anchor_A.dist_m;
        float rB = g_uwb_state.anchor_B.dist_m;

        if (rA > 0.01f && rB > 0.01f && rA < 50.0f && rB < 50.0f) {
            float rA2 = rA * rA;
            float rB2 = rB * rB;
            float d2  = d * d;

            g_uwb_state.x_m = (rA2 - rB2 + d2) / (2.0f * d);

            float y2 = rA2 - g_uwb_state.x_m * g_uwb_state.x_m;
            g_uwb_state.y_m = (y2 > 0.0f) ? sqrtf(y2) : 0.0f;
        }
    }
}

const volatile uwb_state_t *uwb_get_state(void)
{
    return &g_uwb_state;
}
