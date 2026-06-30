/**
 * @file    uwb.h
 * @brief   MK8000 UWB tag — receives distances from anchors via UART4
 *
 *          Baud:   115200-8-N-1
 *          Frame:  0xF0 0x05 addr_L addr_H dist_L dist_H rssi 0xAA
 */

#ifndef __UWB_H
#define __UWB_H

#include "main.h"

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;

/* Known anchor addresses */
#define UWB_ANCHOR_A    0x0001
#define UWB_ANCHOR_B    0x0002

typedef struct {
    float dist_raw;        /* latest raw measurement (m)  */
    float dist_m;          /* filtered distance (m)       */
    uint8_t rssi;
    uint32_t last_update_ms;
} uwb_anchor_t;

typedef struct {
    uwb_anchor_t anchor_A;
    uwb_anchor_t anchor_B;
    float        x_m;        /* calculated cartesian X (m) */
    float        y_m;        /* calculated cartesian Y (m) */
    uint32_t     pkt_count;
} uwb_state_t;

void uwb_init(void);
void uwb_poll(void);
const volatile uwb_state_t *uwb_get_state(void);

extern volatile uwb_state_t g_uwb_state;
extern volatile uint32_t uwb_poll_cnt;
extern volatile uint32_t g_uwb_rx_bytes;
extern volatile uint32_t g_uwb_hdr_cnt;
extern volatile uint8_t  g_uwb_debug_frame[8];

#endif
