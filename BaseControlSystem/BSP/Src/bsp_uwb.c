/**
 * @file    bsp_uwb.c
 * @brief   MK8000 UWB — trilat2D_3A linear least squares (移植自 Arduino ESP32_UWB_tag2D_3A)
 *
 *   步骤:
 *     1. USART1 IT → 各锚斜距 EMA
 *     2. 减天线延迟 → 勾股水平距 (三锚等高 dz=1.8m)
 *     3. trilat2D_3A 线性最小二乘
 *     4. printf 明文输出
 */

#include "bsp_uwb.h"
#include "usart.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ── 中断接收 ── */
uint8_t         uwb_rx_byte;
static uint8_t  frame[8], fi;
static uint32_t ftot;

/* ── 斜距 EMA ── */
#define A_RAW  0.15f
static float   ea, eb, ec;
static uint8_t rssiA, rssiB, rssiC;
static uint32_t msA, msB, msC;
static uint8_t okA, okB, okC;

UWB_State_t uwb_state;

/* ── 运行时天线延迟 (串口实时可调) ── */
float uwb_delay_a_m = -0.10f;
float uwb_delay_b_m =  0.00f;
float uwb_delay_c_m = -0.20f;

void UWB_SetDelay(char anchor, float val)
{
    if (val < 0.0f) val = 0.0f;
    if (val > 5.0f) val = 5.0f;
    if      (anchor == 'A' || anchor == 'a') uwb_delay_a_m = val;
    else if (anchor == 'B' || anchor == 'b') uwb_delay_b_m = val;
    else if (anchor == 'C' || anchor == 'c') uwb_delay_c_m = val;
}

/* ═══════════════════════════════════════════
 *  中断: MK8000 帧解析
 * ═══════════════════════════════════════════ */
void UWB_RxCallback(uint8_t byte)
{
    if (fi == 0) { if (byte == 0xF0) { frame[0] = byte; fi = 1; } return; }
    frame[fi++] = byte;
    if (fi < 8) return;
    fi = 0;

    if (frame[1] != 0x05 || frame[7] != 0xAA) return;
    uint16_t ad = frame[2] | ((uint16_t)frame[3] << 8);
    uint16_t d  = frame[4] | ((uint16_t)frame[5] << 8);
    uint8_t  rs = frame[6];
    if (!d || d > 50000 || rs < UWB_RSSI_MIN) return;

    uint32_t n = HAL_GetTick();
    ftot++;
    float dm = (float)d * 0.01f;

    if      (ad == UWB_ANCHOR_A_ID) { ea += A_RAW * (dm - ea); rssiA = rs; msA = n; okA = 1; }
    else if (ad == UWB_ANCHOR_B_ID) { eb += A_RAW * (dm - eb); rssiB = rs; msB = n; okB = 1; }
    else if (ad == UWB_ANCHOR_C_ID) { ec += A_RAW * (dm - ec); rssiC = rs; msC = n; okC = 1; }
}

/* ═══════════════════════════════════════════
 *  初始化
 * ═══════════════════════════════════════════ */
extern UART_HandleTypeDef huart1;

void UWB_Init(void)
{
    fi = ftot = 0;
    ea = eb = ec = 0;
    okA = okB = okC = 0;
    rssiA = rssiB = rssiC = 0;
    msA = msB = msC = 0;
    memset(&uwb_state, 0, sizeof(uwb_state));
    huart1.hdmarx = NULL;
    HAL_UART_Receive_IT(&huart1, &uwb_rx_byte, 1);
}

/* ═══════════════════════════════════════════
 *  斜距 → 水平距 (dz = 1.9 - 0.1 = 1.8m 统一)
 *
 *  近锚点场景: 测量值可能略<dz (噪声/标定误差)
 *  → 返回极小水平距而非0, 避免 trilat 矩阵病态
 * ═══════════════════════════════════════════ */
static inline float slant_to_horiz(float slant_m)
{
    float dz = UWB_ANCHOR_HEIGHT_M - UWB_TAG_HEIGHT_M;  /* 1.8m */
    float q  = slant_m * slant_m - dz * dz;
    if (q > 0.0f) return sqrtf(q);
    /* 近锚点: 水平距≈0, 返回 0.03m 防止矩阵奇异 */
    return 0.03f;
}

/* ═══════════════════════════════════════════
 *  trilat2D_3A — Arduino 原版线性最小二乘
 *  参考: ESP32_UWB_tag2D_3A.ino
 *        S. James Remington 1/2022
 *
 *  输入: d_[0..2] = 三个水平距 (m)
 *  输出: x_out, y_out = UWB 局部坐标 (m)
 *
 *  算法: b[i] = d[0]² - d[i]² + k[i] - k[0]
 *        [x,y]' = 0.5 * Ainv * b
 *  A仅依赖锚点坐标，只算一次
 * ═══════════════════════════════════════════ */
static uint8_t trilat2D_3A(const float d_in[], float *x_out, float *y_out, float *rmse)
{
    /* ── 锚点坐标 (UWB局部系: B=(0.2,0), X=B→A, Y⊥BA向北) ── */
    static const float ax0[3] = {8.88f, 0.2f, 9.66f};   /* A(西南), B(东南), C(西北) */
    static const float ay0[3] = {0.0f, 0.0f, 8.93f};

    /*
     *  重排: 选水平距最小的锚作参考, d[0]误差对b扰动最小
     *  → 近B时B作参考=db²≈0, 近A时A作参考=da²≈0
     */
    uint8_t idx[3] = {0, 1, 2};
    if (d_in[1] < d_in[0]) { uint8_t t=idx[0]; idx[0]=idx[1]; idx[1]=t; }
    if (d_in[2] < d_in[0]) { uint8_t t=idx[0]; idx[0]=idx[2]; idx[2]=t; }
    /* 现在 idx[0]=最近锚 */

    float ax[3], ay[3], d_[3];
    for (int i = 0; i < 3; i++) {
        ax[i] = ax0[idx[i]];
        ay[i] = ay0[idx[i]];
        d_[i] = d_in[idx[i]] > 0.05f ? d_in[idx[i]] : 0.05f;
    }

    /* k[i] = ax[i]² + ay[i]² */
    float k[3];
    for (int i = 0; i < 3; i++)
        k[i] = ax[i] * ax[i] + ay[i] * ay[i];

    /* A = [ax[1]-ax[0], ay[1]-ay[0]
            ax[2]-ax[0], ay[2]-ay[0]] */
    float A[2][2] = {
        {ax[1] - ax[0], ay[1] - ay[0]},
        {ax[2] - ax[0], ay[2] - ay[0]}
    };

    float det = A[0][0] * A[1][1] - A[1][0] * A[0][1];
    if (fabsf(det) < 1.0e-4f) return 0;

    det = 1.0f / det;
    float Ainv[2][2] = {
        { det * A[1][1], -det * A[0][1]},
        {-det * A[1][0],  det * A[0][0]}
    };

    /* b[i-1] = d[0]² - d[i]² + k[i] - k[0] */
    float b[2];
    for (int i = 1; i < 3; i++) {
        b[i - 1] = d_[0] * d_[0] - d_[i] * d_[i] + k[i] - k[0];
    }

    if (fabsf(b[0]) > 10000.0f || fabsf(b[1]) > 10000.0f)
        return 0;

    *x_out = 0.5f * (Ainv[0][0] * b[0] + Ainv[0][1] * b[1]);
    *y_out = 0.5f * (Ainv[1][0] * b[0] + Ainv[1][1] * b[1]);

    /* RMS vs 原始锚点坐标 */
    float rs = 0.0f;
    for (int i = 0; i < 3; i++) {
        float dx = *x_out - ax0[i];
        float dy = *y_out - ay0[i];
        float e  = d_in[i] - sqrtf(dx * dx + dy * dy);
        rs += e * e;
    }
    *rmse = sqrtf(rs / 3.0f);

    return 1;
}

/* ═══════════════════════════════════════════
 *  Poll (每 10ms)
 * ═══════════════════════════════════════════ */
void UWB_Poll(void)
{
    uint32_t now = HAL_GetTick();

    /* ── 存活检测 ── */
    uint8_t la = okA && ((uint32_t)(now - msA) <= UWB_TIMEOUT_MS);
    uint8_t lb = okB && ((uint32_t)(now - msB) <= UWB_TIMEOUT_MS);
    uint8_t lc = okC && ((uint32_t)(now - msC) <= UWB_TIMEOUT_MS);

    uint8_t uA = la && ea > 0.2f && ea < UWB_MAX_DIST_M;
    uint8_t uB = lb && eb > 0.2f && eb < UWB_MAX_DIST_M;
    uint8_t uC = lc && ec > 0.2f && ec < UWB_MAX_DIST_M;
    uint8_t n_valid = uA + uB + uC;

    uwb_state.data_valid = 0;

    if (n_valid >= 2) {
        /* ── 减天线延迟 + 斜距→水平距 ── */
        float da = uA ? slant_to_horiz(ea - uwb_delay_a_m) : 0.0f;
        float db = uB ? slant_to_horiz(eb - uwb_delay_b_m) : 0.0f;
        float dc = uC ? slant_to_horiz(ec - uwb_delay_c_m) : 0.0f;

        /* 钳位 (近锚点 slant_to_horiz 已返≈0.03m, 此处不再覆写) */
        if (da < 0.05f) da = 0.05f;
        if (db < 0.05f) db = 0.05f;
        if (dc < 0.05f) dc = 0.05f;

        float d_[3] = { da, db, dc };
        float x, y, r;

        if (n_valid == 3) {
            /*
             *  三锚全在线 → trilat2D_3A
             *  内部自动选最近锚作参考, d[0]²最小→误差传播最小
             */
            if (trilat2D_3A(d_, &x, &y, &r)) {
                uwb_state.x_m     = x;
                uwb_state.y_m     = (y > 0.0f) ? y : 0.0f;
                uwb_state.rmse_m  = r;
                uwb_state.data_valid = 1;
                uwb_state.quality = (r < 0.3f) ? 80 : ((r < 0.8f) ? 50 : ((r < 1.5f) ? 30 : 10));
            }
        } else if (uA && uB) {
            /* AB 回退: B=(0.2,0), A=(8.88,0), d=8.68 */
            float d2 = 8.68f * 8.68f;
            float xx = (db * db - da * da + d2) / (2.0f * 8.68f);
            x = 0.2f + xx;
            float y2 = db * db - xx * xx;
            y = (y2 > 0.0f) ? sqrtf(y2) : 0.0f;
            if (x >= -1.0f && x <= 10.0f && y <= 20.0f) {
                uwb_state.x_m = x; uwb_state.y_m = y;
                uwb_state.rmse_m = 0.0f;
                uwb_state.data_valid = 1;
                uwb_state.quality = 40;
            }
        } else if (uB && uC) {
            /* BC 回退: B=(0.2,0), C=(9.66,8.93) */
            float dcx = 9.66f - 0.2f, dcy = 8.93f;  /* B→C = (9.46, 8.93) */
            float dc_len = sqrtf(dcx*dcx + dcy*dcy);
            if (dc_len > 0.01f) {
                float p = (db*db - dc*dc + dc_len*dc_len) / (2.0f * dc_len);
                float y2 = db*db - p*p;
                float yp = (y2 > 0.0f) ? sqrtf(y2) : 0.0f;
                x = 0.2f + p*dcx/dc_len + yp*(-dcy/dc_len);
                y =          p*dcy/dc_len + yp*( dcx/dc_len);
                if (x >= -1.0f && x <= 10.0f && y <= 20.0f) {
                    uwb_state.x_m = x; uwb_state.y_m = y;
                    uwb_state.rmse_m = 0.0f;
                    uwb_state.data_valid = 1;
                    uwb_state.quality = 40;
                }
            }
        }
    }

    /* ── 3s保持旧值 ── */
    {
        static float    last_x = 0, last_y = 0;
        static uint32_t last_valid_ms = 0;
        static uint8_t  has_last = 0;

        if (uwb_state.data_valid) {
            last_x = uwb_state.x_m;
            last_y = uwb_state.y_m;
            has_last = 1;
            last_valid_ms = now;
        } else if (has_last && (uint32_t)(now - last_valid_ms) < 3000) {
            uwb_state.x_m  = last_x;
            uwb_state.y_m  = last_y;
            uwb_state.data_valid = 1;
        }
    }

    uwb_state.pkt_count     = ftot;

    uwb_state.anchor_A.dist_m = uA ? ea : 0; uwb_state.anchor_A.rssi = rssiA; uwb_state.anchor_A.last_update_ms = msA;
    uwb_state.anchor_B.dist_m = uB ? eb : 0; uwb_state.anchor_B.rssi = rssiB; uwb_state.anchor_B.last_update_ms = msB;
    uwb_state.anchor_C.dist_m = uC ? ec : 0; uwb_state.anchor_C.rssi = rssiC; uwb_state.anchor_C.last_update_ms = msC;

    /* ── 每1s printf ── */
    {
        static uint32_t lt = 0;
        if ((uint32_t)(now - lt) >= 1000) {
            lt = now;
            printf("X=%.2f Y=%.2f eA=%.2f eB=%.2f eC=%.2f Q=%d A=%c B=%c C=%c R=%.3f d=%.2f/%.2f/%.2f\r\n",
                   uwb_state.x_m, uwb_state.y_m,
                   ea, eb, ec,
                   uwb_state.quality,
                   uA ? 'V' : '.', uB ? 'V' : '.', uC ? 'V' : '.',
                   uwb_state.rmse_m,
                   uwb_delay_a_m, uwb_delay_b_m, uwb_delay_c_m);
        }
    }
}

const UWB_State_t * UWB_GetState(void) { return &uwb_state; }
