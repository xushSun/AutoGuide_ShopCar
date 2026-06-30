/**
 * @file    bsp_uart.c
 * @brief   上位机通信 — 二进协议 + ASCII文本双模式
 */

#include "bsp_uart.h"
#include "usart.h"
#include "app_navigate.h"
#include "bsp_motor.h"
#include "coord_solver.h"
#include "path_planner.h"
#include "bsp_uwb.h"
#include <string.h>
#include <stdio.h>

typedef enum {
    HOST_STATE_IDLE = 0, HOST_STATE_CMD, HOST_STATE_LEN,
    HOST_STATE_PAYLOAD, HOST_STATE_CHECKSUM,
} HostRxState_t;

static HostRxState_t rx_state = HOST_STATE_IDLE;
static uint8_t       rx_buf[HOST_RX_BUF_LEN];
static uint8_t       rx_cmd, rx_len, rx_idx;
uint8_t              host_rx_byte;

/* ASCII输入 */
static char   text_line[32];
static uint8_t text_idx;
static uint8_t text_need_y;  /* 1=已收到x, 等y */
volatile uint8_t text_ready;
volatile int     text_tx, text_ty;
volatile uint32_t text_last_ms;  /* 最后一次按键时刻 */

static void host_dispatch(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd) {
    case HOST_CMD_TARGET:
        if (len >= 8) {
            int32_t x = (int32_t)(payload[0] | ((uint32_t)payload[1] << 8)
                     | ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24));
            int32_t y = (int32_t)(payload[4] | ((uint32_t)payload[5] << 8)
                     | ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24));

            /* 三传感器导航: 目标坐标 → 状态机自动规划路线 */
#if DEBUG_ENABLE_NAVI
            Nav_SetTarget(x, y);
#else
            uint8_t wp_cnt = Path_Plan(0, 0, x, y);
            if (wp_cnt > 0) {
                const PpWaypoint_t *wps = Path_GetWPs();
                for (uint8_t i = 0; i < wp_cnt && i < PP_MAX_WP; i++) {
                    uint8_t rsp[10];
                    rsp[0] = i;                  /* 路点序号 0..N-1 */
                    rsp[1] = wp_cnt;             /* 总路点数         */
                    rsp[2] = (uint8_t)(wps[i].x_mm & 0xFF);
                    rsp[3] = (uint8_t)((wps[i].x_mm >> 8) & 0xFF);
                    rsp[4] = (uint8_t)((wps[i].x_mm >> 16) & 0xFF);
                    rsp[5] = (uint8_t)((wps[i].x_mm >> 24) & 0xFF);
                    rsp[6] = (uint8_t)(wps[i].y_mm & 0xFF);
                    rsp[7] = (uint8_t)((wps[i].y_mm >> 8) & 0xFF);
                    rsp[8] = (uint8_t)((wps[i].y_mm >> 16) & 0xFF);
                    rsp[9] = (uint8_t)((wps[i].y_mm >> 24) & 0xFF);
                    HostUART_SendPacket(HOST_CMD_PATH_WP, rsp, 10);
                }
            }
#endif
        }
        break;

#if DEBUG_ENABLE_NAVI
    case HOST_CMD_STOP:      APP_Navigate_Stop(); break;
#endif

#if DEBUG_ENABLE_MOTOR
    case HOST_CMD_OPENLOOP:
        if (len >= 2) {
            APP_Navigate_Stop();
            int16_t l = (int16_t)((int32_t)payload[0] * MOTOR_PWM_MAX / 100);
            int16_t r = (int16_t)((int32_t)payload[1] * MOTOR_PWM_MAX / 100);
            Motor_SetBoth(l, r);
        }
        break;
#endif

#if DEBUG_ENABLE_COORD_SOLVER
    case HOST_CMD_SET_ANCHORS:
        if (len >= 8) {
            int16_t ax = (int16_t)(payload[0] | ((uint16_t)payload[1] << 8));
            int16_t ay = (int16_t)(payload[2] | ((uint16_t)payload[3] << 8));
            int16_t bx = (int16_t)(payload[4] | ((uint16_t)payload[5] << 8));
            int16_t by = (int16_t)(payload[6] | ((uint16_t)payload[7] << 8));
            CoordSolver_SetAnchors(ax, ay, bx, by,
                                   UWB_A_X_M, UWB_A_Y_M,
                                   UWB_B_X_M, UWB_B_Y_M);
        }
        break;
#endif

    default: break;
    }
}

/* ── 前向声明 ── */
static void parse_ascii_number(void);

/* ── 延时调参指令 (A0.5 / B1.2 / C0.3 / ? 帮助) ── */
static void parse_delay_cmd(void)
{
    if (text_idx == 0) return;
    text_line[text_idx] = '\0';

    char cmd = text_line[0];

    /* ? / h → 帮助 */
    if (cmd == '?' || cmd == 'h' || cmd == 'H') {
        extern float uwb_delay_a_m, uwb_delay_b_m, uwb_delay_c_m;
        printf("\r\n=== DELAY ===\r\n");
        printf("A=%.2f  B=%.2f  C=%.2f\r\n", uwb_delay_a_m, uwb_delay_b_m, uwb_delay_c_m);
        printf("Usage: A0.5  B1.2  C0.3\r\n");
        text_idx = 0;
        return;
    }

    /* A/B/C 开头的延时指令 */
    if ((cmd != 'A' && cmd != 'B' && cmd != 'C' &&
         cmd != 'a' && cmd != 'b' && cmd != 'c') || text_idx < 2) {
        /* 不匹配 → 回退给数字解析 */
        parse_ascii_number();
        return;
    }

    /* 解析 float: 整数 + 可选小数 */
    float vf = 0.0f;
    uint8_t j = 1;  /* 跳过首字母 */
    float frac = 0.1f;
    uint8_t in_frac = 0;

    for (; text_line[j]; j++) {
        if (text_line[j] == '.') { in_frac = 1; continue; }
        if (text_line[j] < '0' || text_line[j] > '9') break;
        if (in_frac) { vf += (float)(text_line[j]-'0') * frac; frac *= 0.1f; }
        else         { vf = vf * 10.0f + (float)(text_line[j]-'0'); }
    }

    UWB_SetDelay(cmd, vf);
    printf("OK %c=%.2f\r\n", (cmd>='a'&&cmd<='c')?cmd-32:cmd, vf);
    text_idx = 0;
}

/* ── ASCII解析 (独立函数) ── */
static void parse_ascii_number(void)
{
    if (text_idx == 0) return;
    text_line[text_idx] = '\0';
    uint8_t j = 0, neg = 0;
    if (text_line[0] == '-') { neg = 1; j = 1; }
    int v = 0;
    for (; text_line[j]; j++) v = v*10 + text_line[j] - '0';
    if (neg) v = -v;

    if (text_need_y) {
        text_ty = v;
        text_need_y = 0;
        text_ready = 1;
        /* ISR内直接发路点 (纯寄存器, 无sprintf/无HAL) */
        #define T(c) do{while(!(USART6->SR&USART_SR_TXE));USART6->DR=(c);}while(0)
        #define W(s) do{const char *p_=(s);while(*p_)T(*p_++);}while(0)
        #define WI(vv) do{ \
            int vv_=(vv);uint8_t t_,d_[8],n_=0;if(vv_<0){T('-');vv_=-vv_;} \
            do{d_[n_++]='0'+vv_%10;vv_/=10;}while(vv_); \
            for(t_=n_;t_>0;)T(d_[--t_]);}while(0)
        const UWB_State_t *u = UWB_GetState();
        int32_t swx, swy;
#if DEBUG_ENABLE_UWB && DEBUG_ENABLE_COORD_SOLVER
        CoordSolver_Transform(u->x_m, u->y_m, &swx, &swy);
#else
        /* 纯路径规划模式: 假设起点(0,0) */
        (void)u; swx = 0; swy = 0;
#endif
        uint8_t wc = Path_Plan(swx, swy, text_tx*1000, text_ty*1000);
        W("\r\nfrom "); WI((int)swx); T(','); WI((int)swy);
        W(" to "); WI(text_tx*1000); T(','); WI(text_ty*1000);
        W("mm  WPs="); WI(wc); W("\r\n");
        const PpWaypoint_t *wp = Path_GetWPs(); uint8_t wi;
        for (wi=0; wi<wc; wi++) {
            T(' '); WI((int)(wp[wi].x_mm)); T(','); WI((int)(wp[wi].y_mm)); W("\r\n");
        }
        #undef WI
        #undef W
        #undef T
    } else {
        text_tx = v;
        text_need_y = 1;
    }
    text_idx = 0;
}

void HostUART_Init(void)
{
    rx_state = HOST_STATE_IDLE;
    rx_idx = 0; text_idx = 0; text_need_y = 0; text_ready = 0; text_last_ms = 0;
    memset(rx_buf, 0, sizeof(rx_buf));
    memset(text_line, 0, sizeof(text_line));
    HAL_UART_Receive_IT(&huart6, &host_rx_byte, 1);
}

void HostUART_ParseByte(uint8_t byte)
{
    /* 非0xA5 → ASCII */
    if (byte != HOST_HEADER) {
        text_last_ms = HAL_GetTick();
        if (byte == ' ' || byte == '\r' || byte == '\n') {
            if (text_idx > 0) {
                char c0 = text_line[0];
                if (c0 == 'A' || c0 == 'B' || c0 == 'C' ||
                    c0 == 'a' || c0 == 'b' || c0 == 'c' ||
                    c0 == '?' || c0 == 'h' || c0 == 'H')
                    parse_delay_cmd();
                else
                    parse_ascii_number();
            }
        } else {
            if (text_idx < 31) {
                text_line[text_idx++] = (char)byte;
                if (text_need_y && text_idx > 0) parse_ascii_number();
            }
        }
        rx_state = HOST_STATE_IDLE;
        return;
    }

    /* 二进制协议 */
    switch (rx_state) {
    case HOST_STATE_IDLE:
        if (byte == HOST_HEADER) rx_state = HOST_STATE_CMD;
        break;
    case HOST_STATE_CMD:  rx_cmd=byte; rx_state=HOST_STATE_LEN; break;
    case HOST_STATE_LEN:
        rx_len=byte; rx_idx=0;
        rx_state = (rx_len<=HOST_RX_BUF_LEN) ? HOST_STATE_PAYLOAD : HOST_STATE_IDLE;
        break;
    case HOST_STATE_PAYLOAD:
        rx_buf[rx_idx++]=byte;
        if(rx_idx>=rx_len) rx_state=HOST_STATE_CHECKSUM;
        break;
    case HOST_STATE_CHECKSUM: {
        uint8_t calc=rx_cmd^rx_len;
        for(uint8_t i=0;i<rx_len;i++) calc^=rx_buf[i];
        if(calc==byte) host_dispatch(rx_cmd, rx_buf, rx_len);
        rx_state=HOST_STATE_IDLE;
        break;
    }
    default: rx_state=HOST_STATE_IDLE; break;
    }
}

void HostUART_CheckTimeout(void)
{
    if (text_need_y && text_idx > 0 &&
        ((uint32_t)(HAL_GetTick() - text_last_ms)) > 500) {
        parse_ascii_number();
    }
}

void HostUART_SendPose(int32_t x_mm, int32_t y_mm, int16_t yaw_001deg)
{
    uint8_t payload[10];
    payload[0]=(uint8_t)(x_mm&0xFF); payload[1]=(uint8_t)((x_mm>>8)&0xFF);
    payload[2]=(uint8_t)((x_mm>>16)&0xFF); payload[3]=(uint8_t)((x_mm>>24)&0xFF);
    payload[4]=(uint8_t)(y_mm&0xFF); payload[5]=(uint8_t)((y_mm>>8)&0xFF);
    payload[6]=(uint8_t)((y_mm>>16)&0xFF); payload[7]=(uint8_t)((y_mm>>24)&0xFF);
    payload[8]=(uint8_t)(yaw_001deg&0xFF); payload[9]=(uint8_t)((yaw_001deg>>8)&0xFF);
    HostUART_SendPacket(HOST_CMD_ID_POSE, payload, 10);
}

void HostUART_SendPacket(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t tx_buf[HOST_TX_BUF_LEN];
    uint8_t checksum = cmd ^ len;
    tx_buf[0]=HOST_HEADER; tx_buf[1]=cmd; tx_buf[2]=len;
    for(uint8_t i=0;i<len;i++){tx_buf[3+i]=payload[i];checksum^=payload[i];}
    tx_buf[3+len]=checksum;
    HAL_UART_Transmit(&huart6, tx_buf, 4+len, 10);
}
