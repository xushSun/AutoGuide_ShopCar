/**
 * @file    bsp_callbacks.c
 * @brief   HAL中断回调 — 所有业务逻辑入口
 *
 * TIM4 100Hz: Encoder → UWB_Poll(printf) → HostUART → Gyro → Navi → Chassis → LED
 */

#include "Header.h"

static uint8_t gyro_inited = 0;
static float gyro_yaw_rad   = 0.0f;
static float lock_yaw_rad   = 0.0f;
static float lock_i          = 0.0f;
static uint8_t lock_active   = 0;

#define LOCK_KP  15.0f       /* 直行锁航向 (5→15 强力修偏)   */
#define LOCK_KI  3.0f         /* 积分兜底 (1→3 快积)          */
#define LOCK_IMAX  5.0f       /* 积分上限 (2→5)               */
#define LOCK_DEADBAND 0.0017f /* 死区0.1° (0.5°→0.1° 严格)  */

static float gyro_bias_dps = 0.0f;   /* Gz 零偏 °/s, 上电自动校准 */
static uint8_t gyro_cal_done = 0;

float g_gyro_yaw_err = 0.0f;         /* 当前航向误差 (rad), 诊断用 */
float g_gyro_corr_w  = 0.0f;         /* 当前修正角速度 (rad/s), 诊断用 */

float HLD_get_yaw_deg(void) { return gyro_yaw_rad * 57.29578f; }
float HLD_get_yaw_rad(void) { return gyro_yaw_rad; }
void  GyroYaw_Reset(void)   { gyro_yaw_rad = 0.0f; }

void HoldYaw_Lock(void)  { lock_yaw_rad = gyro_yaw_rad; lock_i = 0.0f; lock_active = 1; }
void HoldYaw_Release(void){ lock_active = 0; lock_i = 0.0f; }                         /* 转弯前解除, 避免锁干扰 */
void Gyro_Enable(void)   { gyro_inited = 1; }

void GyroHold_Update(void)
{
    if (!gyro_inited) return;
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();
    if (last_ms == 0) { last_ms = now; return; }
    float dt = (float)(now - last_ms) * 0.001f;
    last_ms = now;
    gyro_yaw_rad += (gyro_data.Gz - gyro_bias_dps) * 0.0174533f * dt;
}

/*
 * 上电零偏校准: 静止采样2秒, 取平均 Gz 作为零偏
 * 在 MPU6050_Init 成功后、TIM4 启动前调用
 */
void GyroHold_Calibrate(void)
{
    float sum = 0.0f;
    int32_t n = 0;
    uint32_t t0 = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - t0) < 2000) {
        if (GyroHold_ReadIMU()) {
            sum += gyro_data.Gz;
            n++;
        }
        HAL_Delay(5);
    }

    if (n > 50) {
        gyro_bias_dps = sum / (float)n;
        gyro_cal_done = 1;
        printf("GYRO: bias=%.3f dps (%d samples)\r\n", gyro_bias_dps, (int)n);
    } else {
        gyro_bias_dps = 0.0f;
        printf("GYRO: cal FAIL (n=%d)\r\n", (int)n);
    }
}

char GyroHold_ReadIMU(void)
{
    if (!gyro_inited) return 0;
    MPU6050_Read_All(&hi2c1, &gyro_data);
    return 1;
}

float GyroHold_ComputeW(void)
{
    if (!lock_active) { g_gyro_yaw_err = 0.0f; g_gyro_corr_w = 0.0f; return 0.0f; }
    float yaw_err = gyro_yaw_rad - lock_yaw_rad;
    g_gyro_yaw_err = yaw_err;

    /* 真实 dt: 用 HAL_GetTick() 测量相邻两次调用间隔 (默认 0.05s) */
    static uint32_t last_w_ms = 0;
    uint32_t now = HAL_GetTick();
    float dt = (last_w_ms == 0) ? 0.05f : (float)(now - last_w_ms) * 0.001f;
    if (dt <= 0.0f) dt = 0.05f;         /* 防 ticks 回绕 */
    if (dt >  0.5f) dt = 0.05f;         /* 防首次/long gap 放大 */
    last_w_ms = now;

    /*
     * I 始终累加 (不 gate 死区) — 陀螺 bias 产生的微小误差每周期攒,
     * 攒够了自然抵消稳态偏航。
     */
    lock_i += yaw_err * dt;
    if (lock_i >  LOCK_IMAX) lock_i =  LOCK_IMAX;
    if (lock_i < -LOCK_IMAX) lock_i = -LOCK_IMAX;

    /* 死区仅压 P 项 (防陀螺噪声抖舵), I 始终输出 (bias 抵消) */
    float p_term = LOCK_KP * yaw_err;
    if (yaw_err > -LOCK_DEADBAND && yaw_err < LOCK_DEADBAND) {
        p_term = 0.0f;
    }
    g_gyro_corr_w = p_term + LOCK_KI * lock_i;
    return g_gyro_corr_w;
}

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

/* printf → USART6 */
#ifdef __MICROLIB
int fputc(int ch, FILE *f) { HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1, 10); return ch; }
#else
int __io_putchar(int ch)   { HAL_UART_Transmit(&huart6, (uint8_t*)&ch, 1, 10); return ch; }
#endif

/* ==================================================================
 *  TIM4 100Hz
 * ================================================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) {

#if DEBUG_ENABLE_ENCODER
        Encoder_Update(0.01f);
#endif

#if DEBUG_ENABLE_UWB
        UWB_Poll();
#endif

#if DEBUG_ENABLE_HOST_UART
        HostUART_CheckTimeout();
#endif

        /* GyroHold_Update 已移至 main.c 主循环, 避免与 GyroHold_ReadIMU 竞争 gyro_data */

#if DEBUG_CHASSIS_READY
        Chassis_ControlLoop();
#endif

#if DEBUG_ENABLE_LED
        { static uint8_t t=0; if(++t>=50){HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);t=0;} }
#endif
    }
}

/* ==================================================================
 *  UART接收
 * ================================================================== */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
#if DEBUG_ENABLE_UWB
    if (huart->Instance == USART1) {
        extern uint8_t uwb_rx_byte;
        UWB_RxCallback(uwb_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uwb_rx_byte, 1);
    }
#endif
#if DEBUG_ENABLE_HOST_UART
    if (huart->Instance == USART6) {
        extern uint8_t host_rx_byte;
        HostUART_ParseByte(host_rx_byte);
        HAL_UART_Receive_IT(&huart6, &host_rx_byte, 1);
    }
#endif
}

void BSP_UART2_IDLE_Handler(void)
{
#if DEBUG_ENABLE_LIDAR
    uint16_t len = LD06_DMA_BUF_LEN - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    if (len > 0) Lidar_ParseFrame(Lidar_GetDMABuf(), len);
#else
    (void)huart2;
#endif
}
