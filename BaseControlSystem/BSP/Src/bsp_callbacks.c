/**
 * @file    bsp_callbacks.c
 * @brief   HAL中断回调 — 所有业务逻辑入口
 *
 * TIM4 100Hz: Encoder → UWB_Poll(printf) → HostUART → Gyro → Navi → Chassis → LED
 */

#include "Header.h"

#define GYRO_SCALE  0.96f    /* MPU6050 yaw scale, tune by 90deg turn test */

static uint8_t gyro_inited = 0;
static float gyro_yaw_rad   = 0.0f;
static float lock_yaw_rad   = 0.0f;
static float lock_i          = 0.0f;
static uint8_t lock_active   = 0;

#define GYRO_STILL_SPEED_MM_S    8.0f
#define GYRO_STILL_DUTY          20
#define GYRO_STILL_HOLD_MS       300
#define GYRO_BIAS_EMA_ALPHA      0.02f
#define GYRO_STATIC_DEADBAND_DPS 0.08f

#define LOCK_KP  800.0f      /* 直行锁航向 → 占空比修正       */
#define LOCK_KI  20.0f        /* 积分兜底 → 占空比修正         */
#define LOCK_IMAX  300.0f     /* 积分上限 → 占空比修正         */
#define LOCK_DEADBAND 0.0017f /* 死区0.1° (0.5°→0.1° 严格)  */

static float gyro_bias_dps = 0.0f;   /* Gz 零偏 °/s, 上电自动校准 */

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
    static uint32_t still_since_ms = 0;
    uint32_t now = HAL_GetTick();
    if (last_ms == 0) { last_ms = now; return; }
    float dt = (float)(now - last_ms) * 0.001f;
    last_ms = now;

    float abs_l = Encoder_GetLeftSpeed_mm_s();
    float abs_r = Encoder_GetRightSpeed_mm_s();
    if (abs_l < 0.0f) abs_l = -abs_l;
    if (abs_r < 0.0f) abs_r = -abs_r;

    int16_t duty_l = g_duty_L;
    int16_t duty_r = g_duty_R;
    if (duty_l < 0) duty_l = -duty_l;
    if (duty_r < 0) duty_r = -duty_r;

    if (abs_l < GYRO_STILL_SPEED_MM_S && abs_r < GYRO_STILL_SPEED_MM_S &&
        duty_l < GYRO_STILL_DUTY && duty_r < GYRO_STILL_DUTY) {
        if (still_since_ms == 0) still_since_ms = now;
        if ((uint32_t)(now - still_since_ms) >= GYRO_STILL_HOLD_MS) {
            gyro_bias_dps += GYRO_BIAS_EMA_ALPHA * (gyro_data.Gz - gyro_bias_dps);
            g_gyro_yaw_err = 0.0f;
            g_gyro_corr_w = 0.0f;
            return;
        }
    } else {
        still_since_ms = 0;
    }

    float gz_corrected = gyro_data.Gz - gyro_bias_dps;
    if (gz_corrected > -GYRO_STATIC_DEADBAND_DPS && gz_corrected < GYRO_STATIC_DEADBAND_DPS) gz_corrected = 0.0f;
    gyro_yaw_rad += gz_corrected * 0.0174533f * dt * GYRO_SCALE;
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
    float i_term = LOCK_KI * lock_i;
    if (i_term >  LOCK_IMAX) i_term =  LOCK_IMAX;
    if (i_term < -LOCK_IMAX) i_term = -LOCK_IMAX;

    /* 死区仅压 P 项 (防陀螺噪声抖舵), I 始终输出 (bias 抵消) */
    float p_term = LOCK_KP * yaw_err;
    if (yaw_err > -LOCK_DEADBAND && yaw_err < LOCK_DEADBAND) {
        p_term = 0.0f;
    }
    g_gyro_corr_w = p_term + i_term;
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
        __HAL_UART_CLEAR_OREFLAG(&huart1);
        huart1.ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(&huart1, &uwb_rx_byte, 1);
    }
#endif
#if DEBUG_ENABLE_HOST_UART
    if (huart->Instance == USART6) {
        extern uint8_t host_rx_byte;
        HostUART_ParseByte(host_rx_byte);
        __HAL_UART_CLEAR_OREFLAG(&huart6);
        huart6.ErrorCode = HAL_UART_ERROR_NONE;
        HAL_UART_Receive_IT(&huart6, &host_rx_byte, 1);
    }
#endif
}

void BSP_UART2_IDLE_Handler(void)
{
#if DEBUG_ENABLE_LIDAR
    Lidar_MarkDataPending();
#else
    (void)huart2;
#endif
}
