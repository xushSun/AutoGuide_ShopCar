/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Header.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  App_Init();
  printf("NAVI MODE: START\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ── 陀螺仪 I2C 读取 + yaw 积分 (每5ms, 同上下文无竞争) ── */
#if DEBUG_ENABLE_GYRO
    {
        static uint32_t last_imu = 0;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_imu) >= 2) {
            last_imu = now;
            GyroHold_ReadIMU();
            GyroHold_Update();           /* 紧跟读取, 不跨 ISR 边界的 data race */
        }
    }
#endif

    /* ── LIDAR: 转弯开口检测 (右前方 0.5~1.0m 空地判断) ── */
#if DEBUG_ENABLE_LIDAR
    {
        static uint32_t last_lidar = 0;
        static uint32_t last_lidar_ok = 0;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_lidar) >= 30) {
            last_lidar = now;
            Lidar_Poll();
            uint16_t n;
            const LidarPoint_t *pts = Lidar_GetScan(&n);

            if (pts && n >= 100) {
                uint16_t right_zone_cnt = 0;
                uint16_t front_cnt = 0;
                uint16_t left_cnt  = 0;
                uint16_t right_cnt = 0;
                float front_m = 99.0f;
                float left_m  = 99.0f;
                float right_m = 99.0f;

                last_lidar_ok = now;

                for (uint16_t i = 0; i < n; i++) {
                    if (!pts[i].valid) continue;

                    /* 右前方 0.5~1.0m 区域点数: 少=开口空地 */
                    if (pts[i].x > 0.50f && pts[i].x < 1.00f && pts[i].y < -0.15f) {
                        right_zone_cnt++;
                    }
                    if (pts[i].x > 0.15f && pts[i].x < 0.90f &&
                        pts[i].y > -0.30f && pts[i].y < 0.30f) {
                        if (pts[i].x < front_m) front_m = pts[i].x;
                        front_cnt++;
                    }
                    if (pts[i].x > 0.12f && pts[i].x < 0.70f &&
                        pts[i].y > 0.18f && pts[i].y < 0.42f) {
                        if (pts[i].dist < left_m) left_m = pts[i].dist;
                        left_cnt++;
                    }
                    if (pts[i].x > 0.12f && pts[i].x < 0.70f &&
                        pts[i].y < -0.18f && pts[i].y > -0.42f) {
                        if (pts[i].dist < right_m) right_m = pts[i].dist;
                        right_cnt++;
                    }
                }

                /* 点数<5 → 右前方空旷 → 可转弯 */
                g_lidar_turn_open = (right_zone_cnt < 5) ? 1 : 0;
                g_lidar_front_m = (front_cnt >= 2) ? front_m : 99.0f;
                g_lidar_left_m  = (left_cnt  >= 2) ? left_m  : 99.0f;
                g_lidar_right_m = (right_cnt >= 2) ? right_m : 99.0f;
            } else if ((uint32_t)(now - last_lidar_ok) > NAV_LIDAR_TIMEOUT_MS) {
                g_lidar_turn_open = 0;
                g_lidar_front_m   = 99.0f;
                g_lidar_left_m    = 99.0f;
                g_lidar_right_m   = 99.0f;
            }
        }
    }
#endif

    /* ── 导航 (每50ms) ── */
#if DEBUG_ENABLE_NAVI
    {
        static uint32_t last_nav = 0;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_nav) >= 50) {
            last_nav = now;
            /* 检查ISR投递的目标 (防数据竞争) */
            if (nav_target_pending) {
                Nav_SetTarget(nav_pending_x, nav_pending_y);
                nav_target_pending = 0;
            }
            APP_Navigate_Update();
        }
    }
#endif

    /* ── 诊断 (每2s) ── */
    {
        static uint32_t last_diag = 0;
        uint32_t now = HAL_GetTick();
        if ((uint32_t)(now - last_diag) >= 2000) {
            last_diag = now;
            /* 清除 USART6 ORE: printf 发送期间若有字节到达不丢中断 */
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            huart6.ErrorCode = HAL_UART_ERROR_NONE;
            {
                const UWB_State_t *u = UWB_GetState();
                printf("UWB raw: x=%.2f y=%.2f q=%d valid=%d\r\n",
                       u->x_m, u->y_m, u->quality, u->data_valid);
            }
            printf("NAV: st=%d TO=%d la=%d dutL=%d dutR=%d odAvg=%.0f uwb=(%d,%d) spdL=%.0f spdR=%.0f yaw=%.0f err=%.2f cw=%.2f lf=%.2f ll=%.2f lr=%.2f\r\n",
                   nav_state, g_lidar_turn_open, g_lidar_assist_state, g_duty_L, g_duty_R,
                   (Encoder_GetLeftOdom_mm() + Encoder_GetRightOdom_mm()) * 0.5f,
                   (int)nav_pose.x_mm, (int)nav_pose.y_mm,
                   Encoder_GetLeftSpeed_mm_s(), Encoder_GetRightSpeed_mm_s(),
                   HLD_get_yaw_deg(),
                   g_gyro_yaw_err, g_gyro_corr_w,
                   g_lidar_front_m, g_lidar_left_m, g_lidar_right_m);
        }
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) { }
#endif
