/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#define IMU_USE_MAG 1           // Enable magnetometer
#include "robot_actions.h"
#include "robot_sm.h"
#include "heading_fusion.h"
#include "system_health.h"
#include "i2c_utils.h"
#include "console_io.h"
#include "sabertooth.h"
#include "dispersion.h"
#include "proximity.h"
#include "mission.h"
#include "diagnostics.h"
#include "uart_lora.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG2RAD (0.01745329251994f)
#define RAD2DEG (57.2957795130823f)
#define PI_F      3.14159265f
#define TWO_PI_F  (2.0f * PI_F)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

IWDG_TypeDef *hiwdg = IWDG;  // Hardware watchdog timer (direct register access)

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// Global state machine and heading fusion (used by robot_actions.c)
RobotSM_t g_sm = {0};
HeadingFusion_t g_hf = {0};

// ANSI color codes for terminal output
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_RESET   "\033[0m"

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Dispersion_SetRate(uint8_t salt_rate, uint8_t brine_rate);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Volatile flags for ISR communication with main loop
volatile int stop_requested = 0;          // ESTOP button pressed
volatile uint8_t imu_ok = 1;              // IMU health status
volatile uint32_t imu_last_update_ms = 0; // Last IMU read timestamp
volatile uint8_t g_test_mode = 0;         // 1 = stay in test mode until reset

// UART Interrupt callback for GPS, Sabertooth, Dispersion (UART4), and LoRA (UART5)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        GPS_HAL_RxCpltCallback(huart);
    } else if (huart->Instance == USART1) {
      // Sabertooth feedback handler (Plain Text Serial responses)
      if (huart->pRxBuffPtr) {
        uint8_t byte = *(huart->pRxBuffPtr - 1); // HAL advances pRxBuffPtr after receive
        Sabertooth_ProcessRxByte(byte);
      }
    } else if (huart->Instance == UART4) {
        // Dispersion ESP32 response handler
        if (huart->pRxBuffPtr && huart->RxXferSize > 0) {
            Dispersion_RxByte(huart->pRxBuffPtr[0]);
        }
    } else if (huart->Instance == UART5) {
        // LoRA remote control command handler
        if (huart->pRxBuffPtr && huart->RxXferSize > 0) {
            LoRA_RxByte(huart->pRxBuffPtr[0]);
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin)   // CubeMX-defined user button pin
    {
        stop_requested = 1;
    }
}

void UART_Flush_All(void)
{
    __HAL_UART_FLUSH_DRREGISTER(&huart1);
    __HAL_UART_FLUSH_DRREGISTER(&huart2);
    __HAL_UART_FLUSH_DRREGISTER(&huart3);

    HAL_UART_DeInit(&huart1); HAL_UART_Init(&huart1);
    HAL_UART_DeInit(&huart2); HAL_UART_Init(&huart2);
    HAL_UART_DeInit(&huart3); HAL_UART_Init(&huart3);

    GPS_Init(&huart3);
}

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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); // Audio_RST low
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  
  // Console routing: printf -> USART2, GPS on USART3, Sabertooth on USART1
  ConsoleIO_t console = {0};
  Console_Init(&console, &huart2, &huart3, &huart1);
  Console_SetTestModeFlag(&g_test_mode);
  
  // UART_Flush_All(); // Commented out - was causing double initialization
  HAL_Delay(500);  // Wait for all hardware to stabilize
  
  // Clear console
  printf("\033[2J\033[H");

  // Initialize GPS module (requires USART3 already initialized)
  GPS_Init(&huart3);

  // I2C scan on boot: confirms IMU presence and bus health
  HAL_Delay(100);  // Small delay to ensure I2C is ready
  I2C_ScanBus(&hi2c1);
  HAL_Delay(100);
  
  // Health monitor first: establishes baseline state for safety logic
  SystemHealth_Init();
  
  // Hardware watchdog (IWDG): protects against deadlocks and blocking loops
  // Direct register access: Timeout = (Reload + 1) * 2 * Prescaler / LSI_FREQ
  // LSI_FREQ ≈ 32 kHz, Prescaler=64, Reload=4095 → ~2.0 second timeout
  // Sequence: unlock key register, set prescaler, set reload, start
  IWDG->KR = 0x5555;           // Unlock write to PR and RLR
  IWDG->PR = 3;                // Prescaler: 3 = divide by 64
  IWDG->RLR = 4095;            // Reload value (max = 4095)
  IWDG->KR = 0xCCCC;           // Start watchdog
  printf("[WATCHDOG] Hardware watchdog initialized (~2 second timeout)\r\n");
  
  // Initialize sensors and control subsystems
  IMU_Init(&hi2c1);
  RobotSM_Init(&g_sm, STATE_PAUSE);
  HeadingFusion_Init(&g_hf);
  Sabertooth_Init(&huart1);
  Sabertooth_StopAll(); // Ensure motors are stopped on boot/reset
  Proximity_Init();  // Initialize proximity sensors
  Dispersion_Init(&huart4); // Initialize dispersion system (salt + brine) with UART 4
  LoRA_Init(&huart5);       // Initialize LoRA remote control via UART 5
  Mission_Init();    // Initialize mission management module
  HAL_Delay(100);

  imu_last_update_ms = HAL_GetTick();
  imu_ok = 1;

  printf(ANSI_GREEN "========== ROBOTIC ANTI-ICING SYSTEM BOOTING ==========\r\n" ANSI_RESET);
  
    // IMU calibration with watchdog safety: timeout after 10 seconds
  if (imu_ok) {
      printf(ANSI_MAGENTA "Calibrating IMU... keep board still.\r\n" ANSI_RESET);
      uint32_t calib_start = HAL_GetTick();
      IMU_Calibrate(500, 5);  // 500 samples, 5ms each
      uint32_t calib_time = HAL_GetTick() - calib_start;
      if (calib_time < 10000) {  // Calibration completed within timeout
          printf(ANSI_GREEN "IMU Calibration Complete (%lu ms).\r\n" ANSI_RESET, calib_time);
      } else {
          printf(ANSI_YELLOW "IMU Calibration timeout - skipping.\r\n" ANSI_RESET);
          imu_ok = 0;
      }
  } else {
      printf(ANSI_YELLOW "WARNING: IMU not responding - skipping calibration.\r\n" ANSI_RESET);
  }

  // GPS fix gate: short wait to get a fix, then continue even if unavailable
  printf(ANSI_YELLOW "Waiting for GPS fix...\r\n" ANSI_RESET);
  uint32_t gps_timeout_ms = HAL_GetTick() + 10000;  // 10 second timeout
  while (!GPS_Get()->has_fix && HAL_GetTick() < gps_timeout_ms)
  {
      IWDG->KR = 0xAAAA;  // Refresh watchdog during boot wait
      HAL_Delay(200);
  }
  if (!GPS_Get()->has_fix) {
      printf(ANSI_YELLOW "WARNING: GPS fix timeout (10s). Proceeding with caution.\r\n" ANSI_RESET);
  } else {
      printf(ANSI_GREEN "GPS Fix Acquired.\r\n" ANSI_RESET);
  }

  // Final boot banner before entering control loop
  printf(ANSI_GREEN "SYSTEM READY.\r\n" ANSI_RESET);
  printf("\r\n" ANSI_YELLOW "10 seconds to interrupt main loop - type 'T' for test menu\r\n" ANSI_RESET);
    uint32_t wait_start_ms = HAL_GetTick();
    while ((HAL_GetTick() - wait_start_ms) < 10000)
    {
      // Refresh hardware watchdog during boot wait
      IWDG->KR = 0xAAAA;

      if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
        if (ch == 'T' || ch == 't') {
          Console_RxByte(ch, &g_sm);
          Console_RxByte('\r', &g_sm);
          break; // interrupt wait and enter test menu
        }
        Console_RxByte(ch, &g_sm);
      }
      HAL_Delay(1);
    }
  printf("\r\n");

  /* USER CODE END 2 */

  /* Main control loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_50hz_ms = HAL_GetTick();
  uint32_t last_1hz_ms  = HAL_GetTick();

  while (1)
  {
      // Refresh hardware watchdog to prevent reset
      IWDG->KR = 0xAAAA;  // Refresh (reload counter)
      
        // Poll USART2 for console input (non-interrupt, low-latency control)
      if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
          uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
          Console_RxByte(ch, &g_sm);
      }
      
	  uint32_t now_ms = HAL_GetTick();
	  GPS_Tick(now_ms);
	  const GPS_Data_t *gps = GPS_Get();

      // Test mode bypasses autonomous/manual control loop (diagnostics only)
      if (g_test_mode) {
        HAL_Delay(10);
        continue; // stay in test mode until reset
      }

      // Poll Sabertooth feedback (battery voltage, motor current, temperature)
      Sabertooth_PollFeedback();

      if ((now_ms - last_50hz_ms) >= 20)
      {
          last_50hz_ms += 20;

          static IMU_Status_t imu_last;
          IMU_Status_t imu = IMU_Read();
          imu_last = imu;

          if (!imu.ok)
          {
              imu_ok = 0;
          }
          else
          {
        	  imu_ok = 1;
        	  /* Use fixed 50 Hz timestep (20 ms) for consistency */
        	  float dt = 0.02f;
        	  imu_last_update_ms = now_ms;

        	  HeadingFusion_Update(&g_hf, &imu, gps, dt);

          }

          SystemHealthInputs_t sh = {
              .imu_ok = imu_ok,
              .gps_fix = gps->has_fix,
              .estop_button = stop_requested ? 1 : 0
          };

          if (stop_requested) stop_requested = 0;  // consume button press

          RobotState_t req;
          if (SystemHealth_SafetyCheck(&sh, RobotSM_Current(&g_sm), &req))
          {
              RobotSM_Request(&g_sm, req);
          }

	  	  // Transitions + update
	  	  RobotSM_HandleTransitions(&g_sm);
	  	  RobotSM_Update(&g_sm);

	  	  // LEDs
	  	  SystemHealth_UpdateLeds(RobotSM_Current(&g_sm), imu_ok, gps->has_fix);

          // Check for incoming LoRA commands and forward to state machine
          uint8_t lora_cmd = 0;
          if (LoRA_GetPendingCommand(&lora_cmd)) {
              RobotSM_Request(&g_sm, (RobotState_t)lora_cmd);
              printf("[MAIN] LoRA command received: new state = %u\r\n", lora_cmd);
          }

          // State machine task dispatch (Phase 2 and 3)
          RobotState_t current_state = RobotSM_Current(&g_sm);
          switch (current_state)
          {
              case STATE_MANUAL:
                  ManualControl_Task();
                  break;
              case STATE_AUTO:
                  AutonomousControl_Task();
                  Dispersion_Task(); // Monitor salt/brine ratios in auto mode
                  break;
              case STATE_ERROR:
                  Handle_Error();
                  break;
              case STATE_ESTOP:
                  Emergency_Stop();
                  break;
              case STATE_PAUSE:
              default:
                  // Paused - motors idle
                  break;
          }

          // Update LoRA periodic tasks (timeout checking, etc.)
          LoRA_Tick(now_ms);

          // ---- 1 Hz prints and telemetry (SKIP during test mode) ----
          if ((now_ms - last_1hz_ms) >= 1000 && !g_test_mode)
          {
              last_1hz_ms += 1000;
              
              // Print sensor status (IMU, GPS, fusion)
              Console_PrintStatus(NULL, &imu_last, gps, &g_hf);

              // Get proximity sensor readings
              uint16_t prox_left_cm = Proximity_ReadLeft();
              uint16_t prox_right_cm = Proximity_ReadRight();
              
              // Get current motor speeds from Sabertooth
              int m1_speed = Sabertooth_GetM1();
              int m2_speed = Sabertooth_GetM2();
              
              // Get dispersion rates
              uint8_t salt_rate = Dispersion_GetSaltRate();
              uint8_t brine_rate = Dispersion_GetBrineRate();
              
              // Get IMU temperature
              float imu_temp = imu_last.temperature_c;
              
              // Get GPS quality metrics
              uint8_t gps_num_sat = gps->num_satellites;
              float gps_hdop = gps->hdop;
              
              // Send comprehensive telemetry to LoRA ESP32 (mobile app / base station)
              // Includes: state, GPS position + quality, motor speeds, heading, pitch, dispersion rates, temperature, proximity
              LoRA_SendTelemetry(RobotSM_Current(&g_sm),
                                 gps->latitude_deg, gps->longitude_deg, gps->has_fix,
                                 gps_num_sat, gps_hdop,
                                 m1_speed, m2_speed,
                                 g_hf.yaw_deg, g_hf.pitch_deg,
                                 salt_rate, brine_rate, imu_temp,
                                 prox_left_cm, prox_right_cm);
              
              // Debug: Log comprehensive state with ANSI colors
              // Format proximity as "XXcm" or "NO_DETECT"
              char prox_left_str[16], prox_right_str[16];
              if (prox_left_cm == PROX_NO_DETECTION)
                  snprintf(prox_left_str, sizeof(prox_left_str), "NO_DETECT");
              else
                  snprintf(prox_left_str, sizeof(prox_left_str), "%u cm", prox_left_cm);
              
              if (prox_right_cm == PROX_NO_DETECTION)
                  snprintf(prox_right_str, sizeof(prox_right_str), "NO_DETECT");
              else
                  snprintf(prox_right_str, sizeof(prox_right_str), "%u cm", prox_right_cm);
              
              printf(ANSI_YELLOW "[MAIN] TELEM: mode=%u, GPS=%.4f,%.4f (fix:%u, sats:%u, hdop:%.1f), "
                     ANSI_CYAN "M1=%d, M2=%d, " ANSI_GREEN "Heading=%.1f deg, Pitch=%.1f deg, "
                     ANSI_MAGENTA "Salt=%u%%, Brine=%u%%, Temp=%.1f C, "
                     ANSI_RESET "Prox L=%s, R=%s\r\n",
                     RobotSM_Current(&g_sm),
                     gps->latitude_deg, gps->longitude_deg, gps->has_fix,
                     gps_num_sat, gps_hdop,
                     m1_speed, m2_speed,
                     g_hf.yaw_deg, g_hf.pitch_deg,
                     salt_rate, brine_rate, imu_temp,
                     prox_left_str, prox_right_str);
          }
      }  /* Close the 50 Hz if block */
  }  /* Close the while(1) loop */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 167;  // 168 MHz / 168 = 1 MHz (1 µs per tick)
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;  // Hardware noise filter: 8 clock cycles (reject <95ns glitches)
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;  // 84 MHz / 84 = 1 MHz (1 µs per tick)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 8;  // Hardware noise filter: 8 clock cycles (reject <95ns glitches)
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_MCK_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_MCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
