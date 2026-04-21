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

/* main.c is the top-level orchestrator for the STM32 firmware. It brings up
 * peripherals and modules, then runs the periodic control loop that ties
 * together sensing, safety, state transitions, mission execution, and LoRa I/O.
 */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG2RAD (0.01745329251994f)
#define RAD2DEG (57.2957795130823f)
#define PI_F      3.14159265f
#define TWO_PI_F  (2.0f * PI_F)
#define MANUAL_COMMAND_HOLD_TIMEOUT_MS 750u
#define MANUAL_DRIVE_STEP_PCT 12
#define MANUAL_TURN_STEP_PCT 10
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

// High-rate UART console logs can interfere with interactive typing on USART2.
#define MAIN_VERBOSE_TELEMETRY_LOGS 0

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
static uint8_t s_disp_uart4_rx_byte = 0;
static uint8_t s_lora_uart5_rx_byte = 0;
volatile uint8_t g_demo_mode_active = 0;

static void PrintResetCause(void)
{
  uint8_t pin_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) != RESET) ? 1u : 0u;
  uint8_t por_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) != RESET) ? 1u : 0u;
  uint8_t sw_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) != RESET) ? 1u : 0u;
  uint8_t iwdg_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET) ? 1u : 0u;
  uint8_t wwdg_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) != RESET) ? 1u : 0u;
  uint8_t lowpwr_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) != RESET) ? 1u : 0u;

  printf("[BOOT] Reset flags: PIN=%u POR=%u SW=%u IWDG=%u WWDG=%u LPWR=%u\r\n",
         pin_reset,
         por_reset,
         sw_reset,
         iwdg_reset,
         wwdg_reset,
         lowpwr_reset);

  if (pin_reset) {
    printf(ANSI_YELLOW "[BOOT] Reset cause includes NRST pin event (reset button/external reset)\r\n" ANSI_RESET);
  }
  if (iwdg_reset || wwdg_reset) {
    printf(ANSI_YELLOW "[BOOT] Reset cause includes watchdog reset\r\n" ANSI_RESET);
  }
  if (sw_reset) {
    printf("[BOOT] Reset cause includes software reset\r\n");
  }

  __HAL_RCC_CLEAR_RESET_FLAGS();
}

static int clamp_manual_output(int value, int min_value, int max_value)
{
  if (value < min_value) return min_value;
  if (value > max_value) return max_value;
  return value;
}

static int step_toward_manual_output(int current_value, int target_value, int max_step)
{
  if (target_value > current_value + max_step) return current_value + max_step;
  if (target_value < current_value - max_step) return current_value - max_step;
  return target_value;
}

static int s_manual_drive_filtered_pct = 0;
static int s_manual_turn_filtered_pct = 0;
static uint32_t s_last_manual_command_ms = 0;
static uint8_t s_manual_watchdog_stopped = 0;
static uint32_t s_console_uart2_error_resets = 0;
static uint32_t s_last_console_uart2_recovery_log_ms = 0;
static uint32_t s_last_uart_error_log_ms = 0;

static uint8_t PollConsoleUart2(RobotSM_t *sm, uint8_t boot_window)
{
  uint8_t test_menu_requested = 0;

  // Recover from UART line errors that can stall RX handling until SR/DR are drained.
  if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) ||
      __HAL_UART_GET_FLAG(&huart2, UART_FLAG_FE)  ||
      __HAL_UART_GET_FLAG(&huart2, UART_FLAG_NE)  ||
      __HAL_UART_GET_FLAG(&huart2, UART_FLAG_PE)) {
    volatile uint32_t sr = huart2.Instance->SR;
    volatile uint32_t dr = huart2.Instance->DR;
    (void)sr;
    (void)dr;
    s_console_uart2_error_resets++;

    const uint32_t now_ms = HAL_GetTick();
    if (s_last_console_uart2_recovery_log_ms == 0u ||
        (now_ms - s_last_console_uart2_recovery_log_ms) >= 15000u) {
      printf("[CONSOLE] USART2 RX recovered from line error (count=%lu)\r\n",
             (unsigned long)s_console_uart2_error_resets);
      s_last_console_uart2_recovery_log_ms = now_ms;
    }
  }

  // Drain all pending bytes so bursts from terminal do not overrun polling cadence.
  while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
    uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);

    if (boot_window && (ch == 'S' || ch == 's')) {
      Dispersion_BypassStartupCheck();
      printf("[DISP] Startup check bypass requested by console key '%c'\r\n", ch);
      continue;
    }

    if (boot_window && (ch == 'T' || ch == 't')) {
      Console_ProcessCommand("T", sm);
      test_menu_requested = 1;
      continue;
    }

    Console_RxByte(ch, sm);
  }

  return test_menu_requested;
}

static void HandleLoRaManualCommand(LoRA_ManualCommand_t cmd)
{
  int m1_speed = 0;
  int m2_speed = 0;
  int drive_pct = 0;
  int turn_pct = 0;
  const char *cmd_name = "UNKNOWN";

  switch (cmd)
  {
    case LORA_MANUAL_CMD_FORWARD:
      cmd_name = "FORWARD";
      s_manual_drive_filtered_pct = 48;
      s_manual_turn_filtered_pct = 0;
      m1_speed = 40;
      m2_speed = 40;
      break;
    case LORA_MANUAL_CMD_BACK:
      cmd_name = "BACK";
      s_manual_drive_filtered_pct = -42;
      s_manual_turn_filtered_pct = 0;
      m1_speed = -35;
      m2_speed = -35;
      break;
    case LORA_MANUAL_CMD_LEFT:
      cmd_name = "LEFT";
      s_manual_drive_filtered_pct = 0;
      s_manual_turn_filtered_pct = -32;
      m1_speed = -25;
      m2_speed = 25;
      break;
    case LORA_MANUAL_CMD_RIGHT:
      cmd_name = "RIGHT";
      s_manual_drive_filtered_pct = 0;
      s_manual_turn_filtered_pct = 32;
      m1_speed = 25;
      m2_speed = -25;
      break;
    case LORA_MANUAL_CMD_DRIVE: {
      cmd_name = "DRIVE";
      const int requested_drive = clamp_manual_output(LoRA_GetManualDrivePct(), -100, 100);
      const int requested_turn = clamp_manual_output(LoRA_GetManualTurnPct(), -100, 100);

      if (requested_drive == 0 && requested_turn == 0) {
        s_manual_drive_filtered_pct = 0;
        s_manual_turn_filtered_pct = 0;
        m1_speed = 0;
        m2_speed = 0;
        break;
      } 

      s_manual_drive_filtered_pct = step_toward_manual_output(s_manual_drive_filtered_pct, requested_drive, MANUAL_DRIVE_STEP_PCT);
      s_manual_turn_filtered_pct = step_toward_manual_output(s_manual_turn_filtered_pct, requested_turn, MANUAL_TURN_STEP_PCT);

      drive_pct = s_manual_drive_filtered_pct;
      turn_pct = s_manual_turn_filtered_pct;

      const int drive_component = drive_pct;
      const int turn_component = (turn_pct * 40) / 100;
      m1_speed = clamp_manual_output(drive_component + turn_component, -100, 100);
      m2_speed = clamp_manual_output(drive_component - turn_component, -100, 100);
      break;
    }
    case LORA_MANUAL_CMD_STOP:
      cmd_name = "STOP";
      s_manual_drive_filtered_pct = 0;
      s_manual_turn_filtered_pct = 0;
      m1_speed = 0;
      m2_speed = 0;
      break;
    case LORA_MANUAL_CMD_TEST_SALT: {
      const int salt_pct = clamp_manual_output(LoRA_GetManualDrivePct(), 0, 100);
      cmd_name = "TEST_SALT";
      Dispersion_SetRateDirect((uint8_t)salt_pct, 0);
      char ack_msg[64];
      snprintf(ack_msg, sizeof(ack_msg), "ACK:TEST_SALT:%d", salt_pct);
      LoRA_SendRaw(ack_msg);
      printf("[APP CMD RECEIVED] manual=%s pct=%d\r\n", cmd_name, salt_pct);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    }
    case LORA_MANUAL_CMD_TEST_BRINE: {
      const int brine_pct = clamp_manual_output(LoRA_GetManualTurnPct(), 0, 100);
      cmd_name = "TEST_BRINE";
      Dispersion_SetRateDirect(0, (uint8_t)brine_pct);
      char ack_msg[64];
      snprintf(ack_msg, sizeof(ack_msg), "ACK:TEST_BRINE:%d", brine_pct);
      LoRA_SendRaw(ack_msg);
      printf("[APP CMD RECEIVED] manual=%s pct=%d\r\n", cmd_name, brine_pct);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    }
    case LORA_MANUAL_CMD_AGITATOR_ON:
      cmd_name = "AGITATOR_ON";
      Dispersion_SendRaw("AGITATOR ON");
      LoRA_SendRaw("ACK:AGITATOR:ON");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_AGITATOR_OFF:
      cmd_name = "AGITATOR_OFF";
      Dispersion_SendRaw("AGITATOR OFF");
      LoRA_SendRaw("ACK:AGITATOR:OFF");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_THROWER_ON:
      cmd_name = "THROWER_ON";
      Dispersion_SendRaw("THROWER ON");
      LoRA_SendRaw("ACK:THROWER:ON");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_THROWER_OFF:
      cmd_name = "THROWER_OFF";
      Dispersion_SendRaw("THROWER OFF");
      LoRA_SendRaw("ACK:THROWER:OFF");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_RELAY_ON:
      cmd_name = "RELAY_ON";
      Dispersion_SendRaw("RELAY ON");
      LoRA_SendRaw("ACK:RELAY:ON");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_RELAY_OFF:
      cmd_name = "RELAY_OFF";
      Dispersion_SendRaw("RELAY OFF");
      LoRA_SendRaw("ACK:RELAY:OFF");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_VIBRATION_ON:
      cmd_name = "VIBRATION_ON";
      Dispersion_SendRaw("VIBRATION ON");
      LoRA_SendRaw("ACK:VIBRATION:ON");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_VIBRATION_OFF:
      cmd_name = "VIBRATION_OFF";
      Dispersion_SendRaw("VIBRATION OFF");
      LoRA_SendRaw("ACK:VIBRATION:OFF");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_ALL_ON:
      cmd_name = "ALL_ON";
      Dispersion_SetRateDirect(100, 100);
      Dispersion_SendRaw("AGITATOR ON");
      Dispersion_SendRaw("THROWER ON");
      Dispersion_SendRaw("RELAY ON");
      Dispersion_SendRaw("VIBRATION ON");
      Sabertooth_SetM1(100);
      Sabertooth_SetM2(100);
      LoRA_SendRaw("ACK:ALLON");
      printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
      s_last_manual_command_ms = HAL_GetTick();
      s_manual_watchdog_stopped = 0;
      return;
    case LORA_MANUAL_CMD_NONE:
    default:
      return;
  }

  RobotSM_Request(&g_sm, STATE_MANUAL);
  Sabertooth_SetM1(m1_speed);
  Sabertooth_SetM2(m2_speed);
  s_last_manual_command_ms = HAL_GetTick();
  s_manual_watchdog_stopped = 0;

  const uint8_t send_manual_ack = (cmd != LORA_MANUAL_CMD_DRIVE && cmd != LORA_MANUAL_CMD_FORWARD && cmd != LORA_MANUAL_CMD_BACK && cmd != LORA_MANUAL_CMD_LEFT && cmd != LORA_MANUAL_CMD_RIGHT && cmd != LORA_MANUAL_CMD_STOP);
  if (send_manual_ack) {
    char ack_msg[64];
    snprintf(ack_msg, sizeof(ack_msg), "ACK:%s", cmd_name);
    LoRA_SendRaw(ack_msg);
    printf("[APP CMD RECEIVED] manual=%s\r\n", cmd_name);
    printf("[LoRa] manual cmd: %s -> M1=%d M2=%d\r\n", cmd_name, m1_speed, m2_speed);
  }
}

static uint8_t ParseLoRaJsonCmdInMain(const char *frame,
                                      LoRA_ManualCommand_t *out_manual_cmd,
                                      uint8_t *out_state,
                                      uint8_t *out_is_state)
{
  if (!frame || !out_manual_cmd || !out_state || !out_is_state) return 0;
  *out_manual_cmd = LORA_MANUAL_CMD_NONE;
  *out_state = 0;
  *out_is_state = 0;

  const char *json_start = strchr(frame, '{');
  if (!json_start) return 0;

  const char *key = "\"cmd\"";
  const char *p = strstr(json_start, key);
  if (!p) return 0;

  p += strlen(key);
  while (*p == ' ' || *p == '\t') p++;
  if (*p != ':') return 0;
  p++;
  while (*p == ' ' || *p == '\t') p++;
  if (*p != '"') return 0;
  p++;

  char cmd_buf[24];
  int cmd_len = 0;
  while (*p && *p != '"' && cmd_len < (int)sizeof(cmd_buf) - 1) {
    char c = *p++;
    if (c >= 'a' && c <= 'z') {
      c = (char)(c - 'a' + 'A');
    }
    cmd_buf[cmd_len++] = c;
  }
  cmd_buf[cmd_len] = '\0';
  if (cmd_len == 0 || *p != '"') return 0;

  if (strcmp(cmd_buf, "FORWARD") == 0) {
    *out_manual_cmd = LORA_MANUAL_CMD_FORWARD;
    return 1;
  }
  if (strcmp(cmd_buf, "BACK") == 0 || strcmp(cmd_buf, "BACKWARD") == 0) {
    *out_manual_cmd = LORA_MANUAL_CMD_BACK;
    return 1;
  }
  if (strcmp(cmd_buf, "LEFT") == 0) {
    *out_manual_cmd = LORA_MANUAL_CMD_LEFT;
    return 1;
  }
  if (strcmp(cmd_buf, "RIGHT") == 0) {
    *out_manual_cmd = LORA_MANUAL_CMD_RIGHT;
    return 1;
  }
  if (strcmp(cmd_buf, "STOP") == 0) {
    *out_manual_cmd = LORA_MANUAL_CMD_STOP;
    return 1;
  }

  if (strcmp(cmd_buf, "AUTO") == 0) {
    *out_is_state = 1;
    *out_state = 1;
    return 1;
  }
  if (strcmp(cmd_buf, "MANUAL") == 0) {
    *out_is_state = 1;
    *out_state = 0;
    return 1;
  }
  if (strcmp(cmd_buf, "PAUSE") == 0) {
    *out_is_state = 1;
    *out_state = 2;
    return 1;
  }
  if (strcmp(cmd_buf, "ESTOP") == 0) {
    *out_is_state = 1;
    *out_state = 4;
    return 1;
  }

  return 0;
}

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
      Dispersion_RxByte(s_disp_uart4_rx_byte);
      HAL_UART_Receive_IT(&huart4, &s_disp_uart4_rx_byte, 1);
    } else if (huart->Instance == UART5) {
        // LoRA remote control command handler
      LoRA_RxByte(s_lora_uart5_rx_byte);
      HAL_UART_Receive_IT(&huart5, &s_lora_uart5_rx_byte, 1);
    }
}

  void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
  {
    if (!huart) return;

    uint32_t err = HAL_UART_GetError(huart);

    if (huart->Instance == USART2) {
      if (err != HAL_UART_ERROR_NONE) {
        s_console_uart2_error_resets++;
        const uint32_t now_ms = HAL_GetTick();
        if (s_last_uart_error_log_ms == 0u || (now_ms - s_last_uart_error_log_ms) >= 15000u) {
          printf("[USART2] ERROR: 0x%08lX, recovered (count=%lu)\r\n",
                 err,
                 (unsigned long)s_console_uart2_error_resets);
          s_last_uart_error_log_ms = now_ms;
        }
      }
      volatile uint32_t sr = huart2.Instance->SR;
      volatile uint32_t dr = huart2.Instance->DR;
      (void)sr;
      (void)dr;
      __HAL_UART_CLEAR_OREFLAG(&huart2);
    } else if (huart->Instance == USART3) {
      if (err != HAL_UART_ERROR_NONE) {
        printf("[USART3] ERROR: 0x%08lX, rearming GPS RX\r\n", err);
      }
      volatile uint32_t sr = huart3.Instance->SR;
      volatile uint32_t dr = huart3.Instance->DR;
      (void)sr;
      (void)dr;
      __HAL_UART_CLEAR_OREFLAG(&huart3);
      GPS_CheckAndRecover();
    } else if (huart->Instance == UART4) {
      if (err != HAL_UART_ERROR_NONE) {
        printf("[UART4] ERROR: 0x%08lX, rearming RX\r\n", err);
      }
      __HAL_UART_CLEAR_OREFLAG(&huart4);
      HAL_UART_Receive_IT(&huart4, &s_disp_uart4_rx_byte, 1);
    } else if (huart->Instance == UART5) {
      if (err != HAL_UART_ERROR_NONE) {
        printf("[UART5] ERROR: 0x%08lX, rearming RX\r\n", err);
      }
      __HAL_UART_CLEAR_OREFLAG(&huart5);
      HAL_UART_Receive_IT(&huart5, &s_lora_uart5_rx_byte, 1);
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
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    __HAL_UART_CLEAR_OREFLAG(&huart2);

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
  PrintResetCause();
  // Console RX is polled in main loop; keep USART2 IRQ disabled to avoid
  // interrupt-side handling fighting with the polled console path.
  HAL_NVIC_DisableIRQ(USART2_IRQn);
  __HAL_UART_CLEAR_OREFLAG(&huart2);
  
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
  Sabertooth_SetRampM1(160);
  Sabertooth_SetRampM2(160);
  Sabertooth_StopAll(); // Ensure motors are stopped on boot/reset
  Proximity_Init();  // Initialize proximity sensors
  Dispersion_Init(&huart4); // Initialize dispersion system (salt + brine) with UART 4
  LoRA_Init(&huart5);       // Initialize LoRA remote control via UART 5
  HAL_UART_Receive_IT(&huart4, &s_disp_uart4_rx_byte, 1);
  HAL_UART_Receive_IT(&huart5, &s_lora_uart5_rx_byte, 1);
  printf("[UART MAP] SB-ESP: UART4 TX=PC10 RX=PC11 @230400\r\n");
  printf("[UART MAP] LoRa-ESP: UART5 TX=PC12 RX=PD2 @921600\r\n");
  Mission_Init();    // Initialize mission management module
  HAL_Delay(100);

  imu_last_update_ms = HAL_GetTick();
  // Check if IMU initialization actually succeeded; retry once if it failed.
  // The probe can miss the device on first attempt if the I2C bus was left in
  // a bad state by the scan above - a single retry with a clean bus is enough.
  imu_ok = IMU_GetInitStatus();
  if (!imu_ok) {
      printf(ANSI_YELLOW "⚠ WARNING: IMU initialization failed on boot - waiting 300ms then retrying\r\n" ANSI_RESET);
      IWDG->KR = 0xAAAA;
      HAL_Delay(300);
      printf(ANSI_CYAN "[BOOT] Retry IMU initialization (attempt 2/2)...\r\n" ANSI_RESET);
      IMU_Init(&hi2c1);
      imu_ok = IMU_GetInitStatus();
      if (!imu_ok) {
          printf(ANSI_RED "✗ IMU initialization failed on retry - will attempt recovery in main loop\r\n" ANSI_RESET);
      }
  }

  printf(ANSI_GREEN "========== ROBOTIC ANTI-ICING SYSTEM BOOTING ==========\r\n" ANSI_RESET);
  
  // IMU calibration (only if init succeeded; non-critical if skipped)
  if (imu_ok) {
      printf(ANSI_MAGENTA "[BOOT] Calibrating IMU (keep board still)...\r\n" ANSI_RESET);
      uint32_t calib_start = HAL_GetTick();
      IMU_Calibrate(500, 5);  // 500 samples, 5ms each (total ~2.5 sec)
      uint32_t calib_time = HAL_GetTick() - calib_start;
      
      if (calib_time < 10000) {  // Calibration completed within timeout
          if (IMU_IsCalibrated()) {
              printf(ANSI_GREEN "[IMU] ✓ Calibration Complete (%lu ms)\r\n" ANSI_RESET, calib_time);
          } else {
              printf(ANSI_YELLOW "[IMU] ⚠ Calibration ran but may be incomplete (%lu ms)\r\n" ANSI_RESET, calib_time);
              imu_ok = 0;  // Mark as problematic but don't block boot
          }
      } else {
          printf(ANSI_YELLOW "[IMU] ⚠ Calibration timeout (>10s) - skipping\r\n" ANSI_RESET);
          imu_ok = 0;  // Mark problematic, will retry in main loop
      }
  } else {
      printf(ANSI_YELLOW "[BOOT] ⚠ IMU not responding - skipping calibration, will retry in main loop\r\n" ANSI_RESET);
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

      if (PollConsoleUart2(&g_sm, 1)) {
        break; // interrupt wait and enter test menu
      }
      HAL_Delay(1);
    }
  printf("\r\n");

  /* USER CODE END 2 */

  /* Main control loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_50hz_ms = HAL_GetTick();
  uint32_t last_1hz_ms = HAL_GetTick();
  uint32_t last_1hz_status_ms = HAL_GetTick();
  uint32_t last_sabertooth_poll_ms = HAL_GetTick();
  const uint32_t status_print_interval_ms = 5000;
  const uint32_t console_quiet_window_ms = 1500;
  uint32_t last_lora_tx_ms = HAL_GetTick();
  uint32_t last_manual_lora_tx_ms = HAL_GetTick();
  const uint32_t lora_tx_interval_ms = 650;
  const uint32_t lora_manual_tx_interval_ms = 300;
  const uint32_t lora_manual_heartbeat_ms = 1200;
  const float lora_manual_heading_delta_min = 0.6f;
  const uint8_t lora_manual_feedback_enabled = 1u;
  float latest_imu_temp_c = 0.0f;
  uint32_t last_lora_raw_count_seen = LoRA_GetRawFrameCount();
    uint8_t startup_mode_sent = 0;
    uint8_t last_test_mode_seen = g_test_mode;

    if (g_test_mode) {
      Dispersion_BypassStartupCheck();
      startup_mode_sent = 1;
      printf("[DISP] Startup check bypassed (test mode active)\r\n");
    } else {
      Dispersion_BypassStartupCheck();
      startup_mode_sent = 1;
      printf("[DISP] Startup check bypassed (normal mode default)\r\n");
    }

  while (1)
  {
      // Refresh hardware watchdog to prevent reset
      IWDG->KR = 0xAAAA;  // Refresh (reload counter)
      
      // Poll USART2 for console input (non-interrupt, low-latency control)
      (void)PollConsoleUart2(&g_sm, 0);
      
	  uint32_t now_ms = HAL_GetTick();
    // Keep GPS parser active, but disable baud hopping in direct manual mode.
    GPS_SetAutoBaudEnabled((RobotSM_Current(&g_sm) == STATE_MANUAL) ? 0u : 1u);
    GPS_Tick(now_ms);
	  const GPS_Data_t *gps = GPS_Get();

      // Keep LoRa draining steady while prioritizing interactive console latency.
      LoRA_ProcessRxQueue(0);

      if (startup_mode_sent && g_test_mode && !last_test_mode_seen) {
          Dispersion_BypassStartupCheck();
          printf("[DISP] Startup check bypassed (entered test mode)\r\n");
      }
      last_test_mode_seen = g_test_mode;

      // Test mode bypasses autonomous/manual control actions, but still emits a
      // lightweight LoRa heartbeat so the rest of the stack can tell the STM is alive.
      if (g_test_mode) {
        // If a remote control command arrives while test mode is latched,
        // exit test mode so field-control is not silently blocked.
        uint8_t lora_test_state = 0;
        LoRA_ManualCommand_t lora_test_manual_cmd = LORA_MANUAL_CMD_NONE;
        if (LoRA_GetPendingCommand(&lora_test_state)) {
          g_test_mode = 0;
          RobotSM_Request(&g_sm, (RobotState_t)lora_test_state);
          printf("[MODE] Exiting test mode via LoRa state command (%u)\r\n", lora_test_state);
          continue;
        }
        if (LoRA_GetPendingManualCommand(&lora_test_manual_cmd)) {
          g_test_mode = 0;
          printf("[MODE] Exiting test mode via LoRa manual command\r\n");
          HandleLoRaManualCommand(lora_test_manual_cmd);
          continue;
        }

        LoRA_Tick(now_ms);
        if ((now_ms - last_lora_tx_ms) >= lora_tx_interval_ms) {
          last_lora_tx_ms = now_ms;
          LoRA_SendState(RobotSM_Current(&g_sm),
                         gps->latitude_deg, gps->longitude_deg,
                         Sabertooth_GetM1(), Sabertooth_GetM2());
        }
        HAL_Delay(1);
        continue; // stay in test mode until reset
      }

      // Fast command path: process incoming LoRa commands every loop iteration
      // so manual control and RESET are responsive even if sensor tasks stall.
      {
        uint8_t lora_handled_fast = 0;
        uint8_t demo_mode_enabled = 0;
        if (LoRA_GetPendingDemoModeChange(&demo_mode_enabled)) {
          g_demo_mode_active = demo_mode_enabled ? 1u : 0u;
          printf("[MODE] Demo mode %s via LoRa\r\n",
                 g_demo_mode_active ? "ENABLED" : "DISABLED");
          lora_handled_fast = 1;
        }
        if (LoRA_GetPendingResetRequest()) {
          uint32_t lora_age_ms = (LoRA_GetLastRxMs() > 0u) ? (now_ms - LoRA_GetLastRxMs()) : 0u;
          RobotSM_RequestEstopReset(&g_sm);
          printf("[APP CMD RECEIVED] reset=1 (age=%lums)\r\n", (unsigned long)lora_age_ms);
          lora_handled_fast = 1;
        }

        uint8_t lora_cmd_fast = 0;
        if (LoRA_GetPendingCommand(&lora_cmd_fast)) {
          uint32_t lora_age_ms = (LoRA_GetLastRxMs() > 0u) ? (now_ms - LoRA_GetLastRxMs()) : 0u;
          RobotSM_Request(&g_sm, (RobotState_t)lora_cmd_fast);
          const char *lora_raw_fast = LoRA_GetLastCommand();
          printf("[APP CMD RECEIVED] state=%u (age=%lums)\r\n", lora_cmd_fast, (unsigned long)lora_age_ms);
          printf("[LoRa] command received: %s -> state=%u\r\n",
                 (lora_raw_fast && lora_raw_fast[0] != '\0') ? lora_raw_fast : "<empty>",
                 lora_cmd_fast);
          lora_handled_fast = 1;
        }

        LoRA_ManualCommand_t lora_manual_cmd_fast = LORA_MANUAL_CMD_NONE;
        if (LoRA_GetPendingManualCommand(&lora_manual_cmd_fast)) {
          uint32_t lora_age_ms = (LoRA_GetLastRxMs() > 0u) ? (now_ms - LoRA_GetLastRxMs()) : 0u;
          HandleLoRaManualCommand(lora_manual_cmd_fast);
          (void)lora_age_ms;
          lora_handled_fast = 1;
        }

        if (!lora_handled_fast) {
          uint32_t raw_count_fast = LoRA_GetRawFrameCount();
          if (raw_count_fast != last_lora_raw_count_seen) {
            last_lora_raw_count_seen = raw_count_fast;
            const char *raw_frame_fast = LoRA_GetLastRawFrame();
            LoRA_ManualCommand_t fallback_manual_fast = LORA_MANUAL_CMD_NONE;
            uint8_t fallback_state_fast = 0;
            uint8_t fallback_is_state_fast = 0;
            if (ParseLoRaJsonCmdInMain(raw_frame_fast, &fallback_manual_fast, &fallback_state_fast, &fallback_is_state_fast)) {
              if (fallback_is_state_fast) {
                RobotSM_Request(&g_sm, (RobotState_t)fallback_state_fast);
                printf("[APP CMD RECEIVED] json_state=%u\r\n", fallback_state_fast);
                printf("[LoRa] JSON command received (main): %s -> state=%u\r\n",
                       (raw_frame_fast && raw_frame_fast[0] != '\0') ? raw_frame_fast : "<empty>",
                       fallback_state_fast);
              } else {
                HandleLoRaManualCommand(fallback_manual_fast);
              }
            }
          }
        }
      }

      if (RobotSM_Current(&g_sm) == STATE_MANUAL) {
        const uint32_t manual_now_ms = HAL_GetTick();
        const uint32_t manual_elapsed_ms = (s_last_manual_command_ms == 0u || manual_now_ms < s_last_manual_command_ms)
            ? 0u
            : (manual_now_ms - s_last_manual_command_ms);

        if (s_last_manual_command_ms != 0u &&
            manual_elapsed_ms > MANUAL_COMMAND_HOLD_TIMEOUT_MS) {
          if (!s_manual_watchdog_stopped) {
            s_manual_drive_filtered_pct = 0;
            s_manual_turn_filtered_pct = 0;
            Sabertooth_StopAll();
            s_manual_watchdog_stopped = 1u;
            printf("[LoRa] Manual command stale for %lums -> stop\r\n",
                   (unsigned long)manual_elapsed_ms);
          }
        }
      } else {
        s_last_manual_command_ms = 0u;
        s_manual_watchdog_stopped = 0u;
      }

      // Poll Sabertooth feedback at a bounded cadence to keep loop responsive.
      if ((now_ms - last_sabertooth_poll_ms) >= 50u) {
        last_sabertooth_poll_ms = now_ms;
        Sabertooth_PollFeedback();
      }

        if ((now_ms - last_50hz_ms) >= 20)
      {
          last_50hz_ms += 20;

          static IMU_Status_t imu_last;
          static uint32_t imu_consecutive_failures = 0;
          static uint32_t next_imu_sample_ms = 0;
          IMU_Status_t imu = imu_last;
          uint8_t imu_sample_attempted = 0;

          if (now_ms >= next_imu_sample_ms) {
            imu = IMU_Read();
            imu_last = imu;
            imu_sample_attempted = 1;
          }

          if (imu_sample_attempted && imu.ok)
          {
            imu_consecutive_failures = 0;
            imu_ok = 1;
		  latest_imu_temp_c = imu.temperature_c;
		  /* Use fixed 50 Hz timestep (20 ms) for consistency */
		  float dt = 0.02f;
		  imu_last_update_ms = now_ms;
            next_imu_sample_ms = now_ms + 20u;

		  HeadingFusion_Update(&g_hf, &imu, gps, dt);

          }
          else if (imu_sample_attempted)
          {
            imu_consecutive_failures++;
            imu_ok = ((imu_consecutive_failures < 10u) || ((now_ms - imu_last_update_ms) <= 2000u)) ? 1u : 0u;
            next_imu_sample_ms = now_ms + ((imu_consecutive_failures >= 5u) ? 200u : 20u);
          }
          else
          {
            imu_ok = ((imu_consecutive_failures < 10u) || ((now_ms - imu_last_update_ms) <= 2000u)) ? 1u : 0u;
          }

          latest_imu_temp_c = imu_last.temperature_c;

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

          // ===== Fast LoRa telemetry during manual drive (200 ms = 5 Hz) =====
            // Sends compact telemetry only when values changed or at a heartbeat.
            // This avoids saturating LoRa with repetitive MANUAL packets.
          if (lora_manual_feedback_enabled && RobotSM_Current(&g_sm) == STATE_MANUAL &&
              (now_ms - last_manual_lora_tx_ms) >= lora_manual_tx_interval_ms)
          {
              static uint8_t manual_telem_prev_valid = 0;
              static int manual_telem_prev_m1 = 0;
              static int manual_telem_prev_m2 = 0;
              static float manual_telem_prev_heading = 0.0f;
              static uint16_t manual_telem_prev_pl = PROX_NO_DETECTION;
              static uint16_t manual_telem_prev_pr = PROX_NO_DETECTION;

              int m1_now = Sabertooth_GetM1();
              int m2_now = Sabertooth_GetM2();
              float heading_now = g_hf.yaw_deg;
              uint16_t pl_now = Proximity_ReadLeft();
              uint16_t pr_now = Proximity_ReadRight();

              uint8_t changed = (uint8_t)(
                !manual_telem_prev_valid ||
                (m1_now != manual_telem_prev_m1) ||
                (m2_now != manual_telem_prev_m2) ||
                (heading_now > (manual_telem_prev_heading + lora_manual_heading_delta_min)) ||
                (heading_now < (manual_telem_prev_heading - lora_manual_heading_delta_min)) ||
                (pl_now != manual_telem_prev_pl) ||
                (pr_now != manual_telem_prev_pr));

              uint8_t heartbeat_due = (uint8_t)((now_ms - last_manual_lora_tx_ms) >= lora_manual_heartbeat_ms);
              if (changed || heartbeat_due) {
                last_manual_lora_tx_ms = now_ms;
                LoRA_SendManualTelemetry(m1_now, m2_now, heading_now, pl_now, pr_now);

                manual_telem_prev_valid = 1;
                manual_telem_prev_m1 = m1_now;
                manual_telem_prev_m2 = m2_now;
                manual_telem_prev_heading = heading_now;
                manual_telem_prev_pl = pl_now;
                manual_telem_prev_pr = pr_now;
              }
          }

            // ===== 1Hz Periodic Health Checks and Recovery =====
              if ((now_ms - last_1hz_ms) >= 1000) {
                static uint32_t next_imu_recovery_ms = 0;
                static uint32_t next_gps_recovery_ms = 0;
              last_1hz_ms += 1000;

                const RobotState_t health_state_now = RobotSM_Current(&g_sm);
                const uint8_t suppress_health_recovery = (health_state_now == STATE_MANUAL) ? 1u : 0u;

                if (!suppress_health_recovery) {
                  if (GPS_IsHealthy()) {
                    next_gps_recovery_ms = now_ms;
                  }

                  if (!GPS_IsHealthy() && now_ms >= next_gps_recovery_ms) {
                      printf("[GPS] WARNING: Receiver not responding - rx_bytes=%lu - attempting recovery\r\n",
                             (unsigned long)GPS_GetRxByteCount());
                      GPS_CheckAndRecover();
                    next_gps_recovery_ms = now_ms + 10000u;
                  }

                  if (imu_consecutive_failures >= 10u
                    && (now_ms - imu_last_update_ms) > 2000u
                    && now_ms >= next_imu_recovery_ms) {
                    printf("[IMU] WARNING: No successful samples for %lu ms - attempting recovery\r\n",
                           (unsigned long)(now_ms - imu_last_update_ms));
                    IMU_CheckAndRecover();
                    imu_ok = IMU_GetInitStatus();
                    if (imu_ok) {
                        imu_last_update_ms = now_ms;
                      next_imu_recovery_ms = now_ms + 2000u;
                    } else {
                      next_imu_recovery_ms = now_ms + 10000u;
                    }
                  }
                }
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

          // Keep non-manual telemetry flowing at LoRa cadence even when status logs are throttled.
          if (RobotSM_Current(&g_sm) != STATE_MANUAL &&
              (now_ms - last_lora_tx_ms) >= lora_tx_interval_ms) {
              last_lora_tx_ms = now_ms;

              uint16_t prox_left_cm = Proximity_ReadLeft();
              uint16_t prox_right_cm = Proximity_ReadRight();
              int m1_speed = Sabertooth_GetM1();
              int m2_speed = Sabertooth_GetM2();
              uint8_t salt_rate = Dispersion_GetSaltRate();
              uint8_t brine_rate = Dispersion_GetBrineRate();

              LoRA_SendTelemetry(RobotSM_Current(&g_sm),
                                 gps->latitude_deg, gps->longitude_deg, gps->has_fix,
                                 gps->num_satellites, gps->hdop,
                                 m1_speed, m2_speed,
                                 g_hf.yaw_deg, g_hf.pitch_deg,
                                 salt_rate, brine_rate, latest_imu_temp_c,
                                 prox_left_cm, prox_right_cm);

              const char *lora_tx_payload = LoRA_GetLastTxPayload();
              if (MAIN_VERBOSE_TELEMETRY_LOGS && lora_tx_payload && lora_tx_payload[0] != '\0') {
                printf(ANSI_YELLOW "[MAIN] LoRa TX: " ANSI_RESET "%s", lora_tx_payload);
              }
          }

            // ---- 1 Hz status/readout prints (SKIP during test mode) ----
            if ((now_ms - last_1hz_status_ms) >= status_print_interval_ms && !g_test_mode)
          {
              if (RobotSM_Current(&g_sm) == STATE_MANUAL) {
                  last_1hz_status_ms = now_ms;
                  continue;
              }
              uint32_t last_console_input_ms = Console_GetLastInputMs();
              if (last_console_input_ms > 0u && (now_ms - last_console_input_ms) < console_quiet_window_ms) {
                  last_1hz_status_ms = now_ms;
                  continue;
              }
              last_1hz_status_ms += status_print_interval_ms;
              
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
              
              if (MAIN_VERBOSE_TELEMETRY_LOGS) {
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
  huart4.Init.BaudRate = 230400;
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
  huart5.Init.BaudRate = 921600;
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





