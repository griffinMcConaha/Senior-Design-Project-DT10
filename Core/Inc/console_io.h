/**
 * @file console_io.h
 * @brief Serial console I/O and printf redirect (USART2)
 */
#ifndef CONSOLE_IO_H
#define CONSOLE_IO_H

#include "stm32f4xx_hal.h"
#include "gps.h"
#include "imu_icm20948.h"
#include "heading_fusion.h"
#include "robot_sm.h"

/* ANSI terminal color codes */
#define ANSI_RESET   "\033[0m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_BLUE    "\033[34m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_WHITE   "\033[37m"

/** @brief UART handles for console operations */
typedef struct
{
    UART_HandleTypeDef *uart_printf; /**< Printf output (USART2) */
    UART_HandleTypeDef *uart_gps;    /**< GPS input (USART3) */
    UART_HandleTypeDef *uart_saber;  /**< Motor controller (USART1) */
} ConsoleIO_t;

/** @brief Initialize console I/O with UART handles */
void Console_Init(ConsoleIO_t *c,
                  UART_HandleTypeDef *uart_printf,
                  UART_HandleTypeDef *uart_gps,
                  UART_HandleTypeDef *uart_saber);

/** @brief Set test mode flag storage (set to 1 when entering test mode) */
void Console_SetTestModeFlag(volatile uint8_t *flag);

/** @brief Flush and reinit all UART interfaces */
void Console_FlushAll(ConsoleIO_t *c);

/** @brief Print sensor status (IMU, GPS, fusion) with color formatting */
void Console_PrintStatus(const ConsoleIO_t *c,
                         const IMU_Status_t *imu,
                         const GPS_Data_t *gps,
                         const HeadingFusion_t *hf);

/** @brief Handle single byte from USART2 (console input) */
void Console_RxByte(uint8_t byte, RobotSM_t *sm);

/** @brief Check if ESC key was pressed (for exiting continuous tests) */
uint8_t Console_CheckEscPressed(void);

/** @brief Process serial command and update robot state if valid */
void Console_ProcessCommand(const char *cmd, RobotSM_t *sm);

/** @brief Show quick test menu */
void Console_ShowTestMenu(void);

#endif // CONSOLE_IO_H
