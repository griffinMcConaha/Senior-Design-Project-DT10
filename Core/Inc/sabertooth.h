/**
 * @file sabertooth.h
 * @brief Sabertooth 2x32 motor controller UART driver
 * 
 * Control Protocol (plain text serial):
 * - Motor commands: "M1: <value>\r\n" and "M2: <value>\r\n"
 *   Value range: -2047..2047 (full reverse to full forward)
 * 
 * Feedback Queries (plain text serial):
 * - Battery voltage: "M1: getb\r\n"  -> Response: "M1:B240\r\n" (24.0V)
 * - Motor current:  "M1: getc\r\n"  -> Response: "M1:C320\r\n" (32.0A)
 * - Motor temp:     "M1: gett\r\n"  -> Response: "M1:T65\r\n" (65C)
 * 
 * Note: getb always queries motor 1; getc/gett query the specified motor
 */
#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Initialize Sabertooth on given UART (call with &huart1) */
void Sabertooth_Init(UART_HandleTypeDef *huart);

/** @brief Stop both motors (send 0 speed) */
void Sabertooth_StopAll(void);

/** @brief Set motor 1 speed in percent [-100..+100] (0=stop) */
void Sabertooth_SetM1(int percent);

/** @brief Set motor 2 speed in percent [-100..+100] (0=stop) */
void Sabertooth_SetM2(int percent);

/** @brief Send raw motor command [-2047..+2047] (plain text serial value) */
void Sabertooth_SetMotorRaw(uint8_t motor, int value);

/** @brief Send a raw plain text serial command (e.g., "M2: 500") */
void Sabertooth_SendRawCommand(const char *cmd);

/** @brief Get current motor 1 speed in percent [-100..+100] */
int Sabertooth_GetM1(void);

/** @brief Get current motor 2 speed in percent [-100..+100] */
int Sabertooth_GetM2(void);

/** @brief Query battery voltage (returns last known value in volts, -1.0 if unknown) */
float Sabertooth_GetBatteryVoltage(void);

/** @brief Query motor current (motor 1 or 2, returns last known value in amps, -1.0 if unknown) */
float Sabertooth_GetMotorCurrent(uint8_t motor);

/** @brief Query motor temperature (motor 1 or 2, returns last known value in °C, -999 if unknown) */
int Sabertooth_GetTemperature(uint8_t motor);

/** @brief Poll feedback from Sabertooth (call periodically ~1-10 Hz to update telemetry) */
void Sabertooth_PollFeedback(void);

/** @brief Manually request battery voltage feedback (async, response parsed on RX) */
void Sabertooth_QueryBattery(void);

/** @brief Manually request motor current feedback (async, response parsed on RX) */
void Sabertooth_QueryCurrent(uint8_t motor);

/** @brief Manually request motor temperature feedback (async, response parsed on RX) */
void Sabertooth_QueryTemperature(uint8_t motor);

/** @brief Handle UART RX completion callback (call from HAL_UART_RxCpltCallback) */
void Sabertooth_RxCallback(void);

/** @brief Process a single received UART byte (call from HAL_UART_RxCpltCallback) */
void Sabertooth_ProcessRxByte(uint8_t byte);

// ============================================================================
// MIXED MODE (Tank Drive) Control - MD and MT commands
// ============================================================================

/** @brief Set mixed drive (forward/backward) for both motors in tank mode
 *  @param drive Speed in percent [-100..+100] where 0=stop, +100=full forward, -100=full reverse
 *  Must also set turn command for mixed mode to work
 */
void Sabertooth_SetMixedDrive(int drive);

/** @brief Set mixed turn (left/right) for tank drive
 *  @param turn Speed in percent [-100..+100] where 0=straight, +100=full right, -100=full left
 *  Must also set drive command for mixed mode to work
 */
void Sabertooth_SetMixedTurn(int turn);

// ============================================================================
// RAMP RATE Control - R1 and R2 commands
// ============================================================================

/** @brief Set ramp rate for motor 1 (acceleration smoothing)
 *  @param rate Ramp value [-2047..+2047] where 2047 = ~8 seconds from stop to full speed
 *  Smaller values = faster acceleration, larger values = slower/smoother acceleration
 */
void Sabertooth_SetRampM1(int rate);

/** @brief Set ramp rate for motor 2 (acceleration smoothing)
 *  @param rate Ramp value [-2047..+2047] where 2047 = ~8 seconds from stop to full speed
 */
void Sabertooth_SetRampM2(int rate);

#ifdef __cplusplus
}
#endif