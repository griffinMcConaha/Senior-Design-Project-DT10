#ifndef DISPERSION_H
#define DISPERSION_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize dispersion system with PWM outputs for salt auger and brine pump
// Also initializes UART 4 communication with dispersion ESP32
void Dispersion_Init(UART_HandleTypeDef *huart4);

// Set dispersion rates as direct pass-through percentages
// salt_rate: 0-100 (percent output), brine_rate: 0-100 (percent output)
// Sends command to dispersion ESP32 via UART 4: SALT:XX,BRINE:XX\r\n
void Dispersion_SetRate(uint8_t salt_rate, uint8_t brine_rate);

// Set dispersion rates directly without 1:9 ratio enforcement (diagnostic/test use)
// Sends command to dispersion ESP32 via UART 4: SALT:XX,BRINE:XX\r\n
void Dispersion_SetRateDirect(uint8_t salt_rate, uint8_t brine_rate);

// Send percent-only command for ESP32-side test handling (payload: "<percent>\r\n")
void Dispersion_SendPercentOnly(uint8_t percent);

// Send raw command string to dispersion ESP32 (appends CRLF if missing)
void Dispersion_SendRaw(const char *text);

// Read salt flow sensor (mL/min)
uint16_t Dispersion_ReadSaltFlow(void);

// Read brine flow sensor (mL/min)
uint16_t Dispersion_ReadBrineFlow(void);

// Read latest RPM feedback from dispersion ESP32
float Dispersion_ReadRPM(void);

// Process incoming byte from UART 4 RX (call from ISR)
void Dispersion_RxByte(uint8_t byte);

// Dispersion task - monitors flows and processes UART responses (no automatic rate control)
// Call periodically (e.g., every 50ms main loop iteration)
void Dispersion_Task(void);

// Get last status from dispersion ESP32
const char* Dispersion_GetLastStatus(void);

// Link heartbeat metrics for console status
uint32_t Dispersion_GetLastTxMs(void);
uint32_t Dispersion_GetLastRxMs(void);
uint32_t Dispersion_GetTxCount(void);
uint32_t Dispersion_GetRxCount(void);

// Enable/disable verbose ESP response prints for salt/brine test modes
void Dispersion_SetTestResponseMode(uint8_t enable);

// Get current salt dispersion rate (0-100%)
uint8_t Dispersion_GetSaltRate(void);

// Get current brine dispersion rate (0-100%)
uint8_t Dispersion_GetBrineRate(void);

#ifdef __cplusplus
}
#endif

#endif // DISPERSION_H
