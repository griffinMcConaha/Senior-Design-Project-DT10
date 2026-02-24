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

// Set dispersion rates with automatic 1:9 ratio enforcement
// salt_rate: 0-100 (percent output), brine_rate: auto-adjusted to maintain 1:9 ratio
// Sends command to dispersion ESP32 via UART 4: SALT:XX,BRINE:XX\r\n
void Dispersion_SetRate(uint8_t salt_rate, uint8_t brine_rate);

// Read salt flow sensor (mL/min)
uint16_t Dispersion_ReadSaltFlow(void);

// Read brine flow sensor (mL/min)
uint16_t Dispersion_ReadBrineFlow(void);

// Process incoming byte from UART 4 RX (call from ISR)
void Dispersion_RxByte(uint8_t byte);

// Dispersion control task - monitors flows, enforces ratios, processes UART responses
// Call periodically (e.g., every 50ms main loop iteration)
void Dispersion_Task(void);

// Get last status from dispersion ESP32
const char* Dispersion_GetLastStatus(void);

// Get current salt dispersion rate (0-100%)
uint8_t Dispersion_GetSaltRate(void);

// Get current brine dispersion rate (0-100%)
uint8_t Dispersion_GetBrineRate(void);

#ifdef __cplusplus
}
#endif

#endif // DISPERSION_H
