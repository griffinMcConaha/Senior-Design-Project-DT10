#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t  has_fix;        // 1 if last RMC status == 'A'
    float    latitude_deg;   // decimal degrees (+N, -S)
    float    longitude_deg;  // decimal degrees (+E, -W)
    float    speed_knots;    // speed over ground
    float    course_deg;     // course over ground (0–360)
    uint8_t  num_satellites; // number of satellites used in fix
    float    hdop;           // horizontal dilution of precision (1.0=best, 10.0=worst)
    float    vdop;           // vertical dilution of precision
    float    altitude_m;     // altitude above mean sea level (meters)
} GPS_Data_t;

// Call once at boot
void GPS_Init(UART_HandleTypeDef *huart);

// Call from USART RX ISR when a byte arrives
void GPS_RxByte(uint8_t b);

// Call periodically (e.g., in main loop). Handles "freshness" timeout.
void GPS_Tick(uint32_t now_ms);

// Read-only access to latest data
const GPS_Data_t* GPS_Get(void);
// Get GPS fix quality (0.0=no fix, 1.0=excellent)
float GPS_GetQuality(void);
// If you want to know whether you’ve received data recently
uint32_t GPS_GetLastRxMs(void);

void GPS_HAL_RxCpltCallback(UART_HandleTypeDef *huart);

// Reset GPS parser state (for recovery)
void GPS_Reset(void);

// Check if GPS receiver is responsive (1=healthy, 0=stalled)
uint8_t GPS_IsHealthy(void);

// Total raw bytes received since GPS_Init (non-zero means UART RX is flowing)
uint32_t GPS_GetRxByteCount(void);

// Attempt recovery: reset parser and restart RX if stalled
uint8_t GPS_CheckAndRecover(void);

// Runtime control for auto-baud behavior.
// Set to 0 in direct manual mode to prevent unnecessary baud hopping.
void GPS_SetAutoBaudEnabled(uint8_t enabled);
uint8_t GPS_GetAutoBaudEnabled(void);

#ifdef __cplusplus
}
#endif
