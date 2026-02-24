#ifndef UART_LORA_H
#define UART_LORA_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// Initialize LoRA UART interface (UART5, 9600 baud)
void LoRA_Init(UART_HandleTypeDef *huart5);

// Process incoming byte from UART 5 RX (call from ISR)
void LoRA_RxByte(uint8_t byte);

// Periodic update (call from main loop)
void LoRA_Tick(uint32_t now_ms);

// Send state diagnostics to base station
// state: current robot state (0=MANUAL, 1=AUTO, 2=PAUSE, 3=ERROR, 4=ESTOP)
// gps_lat, gps_lon: current GPS position (or 0 if no fix)
// motor_m1, motor_m2: current motor speeds (-100 to +100)
void LoRA_SendState(uint8_t state, float gps_lat, float gps_lon,
                    int motor_m1, int motor_m2);

// Send comprehensive telemetry to base station
// state: current robot state
// gps_lat, gps_lon: GPS position
// gps_has_fix: 1 if GPS has valid fix
// gps_num_sat: number of satellites used
// gps_hdop: horizontal dilution of precision
// motor_m1, motor_m2: motor speeds (-100 to +100)
// yaw_deg: heading from IMU/GPS fusion (degrees)
// pitch_deg: pitch angle from IMU (degrees)
// salt_rate, brine_rate: dispersion rates (0-100%)
// temp_c: IMU temperature (Celsius)
// prox_left_cm, prox_right_cm: proximity sensor distances (cm)
void LoRA_SendTelemetry(uint8_t state, float gps_lat, float gps_lon, uint8_t gps_has_fix,
                        uint8_t gps_num_sat, float gps_hdop,
                        int motor_m1, int motor_m2, float yaw_deg, float pitch_deg,
                        uint8_t salt_rate, uint8_t brine_rate, float temp_c,
                        uint16_t prox_left_cm, uint16_t prox_right_cm);

// Send fault information to base station
// fault_code: fault code number (0-8)
// action: action taken (0=PAUSE, 1=ESTOP, 2=LOG_ONLY)
void LoRA_SendFault(uint8_t fault_code, uint8_t action);

// Parse and execute incoming command
// Returns: 1 if command valid, 0 if invalid
// Sets *out_state to requested state if valid command received
uint8_t LoRA_GetPendingCommand(uint8_t *out_state);

// Get last command for debugging
const char* LoRA_GetLastCommand(void);

#ifdef __cplusplus
}
#endif

#endif // UART_LORA_H
