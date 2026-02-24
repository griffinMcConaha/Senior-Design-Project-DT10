#include "sabertooth.h"
#include "system_health.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Module-level UART handle for motor driver communication
static UART_HandleTypeDef *s_huart = NULL;

// Store current motor speeds for readback
static int s_m1_speed = 0;
static int s_m2_speed = 0;

// Track last successful command for health monitoring
static uint32_t s_last_cmd_ms = 0;

// Sabertooth feedback values (Plain Text Serial)
static float s_battery_voltage = -1.0f;  // Volts, -1.0 = unknown
static float s_m1_current = -1.0f;       // Amps, -1.0 = unknown
static float s_m2_current = -1.0f;       // Amps, -1.0 = unknown
static int s_m1_temperature = -999;      // Celsius, -999 = unknown
static int s_m2_temperature = -999;      // Celsius, -999 = unknown

// RX buffer for Plain Text Serial responses
#define SABERTOOTH_RX_BUFFER_SIZE 32
static uint8_t s_rx_buffer[SABERTOOTH_RX_BUFFER_SIZE];
static uint8_t s_rx_index = 0;
static uint8_t s_rx_complete = 0;

// Query tracking (which feedback to request next)
static uint8_t s_query_state = 0;  // 0=voltage, 1=M1 current, 2=M2 current, 3=M1 temp, 4=M2 temp

// Sabertooth plain text serial protocol: value range and percent limits
#define SABERTOOTH_CMD_MAX     2047  // Full-scale command value
#define SABERTOOTH_PERCENT_MAX 100   // Percent range limit

// Store UART handle for transmitting motor commands
void Sabertooth_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_index = 0;
    s_rx_complete = 0;
    printf("[SABERTOOTH] Initialized on UART %p\r\n", (void *)huart);
    
    // Start RX interrupt to listen for Sabertooth feedback
    if (huart != NULL) {
        HAL_UART_Receive_IT(huart, &s_rx_buffer[0], 1);
        printf("[SABERTOOTH] RX interrupt enabled\r\n");
    }
}

// Send Plain Text Serial command
static void sabertooth_send_command(const char *cmd)
{
    if (s_huart == NULL) return;
    uint16_t len = strlen(cmd);
    HAL_UART_Transmit(s_huart, (uint8_t *)cmd, len, HAL_MAX_DELAY);
    s_last_cmd_ms = HAL_GetTick();  // Track successful transmission
    HAL_Delay(5);  // Small delay between commands
}

// Send a raw plain text serial command, appending CRLF if missing
void Sabertooth_SendRawCommand(const char *cmd)
{
    if (cmd == NULL || *cmd == '\0') {
        printf("[MOTOR] Empty command ignored\r\n");
        return;
    }

    size_t len = strlen(cmd);
    char buffer[64];

    // If command already ends with CR or LF, send as-is
    if (cmd[len - 1] == '\n' || cmd[len - 1] == '\r') {
        sabertooth_send_command(cmd);
        return;
    }

    // Append CRLF
    if (len + 2 >= sizeof(buffer)) {
        printf("[MOTOR] Command too long\r\n");
        return;
    }

    snprintf(buffer, sizeof(buffer), "%s\r\n", cmd);
    sabertooth_send_command(buffer);
}

// Parse Plain Text Serial response (e.g., "M1:B240" → battery=24.0V)
static void sabertooth_parse_response(void)
{
    if (s_rx_index < 5) return;  // Minimum: "M1:B0\r"
    
    // Format: "M1:B240\r\n" or "M1:C320\r\n" or "M1:T30\r\n"
    // Extract the type (B=battery, C=current, T=temperature)
    char response_type = s_rx_buffer[3];  // Position of type letter
    
    // Extract the numeric value
    uint8_t value_start = 4;
    uint8_t value_len = 0;
    for (uint8_t i = 4; i < s_rx_index; i++) {
        if (s_rx_buffer[i] == '\r' || s_rx_buffer[i] == '\n') break;
        value_len++;
    }
    
    char value_str[16] = {0};
    strncpy(value_str, (char *)&s_rx_buffer[value_start], value_len);
    int value = atoi(value_str);
    
    // Determine which motor is responding (M1 or M2)
    uint8_t motor = (s_rx_buffer[1] == '1') ? 1 : 2;
    
    // Store the parsed value
    switch (response_type) {
        case 'B':  // Battery voltage (in tenths of volt)
            s_battery_voltage = value / 10.0f;
            printf("[FEEDBACK] Battery: %.1f V\r\n", s_battery_voltage);
            break;
        case 'C':  // Motor current (in tenths of amp)
            if (motor == 1) {
                s_m1_current = value / 10.0f;
                printf("[FEEDBACK] M1 Current: %.1f A\r\n", s_m1_current);
            } else {
                s_m2_current = value / 10.0f;
                printf("[FEEDBACK] M2 Current: %.1f A\r\n", s_m2_current);
            }
            break;
        case 'T':  // Temperature (in Celsius)
            if (motor == 1) {
                s_m1_temperature = value;
                printf("[FEEDBACK] M1 Temp: %d C\r\n", s_m1_temperature);
            } else {
                s_m2_temperature = value;
                printf("[FEEDBACK] M2 Temp: %d C\r\n", s_m2_temperature);
            }
            break;
    }
}

// Stop both motors by sending plain text commands
void Sabertooth_StopAll(void)
{
    // Send stop commands to both motors (speed = 0)
    sabertooth_send_command("M1: 0\r\n");
    sabertooth_send_command("M2: 0\r\n");
    HAL_Delay(5);
}

// Set M1 motor speed in percent: -100 (reverse) to +100 (forward)
// Plain Text Serial: "M1: value\r\n" where value is -2047..2047
void Sabertooth_SetM1(int percent)
{
    // Clamp speed to valid range
    if (percent > SABERTOOTH_PERCENT_MAX) percent = SABERTOOTH_PERCENT_MAX;
    if (percent < -SABERTOOTH_PERCENT_MAX) percent = -SABERTOOTH_PERCENT_MAX;

    // Store current speed for readback
    s_m1_speed = percent;

    // Scale -100..+100 to -2047..+2047 for Plain Text Serial
    int scaled_speed = (percent * SABERTOOTH_CMD_MAX) / SABERTOOTH_PERCENT_MAX;
    
    // Send Plain Text Serial command
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "M1: %d\r\n", scaled_speed);
    printf("[M1] Speed: %d%% (scaled: %d)\r\n", percent, scaled_speed);
    sabertooth_send_command(cmd);
}

// Set M2 motor speed in percent: -100 (reverse) to +100 (forward)
// Plain Text Serial: "M2: value\r\n" where value is -2047..2047
void Sabertooth_SetM2(int percent)
{
    // Clamp speed to valid range
    if (percent > SABERTOOTH_PERCENT_MAX) percent = SABERTOOTH_PERCENT_MAX;
    if (percent < -SABERTOOTH_PERCENT_MAX) percent = -SABERTOOTH_PERCENT_MAX;

    // Store current speed for readback
    s_m2_speed = percent;

    // Scale -100..+100 to -2047..+2047 for Plain Text Serial
    int scaled_speed = (percent * SABERTOOTH_CMD_MAX) / SABERTOOTH_PERCENT_MAX;
    
    // Send Plain Text Serial command
    char cmd[16];
    snprintf(cmd, sizeof(cmd), "M2: %d\r\n", scaled_speed);
    printf("[M2] Speed: %d%% (scaled: %d)\r\n", percent, scaled_speed);
    sabertooth_send_command(cmd);
}

// Send raw motor command value [-2047..+2047] directly (plain text serial)
void Sabertooth_SetMotorRaw(uint8_t motor, int value)
{
    if (motor < 1 || motor > 2) {
        printf("[MOTOR] Invalid motor %u (expected 1 or 2)\r\n", motor);
        return;
    }

    if (value > SABERTOOTH_CMD_MAX) value = SABERTOOTH_CMD_MAX;
    if (value < -SABERTOOTH_CMD_MAX) value = -SABERTOOTH_CMD_MAX;

    char cmd[20];
    snprintf(cmd, sizeof(cmd), "M%u: %d\r\n", motor, value);
    sabertooth_send_command(cmd);
}

// Get current motor 1 speed in percent [-100..+100]
int Sabertooth_GetM1(void)
{
    return s_m1_speed;
}

// Get current motor 2 speed in percent [-100..+100]
int Sabertooth_GetM2(void)
{
    return s_m2_speed;
}

// Get battery voltage (in volts)
float Sabertooth_GetBatteryVoltage(void)
{
    return s_battery_voltage;
}

// Get motor current (in amps)
float Sabertooth_GetMotorCurrent(uint8_t motor)
{
    if (motor == 1) return s_m1_current;
    if (motor == 2) return s_m2_current;
    return -1.0f;
}

// Get motor temperature (in Celsius)
int Sabertooth_GetTemperature(uint8_t motor)
{
    if (motor == 1) return s_m1_temperature;
    if (motor == 2) return s_m2_temperature;
    return -999;
}

// Handle UART RX completion callback
void Sabertooth_RxCallback(void)
{
    // Response received, parse it
    if (s_rx_complete) {
        sabertooth_parse_response();
        s_rx_complete = 0;
        s_rx_index = 0;
    }
}

// Poll feedback from Sabertooth (call periodically ~1-10 Hz)
void Sabertooth_PollFeedback(void)
{
    if (s_huart == NULL) return;
    
    // Setup RX for next response if ready
    if (!s_rx_complete && s_rx_index == 0) {
        HAL_UART_Receive_IT(s_huart, &s_rx_buffer[s_rx_index], 1);
    }
    
    // Rotate through different queries
    static uint32_t last_query_ms = 0;
    uint32_t now_ms = HAL_GetTick();
    
    // Report health status to system health monitor
    // Consider Sabertooth healthy if we've received feedback or transmitted recently
    if (s_last_cmd_ms == 0 || ((now_ms - s_last_cmd_ms) > 5000)) {
        // No command activity for >5 seconds = timeout
        SystemHealth_SetSensorStatus(SENSOR_SABERTOOTH, SENSOR_TIMEOUT);
    } else {
        // Recent command activity detected
        SystemHealth_SetSensorStatus(SENSOR_SABERTOOTH, SENSOR_OK);
    }
    
    if ((now_ms - last_query_ms) >= 100) {  // Query every 100ms
        last_query_ms = now_ms;
        
        switch (s_query_state) {
            case 0:
                Sabertooth_QueryBattery();
                break;
            case 1:
                Sabertooth_QueryCurrent(1);
                break;
            case 2:
                Sabertooth_QueryCurrent(2);
                break;
            case 3:
                Sabertooth_QueryTemperature(1);
                break;
            case 4:
                Sabertooth_QueryTemperature(2);
                break;
        }
        
        s_query_state = (s_query_state + 1) % 5;  // Cycle through 5 queries
    }
}

// Manually request battery voltage feedback
void Sabertooth_QueryBattery(void)
{
    sabertooth_send_command("M1: getb\r\n");
}

// Manually request motor current feedback
void Sabertooth_QueryCurrent(uint8_t motor)
{
    if (motor != 1 && motor != 2) return;
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "M%u: getc\r\n", motor);
    sabertooth_send_command(cmd);
}

// Manually request motor temperature feedback
void Sabertooth_QueryTemperature(uint8_t motor)
{
    if (motor != 1 && motor != 2) return;
    char cmd[20];
    snprintf(cmd, sizeof(cmd), "M%u: gett\r\n", motor);
    sabertooth_send_command(cmd);
}

// Interrupt handler for UART RX (called by HAL_UART_RxCpltCallback)
void Sabertooth_ProcessRxByte(uint8_t byte)
{
    // Build response string
    if (s_rx_index < SABERTOOTH_RX_BUFFER_SIZE) {
        s_rx_buffer[s_rx_index++] = byte;
    }
    
    // Check for end of line (\r\n)
    if (s_rx_index >= 2 && s_rx_buffer[s_rx_index - 2] == '\r' && s_rx_buffer[s_rx_index - 1] == '\n') {
        s_rx_complete = 1;
        sabertooth_parse_response();
        s_rx_index = 0;
    }
    
    // Timeout: reset if buffer gets too full
    if (s_rx_index >= SABERTOOTH_RX_BUFFER_SIZE - 1) {
        s_rx_index = 0;
    }

    // Rearm RX for next byte
    if (s_huart != NULL) {
        HAL_UART_Receive_IT(s_huart, &s_rx_buffer[s_rx_index], 1);
    }
}

