#include "uart_lora.h"
#include "robot_sm.h"
#include "system_health.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// ============================================================================
// LoRA UART COMMUNICATION MODULE (UART 5)
// Receives commands from mobile app via LoRA ESP32
// Sends diagnostic data back to base station
// ============================================================================

#define LORA_RX_BUFFER_SIZE 128
#define LORA_COMMAND_TIMEOUT_MS 5000

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];
    uint16_t rx_index;
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;  // Track last successful transmission for health monitoring
    char last_command[64];
    uint8_t pending_state_request; // 0=none, else state value
    uint8_t command_valid;
} LoRA_State_t;

static LoRA_State_t lora_state = {0};

// Initialize LoRA UART interface (UART5, 9600 baud)
void LoRA_Init(UART_HandleTypeDef *huart5)
{
    if (!huart5) return;

    lora_state.huart = huart5;
    lora_state.rx_index = 0;
    lora_state.last_rx_ms = 0;
    lora_state.last_tx_ms = 0;
    lora_state.pending_state_request = 0;
    memset(lora_state.rx_buffer, 0, LORA_RX_BUFFER_SIZE);
    memset(lora_state.last_command, 0, sizeof(lora_state.last_command));

    printf("[LORA] LoRA module initialized on UART5 (9600 baud)\r\n");
}

// Process incoming byte from UART 5 RX (call from ISR)
void LoRA_RxByte(uint8_t byte)
{
    if (!lora_state.huart) return;

    // Check for line ending (CR or LF)
    if (byte == '\r' || byte == '\n')
    {
        // Process complete message
        if (lora_state.rx_index > 0)
        {
            lora_state.rx_buffer[lora_state.rx_index] = '\0';
            lora_state.last_rx_ms = HAL_GetTick();

            // Copy to last_command for debugging
            strncpy(lora_state.last_command, (const char *)lora_state.rx_buffer,
                    sizeof(lora_state.last_command) - 1);
            lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';

            printf("[LORA] Received: %s\r\n", lora_state.rx_buffer);

            // Parse command
            lora_state.command_valid = 0;
            const char *cmd = (const char *)lora_state.rx_buffer;

            // CMD:AUTO[,SALT:XX,BRINE:XX]
            if (strncmp(cmd, "CMD:AUTO", 8) == 0)
            {
                lora_state.pending_state_request = 1; // STATE_AUTO
                lora_state.command_valid = 1;
                printf("[LORA] Valid command: AUTO\r\n");
            }
            // CMD:MANUAL
            else if (strcmp(cmd, "CMD:MANUAL") == 0)
            {
                lora_state.pending_state_request = 0; // STATE_MANUAL
                lora_state.command_valid = 1;
                printf("[LORA] Valid command: MANUAL\r\n");
            }
            // CMD:PAUSE
            else if (strcmp(cmd, "CMD:PAUSE") == 0)
            {
                lora_state.pending_state_request = 2; // STATE_PAUSE
                lora_state.command_valid = 1;
                printf("[LORA] Valid command: PAUSE\r\n");
            }
            // CMD:ESTOP
            else if (strcmp(cmd, "CMD:ESTOP") == 0)
            {
                lora_state.pending_state_request = 4; // STATE_ESTOP
                lora_state.command_valid = 1;
                printf("[LORA] Valid command: ESTOP\r\n");
            }
            else
            {
                printf("[LORA] Invalid command: %s\r\n", cmd);
            }

            // Reset buffer
            lora_state.rx_index = 0;
        }
        return;
    }

    // Add byte to buffer
    if (lora_state.rx_index < (LORA_RX_BUFFER_SIZE - 1))
    {
        lora_state.rx_buffer[lora_state.rx_index++] = byte;
    }
    else
    {
        // Buffer overflow - reset
        printf("[LORA] RX buffer overflow, resetting\r\n");
        lora_state.rx_index = 0;
    }
}

// Periodic update (call from main loop)
void LoRA_Tick(uint32_t now_ms)
{
    // Optional: Check for timeout on incomplete messages
    if (lora_state.rx_index > 0)
    {
        if ((now_ms - lora_state.last_rx_ms) > LORA_COMMAND_TIMEOUT_MS)
        {
            printf("[LORA] RX timeout, discarding incomplete message\r\n");
            lora_state.rx_index = 0;
        }
    }

    // Report health status to system health monitor
    // Consider LoRA healthy if we've transmitted recently (heartbeat every ~5 seconds from main loop)
    if (lora_state.last_tx_ms == 0) {
        // Not initialized yet
        SystemHealth_SetSensorStatus(SENSOR_LORA, SENSOR_TIMEOUT);
    } else if ((now_ms - lora_state.last_tx_ms) > 10000) {
        // No transmission for >10 seconds = timeout
        SystemHealth_SetSensorStatus(SENSOR_LORA, SENSOR_TIMEOUT);
    } else {
        // Regular transmissions happening
        SystemHealth_SetSensorStatus(SENSOR_LORA, SENSOR_OK);
    }
}

// Send state diagnostics to base station (JSON format)
void LoRA_SendState(uint8_t state, float gps_lat, float gps_lon,
                    int motor_m1, int motor_m2)
{
    if (!lora_state.huart) return;

    char state_name[16] = "UNKNOWN";
    switch (state)
    {
        case 0: strcpy(state_name, "MANUAL"); break;
        case 1: strcpy(state_name, "AUTO"); break;
        case 2: strcpy(state_name, "PAUSE"); break;
        case 3: strcpy(state_name, "ERROR"); break;
        case 4: strcpy(state_name, "ESTOP"); break;
    }

    // JSON format: {"state":"MODE","gps":{"lat":0.0,"lon":0.0},"motor":{"m1":0,"m2":0}}
    char msg[160];
    snprintf(msg, sizeof(msg),
             "{\"state\":\"%s\",\"gps\":{\"lat\":%.4f,\"lon\":%.4f},\"motor\":{\"m1\":%d,\"m2\":%d}}\r\n",
             state_name, gps_lat, gps_lon, motor_m1, motor_m2);

    HAL_UART_Transmit(lora_state.huart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    lora_state.last_tx_ms = HAL_GetTick();
    printf("[LORA] Sent: %s", msg);
}

// Send comprehensive telemetry to base station (JSON format)
void LoRA_SendTelemetry(uint8_t state, float gps_lat, float gps_lon, uint8_t gps_has_fix,
                        uint8_t gps_num_sat, float gps_hdop,
                        int motor_m1, int motor_m2, float yaw_deg, float pitch_deg,
                        uint8_t salt_rate, uint8_t brine_rate, float temp_c,
                        uint16_t prox_left_cm, uint16_t prox_right_cm)
{
    if (!lora_state.huart) return;

    char state_name[16] = "UNKNOWN";
    switch (state)
    {
        case 0: strcpy(state_name, "MANUAL"); break;
        case 1: strcpy(state_name, "AUTO"); break;
        case 2: strcpy(state_name, "PAUSE"); break;
        case 3: strcpy(state_name, "ERROR"); break;
        case 4: strcpy(state_name, "ESTOP"); break;
    }

    // JSON format with proximity sensors
    char msg[320];
    snprintf(msg, sizeof(msg),
             "{\"state\":\"%s\",\"gps\":{\"lat\":%.4f,\"lon\":%.4f,\"fix\":%u,\"sat\":%u,\"hdop\":%.1f},\"motor\":{\"m1\":%d,\"m2\":%d},\"heading\":{\"yaw\":%.1f,\"pitch\":%.1f},\"disp\":{\"salt\":%u,\"brine\":%u},\"temp\":%.1f,\"prox\":{\"left\":%u,\"right\":%u}}\r\n",
             state_name, gps_lat, gps_lon, gps_has_fix, gps_num_sat, gps_hdop,
             motor_m1, motor_m2, yaw_deg, pitch_deg, salt_rate, brine_rate, temp_c,
             prox_left_cm, prox_right_cm);

    HAL_UART_Transmit(lora_state.huart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    lora_state.last_tx_ms = HAL_GetTick();
    printf("[LORA] Sent telemetry: %s", msg);
}

// Send fault information to base station (JSON format)
void LoRA_SendFault(uint8_t fault_code, uint8_t action)
{
    if (!lora_state.huart) return;

    const char *fault_name = "UNKNOWN";
    switch (fault_code)
    {
        case 0: fault_name = "NONE"; break;
        case 1: fault_name = "IMU_TIMEOUT"; break;
        case 2: fault_name = "GPS_LOSS"; break;
        case 3: fault_name = "PROXIMITY_WARN"; break;
        case 4: fault_name = "PROXIMITY_CRIT"; break;
        case 5: fault_name = "BATTERY_COLD"; break;
        case 6: fault_name = "DISPERSION_CLOG"; break;
        case 7: fault_name = "GENERIC"; break;
        case 8: fault_name = "RESERVED"; break;
    }

    const char *action_name = "UNKNOWN";
    switch (action)
    {
        case 0: action_name = "PAUSE"; break;
        case 1: action_name = "ESTOP"; break;
        case 2: action_name = "LOG_ONLY"; break;
    }

    // JSON format: {"fault":"IMU_TIMEOUT","action":"ESTOP"}
    char msg[96];
    snprintf(msg, sizeof(msg), "{\"fault\":\"%s\",\"action\":\"%s\"}\r\n", fault_name, action_name);

    HAL_UART_Transmit(lora_state.huart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    lora_state.last_tx_ms = HAL_GetTick();
    printf("[LORA] Sent: %s", msg);
}

// Parse and execute incoming command
uint8_t LoRA_GetPendingCommand(uint8_t *out_state)
{
    if (!out_state) return 0;

    if (lora_state.command_valid)
    {
        *out_state = lora_state.pending_state_request;
        lora_state.command_valid = 0;
        return 1;
    }

    return 0;
}

// Get last command for debugging
const char* LoRA_GetLastCommand(void)
{
    return lora_state.last_command;
}
