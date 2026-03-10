#include "dispersion.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// ============================================================================
// SALT AND BRINE DISPERSION CONTROL (PHASE 4)
// UART 4 communication with dispersion ESP32 (9600 baud)
// ============================================================================

#define DISP_RX_BUFFER_SIZE 128

typedef struct {
    UART_HandleTypeDef *huart;  // UART 4 handle
    uint8_t initialized;        // Module initialized
    uint8_t salt_rate_percent;  // Salt auger speed 0-100%
    uint8_t brine_rate_percent; // Brine pump speed 0-100%
    uint16_t salt_flow_mlmin;   // Last salt flow reading (mL/min)
    uint16_t brine_flow_mlmin;  // Last brine flow reading (mL/min)
    float rpm_feedback;         // Last RPM feedback from ESP32
    uint8_t rx_buffer[DISP_RX_BUFFER_SIZE];
    uint16_t rx_index;
    char last_status[64];
    uint32_t clog_timeout_ticks;// Ticks until clog detection timeout
    uint8_t verbose_test_responses; // 1=print ESP responses for test salt/brine modes
    uint32_t last_tx_ms;
    uint32_t last_rx_ms;
    uint32_t tx_count;
    uint32_t rx_count;
} Dispersion_State_t;

static Dispersion_State_t disp_state = {0};

// Initialize dispersion system
// - PWM outputs: salt auger (motor 3) and brine pump (motor 4)
// - UART4 link to dispersion ESP32 for telemetry and control
void Dispersion_Init(UART_HandleTypeDef *huart4)
{
    // TODO: Configure PWM outputs for salt auger and brine pump
    // TODO: Initialize ADC/GPIO for flow sensor inputs (salt on ADC, brine on ADC)
    
    disp_state.huart = huart4;
    disp_state.initialized = 1;
    disp_state.salt_rate_percent = 0;
    disp_state.brine_rate_percent = 0;
    disp_state.salt_flow_mlmin = 0;
    disp_state.brine_flow_mlmin = 0;
    disp_state.rpm_feedback = 0.0f;
    disp_state.rx_index = 0;
    memset(disp_state.rx_buffer, 0, DISP_RX_BUFFER_SIZE);
    memset(disp_state.last_status, 0, sizeof(disp_state.last_status));
    disp_state.verbose_test_responses = 0;
    disp_state.last_tx_ms = 0;
    disp_state.last_rx_ms = 0;
    disp_state.tx_count = 0;
    disp_state.rx_count = 0;
    
    printf("[DISP] Dispersion system initialized (UART 4 at 9600 baud)\r\n");
}

// Set dispersion rates as direct pass-through percentages
// salt_rate: 0-100 (0-100% output)
// brine_rate: 0-100 (0-100% output)
void Dispersion_SetRate(uint8_t salt_rate, uint8_t brine_rate)
{
    if (!disp_state.initialized)
        return;

    // Clamp input ranges
    if (salt_rate > 100) salt_rate = 100;
    if (brine_rate > 100) brine_rate = 100;

    // Store rates
    disp_state.salt_rate_percent = salt_rate;
    disp_state.brine_rate_percent = brine_rate;

    // Send command to dispersion ESP32 via UART 4
    // Format: SALT:XX,BRINE:XX\r\n
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "SALT:%d,BRINE:%d\r\n", salt_rate, brine_rate);
    
    if (disp_state.huart)
    {
        HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
        disp_state.last_tx_ms = HAL_GetTick();
        disp_state.tx_count++;
    }

    // TODO: Send PWM commands to salt auger and brine pump
    // PWM format: scale percent to motor speed range (0-255 or similar)
    // salt_pwm = (salt_rate * 255) / 100;
    // brine_pwm = (brine_rate * 255) / 100;

    // Log change
    printf("[DISP] Dispersion rates: salt=%d%%, brine=%d%%\r\n",
           disp_state.salt_rate_percent, disp_state.brine_rate_percent);
}

// Set dispersion rates directly without ratio enforcement (test/diagnostic use)
void Dispersion_SetRateDirect(uint8_t salt_rate, uint8_t brine_rate)
{
    if (!disp_state.initialized)
        return;

    if (salt_rate > 100) salt_rate = 100;
    if (brine_rate > 100) brine_rate = 100;

    disp_state.salt_rate_percent = salt_rate;
    disp_state.brine_rate_percent = brine_rate;

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "SALT:%d,BRINE:%d\r\n", salt_rate, brine_rate);

    if (disp_state.huart)
    {
        HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
        disp_state.last_tx_ms = HAL_GetTick();
        disp_state.tx_count++;
    }

    printf("[DISP] Direct test rates: salt=%d%%, brine=%d%%\r\n",
           disp_state.salt_rate_percent, disp_state.brine_rate_percent);
}

void Dispersion_SendPercentOnly(uint8_t percent)
{
    if (!disp_state.initialized)
        return;

    if (percent > 100) percent = 100;

    if (disp_state.huart)
    {
        char cmd[8];
        int len = snprintf(cmd, sizeof(cmd), "%u\r\n", percent);
        HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, (uint16_t)len, HAL_MAX_DELAY);
        disp_state.last_tx_ms = HAL_GetTick();
        disp_state.tx_count++;
    }

    printf("[DISP] Percent-only test send: %u%%\r\n", percent);
}

void Dispersion_SendRaw(const char *text)
{
    if (!disp_state.initialized || !text || text[0] == '\0')
        return;

    if (disp_state.huart)
    {
        char cmd[192];
        size_t in_len = strlen(text);

        if (in_len >= sizeof(cmd) - 3) {
            in_len = sizeof(cmd) - 3;
        }

        memcpy(cmd, text, in_len);
        cmd[in_len] = '\0';

        if (in_len == 0 || cmd[in_len - 1] != '\n') {
            cmd[in_len++] = '\r';
            cmd[in_len++] = '\n';
            cmd[in_len] = '\0';
        }

        HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, (uint16_t)in_len, HAL_MAX_DELAY);
        disp_state.last_tx_ms = HAL_GetTick();
        disp_state.tx_count++;
        printf("[DISP] Raw TX: %s", cmd);
    }
}

// Read salt auger flow from sensor (mL/min)
// Returns: Flow rate in mL/min, 0 if sensor error
uint16_t Dispersion_ReadSaltFlow(void)
{
    // TODO: Read ADC from salt flow sensor
    // Convert ADC to mL/min (depends on sensor specs)
    // Typical: 0V=0mL/min, 5V=max_flow
    
    // Placeholder: Return last cached value
    return disp_state.salt_flow_mlmin;
}

// Read brine pump flow from sensor (mL/min)
// Returns: Flow rate in mL/min, 0 if sensor error
uint16_t Dispersion_ReadBrineFlow(void)
{
    // TODO: Read ADC from brine flow sensor
    // Convert ADC to mL/min (depends on sensor specs)
    
    // Placeholder: Return last cached value
    return disp_state.brine_flow_mlmin;
}

float Dispersion_ReadRPM(void)
{
    return disp_state.rpm_feedback;
}

// Process incoming byte from UART 4 RX (call from ISR)
void Dispersion_RxByte(uint8_t byte)
{
    if (!disp_state.initialized) return;

    // Check for line ending (CR or LF)
    if (byte == '\r' || byte == '\n')
    {
        // Process complete message
        if (disp_state.rx_index > 0)
        {
            disp_state.rx_buffer[disp_state.rx_index] = '\0';

            // Copy to last_status for debugging
            strncpy(disp_state.last_status, (const char *)disp_state.rx_buffer,
                    sizeof(disp_state.last_status) - 1);
            disp_state.last_status[sizeof(disp_state.last_status) - 1] = '\0';
            disp_state.last_rx_ms = HAL_GetTick();
            disp_state.rx_count++;

            if (disp_state.verbose_test_responses) {
                printf("[DISP] ESP32 Response: %s\r\n", disp_state.rx_buffer);
            }

            // Parse response
            const char *resp = (const char *)disp_state.rx_buffer;

            // STATUS:OK
            if (strcmp(resp, "STATUS:OK") == 0)
            {
                if (disp_state.verbose_test_responses) {
                    printf("[DISP] Dispersion OK\r\n");
                }
            }
            // STATUS:ERROR,CLOG
            else if (strstr(resp, "STATUS:ERROR") != NULL)
            {
                if (disp_state.verbose_test_responses) {
                    printf("[DISP] ERROR from ESP32: %s\r\n", resp);
                }
                // TODO: Set fault code FAULT_DISPERSION_CLOG
            }
            // FLOW:SALT:XXX,BRINE:XXXX
            else if (strstr(resp, "FLOW:") != NULL)
            {
                uint16_t salt_flow = 0, brine_flow = 0;
                float rpm = disp_state.rpm_feedback;
                int parsed = sscanf(resp, "FLOW:SALT:%hu,BRINE:%hu,RPM:%f", &salt_flow, &brine_flow, &rpm);
                if (parsed >= 2)
                {
                    disp_state.salt_flow_mlmin = salt_flow;
                    disp_state.brine_flow_mlmin = brine_flow;
                    if (parsed == 3)
                    {
                        disp_state.rpm_feedback = rpm;
                    }
                    if (disp_state.verbose_test_responses) {
                        printf("[DISP] Flow updated: Salt=%d mL/min, Brine=%d mL/min, RPM=%.1f\r\n",
                               salt_flow, brine_flow, disp_state.rpm_feedback);
                    }
                }
            }

            // Reset buffer
            disp_state.rx_index = 0;
        }
        return;
    }

    // Add byte to buffer
    if (disp_state.rx_index < (DISP_RX_BUFFER_SIZE - 1))
    {
        disp_state.rx_buffer[disp_state.rx_index++] = byte;
    }
    else
    {
        // Buffer overflow - reset
        printf("[DISP] RX buffer overflow, resetting\r\n");
        disp_state.rx_index = 0;
    }
}

// Dispersion task - called periodically to monitor/telemetry only
void Dispersion_Task(void)
{
    if (!disp_state.initialized)
        return;

    // Read flow sensors
    uint16_t salt_flow = Dispersion_ReadSaltFlow();
    uint16_t brine_flow = Dispersion_ReadBrineFlow();

    // Check for clogged dispensers (monitor only; no automatic rate overrides)
    if (disp_state.salt_rate_percent > 10 && salt_flow < 5)
    {
        printf("[DISP] WARNING: Salt auger may be clogged (rate=%d%%, flow=%d mL/min)\r\n",
               disp_state.salt_rate_percent, salt_flow);
    }

    if (disp_state.brine_rate_percent > 10 && brine_flow < 5)
    {
        printf("[DISP] WARNING: Brine pump may be clogged (rate=%d%%, flow=%d mL/min)\r\n",
               disp_state.brine_rate_percent, brine_flow);
    }

    // Periodic monitoring (every 5 seconds)
    static uint32_t last_log = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_log) >= 5000)
    {
        last_log = now;
        printf("[DISP] Flows: salt=%d mL/min, brine=%d mL/min (ratio=%.1f:1)\r\n",
               salt_flow, brine_flow,
               (salt_flow > 0) ? (float)brine_flow / (float)salt_flow : 0.0f);
    }
}

// Get last status from dispersion ESP32
const char* Dispersion_GetLastStatus(void)
{
    return disp_state.last_status;
}

uint32_t Dispersion_GetLastTxMs(void)
{
    return disp_state.last_tx_ms;
}

uint32_t Dispersion_GetLastRxMs(void)
{
    return disp_state.last_rx_ms;
}

uint32_t Dispersion_GetTxCount(void)
{
    return disp_state.tx_count;
}

uint32_t Dispersion_GetRxCount(void)
{
    return disp_state.rx_count;
}

void Dispersion_SetTestResponseMode(uint8_t enable)
{
    disp_state.verbose_test_responses = enable ? 1u : 0u;
}

// Get current salt dispersion rate (0-100%)
uint8_t Dispersion_GetSaltRate(void)
{
    return disp_state.salt_rate_percent;
}

// Get current brine dispersion rate (0-100%)
uint8_t Dispersion_GetBrineRate(void)
{
    return disp_state.brine_rate_percent;
}
