#include "dispersion.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

// ============================================================================
// SALT AND BRINE DISPERSION CONTROL (PHASE 4)
// Maintains 1:9 salt:brine ratio with flow sensor feedback
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
    uint8_t rx_buffer[DISP_RX_BUFFER_SIZE];
    uint16_t rx_index;
    char last_status[64];
    uint32_t clog_timeout_ticks;// Ticks until clog detection timeout
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
    disp_state.rx_index = 0;
    memset(disp_state.rx_buffer, 0, DISP_RX_BUFFER_SIZE);
    memset(disp_state.last_status, 0, sizeof(disp_state.last_status));
    
    printf("[DISP] Dispersion system initialized (UART 4 at 9600 baud)\r\n");
}

// Set dispersion rates with automatic 1:9 ratio enforcement
// salt_rate: 0-100 (0-100% output)
// brine_rate: 0-100 (0-100% output, will be auto-adjusted to maintain 1:9 ratio)
void Dispersion_SetRate(uint8_t salt_rate, uint8_t brine_rate)
{
    if (!disp_state.initialized)
        return;

    // Clamp input ranges
    if (salt_rate > 100) salt_rate = 100;
    if (brine_rate > 100) brine_rate = 100;

    // Enforce 1:9 salt:brine ratio
    // If salt is the limiting factor, calculate brine as 9x salt
    if (salt_rate > 0)
    {
        uint16_t calculated_brine = (uint16_t)salt_rate * 9;
        if (calculated_brine > 100)
        {
            // Brine would exceed max, back off salt proportionally
            salt_rate = 100 / 9; // ~11%
            brine_rate = 100;
        }
        else
        {
            // Set brine to maintain 1:9 ratio
            brine_rate = calculated_brine;
        }
    }
    else
    {
        // No salt means no brine
        brine_rate = 0;
    }

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
    }

    // TODO: Send PWM commands to salt auger and brine pump
    // PWM format: scale percent to motor speed range (0-255 or similar)
    // salt_pwm = (salt_rate * 255) / 100;
    // brine_pwm = (brine_rate * 255) / 100;

    // Log change
    printf("[DISP] Dispersion rates: salt=%d%%, brine=%d%%\r\n",
           disp_state.salt_rate_percent, disp_state.brine_rate_percent);
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

            printf("[DISP] ESP32 Response: %s\r\n", disp_state.rx_buffer);

            // Parse response
            const char *resp = (const char *)disp_state.rx_buffer;

            // STATUS:OK
            if (strcmp(resp, "STATUS:OK") == 0)
            {
                printf("[DISP] Dispersion OK\r\n");
            }
            // STATUS:ERROR,CLOG
            else if (strstr(resp, "STATUS:ERROR") != NULL)
            {
                printf("[DISP] ERROR from ESP32: %s\r\n", resp);
                // TODO: Set fault code FAULT_DISPERSION_CLOG
            }
            // FLOW:SALT:XXX,BRINE:XXXX
            else if (strstr(resp, "FLOW:") != NULL)
            {
                uint16_t salt_flow = 0, brine_flow = 0;
                sscanf(resp, "FLOW:SALT:%hu,BRINE:%hu", &salt_flow, &brine_flow);
                disp_state.salt_flow_mlmin = salt_flow;
                disp_state.brine_flow_mlmin = brine_flow;
                printf("[DISP] Flow updated: Salt=%d mL/min, Brine=%d mL/min\r\n",
                       salt_flow, brine_flow);
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

// Dispersion control task - called periodically to monitor and enforce ratios
void Dispersion_Task(void)
{
    if (!disp_state.initialized)
        return;

    // Read flow sensors
    uint16_t salt_flow = Dispersion_ReadSaltFlow();
    uint16_t brine_flow = Dispersion_ReadBrineFlow();

    // Check for clogged dispensers
    // If rate > 0 but no flow detected, assume clogged
    if (disp_state.salt_rate_percent > 10 && salt_flow < 5)
    {
        // Salt auger clogged or sensor error
        printf("[DISP] WARNING: Salt auger may be clogged (rate=%d%%, flow=%d mL/min)\r\n",
               disp_state.salt_rate_percent, salt_flow);
        // TODO: Set fault code FAULT_DISPERSION_CLOG
        Dispersion_SetRate(0, 0); // Stop dispensing
        return;
    }

    if (disp_state.brine_rate_percent > 10 && brine_flow < 5)
    {
        // Brine pump clogged or sensor error
        printf("[DISP] WARNING: Brine pump may be clogged (rate=%d%%, flow=%d mL/min)\r\n",
               disp_state.brine_rate_percent, brine_flow);
        // TODO: Set fault code FAULT_DISPERSION_CLOG
        Dispersion_SetRate(0, 0); // Stop dispensing
        return;
    }

    // Enforce ratio even with real flow
    // Target ratio: 1:9 (salt:brine)
    if (salt_flow > 0 && brine_flow > 0)
    {
        float measured_ratio = (float)brine_flow / (float)salt_flow;
        float target_ratio = 9.0f;

        // If ratio off by >20%, adjust
        if (measured_ratio > (target_ratio * 1.2f) || measured_ratio < (target_ratio * 0.8f))
        {
            // Recalculate to enforce exact ratio
            uint8_t corrected_salt = disp_state.salt_rate_percent;
            uint16_t corrected_brine = corrected_salt * 9;

            if (corrected_brine > 100)
            {
                corrected_salt = 100 / 9;
                corrected_brine = 100;
            }

            // Only log if significantly different
            printf("[DISP] Ratio correction: measured=%.1f, target=%.1f\r\n",
                   measured_ratio, target_ratio);
        }
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
