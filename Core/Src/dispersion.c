#include "dispersion.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>

// ============================================================================
// SALT AND BRINE DISPERSION CONTROL (PHASE 4)
// UART 4 communication with dispersion ESP32 (9600 baud)
// ============================================================================

#define DISP_RX_BUFFER_SIZE 128
#define DISP_RATE_RESEND_MS 1500u

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
    uint8_t rx_discard_until_eol;
    char last_status[128];
    uint32_t clog_timeout_ticks;// Ticks until clog detection timeout
    uint8_t verbose_test_responses; // 1=print ESP responses for test salt/brine modes
    uint32_t last_tx_ms;
    uint32_t last_rx_ms;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t raw_rx_byte_count;
    // PHASE 4: Startup check state tracking (sync with ESP32 startup gating)
    uint8_t startup_check_in_progress;  // ESP32 is running startup check
    uint8_t startup_check_completed;    // ESP32 has completed startup check
    uint32_t startup_check_started_ms;  // Timestamp when startup check was requested
    uint32_t startup_check_timeout_ms;  // Max duration allowed for startup (180s on ESP32)
} Dispersion_State_t;

static Dispersion_State_t disp_state = {0};

void Dispersion_RequestStartupCheck(void)
{
    if (!disp_state.initialized || !disp_state.huart)
        return;

    const char *cmd = "STARTUP_CHECK\r\n";
    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
    if (tx_status == HAL_OK) {
        disp_state.last_tx_ms = HAL_GetTick();
        disp_state.tx_count++;
        // Mark startup check as in-progress
        disp_state.startup_check_in_progress = 1;
        disp_state.startup_check_completed = 0;
        disp_state.startup_check_started_ms = HAL_GetTick();
        printf("[DISP] TX CMD: %s", cmd);
        printf("[DISP] Startup check initiated (ESP32 will agitate for 180s)\r\n");
    } else {
        printf("[DISP] UART4 TX failed (StartupCheck), status=%d\r\n", (int)tx_status);
    }
}

void Dispersion_BypassStartupCheck(void)
{
    if (!disp_state.initialized || !disp_state.huart)
        return;

    const char *cmd = "STARTUP_BYPASS\r\n";
    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
    if (tx_status == HAL_OK) {
        disp_state.last_tx_ms = HAL_GetTick();
        disp_state.tx_count++;
        // Mark startup check as bypassed (completed immediately)
        disp_state.startup_check_in_progress = 0;
        disp_state.startup_check_completed = 1;
        disp_state.startup_check_started_ms = HAL_GetTick();
        printf("[DISP] TX CMD: %s", cmd);
        printf("[DISP] Startup check bypassed (immediate unlock)\r\n");
    } else {
        printf("[DISP] UART4 TX failed (StartupBypass), status=%d\r\n", (int)tx_status);
    }
}

// Initialize dispersion passthrough link
// - STM32 uses UART4 to command the dedicated dispersion ESP32
// - The ESP32 owns external outputs, startup gating, and feedback telemetry
void Dispersion_Init(UART_HandleTypeDef *huart4)
{
    // Hardware ownership note: the dedicated ESP32 handles external outputs and
    // local feedback processing, so the STM32 only maintains the UART control link.
    
    disp_state.huart = huart4;
    disp_state.initialized = 1;
    disp_state.salt_rate_percent = 0;
    disp_state.brine_rate_percent = 0;
    disp_state.salt_flow_mlmin = 0;
    disp_state.brine_flow_mlmin = 0;
    disp_state.rpm_feedback = 0.0f;
    disp_state.rx_index = 0;
    disp_state.rx_discard_until_eol = 0;
    memset(disp_state.rx_buffer, 0, DISP_RX_BUFFER_SIZE);
    memset(disp_state.last_status, 0, sizeof(disp_state.last_status));
    disp_state.verbose_test_responses = 0;
    disp_state.last_tx_ms = 0;
    disp_state.last_rx_ms = 0;
    disp_state.tx_count = 0;
    disp_state.rx_count = 0;
    disp_state.raw_rx_byte_count = 0;
    // Initialize startup check state: not in progress, not completed
    disp_state.startup_check_in_progress = 0;
    disp_state.startup_check_completed = 0;
    disp_state.startup_check_started_ms = 0;
    disp_state.startup_check_timeout_ms = 190000;  // 190s (ESP32 runs 180s check + 10s margin)

    printf("[DISP] Dispersion system initialized (UART 4 at 9600 baud)\r\n");
}

// Set dispersion rates as direct pass-through percentages
// salt_rate: 0-100 (0-100% output)
// brine_rate: 0-100 (0-100% output)
void Dispersion_SetRate(uint8_t salt_rate, uint8_t brine_rate)
{
    if (!disp_state.initialized)
        return;

    // PHASE 4: Gate commands during startup check
    // ESP32 will reject SALT/BRINE commands until startup_check_completed
    if (!disp_state.startup_check_completed) {
        if (disp_state.startup_check_in_progress) {
            printf("[DISP] WARNING: Ignoring SALT/BRINE command during startup check (in progress)\r\n");
        } else {
            printf("[DISP] WARNING: Ignoring SALT/BRINE command - startup check not yet initiated\r\n");
        }
        return;  // Do not send; gate the command
    }

    // Clamp input ranges
    if (salt_rate > 100) salt_rate = 100;
    if (brine_rate > 100) brine_rate = 100;

    uint32_t now_ms = HAL_GetTick();
    if (disp_state.salt_rate_percent == salt_rate &&
        disp_state.brine_rate_percent == brine_rate &&
        (now_ms - disp_state.last_tx_ms) < DISP_RATE_RESEND_MS) {
        return;
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
        HAL_StatusTypeDef tx_status = HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
        if (tx_status == HAL_OK) {
            disp_state.last_tx_ms = HAL_GetTick();
            disp_state.tx_count++;
            printf("[DISP] TX CMD: %s", cmd);
        } else {
            printf("[DISP] UART4 TX failed (SetRate), status=%d\r\n", (int)tx_status);
        }
    }

    // The dedicated ESP32 applies the actual salt/brine output logic once it receives
    // the UART command, so there is no local PWM command path on the STM32.

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

    uint32_t now_ms = HAL_GetTick();
    if (disp_state.salt_rate_percent == salt_rate &&
        disp_state.brine_rate_percent == brine_rate &&
        (now_ms - disp_state.last_tx_ms) < 250u) {
        return;
    }

    disp_state.salt_rate_percent = salt_rate;
    disp_state.brine_rate_percent = brine_rate;

    char cmd[32];
    snprintf(cmd, sizeof(cmd), "SALT:%d,BRINE:%d\r\n", salt_rate, brine_rate);

    if (disp_state.huart)
    {
        HAL_StatusTypeDef tx_status = HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
        if (tx_status == HAL_OK) {
            disp_state.last_tx_ms = HAL_GetTick();
            disp_state.tx_count++;
            printf("[DISP] TX CMD: %s", cmd);
        } else {
            printf("[DISP] UART4 TX failed (SetRateDirect), status=%d\r\n", (int)tx_status);
        }
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
        HAL_StatusTypeDef tx_status = HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, (uint16_t)len, HAL_MAX_DELAY);
        if (tx_status == HAL_OK) {
            disp_state.last_tx_ms = HAL_GetTick();
            disp_state.tx_count++;
            printf("[DISP] TX CMD: %s", cmd);
        } else {
            printf("[DISP] UART4 TX failed (SendPercentOnly), status=%d\r\n", (int)tx_status);
        }
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

        HAL_StatusTypeDef tx_status = HAL_UART_Transmit(disp_state.huart, (uint8_t *)cmd, (uint16_t)in_len, HAL_MAX_DELAY);
        if (tx_status == HAL_OK) {
            disp_state.last_tx_ms = HAL_GetTick();
            disp_state.tx_count++;
            printf("[DISP] Raw TX: %s", cmd);
        } else {
            printf("[DISP] UART4 TX failed (SendRaw), status=%d\r\n", (int)tx_status);
        }
    }
}

// Read latest salt flow estimate relayed by the dispersion ESP32 (mL/min)
// Returns: Flow rate in mL/min, 0 if no feedback has been received yet
uint16_t Dispersion_ReadSaltFlow(void)
{
    // The STM32 relies on the latest flow value reported by the dispersion ESP32.

    // Return last cached value
    
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

    disp_state.raw_rx_byte_count++;

    // Ignore NUL and other non-printable control noise bytes (except CR/LF)
    if (byte != '\r' && byte != '\n' && !isprint((int)byte)) {
        return;
    }

    // Check for line ending (CR or LF)
    if (byte == '\r' || byte == '\n')
    {
        // End overflow-discard mode once line terminates
        if (disp_state.rx_discard_until_eol)
        {
            disp_state.rx_discard_until_eol = 0;
            disp_state.rx_index = 0;
            return;
        }

        // Process complete message
        if (disp_state.rx_index > 0)
        {
            disp_state.rx_buffer[disp_state.rx_index] = '\0';

            char *line = (char *)disp_state.rx_buffer;
            while (*line && isspace((unsigned char)*line)) {
                line++;
            }

            char *line_end = line + strlen(line);
            while (line_end > line && isspace((unsigned char)line_end[-1])) {
                line_end--;
            }
            *line_end = '\0';

            if (line[0] != '\0') {
                const char *resp = line;
                const char *known_prefix = strstr(line, "STATUS:");
                if (!known_prefix) known_prefix = strstr(line, "FLOW:");
                if (!known_prefix) {
                    // Ignore malformed/noise lines so RX health reflects only valid ESP frames.
                    disp_state.rx_index = 0;
                    return;
                }
                resp = known_prefix;

                strncpy(disp_state.last_status, resp, sizeof(disp_state.last_status) - 1);
                disp_state.last_status[sizeof(disp_state.last_status) - 1] = '\0';
                disp_state.last_rx_ms = HAL_GetTick();
                disp_state.rx_count++;

                // PHASE 4: Parse startup check state from ESP32 responses
                if (strstr(resp, "STARTUP_CHECK_STARTED") != NULL) {
                    disp_state.startup_check_in_progress = 1;
                    disp_state.startup_check_completed = 0;
                    printf("[DISP] RX: Startup check started on ESP32 (will complete in ~180s)\r\n");
                }
                else if (strstr(resp, "STARTUP_CHECK_BYPASSED") != NULL) {
                    disp_state.startup_check_in_progress = 0;
                    disp_state.startup_check_completed = 1;
                    printf("[DISP] RX: Startup check bypassed on ESP32 (immediately ready)\r\n");
                }
                else if (strstr(resp, "STARTUP_CHECK_RUNNING") != NULL) {
                    disp_state.startup_check_in_progress = 1;
                    disp_state.startup_check_completed = 0;
                    printf("[DISP] RX: Startup check already running on ESP32\r\n");
                }
                else if (strstr(resp, "STARTUP_REQUIRED") != NULL) {
                    printf("[DISP] RX: Command rejected - startup check required on ESP32\r\n");
                }

                if (strncmp(resp, "STATUS:OK", 9) == 0)
                {
                    // OK status received (may have additional qualifiers parsed above)
                }
                else if (strncmp(resp, "STATUS:ERROR", 12) == 0)
                {
                    // ERROR status - may be startup required or clog
                    if (strstr(resp, "STARTUP_REQUIRED") == NULL) {
                        // Assume clog if not startup-related
                        printf("[DISP] RX: Error response from ESP32 (possible clog)\r\n");
                    }
                }
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
                    }
                }
            }

            // Reset buffer
            disp_state.rx_index = 0;
        }
        return;
    }

    // If prior overflow happened, discard until end-of-line
    if (disp_state.rx_discard_until_eol)
    {
        return;
    }

    // Add byte to buffer
    if (disp_state.rx_index < (DISP_RX_BUFFER_SIZE - 1))
    {
        disp_state.rx_buffer[disp_state.rx_index++] = byte;
    }
    else
    {
        // Buffer overflow - discard remainder of this line until EOL
        printf("[DISP] RX line overflow, discarding until newline\r\n");
        disp_state.rx_discard_until_eol = 1;
        disp_state.rx_index = 0;
    }
}

// Dispersion task - called periodically to monitor/telemetry only
void Dispersion_Task(void)
{
    if (!disp_state.initialized)
        return;

    // PHASE 4: Monitor startup check timeout
    uint32_t now = HAL_GetTick();
    if (disp_state.startup_check_in_progress && !disp_state.startup_check_completed)
    {
        uint32_t elapsed = now - disp_state.startup_check_started_ms;
        if (elapsed > disp_state.startup_check_timeout_ms)
        {
            printf("[DISP] ERROR: Startup check timeout! (elapsed=%lu ms, timeout=%lu ms)\r\n",
                   (unsigned long)elapsed,
                   (unsigned long)disp_state.startup_check_timeout_ms);
            printf("[DISP] Auto-completing startup check (may indicate ESP32 hang)\r\n");
            disp_state.startup_check_in_progress = 0;
            disp_state.startup_check_completed = 1;
        }
    }

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

uint32_t Dispersion_GetRawRxByteCount(void)
{
    return disp_state.raw_rx_byte_count;
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

// PHASE 4: Get startup check state
// Returns: 1 if startup check is complete and commands can be sent, 0 otherwise
uint8_t Dispersion_IsStartupCheckComplete(void)
{
    return disp_state.startup_check_completed;
}

// Get startup check in-progress status
// Returns: 1 if startup check is currently running, 0 otherwise
uint8_t Dispersion_IsStartupCheckInProgress(void)
{
    return disp_state.startup_check_in_progress;
}

// Get time elapsed since startup check started (milliseconds)
// Useful for progress indication to operator
uint32_t Dispersion_GetStartupCheckElapsedMs(void)
{
    if (!disp_state.startup_check_in_progress && !disp_state.startup_check_started_ms)
        return 0;  // Not started
    
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = (disp_state.startup_check_started_ms > 0) ? 
                       (now - disp_state.startup_check_started_ms) : 0;
    return elapsed;
}

