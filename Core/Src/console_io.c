#include "console_io.h"
#include "robot_sm.h"
#include "robot_actions.h"
#include "diagnostics.h"
#include "sabertooth.h"
#include "uart_lora.h"
#include "dispersion.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <ctype.h>

static int ParseTwoInts(const char *s, int *out_a, int *out_b)
{
    if (!s || !out_a || !out_b) return 0;

    int found = 0;
    const char *p = s;
    while (*p && found < 2) {
        while (*p && !isdigit((unsigned char)*p) && *p != '-' && *p != '+') {
            p++;
        }
        if (!*p) break;
        char *end = NULL;
        long val = strtol(p, &end, 10);
        if (p == end) {
            p++;
            continue;
        }
        if (found == 0) *out_a = (int)val;
        else *out_b = (int)val;
        found++;
        p = end;
    }

    return (found == 2);
}

// Module-level pointer for printf redirection (USART2 by default)
static UART_HandleTypeDef *g_printf_uart = NULL;

// Store UART handles for printf, GPS, and Sabertooth motor driver
void Console_Init(ConsoleIO_t *c,
                  UART_HandleTypeDef *uart_printf,
                  UART_HandleTypeDef *uart_gps,
                  UART_HandleTypeDef *uart_saber)
{
    if (!c) return;
    c->uart_printf = uart_printf;
    c->uart_gps    = uart_gps;
    c->uart_saber  = uart_saber;

    g_printf_uart = uart_printf;
}

// System hook for printf: redirect to USART2
int _write(int file, char *ptr, int len)
{
    (void)file;
    if (g_printf_uart == NULL) return len;
    HAL_UART_Transmit(g_printf_uart, (uint8_t *)ptr, len, HAL_MAX_DELAY);
    return len;
}

// Reinitialize UARTs and clear terminal (use sparingly in live operation)
void Console_FlushAll(ConsoleIO_t *c)
{
    if (!c) return;

    if (c->uart_saber) {
        __HAL_UART_FLUSH_DRREGISTER(c->uart_saber);
        HAL_UART_DeInit(c->uart_saber);
        HAL_UART_Init(c->uart_saber);
    }

    if (c->uart_printf) {
        __HAL_UART_FLUSH_DRREGISTER(c->uart_printf);
        HAL_UART_DeInit(c->uart_printf);
        HAL_UART_Init(c->uart_printf);
    }

    if (c->uart_gps) {
        __HAL_UART_FLUSH_DRREGISTER(c->uart_gps);
        HAL_UART_DeInit(c->uart_gps);
        HAL_UART_Init(c->uart_gps);

        // IMPORTANT: re-init GPS parser after UART restart
        GPS_Init(c->uart_gps);
    }

    // Clear terminal + home cursor
    printf("\033[2J\033[H");
    printf("Serial buffers cleared.\r\n");
}

// Print sensor data (IMU, GPS, fusion) to console with color formatting
void Console_PrintStatus(const ConsoleIO_t *c,
                         const IMU_Status_t *imu,
                         const GPS_Data_t *gps,
                         const HeadingFusion_t *hf)
{
    (void)c;
    if (!imu || !gps || !hf) return;

    uint32_t now_ms = HAL_GetTick();
    uint32_t disp_last_tx = Dispersion_GetLastTxMs();
    uint32_t disp_last_rx = Dispersion_GetLastRxMs();
    uint32_t lora_last_tx = LoRA_GetLastTxMs();
    uint32_t lora_last_rx = LoRA_GetLastRxMs();

    uint32_t disp_tx_age = disp_last_tx ? (now_ms - disp_last_tx) : 0xFFFFFFFFu;
    uint32_t disp_rx_age = disp_last_rx ? (now_ms - disp_last_rx) : 0xFFFFFFFFu;
    uint32_t lora_tx_age = lora_last_tx ? (now_ms - lora_last_tx) : 0xFFFFFFFFu;
    uint32_t lora_rx_age = lora_last_rx ? (now_ms - lora_last_rx) : 0xFFFFFFFFu;

    uint32_t disp_tx_count = Dispersion_GetTxCount();
    uint32_t disp_rx_count = Dispersion_GetRxCount();
    uint32_t lora_tx_count = LoRA_GetTxCount();
    uint32_t lora_rx_count = LoRA_GetRxCount();

    static uint32_t prev_disp_tx_count = 0;
    static uint32_t prev_disp_rx_count = 0;
    static uint32_t prev_lora_tx_count = 0;
    static uint32_t prev_lora_rx_count = 0;

    char disp_tx_hb = (disp_tx_count != prev_disp_tx_count) ? '*' : '-';
    char disp_rx_hb = (disp_rx_count != prev_disp_rx_count) ? '*' : '-';
    char lora_tx_hb = (lora_tx_count != prev_lora_tx_count) ? '*' : '-';
    char lora_rx_hb = (lora_rx_count != prev_lora_rx_count) ? '*' : '-';

    prev_disp_tx_count = disp_tx_count;
    prev_disp_rx_count = disp_rx_count;
    prev_lora_tx_count = lora_tx_count;
    prev_lora_rx_count = lora_rx_count;

    printf(ANSI_MAGENTA "\r\n================ SENSOR STATUS ================\r\n" ANSI_RESET);

    printf(ANSI_CYAN "ACC (g):      %+10.6f  %+10.6f  %+10.6f\r\n" ANSI_RESET,
           imu->ax_g, imu->ay_g, imu->az_g);

    printf(ANSI_CYAN "GYR (rad/s):  %+10.6f  %+10.6f  %+10.6f\r\n" ANSI_RESET,
           imu->gx_rad_s, imu->gy_rad_s, imu->gz_rad_s);

    // If you later enable mag, add it here (for now keep consistent with your print)
    // printf(ANSI_CYAN "MAG (uT):     ...\r\n" ANSI_RESET);

    printf(ANSI_CYAN "YPR (deg):    " ANSI_YELLOW
           "Yaw=%+10.6f  Pitch=%+10.6f  Roll=%+10.6f\r\n" ANSI_RESET,
           hf->yaw_deg, hf->pitch_deg, hf->roll_deg);

    if (gps->has_fix)
        printf(ANSI_GREEN "GPS Fix:      VALID\r\n" ANSI_RESET);
    else
        printf(ANSI_RED   "GPS Fix:      NO FIX\r\n" ANSI_RESET);

    printf(ANSI_YELLOW "Latitude:     %+12.6f deg\r\n", gps->latitude_deg);
    printf("Longitude:    %+12.6f deg\r\n", gps->longitude_deg);
    printf("Speed:        %10.6f kn\r\n", gps->speed_knots);
    printf("Course:       %10.6f deg\r\n" ANSI_RESET, gps->course_deg);

    float gps_course_deg_filt = hf->gps_course_filt_rad * (57.2957795130823f);

    printf(ANSI_CYAN "Heading Fused:        " ANSI_YELLOW "%10.6f deg\r\n" ANSI_RESET,
           hf->yaw_deg);

    printf(ANSI_CYAN "GPS Course (raw/filt): %10.6f / %10.6f deg\r\n" ANSI_RESET,
           gps->course_deg, gps_course_deg_filt);

    printf(ANSI_CYAN "Speed (raw/filt):      %10.6f / %10.6f kn\r\n" ANSI_RESET,
           gps->speed_knots, hf->gps_speed_filt_kn);

    printf(ANSI_CYAN "Speed Accel Est.:      %10.6f kn\r\n" ANSI_RESET,
           hf->accel_speed_filt_kn);

    printf(ANSI_GREEN "Speed FUSED (best):    %10.6f kn\r\n" ANSI_RESET,
           hf->speed_fused_kn);

    printf(ANSI_GREEN "Heading Confidence:    %.2f  " ANSI_RESET
           "(GPS weight=%.3f, Gyro weight=%.3f)\r\n",
           hf->heading_confidence,
           hf->gps_weight,
           1.0f - hf->gps_weight);

        printf(ANSI_CYAN "ESP32 SB Link: " ANSI_RESET
            "TX[%c] %s  RX[%c] %s\r\n",
            disp_tx_hb,
            (disp_tx_age == 0xFFFFFFFFu) ? "--" : (disp_tx_age < 3000u ? "OK" : "STALE"),
            disp_rx_hb,
            (disp_rx_age == 0xFFFFFFFFu) ? "--" : (disp_rx_age < 10000u ? "OK" : "STALE"));

        printf(ANSI_CYAN "ESP32 LoRa Link: " ANSI_RESET
            "TX[%c] %s  RX[%c] %s\r\n",
            lora_tx_hb,
            (lora_tx_age == 0xFFFFFFFFu) ? "--" : (lora_tx_age < 3000u ? "OK" : "STALE"),
            lora_rx_hb,
            (lora_rx_age == 0xFFFFFFFFu) ? "--" : (lora_rx_age < 15000u ? "OK" : "STALE"));

    printf(ANSI_MAGENTA "================================================\r\n" ANSI_RESET);
}

// Console input buffer and handler (USART2 polled RX)
static char console_input_buf[256] = {0};
static uint16_t console_input_idx = 0;
static volatile uint8_t *s_test_mode_flag = NULL;
static volatile uint8_t s_esc_pressed = 0;

static inline void Console_WatchdogKick(void)
{
    IWDG->KR = 0xAAAA;
}

static int Console_ReadLineBlocking(const char *prompt, char *out, size_t out_size)
{
    if (!out || out_size < 2) return 0;

    extern UART_HandleTypeDef huart2;
    size_t idx = 0;
    out[0] = '\0';

    // Clear any buffered CR/LF from the command that entered this prompt
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        (void)(huart2.Instance->DR & 0xFF);
    }

    if (prompt && prompt[0] != '\0') {
        printf("%s", prompt);
    }

    while (1)
    {
        Console_WatchdogKick();

        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);

            if (ch == 0x1B) {
                printf("\r\n[DIAG] Input cancelled\r\n");
                out[0] = '\0';
                return 0;
            }

            if (ch == '\r' || ch == '\n') {
                // Ignore stray newline before user typed anything
                if (idx == 0) {
                    continue;
                }
                out[idx] = '\0';
                printf("\r\n");
                return 1;
            }

            if (ch == 0x08 || ch == 0x7F) {
                if (idx > 0) {
                    idx--;
                    printf("\b \b");
                }
                continue;
            }

            if (ch >= 32 && ch < 127) {
                if (idx < (out_size - 1)) {
                    out[idx++] = (char)ch;
                    printf("%c", ch);
                }
            }
        }

        if (Console_CheckEscPressed()) {
            printf("\r\n[DIAG] Input cancelled\r\n");
            out[0] = '\0';
            return 0;
        }

        HAL_Delay(1);
    }
}

static void Console_SendLoRaExampleCommands(void)
{
    static const char *examples[] = {
        "CMD:MANUAL",
        "CMD:AUTO,SALT:25,BRINE:75",
        "CMD:PAUSE",
        "CMD:ESTOP"
    };

    printf(ANSI_CYAN "[DIAG] Sending example LoRa control commands...\r\n" ANSI_RESET);
    for (size_t i = 0; i < (sizeof(examples) / sizeof(examples[0])); i++) {
        Console_WatchdogKick();
        LoRA_SendRaw(examples[i]);
        printf("[DIAG] LoRa example TX: %s\r\n", examples[i]);
        HAL_Delay(200);
    }
    printf(ANSI_GREEN "[DIAG] LoRa example command sequence complete\r\n" ANSI_RESET);
}

static int Console_IsLikelySbEspFrame(const char *s)
{
    if (!s || s[0] == '\0') return 0;

    // Known SB-ESP payload markers from UART4 path
    if (strstr(s, "FLOW:") != NULL) return 1;
    if (strstr(s, "STATUS:") != NULL) return 1;
    if (strstr(s, "SALT:") != NULL) return 1;
    if (strstr(s, "BRINE:") != NULL) return 1;

    return 0;
}

static int Console_IsMostlyPrintableAscii(const char *s)
{
    if (!s || s[0] == '\0') return 0;

    uint32_t printable = 0;
    uint32_t total = 0;
    for (const unsigned char *p = (const unsigned char *)s; *p != '\0'; ++p) {
        total++;
        if ((*p >= 32 && *p <= 126) || *p == '\t') {
            printable++;
        }
    }

    if (total == 0) return 0;

    // Require at least 85% printable bytes
    return (printable * 100u) >= (85u * total);
}

void Console_ShowTestMenu(void)
{
    // Clear console to remove old sensor status messages
    printf("\033[2J\033[H");
    
    printf(ANSI_YELLOW "========== QUICK TEST MENU ==========" ANSI_RESET "\r\n");
    printf("1. TEST ALL       - Run all diagnostics\r\n");
    printf("2. TEST MOTOR 1 30 - Motor 1 forward (percent)\r\n");
    printf("3. TEST MOTOR 2 30 - Motor 2 forward (percent)\r\n");
    printf("4. TEST GPS       - GPS position + quality\r\n");
    printf("5. TEST IMU       - Accel/Gyro/Magnetometer\r\n");
    printf("6. TEST TEMP      - Temperature sensor\r\n");
    printf("7. TEST SALT 50   - Salt pump 50%%\r\n");
    printf("8. TEST BRINE 50  - Brine pump 50%%\r\n");
    printf("9. TEST STATE     - Show current state\r\n");
    printf("0. TEST MOTORSWEEP - Sweep motor speeds (percent)\r\n");
    printf("H. TEST HEALTH    - System health diagnostic\r\n");
    printf("C. TEST RAWCMD M2: 500 - Send raw Sabertooth command\r\n");
    printf(ANSI_CYAN "I. TEST I2C       - Scan I2C bus\r\n");
    printf("D. TEST IMUDETAIL - Detailed IMU check\r\n");
    printf("L. TEST LORA SEND - Type and send raw LoRa string\r\n");
    printf("R. TEST LORA RX   - Monitor incoming LoRa messages (press ESC)\r\n");
    printf("X. TEST LORACMDS  - Send example LoRa control command set\r\n");
    printf("E. TEST SBESP SEND - Type and send Salt-Brine ESP32 string\r\n");
    printf("Q. TEST SBESP RX   - Monitor Salt-Brine ESP32 RX (press ESC)\r\n");
    printf("P. TEST PROXIMITY - Ultrasonic sensors left/right (press ESC)\r\n");
    printf("M. TEST SABERTOOTH MONITOR - Live feedback polling (press ESC)\r\n" ANSI_RESET);
    printf("Or type the full command (e.g. TEST MOTOR 1 25)\r\n");
    printf(ANSI_CYAN "Type HELP (or ?) for command parameters\r\n" ANSI_RESET);
    printf(ANSI_CYAN "Press ESC during tests to return to menu\r\n" ANSI_RESET);
    printf(ANSI_RED "Type EXIT to resume normal operation\r\n" ANSI_RESET);
    printf(ANSI_YELLOW "======================================" ANSI_RESET "\r\n");
}

void Console_SetTestModeFlag(volatile uint8_t *flag)
{
    s_test_mode_flag = flag;
    if (s_test_mode_flag) {
        *s_test_mode_flag = 0;
    }
}

uint8_t Console_CheckEscPressed(void)
{
    if (s_esc_pressed) {
        s_esc_pressed = 0;
        return 1;
    }
    return 0;
}

// Call this from USART2 RX interrupt handler or polling loop to accumulate input
void Console_RxByte(uint8_t byte, RobotSM_t *sm)
{
    // ESC key (0x1B) - exit continuous tests
    if (byte == 0x1B) {
        s_esc_pressed = 1;
        printf("\r\n[ESC pressed]\r\n");
        return;
    }
    
    // Backspace handling
    if (byte == 0x08 || byte == 0x7F) {
        if (console_input_idx > 0) {
            console_input_idx--;
            printf("\b \b");  // Erase character on terminal
        }
        return;
    }

    // Enter/newline: process command
    if (byte == '\r' || byte == '\n') {
        console_input_buf[console_input_idx] = '\0';
        printf("\r\n");
        Console_ProcessCommand(console_input_buf, sm);
        console_input_idx = 0;
        return;
    }

    // Printable characters
    if (byte >= 32 && byte < 127 && console_input_idx < sizeof(console_input_buf) - 1) {
        console_input_buf[console_input_idx++] = (char)byte;
        printf("%c", byte);  // Echo character
    }
}

void Console_ProcessCommand(const char *cmd, RobotSM_t *sm)
{
    if (!cmd || !sm) return;

    // Optional: ignore empty lines / whitespace-only
    if (cmd[0] == '\0') return;

    char cmd_upper[256];
    size_t cmd_len = strnlen(cmd, sizeof(cmd_upper) - 1);
    for (size_t i = 0; i < cmd_len; i++) {
        cmd_upper[i] = (char)toupper((unsigned char)cmd[i]);
    }
    cmd_upper[cmd_len] = '\0';

    // ========== EXIT TEST MODE ==========
    if (strcmp(cmd_upper, "EXIT") == 0)
    {
        if (s_test_mode_flag) {
            extern UART_HandleTypeDef huart2;
            *s_test_mode_flag = 0;
            // Wait for all pending TX to complete
            HAL_Delay(300);
            // Aggressive flush: disable, clear, reinit
            __HAL_UART_DISABLE(&huart2);
            __HAL_UART_FLUSH_DRREGISTER(&huart2);
            HAL_UART_DeInit(&huart2);
            HAL_UART_Init(&huart2);
            // Reassign global printf pointer after reinit
            g_printf_uart = &huart2;
            // Extra wait to ensure reinitialization complete
            HAL_Delay(150);
            // Clear console multiple times to ensure clean state
            printf("\033[2J\033[H");
            fflush(stdout);
            HAL_Delay(100);
            printf(ANSI_GREEN "Exiting test mode. Resuming normal operation.\r\n" ANSI_RESET);
            fflush(stdout);
        } else {
            printf("Not in test mode.\r\n");
        }
        return;
    }

    // ========== QUICK TEST MENU ==========
    if (strcmp(cmd_upper, "T") == 0)
    {
        if (s_test_mode_flag) {
            *s_test_mode_flag = 1;
        }
        Console_ShowTestMenu();
        return;
    }

    // ========== QUICK MENU SHORTCUTS ==========
    if (strcmp(cmd_upper, "1") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestAll();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "2") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestMotor(1, 30);
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "3") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestMotor(2, 30);
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "4") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestGPS();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "0") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestMotorSweep(100);
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "5") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestIMU();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "6") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        extern UART_HandleTypeDef huart2;
        printf(ANSI_YELLOW "[DIAG] ========== IMU TEMPERATURE TEST ==========\r\n" ANSI_RESET);
        printf("[DIAG] Continuous display (press ESC to exit)\r\n\r\n");
        while (1)
        {
            Console_WatchdogKick();
            if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
                uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
                if (ch == 0x1B) break;
            }
            if (Console_CheckEscPressed()) break;
            IMU_Status_t imu = IMU_Read();
            printf("\r[DIAG] IMU Temp: %.1f deg C  ", imu.temperature_c);
            fflush(stdout);
            HAL_Delay(100);
        }
        printf("\r\n");
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "7") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestSaltRate(50);
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "8") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestBrineRate(50);
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "9") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_PrintState();
        Console_ShowTestMenu();
        return;
    }



    if (strcmp(cmd_upper, "C") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        // Launch raw command prompt with example
        printf(ANSI_CYAN "[DIAG] Raw Sabertooth command (e.g. M2: 500)\r\n" ANSI_RESET);
        printf("[DIAG] Enter command: ");
        // Note: This would need additional input handling, for now show usage
        printf("\r\n[DIAG] Use: TEST RAWCMD M2: 500\r\n");
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "I") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_ScanI2C();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "D") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestIMUDetailed();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "P") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_TestProximity();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "M") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_MonitorSabertoothFeedback();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "H") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Diag_SystemHealth();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "L") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        char lora_text[160] = {0};
        printf(ANSI_CYAN "\r\n[DIAG] ===== LoRa SEND MENU =====\r\n" ANSI_RESET);
        printf("[DIAG] Type text and press ENTER to send over UART5\r\n");
        printf("[DIAG] Press ESC to cancel and return\r\n");
        printf("[DIAG] Example: PING or HELLO\r\n\r\n");
        if (!Console_ReadLineBlocking("[DIAG] Enter LoRa text to send (ESC to cancel): ", lora_text, sizeof(lora_text))) {
            Console_ShowTestMenu();
            return;
        }
        if (lora_text[0] == '\0') {
            printf("[DIAG] Empty input, nothing sent\r\n");
            Console_ShowTestMenu();
            return;
        }
        uint32_t tx_before = LoRA_GetTxCount();
        LoRA_SendRaw(lora_text);
        uint32_t tx_after = LoRA_GetTxCount();
        if (tx_after > tx_before) {
            printf("[DIAG] LoRa TX OK: %s\r\n", lora_text);
        } else {
            printf("[DIAG] LoRa TX FAILED (UART5 not ready?)\r\n");
        }
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "R") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        extern UART_HandleTypeDef huart2;
        uint32_t last_seen_count = LoRA_GetRawFrameCount();
        uint32_t filtered_count = 0;
        printf(ANSI_CYAN "[DIAG] LoRa RX monitor active (press ESC to exit)\r\n" ANSI_RESET);
        while (1)
        {
            Console_WatchdogKick();
            if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
                uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
                if (ch == 0x1B) break;
            }
            if (Console_CheckEscPressed()) break;

            uint32_t current_count = LoRA_GetRawFrameCount();
            if (current_count != last_seen_count) {
                last_seen_count = current_count;
                const char *raw = LoRA_GetLastRawFrame();
                if (raw && raw[0] != '\0') {
                    if (!Console_IsMostlyPrintableAscii(raw) || Console_IsLikelySbEspFrame(raw)) {
                        filtered_count++;
                    } else {
                        printf("[DIAG] LoRa RX: %s\r\n", raw);
                    }
                }
            }

            HAL_Delay(50);
        }
        if (filtered_count > 0) {
            printf("[DIAG] LoRa RX filtered %lu SB-ESP-like frame(s)\r\n", filtered_count);
        }
        printf("[DIAG] LoRa RX monitor stopped\r\n");
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "X") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        Console_SendLoRaExampleCommands();
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "E") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        char sb_text[160] = {0};
        printf(ANSI_CYAN "\r\n[DIAG] ===== SB-ESP SEND MENU =====\r\n" ANSI_RESET);
        printf("[DIAG] Type text and press ENTER to send over UART4\r\n");
        printf("[DIAG] Press ESC to cancel and return\r\n");
        printf("[DIAG] Example: PING or SALT:25,BRINE:75\r\n\r\n");
        if (!Console_ReadLineBlocking("[DIAG] Enter SB-ESP text to send (ESC to cancel): ", sb_text, sizeof(sb_text))) {
            Console_ShowTestMenu();
            return;
        }
        if (sb_text[0] == '\0') {
            printf("[DIAG] Empty input, nothing sent\r\n");
            Console_ShowTestMenu();
            return;
        }

        const char *before = Dispersion_GetLastStatus();
        char before_buf[64] = {0};
        if (before) {
            strncpy(before_buf, before, sizeof(before_buf) - 1);
            before_buf[sizeof(before_buf) - 1] = '\0';
        }

        Dispersion_SetTestResponseMode(1);
        Dispersion_SendRaw(sb_text);
        HAL_Delay(250);
        const char *after = Dispersion_GetLastStatus();
        if (after && strcmp(after, before_buf) != 0) {
            printf("[DIAG] SB-ESP RX: %s\r\n", after);
        } else {
            printf("[DIAG] SB-ESP: no new RX yet (use Q / TEST SBESPRX to monitor)\r\n");
        }
        Dispersion_SetTestResponseMode(0);
        Console_ShowTestMenu();
        return;
    }

    if (strcmp(cmd_upper, "Q") == 0)
    {
        if (s_test_mode_flag) *s_test_mode_flag = 1;
        extern UART_HandleTypeDef huart2;
        char last_seen[64] = {0};
        Dispersion_SetTestResponseMode(1);
        printf(ANSI_CYAN "[DIAG] SB-ESP RX monitor active (press ESC to exit)\r\n" ANSI_RESET);
        while (1)
        {
            Console_WatchdogKick();
            if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
                uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
                if (ch == 0x1B) break;
            }
            if (Console_CheckEscPressed()) break;

            const char *msg = Dispersion_GetLastStatus();
            if (msg && msg[0] != '\0' && strcmp(msg, last_seen) != 0) {
                strncpy(last_seen, msg, sizeof(last_seen) - 1);
                last_seen[sizeof(last_seen) - 1] = '\0';
                printf("[DIAG] SB-ESP RX: %s\r\n", last_seen);
            }

            HAL_Delay(50);
        }
        Dispersion_SetTestResponseMode(0);
        printf("[DIAG] SB-ESP RX monitor stopped\r\n");
        Console_ShowTestMenu();
        return;
    }

    // ========== HELP COMMAND ==========
    if (strcmp(cmd_upper, "HELP") == 0 || strcmp(cmd_upper, "?") == 0)
    {
        printf(ANSI_CYAN "\r\n========== COMMAND HELP ==========\r\n" ANSI_RESET);
        printf("Quick Menu: 1, 2, 3, 4, 5, 6, 7, 8, 9, S, D, I, 0\r\n");
        printf("\r\n" ANSI_YELLOW "Direct Sabertooth Commands (no abstraction):\r\n" ANSI_RESET);
        printf("  M1: <value>                     - Motor 1 speed (-2047 to +2047)\r\n");
        printf("  M2: <value>                     - Motor 2 speed (-2047 to +2047)\r\n");
        printf("  Examples: M1: 1000   M2: -500   M2: 0\r\n");
        printf("\r\n" ANSI_YELLOW "Commands with Parameters:\r\n" ANSI_RESET);
        printf("  TEST MOTOR <motor> <speed>      - Motor 1 or 2, speed: -100 to +100 (percent)\r\n");
        printf("  TEST RAWCMD <text>              - Send raw Sabertooth text (e.g. M2: 500)\r\n");
        printf("  TEST MOTORSWEEP [delay]         - Sweep motors, delay in ms (default 100)\r\n");
        printf("  TEST SALT <percent>             - Salt pump, 0-100%% (default 50)\r\n");
        printf("  TEST SALTSWEEP [delay]          - Sweep salt pump, delay in ms (default 100)\r\n");
        printf("  TEST BRINE <percent>            - Brine pump, 0-100%% (default 50)\r\n");
        printf("  TEST BRINESWEEP [delay]         - Sweep brine pump, delay in ms (default 100)\r\n");
        printf("  TEST SBESP <text>               - Send raw Salt-Brine ESP32 string\r\n");
        printf("  TEST SBESPRX                    - Monitor Salt-Brine ESP32 messages\r\n");
        printf("  TEST LORA <text>                - Send raw LoRa string to ESP32\r\n");
        printf("  TEST LORARX                     - Monitor incoming LoRa messages\r\n");
        printf("  TEST LORACMDS                   - Send example LoRa control command set\r\n");
        printf("\r\n" ANSI_YELLOW "Other Commands:\r\n" ANSI_RESET);
        printf("  T                               - Show test menu\r\n");
        printf("  EXIT                            - Exit test mode\r\n");
        printf(ANSI_CYAN "=================================\r\n" ANSI_RESET "\r\n");
        return;
    }

    // ========== DIRECT SABERTOOTH COMMANDS ==========
    // Allow direct commands like "M1: 500" or "M2: -1000" without TEST prefix
    if ((strncmp(cmd_upper, "M1:", 3) == 0) || (strncmp(cmd_upper, "M2:", 3) == 0))
    {
        printf(ANSI_CYAN "[DIRECT] Sending to Sabertooth: %s\r\n" ANSI_RESET, cmd);
        Sabertooth_SendRawCommand(cmd);
        return;
    }

    // ========== TEST COMMANDS (Diagnostics) ==========
    if (strncmp(cmd_upper, "TEST", 4) == 0)
    {
        if (s_test_mode_flag) {
            *s_test_mode_flag = 1;
        }
        // Parse TEST subcommand: TEST <COMPONENT> [ARGS]
        const char *subcmd_up = cmd_upper + 4;
        const char *subcmd_orig = cmd + 4;
        while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
            subcmd_up++;
            subcmd_orig++;
        }

        // TEST MOTORSWEEP <delay_ms> (also accept "TEST MOTOR SWEEP")
        {
            const char *sweep_up = NULL;
            const char *sweep_orig = NULL;

            if (strncmp(subcmd_up, "MOTORSWEEP", 10) == 0) {
                sweep_up = subcmd_up + 10;
                sweep_orig = subcmd_orig + 10;
            } else if (strncmp(subcmd_up, "MOTOR", 5) == 0) {
                const char *tmp_up = subcmd_up + 5;
                const char *tmp_orig = subcmd_orig + 5;
                while (*tmp_up == ' ' && *tmp_orig == ' ') {
                    tmp_up++;
                    tmp_orig++;
                }
                if (strncmp(tmp_up, "SWEEP", 5) == 0) {
                    sweep_up = tmp_up + 5;
                    sweep_orig = tmp_orig + 5;
                }
            }

            if (sweep_orig) {
                while (*sweep_up == ' ' && *sweep_orig == ' ') {
                    sweep_up++;
                    sweep_orig++;
                }

                uint16_t delay_ms = 100; // Default 100ms
                sscanf(sweep_orig, "%hu", &delay_ms);

                Diag_TestMotorSweep(delay_ms);
                return;
            }
        }

        // TEST MOTOR <1|2> <speed>
        if (strncmp(subcmd_up, "MOTOR", 5) == 0)
        {
            subcmd_up += 5;
            subcmd_orig += 5;
            while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
                subcmd_up++;
                subcmd_orig++;
            }

            int motor = 0;
            int speed = 0;
            if (ParseTwoInts(subcmd_orig, &motor, &speed) && (motor == 1 || motor == 2))
                Diag_TestMotor((uint8_t)motor, speed);
            else
                printf("[DIAG] Usage: TEST MOTOR <1|2> <speed>\r\n");
            return;
        }

        // TEST RAWCMD <text> (send raw Sabertooth command)
        if (strncmp(subcmd_up, "RAWCMD", 6) == 0 || strncmp(subcmd_up, "SABER", 5) == 0)
        {
            if (strncmp(subcmd_up, "RAWCMD", 6) == 0) {
                subcmd_up += 6;
                subcmd_orig += 6;
            } else {
                subcmd_up += 5;
                subcmd_orig += 5;
            }
            while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
                subcmd_up++;
                subcmd_orig++;
            }

            if (*subcmd_orig == '\0') {
                printf("[DIAG] Usage: TEST RAWCMD <text>\r\n");
            } else {
                Sabertooth_SendRawCommand(subcmd_orig);
            }
            return;
        }


        // TEST SALT <rate_0_100>
        if (strncmp(subcmd_up, "SALT", 4) == 0)
        {
            subcmd_up += 4;
            subcmd_orig += 4;
            while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
                subcmd_up++;
                subcmd_orig++;
            }

            uint8_t rate = 0;
            sscanf(subcmd_orig, "%hhu", &rate);

            Diag_TestSaltRate(rate);
            return;
        }

        // TEST SALTSWEEP <delay_ms>
        if (strncmp(subcmd_up, "SALTSWEEP", 9) == 0)
        {
            subcmd_up += 9;
            subcmd_orig += 9;
            while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
                subcmd_up++;
                subcmd_orig++;
            }

            uint16_t delay_ms = 100; // Default 100ms
            sscanf(subcmd_orig, "%hu", &delay_ms);

            Diag_TestSaltSweep(delay_ms);
            return;
        }

        // TEST BRINE <rate_0_100>
        if (strncmp(subcmd_up, "BRINE", 5) == 0)
        {
            subcmd_up += 5;
            subcmd_orig += 5;
            while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
                subcmd_up++;
                subcmd_orig++;
            }

            uint8_t rate = 0;
            sscanf(subcmd_orig, "%hhu", &rate);

            Diag_TestBrineRate(rate);
            return;
        }

        // TEST BRINESWEEP <delay_ms>
        if (strncmp(subcmd_up, "BRINESWEEP", 10) == 0)
        {
            subcmd_up += 10;
            subcmd_orig += 10;
            while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
                subcmd_up++;
                subcmd_orig++;
            }

            uint16_t delay_ms = 100; // Default 100ms
            sscanf(subcmd_orig, "%hu", &delay_ms);

            Diag_TestBrineSweep(delay_ms);
            return;
        }

        // TEST GPS
        if (strcmp(subcmd_up, "GPS") == 0)
        {
            Diag_TestGPS();
            return;
        }

        // TEST SBESP <text> (accept legacy alias TEST DISP <text>)
        if (strncmp(subcmd_up, "SBESP", 5) == 0 || strncmp(subcmd_up, "DISP", 4) == 0)
        {
            if (strncmp(subcmd_up, "SBESP", 5) == 0) {
                subcmd_up += 5;
                subcmd_orig += 5;
            } else {
                subcmd_up += 4;
                subcmd_orig += 4;
            }
            while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
                subcmd_up++;
                subcmd_orig++;
            }

            if (*subcmd_orig == '\0') {
                printf("[DIAG] Usage: TEST SBESP <text>\r\n");
            } else {
                const char *before = Dispersion_GetLastStatus();
                char before_buf[64] = {0};
                if (before) {
                    strncpy(before_buf, before, sizeof(before_buf) - 1);
                    before_buf[sizeof(before_buf) - 1] = '\0';
                }

                Dispersion_SetTestResponseMode(1);
                Dispersion_SendRaw(subcmd_orig);
                HAL_Delay(250);
                const char *after = Dispersion_GetLastStatus();
                if (after && strcmp(after, before_buf) != 0) {
                    printf("[DIAG] SB-ESP RX: %s\r\n", after);
                } else {
                    printf("[DIAG] SB-ESP: no new RX yet (use TEST SBESPRX to monitor)\r\n");
                }
                Dispersion_SetTestResponseMode(0);
            }
            return;
        }

        // TEST SBESPRX (accept legacy alias TEST DISPRX)
        if (strcmp(subcmd_up, "SBESPRX") == 0 || strcmp(subcmd_up, "DISPRX") == 0)
        {
            extern UART_HandleTypeDef huart2;
            char last_seen[64] = {0};
            Dispersion_SetTestResponseMode(1);
            printf(ANSI_CYAN "[DIAG] SB-ESP RX monitor active (press ESC to exit)\r\n" ANSI_RESET);
            while (1)
            {
                Console_WatchdogKick();
                if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
                    uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
                    if (ch == 0x1B) break;
                }
                if (Console_CheckEscPressed()) break;

                const char *msg = Dispersion_GetLastStatus();
                if (msg && msg[0] != '\0' && strcmp(msg, last_seen) != 0) {
                    strncpy(last_seen, msg, sizeof(last_seen) - 1);
                    last_seen[sizeof(last_seen) - 1] = '\0';
                    printf("[DIAG] SB-ESP RX: %s\r\n", last_seen);
                }

                HAL_Delay(50);
            }
            Dispersion_SetTestResponseMode(0);
            printf("[DIAG] SB-ESP RX monitor stopped\r\n");
            Console_ShowTestMenu();
            return;
        }

        // TEST LORACMDS
        if (strcmp(subcmd_up, "LORACMDS") == 0)
        {
            Console_SendLoRaExampleCommands();
            return;
        }

        // TEST LORA <text>
        if (strncmp(subcmd_up, "LORA", 4) == 0)
        {
            subcmd_up += 4;
            subcmd_orig += 4;
            while (*subcmd_up == ' ' && *subcmd_orig == ' ') {
                subcmd_up++;
                subcmd_orig++;
            }

            if (*subcmd_orig == '\0') {
                printf("[DIAG] Usage: TEST LORA <text>\r\n");
            } else {
                uint32_t tx_before = LoRA_GetTxCount();
                LoRA_SendRaw(subcmd_orig);
                uint32_t tx_after = LoRA_GetTxCount();
                if (tx_after > tx_before) {
                    printf("[DIAG] LoRa TX OK: %s\r\n", subcmd_orig);
                } else {
                    printf("[DIAG] LoRa TX FAILED (UART5 not ready?)\r\n");
                }
            }
            return;
        }

        // TEST LORARX
        if (strcmp(subcmd_up, "LORARX") == 0)
        {
            extern UART_HandleTypeDef huart2;
            uint32_t last_seen_count = LoRA_GetRawFrameCount();
            uint32_t filtered_count = 0;
            printf(ANSI_CYAN "[DIAG] LoRa RX monitor active (press ESC to exit)\r\n" ANSI_RESET);
            while (1)
            {
                Console_WatchdogKick();
                if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
                    uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
                    if (ch == 0x1B) break;
                }
                if (Console_CheckEscPressed()) break;

                uint32_t current_count = LoRA_GetRawFrameCount();
                if (current_count != last_seen_count) {
                    last_seen_count = current_count;
                    const char *raw = LoRA_GetLastRawFrame();
                    if (raw && raw[0] != '\0') {
                        if (!Console_IsMostlyPrintableAscii(raw) || Console_IsLikelySbEspFrame(raw)) {
                            filtered_count++;
                        } else {
                            printf("[DIAG] LoRa RX: %s\r\n", raw);
                        }
                    }
                }

                HAL_Delay(50);
            }
            if (filtered_count > 0) {
                printf("[DIAG] LoRa RX filtered %lu SB-ESP-like frame(s)\r\n", filtered_count);
            }
            printf("[DIAG] LoRa RX monitor stopped\r\n");
            Console_ShowTestMenu();
            return;
        }

        // TEST IMU
        if (strcmp(subcmd_up, "IMU") == 0)
        {
            Diag_TestIMU();
            return;
        }

        // TEST IMUDETAIL - Detailed IMU communication check
        if (strcmp(subcmd_up, "IMUDETAIL") == 0)
        {
            Diag_TestIMUDetailed();
            return;
        }

        // TEST I2C - Scan I2C bus
        if (strcmp(subcmd_up, "I2C") == 0)
        {
            Diag_ScanI2C();
            return;
        }

        // TEST TEMP - Read temperature sensor continuously
        if (strcmp(subcmd_up, "TEMP") == 0)
        {
            extern UART_HandleTypeDef huart2;
            printf(ANSI_YELLOW "[DIAG] ========== IMU TEMPERATURE TEST ==========\r\n" ANSI_RESET);
            printf(ANSI_CYAN "[DIAG] Reading temperature sensor (press ESC to exit)...\r\n\r\n" ANSI_RESET);
            
            while (1)
            {
                Console_WatchdogKick();
                if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
                    uint8_t ch = (uint8_t)(huart2.Instance->DR & 0xFF);
                    if (ch == 0x1B) {
                        break;
                    }
                }
                if (Console_CheckEscPressed()) {
                    break;
                }

                IMU_Status_t imu = IMU_Read();
                if (imu.ok)
                {
                    // Color code temperature: Blue (cold), Green (normal), Yellow (warm), Red (hot)
                    const char *temp_color = ANSI_BLUE;
                    if (imu.temperature_c > 25.0f) temp_color = ANSI_GREEN;
                    if (imu.temperature_c > 40.0f) temp_color = ANSI_YELLOW;
                    if (imu.temperature_c > 60.0f) temp_color = ANSI_RED;
                    
                    printf("\r[DIAG] Temp: %s%.2f°C" ANSI_RESET "    ", temp_color, imu.temperature_c);
                }
                else
                {
                    printf("\r" ANSI_RED "[DIAG] ERROR: IMU read failed" ANSI_RESET "    ");
                }
                fflush(stdout);
                HAL_Delay(1000);  // Update 1x per second
            }
            printf("\r\n" ANSI_GREEN "[DIAG] Temperature test complete\r\n" ANSI_RESET);
            Console_ShowTestMenu();
            return;
        }

        // TEST PROXIMITY
        if (strcmp(subcmd_up, "PROXIMITY") == 0)
        {
            Diag_TestProximity();
            return;
        }

        // TEST STATE
        if (strcmp(subcmd_up, "STATE") == 0)
        {
            Diag_PrintState();
            return;
        }

        // TEST SABERTOOTH MONITOR (live feedback polling)
        if (strncmp(subcmd_up, "SABERTOOTH", 10) == 0)
        {
            const char *extra_up = subcmd_up + 10;
            const char *extra_orig = subcmd_orig + 10;
            while (*extra_up == ' ' && *extra_orig == ' ') {
                extra_up++;
                extra_orig++;
            }
            if (strcmp(extra_up, "MONITOR") == 0)
            {
                Diag_MonitorSabertoothFeedback();
                return;
            }
        }

        // TEST ALL
        if (strcmp(subcmd_up, "ALL") == 0)
        {
            Diag_TestAll();
            return;
        }

        // Unknown test command
        printf("[DIAG] Unknown test command. Options:\r\n");
        printf("  TEST MOTOR <1|2> <speed>     - Single motor test (percent)\r\n");
        printf("  TEST RAWMOTOR <1|2> <value>  - Raw motor command (-2047 to +2047)\r\n");
        printf("  TEST RAWCMD <text>           - Send raw Sabertooth text (e.g. M2: 500)\r\n");
        printf("  TEST MOTORSWEEP [delay_ms]   - Sweep motors -100 to +100\r\n");
        printf("  TEST SALT <0-100>            - Single salt rate test\r\n");
        printf("  TEST SALTSWEEP [delay_ms]    - Sweep salt 0-100%%\r\n");
        printf("  TEST BRINE <0-100>           - Single brine rate test\r\n");
        printf("  TEST BRINESWEEP [delay_ms]   - Sweep brine 0-100%%\r\n");
        printf("  TEST SBESP <text>            - Send raw Salt-Brine ESP32 string (alias: TEST DISP)\r\n");
        printf("  TEST SBESPRX                 - Monitor Salt-Brine ESP32 messages (alias: TEST DISPRX)\r\n");
        printf("  TEST LORA <text>             - Send raw LoRa string to ESP32\r\n");
        printf("  TEST LORARX                  - Monitor incoming LoRa messages\r\n");
        printf("  TEST LORACMDS                - Send example LoRa control command set\r\n");
        printf("  TEST GPS                     - Read GPS data\r\n");
        printf("  TEST IMU                     - Read IMU data\r\n");
        printf("  TEST IMUDETAIL               - Detailed IMU I2C check\r\n");
        printf("  TEST I2C                     - Scan I2C bus for devices\r\n");
        printf("  TEST TEMP                    - Read IMU temperature sensor\r\n");
        printf("  TEST PROXIMITY               - Read proximity sensors\r\n");
        printf("  TEST STATE                   - Print state machine info\r\n");
        printf("  TEST ALL                     - Run all basic tests\r\n");
        return;
    }

    // ========== CONTROL COMMANDS ==========
    if (strcmp(cmd, "ESTOP") == 0) {
        RobotSM_Request(sm, STATE_ESTOP);
        return;
    }

    if (strcmp(cmd, "RESET") == 0) {
        RobotState_t cur = RobotSM_Current(sm);
        if (cur == STATE_ESTOP || cur == STATE_ERROR) {
            RobotSM_Request(sm, STATE_PAUSE);
        }
        return;
    }

    if (strcmp(cmd, "PAUSE") == 0) {
        RobotSM_Request(sm, STATE_PAUSE);
        return;
    }

    if (strcmp(cmd, "AUTO") == 0) {
        RobotSM_Request(sm, STATE_AUTO);
        return;
    }

    if (strcmp(cmd, "MANUAL") == 0) {
        RobotSM_Request(sm, STATE_MANUAL);
        return;
    }

    // Only parse speed commands in MANUAL
    if (RobotSM_Current(sm) == STATE_MANUAL) {
        ParseManualSpeedCommand(cmd);
        return;
    }

    // Optional: feedback when command is ignored
    // printf(ANSI_YELLOW "Unknown/ignored cmd: %s\r\n" ANSI_RESET, cmd);
}
