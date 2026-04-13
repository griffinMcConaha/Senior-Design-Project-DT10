#include "gps.h"
#include "system_health.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>

// GPS line buffer size (NMEA sentence max ~128 chars)
#ifndef GPS_LINE_MAX
#define GPS_LINE_MAX 128
#endif

// GPS data freshness timeout: mark stale if no update for this long
#ifndef GPS_STALE_MS
#define GPS_STALE_MS 5000u
#endif

#ifndef GPS_AUTO_BAUD_ENABLED
#define GPS_AUTO_BAUD_ENABLED 1
#endif

#define GPS_NMEA_SILENT_SWITCH_MS 3500u
#define GPS_BAUD_SWITCH_COOLDOWN_MS 4000u
#define GPS_UART_ACTIVE_MS 1200u
#define GPS_RECOVERY_LOG_THROTTLE_MS 10000u
#define GPS_AUTO_BAUD_LOG_THROTTLE_MS 15000u

// Module-level UART handle and receive buffer
static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_byte = 0;

// Sentence accumulator and index
static char s_line[GPS_LINE_MAX];
static volatile int s_idx = 0;

// Last RX timestamp and current GPS data
static volatile uint32_t s_last_rx_ms = 0;
static volatile uint32_t s_rx_byte_count = 0;  // total bytes received; 0 = UART not flowing
static volatile uint32_t s_last_nmea_ms = 0;
static volatile uint32_t s_last_baud_switch_ms = 0;
static volatile GPS_Data_t s_gps = {0};
static volatile uint8_t s_rmc_fix_valid = 0;
static volatile uint8_t s_gga_fix_valid = 0;
static volatile uint8_t s_gsa_fix_valid = 0;
static volatile uint32_t s_nmea_line_count = 0;
static volatile uint32_t s_nmea_unknown_count = 0;

static const uint32_t s_gps_baud_table[] = {9600u, 38400u, 57600u, 115200u};
static uint8_t s_gps_baud_index = 0;
static volatile uint8_t s_auto_baud_runtime_enabled = GPS_AUTO_BAUD_ENABLED ? 1u : 0u;
static uint32_t s_last_recovery_log_ms = 0;
static uint32_t s_last_auto_baud_log_ms = 0;

static void GPS_LogThrottled(uint32_t *last_log_ms, uint32_t throttle_ms, const char *fmt, ...)
{
    if (!last_log_ms || !fmt) {
        return;
    }

    uint32_t now_ms = HAL_GetTick();
    if (*last_log_ms != 0u && (now_ms - *last_log_ms) < throttle_ms) {
        return;
    }

    *last_log_ms = now_ms;

    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

static void GPS_UpdateFixState(void)
{
    s_gps.has_fix = (s_rmc_fix_valid || s_gga_fix_valid || s_gsa_fix_valid) ? 1u : 0u;
}

static void GPS_SwitchToBaud(uint32_t baud)
{
    if (!s_huart) return;

    HAL_UART_AbortReceive_IT(s_huart);
    __HAL_UART_CLEAR_OREFLAG(s_huart);
    (void)HAL_UART_DeInit(s_huart);

    s_huart->Init.BaudRate = baud;
    if (HAL_UART_Init(s_huart) == HAL_OK) {
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
        GPS_LogThrottled(&s_last_auto_baud_log_ms, GPS_AUTO_BAUD_LOG_THROTTLE_MS,
                         "[GPS] Auto-baud switched to %lu\r\n", (unsigned long)baud);
    } else {
        GPS_LogThrottled(&s_last_auto_baud_log_ms, GPS_AUTO_BAUD_LOG_THROTTLE_MS,
                         "[GPS] Auto-baud switch failed for %lu\r\n", (unsigned long)baud);
    }
}

static void GPS_AutoBaudTick(uint32_t now_ms)
{
#if GPS_AUTO_BAUD_ENABLED
    if (!s_auto_baud_runtime_enabled) {
        return;
    }

    if (!s_huart) return;

    // If valid NMEA lines are arriving recently, keep current baud.
    if (s_last_nmea_ms != 0u && (now_ms - s_last_nmea_ms) <= GPS_NMEA_SILENT_SWITCH_MS) {
        return;
    }

    if ((now_ms - s_last_baud_switch_ms) < GPS_BAUD_SWITCH_COOLDOWN_MS) {
        return;
    }

    // Switch baud if there is no UART activity yet, UART activity is stale,
    // or bytes are present but no valid NMEA has been parsed for a while.
    if (s_last_rx_ms != 0u && (now_ms - s_last_rx_ms) <= GPS_UART_ACTIVE_MS && s_last_nmea_ms != 0u) {
        return;
    }

    s_gps_baud_index = (uint8_t)((s_gps_baud_index + 1u) % (sizeof(s_gps_baud_table) / sizeof(s_gps_baud_table[0])));
    s_last_baud_switch_ms = now_ms;
    GPS_SwitchToBaud(s_gps_baud_table[s_gps_baud_index]);
#else
    (void)now_ms;
#endif
}

void GPS_SetAutoBaudEnabled(uint8_t enabled)
{
    s_auto_baud_runtime_enabled = enabled ? 1u : 0u;
}

uint8_t GPS_GetAutoBaudEnabled(void)
{
    return s_auto_baud_runtime_enabled;
}

// Convert NMEA degree-minute format (ddmm.mmmm) to decimal degrees
static double NMEA_DegMin_To_Deg(const char *field, char hemi)
{
    if (field == NULL || *field == 0) return 0.0;

    double val = atof(field);        // ddmm.mmmm
    int deg = (int)(val / 100);
    double minutes = val - deg * 100;
    double deg_dec = deg + minutes / 60.0;

    if (hemi == 'S' || hemi == 'W') deg_dec = -deg_dec;
    return deg_dec;
}

// Parse RMC sentence (Recommended Minimum Course and Ground Speed)
static void GPS_Parse_RMC(const char *sentence)
{
    // Accept any NMEA talker ID with RMC payload, e.g. $GPRMC, $GNRMC.
    if (!sentence || strlen(sentence) < 6 || sentence[0] != '$' || strncmp(sentence + 3, "RMC", 3) != 0)
        return;

    char buf[GPS_LINE_MAX];
    strncpy(buf, sentence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;

    char *token = NULL;
    int field = 0;

    char status = 'V';
    const char *lat_field = NULL;
    char lat_hemi = 'N';
    const char *lon_field = NULL;
    char lon_hemi = 'E';
    const char *speed_field = NULL;
    const char *course_field = NULL;

    token = strtok(buf, ",");
    while (token != NULL)
    {
        switch (field)
        {
            case 2: status = token[0]; break;         // A/V (Active/Void)
            case 3: lat_field = token; break;         // ddmm.mmmm
            case 4: lat_hemi = token[0]; break;       // N/S
            case 5: lon_field = token; break;         // dddmm.mmmm
            case 6: lon_hemi = token[0]; break;       // E/W
            case 7: speed_field = token; break;       // knots
            case 8:
                course_field = (token[0] != 0) ? token : NULL;
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    s_rmc_fix_valid = (status == 'A') ? 1u : 0u;
    GPS_UpdateFixState();

    // Update position, speed, and course (preserve hdop, vdop, altitude, sat count from GGA/GSA)
    if (lat_field && *lat_field)
        s_gps.latitude_deg = (float)NMEA_DegMin_To_Deg(lat_field, lat_hemi);

    if (lon_field && *lon_field)
        s_gps.longitude_deg = (float)NMEA_DegMin_To_Deg(lon_field, lon_hemi);

    if (speed_field && *speed_field)
        s_gps.speed_knots = (float)atof(speed_field);

    if (course_field && *course_field)
    {
        float c = (float)atof(course_field);
        if (c >= 0.0f && c <= 360.0f) s_gps.course_deg = c;
    }
}

// Parse GGA sentence (Global Positioning System Fix Data)
// Extracts: quality, num_satellites, HDOP, altitude
static void GPS_Parse_GGA(const char *sentence)
{
    // Accept any NMEA talker ID with GGA payload, e.g. $GPGGA, $GNGGA.
    if (!sentence || strlen(sentence) < 6 || sentence[0] != '$' || strncmp(sentence + 3, "GGA", 3) != 0)
        return;

    char buf[GPS_LINE_MAX];
    strncpy(buf, sentence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;

    char *token = NULL;
    int field = 0;

    uint8_t quality = 0;
    const char *lat_field = NULL;
    char lat_hemi = 'N';
    const char *lon_field = NULL;
    char lon_hemi = 'E';
    uint8_t num_sat = 0;
    float hdop = 10.0f;  // worst case
    float alt = 0.0f;

    token = strtok(buf, ",");
    while (token != NULL)
    {
        switch (field)
        {
            case 2:
                lat_field = token;
                break;
            case 3:
                if (*token) lat_hemi = token[0];
                break;
            case 4:
                lon_field = token;
                break;
            case 5:
                if (*token) lon_hemi = token[0];
                break;
            case 6:
                if (*token) quality = (uint8_t)atoi(token);
                break;
            case 7:
                // Number of satellites in use
                if (*token) num_sat = (uint8_t)atoi(token);
                break;
            case 8:
                // HDOP (Horizontal Dilution of Precision)
                if (*token) hdop = (float)atof(token);
                break;
            case 9:
                // Altitude above mean sea level
                if (*token) alt = (float)atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    // Update GPS data
    if (lat_field && *lat_field) {
        s_gps.latitude_deg = (float)NMEA_DegMin_To_Deg(lat_field, lat_hemi);
    }

    if (lon_field && *lon_field) {
        s_gps.longitude_deg = (float)NMEA_DegMin_To_Deg(lon_field, lon_hemi);
    }

    s_gps.num_satellites = num_sat;
    s_gps.hdop = hdop;
    s_gps.altitude_m = alt;
    // Some receivers briefly report quality>0 while satellites field is not yet stable.
    // Treat quality as the primary fix signal so we don't miss valid locks.
    s_gga_fix_valid = (quality > 0u) ? 1u : 0u;
    GPS_UpdateFixState();
}

// Parse GSA sentence (DOP and active satellites)
// Extracts: VDOP (vertical dilution of precision)
static void GPS_Parse_GSA(const char *sentence)
{
    // Accept any NMEA talker ID with GSA payload, e.g. $GPGSA, $GNGSA.
    if (!sentence || strlen(sentence) < 6 || sentence[0] != '$' || strncmp(sentence + 3, "GSA", 3) != 0)
        return;

    char buf[GPS_LINE_MAX];
    strncpy(buf, sentence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;

    char *token = NULL;
    int field = 0;
    uint8_t fix_type = 1; // 1=no fix, 2=2D, 3=3D
    float vdop = 10.0f;

    token = strtok(buf, ",");
    while (token != NULL)
    {
        switch (field)
        {
            case 2:
                if (*token) fix_type = (uint8_t)atoi(token);
                break;
            case 17:
                // VDOP (Vertical Dilution of Precision)
                if (*token) vdop = (float)atof(token);
                break;
            default:
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    s_gps.vdop = vdop;
    s_gsa_fix_valid = (fix_type >= 2u) ? 1u : 0u;
    GPS_UpdateFixState();
}

// Initialize GPS parser with UART handle and enable RX interrupt
void GPS_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_idx = 0;
    s_line[0] = 0;
    s_last_rx_ms = 0;
    s_rx_byte_count = 0;
    s_last_nmea_ms = 0;
    s_last_baud_switch_ms = HAL_GetTick();
    s_gps.has_fix = 0;
    s_rmc_fix_valid = 0;
    s_gga_fix_valid = 0;
    s_gsa_fix_valid = 0;
    s_nmea_line_count = 0;
    s_nmea_unknown_count = 0;

    if (s_huart) {
        uint32_t configured_baud = s_huart->Init.BaudRate;
        for (uint8_t i = 0; i < (sizeof(s_gps_baud_table) / sizeof(s_gps_baud_table[0])); i++) {
            if (s_gps_baud_table[i] == configured_baud) {
                s_gps_baud_index = i;
                break;
            }
        }
    }

    // Start RX interrupt for 1 byte
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

uint32_t GPS_GetRxByteCount(void)
{
    return s_rx_byte_count;
}

void GPS_RxByte(uint8_t b)
{
	s_last_rx_ms = HAL_GetTick();   // mark activity for freshness
    s_rx_byte_count++;
    if (b == '\n' || b == '\r')
    {
        if (s_idx > 0)
        {
            s_line[s_idx] = 0;
            if (s_line[0] == '$' && strlen(s_line) >= 6u) {
                s_last_nmea_ms = s_last_rx_ms;
                s_nmea_line_count++;

                if (strncmp(s_line + 3, "RMC", 3) == 0) {
                    GPS_Parse_RMC(s_line);
                } else if (strncmp(s_line + 3, "GGA", 3) == 0) {
                    GPS_Parse_GGA(s_line);
                } else if (strncmp(s_line + 3, "GSA", 3) == 0) {
                    GPS_Parse_GSA(s_line);
                } else {
                    s_nmea_unknown_count++;
                }
            }
        }
        s_idx = 0;
    }
    else
    {
        if (s_idx < (GPS_LINE_MAX - 1))
            s_line[s_idx++] = (char)b;
        else
            s_idx = 0; // overflow -> reset line
    }
}

void GPS_Tick(uint32_t now_ms)
{
    // If stale, drop fix (this mirrors what you were doing in main)
    if (s_last_rx_ms != 0 && (now_ms - s_last_rx_ms) > GPS_STALE_MS) {
        s_rmc_fix_valid = 0;
        s_gga_fix_valid = 0;
        s_gsa_fix_valid = 0;
        s_gps.has_fix = 0;
        SystemHealth_SetSensorStatus(SENSOR_GPS, SENSOR_TIMEOUT);
    } else if (s_gps.has_fix) {
        // GPS is locked and receiving data
        SystemHealth_SetSensorStatus(SENSOR_GPS, SENSOR_OK);
    } else if (s_last_rx_ms != 0u) {
        // GPS UART is alive but there is no fix yet: degraded, not timeout.
        SystemHealth_SetSensorStatus(SENSOR_GPS, SENSOR_DEGRADED);
    } else {
        // GPS is running but no fix yet
        SystemHealth_SetSensorStatus(SENSOR_GPS, SENSOR_TIMEOUT);
    }

    GPS_AutoBaudTick(now_ms);
}

const GPS_Data_t* GPS_Get(void)
{
    return (const GPS_Data_t*)&s_gps;
}

uint32_t GPS_GetLastRxMs(void)
{
    return s_last_rx_ms;
}

// Calculate GPS quality score (0.0 = no fix, 1.0 = excellent)
// Based on HDOP: 1.0-2.0=excellent, 2.0-5.0=good, 5.0-10.0=fair, >10.0=poor
float GPS_GetQuality(void)
{
    if (!s_gps.has_fix) return 0.0f;
    
    // Quality metric: inverse of HDOP (scaled)
    // HDOP 1.0 -> quality 1.0
    // HDOP 2.0 -> quality 0.9
    // HDOP 5.0 -> quality 0.7
    // HDOP 10.0 -> quality 0.4
    float quality = 1.0f / (1.0f + (s_gps.hdop - 1.0f) * 0.1f);
    if (quality < 0.0f) quality = 0.0f;
    if (quality > 1.0f) quality = 1.0f;
    return quality;
}

// Hook this from HAL_UART_RxCpltCallback
void GPS_HAL_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL) return;
    if (huart->Instance != s_huart->Instance) return;

    GPS_RxByte(s_rx_byte);
    s_last_rx_ms = HAL_GetTick();

    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

// Reset GPS parser (clear buffers and fix state) for recovery
void GPS_Reset(void)
{
    s_idx = 0;
    s_line[0] = 0;
    s_gps.has_fix = 0;
    s_rmc_fix_valid = 0;
    s_gga_fix_valid = 0;
    s_gsa_fix_valid = 0;
    GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                     "[GPS] Parser reset\r\n");
}

// Get GPS receiver status: 1 if healthy, 0 if not responding
uint8_t GPS_IsHealthy(void)
{
    // Healthy if we're receiving data fairly recently
    // Even without a fix, receiving data is good
    uint32_t now_ms = HAL_GetTick();
    if (s_last_rx_ms == 0) return 0;  // Never received anything
    
    uint32_t age_ms = now_ms - s_last_rx_ms;
    return (age_ms < 10000u) ? 1 : 0;  // Not responsive if no data for 10 seconds
}

// Attempt recovery: reset parser and restart RX if possible
uint8_t GPS_CheckAndRecover(void)
{
    if (s_huart == NULL) {
        GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                         "[GPS] Recovery FAILED: UART not initialized\r\n");
        return 0;
    }
    
    // Check if receiver is dead (no data for >10 seconds)
    if (GPS_IsHealthy()) {
        // Already healthy, no recovery needed
        return 1;
    }
    
    GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                     "[GPS] Recovery: Parser stalled, resetting...\r\n");
    GPS_Reset();
    __HAL_UART_CLEAR_OREFLAG(s_huart);

    // If bytes have never flowed (or have been stale for a long time),
    // advance baud before re-arming RX to avoid being stuck at a wrong baud.
    uint32_t now_ms = HAL_GetTick();
    if (s_rx_byte_count == 0u || (s_last_rx_ms != 0u && (now_ms - s_last_rx_ms) > 15000u)) {
        s_gps_baud_index = (uint8_t)((s_gps_baud_index + 1u) % (sizeof(s_gps_baud_table) / sizeof(s_gps_baud_table[0])));
        s_last_baud_switch_ms = now_ms;
        GPS_SwitchToBaud(s_gps_baud_table[s_gps_baud_index]);
    }

    // Re-enable RX interrupt. HAL_BUSY here usually means RX is already armed.
    HAL_StatusTypeDef status = HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
    if (status == HAL_OK || status == HAL_BUSY) {
        if (status == HAL_OK) {
            GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                             "[GPS] Recovery: RX interrupt re-enabled\r\n");
        } else {
            GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                             "[GPS] Recovery: RX already armed\r\n");
        }
        return 1;
    } else {
        GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                         "[GPS] Recovery: RX re-arm failed (%d), reinitializing UART...\r\n", status);
        (void)HAL_UART_DeInit(s_huart);
        if (HAL_UART_Init(s_huart) == HAL_OK) {
            HAL_StatusTypeDef retry = HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
            if (retry == HAL_OK || retry == HAL_BUSY) {
                GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                                 "[GPS] Recovery: UART reinitialized\r\n");
                return 1;
            }
            GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                             "[GPS] Recovery FAILED: UART reinit done but RX re-arm failed (%d)\r\n", retry);
            return 0;
        }
        GPS_LogThrottled(&s_last_recovery_log_ms, GPS_RECOVERY_LOG_THROTTLE_MS,
                         "[GPS] Recovery FAILED: UART reinit failed\r\n");
        return 0;
    }
}

