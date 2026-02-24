#include "gps.h"
#include "system_health.h"
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

// Module-level UART handle and receive buffer
static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_byte = 0;

// Sentence accumulator and index
static char s_line[GPS_LINE_MAX];
static volatile int s_idx = 0;

// Last RX timestamp and current GPS data
static volatile uint32_t s_last_rx_ms = 0;
static volatile GPS_Data_t s_gps = {0};

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
    // Expect "$GNRMC" or "$GPRMC"
    if (strncmp(sentence, "$GNRMC", 6) != 0 &&
        strncmp(sentence, "$GPRMC", 6) != 0)
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

    // Update fix status
    s_gps.has_fix = (status == 'A') ? 1 : 0;

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
    // Expect "$GNGGA" or "$GPGGA"
    if (strncmp(sentence, "$GNGGA", 6) != 0 &&
        strncmp(sentence, "$GPGGA", 6) != 0)
        return;

    char buf[GPS_LINE_MAX];
    strncpy(buf, sentence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;

    char *token = NULL;
    int field = 0;

    uint8_t num_sat = 0;
    float hdop = 10.0f;  // worst case
    float alt = 0.0f;

    token = strtok(buf, ",");
    while (token != NULL)
    {
        switch (field)
        {
            case 6:
                // Quality: 0=invalid, 1=GPS fix, 2=DGPS fix, 6=dead reckoning
                // (has_fix already set by RMC parser)
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
    s_gps.num_satellites = num_sat;
    s_gps.hdop = hdop;
    s_gps.altitude_m = alt;
    // Quality already set in RMC (has_fix), but GGA provides redundant confirmation
}

// Parse GSA sentence (DOP and active satellites)
// Extracts: VDOP (vertical dilution of precision)
static void GPS_Parse_GSA(const char *sentence)
{
    // Expect "$GNGSA" or "$GPGSA"
    if (strncmp(sentence, "$GNGSA", 6) != 0 &&
        strncmp(sentence, "$GPGSA", 6) != 0)
        return;

    char buf[GPS_LINE_MAX];
    strncpy(buf, sentence, sizeof(buf)-1);
    buf[sizeof(buf)-1] = 0;

    char *token = NULL;
    int field = 0;
    float vdop = 10.0f;

    token = strtok(buf, ",");
    while (token != NULL)
    {
        switch (field)
        {
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
}

// Initialize GPS parser with UART handle and enable RX interrupt
void GPS_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    s_idx = 0;
    s_line[0] = 0;
    s_last_rx_ms = 0;
    s_gps.has_fix = 0;

    // Start RX interrupt for 1 byte
    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void GPS_RxByte(uint8_t b)
{
	s_last_rx_ms = HAL_GetTick();   // mark activity for freshness
    if (b == '\n' || b == '\r')
    {
        if (s_idx > 0)
        {
            s_line[s_idx] = 0;
            GPS_Parse_RMC(s_line);
            GPS_Parse_GGA(s_line);
            GPS_Parse_GSA(s_line);
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
        s_gps.has_fix = 0;
        SystemHealth_SetSensorStatus(SENSOR_GPS, SENSOR_TIMEOUT);
    } else if (s_gps.has_fix) {
        // GPS is locked and receiving data
        SystemHealth_SetSensorStatus(SENSOR_GPS, SENSOR_OK);
    } else {
        // GPS is running but no fix yet
        SystemHealth_SetSensorStatus(SENSOR_GPS, SENSOR_TIMEOUT);
    }
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
