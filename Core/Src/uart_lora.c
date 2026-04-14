#include "uart_lora.h"
#include "robot_sm.h"
#include "system_health.h"
#include "lora_contracts.h"
#include "mission.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

// ============================================================================
// LoRA UART COMMUNICATION MODULE (UART 5)
// Receives commands from mobile app via LoRA ESP32
// Sends diagnostic data back to base station
// ============================================================================

#define LORA_RX_BUFFER_SIZE 256
#define LORA_COMMAND_TIMEOUT_MS 2500
#define LORA_STREAM_BUFFER_SIZE 512
#define LORA_ISR_QUEUE_SIZE 512
#define LORA_TX_DIAGNOSTICS_ENABLED 1
#define LORA_UART_TX_TIMEOUT_MS 20
#define LORA_TX_FAIL_TIMEOUT_STREAK 5u
#define LORA_HEALTH_INACTIVE_MS 45000u
#define LORA_HEALTH_STALE_TICKS_TO_TIMEOUT 5u
#define LORA_HEALTH_FRESH_TICKS_TO_RECOVER 2u

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];
    uint16_t rx_index;
    uint8_t raw_buffer[LORA_RX_BUFFER_SIZE];
    uint16_t raw_index;
    char last_raw_frame[LORA_RX_BUFFER_SIZE];
    uint32_t raw_frame_count;
    uint32_t last_rx_ms;
    uint32_t last_tx_ms;  // Track last successful transmission for health monitoring
    uint32_t rx_count;
    uint32_t tx_count;
    char last_command[128];
    char last_tx_payload[320];
    uint8_t pending_state_request; // 0=none, else state value
    uint8_t pending_reset_request;
    LoRA_ManualCommand_t pending_manual_cmd;
    uint8_t manual_command_valid;
    int8_t manual_drive_pct;
    int8_t manual_turn_pct;
    uint32_t manual_seq;
    uint8_t manual_seq_valid;
    uint8_t command_valid;
    uint32_t invalid_command_count;
    uint32_t start_filter_drop_count;
    uint32_t prefix_reject_count;
    uint32_t rx_overflow_count;
    uint8_t stream_seq_init;
    uint32_t stream_expected_seq;
    uint32_t stream_gap_count;
    char stream_buffer[LORA_STREAM_BUFFER_SIZE];
    uint16_t stream_len;
    uint8_t drop_until_eol;
    volatile uint16_t isr_q_head;
    volatile uint16_t isr_q_tail;
    uint8_t isr_q[LORA_ISR_QUEUE_SIZE];
    uint32_t isr_q_overflow_count;
    uint8_t tx_fail_streak;
    uint8_t health_stale_ticks;
    uint8_t health_fresh_ticks;
} LoRA_State_t;

static LoRA_State_t lora_state = {0};
static uint8_t s_lora_verbose = 0;

static uint8_t lora_isr_queue_push(uint8_t byte)
{
    uint16_t head = lora_state.isr_q_head;
    uint16_t next = (uint16_t)((head + 1u) % LORA_ISR_QUEUE_SIZE);
    if (next == lora_state.isr_q_tail) {
        lora_state.isr_q_overflow_count++;
        return 0;
    }

    lora_state.isr_q[head] = byte;
    lora_state.isr_q_head = next;
    return 1;
}

static uint8_t lora_isr_queue_pop(uint8_t *out_byte)
{
    if (!out_byte) return 0;

    uint16_t tail = lora_state.isr_q_tail;
    if (tail == lora_state.isr_q_head) {
        return 0;
    }

    *out_byte = lora_state.isr_q[tail];
    lora_state.isr_q_tail = (uint16_t)((tail + 1u) % LORA_ISR_QUEUE_SIZE);
    return 1;
}

static uint8_t lora_is_valid_start_byte(uint8_t byte, uint8_t upper)
{
    // Accept common command starts used by server/app/base-station payloads.
    // This keeps DRIVE/JOY/FORWARD style manual commands from being dropped.
    return (uint8_t)(byte == '{' ||
                     upper == 'C' || upper == 'S' || upper == 'A' || upper == 'M' ||
                     upper == 'P' || upper == 'E' || upper == 'W' || upper == 'D' ||
                     upper == 'F' || upper == 'B' || upper == 'L' || upper == 'R' ||
                     upper == 'J' || upper == 'T');
}

static uint8_t lora_cmd_prefix_rejects_byte(uint16_t rx_index, uint8_t first_char, uint8_t upper_byte, uint8_t byte)
{
    if ((uint8_t)toupper((unsigned char)first_char) != 'C') {
        return 0;
    }

    if (rx_index == 1) {
        return (uint8_t)(upper_byte != 'M');
    }
    if (rx_index == 2) {
        return (uint8_t)(upper_byte != 'D');
    }
    if (rx_index == 3) {
        return (uint8_t)(byte != ':');
    }

    return 0;
}

static const char* lora_state_name(uint8_t state)
{
    switch (state)
    {
        case 0: return "MANUAL";
        case 1: return "AUTO";
        case 2: return "PAUSE";
        case 3: return "ERROR";
        case 4: return "ESTOP";
        default: return "UNKNOWN";
    }
}

static const char* lora_manual_cmd_name(LoRA_ManualCommand_t cmd)
{
    switch (cmd)
    {
        case LORA_MANUAL_CMD_DRIVE: return "DRIVE";
        case LORA_MANUAL_CMD_FORWARD: return "FORWARD";
        case LORA_MANUAL_CMD_BACK: return "BACKWARD";
        case LORA_MANUAL_CMD_LEFT: return "LEFT";
        case LORA_MANUAL_CMD_RIGHT: return "RIGHT";
        case LORA_MANUAL_CMD_STOP: return "STOP";
        case LORA_MANUAL_CMD_ALL_ON: return "ALLON";
        case LORA_MANUAL_CMD_AGITATOR_ON: return "AGITATOR_ON";
        case LORA_MANUAL_CMD_AGITATOR_OFF: return "AGITATOR_OFF";
        case LORA_MANUAL_CMD_THROWER_ON: return "THROWER_ON";
        case LORA_MANUAL_CMD_THROWER_OFF: return "THROWER_OFF";
        case LORA_MANUAL_CMD_RELAY_ON: return "RELAY_ON";
        case LORA_MANUAL_CMD_RELAY_OFF: return "RELAY_OFF";
        case LORA_MANUAL_CMD_VIBRATION_ON: return "VIBRATION_ON";
        case LORA_MANUAL_CMD_VIBRATION_OFF: return "VIBRATION_OFF";
        case LORA_MANUAL_CMD_TEST_SALT: return "TEST_SALT";
        case LORA_MANUAL_CMD_TEST_BRINE: return "TEST_BRINE";
        case LORA_MANUAL_CMD_NONE:
        default:
            return "UNKNOWN";
    }
}

static void lora_send_received_ack_for_state(uint8_t requested_state)
{
    char ack_msg[48];
    snprintf(ack_msg, sizeof(ack_msg), "ACK:STATE:%s", lora_state_name(requested_state));
    LoRA_SendRaw(ack_msg);
}

static void lora_send_received_ack_for_manual(LoRA_ManualCommand_t cmd)
{
    char ack_msg[48];

    if (cmd == LORA_MANUAL_CMD_DRIVE && lora_state.manual_seq_valid) {
        snprintf(ack_msg, sizeof(ack_msg), "ACK:S:%lu", (unsigned long)lora_state.manual_seq);
    } else {
        snprintf(ack_msg, sizeof(ack_msg), "ACK:CMD:%s", lora_manual_cmd_name(cmd));
    }

    LoRA_SendRaw(ack_msg);
}

static int8_t lora_clamp_percent(int value)
{
    if (value > 100) return 100;
    if (value < -100) return -100;
    return (int8_t)value;
}

static uint8_t lora_parse_percent_field(const char *payload, const char *key, int8_t *out_value)
{
    if (!payload || !key || !out_value) return 0;

    char pattern[24];
    snprintf(pattern, sizeof(pattern), "%s:", key);
    const char *field = strstr(payload, pattern);
    if (!field) return 0;

    field += strlen(pattern);
    char *endptr = NULL;
    long parsed = strtol(field, &endptr, 10);
    if (endptr == field) return 0;

    *out_value = lora_clamp_percent((int)parsed);
    return 1;
}

static uint8_t lora_parse_compact_drive(const char *payload, int8_t *out_drive, int8_t *out_turn)
{
    if (!payload || !out_drive || !out_turn) return 0;
    if (!(payload[0] == 'D' && payload[1] == ':')) return 0;

    char *endptr = NULL;
    long drive = strtol(payload + 2, &endptr, 10);
    if (endptr == (payload + 2) || !endptr || *endptr != ',') return 0;

    long turn = strtol(endptr + 1, &endptr, 10);
    if (!endptr) return 0;

    while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
        endptr++;
    }

    if (*endptr == ',') {
        endptr++;
        while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
            endptr++;
        }

        if (*endptr == 'S') {
            endptr++;
            while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
                endptr++;
            }
            if (*endptr != ':') return 0;
            endptr++;

            char *seq_endptr = NULL;
            (void)strtol(endptr, &seq_endptr, 10);
            if (!seq_endptr || seq_endptr == endptr) return 0;
            endptr = seq_endptr;
        } else {
            return 0;
        }
    }

    while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
        endptr++;
    }
    if (*endptr != '\0') return 0;

    *out_drive = lora_clamp_percent((int)drive);
    *out_turn = lora_clamp_percent((int)turn);
    return 1;
}

static uint8_t lora_parse_compact_manual_packet(const char *payload, uint32_t *out_seq, int8_t *out_drive, int8_t *out_turn)
{
    if (!payload || !out_seq || !out_drive || !out_turn) return 0;
    if (!(payload[0] == 'J' && payload[1] == ':')) return 0;

    char *endptr = NULL;
    unsigned long seq = strtoul(payload + 2, &endptr, 10);
    if (endptr == (payload + 2) || !endptr || *endptr != ',') return 0;

    long drive = strtol(endptr + 1, &endptr, 10);
    if (!endptr || *endptr != ',') return 0;

    long turn = strtol(endptr + 1, &endptr, 10);
    if (!endptr) return 0;

    while (*endptr != '\0' && isspace((unsigned char)*endptr)) {
        endptr++;
    }
    if (*endptr != '\0') return 0;

    *out_seq = (uint32_t)seq;
    *out_drive = lora_clamp_percent((int)drive);
    *out_turn = lora_clamp_percent((int)turn);
    return 1;
}

static uint8_t lora_manual_seq_is_newer(uint32_t next_seq)
{
    if (!lora_state.manual_seq_valid) {
        return 1;
    }
    if (next_seq == lora_state.manual_seq) {
        return 0;
    }
    if (next_seq > lora_state.manual_seq) {
        return 1;
    }
    return (uint32_t)(lora_state.manual_seq - next_seq) > 1000000u;
}

static Waypoint_t s_lora_wp_buf[MAX_WAYPOINTS];
static uint8_t s_lora_wp_count = 0;
extern RobotSM_t g_sm;

static uint8_t lora_parse_state_request(const char *cmd, uint8_t *out_state)
{
    if (!cmd || !out_state) return 0;

    char normalized[64];
    size_t in_len = strlen(cmd);
    if (in_len >= sizeof(normalized)) {
        in_len = sizeof(normalized) - 1;
    }

    memcpy(normalized, cmd, in_len);
    normalized[in_len] = '\0';

    // Trim leading/trailing whitespace
    char *start = normalized;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }

    char *end = start + strlen(start);
    while (end > start && isspace((unsigned char)*(end - 1))) {
        end--;
    }
    *end = '\0';

    // Normalize case for robust matching
    for (char *p = start; *p != '\0'; ++p) {
        *p = (char)toupper((unsigned char)*p);
    }

    // Accept optional "CMD:" prefix
    const char *payload = start;
    if (strncmp(payload, "CMD:", 4) == 0) {
        payload += 4;
    }

    // Strip optional parameter section (e.g. AUTO,SALT:20)
    char *comma = strchr((char *)payload, ',');
    if (comma) {
        *comma = '\0';
    }

    if (strcmp(payload, "A") == 0 || strcmp(payload, LORA_CMD_AUTO) == 0) {
        *out_state = STATE_AUTO;
        return 1;
    }
    if (strcmp(payload, "M") == 0 || strcmp(payload, LORA_CMD_MANUAL) == 0) {
        *out_state = STATE_MANUAL;
        return 1;
    }
    if (strcmp(payload, "P") == 0 || strcmp(payload, LORA_CMD_PAUSE) == 0) {
        *out_state = STATE_PAUSE;
        return 1;
    }
    if (strcmp(payload, "E") == 0 || strcmp(payload, LORA_CMD_ESTOP) == 0) {
        *out_state = STATE_ESTOP;
        return 1;
    }
    if (strcmp(payload, "X") == 0 || strcmp(payload, LORA_CMD_RESET) == 0) {
        return 2;
    }

    return 0;
}

static uint8_t lora_parse_manual_request(const char *cmd, LoRA_ManualCommand_t *out_cmd)
{
    if (!cmd || !out_cmd) return 0;

    char normalized[80];
    size_t in_len = strlen(cmd);
    if (in_len >= sizeof(normalized)) {
        in_len = sizeof(normalized) - 1;
    }

    memcpy(normalized, cmd, in_len);
    normalized[in_len] = '\0';

    char *start = normalized;
    while (*start && isspace((unsigned char)*start)) {
        start++;
    }

    char *end = start + strlen(start);
    while (end > start && isspace((unsigned char)*(end - 1))) {
        end--;
    }
    *end = '\0';

    for (char *p = start; *p != '\0'; ++p) {
        *p = (char)toupper((unsigned char)*p);
    }

    const char *payload = start;
    if (strncmp(payload, "CMD:", 4) == 0) {
        payload += 4;
    }

    if (strncmp(payload, "DRIVE", 5) == 0 || strncmp(payload, "JOY", 3) == 0 || strncmp(payload, "JOYSTICK", 8) == 0) {
        int8_t drive_pct = 0;
        int8_t turn_pct = 0;
        uint8_t have_drive = lora_parse_percent_field(payload, "THROTTLE", &drive_pct)
            || lora_parse_percent_field(payload, "DRIVE", &drive_pct);
        uint8_t have_turn = lora_parse_percent_field(payload, "TURN", &turn_pct)
            || lora_parse_percent_field(payload, "STEER", &turn_pct);

        if (have_drive || have_turn) {
            lora_state.manual_drive_pct = drive_pct;
            lora_state.manual_turn_pct = turn_pct;
            *out_cmd = LORA_MANUAL_CMD_DRIVE;
            return 1;
        }
    }

    {
        uint32_t manual_seq = 0;
        int8_t drive_pct = 0;
        int8_t turn_pct = 0;
        if (lora_parse_compact_manual_packet(payload, &manual_seq, &drive_pct, &turn_pct)) {
            if (!lora_manual_seq_is_newer(manual_seq)) {
                return 0;
            }
            lora_state.manual_seq = manual_seq;
            lora_state.manual_seq_valid = 1;
            lora_state.manual_drive_pct = drive_pct;
            lora_state.manual_turn_pct = turn_pct;
            *out_cmd = LORA_MANUAL_CMD_DRIVE;
            return 1;
        }
    }

    {
        int8_t drive_pct = 0;
        int8_t turn_pct = 0;
        if (lora_parse_compact_drive(payload, &drive_pct, &turn_pct)) {
            lora_state.manual_drive_pct = drive_pct;
            lora_state.manual_turn_pct = turn_pct;
            *out_cmd = LORA_MANUAL_CMD_DRIVE;
            return 1;
        }
    }

    if (strcmp(payload, "F") == 0 || strcmp(payload, "FORWARD") == 0) {
        *out_cmd = LORA_MANUAL_CMD_FORWARD;
        return 1;
    }
    if (strcmp(payload, "B") == 0 || strcmp(payload, "BACK") == 0 || strcmp(payload, "BACKWARD") == 0) {
        *out_cmd = LORA_MANUAL_CMD_BACK;
        return 1;
    }
    if (strcmp(payload, "L") == 0 || strcmp(payload, "LEFT") == 0) {
        *out_cmd = LORA_MANUAL_CMD_LEFT;
        return 1;
    }
    if (strcmp(payload, "R") == 0 || strcmp(payload, "RIGHT") == 0) {
        *out_cmd = LORA_MANUAL_CMD_RIGHT;
        return 1;
    }
    if (strcmp(payload, "S") == 0 || strcmp(payload, "STOP") == 0) {
        *out_cmd = LORA_MANUAL_CMD_STOP;
        return 1;
    }

    if (strcmp(payload, "ALLON") == 0 || strcmp(payload, "FULLON") == 0) {
        *out_cmd = LORA_MANUAL_CMD_ALL_ON;
        return 1;
    }
    if (strcmp(payload, "AGITATOR ON") == 0 || strcmp(payload, "TEST AGITATOR ON") == 0) {
        *out_cmd = LORA_MANUAL_CMD_AGITATOR_ON;
        return 1;
    }
    if (strcmp(payload, "AGITATOR OFF") == 0 || strcmp(payload, "TEST AGITATOR OFF") == 0) {
        *out_cmd = LORA_MANUAL_CMD_AGITATOR_OFF;
        return 1;
    }
    if (strcmp(payload, "THROWER ON") == 0 || strcmp(payload, "TEST THROWER ON") == 0) {
        *out_cmd = LORA_MANUAL_CMD_THROWER_ON;
        return 1;
    }
    if (strcmp(payload, "THROWER OFF") == 0 || strcmp(payload, "TEST THROWER OFF") == 0) {
        *out_cmd = LORA_MANUAL_CMD_THROWER_OFF;
        return 1;
    }
    if (strcmp(payload, "RELAY ON") == 0 || strcmp(payload, "TEST RELAY ON") == 0) {
        *out_cmd = LORA_MANUAL_CMD_RELAY_ON;
        return 1;
    }
    if (strcmp(payload, "RELAY OFF") == 0 || strcmp(payload, "TEST RELAY OFF") == 0) {
        *out_cmd = LORA_MANUAL_CMD_RELAY_OFF;
        return 1;
    }
    if (strcmp(payload, "VIBRATION ON") == 0 || strcmp(payload, "TEST VIBRATION ON") == 0) {
        *out_cmd = LORA_MANUAL_CMD_VIBRATION_ON;
        return 1;
    }
    if (strcmp(payload, "VIBRATION OFF") == 0 || strcmp(payload, "TEST VIBRATION OFF") == 0) {
        *out_cmd = LORA_MANUAL_CMD_VIBRATION_OFF;
        return 1;
    }
    if (strncmp(payload, "TEST SALT", 9) == 0) {
        int8_t salt_pct = 50;
        const char *pct = payload + 9;
        while (*pct == ' ') pct++;
        if (*pct != '\0') {
            salt_pct = (int8_t)atoi(pct);
        }
        if (salt_pct < 0) salt_pct = 0;
        if (salt_pct > 100) salt_pct = 100;
        lora_state.manual_drive_pct = salt_pct;
        lora_state.manual_turn_pct = 0;
        *out_cmd = LORA_MANUAL_CMD_TEST_SALT;
        return 1;
    }
    if (strncmp(payload, "TEST BRINE", 10) == 0) {
        int8_t brine_pct = 50;
        const char *pct = payload + 10;
        while (*pct == ' ') pct++;
        if (*pct != '\0') {
            brine_pct = (int8_t)atoi(pct);
        }
        if (brine_pct < 0) brine_pct = 0;
        if (brine_pct > 100) brine_pct = 100;
        lora_state.manual_drive_pct = 0;
        lora_state.manual_turn_pct = brine_pct;
        *out_cmd = LORA_MANUAL_CMD_TEST_BRINE;
        return 1;
    }
    // Basic JSON compatibility for upcoming app payloads
    if (payload[0] == '{') {
        int8_t drive_pct = 0;
        int8_t turn_pct = 0;
        uint8_t have_drive = lora_parse_percent_field(payload, "\"THROTTLE\"", &drive_pct)
            || lora_parse_percent_field(payload, "\"DRIVE\"", &drive_pct)
            || lora_parse_percent_field(payload, "THROTTLE", &drive_pct)
            || lora_parse_percent_field(payload, "DRIVE", &drive_pct);
        uint8_t have_turn = lora_parse_percent_field(payload, "\"TURN\"", &turn_pct)
            || lora_parse_percent_field(payload, "\"STEER\"", &turn_pct)
            || lora_parse_percent_field(payload, "TURN", &turn_pct)
            || lora_parse_percent_field(payload, "STEER", &turn_pct);

        if (have_drive || have_turn) {
            lora_state.manual_drive_pct = drive_pct;
            lora_state.manual_turn_pct = turn_pct;
            *out_cmd = LORA_MANUAL_CMD_DRIVE;
            return 1;
        }

        if (strstr(payload, "FORWARD") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_FORWARD;
            return 1;
        }
        if (strstr(payload, "BACKWARD") != NULL || strstr(payload, "BACK") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_BACK;
            return 1;
        }
        if (strstr(payload, "LEFT") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_LEFT;
            return 1;
        }
        if (strstr(payload, "RIGHT") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_RIGHT;
            return 1;
        }
        if (strstr(payload, "STOP") != NULL) {
            *out_cmd = LORA_MANUAL_CMD_STOP;
            return 1;
        }
    }

    return 0;
}

static uint8_t lora_accept_command_text(const char *cmd)
{
    if (!cmd || !cmd[0]) return 0;

    lora_state.command_valid = 0;
    lora_state.manual_command_valid = 0;

    uint8_t parsed_state_cmd = lora_parse_state_request(cmd, &lora_state.pending_state_request);
    if (parsed_state_cmd == 1)
    {
        lora_state.command_valid = 1;
        strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
        lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
        lora_send_received_ack_for_state(lora_state.pending_state_request);
        if (s_lora_verbose) {
            printf("[LORA] Valid command: %s -> state=%u\r\n", cmd, lora_state.pending_state_request);
        }
        return 1;
    }
    else if (parsed_state_cmd == 2)
    {
        lora_state.pending_reset_request = 1;
        strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
        lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
        if (s_lora_verbose) {
            printf("[LORA] Valid reset request: %s\r\n", cmd);
        }
        return 1;
    }
    else if (lora_parse_manual_request(cmd, &lora_state.pending_manual_cmd))
    {
        lora_state.manual_command_valid = 1;
        strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
        lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
        lora_send_received_ack_for_manual(lora_state.pending_manual_cmd);
        if (s_lora_verbose) {
            printf("[LORA] Valid manual command: %s -> cmd=%u\r\n", cmd, (unsigned)lora_state.pending_manual_cmd);
        }
        return 1;
    }
    else if (strcmp(cmd, LORA_WP_CLEAR) == 0)
    {
        memset(s_lora_wp_buf, 0, sizeof(s_lora_wp_buf));
        s_lora_wp_count = 0;
        Mission_ClearCurrent();
        Mission_ClearPersisted();
        strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
        lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
        LoRA_SendRaw(LORA_WP_ACK_CLEAR);
        if (s_lora_verbose) {
            printf("[LORA] Cleared staged waypoint set\r\n");
        }
        return 1;
    }
    else if (strncmp(cmd, LORA_WP_ADD_PREFIX ":", 3) == 0)
    {
        uint8_t idx = 0;
        float lat = 0.0f;
        float lon = 0.0f;
        int salt_pct = 0;
        int brine_pct = 0;

        if (sscanf(cmd + 3, "%hhu:%f,%f,%d,%d", &idx, &lat, &lon, &salt_pct, &brine_pct) == 5 && idx < MAX_WAYPOINTS)
        {
            char ack[24];

            if (salt_pct < 0) salt_pct = 0;
            if (salt_pct > 100) salt_pct = 100;
            if (brine_pct < 0) brine_pct = 0;
            if (brine_pct > 100) brine_pct = 100;

            s_lora_wp_buf[idx].latitude = lat;
            s_lora_wp_buf[idx].longitude = lon;
            s_lora_wp_buf[idx].salt_rate = (float)salt_pct / 100.0f;
            s_lora_wp_buf[idx].brine_rate = (float)brine_pct / 100.0f;
            if ((uint8_t)(idx + 1) > s_lora_wp_count) {
                s_lora_wp_count = (uint8_t)(idx + 1);
            }

            Mission_AddWaypoint(lat, lon, (uint8_t)salt_pct, (uint8_t)brine_pct);

            strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
            lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
            snprintf(ack, sizeof(ack), "%s:%hhu", LORA_WP_ACK_ADD_PREFIX, idx);
            LoRA_SendRaw(ack);
            if (s_lora_verbose) {
                printf("[LORA] Staged waypoint %u lat=%.6f lon=%.6f salt=%d brine=%d\r\n",
                       idx, lat, lon, salt_pct, brine_pct);
            }
            return 1;
        }
    }
    else if (strncmp(cmd, LORA_WP_BATCH_PREFIX ":", 4) == 0)
    {
        const char *payload = cmd + 4;
        char *endptr = NULL;
        long start_idx_long = strtol(payload, &endptr, 10);

        if (endptr != payload && endptr && *endptr == ':' && start_idx_long >= 0 && start_idx_long < MAX_WAYPOINTS)
        {
            uint8_t idx = (uint8_t)start_idx_long;
            uint8_t start_idx = idx;
            uint8_t count = 0;
            const char *cursor = endptr + 1;
            bool parse_ok = true;

            while (*cursor != '\0')
            {
                if (idx >= MAX_WAYPOINTS) {
                    parse_ok = false;
                    break;
                }

                float lat = 0.0f;
                float lon = 0.0f;
                int salt_pct = 0;
                int brine_pct = 0;
                int consumed = 0;

                if (sscanf(cursor, "%f,%f,%d,%d%n", &lat, &lon, &salt_pct, &brine_pct, &consumed) != 4 || consumed <= 0) {
                    parse_ok = false;
                    break;
                }

                if (salt_pct < 0) salt_pct = 0;
                if (salt_pct > 100) salt_pct = 100;
                if (brine_pct < 0) brine_pct = 0;
                if (brine_pct > 100) brine_pct = 100;

                s_lora_wp_buf[idx].latitude = lat;
                s_lora_wp_buf[idx].longitude = lon;
                s_lora_wp_buf[idx].salt_rate = (float)salt_pct / 100.0f;
                s_lora_wp_buf[idx].brine_rate = (float)brine_pct / 100.0f;

                Mission_AddWaypoint(lat, lon, (uint8_t)salt_pct, (uint8_t)brine_pct);

                idx++;
                count++;
                cursor += consumed;

                if (*cursor == '\0') {
                    break;
                }
                if (*cursor != ';') {
                    parse_ok = false;
                    break;
                }
                cursor++;
            }

            if (parse_ok && count > 0)
            {
                char ack[32];

                if (idx > s_lora_wp_count) {
                    s_lora_wp_count = idx;
                }

                strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
                lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
                snprintf(ack, sizeof(ack), "%s:%hhu:%hhu", LORA_WP_ACK_BATCH_PREFIX, start_idx, count);
                LoRA_SendRaw(ack);
                if (s_lora_verbose) {
                    printf("[LORA] Staged waypoint batch start=%u count=%u\r\n",
                           start_idx,
                           count);
                }
                return 1;
            }
        }
    }
    else if (strncmp(cmd, LORA_WP_LOAD_PREFIX ":", 7) == 0)
    {
        uint8_t count = 0;
        if (sscanf(cmd + 7, "%hhu", &count) == 1 && count > 0 && count == s_lora_wp_count)
        {
            char ack[28];

            RobotSM_LoadMission(&g_sm, s_lora_wp_buf, s_lora_wp_count);
            strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
            lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
            snprintf(ack, sizeof(ack), "%s:%hhu", LORA_WP_ACK_LOAD_PREFIX, count);
            LoRA_SendRaw(ack);
            if (s_lora_verbose) {
                printf("[LORA] Loaded %u staged waypoints into mission state\r\n", count);
            }
            return 1;
        }
    }

    lora_state.invalid_command_count++;
    if (s_lora_verbose) {
        printf("[LORA] Invalid command: %s\r\n", cmd);
    }
    return 0;
}

static void lora_uart5_send(const char *msg, const char *tag)
{
    if (!lora_state.huart || !msg) return;

#if LORA_TX_DIAGNOSTICS_ENABLED
    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(
        lora_state.huart,
        (uint8_t *)msg,
        strlen(msg),
        LORA_UART_TX_TIMEOUT_MS);

    if (tx_status != HAL_OK) {
        if (lora_state.tx_fail_streak < 255u) {
            lora_state.tx_fail_streak++;
        }
        if (s_lora_verbose) {
            printf("[LORA] TX failed for %s (status=%d)\r\n", tag ? tag : "payload", (int)tx_status);
        }
        return;
    }

    strncpy(lora_state.last_tx_payload, msg, sizeof(lora_state.last_tx_payload) - 1);
    lora_state.last_tx_payload[sizeof(lora_state.last_tx_payload) - 1] = '\0';
    lora_state.last_tx_ms = HAL_GetTick();
    lora_state.tx_count++;
    lora_state.tx_fail_streak = 0;
    if (s_lora_verbose) {
        printf("[LORA] Sent %s: %s", tag ? tag : "", msg);
    }
#else
    (void)tag;
    if (s_lora_verbose) {
        printf("[LORA] TX suppressed\r\n");
    }
#endif
}

static const char* lora_unwrap_stream_frame(char *msg, uint32_t *out_seq, uint8_t *out_has_seq)
{
    if (out_has_seq) *out_has_seq = 0;
    if (out_seq) *out_seq = 0;
    if (!msg) return NULL;

    if (strncmp(msg, "S:", 2) != 0) {
        return msg;
    }

    char *seq_start = msg + 2;
    char *endptr = NULL;
    unsigned long seq = strtoul(seq_start, &endptr, 10);
    if (endptr == seq_start || !endptr || *endptr != ':') {
        return msg;
    }

    if (out_seq) {
        *out_seq = (uint32_t)seq;
    }
    if (out_has_seq) {
        *out_has_seq = 1;
    }

    return endptr + 1;
}

uint8_t LoRA_InjectCommandText(const char *text)
{
    if (!text) return 0;

    char normalized[LORA_RX_BUFFER_SIZE];
    size_t in_len = strlen(text);
    while (in_len > 0 && (text[in_len - 1] == '\r' || text[in_len - 1] == '\n')) {
        in_len--;
    }
    while (*text == ' ' || *text == '\t') {
        text++;
    }
    while (in_len > 0 && (text[in_len - 1] == ' ' || text[in_len - 1] == '\t')) {
        in_len--;
    }
    if (in_len == 0) return 0;
    if (in_len >= sizeof(normalized)) {
        in_len = sizeof(normalized) - 1;
    }

    memcpy(normalized, text, in_len);
    normalized[in_len] = '\0';

    strncpy(lora_state.last_raw_frame, normalized, sizeof(lora_state.last_raw_frame) - 1);
    lora_state.last_raw_frame[sizeof(lora_state.last_raw_frame) - 1] = '\0';
    lora_state.raw_frame_count++;
    lora_state.last_rx_ms = HAL_GetTick();
    lora_state.rx_count++;

    return lora_accept_command_text(normalized);
}

// Initialize LoRA UART interface (UART5, 921600 baud)
void LoRA_Init(UART_HandleTypeDef *huart5)
{
    if (!huart5) return;

    lora_state.huart = huart5;
    lora_state.rx_index = 0;
    lora_state.raw_index = 0;
    lora_state.raw_frame_count = 0;
    lora_state.last_rx_ms = 0;
    lora_state.last_tx_ms = 0;
    lora_state.rx_count = 0;
    lora_state.tx_count = 0;
    lora_state.pending_state_request = 0;
    lora_state.pending_reset_request = 0;
    lora_state.pending_manual_cmd = LORA_MANUAL_CMD_NONE;
    lora_state.manual_command_valid = 0;
    lora_state.manual_drive_pct = 0;
    lora_state.manual_turn_pct = 0;
    lora_state.manual_seq = 0;
    lora_state.manual_seq_valid = 0;
    lora_state.invalid_command_count = 0;
    lora_state.start_filter_drop_count = 0;
    lora_state.prefix_reject_count = 0;
    lora_state.rx_overflow_count = 0;
    lora_state.stream_seq_init = 0;
    lora_state.stream_expected_seq = 0;
    lora_state.stream_gap_count = 0;
    lora_state.stream_len = 0;
    lora_state.drop_until_eol = 0;
    lora_state.isr_q_head = 0;
    lora_state.isr_q_tail = 0;
    lora_state.isr_q_overflow_count = 0;
    lora_state.tx_fail_streak = 0;
    memset(lora_state.rx_buffer, 0, LORA_RX_BUFFER_SIZE);
    memset(lora_state.raw_buffer, 0, LORA_RX_BUFFER_SIZE);
    memset(lora_state.last_raw_frame, 0, sizeof(lora_state.last_raw_frame));
    memset(lora_state.last_command, 0, sizeof(lora_state.last_command));
    memset(lora_state.last_tx_payload, 0, sizeof(lora_state.last_tx_payload));
    memset(lora_state.stream_buffer, 0, sizeof(lora_state.stream_buffer));
    memset(lora_state.isr_q, 0, sizeof(lora_state.isr_q));

    if (s_lora_verbose) {
        printf("[LORA] LoRA module initialized on UART5 (921600 baud)\r\n");
    }
}

// Parse one UART5 byte in main-loop context.
static void lora_process_rx_byte(uint8_t byte)
{
    if (!lora_state.huart) return;

    const uint8_t upper = (uint8_t)toupper((unsigned char)byte);
    lora_state.last_rx_ms = HAL_GetTick();

    // Check for line ending (CR or LF)
    if (byte == '\r' || byte == '\n')
    {
        if (lora_state.raw_index > 0)
        {
            lora_state.raw_buffer[lora_state.raw_index] = '\0';
            strncpy(lora_state.last_raw_frame, (const char *)lora_state.raw_buffer,
                    sizeof(lora_state.last_raw_frame) - 1);
            lora_state.last_raw_frame[sizeof(lora_state.last_raw_frame) - 1] = '\0';
            lora_state.raw_frame_count++;
            lora_state.raw_index = 0;
        }

        // Process complete message
        if (lora_state.rx_index > 0)
        {
            lora_state.rx_buffer[lora_state.rx_index] = '\0';
            lora_state.last_rx_ms = HAL_GetTick();
            lora_state.rx_count++;

            if (s_lora_verbose) {
                printf("[LORA] Received: %s\r\n", lora_state.rx_buffer);
            }

            // Parse command
            lora_state.command_valid = 0;
            lora_state.manual_command_valid = 0;
            uint32_t stream_seq = 0;
            uint8_t has_stream_seq = 0;
            const char *cmd = lora_unwrap_stream_frame((char *)lora_state.rx_buffer, &stream_seq, &has_stream_seq);
            const char *payload = cmd;
            uint8_t chunk_marked = 0;
            uint8_t chunk_is_end = 1;

            if (has_stream_seq) {
                if (!lora_state.stream_seq_init) {
                    lora_state.stream_expected_seq = stream_seq + 1;
                    lora_state.stream_seq_init = 1;
                } else {
                    if (stream_seq != lora_state.stream_expected_seq) {
                        lora_state.stream_gap_count++;
                        if (s_lora_verbose) {
                            printf("[LORA] Stream sequence gap: got %lu expected %lu (gaps=%lu)\r\n",
                                   (unsigned long)stream_seq,
                                   (unsigned long)lora_state.stream_expected_seq,
                                   (unsigned long)lora_state.stream_gap_count);
                        }
                        lora_state.stream_len = 0;
                    }
                    lora_state.stream_expected_seq = stream_seq + 1;
                }

                if (payload && (payload[0] == 'M' || payload[0] == 'E') && payload[1] == ':') {
                    chunk_marked = 1;
                    chunk_is_end = (payload[0] == 'E') ? 1u : 0u;
                    payload += 2;
                }

                if (chunk_marked) {
                    size_t pl = strlen(payload);
                    size_t remain = (size_t)(LORA_STREAM_BUFFER_SIZE - 1 - lora_state.stream_len);
                    if (pl > remain) {
                        pl = remain;
                    }
                    if (pl > 0) {
                        memcpy(&lora_state.stream_buffer[lora_state.stream_len], payload, pl);
                        lora_state.stream_len += (uint16_t)pl;
                        lora_state.stream_buffer[lora_state.stream_len] = '\0';
                    }

                    if (!chunk_is_end) {
                        lora_state.rx_index = 0;
                        return;
                    }

                    cmd = lora_state.stream_buffer;
                    lora_state.stream_len = 0;
                } else {
                    cmd = payload;
                }
            }

            uint8_t parsed_state_cmd = lora_parse_state_request(cmd, &lora_state.pending_state_request);
            if (parsed_state_cmd == 1)
            {
                lora_state.command_valid = 1;
                strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
                lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
                lora_send_received_ack_for_state(lora_state.pending_state_request);
                if (s_lora_verbose) {
                    printf("[LORA] Valid command: %s -> state=%u\r\n", cmd, lora_state.pending_state_request);
                }
            }
            else if (parsed_state_cmd == 2)
            {
                lora_state.pending_reset_request = 1;
                strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
                lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
                if (s_lora_verbose) {
                    printf("[LORA] Valid reset request: %s\r\n", cmd);
                }
            }
            else if (lora_parse_manual_request(cmd, &lora_state.pending_manual_cmd))
            {
                lora_state.manual_command_valid = 1;
                strncpy(lora_state.last_command, cmd, sizeof(lora_state.last_command) - 1);
                lora_state.last_command[sizeof(lora_state.last_command) - 1] = '\0';
                lora_send_received_ack_for_manual(lora_state.pending_manual_cmd);
                if (s_lora_verbose) {
                    printf("[LORA] Valid manual command: %s -> cmd=%u\r\n", cmd, (unsigned)lora_state.pending_manual_cmd);
                }
            }
            else if (strcmp(cmd, LORA_WP_CLEAR) == 0)
            {
                memset(s_lora_wp_buf, 0, sizeof(s_lora_wp_buf));
                s_lora_wp_count = 0;
                Mission_ClearCurrent();
                Mission_ClearPersisted();
                LoRA_SendRaw(LORA_WP_ACK_CLEAR);
                if (s_lora_verbose) {
                    printf("[LORA] Cleared staged waypoint set\r\n");
                }
            }
            else if (strncmp(cmd, LORA_WP_ADD_PREFIX ":", 3) == 0)
            {
                uint8_t idx = 0;
                float lat = 0.0f;
                float lon = 0.0f;
                int salt_pct = 0;
                int brine_pct = 0;

                if (sscanf(cmd + 3, "%hhu:%f,%f,%d,%d", &idx, &lat, &lon, &salt_pct, &brine_pct) == 5 && idx < MAX_WAYPOINTS)
                {
                    char ack[24];

                    if (salt_pct < 0) salt_pct = 0;
                    if (salt_pct > 100) salt_pct = 100;
                    if (brine_pct < 0) brine_pct = 0;
                    if (brine_pct > 100) brine_pct = 100;

                    s_lora_wp_buf[idx].latitude = lat;
                    s_lora_wp_buf[idx].longitude = lon;
                    s_lora_wp_buf[idx].salt_rate = (float)salt_pct / 100.0f;
                    s_lora_wp_buf[idx].brine_rate = (float)brine_pct / 100.0f;
                    if ((uint8_t)(idx + 1) > s_lora_wp_count) {
                        s_lora_wp_count = (uint8_t)(idx + 1);
                    }

                    Mission_AddWaypoint(lat, lon, (uint8_t)salt_pct, (uint8_t)brine_pct);

                    snprintf(ack, sizeof(ack), "%s:%hhu", LORA_WP_ACK_ADD_PREFIX, idx);
                    LoRA_SendRaw(ack);
                    if (s_lora_verbose) {
                        printf("[LORA] Staged waypoint %u lat=%.6f lon=%.6f salt=%d brine=%d\r\n",
                               idx, lat, lon, salt_pct, brine_pct);
                    }
                }
            }
            else if (strncmp(cmd, LORA_WP_LOAD_PREFIX ":", 7) == 0)
            {
                uint8_t count = 0;
                if (sscanf(cmd + 7, "%hhu", &count) == 1 && count > 0 && count == s_lora_wp_count)
                {
                    char ack[28];

                    RobotSM_LoadMission(&g_sm, s_lora_wp_buf, s_lora_wp_count);
                    snprintf(ack, sizeof(ack), "%s:%hhu", LORA_WP_ACK_LOAD_PREFIX, count);
                    LoRA_SendRaw(ack);
                    if (s_lora_verbose) {
                        printf("[LORA] Loaded %u staged waypoints into mission state\r\n", count);
                    }
                }
            }
            else
            {
                lora_state.invalid_command_count++;
                if (s_lora_verbose) {
                    printf("[LORA] Invalid command: %s\r\n", cmd);
                }
            }

            // Reset buffer
            lora_state.rx_index = 0;
        }

        lora_state.drop_until_eol = 0;
        return;
    }

    // Always capture raw UART5 bytes for monitor mode
    if (lora_state.raw_index < (LORA_RX_BUFFER_SIZE - 1))
    {
        lora_state.raw_buffer[lora_state.raw_index++] = byte;
    }
    else
    {
        lora_state.raw_index = 0;
    }

    if (lora_state.drop_until_eol) {
        return;
    }

    // Accept control frames case-insensitively; also allow JSON payload starts.
    if (lora_state.rx_index == 0) {
        if (!lora_is_valid_start_byte(byte, upper)) {
            lora_state.start_filter_drop_count++;
            lora_state.drop_until_eol = 1;
            return;
        }
    } else if (lora_cmd_prefix_rejects_byte(lora_state.rx_index, lora_state.rx_buffer[0], upper, byte)) {
        lora_state.prefix_reject_count++;
        lora_state.rx_index = 0;
        return;
    }

    // Add byte to buffer
    if (lora_state.rx_index < (LORA_RX_BUFFER_SIZE - 1))
    {
        lora_state.rx_buffer[lora_state.rx_index++] = byte;
    }
    else
    {
        lora_state.rx_overflow_count++;
        // Buffer overflow - reset
        if (s_lora_verbose) {
            printf("[LORA] RX buffer overflow, resetting\r\n");
        }
        lora_state.rx_index = 0;
    }
}

// ISR entrypoint: enqueue bytes only.
void LoRA_RxByte(uint8_t byte)
{
    if (!lora_state.huart) return;

    if (!lora_isr_queue_push(byte) && s_lora_verbose) {
        printf("[LORA] ISR queue overflow (count=%lu)\r\n",
               (unsigned long)lora_state.isr_q_overflow_count);
    }
}

void LoRA_ProcessRxQueue(uint16_t max_bytes)
{
    uint16_t processed = 0;
    uint8_t byte = 0;

    while (lora_isr_queue_pop(&byte)) {
        lora_process_rx_byte(byte);
        processed++;
        if (max_bytes != 0 && processed >= max_bytes) {
            break;
        }
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
            if (s_lora_verbose) {
                printf("[LORA] RX timeout, discarding incomplete message\r\n");
            }
            lora_state.rx_index = 0;
        }
    }

    // Report health status to system health monitor
    // Consider LoRa healthy if any RX/TX activity has happened recently.
    uint32_t last_activity_ms = lora_state.last_tx_ms;
    if (lora_state.last_rx_ms > last_activity_ms) {
        last_activity_ms = lora_state.last_rx_ms;
    }

    uint8_t inactive = 0;
    if (last_activity_ms == 0) {
        inactive = 1;
    } else if ((now_ms - last_activity_ms) > LORA_HEALTH_INACTIVE_MS) {
        inactive = 1;
    }

    if (inactive) {
        if (lora_state.health_stale_ticks < 255u) {
            lora_state.health_stale_ticks++;
        }
        lora_state.health_fresh_ticks = 0;
        if (lora_state.health_stale_ticks >= LORA_HEALTH_STALE_TICKS_TO_TIMEOUT) {
            SystemHealth_SetSensorStatus(SENSOR_LORA, SENSOR_TIMEOUT);
        }
    } else {
        if (lora_state.health_fresh_ticks < 255u) {
            lora_state.health_fresh_ticks++;
        }
        lora_state.health_stale_ticks = 0;
        if (lora_state.health_fresh_ticks >= LORA_HEALTH_FRESH_TICKS_TO_RECOVER) {
            SystemHealth_SetSensorStatus(SENSOR_LORA, SENSOR_OK);
        }
    }
}

// Send state diagnostics to base station (JSON format)
void LoRA_SendState(uint8_t state, float gps_lat, float gps_lon,
                    int motor_m1, int motor_m2)
{
    if (!lora_state.huart) return;

    float approx_speed = (float)abs((motor_m1 + motor_m2) / 2) / 100.0f;
    char msg[96];
    snprintf(msg, sizeof(msg),
             "S:%s,%.6f,%.6f,0.0,%.2f,0,0.0,0\r\n",
             lora_state_name(state), gps_lat, gps_lon, approx_speed);

    lora_uart5_send(msg, "state");
}

// Send comprehensive telemetry to base station (JSON format)
void LoRA_SendManualTelemetry(int motor_m1, int motor_m2,
                               float yaw_deg,
                               uint16_t prox_left_cm, uint16_t prox_right_cm)
{
    if (!lora_state.huart) return;

    const uint8_t imu_ok = (SystemHealth_GetSensorStatus(SENSOR_IMU) == SENSOR_OK) ? 1u : 0u;
    const uint8_t gps_ok = (SystemHealth_GetSensorStatus(SENSOR_GPS) == SENSOR_OK) ? 1u : 0u;
    const uint8_t lora_ok = (SystemHealth_GetSensorStatus(SENSOR_LORA) == SENSOR_OK) ? 1u : 0u;
    const uint8_t degraded = (imu_ok && gps_ok && lora_ok) ? 0u : 1u;

    (void)prox_left_cm;
    (void)prox_right_cm;

    char msg[72];
    snprintf(msg, sizeof(msg),
             "M:%.1f,%d,%d,%u,%u,%u,%u\r\n",
             yaw_deg, motor_m1, motor_m2,
             (unsigned)imu_ok, (unsigned)gps_ok, (unsigned)lora_ok, (unsigned)degraded);

    lora_uart5_send(msg, "manual");
}

// Send comprehensive telemetry to base station (JSON format)
void LoRA_SendTelemetry(uint8_t state, float gps_lat, float gps_lon, uint8_t gps_has_fix,
                        uint8_t gps_num_sat, float gps_hdop,
                        int motor_m1, int motor_m2, float yaw_deg, float pitch_deg,
                        uint8_t salt_rate, uint8_t brine_rate, float temp_c,
                        uint16_t prox_left_cm, uint16_t prox_right_cm)
{
    if (!lora_state.huart) return;

    (void)pitch_deg;
    (void)temp_c;
    (void)prox_left_cm;
    (void)prox_right_cm;

    float approx_speed = (float)abs((motor_m1 + motor_m2) / 2) / 100.0f;
    char msg[128];
    snprintf(msg, sizeof(msg),
             "T:%s,%.6f,%.6f,%.1f,%.2f,%u,%.1f,%u,%d,%d,%u,%u\r\n",
             lora_state_name(state), gps_lat, gps_lon, yaw_deg, approx_speed,
             gps_has_fix, gps_hdop, gps_num_sat, motor_m1, motor_m2, salt_rate, brine_rate);

    lora_uart5_send(msg, "telemetry");
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
        case 3: fault_name = "MOTOR_FEEDBACK"; break;
        case 4: fault_name = "BATTERY_COLD"; break;
        case 5: fault_name = "PROXIMITY_WARN"; break;
        case 6: fault_name = "PROXIMITY_CRIT"; break;
        case 7: fault_name = "DISPERSION_CLOG"; break;
        default: fault_name = "GENERIC"; break;
    }

    const char *action_name = "UNKNOWN";
    switch (action)
    {
        case 0: action_name = "PAUSE"; break;
        case 1: action_name = "ESTOP"; break;
        case 2: action_name = "LOG_ONLY"; break;
    }

    char msg[64];
    snprintf(msg, sizeof(msg), "F:%s,%s\r\n", fault_name, action_name);

    lora_uart5_send(msg, "fault");
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

uint8_t LoRA_GetPendingResetRequest(void)
{
    if (lora_state.pending_reset_request)
    {
        lora_state.pending_reset_request = 0;
        return 1;
    }

    return 0;
}

uint8_t LoRA_GetPendingManualCommand(LoRA_ManualCommand_t *out_cmd)
{
    if (!out_cmd) return 0;

    if (lora_state.manual_command_valid)
    {
        *out_cmd = lora_state.pending_manual_cmd;
        lora_state.manual_command_valid = 0;
        return 1;
    }

    return 0;
}

int LoRA_GetManualDrivePct(void)
{
    return (int)lora_state.manual_drive_pct;
}

int LoRA_GetManualTurnPct(void)
{
    return (int)lora_state.manual_turn_pct;
}

// Get last command for debugging
const char* LoRA_GetLastCommand(void)
{
    return lora_state.last_command;
}

const char* LoRA_GetLastRawFrame(void)
{
    return lora_state.last_raw_frame;
}

uint32_t LoRA_GetRawFrameCount(void)
{
    return lora_state.raw_frame_count;
}

const char* LoRA_GetLastTxPayload(void)
{
    return lora_state.last_tx_payload;
}

// Send raw test string over LoRa UART
void LoRA_SendRaw(const char *text)
{
    if (!lora_state.huart || !text || text[0] == '\0') return;

    char msg[192];
    size_t in_len = strlen(text);

    if (in_len >= sizeof(msg) - 3) {
        in_len = sizeof(msg) - 3;
    }

    memcpy(msg, text, in_len);
    msg[in_len] = '\0';

    // Ensure CRLF line ending for ESP32 line parser compatibility
    if (in_len == 0 || msg[in_len - 1] != '\n') {
        msg[in_len++] = '\r';
        msg[in_len++] = '\n';
        msg[in_len] = '\0';
    }

    HAL_StatusTypeDef tx_status = HAL_UART_Transmit(lora_state.huart, (uint8_t *)msg, in_len, LORA_UART_TX_TIMEOUT_MS);
    if (tx_status != HAL_OK) {
        if (lora_state.tx_fail_streak < 255u) {
            lora_state.tx_fail_streak++;
        }
        if (s_lora_verbose) {
            printf("[LORA] Raw TX failed (status=%d)\r\n", (int)tx_status);
        }
        return;
    }

    strncpy(lora_state.last_tx_payload, msg, sizeof(lora_state.last_tx_payload) - 1);
    lora_state.last_tx_payload[sizeof(lora_state.last_tx_payload) - 1] = '\0';
    lora_state.last_tx_ms = HAL_GetTick();
    lora_state.tx_count++;
    lora_state.tx_fail_streak = 0;
    if (s_lora_verbose) {
        printf("[LORA] Raw TX: %s", msg);
    }
}

void LoRA_SetVerbose(uint8_t enable)
{
    s_lora_verbose = enable ? 1u : 0u;
}

uint32_t LoRA_GetLastTxMs(void)
{
    return lora_state.last_tx_ms;
}

uint32_t LoRA_GetLastRxMs(void)
{
    return lora_state.last_rx_ms;
}

uint32_t LoRA_GetTxCount(void)
{
    return lora_state.tx_count;
}

uint32_t LoRA_GetRxCount(void)
{
    return lora_state.rx_count;
}

uint32_t LoRA_GetInvalidCommandCount(void)
{
    return lora_state.invalid_command_count;
}

uint32_t LoRA_GetStartFilterDropCount(void)
{
    return lora_state.start_filter_drop_count;
}

uint32_t LoRA_GetPrefixRejectCount(void)
{
    return lora_state.prefix_reject_count;
}

uint32_t LoRA_GetRxOverflowCount(void)
{
    return lora_state.rx_overflow_count;
}

uint32_t LoRA_GetIsrQueueOverflowCount(void)
{
    return lora_state.isr_q_overflow_count;
}



