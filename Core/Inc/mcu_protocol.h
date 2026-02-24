/*
 * mcu_protocol.h
 *
 * Multi-MCU Communication Protocol Definition
 *
 * This file defines the communication protocol for inter-MCU communication
 * between the main STM32F407 controller and additional microcontrollers
 * (e.g., sensor processors, auxiliary controllers).
 *
 * Transport: CAN Bus (primary) or UART (secondary/fallback)
 * 
 * Created on: Jan 2025
 */

#ifndef MCU_PROTOCOL_H
#define MCU_PROTOCOL_H

#include <stdint.h>
#include <string.h>

// ============================================================================
// PROTOCOL VERSION AND CONFIGURATION
// ============================================================================

#define MCU_PROTOCOL_VERSION        0x01        // Protocol version 1.0
#define MCU_MAX_PAYLOAD_SIZE        64          // Maximum payload size (bytes)
#define MCU_PACKET_TIMEOUT_MS       5000        // 5 second timeout for responses
#define MCU_HEARTBEAT_INTERVAL_MS   1000        // Send heartbeat every 1 second
#define MCU_MAX_RETRIES             3           // Max retry attempts

// ============================================================================
// MCU IDENTIFIERS
// ============================================================================

typedef enum {
    MCU_ID_MAIN         = 0x01,    // Main STM32F407 controller
    MCU_ID_SENSOR_1     = 0x02,    // Sensor processor 1 (e.g., IMU/GPS fusion)
    MCU_ID_SENSOR_2     = 0x03,    // Sensor processor 2 (e.g., advanced dispersion)
    MCU_ID_POWER        = 0x04,    // Power management controller
    MCU_ID_COMMS        = 0x05,    // Extended communications controller
} MCU_ID_t;

// ============================================================================
// MESSAGE TYPES
// ============================================================================

typedef enum {
    // System messages
    MSG_HEARTBEAT           = 0x00,    // Periodic heartbeat (keep-alive)
    MSG_ACK                 = 0x01,    // Acknowledgement response
    MSG_NAK                 = 0x02,    // Negative acknowledgement (error)
    MSG_RESET               = 0x03,    // System reset command
    
    // Status/Health queries
    MSG_STATUS_REQUEST      = 0x10,    // Request health/status report
    MSG_STATUS_RESPONSE     = 0x11,    // Health/status response
    MSG_FAULT_REPORT        = 0x12,    // Asynchronous fault notification
    
    // Sensor data
    MSG_SENSOR_DATA         = 0x20,    // Sensor reading data
    MSG_CALIBRATION_DATA    = 0x21,    // Calibration/baseline data
    
    // Motor/Actuation commands
    MSG_MOTOR_COMMAND       = 0x30,    // Motor speed/control command
    MSG_MOTOR_FEEDBACK      = 0x31,    // Motor feedback (current, temp, etc)
    
    // Dispersion commands
    MSG_DISPERSION_COMMAND  = 0x40,    // Salt/brine dispensing command
    MSG_DISPERSION_STATUS   = 0x41,    // Dispersion system status
    
    // Mission/Navigation
    MSG_MISSION_COMMAND     = 0x50,    // Mission waypoint/command
    MSG_MISSION_STATUS      = 0x51,    // Mission status update
    
    // Configuration
    MSG_CONFIG_SET          = 0x60,    // Set configuration parameter
    MSG_CONFIG_GET          = 0x61,    // Get configuration parameter
    MSG_CONFIG_RESPONSE     = 0x62,    // Configuration response
} MessageType_t;

// ============================================================================
// PACKET STRUCTURE
// ============================================================================

typedef struct __attribute__((packed)) {
    // Header (8 bytes)
    uint8_t     start_byte;             // 0xAA (start marker)
    uint8_t     protocol_version;       // MCU_PROTOCOL_VERSION
    uint8_t     from_mcu_id;            // Source MCU ID
    uint8_t     to_mcu_id;              // Destination MCU ID
    uint8_t     message_type;           // Message type from MessageType_t
    uint8_t     message_id;             // Message ID for correlation (request/response)
    uint16_t    payload_length;         // Payload size in bytes (0-MCU_MAX_PAYLOAD_SIZE)
    
    // Payload (variable, max 64 bytes)
    uint8_t     payload[MCU_MAX_PAYLOAD_SIZE];
    
    // Footer (2 bytes)
    uint16_t    checksum;               // CRC16 of header+payload
    uint8_t     end_byte;               // 0x55 (end marker)
} MCU_Packet_t;

#define MCU_PACKET_HEADER_SIZE  8
#define MCU_PACKET_FOOTER_SIZE  3
#define MCU_PACKET_MIN_SIZE     (MCU_PACKET_HEADER_SIZE + MCU_PACKET_FOOTER_SIZE)
#define MCU_PACKET_MAX_SIZE     (MCU_PACKET_HEADER_SIZE + MCU_MAX_PAYLOAD_SIZE + MCU_PACKET_FOOTER_SIZE)

// Start/end markers
#define MCU_START_BYTE          0xAA
#define MCU_END_BYTE            0x55

// ============================================================================
// PAYLOAD STRUCTURES FOR SPECIFIC MESSAGE TYPES
// ============================================================================

// MSG_HEARTBEAT: No payload (just header + checksum)
// Used to verify MCU is alive

// MSG_STATUS_RESPONSE: Health status report
typedef struct __attribute__((packed)) {
    uint8_t     mcu_state;              // 0=OK, 1=DEGRADED, 2=ERROR, 3=CRITICAL
    uint8_t     uptime_sec_hi;          // Uptime in seconds (upper byte)
    uint8_t     uptime_sec_mid;         // Uptime (middle byte)
    uint8_t     uptime_sec_lo;          // Uptime (lower byte)
    int8_t      temperature_c;          // Temperature in Celsius
    uint8_t     error_count;            // Total error count
    uint8_t     reserved[2];            // Reserved for future use
} MCU_StatusPayload_t;

// MSG_SENSOR_DATA: Generic sensor reading
typedef struct __attribute__((packed)) {
    uint8_t     sensor_id;              // Sensor identifier
    uint8_t     sensor_type;            // 0=IMU, 1=GPS, 2=Prox, 3=Temp, etc
    uint8_t     data_length;            // Length of sensor data (1-60 bytes)
    uint8_t     reserved;               // Reserved
    uint8_t     sensor_data[60];        // Variable-length sensor data
} MCU_SensorPayload_t;

// MSG_MOTOR_COMMAND: Motor control
typedef struct __attribute__((packed)) {
    uint8_t     motor_id;               // 1=M1, 2=M2
    int16_t     speed_cmd;              // Speed command (-2047 to +2047) or percentage
    uint8_t     cmd_type;               // 0=speed%, 1=raw_ticks, 2=ramp
    uint8_t     reserved;
} MCU_MotorPayload_t;

// MSG_MOTOR_FEEDBACK: Motor status
typedef struct __attribute__((packed)) {
    uint8_t     motor_id;               // 1=M1, 2=M2
    int16_t     current_cmd;            // Current command value
    uint8_t     temperature;            // Motor temperature (0-150°C, offset by -40)
    uint8_t     error_flags;            // Bit flags: OVR_TEMP, OVER_CURRENT, COMM_ERROR, etc
} MCU_MotorFeedbackPayload_t;

// MSG_DISPERSION_COMMAND: Dispersion system control
typedef struct __attribute__((packed)) {
    uint8_t     dispenser_id;           // 1=SALT, 2=BRINE, 3=BOTH
    uint8_t     rate_percent;           // Dispensing rate (0-100%)
    uint16_t    duration_ms;            // Duration in milliseconds (0=continuous)
    uint8_t     reserved[2];
} MCU_DispersionPayload_t;

// MSG_FAULT_REPORT: Asynchronous fault/error notification
typedef struct __attribute__((packed)) {
    uint8_t     fault_code;             // Fault category code
    uint8_t     fault_severity;         // 0=INFO, 1=WARNING, 2=CRITICAL
    uint8_t     affected_subsystem;     // Subsystem that failed
    uint8_t     timestamp_sec;          // Fault timestamp (relative)
    uint32_t    fault_context;          // Additional context (sensor value, register, etc)
} MCU_FaultPayload_t;

// ============================================================================
// CHECKSUM CALCULATION
// ============================================================================

/**
 * Calculate CRC16 checksum for packet header + payload
 * Uses standard CRC16-CCITT algorithm
 */
static inline uint16_t MCU_CalculateChecksum(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// PACKET CONSTRUCTION HELPERS
// ============================================================================

/**
 * Initialize a packet with header and footer
 */
static inline void MCU_InitPacket(MCU_Packet_t *pkt, 
                                   uint8_t from_id, uint8_t to_id,
                                   uint8_t msg_type, uint8_t msg_id,
                                   const uint8_t *payload, uint16_t payload_len)
{
    if (!pkt || payload_len > MCU_MAX_PAYLOAD_SIZE) return;
    
    pkt->start_byte = MCU_START_BYTE;
    pkt->protocol_version = MCU_PROTOCOL_VERSION;
    pkt->from_mcu_id = from_id;
    pkt->to_mcu_id = to_id;
    pkt->message_type = msg_type;
    pkt->message_id = msg_id;
    pkt->payload_length = payload_len;
    
    if (payload && payload_len > 0) {
        memcpy(pkt->payload, payload, payload_len);
    }
    
    // Calculate checksum (over header + payload, excluding start/end bytes)
    uint8_t csum_data[MCU_PACKET_HEADER_SIZE + MCU_MAX_PAYLOAD_SIZE];
    memcpy(csum_data, (uint8_t *)pkt + 1, MCU_PACKET_HEADER_SIZE - 1);  // Skip start byte
    memcpy(csum_data + (MCU_PACKET_HEADER_SIZE - 1), pkt->payload, payload_len);
    
    pkt->checksum = MCU_CalculateChecksum(csum_data, MCU_PACKET_HEADER_SIZE - 1 + payload_len);
    pkt->end_byte = MCU_END_BYTE;
}

/**
 * Verify packet integrity (markers and checksum)
 */
static inline uint8_t MCU_VerifyPacket(const MCU_Packet_t *pkt)
{
    if (!pkt) return 0;
    
    // Check markers
    if (pkt->start_byte != MCU_START_BYTE) return 0;
    if (pkt->end_byte != MCU_END_BYTE) return 0;
    if (pkt->payload_length > MCU_MAX_PAYLOAD_SIZE) return 0;
    
    // Verify checksum
    uint8_t csum_data[MCU_PACKET_HEADER_SIZE + MCU_MAX_PAYLOAD_SIZE];
    memcpy(csum_data, (uint8_t *)pkt + 1, MCU_PACKET_HEADER_SIZE - 1);
    memcpy(csum_data + (MCU_PACKET_HEADER_SIZE - 1), pkt->payload, pkt->payload_length);
    
    uint16_t calc_checksum = MCU_CalculateChecksum(csum_data, MCU_PACKET_HEADER_SIZE - 1 + pkt->payload_length);
    
    return (pkt->checksum == calc_checksum) ? 1 : 0;
}

// ============================================================================
// TRANSPORT ABSTRACTION
// ============================================================================

/**
 * MCU communication interface
 * Implementations can use CAN, UART, SPI, or other transports
 */
typedef struct {
    uint8_t (*send_packet)(const MCU_Packet_t *pkt);           // Send a packet
    uint8_t (*recv_packet)(MCU_Packet_t *pkt, uint32_t timeout_ms);  // Receive with timeout
    uint8_t (*is_ready)(void);                                  // Check if transport is ready
    void    (*reset)(void);                                     // Reset transport state
} MCU_Transport_t;

// ============================================================================
// ERROR CODES
// ============================================================================

typedef enum {
    MCU_ERR_OK              = 0,    // No error
    MCU_ERR_INVALID_PACKET  = 1,    // Packet structure invalid
    MCU_ERR_CHECKSUM        = 2,    // Checksum mismatch
    MCU_ERR_TIMEOUT         = 3,    // Response timeout
    MCU_ERR_NAK             = 4,    // Negative acknowledgement
    MCU_ERR_TRANSPORT       = 5,    // Transport layer error
    MCU_ERR_UNKNOWN_MSG     = 6,    // Unknown message type
    MCU_ERR_UNKNOWN_MCU     = 7,    // Unknown MCU ID
} MCU_ErrorCode_t;

#endif  // MCU_PROTOCOL_H
