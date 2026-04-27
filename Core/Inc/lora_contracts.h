/*
 * lora_contracts.h
 *
 * LoRa Communication Protocol Contracts
 *
 * Defines the string-based command tokens shared between the STM32 firmware
 * and the base-station / mobile app over the LoRa UART link (UART5).
 *
 * All commands are case-insensitive on the receive side; the firmware
 * normalises incoming bytes to upper case before matching.
 *
 * Command format: plain ASCII strings terminated with CR+LF (\r\n).
 * Responses use the ACK: prefix defined below.
 */

#ifndef LORA_CONTRACTS_H
#define LORA_CONTRACTS_H

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// STATE CONTROL COMMANDS
// Sent by the base station to switch the robot's operating mode.
// ============================================================================

#define LORA_CMD_AUTO   "AUTO"      // Enter autonomous navigation mode
#define LORA_CMD_MANUAL "MANUAL"    // Enter manual (remote-control) mode
#define LORA_CMD_PAUSE  "PAUSE"     // Pause operation; motors stop, state preserved
#define LORA_CMD_ESTOP  "ESTOP"     // Emergency stop; latched until RESET is received
#define LORA_CMD_STOP   "STOP"      // Alias for PAUSE (stops current motion)
#define LORA_CMD_RESET  "RESET"     // Clear ESTOP latch and return to PAUSE state

// ============================================================================
// WAYPOINT MANAGEMENT COMMANDS
// Used to build and commit a mission waypoint list before autonomous mode.
// Each command has a full-name prefix and a shorter alias for bandwidth savings.
//
// Single waypoint:  "WP:<lat>,<lon>"  or  "W:<lat>,<lon>"
// Batch waypoints:  "WPB:<lat1>,<lon1>;<lat2>,<lon2>;..."  or  "WB:..."
// Load & activate:  "WPLOAD:<count>"  or  "WL:<count>"
// Clear all:        "WPCLEAR"         or  "WC"
// ============================================================================

#define LORA_WP_CLEAR              "WPCLEAR"   // Clear the staged waypoint buffer
#define LORA_WP_CLEAR_ALIAS        "WC"        // Short alias for WPCLEAR

#define LORA_WP_ADD_PREFIX         "WP"        // Add a single waypoint; payload: "<lat>,<lon>"
#define LORA_WP_ADD_ALIAS_PREFIX   "W"         // Short alias for WP

#define LORA_WP_BATCH_PREFIX       "WPB"       // Add multiple waypoints in one message
#define LORA_WP_BATCH_ALIAS_PREFIX "WB"        // Short alias for WPB

#define LORA_WP_LOAD_PREFIX        "WPLOAD"    // Commit staged waypoints to the active mission
#define LORA_WP_LOAD_ALIAS_PREFIX  "WL"        // Short alias for WPLOAD

// ============================================================================
// ACKNOWLEDGEMENT RESPONSES
// Sent by the STM32 back to the base station to confirm waypoint operations.
// ============================================================================

#define LORA_WP_ACK_CLEAR        "ACK:WPCLEAR"   // Waypoint buffer cleared successfully (sent as-is)
#define LORA_WP_ACK_ADD_PREFIX   "ACK:WP"        // Base string; firmware appends :<index> at runtime
#define LORA_WP_ACK_BATCH_PREFIX "ACK:WPB"       // Base string; firmware appends :<start>:<count> at runtime
#define LORA_WP_ACK_LOAD_PREFIX  "ACK:WPLOAD"    // Base string; firmware appends :<total_waypoints> at runtime

#ifdef __cplusplus
}
#endif

#endif  // LORA_CONTRACTS_H