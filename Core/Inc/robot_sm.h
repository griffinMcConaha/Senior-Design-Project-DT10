#ifndef ROBOT_SM_H
#define ROBOT_SM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Five operational states: manual control, autonomous, paused, sensor failure, emergency stop
typedef enum {
    STATE_MANUAL = 0,  // Operator directly controls motors via RC/app
    STATE_AUTO   = 1,  // Autonomous mode (GPS waypoint following)
    STATE_PAUSE  = 2,  // All motors stopped, safe idle state
    STATE_ERROR  = 3,  // Recoverable sensor failure, motors stopped
    STATE_ESTOP  = 4   // Emergency stop pressed, motors stopped (latched)
} RobotState_t;

// Fault code enumeration for STATE_ERROR diagnostics
typedef enum {
    FAULT_NONE          = 0,  // No fault
    FAULT_IMU_TIMEOUT   = 1,  // IMU accel/gyro not updating
    FAULT_GPS_LOSS      = 2,  // GPS fix lost for >30 seconds
    FAULT_MOTOR_FEEDBACK= 3,  // Encoder/flow sensor failure
    FAULT_BATTERY_COLD  = 4,  // Battery temperature <0°C (charging inhibited)
    FAULT_PROXIMITY_WARN= 5,  // Object detected <30cm (warning)
    FAULT_PROXIMITY_CRIT= 6,  // Object detected <10cm (critical)
    FAULT_DISPERSION    = 7,  // Salt/brine system error
    FAULT_GENERIC       = 255 // Unknown/unclassified error
} FaultCode_t;

// Waypoint definition for autonomous missions
typedef struct {
    float latitude;        // GPS latitude coordinate
    float longitude;       // GPS longitude coordinate
    float salt_rate;       // Salt dispersion 0.0 to 1.0
    float brine_rate;      // Brine dispersion 0.0 to 1.0
} Waypoint_t;

// Mission state for autonomous mode tracking
typedef struct {
    Waypoint_t *waypoints;  // Array of waypoints
    uint16_t total_waypoints; // Total waypoints in mission
    uint16_t current_index;   // Current waypoint index (0 to total-1)
    bool mission_active;      // True if mission in progress
    uint32_t start_time_ms;   // Timestamp when mission started
} MissionState_t;

// State machine struct: tracks current, requested, previous states + diagnostics
typedef struct {
    RobotState_t current;        // Active state
    RobotState_t requested;      // Next requested state
    RobotState_t prev;           // Previous state for debugging
    FaultCode_t fault_code;      // Active fault code (if in STATE_ERROR)
    uint32_t fault_timestamp_ms; // When fault occurred
    MissionState_t mission;      // Mission data for autonomous mode
    bool estop_latched;          // True if ESTOP activated (blocks auto-recovery)
} RobotSM_t;

// Initialize state machine with given starting state
void RobotSM_Init(RobotSM_t *sm, RobotState_t initial_state);

// Request a state transition (validated during HandleTransitions)
void RobotSM_Request(RobotSM_t *sm, RobotState_t new_state);

// Set fault code and record timestamp (called when transitioning to STATE_ERROR)
void RobotSM_SetFault(RobotSM_t *sm, FaultCode_t fault);

// Clear fault code and allow recovery from STATE_ERROR
void RobotSM_ClearFault(RobotSM_t *sm);

// Validate and apply state transitions with safety checks (blocks invalid paths)
void RobotSM_HandleTransitions(RobotSM_t *sm);

// Dispatch to state-specific control task based on current state
void RobotSM_Update(RobotSM_t *sm);

// Get current state
RobotState_t RobotSM_Current(const RobotSM_t *sm);

// Get requested state
RobotState_t RobotSM_Requested(const RobotSM_t *sm);

// Get current fault code (returns FAULT_NONE if no error)
FaultCode_t RobotSM_GetFault(const RobotSM_t *sm);

// Called when entering a new state; stops motors in safe states
void RobotSM_OnEnter(RobotState_t s, RobotSM_t *sm);

// Load mission waypoints (call before requesting STATE_AUTO)
void RobotSM_LoadMission(RobotSM_t *sm, Waypoint_t *waypoints, uint16_t count);

// Reset mission progress to waypoint 0
void RobotSM_ResetMission(RobotSM_t *sm);

// Advance to next waypoint (returns true if more waypoints, false if complete)
bool RobotSM_AdvanceWaypoint(RobotSM_t *sm);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_SM_H
