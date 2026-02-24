#include "robot_sm.h"
#include "robot_actions.h"
#include "stdio.h"

// Convert fault code to human-readable string for logging
static const char* FaultCodeToString(FaultCode_t fault)
{
    switch (fault)
    {
        case FAULT_NONE:            return "NONE";
        case FAULT_IMU_TIMEOUT:     return "IMU_TIMEOUT";
        case FAULT_GPS_LOSS:        return "GPS_LOSS";
        case FAULT_MOTOR_FEEDBACK:  return "MOTOR_FEEDBACK";
        case FAULT_BATTERY_COLD:    return "BATTERY_COLD";
        case FAULT_PROXIMITY_WARN:  return "PROXIMITY_WARN";
        case FAULT_PROXIMITY_CRIT:  return "PROXIMITY_CRIT";
        case FAULT_DISPERSION:      return "DISPERSION";
        case FAULT_GENERIC:         return "GENERIC";
        default:                    return "UNKNOWN";
    }
}

// Convert state enum to human-readable string for logging
static const char* StateToString(RobotState_t state)
{
    switch (state)
    {
        case STATE_MANUAL: return "MANUAL";
        case STATE_AUTO:   return "AUTO";
        case STATE_PAUSE:  return "PAUSE";
        case STATE_ERROR:  return "ERROR";
        case STATE_ESTOP:  return "ESTOP";
        default:           return "UNKNOWN";
    }
}

// Called when entering a new state; enforces safe motor behavior by state
void RobotSM_OnEnter(RobotState_t s, RobotSM_t *sm)
{
    printf("[SM] Entering state: %s\r\n", StateToString(s));

    switch (s)
    {
        case STATE_ESTOP:
            // Emergency stop: immediate motor shutdown, latched
            Stop_Motors();
            if (sm) sm->estop_latched = true;
            printf("[SM] ESTOP LATCHED - manual reset required\r\n");
            break;

        case STATE_ERROR:
            // Error state: stop motors, preserve fault code for diagnostics
            Stop_Motors();
            if (sm)
                printf("[SM] Error state: fault=%s\r\n", FaultCodeToString(sm->fault_code));
            break;

        case STATE_PAUSE:
            // Safe idle: stop motors, wait for next command
            Stop_Motors();
            printf("[SM] Safe idle, ready for next command\r\n");
            break;

        case STATE_MANUAL:
            // Manual control: motors disabled until operator sends commands
            Stop_Motors();
            printf("[SM] Manual control enabled - waiting for RC input\r\n");
            break;

        case STATE_AUTO:
            // Autonomous mode: prepare mission, reset waypoint index
            Stop_Motors();
            if (sm)
            {
                if (sm->mission.waypoints && sm->mission.total_waypoints > 0)
                {
                    sm->mission.current_index = 0;
                    sm->mission.mission_active = true;
                    sm->mission.start_time_ms = 0; // Populated by AutonomousControl_Task
                    printf("[SM] Autonomous mission loaded: %d waypoints\r\n",
                           (int)sm->mission.total_waypoints);
                }
                else
                {
                    printf("[SM] ERROR: No mission loaded! Returning to PAUSE.\r\n");
                    sm->requested = STATE_PAUSE;
                }
            }
            break;

        default:
            Stop_Motors();
            break;
    }
}

// Initialize state machine to given state (safe default is STATE_PAUSE)
void RobotSM_Init(RobotSM_t *sm, RobotState_t initial_state)
{
    if (!sm) return;

    sm->current = initial_state;
    sm->requested = initial_state;
    sm->prev = initial_state;
    sm->fault_code = FAULT_NONE;
    sm->fault_timestamp_ms = 0;
    sm->estop_latched = false;

    // Initialize mission state
    sm->mission.waypoints = NULL;
    sm->mission.total_waypoints = 0;
    sm->mission.current_index = 0;
    sm->mission.mission_active = false;
    sm->mission.start_time_ms = 0;

    RobotSM_OnEnter(initial_state, sm);
    printf("[SM] State machine initialized to %s\r\n", StateToString(initial_state));
}

// Request a state change; validation happens in HandleTransitions
void RobotSM_Request(RobotSM_t *sm, RobotState_t new_state)
{
    if (!sm) return;

    if (new_state != sm->requested)
    {
        printf("[SM] State transition requested: %s -> %s\r\n",
               StateToString(sm->current), StateToString(new_state));
        sm->requested = new_state;
    }
}

// Set fault code and record timestamp (transition to ERROR is handled here)
void RobotSM_SetFault(RobotSM_t *sm, FaultCode_t fault)
{
    if (!sm) return;

    sm->fault_code = fault;
    sm->fault_timestamp_ms = 0; // TODO: get actual system time
    printf("[SM] Fault detected: %s\r\n", FaultCodeToString(fault));

    // Request transition to ERROR state
    sm->requested = STATE_ERROR;
}

// Clear fault code and allow recovery
void RobotSM_ClearFault(RobotSM_t *sm)
{
    if (!sm) return;

    printf("[SM] Fault cleared: %s -> NONE\r\n", FaultCodeToString(sm->fault_code));
    sm->fault_code = FAULT_NONE;
    sm->fault_timestamp_ms = 0;
}

// Get current active state
RobotState_t RobotSM_Current(const RobotSM_t *sm)
{
    return sm ? sm->current : STATE_ERROR;
}

// Get requested state waiting for transition
RobotState_t RobotSM_Requested(const RobotSM_t *sm)
{
    return sm ? sm->requested : STATE_ERROR;
}

// Get current fault code
FaultCode_t RobotSM_GetFault(const RobotSM_t *sm)
{
    return sm ? sm->fault_code : FAULT_NONE;
}

// Load mission waypoints
void RobotSM_LoadMission(RobotSM_t *sm, Waypoint_t *waypoints, uint16_t count)
{
    if (!sm || !waypoints || count == 0) return;

    sm->mission.waypoints = waypoints;
    sm->mission.total_waypoints = count;
    sm->mission.current_index = 0;
    sm->mission.mission_active = false;

    printf("[SM] Mission loaded: %d waypoints\r\n", (int)count);
}

// Reset mission progress
void RobotSM_ResetMission(RobotSM_t *sm)
{
    if (!sm || !sm->mission.waypoints) return;

    sm->mission.current_index = 0;
    sm->mission.mission_active = false;
    printf("[SM] Mission reset to waypoint 0\r\n");
}

// Advance to next waypoint (returns true if more exist, false if complete)
bool RobotSM_AdvanceWaypoint(RobotSM_t *sm)
{
    if (!sm || !sm->mission.waypoints) return false;

    sm->mission.current_index++;
    bool has_more = (sm->mission.current_index < sm->mission.total_waypoints);

    if (has_more)
    {
        printf("[SM] Advanced to waypoint %d/%d\r\n",
               (int)sm->mission.current_index + 1,
               (int)sm->mission.total_waypoints);
    }
    else
    {
        printf("[SM] Mission complete - all waypoints reached\r\n");
        sm->mission.mission_active = false;
    }

    return has_more;
}

// Validate and apply state transitions with comprehensive safety checks
void RobotSM_HandleTransitions(RobotSM_t *sm)
{
    if (!sm) return;

    // No transition needed if already in requested state
    if (sm->requested == sm->current)
        return;

    RobotState_t from = sm->current;
    RobotState_t to = sm->requested;

    // SAFETY RULE 1: ESTOP latches - can only exit via explicit reset through PAUSE
    if (sm->estop_latched && to != STATE_PAUSE)
    {
        printf("[SM] SAFETY: ESTOP latched - must transition through PAUSE first\r\n");
        return;
    }

    // SAFETY RULE 2: Cannot transition directly from ESTOP/ERROR to MANUAL/AUTO
    // Must go through PAUSE intermediate state
    if ((from == STATE_ESTOP || from == STATE_ERROR) &&
        (to == STATE_MANUAL || to == STATE_AUTO))
    {
        printf("[SM] SAFETY: Cannot go directly from %s to %s (must use PAUSE as intermediate)\r\n",
               StateToString(from), StateToString(to));
        return;
    }

    // SAFETY RULE 3: Allow recovery from ERROR via PAUSE
    if (from == STATE_ERROR && to == STATE_PAUSE)
    {
        RobotSM_ClearFault(sm);
    }

    // SAFETY RULE 4: Exiting ESTOP through PAUSE clears latch
    if (from == STATE_ESTOP && to == STATE_PAUSE)
    {
        sm->estop_latched = false;
        printf("[SM] ESTOP latch cleared - robot ready for commands\r\n");
    }

    // All validations passed - perform transition
    sm->prev = sm->current;
    sm->current = sm->requested;

    printf("[SM] STATE CHANGE: %s -> %s\r\n", StateToString(sm->prev), StateToString(sm->current));

    RobotSM_OnEnter(sm->current, sm);
}

// Dispatch to state-specific control task based on current state
void RobotSM_Update(RobotSM_t *sm)
{
    if (!sm) return;

    switch (sm->current)
    {
        case STATE_MANUAL:
            ManualControl_Task();
            break;

        case STATE_AUTO:
            AutonomousControl_Task();
            break;

        case STATE_PAUSE:
            Stop_Motors();
            break;

        case STATE_ERROR:
            Handle_Error();
            break;

        case STATE_ESTOP:
            Emergency_Stop();
            break;

        default:
            /* Unknown state -> fail safe */
            Stop_Motors();
            printf("[SM] Unknown state %d - defaulting to safe stop\r\n", (int)sm->current);
            break;
    }
}
