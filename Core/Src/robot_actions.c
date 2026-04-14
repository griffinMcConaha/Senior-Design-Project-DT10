#include "robot_actions.h"
#include "sabertooth.h"
#include "robot_sm.h"
#include "proximity.h"
#include "dispersion.h"
#include "imu_icm20948.h"
#include "gps.h"
#include "heading_fusion.h"
#include "system_health.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// ============================================================================
// MANUAL CONTROL IMPLEMENTATION
// ============================================================================

// Parse manual speed command (e.g., "M1:32 M2:-16") and set motor speeds
void ParseManualSpeedCommand(const char *cmd)
{
    // Input validation
    if (!cmd || *cmd == 0)
    {
        printf("[RC] Invalid command (empty)\r\n");
        return;
    }

    if (strlen(cmd) > 100)
    {
        printf("[RC] Invalid command (too long)\r\n");
        return;
    }

    // Parse M1 and M2 values from format "M1:<speed> M2:<speed>"
    int m1_speed = 0, m2_speed = 0;
    int parsed = sscanf(cmd, "M1:%d M2:%d", &m1_speed, &m2_speed);

    if (parsed != 2)
    {
        printf("[RC] Parse error - expected format: 'M1:<speed> M2:<speed>'\r\n");
        printf("[RC] Example: M1:32 M2:-16\r\n");
        return;
    }

    // Clamp speeds to valid range [-100, +100] (percent)
    if (m1_speed < -100)
    {
        printf("[RC] CLAMP: M1 speed %d -> -100 (out of range)\r\n", m1_speed);
        m1_speed = -100;
    }
    if (m1_speed > 100)
    {
        printf("[RC] CLAMP: M1 speed %d -> 100 (out of range)\r\n", m1_speed);
        m1_speed = 100;
    }

    if (m2_speed < -100)
    {
        printf("[RC] CLAMP: M2 speed %d -> -100 (out of range)\r\n", m2_speed);
        m2_speed = -100;
    }
    if (m2_speed > 100)
    {
        printf("[RC] CLAMP: M2 speed %d -> 100 (out of range)\r\n", m2_speed);
        m2_speed = 100;
    }

    // Apply motor commands to Sabertooth driver
    Sabertooth_SetM1(m1_speed);
    Sabertooth_SetM2(m2_speed);

    printf("[RC] Motors: M1=%d  M2=%d\r\n", m1_speed, m2_speed);
}

// Manual control mode: operator input drives motors directly
void ManualControl_Task(void)
{
    // Log entry once to indicate mode transition
    static uint8_t once = 0;
    if (!once)
    {
        printf("[RC] Manual control enabled - ready for speed commands\r\n");
        printf("[RC] Format: M1:<speed> M2:<speed>  (speed range: -100 to +100)\r\n");
        printf("[RC] Example: M1:50 M2:50  (drive forward)\r\n");
        once = 1;
    }

    // Optional: Check sensor health periodically (every 1Hz)
    static uint32_t last_health_check = 0;
    static uint8_t last_logged_imu_ok = 1;
    static uint8_t last_logged_gps_fix = 1;
    uint32_t now = HAL_GetTick();

    if ((now - last_health_check) >= 10000)
    {
         // Use debounced health state from main loop instead of issuing an
         // extra IMU bus read in manual mode.
         const SystemHealthState_t *hs = SystemHealth_GetState();
         const uint8_t imu_is_ok = (hs->sensor_status[SENSOR_IMU] == SENSOR_OK) ? 1u : 0u;
         const char *imu_s = imu_is_ok ? "OK" : "FAIL";
        const GPS_Data_t *gps = GPS_Get();
        const uint8_t gps_has_fix = gps->has_fix ? 1u : 0u;

        if (imu_is_ok != last_logged_imu_ok || gps_has_fix != last_logged_gps_fix || last_health_check == 0u) {
            printf("[RC] Health check - IMU: %s  GPS: %s\r\n",
                   imu_s, gps_has_fix ? "FIX" : "NO_FIX");
            last_logged_imu_ok = imu_is_ok;
            last_logged_gps_fix = gps_has_fix;
        }
        last_health_check = now;

        // Optional: Detect and report sensor failures
        // Could call: RobotSM_SetFault(&g_sm, FAULT_IMU_TIMEOUT) if imu fails
        // This would trigger automatic transition to STATE_ERROR
    }

    // ParseManualSpeedCommand is called from Console_ProcessCommand
    // when user sends RC input via serial console or mobile app
}

// ============================================================================
// AUTONOMOUS CONTROL IMPLEMENTATION (PHASE 3)
// ============================================================================

// Helper: Calculate distance between two GPS points (simplified, assumes small area)
// For small areas (<10km), treats lat/lon as cartesian with slight error
static float GPS_Distance(float lat1, float lon1, float lat2, float lon2)
{
    // Convert degrees to meters (rough estimate for small areas)
    float lat_diff = (lat2 - lat1) * 111000.0f; // 1 degree latitude ≈ 111 km
    float lon_diff = (lon2 - lon1) * 111000.0f * cosf((lat1 + lat2) / 2.0f * 3.14159f / 180.0f);
    return sqrtf(lat_diff * lat_diff + lon_diff * lon_diff);
}

// Helper: Calculate desired heading from current to target position (0-360 degrees)
static float GPS_Heading(float lat1, float lon1, float lat2, float lon2)
{
    float lat_diff = (lat2 - lat1) * 111000.0f;
    float lon_diff = (lon2 - lon1) * 111000.0f * cosf((lat1 + lat2) / 2.0f * 3.14159f / 180.0f);
    float heading = atan2f(lon_diff, lat_diff) * 180.0f / 3.14159f;
    
    // Normalize to 0-360 range
    if (heading < 0.0f) heading += 360.0f;
    return heading;
}

// Helper: Normalize heading error to [-180, +180] range
static float Heading_Error(float desired, float actual)
{
    float error = desired - actual;
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

// Helper: Simple proportional controller for motor speed difference based on heading error
// Returns: speed difference to apply to motors (positive = turn right, negative = turn left)
static int PID_HeadingControl(float heading_error_deg)
{
    // Simple proportional controller: error -> speed difference
    // Gain tuning: 0.5 to 1.0 typically (adjust based on robot responsiveness)
    float base_speed_diff = heading_error_deg * 0.5f;
    
    // Clamp to reasonable range (±30 speed difference)
    if (base_speed_diff > 30.0f) base_speed_diff = 30.0f;
    if (base_speed_diff < -30.0f) base_speed_diff = -30.0f;
    
    return (int)base_speed_diff;
}

#define AUTO_AVOID_APPROACH_MS 500u
#define AUTO_AVOID_BYPASS_MS   1200u
#define AUTO_AVOID_REJOIN_MS   700u
#define AUTO_AVOID_PINCH_MS    1500u
#define AUTO_AVOID_SIDE_NONE   0
#define AUTO_AVOID_SIDE_LEFT  -1
#define AUTO_AVOID_SIDE_RIGHT  1

static uint8_t avoid_phase = 0;
static int8_t avoid_side = AUTO_AVOID_SIDE_NONE;
static uint32_t avoid_started_ms = 0;
static uint32_t avoid_pinch_started_ms = 0;

// Autonomous control mode: GPS path following with heading correction
void AutonomousControl_Task(void)
{
    // Log entry once to indicate mode transition
    static uint8_t once = 0;
    if (!once)
    {
        printf("[AUTO] Autonomous mission control enabled\r\n");
        printf("[AUTO] Following GPS waypoints with heading correction\r\n");
        once = 1;
    }

    // Get global state machine pointer (extern from main.c)
    extern RobotSM_t g_sm;
    
    // Mission must be loaded before entering STATE_AUTO
    if (!g_sm.mission.waypoints || g_sm.mission.total_waypoints == 0)
    {
        printf("[AUTO] ERROR: No mission loaded\r\n");
        RobotSM_SetFault(&g_sm, FAULT_GENERIC);
        return;
    }

    // Get current sensor readings
    IMU_Status_t imu = IMU_Read();
    const GPS_Data_t *gps = GPS_Get();

    // Check sensor health - if IMU fails, cannot determine heading
    if (!imu.ok)
    {
        printf("[AUTO] ERROR: IMU not responding\r\n");
        RobotSM_SetFault(&g_sm, FAULT_IMU_TIMEOUT);
        return;
    }

    // Check GPS health - if no fix, cannot navigate
    if (!gps->has_fix)
    {
        printf("[AUTO] WARNING: No GPS fix (using dead reckoning)\r\n");
        // Continue anyway with last known position (dead reckoning mode)
        // Could trigger FAULT_GPS_LOSS if timeout exceeded
    }

    // Get current waypoint index
    uint16_t wp_index = g_sm.mission.current_index;
    if (wp_index >= g_sm.mission.total_waypoints)
    {
        // Mission complete
        printf("[AUTO] Mission complete: all %d waypoints reached\r\n", 
               (int)g_sm.mission.total_waypoints);
        Stop_Motors();
        RobotSM_Request(&g_sm, STATE_PAUSE);
        return;
    }

    // Get target waypoint
    Waypoint_t *target = &g_sm.mission.waypoints[wp_index];

    // Calculate distance to waypoint (meters)
    float distance_to_waypoint = GPS_Distance(gps->latitude_deg, gps->longitude_deg,
                                              target->latitude, target->longitude);

    // Waypoint completion threshold (2 meters)
    float waypoint_threshold = 2.0f;

    // Check if waypoint reached
    if (distance_to_waypoint < waypoint_threshold)
    {
        printf("[AUTO] Waypoint %d/%d reached (%.1f m)\r\n",
               (int)wp_index + 1, (int)g_sm.mission.total_waypoints, distance_to_waypoint);
        
        // Apply dispersion rates from waypoint before advancing
        // PHASE 4: Convert float rates (0.0-1.0) to percentages (0-100) for ESP32
        uint8_t salt_pct = (uint8_t)(target->salt_rate * 100.0f + 0.5f);  // Round to nearest
        uint8_t brine_pct = (uint8_t)(target->brine_rate * 100.0f + 0.5f);
        Dispersion_SetRate(salt_pct, brine_pct);
        
        // Advance to next waypoint
        if (!RobotSM_AdvanceWaypoint(&g_sm))
        {
            // Mission complete
            printf("[AUTO] Mission complete!\r\n");
            Stop_Motors();
            Dispersion_SetRate(0, 0); // Stop dispensing
            RobotSM_Request(&g_sm, STATE_PAUSE);
        }
        return;
    }

    // Calculate desired heading to waypoint
    float desired_heading = GPS_Heading(gps->latitude_deg, gps->longitude_deg,
                                       target->latitude, target->longitude);

    // Get current heading from IMU+GPS fusion (yaw angle)
    extern HeadingFusion_t g_hf;
    float current_heading = g_hf.yaw_deg; // Range: -180 to +180 or 0 to 360

    // Calculate heading error (normalize to [-180, +180])
    float heading_error = Heading_Error(desired_heading, current_heading);

    // Apply dispersion rates from current waypoint
    // PHASE 4: Convert float rates (0.0-1.0) to percentages (0-100) for ESP32
    uint8_t salt_pct_ongoing = (uint8_t)(target->salt_rate * 100.0f + 0.5f);
    uint8_t brine_pct_ongoing = (uint8_t)(target->brine_rate * 100.0f + 0.5f);
    Dispersion_SetRate(salt_pct_ongoing, brine_pct_ongoing);

    // Motor control: base forward speed with heading correction
    int base_speed = 40; // Moderate forward speed
    int speed_diff = PID_HeadingControl(heading_error);

    // Apply heading correction to motors
    int m1_speed = base_speed - speed_diff; // Left motor
    int m2_speed = base_speed + speed_diff; // Right motor

    // Clamp motor speeds to valid range [-100, +100]
    if (m1_speed > 100) m1_speed = 100;
    if (m1_speed < -100) m1_speed = -100;
    if (m2_speed > 100) m2_speed = 100;
    if (m2_speed < -100) m2_speed = -100;

    // Set motor speeds
    Sabertooth_SetM1(m1_speed);
    Sabertooth_SetM2(m2_speed);

    // Proximity sensor monitoring (obstacle avoidance)
    uint16_t prox_left = Proximity_ReadLeft();
    uint16_t prox_right = Proximity_ReadRight();
    // Check for obstacles
    uint8_t status_left = Proximity_GetStatus(prox_left);
    uint8_t status_right = Proximity_GetStatus(prox_right);

    uint32_t now = HAL_GetTick();

    if (status_left >= 2 || status_right >= 2)
    {
        // Critical distance - emergency stop.
        printf("[AUTO] OBSTACLE: Critical distance detected (%u cm, %u cm)\r\n",
               prox_left, prox_right);
        Stop_Motors();
        avoid_phase = 0;
        avoid_side = AUTO_AVOID_SIDE_NONE;
        avoid_pinch_started_ms = 0;
        RobotSM_SetFault(&g_sm, FAULT_PROXIMITY_CRIT);
        return;
    }

    if (status_left >= 1 || status_right >= 1)
    {
        if (status_left >= 1 && status_right >= 1)
        {
            // Both sides are constrained: creep briefly, then stop and fault if the path stays pinched.
            int creep_speed = base_speed / 3;
            Sabertooth_SetM1(creep_speed);
            Sabertooth_SetM2(creep_speed);
            avoid_side = AUTO_AVOID_SIDE_NONE;
            avoid_phase = 0;
            if (avoid_pinch_started_ms == 0)
            {
                avoid_pinch_started_ms = now;
            }
            else if ((now - avoid_pinch_started_ms) >= AUTO_AVOID_PINCH_MS)
            {
                printf("[AUTO] OBSTACLE: Warning on both sides persisted (%u cm, %u cm)\r\n",
                       prox_left, prox_right);
                Stop_Motors();
                avoid_phase = 0;
                avoid_side = AUTO_AVOID_SIDE_NONE;
                avoid_pinch_started_ms = 0;
                RobotSM_SetFault(&g_sm, FAULT_PROXIMITY_CRIT);
                return;
            }
        }
        else
        {
            int8_t detected_side = status_left >= 1 ? AUTO_AVOID_SIDE_LEFT : AUTO_AVOID_SIDE_RIGHT;
            if (avoid_side != detected_side)
            {
                avoid_side = detected_side;
                avoid_phase = 1;
                avoid_started_ms = now;
                avoid_pinch_started_ms = 0;
            }

            int reduced_speed = base_speed / 2;
            if (avoid_phase == 1)
            {
                // Phase 1: move forward briefly to get close enough to the obstacle edge before bypassing.
                Sabertooth_SetM1(reduced_speed);
                Sabertooth_SetM2(reduced_speed);
                if ((now - avoid_started_ms) >= AUTO_AVOID_APPROACH_MS)
                {
                    avoid_phase = 2;
                    avoid_started_ms = now;
                }
            }
            else if (avoid_phase == 2)
            {
                // Phase 2: bias away from the blocked side to clear the obstacle edge.
                int steer_inner = reduced_speed / 3;
                int steer_outer = reduced_speed;
                if (avoid_side == AUTO_AVOID_SIDE_LEFT)
                {
                    Sabertooth_SetM1(steer_inner);
                    Sabertooth_SetM2(steer_outer);
                }
                else
                {
                    Sabertooth_SetM1(steer_outer);
                    Sabertooth_SetM2(steer_inner);
                }

                if ((now - avoid_started_ms) >= AUTO_AVOID_BYPASS_MS)
                {
                    avoid_phase = 3;
                    avoid_started_ms = now;
                }
            }
            else
            {
                // Phase 3: gently rejoin the planned line instead of snapping straight back to waypoint tracking.
                int steer_inner = reduced_speed / 2;
                int steer_outer = reduced_speed;
                if (avoid_side == AUTO_AVOID_SIDE_LEFT)
                {
                    Sabertooth_SetM1(steer_outer);
                    Sabertooth_SetM2(steer_inner);
                }
                else
                {
                    Sabertooth_SetM1(steer_inner);
                    Sabertooth_SetM2(steer_outer);
                }

                if ((now - avoid_started_ms) >= AUTO_AVOID_REJOIN_MS)
                {
                    avoid_phase = 0;
                    avoid_side = AUTO_AVOID_SIDE_NONE;
                }
            }
        }
    }
    else
    {
        avoid_phase = 0;
        avoid_side = AUTO_AVOID_SIDE_NONE;
        avoid_pinch_started_ms = 0;
    }

    // Periodic status logging (1 Hz)
    static uint32_t last_log = 0;
    if ((now - last_log) >= 1000)
    {
        last_log = now;
        printf("[AUTO] WP %d: dist=%.1f m, heading=%.0f (error=%.0f), motors=%d/%d\r\n",
               (int)wp_index + 1, distance_to_waypoint, current_heading, heading_error,
               m1_speed, m2_speed);
    }
}

// ============================================================================
// ERROR & EMERGENCY HANDLING
// ============================================================================

// Immediate stop: both motors to stop value (fail-safe)
void Stop_Motors(void)
{
    Sabertooth_StopAll();
    // All motors now stopped; dispersion also stopped by state machine
}

// Error state handler: log fault and keep motors stopped
void Handle_Error(void)
{
    // Ensure motors are stopped
    Sabertooth_StopAll();

    // Log every 10 seconds - frequent enough to notice, infrequent enough not to spam
    static uint32_t last_log = 0;
    uint32_t now = HAL_GetTick();

    if ((now - last_log) >= 10000)
    {
        last_log = now;

        // Get the active fault code from state machine
        extern RobotSM_t g_sm;
        FaultCode_t fault = RobotSM_GetFault(&g_sm);

        // Report which sensors are causing the error for easier diagnosis
        const SystemHealthState_t *hs = SystemHealth_GetState();
        const char *imu_s = (hs->sensor_status[SENSOR_IMU] == SENSOR_OK) ? "OK" : "FAIL";
        const char *gps_s = (hs->sensor_status[SENSOR_GPS] == SENSOR_OK) ? "OK" : "no-fix";

        printf("[ERROR] State: ERROR  Fault=%d  IMU=%s  GPS=%s\r\n", (int)fault, imu_s, gps_s);
        printf("[ERROR] Send 'PAUSE' to recover (or 'T' at boot for test mode)\r\n");
    }

    // TODO: Future enhancements
    // - Latch error status to persistent memory
    // - Attempt sensor recovery (e.g., IMU recalibration)
    // - Log detailed diagnostics (accel/gyro bias, GPS quality, etc.)
}

// Emergency stop handler: critical failure or user button, latched until reset
static uint8_t s_estop_logged = 0;

void Emergency_Stop_ResetOnce(void)
{
    s_estop_logged = 0;
}

void Emergency_Stop(void)
{
    // Ensure motors are stopped immediately
    Sabertooth_StopAll();

    // Log ESTOP activation once per latch cycle.
    // s_estop_logged is reset by Emergency_Stop_ResetOnce() when the ESTOP
    // latch is cleared (ESTOP -> PAUSE), so the next ESTOP always prints.
    if (!s_estop_logged)
    {
        printf("[ESTOP] EMERGENCY STOP ACTIVATED\r\n");
        printf("[ESTOP] Motor shutdown complete\r\n");
        printf("[ESTOP] System is LATCHED - send 'PAUSE' then manual command to recover\r\n");
        s_estop_logged = 1;
    }

    // Note: ESTOP latch is managed by state machine (RobotSM_HandleTransitions)
    // Cannot exit this state without going through PAUSE first
}
