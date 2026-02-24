#include "system_health.h"
#include "stm32f4xx_hal.h"
#include "main.h"   // for LD3_Pin/LD4_Pin/LD5_Pin/LD6_Pin and GPIOD
#include "sabertooth.h"
#include <stdio.h>
#include <string.h>

// ANSI color codes for terminal output
#define ANSI_RED      "\033[31m"
#define ANSI_YELLOW   "\033[33m"
#define ANSI_MAGENTA  "\033[35m"
#define ANSI_RESET    "\033[0m"

// ============================================================================
// SYSTEM HEALTH STATE
// ============================================================================

static SystemHealthState_t health_state = {
    .emergency_stop_active = 0,
    .last_healthy_tick_ms = 0,
    .error_count = 0,
    .degraded_sensor_flags = 0
};

// ============================================================================
// INITIALIZATION
// ============================================================================

void SystemHealth_Init(void)
{
    memset(&health_state, 0, sizeof(health_state));
    
    // Initialize all sensors as OK; drivers update this as they initialize and run
    for (int i = 0; i < SENSOR_COUNT; i++) {
        health_state.sensor_status[i] = SENSOR_OK;
    }
    
    health_state.last_healthy_tick_ms = HAL_GetTick();
    printf("[HEALTH] System health monitoring initialized\r\n");
}

// ============================================================================
// EMERGENCY STOP FUNCTIONS
// ============================================================================

void SystemHealth_TriggerEmergencyStop(const char *reason)
{
    if (health_state.emergency_stop_active)
        return;  // Already triggered
    
    health_state.emergency_stop_active = 1;
    health_state.error_count++;
    
    // Kill motors immediately (Sabertooth_StopAll also stops drivetrain and pump)
    Sabertooth_StopAll();
    
    // Log the event
    printf(ANSI_RED "[HEALTH] *** EMERGENCY STOP TRIGGERED ***\r\n");
    printf("[HEALTH] Reason: %s\r\n", reason ? reason : "unknown");
    printf("[HEALTH] Motors disabled, drivetrain stopped" ANSI_RESET "\r\n");
}

uint8_t SystemHealth_ResetEmergencyStop(void)
{
    if (!health_state.emergency_stop_active)
        return 1;  // Not in ESTOP, nothing to reset
    
    // Check if conditions are safe for reset (minimum: IMU + GPS healthy)
    if (health_state.sensor_status[SENSOR_IMU] != SENSOR_OK) {
        printf(ANSI_YELLOW "[HEALTH] Cannot reset ESTOP: IMU not healthy\r\n" ANSI_RESET);
        return 0;
    }
    if (health_state.sensor_status[SENSOR_GPS] != SENSOR_OK) {
        printf(ANSI_YELLOW "[HEALTH] Cannot reset ESTOP: GPS not healthy\r\n" ANSI_RESET);
        return 0;
    }

    health_state.emergency_stop_active = 0;
    
    printf("[HEALTH] Emergency stop reset, resuming normal operation\r\n");
    return 1;
}

uint8_t SystemHealth_IsEmergencyStopActive(void)
{
    return health_state.emergency_stop_active;
}

// ============================================================================
// SENSOR HEALTH TRACKING
// ============================================================================

void SystemHealth_SetSensorStatus(SensorName_t sensor, SensorHealth_t status)
{
    if (sensor >= SENSOR_COUNT)
        return;
    
    SensorHealth_t old_status = health_state.sensor_status[sensor];
    health_state.sensor_status[sensor] = status;
    
    // Track degraded sensors for summary reporting
    if (status == SENSOR_DEGRADED) {
        health_state.degraded_sensor_flags |= (1 << sensor);
    } else if (status == SENSOR_OK) {
        health_state.degraded_sensor_flags &= ~(1 << sensor);
    }
    
    // Log status changes for operator visibility
    const char *sensor_names[] = {"IMU", "GPS", "ProxL", "ProxR", "LoRA", "Sabertooth"};
    const char *status_names[] = {"OK", "TIMEOUT", "INVALID", "DEGRADED"};
    
    if (old_status != status) {
        if (status == SENSOR_OK) {
            printf("[HEALTH] %s: %s (recovered)\r\n", sensor_names[sensor], status_names[status]);
        } else {
            printf(ANSI_YELLOW "[HEALTH] %s: %s\r\n" ANSI_RESET "\r\n", sensor_names[sensor], status_names[status]);
        }
    }

    // Update last healthy tick when all sensors are OK
    uint8_t all_ok = 1;
    for (int i = 0; i < SENSOR_COUNT; i++) {
        if (health_state.sensor_status[i] != SENSOR_OK) {
            all_ok = 0;
            break;
        }
    }
    if (all_ok) {
        health_state.last_healthy_tick_ms = HAL_GetTick();
    }
}

SensorHealth_t SystemHealth_GetSensorStatus(SensorName_t sensor)
{
    if (sensor >= SENSOR_COUNT)
        return SENSOR_INVALID;
    
    return health_state.sensor_status[sensor];
}

// ============================================================================
// SYSTEM STATE QUERY
// ============================================================================

SystemHealthState_t* SystemHealth_GetState(void)
{
    return &health_state;
}

void SystemHealth_PrintReport(void)
{
    printf("\r\n");
    printf(ANSI_MAGENTA "========== SYSTEM HEALTH REPORT ==========" ANSI_RESET "\r\n");
    printf("Emergency Stop: %s\r\n", health_state.emergency_stop_active ? "ACTIVE" : "inactive");
    printf("Total Errors: %lu\r\n", health_state.error_count);
    printf("Uptime: %lu ms\r\n", HAL_GetTick());
    
    printf("\nSensor Status:\r\n");
    const char *sensor_names[] = {"  IMU", "  GPS", "  Prox-L", "  Prox-R", "  LoRA", "  Sabertooth"};
    const char *status_names[] = {"OK", "TIMEOUT", "INVALID", "DEGRADED"};
    
    for (int i = 0; i < SENSOR_COUNT; i++) {
        printf("  %s: %s\r\n", sensor_names[i], status_names[health_state.sensor_status[i]]);
    }
    
    printf(ANSI_MAGENTA "==========================================" ANSI_RESET "\r\n");
}

// ============================================================================
// LEGACY SAFETY CHECK (kept for compatibility)
// ============================================================================

// Safety priority check: determines if state change is needed based on sensor health
// Priority: ESTOP > IMU failure > GPS loss
uint8_t SystemHealth_SafetyCheck(const SystemHealthInputs_t *in,
                                 RobotState_t current,
                                 RobotState_t *out_request_state)
{
    // Update sensor health from inputs
    SystemHealth_SetSensorStatus(SENSOR_IMU, in->imu_ok ? SENSOR_OK : SENSOR_TIMEOUT);
    SystemHealth_SetSensorStatus(SENSOR_GPS, in->gps_fix ? SENSOR_OK : SENSOR_TIMEOUT);
    
    // Highest priority: ESTOP button or already active
    if (in->estop_button || health_state.emergency_stop_active) {
        *out_request_state = STATE_ESTOP;
        if (!health_state.emergency_stop_active) {
            SystemHealth_TriggerEmergencyStop("ESTOP button pressed");
        }
        return 1;
    }

    // Next: IMU failure => ESTOP
    if (!in->imu_ok) {
        *out_request_state = STATE_ESTOP;
        SystemHealth_TriggerEmergencyStop("IMU failure - cannot navigate safely");
        return 1;
    }

    // Next: GPS no-fix => ERROR (but don't override ESTOP)
    if (!in->gps_fix) {
        if (current != STATE_ESTOP) {
            *out_request_state = STATE_ERROR;
            return 1;
        }
    }

    return 0;
}

void SystemHealth_UpdateLeds(RobotState_t state, uint8_t imu_ok, uint8_t gps_fix)
{
    // Clear LEDs
    HAL_GPIO_WritePin(GPIOD, LD3_Pin|LD4_Pin|LD5_Pin|LD6_Pin, GPIO_PIN_RESET);

    /* LED Status:
       - ESTOP/ERROR => red (LD5)
       - IMU bad => orange (LD3)
       - no GPS fix => red (LD5)
       - else => green (LD4)
    */

    if (state == STATE_ESTOP || state == STATE_ERROR) {
        HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
    } else if (!imu_ok) {
        HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
    } else if (!gps_fix) {
        HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
    }
}
