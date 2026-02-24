/**
 * @file system_health.h
 * @brief System safety monitoring, emergency stop, and sensor health tracking
 * 
 * Safety Priority (for state transitions):
 * 1. ESTOP button → STATE_ESTOP (highest priority)
 * 2. IMU failure → STATE_ESTOP (cannot navigate safely)
 * 3. GPS loss → STATE_ERROR (cannot follow path)
 * 4. Sensor degradation → log warning, continue monitoring
 * 
 * Emergency Stop: Once triggered, requires explicit reset via SystemHealth_ResetEmergencyStop()
 */
#ifndef SYSTEM_HEALTH_H
#define SYSTEM_HEALTH_H

#include <stdint.h>
#include "robot_sm.h"

/** @brief Sensor health states */
typedef enum {
    SENSOR_OK = 0,           /**< Sensor healthy and responsive */
    SENSOR_TIMEOUT = 1,      /**< No fresh data received (timeout) */
    SENSOR_INVALID = 2,      /**< Invalid values detected */
    SENSOR_DEGRADED = 3      /**< High error rate or drift */
} SensorHealth_t;

/** @brief System sensors being monitored */
typedef enum {
    SENSOR_IMU = 0,
    SENSOR_GPS = 1,
    SENSOR_PROX_LEFT = 2,
    SENSOR_PROX_RIGHT = 3,
    SENSOR_LORA = 4,
    SENSOR_SABERTOOTH = 5,
    SENSOR_COUNT = 6
} SensorName_t;

/** @brief Sensor health snapshot for safety decision logic */
typedef struct
{
    uint8_t imu_ok;        /**< 1=IMU OK, 0=failure */
    uint8_t gps_fix;       /**< 1=GPS has fix, 0=no signal */
    uint8_t estop_button;  /**< 1=emergency stop pressed */
} SystemHealthInputs_t;

/** @brief Overall system state summary */
typedef struct {
    uint8_t emergency_stop_active;           /**< 1=ESTOP triggered, 0=normal */
    SensorHealth_t sensor_status[SENSOR_COUNT];  /**< Health of each sensor */
    uint32_t last_healthy_tick_ms;           /**< Last time all sensors were healthy */
    uint32_t error_count;                    /**< Total errors logged this session */
    uint8_t degraded_sensor_flags;           /**< Bitmask of degraded sensors */
} SystemHealthState_t;

/**
 * @brief Initialize system health monitoring
 */
void SystemHealth_Init(void);

/**
 * @brief Check sensor health and request safety-critical state transitions
 * @param in Sensor health snapshot
 * @param current Current robot state
 * @param out_request_state Requested state if return=1
 * @return 1=transition requested, 0=no change needed
 * 
 * Priority order: ESTOP button → IMU failure → GPS loss
 * Never downgrades from ESTOP (must explicitly reset)
 */
uint8_t SystemHealth_SafetyCheck(const SystemHealthInputs_t *in,
                                 RobotState_t current,
                                 RobotState_t *out_request_state);

/**
 * @brief Update health status of a specific sensor
 * @param sensor Sensor to update
 * @param status New health status
 * 
 * Called by sensor drivers when they detect issues or receive fresh data
 */
void SystemHealth_SetSensorStatus(SensorName_t sensor, SensorHealth_t status);

/**
 * @brief Get health status of a sensor
 * @param sensor Sensor to query
 * @return Current health status
 */
SensorHealth_t SystemHealth_GetSensorStatus(SensorName_t sensor);

/**
 * @brief Trigger emergency stop (motor kill, disable dispersion)
 * @param reason Human-readable reason for the stop
 * 
 * Immediately disables motors and sets emergency flag.
 * Requires explicit SystemHealth_ResetEmergencyStop() to recover.
 */
void SystemHealth_TriggerEmergencyStop(const char *reason);

/**
 * @brief Reset emergency stop after resolving the issue
 * @return 1=success, 0=cannot reset (e.g., still in unsafe condition)
 */
uint8_t SystemHealth_ResetEmergencyStop(void);

/**
 * @brief Check if emergency stop is currently active
 * @return 1=ESTOP active, 0=normal operation
 */
uint8_t SystemHealth_IsEmergencyStopActive(void);

/**
 * @brief Get complete system health snapshot
 * @return Pointer to current health state
 */
SystemHealthState_t* SystemHealth_GetState(void);

/**
 * @brief Print system health report (for debugging)
 */
void SystemHealth_PrintReport(void);

/**
 * @brief Update status LEDs (LD3/4/5/6 on GPIOD)
 * @param state Current robot state
 * @param imu_ok 1=IMU healthy, 0=failure
 * @param gps_fix 1=GPS has fix, 0=no signal
 * 
 * LED Mapping: LD4=Green(OK), LD3=Orange(IMU), LD5=Red(Error/ESTOP)
 */
void SystemHealth_UpdateLeds(RobotState_t state, uint8_t imu_ok, uint8_t gps_fix);

#endif
