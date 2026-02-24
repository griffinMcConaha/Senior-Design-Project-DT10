#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

#include <stdint.h>
#include "proximity.h"

#ifdef __cplusplus
extern "C" {
#endif

// Test individual motor speed
void Diag_TestMotor(uint8_t motor, int speed);

// Test individual motor raw command value (-2047 to +2047)
void Diag_TestMotorRaw(uint8_t motor, int value);

// Sweep all Sabertooth motor speeds (-100 to +100) with delay between each
// delay_ms: milliseconds to hold each speed
void Diag_TestMotorSweep(uint16_t delay_ms);

// Test single salt dispersion rate (0-100%)
void Diag_TestSaltRate(uint8_t rate_percent);

// Sweep all salt dispersion rates (0-100%) with delay between each
// delay_ms: milliseconds to hold each rate
void Diag_TestSaltSweep(uint16_t delay_ms);

// Test single brine dispersion rate (0-100%)
void Diag_TestBrineRate(uint8_t rate_percent);

// Sweep all brine dispersion rates (0-100%) with delay between each
// delay_ms: milliseconds to hold each rate
void Diag_TestBrineSweep(uint16_t delay_ms);

// Test GPS sensor - read current position
void Diag_TestGPS(void);

// Test IMU sensor - read accel/gyro/mag
void Diag_TestIMU(void);

// Detailed IMU communication check (I2C, WHO_AM_I, register reads)
void Diag_TestIMUDetailed(void);

// Scan I2C bus for all devices
void Diag_ScanI2C(void);

// Test proximity sensors - read left and right distances
void Diag_TestProximity(void);

// Display current state machine state
void Diag_PrintState(void);

// Live Sabertooth feedback monitor (continuous polling with ESC to exit)
void Diag_MonitorSabertoothFeedback(void);

// System health diagnostic - display health status of all sensors and systems
void Diag_SystemHealth(void);

// Run all tests sequentially
void Diag_TestAll(void);

#ifdef __cplusplus
}
#endif

#endif // DIAGNOSTICS_H
