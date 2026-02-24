/**
 * @file imu_icm20948.h
 * @brief ICM-20948 9-DoF IMU driver (I2C interface, I2C1)
 * 
 * 3-axis accelerometer + 3-axis gyroscope (+ optional magnetometer)
 */
#pragma once

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Enable magnetometer fusion (0=accel+gyro only, 1=add mag)
 *  NOTE: Only works if your ICM-20948 module includes the integrated AK09916 magnetometer
 *  Most ICM-20948 breakout boards do NOT include it. Check your module documentation.
 */
#ifndef IMU_USE_MAG
#define IMU_USE_MAG 1
#endif

/** @brief Latest IMU sensor reading */
typedef struct
{
    uint8_t ok;  /**< 1=read succeeded, 0=I2C failure */
    
    /* Raw sensor counts */
    float raw_ax, raw_ay, raw_az;
    float raw_gx, raw_gy, raw_gz;
#if IMU_USE_MAG
    float raw_mx, raw_my, raw_mz;
#endif
    
    /* Calibration biases */
    float acc_bias[3];   /**< Accelerometer bias (counts) */
    float gyro_bias[3];  /**< Gyroscope bias (counts) */
#if IMU_USE_MAG
    float mag_bias[3];
#endif
    
    /* Scaled outputs */
    float ax_g, ay_g, az_g;              /**< Acceleration (g units) */
    float gx_rad_s, gy_rad_s, gz_rad_s;  /**< Angular velocity (rad/s) */
#if IMU_USE_MAG
    float mx_uT, my_uT, mz_uT;           /**< Magnetic field (µT) */
#endif
    float temperature_c;                 /**< Temperature (Celsius) */
} IMU_Status_t;

/** @brief Initialize IMU on I2C (call with &hi2c1) */
void IMU_Init(I2C_HandleTypeDef *hi2c);

/** @brief Set IMU I2C address (7-bit: 0x68 or 0x69, must call before IMU_Init) */
void IMU_SetAddress(uint8_t addr_7bit);

/** @brief Get current IMU I2C address (7-bit) */
uint8_t IMU_GetAddress(void);

/** @brief Read accelerometer + gyroscope (+ mag if enabled) */
IMU_Status_t IMU_Read(void);

/** @brief Calibrate gyro bias (samples=500, delay=5ms typical) */
void IMU_Calibrate(uint16_t samples, uint16_t delay_ms);

/** @brief Check if calibration completed successfully */
uint8_t IMU_IsCalibrated(void);

/** @brief Check if last read succeeded */
uint8_t IMU_LastOk(void);

/** @brief Force IMU reset */
void IMU_SoftReset(void);

/** @brief Verify IMU present (read WHO_AM_I register) */
void IMU_RuntimeCheckWHOAMI(void);

/** @brief Degree-to-radian conversion */
#ifndef IMU_DEG2RAD
#define IMU_DEG2RAD (0.01745329251994f)
#endif

#ifdef __cplusplus
}
#endif
