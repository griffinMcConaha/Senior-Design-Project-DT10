/**
 * @file heading_fusion.h
 * @brief Gyro + GPS heading fusion (fast gyro + accurate GPS)
 */
#ifndef HEADING_FUSION_H
#define HEADING_FUSION_H

#include <stdint.h>
#include "gps.h"
#include "imu_icm20948.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Fused orientation estimate state */
typedef struct
{
    float yaw_gyro_rad;        /**< Integrated gyro yaw (rad) */
    float yaw_fused_rad;       /**< GPS-corrected yaw (rad) */
    float yaw_deg;             /**< Fused yaw (degrees) - USE THIS */
    float pitch_deg;           /**< Pitch angle from IMU (deg) */
    float roll_deg;            /**< Roll angle from IMU (deg) */
    float gps_speed_filt_kn;   /**< Filtered GPS speed (knots) */
    float gps_course_filt_rad; /**< Filtered GPS course (rad) */
    uint8_t gps_course_filt_init;  /**< 1 after first GPS update */
    float heading_confidence;  /**< Trust level 0..1 */
    float gps_weight;          /**< GPS correction factor 0..0.1 */
    float accel_speed_filt_kn; /**< Accel-integrated speed estimate (knots) */
    float speed_fused_kn;      /**< GPS + accel blended speed (knots) - USE THIS */
} HeadingFusion_t;

/** @brief Initialize heading fusion filter */
void HeadingFusion_Init(HeadingFusion_t *hf);

/**
 * @brief Update fusion with latest IMU and GPS readings
 * @param hf Filter state (updated in-place)
 * @param imu Latest IMU status (accel/gyro)
 * @param gps Latest GPS data (position/course)
 * @param dt_s Time step in seconds (0.02f = 50 Hz typical)
 */
void HeadingFusion_Update(HeadingFusion_t *hf,
                          const IMU_Status_t *imu,
                          const GPS_Data_t *gps,
                          float dt_s);

/** @brief Get fused yaw estimate (degrees) - PRIMARY output for heading control */
static inline float HeadingFusion_GetYawDeg(const HeadingFusion_t *hf) { return hf->yaw_deg; }

/** @brief Get pitch estimate (degrees) */
static inline float HeadingFusion_GetPitchDeg(const HeadingFusion_t *hf) { return hf->pitch_deg; }

/** @brief Get roll estimate (degrees) */
static inline float HeadingFusion_GetRollDeg(const HeadingFusion_t *hf) { return hf->roll_deg; }

/** @brief Get fused speed estimate (knots) - blended accel + GPS, smoother than GPS alone */
static inline float HeadingFusion_GetSpeedKn(const HeadingFusion_t *hf) { return hf->speed_fused_kn; }

#ifdef __cplusplus
}
#endif

#endif // HEADING_FUSION_H
