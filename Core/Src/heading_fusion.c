#include "heading_fusion.h"
#include <math.h>

// Math constants for angle wrapping and conversion
#define PI_F      3.14159265f      // Pi
#define TWO_PI_F  (2.0f * PI_F)    // 2*Pi
#define RAD2DEG   (57.2957795130823f)  // Radians to degrees conversion

// Low-pass filter: blend previous value with new measurement
static inline float LPF(float prev, float current, float alpha)
{
    return prev + alpha * (current - prev);
}

// Wrap angle to [-Pi, Pi] range (converts from radians)
static inline float WrapPi(float a)
{
    if (a >  PI_F) a -= TWO_PI_F;
    if (a < -PI_F) a += TWO_PI_F;
    return a;
}

// Compute fused speed: blend accel-integrated speed with GPS speed
// Accel provides smooth high-rate estimate, GPS provides long-term accuracy
static float Compute_FusedSpeed(HeadingFusion_t *hf,
                                float accel_speed_kn,
                                float gps_speed_kn,
                                uint8_t gps_has_fix)
{
    // Complementary filter: favor accel for fast changes, GPS for correction
    // When GPS has fix: blend (90% accel, 10% GPS)
    // When no fix: use accel alone with gradual decay
    
    const float alpha_gps = 0.10f;  // GPS correction factor (0.10 = 10% trust GPS)
    
    float fused = accel_speed_kn;
    
    if (gps_has_fix && gps_speed_kn >= 0.1f) {
        // Blend with GPS: accel provides responsive estimate, GPS corrects drift
        fused = accel_speed_kn + alpha_gps * (gps_speed_kn - accel_speed_kn);
    } else if (!gps_has_fix) {
        // No GPS fix: gradual decay of accel estimate to zero (assume coasting to stop)
        const float decay = 0.98f;  // Per update: lose 2% per 50Hz = ~15% per second
        fused = accel_speed_kn * decay;
    }
    
    // Clamp to non-negative (can't have negative speed in knots)
    if (fused < 0.0f) fused = 0.0f;
    
    return fused;
}

// Compute fused heading: combine fast gyro with accurate GPS course correction
static float Compute_FusedHeading(HeadingFusion_t *hf,
                                  float gyro_yaw_rad,
                                  const GPS_Data_t *gps,
                                  uint8_t imu_ok)
{
    float fused = gyro_yaw_rad;
    float weight = 0.0f;

    // same weighting idea you had, but now uses hf's filtered GPS state
    if (!gps->has_fix || !hf->gps_course_filt_init) {
        weight = 0.0f;
        hf->gps_speed_filt_kn = 0.0f;
    } else if (hf->gps_speed_filt_kn < 1.0f) {
        weight = 0.0f;
    } else if (hf->gps_speed_filt_kn > 2.0f) {
        weight = 0.10f;
    } else {
        weight = (hf->gps_speed_filt_kn - 1.0f) * 0.10f;
    }

    hf->gps_weight = weight;

    if (weight > 0.0f) {
        float err = hf->gps_course_filt_rad - gyro_yaw_rad;
        err = WrapPi(err);
        fused = gyro_yaw_rad + weight * err;
    }

    fused = WrapPi(fused);

    // same confidence heuristic you used
    if (!imu_ok) {
        hf->heading_confidence = 0.1f;
    } else if (!gps->has_fix || hf->gps_speed_filt_kn < 0.3f) {
        hf->heading_confidence = 0.4f;
    } else if (hf->gps_speed_filt_kn < 1.0f) {
        hf->heading_confidence = 0.6f;
    } else if (hf->gps_speed_filt_kn < 2.0f) {
        hf->heading_confidence = 0.8f;
    } else {
        hf->heading_confidence = 0.9f;
    }

    return fused;
}

// Initialize heading fusion filter to zero state
void HeadingFusion_Init(HeadingFusion_t *hf)
{
    if (!hf) return;

    hf->yaw_gyro_rad = 0.0f;
    hf->yaw_fused_rad = 0.0f;

    hf->yaw_deg = 0.0f;
    hf->pitch_deg = 0.0f;
    hf->roll_deg = 0.0f;

    hf->gps_speed_filt_kn = 0.0f;
    hf->gps_course_filt_rad = 0.0f;
    hf->gps_course_filt_init = 0;

    hf->accel_speed_filt_kn = 0.0f;
    hf->speed_fused_kn = 0.0f;

    hf->heading_confidence = 0.0f;
    hf->gps_weight = 0.0f;
}

// Update fusion filter: integrate gyro, apply GPS corrections, output fused heading
void HeadingFusion_Update(HeadingFusion_t *hf,
                          const IMU_Status_t *imu,
                          const GPS_Data_t *gps,
                          float dt_s)
{
    if (!hf || !imu || !gps) return;

    // guard dt like your main loop
    if (dt_s <= 0.0f || dt_s > 0.2f) dt_s = 0.02f;

    // 0) Estimate speed from accelerometer (forward acceleration in body frame)
    // Project acceleration vector onto heading direction for forward speed estimate
    {
        // Use filtered pitch to estimate forward accel component
        float pitch_rad = hf->pitch_deg * (PI_F / 180.0f);
        
        // Forward accel (body frame): ax = -g*sin(pitch) + forward_accel
        // So: forward_accel ≈ ax + g*sin(pitch)
        float g_ms2 = 9.81f;
        float forward_accel_ms2 = imu->ax_g * g_ms2 + g_ms2 * sinf(pitch_rad);
        
        // Integrate to get speed (simple Euler integration)
        // Then convert m/s to knots (1 knot = 0.51444 m/s)
        const float ms2_to_knots = 1.0f / 0.51444f;  // ~1.944 knots per m/s
        hf->accel_speed_filt_kn += forward_accel_ms2 * dt_s * ms2_to_knots;
        
        // Low-pass filter accel speed to smooth integration noise
        const float a_accel_spd = 0.20f;  // Moderate filtering
        hf->accel_speed_filt_kn = LPF(hf->accel_speed_filt_kn, hf->accel_speed_filt_kn, a_accel_spd);
        
        // Clamp to prevent runaway
        if (hf->accel_speed_filt_kn < 0.0f) hf->accel_speed_filt_kn = 0.0f;
        if (hf->accel_speed_filt_kn > 30.0f) hf->accel_speed_filt_kn = 30.0f;  // Max ~60 km/h
    }

    // 1) Update filtered GPS speed/course (THIS is what your main was missing)
    // Speed LPF
    {
        const float a_spd = 0.10f; // can tune
        if (!gps->has_fix) {
            hf->gps_speed_filt_kn = 0.0f;
        } else {
            hf->gps_speed_filt_kn = LPF(hf->gps_speed_filt_kn, gps->speed_knots, a_spd);
        }
    }

    // Course LPF in radians, but be careful around wrap
    {
        const float a_crs = 0.15f; // can tune
        if (gps->has_fix && gps->speed_knots > 0.5f) {
            float course_rad = gps->course_deg * (PI_F / 180.0f);

            if (!hf->gps_course_filt_init) {
                hf->gps_course_filt_rad = course_rad;
                hf->gps_course_filt_init = 1;
            } else {
                // unwrap error before filtering (wrap-aware LPF)
                float err = WrapPi(course_rad - hf->gps_course_filt_rad);
                hf->gps_course_filt_rad = WrapPi(hf->gps_course_filt_rad + a_crs * err);
            }
        }
        // else: keep last course_filt (don’t “chase noise” when stationary)
    }

    // 2) Compute fused speed: accel + GPS blending
    hf->speed_fused_kn = Compute_FusedSpeed(hf, hf->accel_speed_filt_kn, hf->gps_speed_filt_kn, gps->has_fix);

    // 3) Integrate yaw from gyro Z (rad/s)
    hf->yaw_gyro_rad = WrapPi(hf->yaw_gyro_rad + imu->gz_rad_s * dt_s);

    // 4) Fuse yaw using filtered GPS course + filtered speed
    hf->yaw_fused_rad = Compute_FusedHeading(hf, hf->yaw_gyro_rad, gps, imu->ok);

    // 5) Roll/pitch from accel (rad -> deg)
    float roll_acc  = atan2f(imu->ay_g, imu->az_g);
    float pitch_acc = atan2f(-imu->ax_g, sqrtf(imu->ay_g * imu->ay_g + imu->az_g * imu->az_g));

    // 6) Output LPF (degrees), like your old main
    {
        const float a_out = 0.10f;
        hf->roll_deg  = LPF(hf->roll_deg,  roll_acc  * RAD2DEG, a_out);
        hf->pitch_deg = LPF(hf->pitch_deg, pitch_acc * RAD2DEG, a_out);
        hf->yaw_deg   = LPF(hf->yaw_deg,   hf->yaw_fused_rad * RAD2DEG, a_out);
    }
}
