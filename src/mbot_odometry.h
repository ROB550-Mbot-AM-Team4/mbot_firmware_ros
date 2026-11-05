#ifndef MBOT_ODOMETRY_H
#define MBOT_ODOMETRY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Gyrodometry tuning (heading = integrated, bias-corrected gyro z) */
#ifndef GYRO_BIAS_INIT_SAMPLES
#define GYRO_BIAS_INIT_SAMPLES 200      // stationary samples averaged for the initial bias (2 s at 100 Hz)
#endif
#ifndef GYRO_BIAS_ALPHA
#define GYRO_BIAS_ALPHA 0.01f           // low-pass gain for bias tracking while stopped (~1 s time constant)
#endif
#ifndef GYRO_STATIONARY_VX
#define GYRO_STATIONARY_VX 0.005f       // |vx| below this [m/s] counts as wheels stopped
#endif
#ifndef GYRO_STATIONARY_WZ
#define GYRO_STATIONARY_WZ 0.02f        // |wz| below this [rad/s] counts as wheels stopped
#endif
#ifndef GYRO_STILL_RATE
#define GYRO_STILL_RATE 0.01f           // corrected gyro rate below this while stopped is treated as 0 [rad/s]
#endif


/**
 * @brief Calculate odometry based on velocities
 * 
 * @param vx Forward velocity in m/s
 * @param vy Lateral velocity in m/s (always 0 for diff drive)
 * @param wz Angular velocity in rad/s
 * @param dt Time step in seconds
 * @param x Current x position (will be updated)
 * @param y Current y position (will be updated)
 * @param theta Current orientation (will be updated)
 */
void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta);

/**
 * @brief Calculate gyrodometry: translation from encoder body velocities,
 *        heading from integrating the bias-corrected IMU gyro z rate.
 *        The gyro bias is averaged over the first GYRO_BIAS_INIT_SAMPLES stationary
 *        samples after boot (encoder heading is used until then) and tracked slowly
 *        whenever the wheels are stopped. vx/wz are also used to detect "stopped".
 * 
 * @param vx Forward velocity in m/s
 * @param vy Lateral velocity in m/s (always 0 for diff drive)
 * @param wz Angular velocity in rad/s
 * @param dt Time step in seconds
 * @param gyro_z Gyro z-axis angular velocity in rad/s
 * @param x Current x position (will be updated)
 * @param y Current y position (will be updated)
 * @param theta Current orientation (will be updated)
 */
void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta);

/** @brief Current gyro z bias estimate [rad/s] (for diagnostics) */
float mbot_gyrodometry_get_bias(void);

/** @brief True once the initial bias estimate has been collected */
bool mbot_gyrodometry_bias_ready(void);

#ifdef __cplusplus
}
#endif

#endif // MBOT_ODOMETRY_H 