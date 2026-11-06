#ifndef MBOT_ODOMETRY_H
#define MBOT_ODOMETRY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* gyrodometry tuning */
#ifndef GYRO_BIAS_INIT_SAMPLES
#define GYRO_BIAS_INIT_SAMPLES 200      // samples for the initial bias (2 s)
#endif
#ifndef GYRO_BIAS_ALPHA
#define GYRO_BIAS_ALPHA 0.01f           // bias tracking gain (~1 s)
#endif
#ifndef GYRO_STATIONARY_VX
#define GYRO_STATIONARY_VX 0.005f       // stopped if |vx| below [m/s]
#endif
#ifndef GYRO_STATIONARY_WZ
#define GYRO_STATIONARY_WZ 0.02f        // stopped if |wz| below [rad/s]
#endif
#ifndef GYRO_STILL_RATE
#define GYRO_STILL_RATE 0.01f           // rate held at 0 when at rest [rad/s]
#endif
#ifndef GYRO_BIAS_SETTLE_STEPS
#define GYRO_BIAS_SETTLE_STEPS 200      // stopped steps before bias tracking (2 s)
#endif
#ifndef GYRO_BIAS_MAX_RATE
#define GYRO_BIAS_MAX_RATE 0.05f        // max |gyro - bias| for bias tracking [rad/s]
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
 * @brief Gyrodometry: translation from encoders, heading from bias-corrected gyro z
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

/** @brief Gyro z bias estimate [rad/s] */
float mbot_gyrodometry_get_bias(void);

/** @brief True once the initial bias is set */
bool mbot_gyrodometry_bias_ready(void);

#ifdef __cplusplus
}
#endif

#endif // MBOT_ODOMETRY_H 