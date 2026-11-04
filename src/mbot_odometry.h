#ifndef MBOT_ODOMETRY_H
#define MBOT_ODOMETRY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Heading-increment disagreement (rad per control step) above which
 * gyrodometry trusts the gyro instead of the encoders. At MAIN_LOOP_HZ = 100
 * this is ~0.07 deg per 10 ms step (~7 deg/s). Tune per robot.
 */
#ifndef GYRODOMETRY_THRESHOLD
#define GYRODOMETRY_THRESHOLD 0.00125f
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
 * @brief Calculate odometry based on velocities and gyro readings
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

#ifdef __cplusplus
}
#endif

#endif // MBOT_ODOMETRY_H 