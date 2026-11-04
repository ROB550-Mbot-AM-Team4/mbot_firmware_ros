#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
    #define PI 3.14159265358979323846f
#endif

static inline void normalize_theta(float* theta) {
    while (*theta > PI) *theta -= 2.0f * PI;
    while (*theta <= -PI) *theta += 2.0f * PI;
}

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cosf(*theta) - vy * dt * sinf(*theta);
    *y += vx * dt * sinf(*theta) + vy * dt * cosf(*theta);
    *theta += wz * dt;

    normalize_theta(theta);
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta) {
    // Translation comes from the encoders (same as wheel odometry)
    *x += vx * dt * cosf(*theta) - vy * dt * sinf(*theta);
    *y += vx * dt * sinf(*theta) + vy * dt * cosf(*theta);

    // Heading: compare the encoder and gyro heading increments over this step.
    // When they disagree by more than the threshold (wheel slip, collisions),
    // trust the gyro; otherwise keep the encoder estimate (no gyro drift).
    float dtheta_enc  = wz * dt;
    float dtheta_gyro = gyro_z * dt;
    float dtheta = (fabsf(dtheta_gyro - dtheta_enc) > GYRODOMETRY_THRESHOLD) ? dtheta_gyro : dtheta_enc;
    *theta += dtheta;

    normalize_theta(theta);
}
