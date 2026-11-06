#include "mbot_odometry.h"
#include <math.h>
#include <stdbool.h>

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

/* gyro bias state (kept across odometry resets) */
static float gyro_bias         = 0.0f;   // [rad/s]
static float gyro_bias_sum     = 0.0f;   // initial-bias sum
static int   gyro_bias_samples = 0;      // initial-bias count
static int   still_steps       = 0;      // consecutive stopped steps

float mbot_gyrodometry_get_bias(void) {
    return gyro_bias;
}

bool mbot_gyrodometry_bias_ready(void) {
    return gyro_bias_samples >= GYRO_BIAS_INIT_SAMPLES;
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta) {
    // translation from encoders
    *x += vx * dt * cosf(*theta) - vy * dt * sinf(*theta);
    *y += vx * dt * sinf(*theta) + vy * dt * cosf(*theta);

    // require a sustained stop: wz also crosses 0 mid-turn on direction reversals
    bool wheels_stopped = (fabsf(vx) < GYRO_STATIONARY_VX) && (fabsf(wz) < GYRO_STATIONARY_WZ);
    still_steps = wheels_stopped ? (still_steps + 1) : 0;
    bool settled = still_steps >= GYRO_BIAS_SETTLE_STEPS;

    float dtheta;
    if (gyro_bias_samples < GYRO_BIAS_INIT_SAMPLES) {
        // initial bias: average while still; encoder heading until ready
        if (settled && fabsf(gyro_z) < GYRO_BIAS_MAX_RATE) {
            gyro_bias_sum += gyro_z;
            gyro_bias_samples++;
            if (gyro_bias_samples == GYRO_BIAS_INIT_SAMPLES) {
                gyro_bias = gyro_bias_sum / (float)GYRO_BIAS_INIT_SAMPLES;
            }
        }
        dtheta = wz * dt;
    } else {
        float rate = gyro_z - gyro_bias;
        // track bias only when truly at rest
        if (settled && fabsf(rate) < GYRO_BIAS_MAX_RATE) {
            gyro_bias += GYRO_BIAS_ALPHA * (gyro_z - gyro_bias);
            rate = gyro_z - gyro_bias;
        }
        // hold heading at rest
        if (settled && fabsf(rate) < GYRO_STILL_RATE) {
            rate = 0.0f;
        }
        dtheta = rate * dt;
    }
    *theta += dtheta;

    normalize_theta(theta);
}
