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

/* ---- Gyro bias estimation state (persists across odometry resets on purpose) ---- */
static float gyro_bias        = 0.0f;   // estimated gyro z bias [rad/s]
static float gyro_bias_sum    = 0.0f;   // accumulator for the initial estimate
static int   gyro_bias_samples = 0;     // stationary samples collected so far

float mbot_gyrodometry_get_bias(void) {
    return gyro_bias;
}

bool mbot_gyrodometry_bias_ready(void) {
    return gyro_bias_samples >= GYRO_BIAS_INIT_SAMPLES;
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta) {
    // Translation comes from the encoders (same as wheel odometry), rotated by the current heading
    *x += vx * dt * cosf(*theta) - vy * dt * sinf(*theta);
    *y += vx * dt * sinf(*theta) + vy * dt * cosf(*theta);

    // Wheels not moving (encoder body velocities at quantization-noise level)
    bool stationary = (fabsf(vx) < GYRO_STATIONARY_VX) && (fabsf(wz) < GYRO_STATIONARY_WZ);

    float dtheta;
    if (gyro_bias_samples < GYRO_BIAS_INIT_SAMPLES) {
        // Initial bias estimate: average the gyro while the robot sits still after boot.
        // Until it is ready, fall back to the encoder heading so /odom keeps working.
        if (stationary) {
            gyro_bias_sum += gyro_z;
            gyro_bias_samples++;
            if (gyro_bias_samples == GYRO_BIAS_INIT_SAMPLES) {
                gyro_bias = gyro_bias_sum / (float)GYRO_BIAS_INIT_SAMPLES;
            }
        }
        dtheta = wz * dt;
    } else {
        // Track slow bias changes (temperature) whenever the wheels are stopped
        if (stationary) {
            gyro_bias += GYRO_BIAS_ALPHA * (gyro_z - gyro_bias);
        }
        float rate = gyro_z - gyro_bias;
        // Standing still and the corrected rate is within noise: hold heading (no random walk)
        if (stationary && fabsf(rate) < GYRO_STILL_RATE) {
            rate = 0.0f;
        }
        dtheta = rate * dt;
    }
    *theta += dtheta;

    normalize_theta(theta);
}
