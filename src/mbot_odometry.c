#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
    #define PI 3.14159265358979323846f
#endif

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    *theta += wz * dt;

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta) {
    // Update pose
    static float pre_gyro_z = 0.0;
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    float dw_g = (gyro_z - pre_gyro_z) / dt;
    pre_gyro_z = gyro_z;
    if(wz != 0) {
        wz += dw_g;
        wz /= 2.0;
    }
    *theta += wz * dt;
    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;

    return;
}