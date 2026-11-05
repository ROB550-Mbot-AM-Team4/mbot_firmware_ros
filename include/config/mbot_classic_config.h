#ifndef CONFIG_MBOT_CLASSIC_CONFIG_H
#define CONFIG_MBOT_CLASSIC_CONFIG_H

// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             48.0  // Default encoder resolution.

// MBot Classic Parameters
#define DIFF_WHEEL_DIAMETER          0.082    // measured tire diameter
#define DIFF_WHEEL_RADIUS            0.041
#define DIFF_BASE_RADIUS             0.0853   // effective wheelbase 17.06 cm from two-turn spin test against gyro
#define MOT_R                        1   // Right motor slot
#define MOT_L                        0   // Left motor slot
#define MOT_UNUSED                   2   // Unused motor slot
#define MOT_LIDAR                    3   // M3 SERVO motor slot for Lidar
#define NUM_MOT_SLOTS                3   // Total number of wheel motor slots

// Odometry source for /odom and the odom->base_footprint tf (build-time).
// 0: wheel odometry (encoders only), 1: gyrodometry (encoders + IMU gyro yaw).
// Set via cmake -DMBOT_USE_GYRO_ODOM=ON (or -DMBOT_USE_GYRO_ODOM=1); defaults to wheel odometry.
#ifndef MBOT_USE_GYRO_ODOM
#define MBOT_USE_GYRO_ODOM           0
#endif

// Debug odometry (build-time). 1: also publish the non-selected odometry for comparison
// (checkpoint 1 Task 1.3): /wheel_odom when gyrodometry is selected, /gyrodom otherwise.
// Set via cmake -DMBOT_DEBUG_ODOM=ON; defaults to off (/odom only).
#ifndef MBOT_DEBUG_ODOM
#define MBOT_DEBUG_ODOM              0
#endif

#if MBOT_USE_GYRO_ODOM
#define MBOT_ALT_ODOM_TOPIC          "wheel_odom"
#else
#define MBOT_ALT_ODOM_TOPIC          "gyrodom"
#endif

#endif  /* CONFIG_MBOT_CLASSIC_CONFIG_H */
