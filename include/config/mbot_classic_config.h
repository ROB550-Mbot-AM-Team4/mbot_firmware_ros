#ifndef CONFIG_MBOT_CLASSIC_CONFIG_H
#define CONFIG_MBOT_CLASSIC_CONFIG_H

// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             48.0  // Default encoder resolution.

// MBot Classic Parameters
#define DIFF_WHEEL_DIAMETER          0.084    // from 1.30 m straight-line test
#define DIFF_WHEEL_RADIUS            0.042
#define DIFF_BASE_RADIUS             0.0875   // 175 mm wheelbase / 2
#define MOT_R                        1   // Right motor slot
#define MOT_L                        0   // Left motor slot
#define MOT_UNUSED                   2   // Unused motor slot
#define MOT_LIDAR                    3   // M3 SERVO motor slot for Lidar
#define NUM_MOT_SLOTS                3   // Total number of wheel motor slots

// /odom + tf source: 1 = gyrodometry, 0 = wheel odometry
#ifndef MBOT_USE_GYRO_ODOM
#define MBOT_USE_GYRO_ODOM           1
#endif

// 1 = also publish the non-selected odometry for comparison
#ifndef MBOT_DEBUG_ODOM
#define MBOT_DEBUG_ODOM              0
#endif

#if MBOT_USE_GYRO_ODOM
#define MBOT_ALT_ODOM_TOPIC          "wheel_odom"
#else
#define MBOT_ALT_ODOM_TOPIC          "gyrodom"
#endif

#endif  /* CONFIG_MBOT_CLASSIC_CONFIG_H */
