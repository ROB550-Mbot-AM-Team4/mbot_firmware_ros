#ifndef CONFIG_MBOT_CLASSIC_CONFIG_H
#define CONFIG_MBOT_CLASSIC_CONFIG_H

// Hardware Parameters
#define GEAR_RATIO              78.0
#define ENCODER_RES             48.0  // Default encoder resolution.

// MBot Classic Parameters
#define DIFF_WHEEL_DIAMETER          0.0837
#define DIFF_WHEEL_RADIUS            0.04183
#define DIFF_BASE_RADIUS             0.085
#define MOT_R                        1   // Right motor slot
#define MOT_L                        0   // Left motor slot
#define MOT_UNUSED                   2   // Unused motor slot
#define MOT_LIDAR                    3   // M3 SERVO motor slot for Lidar
#define NUM_MOT_SLOTS                3   // Total number of wheel motor slots

// Debug odometry (build-time). 1: also publish /wheel_odom, /gyro_odom and the
// odom->wheel_odom, odom->gyro_odom tf frames. Set via cmake -DMBOT_DEBUG_ODOM=ON
// (or -DMBOT_DEBUG_ODOM=1 on the compiler command line); defaults to off.
#ifndef MBOT_DEBUG_ODOM
#define MBOT_DEBUG_ODOM              0
#endif

#endif  /* CONFIG_MBOT_CLASSIC_CONFIG_H */
