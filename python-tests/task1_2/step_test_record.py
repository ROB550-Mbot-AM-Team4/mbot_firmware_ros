#!/usr/bin/env python3
"""
Task 1.2 (PID tuning) step-response experiment with rosbag recording.

What it does, in order:
  1. (optional) loads the PID yaml into /mbot_control_node   (--load-yaml)
  2. sets  control_mode  on /mbot_control_node               (ff | pid | ffpid)
  3. starts  `ros2 bag record` (mcap) on the relevant topics
  4. streams the step sequence on /cmd_vel at 50 Hz:
         0 -> vx (hold) -> 0 -> [ -wz (hold) -> 0 -> +wz (hold) -> 0 ]
  5. stops the recorder and writes run_info.json (args + `ros2 param dump`)

Extra topics published by this script so Foxglove can plot them directly:
  /step_test/cmd        geometry_msgs/TwistStamped   commanded body velocity (clean step, incl. zeros)
  /step_test/cmd_wheel  mbot_interfaces/MotorVelocity  commanded wheel velocity [rad/s] (same kinematics as firmware)
  /step_test/meas       geometry_msgs/TwistStamped   body velocity computed from /motor_vel (~90 Hz, finer than /odom)

Usage (robot on the floor, agent running):
  python3 step_test_record.py ff      --label ff
  python3 step_test_record.py ffpid   --label tuned --load-yaml
"""
import argparse
import datetime as dt
import json
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist, TwistStamped
from mbot_interfaces.msg import MotorVelocity

# Must match include/config/mbot_classic_config.h and src/mbot_classic_ros.c
WHEEL_RADIUS = 0.04183   # DIFF_WHEEL_RADIUS [m]
BASE_RADIUS = 0.085      # DIFF_BASE_RADIUS  [m]
MOT_L, MOT_R = 0, 1

MODES = {"ff": 0, "pid": 1, "ffpid": 2}
NODE_NAME = "/mbot_control_node"
DEFAULT_YAML = os.path.expanduser(
    "~/mbot_firmware/mbot_firmware_ros/include/config/mbot_classic_pid.yaml")
RECORD_TOPICS = [
    "/cmd_vel", "/odom", "/motor_vel", "/encoders", "/imu",
    "/step_test/cmd", "/step_test/cmd_wheel", "/step_test/meas",
]


class StepTestNode(Node):
    def __init__(self):
        super().__init__("task1_2_step_test")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_st_pub = self.create_publisher(TwistStamped, "/step_test/cmd", 10)
        self.cmd_wheel_pub = self.create_publisher(MotorVelocity, "/step_test/cmd_wheel", 10)
        self.meas_pub = self.create_publisher(TwistStamped, "/step_test/meas", 50)
        self.create_subscription(MotorVelocity, "/motor_vel", self._motor_vel_cb, qos_profile_sensor_data)
        self.vx = 0.0
        self.wz = 0.0
        self.create_timer(0.02, self._tick)  # 50 Hz command stream (firmware times out after 1 s)

    def _motor_vel_cb(self, msg):
        wl, wr = float(msg.velocity[MOT_L]), float(msg.velocity[MOT_R])
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_footprint"
        # mbot_calculate_diff_body_vel() in mbot_classic_ros.c
        out.twist.linear.x = WHEEL_RADIUS * (wl - wr) / 2.0
        out.twist.angular.z = WHEEL_RADIUS * (-wl - wr) / (2.0 * BASE_RADIUS)
        self.meas_pub.publish(out)

    def _tick(self):
        vx, wz = self.vx, self.wz
        t = Twist()
        t.linear.x, t.angular.z = vx, wz
        self.cmd_pub.publish(t)

        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "base_footprint"
        ts.twist = t
        self.cmd_st_pub.publish(ts)

        # Same mapping as MODE_MBOT_VEL branch of mbot_loop()
        mv = MotorVelocity()
        mv.velocity = [
            (vx - BASE_RADIUS * wz) / WHEEL_RADIUS,
            (-vx - BASE_RADIUS * wz) / WHEEL_RADIUS,
            0.0,
        ]
        self.cmd_wheel_pub.publish(mv)


def run(cmd, check=True, capture=False, timeout=60):
    print(f"$ {' '.join(cmd)}", flush=True)
    r = subprocess.run(cmd, check=False, text=True, timeout=timeout,
                       stdout=subprocess.PIPE if capture else None,
                       stderr=subprocess.STDOUT if capture else None)
    if check and r.returncode != 0:
        if capture:
            print(r.stdout)
        sys.exit(f"command failed ({r.returncode}): {' '.join(cmd)}")
    return r.stdout if capture else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=MODES.keys(), help="control_mode to set before the run")
    ap.add_argument("--label", default=None, help="name prefix for the bag directory (default: mode)")
    ap.add_argument("--out-root", default=os.path.expanduser("~/bags/task1_2"))
    ap.add_argument("--vx", type=float, default=0.5, help="linear step [m/s]")
    ap.add_argument("--wz", type=float, default=1.0, help="rotation step magnitude [rad/s]")
    ap.add_argument("--hold", type=float, default=4.0, help="seconds each step is held")
    ap.add_argument("--pre", type=float, default=1.5, help="seconds of zero command before the step")
    ap.add_argument("--gap", type=float, default=1.0, help="seconds of zero command between steps")
    ap.add_argument("--no-rotate", action="store_true", help="skip the CW/CCW rotation steps")
    ap.add_argument("--load-yaml", nargs="?", const=DEFAULT_YAML, default=None,
                    help=f"run `ros2 param load` with this yaml first (default file: {DEFAULT_YAML})")
    ap.add_argument("--record-warmup", type=float, default=3.0, help="seconds to let the recorder start")
    args = ap.parse_args()

    label = args.label or args.mode
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_dir = os.path.join(args.out_root, f"{label}_{stamp}")
    os.makedirs(args.out_root, exist_ok=True)

    # --- 1/2. parameters -----------------------------------------------------
    if args.load_yaml:
        run(["ros2", "param", "load", NODE_NAME, args.load_yaml])
    run(["ros2", "param", "set", NODE_NAME, "control_mode", str(MODES[args.mode])])
    param_dump = run(["ros2", "param", "dump", NODE_NAME], capture=True)
    print(param_dump)

    # --- node + recorder -------------------------------------------------------
    rclpy.init()
    node = StepTestNode()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    rec_cmd = ["ros2", "bag", "record", "-s", "mcap", "-o", bag_dir, "--topics", *RECORD_TOPICS]
    print(f"$ {' '.join(rec_cmd)}", flush=True)
    recorder = subprocess.Popen(rec_cmd)

    def stop_recorder():
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            try:
                recorder.wait(timeout=20)
            except subprocess.TimeoutExpired:
                recorder.kill()

    def hold(vx, wz, seconds):
        node.vx, node.wz = float(vx), float(wz)
        time.sleep(seconds)

    sequence = []
    t_start = time.time()
    try:
        time.sleep(args.record_warmup)
        if recorder.poll() is not None:
            sys.exit("ros2 bag record exited early; see output above")

        def step(name, vx, wz, seconds):
            sequence.append({"name": name, "vx": vx, "wz": wz, "t_rel": round(time.time() - t_start, 3), "dur": seconds})
            print(f"[{time.time() - t_start:6.2f}s] {name}: vx={vx} wz={wz} for {seconds}s", flush=True)
            hold(vx, wz, seconds)

        step("pre", 0.0, 0.0, args.pre)
        step("linear_step", args.vx, 0.0, args.hold)
        step("gap", 0.0, 0.0, args.gap)
        if not args.no_rotate:
            step("rotate_cw", 0.0, -args.wz, args.hold)
            step("gap", 0.0, 0.0, args.gap)
            step("rotate_ccw", 0.0, +args.wz, args.hold)
            step("post", 0.0, 0.0, args.gap)
    except KeyboardInterrupt:
        print("\ninterrupted - stopping robot", flush=True)
    finally:
        node.vx, node.wz = 0.0, 0.0
        time.sleep(0.3)                      # make sure a zero command goes out
        stop_recorder()
        rclpy.shutdown()

    info = {
        "mode": args.mode, "control_mode": MODES[args.mode], "label": label,
        "bag_dir": bag_dir, "recorded_at": stamp, "args": vars(args),
        "sequence": sequence, "param_dump": param_dump,
        "wheel_radius": WHEEL_RADIUS, "base_radius": BASE_RADIUS,
    }
    with open(os.path.join(bag_dir, "run_info.json"), "w") as f:
        json.dump(info, f, indent=2)
    print(f"\nbag written to: {bag_dir}")
    print(f"control_mode is now {MODES[args.mode]} ({args.mode}); set it back if needed:\n"
          f"  ros2 param set {NODE_NAME} control_mode 2")


if __name__ == "__main__":
    main()
