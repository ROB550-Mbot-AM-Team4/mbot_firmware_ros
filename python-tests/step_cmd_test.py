import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist

def main():
    rclpy.init()
    node = Node('simple_step_cmd')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Messages sent before DDS matches the firmware subscriber are dropped
    # (the first 1-3 s of the first command), so wait for the match first.
    def firmware_matched():
        subs = node.get_subscriptions_info_by_topic('/cmd_vel')
        return pub.get_subscription_count() > 0 and any(
            s.node_name == 'mbot_control_node' for s in subs)

    print("Waiting for mbot_control_node to subscribe to /cmd_vel...")
    t0 = time.time()
    while not firmware_matched():
        if time.time() - t0 > 10.0:
            print("Timed out: is the micro-ROS agent running?")
            node.destroy_node()
            rclpy.shutdown()
            return
        time.sleep(0.05)
    time.sleep(0.2)
    print(f"Matched after {time.time() - t0:.2f} s")

    def move(linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        
        # Publish for the specified duration
        start_time = time.time()
        while time.time() - start_time < duration:
            pub.publish(msg)
            time.sleep(0.05)

    print("Starting sequence...")
    move(0.0, 0.0, 0.0)
    
    # 1. Drive forward 0.5 m/s for 4s
    move(0.5, 0.0, 4.0)
    
    # 2. Wait 1s
    move(0.0, 0.0, 1.0)
    
    # 3. Rotate CW (negative) for 4s
    move(0.0, -1.0, 4.0)
    
    # 4. Wait 1s
    move(0.0, 0.0, 1.0)
    
    # 5. Rotate CCW (positive) for 4s
    move(0.0, 1.0, 4.0)

    # Stop and cleanup
    pub.publish(Twist())
    print("Done.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()