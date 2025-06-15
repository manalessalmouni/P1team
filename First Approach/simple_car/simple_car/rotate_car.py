#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.turn_y = -2.0         # Smaller waypoint to start turn earlier
        self.after_turn_y = -3.0   # Final Y target after turning
        self.target_x = 5.0        # X target after finishing Y movement

        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw = 0.0

        self.state = "move_to_turn_y"
        self.turn_start_angle = None
        self.turn_complete_threshold = math.radians(7)

        self.get_logger().info("🚗 Starting path: move -Y to turn point, turn, then move to final Y and +X")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def control_loop(self):
        twist = Twist()

        if self.state == "move_to_turn_y":
            if self.current_y > self.turn_y + 0.05:
                twist.linear.x = 3.0
                twist.angular.z = 0.0
                self.get_logger().info(f"Moving -Y to turn start (Y: {self.current_y:.2f})", throttle_duration_sec=1.0)
            else:
                self.state = "turn_90"
                self.turn_start_angle = self.yaw
                self.get_logger().info("Reached turn start Y. Starting turn")

        elif self.state == "turn_90":
            angle_diff = (self.yaw - self.turn_start_angle + math.pi) % (2 * math.pi) - math.pi
            target_diff = math.pi / 2

            if abs(angle_diff) < target_diff - self.turn_complete_threshold:
                twist.linear.x = 2.0   # slower forward speed during turn
                twist.angular.z = 3.0  # faster turning speed
                self.get_logger().info(f"Turning... {math.degrees(abs(angle_diff)):.1f}°", throttle_duration_sec=0.3)
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.state = "move_to_final_y"
                self.get_logger().info("✅ Turn done! Moving to final Y target")

        elif self.state == "move_to_final_y":
            if self.current_y > self.after_turn_y + 0.05:
                twist.linear.x = 3.0
                twist.angular.z = 0.0
                self.get_logger().info(f"Moving -Y to final target (Y: {self.current_y:.2f})", throttle_duration_sec=1.0)
            else:
                self.state = "move_x"
                self.get_logger().info("Reached final Y target. Moving +X")

        elif self.state == "move_x":
            if self.current_x < self.target_x - 0.05:
                twist.linear.x = 3.0
                twist.angular.z = 0.0
                self.get_logger().info(f"Moving +X (X: {self.current_x:.2f})", throttle_duration_sec=1.0)
            else:
                self.state = "stop"
                self.get_logger().info("🏁 Reached final X point. Stopping")

        elif self.state == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
