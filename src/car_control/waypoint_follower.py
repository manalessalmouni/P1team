import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import transforms3d.euler
from waypoints_pkg.waypoints import waypoints
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.waypoints = waypoints
        self.current_index = 0
        config_file = os.path.join(get_package_share_directory('car_control'), 'config', 'waypoints.yaml')
        with open(config_file, 'r') as f:
            params = yaml.safe_load(f)
            self.waypoints = [(float(p[0]), float(p[1])) for p in params['waypoints']]

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        q = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        r, p, yaw = transforms3d.euler.quat2euler(q)
        self.get_logger().info(f"📍 Position: x={pos.x:.2f}, y={pos.y:.2f}, yaw={yaw:.2f}")
        self.move_to_waypoint(pos.x, pos.y, yaw)

    def move_to_waypoint(self, x, y, yaw):
        if self.current_index >= len(self.waypoints):
            self.get_logger().info("✅ Tous les waypoints ont été atteints.")
            self.stop()
            return

        goal_x, goal_y = self.waypoints[self.current_index]
        self.get_logger().info(f"🎯 Cible actuelle : Waypoint {self.current_index+1} → ({goal_x:.2f}, {goal_y:.2f})")
        dx = goal_x - x
        dy = goal_y - y
        distance = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - yaw

        # Normaliser l’angle
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        twist = Twist()
        twist.linear.x = 0.5 if distance > 0.1 else 0.0
        twist.angular.z = 1.5 * angle_diff

        self.publisher.publish(twist)

        if distance < 0.1:
            self.current_index += 1
            self.get_logger().info(f"📍 Waypoint atteint : ({goal_x:.2f}, {goal_y:.2f})")

    def stop(self):
        self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

