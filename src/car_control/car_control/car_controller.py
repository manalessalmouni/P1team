import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.1  # secondes
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # avance vers l’avant
        msg.angular.z = 0.0  # pas de rotation
        self.publisher_.publish(msg)
        self.get_logger().info('🚗 Envoi de la commande de mouvement...')

def main(args=None):
    rclpy.init(args=args)
    node = CarController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

