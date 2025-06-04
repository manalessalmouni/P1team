#!/usr/bin/env python3
from transforms3d.euler import euler2quat
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import Quaternion

import os

class CarSpawner(Node):
    def __init__(self):
        super().__init__('spawn_car_node')
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('En attente du service /spawn_entity...')
        self.spawn_car()

    def spawn_car(self):
        request = SpawnEntity.Request()
        request.name = "simple_car"
        request.xml = open(os.path.expanduser('~/models/simple_car/model.urdf'), 'r').read()
        request.robot_namespace = '/'

        # Position initiale
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.1

        # Orientation vers l’axe +x (angle yaw = 0 rad)
        q = euler2quat(0, 0, 0)
        request.initial_pose.orientation = Quaternion(
           x=q[1], y=q[2], z=q[3], w=q[0]  # w en premier avec transforms3d
        )  # (roll, pitch, yaw)

        request.initial_pose = pose

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
           self.get_logger().info("✅ Voiture spawnée avec succès !")
        else:
           self.get_logger().error("❌ Échec du spawn de la voiture.")

def main(args=None):
    rclpy.init(args=args)
    car_spawner = CarSpawner()
    car_spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

