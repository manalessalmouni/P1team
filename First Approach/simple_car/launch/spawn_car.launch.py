import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('simple_car')
    urdf_path = os.path.join(pkg_share, 'urdf', 'car.urdf')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_car', '-file', urdf_path, '-z', '0.5'],
            output='screen'
        ),
    ])
