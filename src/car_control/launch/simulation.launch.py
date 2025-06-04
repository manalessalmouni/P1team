from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': '/home/rony/p1_ws/src/car_control/worlds/empty_world.world'
        }.items()
    )

    spawn_car_node = Node(
        package='car_control',
        executable='spawn_car',
        name='spawn_car_node',
        output='screen'
    )

    follow_waypoints_node = Node(
        package='car_control',
        executable='waypoint_follower',
        name='waypoint_follower_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_car_node,
        follow_waypoints_node
    ])

