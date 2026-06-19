import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mission_manager',
            executable='mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }]
        )
    ])
