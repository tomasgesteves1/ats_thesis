import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # O nosso nó MPC do Drone
    uav_node = Node(
        package='uav_mpc',
        executable='uav_mpc_node',
        name='uav_mpc_node',
        output='screen',
        remappings=[
            # O tópico de odom do uav_mpc tem de se ligar à Odometria vinda do moordyn_marsupial
            ('odom', '/drone/ground_truth/odometry'),
        ]
    )

    return LaunchDescription([
        uav_node
    ])
