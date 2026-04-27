import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Lançar o leitor do comando (joystick) nativo do ROS 2
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        output='screen'
    )

    # Lançar o teu nó controlador de teste
    usv_controller = Node(
        package='ats_control',
        executable='usv_controller',
        name='usv_controller',
        output='screen'
    )

    return LaunchDescription([
        joy_node,
        usv_controller
    ])
