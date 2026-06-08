from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Lançar apenas o driver do joystick
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }],
            output='screen'
        )
    ])
