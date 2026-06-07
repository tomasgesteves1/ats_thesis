from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('follow_height', default_value='7.5'),
        DeclareLaunchArgument('wait_time_s', default_value='5.0'),

        Node(
            package='ats_control',
            executable='drone_tracker',
            name='drone_tracker',
            parameters=[{
                'use_sim_time': True,
                'follow_height': LaunchConfiguration('follow_height'),
                'wait_time_s': LaunchConfiguration('wait_time_s'),
            }],
            output='screen'
        )
    ])
