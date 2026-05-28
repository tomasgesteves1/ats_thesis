import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_localization = get_package_share_directory('ats_localization')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_localization, 'config', 'aligner_params.yaml')

    return LaunchDescription([
        # 1. Frame Aligner Node
        Node(
            package='ats_localization',
            executable='frame_aligner',
            name='frame_aligner',
            parameters=[params_file, {'use_sim_time': True}],
            output='screen'
        ),

        # 2. Static Transforms (Offsets)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='boat_anchor_publisher',
            arguments=['-0.5', '0', '1.3', '0', '0', '0', 'boat/base_link', 'boat/tether_anchor']
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='drone_hook_publisher',
            arguments=['0', '0', '-0.1', '0', '0', '0', 'drone/base_link', 'drone/tether_hook']
        ),
    ])
