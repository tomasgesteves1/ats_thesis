import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_localization = get_package_share_directory('ats_localization')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_localization, 'config', 'aligner_params.yaml')

    return LaunchDescription([
        # Simplified Frame Aligner Node
        # This node bridges /boat/ground_truth/odometry and /drone/ground_truth/odometry
        # directly to TF frames 'boat/base_link' and 'drone/base_link' under 'world'.
        Node(
            package='ats_localization',
            executable='frame_aligner',
            name='frame_aligner',
            parameters=[params_file, {'use_sim_time': True}],
            output='screen'
        )
    ])
