import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_frame_manager = get_package_share_directory('frame_manager')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_frame_manager, 'config', 'frame_manager_params.yaml')

    return LaunchDescription([
        # Frame Manager Node
        # Manages coordinate transformations and bridges Ground Truth to TF.
        Node(
            package='frame_manager',
            executable='frame_manager_node',
            name='frame_manager',
            parameters=[params_file, {'use_sim_time': True}],
            output='screen'
        )
    ])
