import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mocap_bridge')
    params_file = os.path.join(pkg_share, 'config', 'mocap_bridge_params.yaml')

    mocap_bridge_node = Node(
        package='mocap_bridge',
        executable='mocap_bridge_node',
        name='mocap_bridge',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}]
    )

    return LaunchDescription([
        mocap_bridge_node
    ])
