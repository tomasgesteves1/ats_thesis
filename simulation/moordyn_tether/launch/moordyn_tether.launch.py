import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('moordyn_tether')
    config_file = os.path.join(pkg_share, 'config', 'moordyn_tether.yaml')

    return LaunchDescription([
        Node(
            package='moordyn_tether',
            executable='moordyn_tether_node',
            name='moordyn_tether_node',
            output='screen',
            parameters=[config_file, {'use_sim_time': True}],
        ),
    ])
