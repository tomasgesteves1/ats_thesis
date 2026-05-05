import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ats_description = get_package_share_directory('ats_description')
    tether_sdf_path = os.path.join(pkg_ats_description, 'models', 'tether', 'tether.sdf')
    anchor_sdf_path = os.path.join(pkg_ats_description, 'models', 'tether', 'anchor.sdf')

    # Spawn da Âncora
    spawn_anchor = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', anchor_sdf_path,
            '-name', 'anchor',
            '-allow_renaming', 'false',
            '-x', '-0.50', '-y', '0.0', '-z', '1.17'
        ],
        output='screen'
    )

    # Spawn do Tether
    spawn_tether = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', tether_sdf_path,
            '-name', 'tether',
            '-allow_renaming', 'false',
            '-x', '-0.50', '-y', '0.0', '-z', '1.17' 
        ],
        output='screen'
    )

    # Bridge para os comandos da âncora
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/anchor/to_boat@std_msgs/msg/Bool[gz.msgs.Boolean',
            '/anchor/to_tether@std_msgs/msg/Bool[gz.msgs.Boolean',
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_anchor,
        spawn_tether,
        bridge
    ])
