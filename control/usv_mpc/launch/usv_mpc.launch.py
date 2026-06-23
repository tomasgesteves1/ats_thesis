import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get package share directory
    pkg_usv_mpc = get_package_share_directory('usv_mpc')
    
    # Path to YAML config file
    usv_mpc_config = os.path.join(pkg_usv_mpc, 'config', 'usv_mpc.yaml')

    # 1. Boat MPC Node (Dynamic Model)
    usv_mpc_node = Node(
        package='usv_mpc',
        executable='usv_mpc_node',
        name='usv_mpc_node',
        output='screen',
        parameters=[
            usv_mpc_config,
            {'use_sim_time': True}
        ],
        remappings=[
            ('odom', '/boat/ground_truth/odometry'),
            ('cmd_wrench', '/boat/cmd_wrench'),
            ('reference_path', '/usv/reference_path'),
            ('target_position', '/usv/target_position'),
            ('mpc_horizon', '/usv/mpc_horizon')
        ]
    )

    # 2. Thruster Mapper Node (Inverse Thrust Allocation)
    thruster_mapper_node = Node(
        package='usv_mpc',
        executable='thruster_mapper_node',
        name='thruster_mapper_node',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ],
        remappings=[
            ('cmd_wrench', '/boat/cmd_wrench'),
            ('thrusters/left/thrust', '/boat/thrusters/left/thrust'),
            ('thrusters/right/thrust', '/boat/thrusters/right/thrust'),
            ('thrusters/left/pos', '/boat/thrusters/left/pos'),
            ('thrusters/right/pos', '/boat/thrusters/right/pos'),
        ]
    )

    return LaunchDescription([
        usv_mpc_node,
        thruster_mapper_node
    ])
