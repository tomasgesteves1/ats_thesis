import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_mpc = get_package_share_directory('uav_mpc')
    pkg_usv_mpc = get_package_share_directory('usv_mpc')

    # 1. Boat MPC Node
    usv_mpc_config = os.path.join(pkg_usv_mpc, 'config', 'usv_mpc.yaml')
    usv_mpc_node = Node(
        package='usv_mpc',
        executable='usv_mpc_node',
        name='usv_mpc_node',
        output='screen',
        parameters=[
            usv_mpc_config,
            {
                'use_sim_time': True,
                'trajectory_type': 'external',
            }
        ],
        remappings=[
            ('odom', '/boat/ground_truth/odometry'),
            ('cmd_wrench', '/boat/cmd_wrench'),
            ('reference_path', '/usv/reference_path'),
            ('target_position', '/usv/target_position'),
            ('mpc_horizon', '/usv/mpc_horizon')
        ]
    )

    # 2. Boat Thruster Mapper Node
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

    # 3. Boat Circle Trajectory Generator
    usv_circle_node = Node(
        package='test_usv_circle',
        executable='test_usv_circle_node',
        name='usv_trajectory_circle_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'circle_radius': 15.0,
                'circle_omega': 0.05,
                'circle_center_x': 0.0,
                'circle_center_y': 0.0,
                'horizon_stages': 20,
                'control_period': 0.1,
                'update_rate_hz': 10.0,
            }
        ],
        remappings=[
            ('reference_path', '/usv/reference_path'),
        ]
    )

    # 4. UAV MPC Node (with cooperative tether constraints enabled)
    uav_mpc_config = os.path.join(pkg_uav_mpc, 'config', 'uav_mpc.yaml')
    uav_mpc_node = Node(
        package='uav_mpc',
        executable='uav_mpc_node',
        name='uav_mpc_node',
        output='screen',
        parameters=[
            uav_mpc_config,
            {
                'use_sim_time': True,
                'trajectory_type': 'external',
                'use_tether': True,
            }
        ],
        remappings=[
            ('odom', '/drone/ground_truth/odometry'),
            ('tether_length', '/moordyn_tether_node/tether_length'),
            ('reference_path', '/uav/reference_path'),
            ('boat_mpc_horizon', '/usv/mpc_horizon'),
        ]
    )

    # 5. UAV Circle Trajectory Generator (offset radius to test tether limit constraint)
    uav_circle_node = Node(
        package='test_uav_circle',
        executable='test_uav_circle_node',
        name='uav_trajectory_circle_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'circle_radius': 5.0,
                'circle_omega': 0.05,
                'circle_height': 8.0,
                'circle_center_x': 0.0,
                'circle_center_y': 0.0,
                'horizon_stages': 50,
                'control_period': 0.02,
                'update_rate_hz': 50.0,
            }
        ],
        remappings=[
            ('reference_path', '/uav/reference_path'),
        ]
    )

    # 6. UAV Bootstrap Node (Handles Arming, Offboard request)
    uav_bootstrap_node = Node(
        package='test_uav_circle',
        executable='test_uav_circle_bootstrap_node',
        name='uav_bootstrap_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        usv_mpc_node,
        thruster_mapper_node,
        usv_circle_node,
        uav_mpc_node,
        uav_circle_node,
        uav_bootstrap_node
    ])
