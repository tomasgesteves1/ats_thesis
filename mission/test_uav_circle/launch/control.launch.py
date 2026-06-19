import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_mpc = get_package_share_directory('uav_mpc')
    
    # 1. UAV MPC Node
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
                'trajectory_type': 'external', # Override to read external path
            }
        ],
        remappings=[
            ('odom', '/drone/ground_truth/odometry'),
            ('tether_length', '/moordyn_tether_node/tether_length'),
            ('reference_path', '/uav/reference_path'),
        ]
    )

    # 2. UAV Circle Trajectory Generator Node
    uav_circle_node = Node(
        package='test_uav_circle',
        executable='test_uav_circle_node',
        name='uav_trajectory_circle_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'circle_radius': 3.0,
                'circle_omega': 0.2,
                'circle_height': 7.0,
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

    # 3. PX4 Bootstrap Node (Handles Arming, Takeoff, Offboard mode switch)
    uav_bootstrap_node = Node(
        package='draft_control',
        executable='px4_bootstrap_node',
        name='uav_bootstrap_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'takeoff_height': 7.0,
        }],
        remappings=[
            ('odom', '/drone/ground_truth/odometry'),
        ]
    )

    return LaunchDescription([
        uav_mpc_node,
        uav_circle_node,
        uav_bootstrap_node
    ])
