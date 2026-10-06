import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_test_uav_line = get_package_share_directory('test_uav_line')
    
    # 1. (UAV NMPC intentionally not launched: this mission feeds the trajectory straight to the
    #    PX4 position controller in offboard mode, to obtain smooth data for model validation.)

    # 2. UAV Line Trajectory Generator Node
    test_uav_line_config = os.path.join(pkg_test_uav_line, 'config', 'test_uav_line.yaml')
    uav_line_node = Node(
        package='test_uav_line',
        executable='test_uav_line_node',
        name='uav_trajectory_line_node',
        output='screen',
        parameters=[
            test_uav_line_config,
            {'use_sim_time': True}
        ],
        remappings=[
            ('reference_path', '/uav/reference_path'),
            ('odom', '/drone/ground_truth/odometry'),
        ]
    )

    # 3. Custom Bootstrap Node (Handles Arming, immediately sets OFFBOARD mode to let MPC takeoff)
    uav_bootstrap_node = Node(
        package='test_uav_line',
        executable='test_uav_line_bootstrap_node',
        name='uav_bootstrap_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }]
    )

    return LaunchDescription([
        uav_line_node,
        uav_bootstrap_node
    ])
