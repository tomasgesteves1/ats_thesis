import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
                'tether_max_length': ParameterValue(LaunchConfiguration('tether_max_length'), value_type=float),
            }
        ],
        remappings=[
            ('odom', '/boat/ground_truth/odometry'),
            ('thrusters/left/thrust', '/boat/thrusters/left/thrust'),
            ('thrusters/right/thrust', '/boat/thrusters/right/thrust'),
            ('thrusters/left/pos', '/boat/thrusters/left/pos'),
            ('thrusters/right/pos', '/boat/thrusters/right/pos'),
            ('uav_horizon', '/mpc_predicted_path'),
            ('reference_path', '/usv/reference_path'),
            ('target_position', '/usv/target_position'),
            ('mpc_horizon', '/usv/mpc_horizon')
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
                'circle_radius': ParameterValue(LaunchConfiguration('usv_circle_radius'), value_type=float),
                'circle_omega': 0.05,
                'circle_center_x': 0.0,
                'circle_center_y': 0.0,
                'horizon_stages': 20,
                'control_period': 0.1,
                'update_rate_hz': 10.0,
                'wait_for_drone': True,
                'start_altitude': ParameterValue(LaunchConfiguration('start_altitude'), value_type=float),
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
        condition=UnlessCondition(LaunchConfiguration('px4_baseline')),
        parameters=[
            uav_mpc_config,
            {
                'use_sim_time': True,
                'trajectory_type': 'external',
                'use_tether': True,
                'tether_model': ParameterValue(LaunchConfiguration('tether_model'), value_type=bool),
                'tether_max_length': ParameterValue(LaunchConfiguration('tether_max_length'), value_type=float),
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
                'circle_radius': ParameterValue(LaunchConfiguration('uav_circle_radius'), value_type=float),
                'circle_omega': ParameterValue(LaunchConfiguration('uav_circle_omega'), value_type=float),
                'circle_height': ParameterValue(LaunchConfiguration('uav_circle_height'), value_type=float),
                'circle_center_x': ParameterValue(LaunchConfiguration('uav_circle_center_x'), value_type=float),
                'circle_center_y': 0.0,
                'circle_z_amplitude': ParameterValue(LaunchConfiguration('uav_circle_z_amplitude'), value_type=float),
                'wait_for_drone': True,
                'start_altitude': ParameterValue(LaunchConfiguration('start_altitude'), value_type=float),
                'px4_offboard': ParameterValue(LaunchConfiguration('px4_baseline'), value_type=bool),
                'horizon_stages': 50,
                'control_period': 0.02,
                # Published well above the controller rate, so that the reference in use is at most a few ms old
                'update_rate_hz': 250.0,
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
        DeclareLaunchArgument('uav_circle_height', default_value='8.0', description='Height of the UAV reference circle (m)'),
        DeclareLaunchArgument('uav_circle_center_x', default_value='0.0', description='x of the centre of the UAV reference circle (m)'),
        DeclareLaunchArgument('uav_circle_omega', default_value='0.05', description='Angular speed of the UAV reference circle (rad/s)'),
        DeclareLaunchArgument('uav_circle_radius', default_value='5.0', description='Radius of the UAV reference circle (m)'),
        DeclareLaunchArgument('uav_circle_z_amplitude', default_value='0.0',
                              description='Height oscillation of the UAV reference circle (tilted circle) (m)'),
        DeclareLaunchArgument('usv_circle_radius', default_value='15.0',
                              description='Radius of the USV reference circle (m)'),
        DeclareLaunchArgument('start_altitude', default_value='3.0',
                              description='UAV altitude at which both reference circles start (m)'),
        DeclareLaunchArgument('tether_max_length', default_value='15.0', description='d_max of both NMPCs (m)'),
        DeclareLaunchArgument('px4_baseline', default_value='false',
                              description='Fly the UAV circle with the PX4 position controller instead of the NMPC'),
        DeclareLaunchArgument('tether_model', default_value='true',
                              description='Tether force in the UAV prediction model (false: tether-unaware NMPC)'),
        usv_mpc_node,
        usv_circle_node,
        uav_mpc_node,
        uav_circle_node,
        uav_bootstrap_node
    ])
