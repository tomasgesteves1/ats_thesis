import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_uav_mpc = get_package_share_directory('uav_mpc')
    
    # Declare parameters as launch arguments
    takeoff_height_arg = DeclareLaunchArgument('takeoff_height', default_value='4.0')
    step_amplitude_xy_arg = DeclareLaunchArgument('step_amplitude_xy', default_value='3.0')
    step_amplitude_z_arg = DeclareLaunchArgument('step_amplitude_z', default_value='1.5')
    step_duration_arg = DeclareLaunchArgument('step_duration', default_value='8.0')

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
                'trajectory_type': 'external', # Read external path from Python node
            }
        ],
        remappings=[
            ('odom', '/drone/ground_truth/odometry'),
            ('tether_length', '/moordyn_tether_node/tether_length'),
            ('reference_path', '/uav/reference_path'),
        ]
    )

    # 2. UAV Step Trajectory Generator Node (our python node)
    excitation_node = Node(
        package='test_uav_modelling',
        executable='excitation_node.py',
        name='uav_excitation_node',
        output='screen',
        parameters=[{
            'takeoff_height': LaunchConfiguration('takeoff_height'),
            'step_amplitude_xy': LaunchConfiguration('step_amplitude_xy'),
            'step_amplitude_z': LaunchConfiguration('step_amplitude_z'),
            'step_duration': LaunchConfiguration('step_duration'),
            'use_sim_time': True
        }],
        remappings=[
            ('reference_path', '/uav/reference_path'),
        ]
    )

    # 3. UAV Bootstrap Node (handles arming and offboard transition, then shuts down)
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
        takeoff_height_arg,
        step_amplitude_xy_arg,
        step_amplitude_z_arg,
        step_duration_arg,
        uav_mpc_node,
        excitation_node,
        uav_bootstrap_node
    ])
