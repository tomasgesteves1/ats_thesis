import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_usv_mpc = get_package_share_directory('usv_mpc')
    
    # Declare Launch Arguments
    radius_arg = DeclareLaunchArgument('circle_radius', default_value='15.0')
    omega_arg = DeclareLaunchArgument('circle_omega', default_value='0.08')
    mode_step_arg = DeclareLaunchArgument('mode_step', default_value='false')
    mode_open_loop_arg = DeclareLaunchArgument('mode_open_loop', default_value='false')
    step_x_arg = DeclareLaunchArgument('step_x', default_value='10.0')
    step_y_arg = DeclareLaunchArgument('step_y', default_value='0.0')
    step_yaw_arg = DeclareLaunchArgument('step_yaw', default_value='0.0')
    open_loop_X_arg = DeclareLaunchArgument('open_loop_X', default_value='500.0')
    open_loop_Y_arg = DeclareLaunchArgument('open_loop_Y', default_value='0.0')
    open_loop_N_arg = DeclareLaunchArgument('open_loop_N', default_value='0.0')

    # 1. USV Circle Trajectory Generator Node
    usv_circle_trajectory = Node(
        package='test_usv_circle',
        executable='test_usv_circle_node',
        name='usv_trajectory_circle_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'circle_radius': LaunchConfiguration('circle_radius'),
                'circle_omega': LaunchConfiguration('circle_omega'),
                'circle_center_x': 0.0,
                'circle_center_y': 0.0,
                'horizon_stages': 20,
                'control_period': 0.1,
                'update_rate_hz': 10.0,
                'mode_step': LaunchConfiguration('mode_step'),
                'mode_open_loop': LaunchConfiguration('mode_open_loop'),
                'step_x': LaunchConfiguration('step_x'),
                'step_y': LaunchConfiguration('step_y'),
                'step_yaw': LaunchConfiguration('step_yaw'),
                'open_loop_X': LaunchConfiguration('open_loop_X'),
                'open_loop_Y': LaunchConfiguration('open_loop_Y'),
                'open_loop_N': LaunchConfiguration('open_loop_N'),
            }
        ],
        remappings=[
            ('reference_path', '/usv/reference_path'),
            ('cmd_wrench', '/boat/cmd_wrench'),
        ]
    )

    # 2. USV MPC Node (Dynamic Model)
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
                'trajectory_type': 'external', # Override to dynamic path tracking
                'mode_open_loop': LaunchConfiguration('mode_open_loop'),
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

    # 3. Thruster Mapper Node (Inverse Thrust Allocation)
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
        radius_arg,
        omega_arg,
        mode_step_arg,
        mode_open_loop_arg,
        step_x_arg,
        step_y_arg,
        step_yaw_arg,
        open_loop_X_arg,
        open_loop_Y_arg,
        open_loop_N_arg,
        usv_mpc_node,
        thruster_mapper_node,
        usv_circle_trajectory
    ])
