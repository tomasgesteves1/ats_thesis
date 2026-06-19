import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package directories
    pkg_draft_control = get_package_share_directory('draft_control')
    pkg_uav_mpc = get_package_share_directory('uav_mpc')

    # Launch configuration arguments
    predict_movement_arg = DeclareLaunchArgument(
        'predict_movement',
        default_value='true',
        description='Whether to predict future boat position (dynamic CV model) or output static repeated current boat position'
    )
    
    # 1. Boat Joystick Control
    # This launches both the joy driver and the boat controller node
    boat_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_draft_control, 'launch', 'boat_control.launch.py')
        )
    )

    # 2. UAV MPC Node
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

    # 3. UAV Boat Follower Trajectory Generator Node
    uav_follower_node = Node(
        package='uav_trajectory',
        executable='uav_trajectory_boat_follower_node',
        name='uav_trajectory_boat_follower_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'offset_x': 0.0,
                'offset_y': 0.0,
                'offset_z': 6.0, # Hover 6 meters above the boat
                'horizon_stages': 50,
                'control_period': 0.02,
                'update_rate_hz': 50.0,
                'predict_movement': LaunchConfiguration('predict_movement'),
            }
        ],
        remappings=[
            ('boat_odom', '/boat/ground_truth/odometry'),
            ('reference_path', '/uav/reference_path'),
        ]
    )

    return LaunchDescription([
        predict_movement_arg,
        boat_control,
        uav_mpc_node,
        uav_follower_node
    ])
