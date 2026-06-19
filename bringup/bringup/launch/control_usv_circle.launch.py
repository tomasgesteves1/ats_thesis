import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_usv_mpc = get_package_share_directory('usv_mpc')
    
    # 1. Include the USV MPC launcher (which loads YAML and launches the parameter bridge)
    usv_mpc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_usv_mpc, 'launch', 'usv_mpc.launch.py')
        )
    )

    # 2. USV Circle Trajectory Generator Node
    usv_circle_trajectory = Node(
        package='trajectory_generator',
        executable='usv_trajectory_circle_node',
        name='usv_trajectory_circle_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'circle_radius': 15.0,     # 15m radius
                'circle_omega': 0.08,      # ~1.2 m/s linear velocity
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

    # Note: In usv_mpc.launch.py, we must override the trajectory_type parameter to 'external'
    # so that the MPC tracks the reference path instead of holding the initial position.
    # To do this, we can pass it as a parameter override when we launch usv_mpc_node.
    # In ROS 2, when including a launch file that contains a node, we can either pass parameters
    # via launch arguments if configured, or launch the node directly here.
    # Let's launch the nodes directly here to have full control of parameters and remappings,
    # which is simpler and more robust.
    
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
            }
        ],
        remappings=[
            ('odom', '/boat/ground_truth/odometry'),
            ('cmd_vel', '/cmd_vel'),
            ('reference_path', '/usv/reference_path'),
            ('target_position', '/usv/target_position')
        ]
    )

    # Bridge cmd_vel from ROS 2 to Gazebo Sim (/model/boat/cmd_vel)
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='usv_cmd_vel_bridge',
        arguments=[
            '/model/boat/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            ('/model/boat/cmd_vel', '/cmd_vel')
        ],
        output='screen'
    )

    return LaunchDescription([
        usv_mpc_node,
        cmd_vel_bridge,
        usv_circle_trajectory
    ])
