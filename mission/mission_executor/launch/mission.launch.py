"""Controllers of both vehicles and the mission executor (the simulation is launched separately, e.g.
bringup moordyn_marsupial.launch.py). Usage: ros2 launch mission_executor mission.launch.py mission:=landing_demo"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    uav_cfg = os.path.join(get_package_share_directory('uav_mpc'), 'config', 'uav_mpc.yaml')
    usv_cfg = os.path.join(get_package_share_directory('usv_mpc'), 'config', 'usv_mpc.yaml')
    exe_cfg = os.path.join(get_package_share_directory('mission_executor'), 'config', 'mission_executor.yaml')

    usv_mpc = Node(
        package='usv_mpc', executable='usv_mpc_node', name='usv_mpc_node', output='screen',
        parameters=[usv_cfg, {'use_sim_time': True, 'trajectory_type': 'external',
                              'tether_max_length': ParameterValue(LaunchConfiguration('tether_max_length'),
                                                                  value_type=float)}],
        remappings=[('odom', '/boat/ground_truth/odometry'),
                    ('thrusters/left/thrust', '/boat/thrusters/left/thrust'),
                    ('thrusters/right/thrust', '/boat/thrusters/right/thrust'),
                    ('thrusters/left/pos', '/boat/thrusters/left/pos'),
                    ('thrusters/right/pos', '/boat/thrusters/right/pos'),
                    ('uav_horizon', '/mpc_predicted_path'),
                    ('reference_path', '/usv/reference_path'),
                    ('mpc_horizon', '/usv/mpc_horizon')])

    uav_mpc = Node(
        package='uav_mpc', executable='uav_mpc_node', name='uav_mpc_node', output='screen',
        parameters=[uav_cfg, {'use_sim_time': True, 'trajectory_type': 'external', 'use_tether': True,
                              'disturbance_observer': ParameterValue(LaunchConfiguration('disturbance_observer'),
                                                                     value_type=bool),
                              'tether_max_length': ParameterValue(LaunchConfiguration('tether_max_length'),
                                                                  value_type=float)}],
        remappings=[('odom', '/drone/ground_truth/odometry'),
                    ('tether_length', '/moordyn_tether_node/tether_length'),
                    ('reference_path', '/uav/reference_path'),
                    ('boat_mpc_horizon', '/usv/mpc_horizon'),
                    ('yaw_from_reference', '/uav/yaw_from_reference'),
                    ('low_altitude', '/uav/low_altitude'),
                    ('boat_prediction', '/uav/boat_prediction'),
                    ('tether_pull', '/moordyn_tether_node/tether_pull')])

    executor = Node(
        package='mission_executor', executable='mission_executor_node', name='mission_executor_node',
        output='screen',
        parameters=[exe_cfg, {'use_sim_time': True, 'mission': LaunchConfiguration('mission')}],
        remappings=[('uav/odom', '/drone/ground_truth/odometry'),
                    ('usv/odom', '/boat/ground_truth/odometry'),
                    ('usv/mpc_horizon', '/usv/mpc_horizon'),
                    ('target/odom', '/intruder/odometry'),
                    ('px4/vehicle_status', '/px4_1/fmu/out/vehicle_status_v1'),
                    ('px4/vehicle_command', '/px4_1/fmu/in/vehicle_command'),
                    ('uav/reference_path', '/uav/reference_path'),
                    ('usv/reference_path', '/usv/reference_path'),
                    ('uav/yaw_from_reference', '/uav/yaw_from_reference'),
                    ('uav/low_altitude', '/uav/low_altitude'),
                    ('uav/boat_prediction', '/uav/boat_prediction'),
                    ('tether/pull', '/moordyn_tether_node/tether_pull')])

    # Intruder boat of the interdiction missions
    intruder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('intruder_sim'), 'launch',
                                                   'intruder_sim.launch.py')),
        condition=IfCondition(LaunchConfiguration('intruder')))

    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='landing_demo',
                              description='Mission file in share/mission_executor/missions (or absolute path)'),
        DeclareLaunchArgument('tether_max_length', default_value='15.0',
                              description='Maximum tether length d_max of both NMPCs (m)'),
        DeclareLaunchArgument('disturbance_observer', default_value='true',
                              description='Disturbance observer of the UAV NMPC'),
        DeclareLaunchArgument('intruder', default_value='false',
                              description='Run the intruder boat (interdiction missions)'),
        usv_mpc, uav_mpc, executor, intruder])
