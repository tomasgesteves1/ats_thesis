"""Intruder boat (kinematic) and the bridge of the Gazebo set_pose service used to move it."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('intruder_sim'), 'config', 'intruder_sim.yaml')
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='intruder_set_pose_bridge', output='screen',
        arguments=['/world/wamv_world/set_pose@ros_gz_interfaces/srv/SetEntityPose'])
    intruder = Node(
        package='intruder_sim', executable='intruder_sim_node', name='intruder_sim_node', output='screen',
        parameters=[cfg, {'use_sim_time': True}],
        remappings=[('uav/odom', '/drone/ground_truth/odometry'),
                    ('usv/odom', '/boat/ground_truth/odometry'),
                    ('odometry', '/intruder/odometry'),
                    ('set_pose', '/world/wamv_world/set_pose')])
    return LaunchDescription([bridge, intruder])
