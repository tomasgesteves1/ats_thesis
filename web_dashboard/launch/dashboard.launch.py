"""Web dashboard (http://localhost:8080): launches the simulation and the missions of the mission executor."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='web_dashboard', executable='dashboard_node', name='web_dashboard_node', output='screen',
             parameters=[{'use_sim_time': False}]),
    ])
