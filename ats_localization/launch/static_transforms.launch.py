from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Boat Anchor: x y z roll pitch yaw
        # Offset from boat center to the tether anchor point
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='boat_anchor_publisher',
            arguments=['-0.5', '0', '1.3', '0', '0', '0', 'boat/base_link', 'boat/tether_anchor']
        ),
        
        # Drone Hook: x y z roll pitch yaw
        # Offset from drone center to the tether hook point
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='drone_hook_publisher',
            arguments=['0', '0', '-0.1', '0', '0', '0', 'drone/base_link', 'drone/tether_hook']
        ),
    ])
