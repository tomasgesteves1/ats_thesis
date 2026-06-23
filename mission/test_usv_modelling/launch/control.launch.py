from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. USV Automated Excitation Generator Node (System ID sequence)
    # This node publishes PRBS/Chirp sequences on the thrusters (forces and angles)
    # to excite the full 3-DOF dynamics of the WAM-V.
    excitation_node = Node(
        package='usv_mpc',
        executable='generate_excitation.py',
        name='usv_excitation_generator_node',
        output='screen',
        parameters=[{
            'frequency': 20.0,
            'total_duration': 120.0
        }]
    )

    return LaunchDescription([
        excitation_node
    ])
