import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_ats_control = get_package_share_directory('ats_control')

    # 1. Controlo do Barco (inclui Joystick)
    boat_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_control, 'launch', 'boat_control.launch.py')
        )
    )

    # 2. Controlo do Drone (Tracking)
    drone_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_control, 'launch', 'drone_control.launch.py')
        )
    )

    return LaunchDescription([
        boat_control,
        drone_control
    ])
