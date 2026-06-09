import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_draft_control = get_package_share_directory('draft_control')

    # 1. Boat Control
    boat_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_draft_control, 'launch', 'boat_control.launch.py')
        )
    )

    # 2. Drone Tracker
    drone_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_draft_control, 'launch', 'drone_control.launch.py')
        )
    )

    return LaunchDescription([
        boat_control,
        drone_control
    ])
