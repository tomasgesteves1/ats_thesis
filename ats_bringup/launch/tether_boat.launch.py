import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_ats_bringup = get_package_share_directory('ats_bringup')

    # 1. Mundo
    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'world.launch.py')
        )
    )

    # 2. Barco (WAM-V)
    boat = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'boat.launch.py')
        )
    )

    # 3. Tether (Cabo + Âncora + Bridge)
    tether = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'tether.launch.py')
        )
    )

    return LaunchDescription([
        world,
        boat,
        tether
    ])
