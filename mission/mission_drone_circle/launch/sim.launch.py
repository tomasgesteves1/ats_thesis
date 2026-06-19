import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'moordyn_marsupial.launch.py')
            ),
            launch_arguments={
                'headless': 'true',
                'use_tether': 'true'
            }.items()
        )
    ])
