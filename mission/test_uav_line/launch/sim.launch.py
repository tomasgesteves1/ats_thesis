import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_bringup = get_package_share_directory('bringup')
    
    use_tether_arg = DeclareLaunchArgument(
        'use_tether',
        default_value='true',
        description='Whether to enable MoorDyn tether simulation'
    )

    return LaunchDescription([
        use_tether_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_bringup, 'launch', 'moordyn_marsupial.launch.py')
            ),
            launch_arguments={
                'headless': 'true',
                'use_tether': LaunchConfiguration('use_tether')
            }.items()
        )
    ])
