import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ats_control = get_package_share_directory('ats_control')

    # 1. Incluir o driver do Joystick
    joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_control, 'launch', 'joystick.launch.py')
        )
    )

    # 2. Lançar o Controlador do Barco (WAM-V)
    usv_controller = Node(
        package='ats_control',
        executable='boat_controller',
        name='usv_controller',
        output='screen'
    )

    return LaunchDescription([
        joy_launch,
        usv_controller
    ])
