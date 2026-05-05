import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_ats_bringup = get_package_share_directory('ats_bringup')

    # Argumentos de Missão
    world_name_arg = DeclareLaunchArgument('world', default_value='wamv_world')

    # 1. Mundo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'world.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # 2. Barco (WAM-V)
    boat_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'boat.launch.py')
        ),
        launch_arguments={'x': '0.0', 'y': '0.0', 'z': '1.0'}.items()
    )

    # 3. Drone (PX4 + MicroXRCE)
    drone_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'drone.launch.py')
        )
    )

    # 4. Foxglove Bridge (Opcional, mas útil para visualização)
    # Nota: Se não tiveres o foxglove_bridge instalado, podes comentar isto.
    try:
        foxglove_bridge_pkg = get_package_share_directory('foxglove_bridge')
        foxglove_bridge = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(foxglove_bridge_pkg, 'launch', 'foxglove_bridge_launch.xml')
            )
        )
    except Exception:
        foxglove_bridge = None

    ld = LaunchDescription([
        world_name_arg,
        world_launch,
        boat_launch,
        drone_launch,
    ])

    if foxglove_bridge:
        ld.add_action(foxglove_bridge)

    return ld
