import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Pacotes
    pkg_ats_description = get_package_share_directory('ats_description')
    pkg_ats_gazebo = get_package_share_directory('ats_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_vrx_gazebo = get_package_share_directory('vrx_gazebo')
    pkg_wamv_description = get_package_share_directory('wamv_description')
    pkg_wamv_gazebo = get_package_share_directory('wamv_gazebo')

    # Configuração de Recursos do Gazebo
    gz_resource_path = os.pathsep.join([
        os.path.join(pkg_ats_description, 'models'),
        os.path.join(pkg_ats_gazebo, 'models'),
        os.path.join(pkg_vrx_gazebo, '..'),
        os.path.join(pkg_wamv_description, '..'),
        os.path.join(pkg_wamv_gazebo, '..')
    ])

    # Argumentos
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='wamv_world',
        description='Nome do mundo (sem .sdf)'
    )

    paused_arg = DeclareLaunchArgument(
        'paused',
        default_value='false',
        description='Iniciar simulação pausada'
    )

    # Gazebo Sim
    gz_args_base = [
        '-v 1 ',
        PathJoinSubstitution([pkg_ats_gazebo, 'worlds', LaunchConfiguration('world')]),
        '.sdf'
    ]

    # Condicionalmente adicionar -r se NÃO estiver pausado
    # Infelizmente o launch do ROS2 com listas é chato. Vamos usar uma PythonExpression.
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                PythonExpression(["'-r ' if '", LaunchConfiguration('paused'), "' == 'false' else ''"]),
                '-v 1 ',
                PathJoinSubstitution([pkg_ats_gazebo, 'worlds', LaunchConfiguration('world')]),
                '.sdf'
            ],
            'on_exit_shutdown': 'True'
        }.items()
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        world_arg,
        paused_arg,
        gz_sim,
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{'use_sim_time': True}]
        )
    ])
