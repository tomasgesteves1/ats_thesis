import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Pacotes locais
    pkg_description = get_package_share_directory('description')
    pkg_gazebo = get_package_share_directory('gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Configuração de Recursos do Gazebo (Apenas pastas locais)
    gz_resource_path = os.pathsep.join([
        os.path.join(pkg_description, 'models'),
        os.path.join(pkg_gazebo, 'models')
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

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Executar o Gazebo em modo headless (sem interface gráfica)'
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                PythonExpression(["'-s ' if '", LaunchConfiguration('headless'), "' == 'true' else ''"]),
                PythonExpression(["'-r ' if '", LaunchConfiguration('paused'), "' == 'false' else ''"]),
                '-v 1 ',
                PathJoinSubstitution([pkg_gazebo, 'worlds', LaunchConfiguration('world')]),
                '.sdf'
            ],
            'on_exit_shutdown': 'True'
        }.items()
    )

    # Hack para matar o Gazebo Headless que fica pendurado no Ctrl+C
    force_kill_gazebo = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['pkill', '-9', '-f', 'gz sim'],
                    name='kill_gz_sim'
                )
            ]
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path),
        world_arg,
        paused_arg,
        headless_arg,
        gz_sim,
        force_kill_gazebo
    ])
