import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess, TimerAction, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Pacotes
    pkg_bringup = get_package_share_directory('bringup')
    pkg_description = get_package_share_directory('description')

    # Configurar Caminhos de Recursos
    local_models_dir = os.path.join(pkg_description, 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_resource_path = f"{local_models_dir}:{gz_resource_path}"

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_resource_path
    )

    # Argumentos
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.2')
    world_arg = DeclareLaunchArgument('world', default_value='wamv_world')
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Executar o Gazebo em modo headless (sem interface gráfica)'
    )

    # Caminho para o modelo SDF local
    boat_sdf_path = os.path.join(local_models_dir, 'wamv', 'model.sdf')

    # 1. Incluir o Mundo (Inicia Pausado)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'world.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'paused': 'true',
            'headless': LaunchConfiguration('headless'),
        }.items()
    )

    # 2. Spawn do WAM-V
    spawn_boat = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', boat_sdf_path,
            '-name', 'boat',
            '-allow_renaming', 'false',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z')
        ],
        output='screen'
    )

    # 3. Unpause da Física (após spawn do barco)
    unpause_gz = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/wamv_world/control',
             '--reqtype', 'gz.msgs.WorldControl',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '2000',
             '--req', 'pause: false'],
        output='screen'
    )

    unpause_physics_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_boat,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[unpause_gz]
                )
            ]
        )
    )

    # 4. Bridge de tópicos
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/left/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/boat/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/wamv_world/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
            '/world/wamv_world/stats@ros_gz_interfaces/msg/WorldStatistics[gz.msgs.WorldStatistics',
        ],
        remappings=[
            ('/wamv/thrusters/left/thrust', '/boat/thrusters/left/thrust'),
            ('/wamv/thrusters/right/thrust', '/boat/thrusters/right/thrust'),
            ('/wamv/thrusters/left/pos', '/boat/thrusters/left/pos'),
            ('/wamv/thrusters/right/pos', '/boat/thrusters/right/pos'),
            ('/model/boat/odometry', '/boat/ground_truth/odometry'),
            ('/world/wamv_world/stats', '/simulation/stats'),
        ],
        output='screen'
    )

    # 5. Foxglove Bridge
    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{
            'use_sim_time': True,
            'asset_uri_allowlist': ['.*']
        }]
    )

    # 6. Frame Manager
    frame_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('frame_manager'), 'launch', 'frame_manager.launch.py')
        )
    )

    return LaunchDescription([
        set_gz_resource_path,
        x_arg,
        y_arg,
        z_arg,
        world_arg,
        headless_arg,
        world_launch,
        spawn_boat,
        unpause_physics_handler,
        bridge,
        foxglove_bridge,
        frame_manager_launch,
    ])
