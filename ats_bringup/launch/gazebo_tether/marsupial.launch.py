import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ats_bringup = get_package_share_directory('ats_bringup')
    pkg_ats_description = get_package_share_directory('ats_description')
    
    # 1. Configurações de Posição
    # Barco
    boat_x, boat_y, boat_z = 0.0, 0.0, 0.2
    # Drone (Pousado no Convés)
    drone_world_x, drone_world_z = 0.3, 1.65
    # Gancho (Onde o tether começa)
    hook_world_x, hook_world_z = -0.5, 1.5

    # Caminhos PX4
    px4_dir = os.environ.get('PX4_DIR', os.path.expanduser('~/PX4-Autopilot'))
    px4_build_dir = os.path.join(px4_dir, 'build', 'px4_sitl_default')

    # Configurar Caminhos de Recursos
    local_models_dir = os.path.join(pkg_ats_description, 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_resource_path = f"{local_models_dir}:{gz_resource_path}"

    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=new_gz_resource_path
    )

    # 1. Mundo (Inicia Pausado)
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ats_bringup, 'launch', 'world.launch.py')
        ),
        launch_arguments={'world': 'wamv_world', 'paused': 'true'}.items()
    )

    # 2. Spawn das Entidades
    spawn_boat = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'wamv_world', '-file', os.path.join(local_models_dir, 'wamv', 'model.sdf'),
                   '-name', 'wamv', '-x', str(boat_x), '-y', str(boat_y), '-z', str(boat_z)],
        output='screen'
    )

    spawn_drone = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'wamv_world', '-file', os.path.join(local_models_dir, 'x500', 'model.sdf'),
                   '-name', 'x500', '-x', str(drone_world_x), '-y', str(boat_y), '-z', str(drone_world_z)],
        output='screen'
    )

    spawn_tether = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'wamv_world', '-file', os.path.join(local_models_dir, 'tether', 'model.sdf'),
                   '-name', 'tether', '-x', str(hook_world_x), '-y', str(boat_y), '-z', str(hook_world_z)],
        output='screen'
    )

    # 3. Comando para Resume à Física
    unpause_gz = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/wamv_world/control', 
             '--reqtype', 'gz.msgs.WorldControl', 
             '--reptype', 'gz.msgs.Boolean', 
             '--timeout', '2000', 
             '--req', 'pause: false'],
        output='screen'
    )

    # Sequenciamento: Barco+Drone -> (2s) -> Tether -> (8s) -> Unpause
    delayed_spawn_tether = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_boat,
            on_exit=[
                TimerAction(
                    period=2.0,
                    actions=[spawn_tether]
                )
            ]
        )
    )

    unpause_physics_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_tether,
            on_exit=[
                TimerAction(
                    period=8.0,
                    actions=[unpause_gz]
                )
            ]
        )
    )

    # 4. Ligar o PX4 ao Drone
    px4_env = os.environ.copy()
    px4_env.update({
        'PX4_GZ_WORLD': 'wamv_world', 
        'PX4_GZ_MODEL_NAME': 'x500',
        'PX4_SYS_AUTOSTART': '4001',
        'GZ_SIM_RESOURCE_PATH': new_gz_resource_path
    })

    px4_sitl_process = TimerAction(
        period=12.0, 
        actions=[
            ExecuteProcess(
                cmd=[os.path.join(px4_build_dir, 'bin', 'px4'), '-i', '1'],
                cwd=px4_build_dir,
                output='screen',
                env=px4_env
            )
        ]
    )

    # 5. Micro XRCE-DDS Agent e Bridge
    micro_xrce_agent_path = os.path.join(
        get_package_prefix('micro_xrce_vendor'), 'lib', 'micro_xrce_vendor', 'MicroXRCEAgent'
    )
    micro_ros_agent = ExecuteProcess(
        cmd=[micro_xrce_agent_path, 'udp4', '-p', '8888'],
        output='log'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/wamv/thrusters/left/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/thrust@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/left/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/wamv/thrusters/right/pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/model/wamv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/x500/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/wamv/thrusters/left/thrust', '/boat/thrusters/left/thrust'),
            ('/wamv/thrusters/right/thrust', '/boat/thrusters/right/thrust'),
            ('/wamv/thrusters/left/pos', '/boat/thrusters/left/pos'),
            ('/wamv/thrusters/right/pos', '/boat/thrusters/right/pos'),
            ('/model/wamv/odometry', '/boat/ground_truth/odometry'),
            ('/model/x500/odometry', '/drone/ground_truth/odometry'),
        ],
        output='screen'
    )

    foxglove_bridge = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_gz_resource_path,
        world_launch,
        spawn_boat,
        spawn_drone,
        delayed_spawn_tether,
        unpause_physics_handler,
        px4_sitl_process,
        micro_ros_agent,
        bridge,
        foxglove_bridge
    ])
