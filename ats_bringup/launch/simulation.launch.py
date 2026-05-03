import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    # 1. Obter caminhos de forma dinâmica (NUNCA usar caminhos absolutos /home/tomas/...)
    pkg_ats_gazebo = get_package_share_directory('ats_gazebo')
    pkg_ats_description = get_package_share_directory('ats_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_vrx_gazebo = get_package_share_directory('vrx_gazebo')
    pkg_wamv_description = get_package_share_directory('wamv_description')
    pkg_wamv_gazebo = get_package_share_directory('wamv_gazebo')
    
    # Adicionar o caminho dos modelos do PX4 para o Gazebo conseguir fazer o spawn do drone
    px4_dir = os.path.expanduser('~/PX4-Autopilot')
    px4_models_dir = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')

    world_path = os.path.join(pkg_ats_gazebo, 'worlds', 'wamv_world.sdf')
    models_path = os.path.join(pkg_ats_gazebo, 'models')
    
    # Substitui 'wamv_target.urdf.xacro' pelo nome real do teu ficheiro principal
    xacro_file = os.path.join(pkg_ats_description, 'urdf', 'wamv_target.urdf.xacro')

    gz_resource_path = os.pathsep.join([
        models_path,
        os.path.join(pkg_vrx_gazebo, '..'),
        os.path.join(pkg_wamv_description, '..'),
        os.path.join(pkg_wamv_gazebo, '..'),
        px4_models_dir
    ])

    # 2. Configurar o Gazebo
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r -v 4 {world_path}',
            'on_exit_shutdown': 'True'
        }.items()
    )

    # 3. Processar o XACRO -> URDF
    robot_description = Command(['xacro ', xacro_file])

    # 4. Publicar o estado do robô (Essencial para TFs, Sensores e Controlo)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 5. Fazer Spawn do robô no Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_description,
            '-name', 'wamv',
            '-allow_renaming', 'false',
            '-x', '0.0', '-y', '0.0', '-z', '1.0' # Posição inicial (Z alto para cair na água)
        ],
        output='screen'
    )

    # 6. A tua Ponte de Comunicação (Atualizada para os tópicos do VRX)
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
        ],
        remappings=[
            ('/model/wamv/odometry', '/wamv/ground_truth/odometry'),
        ],
        output='screen'
    )

    return LaunchDescription([
        set_gz_resource_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        bridge
    ])