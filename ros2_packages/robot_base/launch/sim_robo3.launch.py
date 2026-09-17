"""Simulador do ROBÔ 3 — Gazebo Harmonic.

Irmão do `sim.launch.py` (robô 2), que continua de pé: o robô 2 é a linha de
base contra a qual todo CSV histórico foi medido, e sobrepor os dois no mesmo
launch faria perder a comparação.

    ros2 launch robot_base sim_robo3.launch.py

⚠️ O QUE ESTE SIMULADOR AINDA NÃO SABE (`docs/ROBO3_REVISAO_CRUZADA.md`):
  - massa e centro de massa são PROVISÓRIOS: o Livox e o NUC nem subiram no
    robô. Não tire conclusão de inércia, transferência de peso ou tombamento.
  - a altura do Livox (0,24 m) é CHUTE: a decisão de onde montar é do dono, e
    a zona cega escala com ela (8,1 × altura).
  - a placa pode ser outra. O modelo de atuador aqui é o do robô 2.

O que ele JÁ serve para olhar, e é por isso que ele existe agora:
  - a forma: 47,5 cm de largura contra 33,2 de comprimento, quase 2× mais largo
    que comprido, com as motrizes em balanço fora da caixa;
  - o balanço dos QUATRO apoios rígidos sem mola (C3) — em piso irregular uma
    boba sai do chão e o corpo balança entre as diagonais;
  - o chicote das DUAS bobas de trail 20 mm numa inversão de marcha (C7).
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Geometria do robô 3. Mora aqui e no YAML do controlador, e os dois são
# conferidos em par por `test_urdf_robo3.py` — mudar um sozinho quebra o teste
# de propósito.
BITOLA = 0.320   # 🟢 trena 17-09: (38,0 + 26,0)/2. Era 0,3225 (Gazebo, §5.9)
RAIO = 0.0825    # catálogo do hub motor 6,5" (165 mm OD)


def generate_launch_description():
    pkg = get_package_share_directory('robot_base')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    mundo_padrao = os.path.join(pkg, 'worlds', 'pista_livre.sdf')
    xacro_path = os.path.join(pkg, 'description', 'robo3.urdf.xacro')
    controllers = os.path.join(pkg, 'config', 'hoverboard_controllers_sim_robo3.yaml')

    args = [
        DeclareLaunchArgument('mundo', default_value=mundo_padrao),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false roda headless'),
        DeclareLaunchArgument(
            'placa', default_value='medido',
            description='modelo do atuador — HERDADO DO ROBÔ 2 até a placa do '
                        'robô 3 ser decidida: "medido", "cru" ou "ideal"'),
    ]
    mundo = LaunchConfiguration('mundo')
    gui = LaunchConfiguration('gui')

    def descricao():
        return xacro.process_file(
            xacro_path,
            mappings={'sim': 'true', 'controllers_yaml': controllers},
        ).toxml()

    gz_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    gazebo_com_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': [mundo, ' -r ']}.items(),
        condition=IfCondition(gui),
    )
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={'gz_args': [mundo, ' -r -s --headless-rendering']}.items(),
        condition=UnlessCondition(gui),
    )

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': descricao(), 'use_sim_time': True}],
    )

    # A placa do hoverboard, fingida. ⚠️ `rendimento_giro` 0,80 é o do robô 2 na
    # planta lenta, MEDIDO NAQUELE CONTATO: bitola diferente e quatro apoios em
    # vez de três mudam quanto do giro pedido o chão entrega. Fica aqui como
    # ponto de partida declarado, e é dos primeiros a remedir na bancada.
    placa = Node(
        package='robot_base',
        executable='placa_simulada',
        name='placa_simulada',
        output='both',
        parameters=[{'modelo': LaunchConfiguration('placa'),
                     'bitola': BITOLA,
                     'raio': RAIO,
                     'rendimento_giro': 0.80,
                     'use_sim_time': True}],
    )

    # z acima do solo: o robô assenta nas rodas na primeira iteração da física.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robo3', '-z', '0.05',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-Y', LaunchConfiguration('yaw')],
        output='both',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/Odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf_odom@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # A NUVEM, não o LaserScan: o `gpu_lidar` publica os dois e só
            # `/points` presta. Ver decisão 017 — igualar o nome sem igualar o
            # tipo escondeu a diferença por três semanas.
            '/livox/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[('/tf_odom', '/tf'),
                    ('/livox/lidar/points', '/livox/pontos')],
        parameters=[{'use_sim_time': True}],
        output='both',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )
    base_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['hoverboard_base_controller', '--controller-manager', '/controller_manager'],
    )

    ordem = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[base_controller],
        )
    )
    apos_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])
    )

    scan_2d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'scan_2d.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription(args + [
        gazebo_com_gui,
        gazebo_headless,
        robot_state_pub,
        placa,
        bridge,
        spawn,
        apos_spawn,
        ordem,
        scan_2d,
    ])
