"""Simulador do robô 2 — Gazebo Harmonic.

Substitui SÓ o hardware: sobe o mesmo `controller_manager` e o mesmo
`diff_drive_controller` do robô real, com a interface de hardware do Gazebo no
lugar da placa serial. É o que faz o ajuste de movimentação feito aqui valer
alguma coisa lá fora.

    ros2 launch robot_base sim.launch.py

O robô fica em `/hoverboard_base_controller/cmd_vel` (TwistStamped, SI) e
publica a pose verdadeira em `/Odometry` — mesmo tópico e mesmo papel do LIO no
robô real, para que o controlador de movimentação não saiba a diferença.
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


def generate_launch_description():
    pkg = get_package_share_directory('robot_base')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    mundo_padrao = os.path.join(pkg, 'worlds', 'pista_livre.sdf')
    # Escolhido depois, em função do argumento `planta`.
    xacro_path = os.path.join(pkg, 'description', 'robo2.urdf.xacro')

    args = [
        DeclareLaunchArgument('mundo', default_value=mundo_padrao),
        # Onde o robô nasce. Padrão (0,0) para não mudar nada de quem já usa a
        # pista livre; na pista de obstáculos a origem cai DENTRO da parede
        # (o perímetro começa em 0), então lá é obrigatório passar um ponto
        # livre.
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false roda headless (útil para teste automatizado)'),
        DeclareLaunchArgument(
            'planta', default_value='lenta',
            description='"lenta" (a_dec 0,3 — pessimista, onde a movimentação '
                        'foi validada) ou "normal" (a_dec 1,5)'),
        DeclareLaunchArgument(
            'zona_morta', default_value='0.10',
            description='zona morta de RODA da placa simulada [m/s]; 0 = fio. '
                        'CHUTE — o valor real sai do tools/banco (BO-3)'),
    ]
    mundo = LaunchConfiguration('mundo')
    gui = LaunchConfiguration('gui')

    def descricao(contexto):
        # O xacro é processado aqui (e não por substitution) porque o caminho do
        # controllers_yaml precisa entrar DENTRO do URDF, no plugin do
        # gz_ros2_control.
        perfil = LaunchConfiguration('planta').perform(contexto).lower()
        nome = ('hoverboard_controllers_sim_lento.yaml' if perfil == 'lenta'
                else 'hoverboard_controllers_sim.yaml')
        return xacro.process_file(
            xacro_path,
            mappings={'sim': 'true',
                      'controllers_yaml': os.path.join(pkg, 'config', nome)},
        ).toxml()

    gz_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    # Com janela (ajuste manual, ver o robô andar) e sem janela (teste
    # automatizado, CI) — mesma simulação, só muda a interface.
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

    def robot_state_pub(contexto, *_a, **_k):
        return [Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': descricao(contexto),
                         'use_sim_time': True}],
        )]

    # A placa do hoverboard, fingida: engole comando de roda pequeno demais,
    # como a de verdade. Sem ela o simulador obedece qualquer coisa e o
    # controle é ajustado contra um atuador que não existe.
    placa = Node(
        package='robot_base',
        executable='placa_simulada',
        name='placa_simulada',
        output='both',
        parameters=[{'zona_morta': LaunchConfiguration('zona_morta'),
                     'bitola': 0.270,  # medida com trena 2026-07-29
                     'use_sim_time': True}],
    )

    # Spawn com z acima do solo: o robô assenta nas rodas na primeira iteração
    # da física. Nascer exatamente na altura final faz as rodas penetrarem o
    # chão e o robô sair pulando.
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robo2', '-z', '0.05',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y')],
        output='both',
    )

    # Ponte GZ↔ROS. O /Odometry é a pose VERDADEIRA do Gazebo — no robô real
    # quem publica esse tópico é o LIO. Mesmo nome de propósito.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/Odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf_odom@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[('/tf_odom', '/tf')],
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

    # Mesma ordem do robô real: o diferencial só depois do broadcaster de juntas.
    ordem = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[base_controller],
        )
    )
    # E os controladores só depois de o modelo existir no mundo.
    apos_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn, on_exit=[joint_state_broadcaster])
    )

    return LaunchDescription(args + [
        gazebo_com_gui,
        gazebo_headless,
        OpaqueFunction(function=robot_state_pub),
        placa,
        bridge,
        spawn,
        apos_spawn,
        ordem,
    ])
