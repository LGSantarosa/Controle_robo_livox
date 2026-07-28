"""Bancada do planner do robô 2 — julga o CAMINHO, sem mover nada.

    ros2 launch robot_planning bancada_planner.launch.py

No RViz: **"2D Pose Estimate"** marca de onde o robô sairia, **"2D Goal Pose"**
marca o destino. Os dois planners desenham, e a tabela com os números sai no
terminal. Nada se move: não há robô, não há simulador, não há sensor.

Sobe: `map_server` (a pista gerada por `tools/mundo/gera_pista.py`),
`planner_server` com Theta* e Smac Hybrid-A*, o `lifecycle_manager` que ativa
os dois, e a bancada.

⚠️ **TF fixa provisória.** O costmap do Nav2 exige a pose do robô mesmo só
planejando, e aqui não há robô. Duas transformadas fixas (`map→odom→base_link`)
seguram essa exigência; o caminho é calculado entre os dois pontos clicados
(`use_start`), então a TF fixa não influencia o resultado. Quando o modelo 3D
entrar no simulador, ela sai e quem publica é a odometria de verdade.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# A pista vive no repo, não no pacote instalado: ela é gerada e versionada em
# maps/, e o mundo do Gazebo sai da MESMA planta.
RAIZ = os.path.abspath(os.path.join(
    get_package_share_directory('robot_planning'),
    '..', '..', '..', '..'))
MAPA_PADRAO = os.path.join(RAIZ, 'maps', 'pista_obstaculos.yaml')


def generate_launch_description():
    pkg = get_package_share_directory('robot_planning')
    params = os.path.join(pkg, 'config', 'bancada_planner.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'bancada_planner.rviz')

    mapa = LaunchConfiguration('mapa')
    rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mapa', default_value=MAPA_PADRAO,
            description='mapa da pista (gerado por tools/mundo/gera_pista.py)'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='false sobe só a bancada, sem interface'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='both',
            parameters=[params, {'yaml_filename': mapa}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='both',
            parameters=[params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_bancada',
            output='both',
            parameters=[{'autostart': True,
                         'node_names': ['map_server', 'planner_server']}],
        ),

        # --- TF provisória: só para o costmap parar de pedir pose de robô ---
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_map_odom', output='log',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_odom_base', output='log',
             arguments=['--frame-id', 'odom', '--child-frame-id', 'base_link']),

        Node(
            package='robot_planning',
            executable='bancada_planner',
            name='bancada_planner',
            output='both',
            parameters=[params],
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
            output='log',
        ),
    ])
