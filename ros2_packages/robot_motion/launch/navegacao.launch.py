"""Pilha de movimento do robô 2: navegação por cima da movimentação.

    ros2 launch robot_motion navegacao.launch.py                        # robô
    ros2 launch robot_motion navegacao.launch.py sim:=true              # simulador
    ros2 launch robot_motion navegacao.launch.py sim:=true rviz:=true   # + clicar

Espera a base já de pé (`robot_base base.launch.py` ou `sim.launch.py`).

O objetivo entra em **`/goal_pose`** (`geometry_msgs/PoseStamped`), que é o nome
padrão do ROS para isso. Com um só fio servem os três jeitos de mandar o robô a
um ponto:

- clicando em "2D Goal Pose" no RViz (`rviz:=true`);
- pela GUI web, que já publica nesse mesmo tópico;
- na mão:

      ros2 topic pub -1 /goal_pose geometry_msgs/msg/PoseStamped \\
        "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 2.0}}}"

⚠️ NÃO desvia de obstáculo. Isso é a fatia B, e depende do Livox.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nos(contexto, *_args, **_kwargs):
    pkg = get_package_share_directory('robot_motion')
    rviz_config = os.path.join(pkg, 'rviz', 'navegacao.rviz')

    sim = LaunchConfiguration('sim')
    rviz = LaunchConfiguration('rviz')
    topico_objetivo = LaunchConfiguration('topico_objetivo')

    # O simulador tem parâmetros PRÓPRIOS, e não é conveniência: lá a zona
    # morta é zero de verdade. Rodar o simulador com o chute pessimista do
    # robô real proibia o pivô e produzia arcos enormes.
    no_sim = sim.perform(contexto).lower() in ('true', '1', 'yes')
    sufixo = '_sim' if no_sim else ''
    movimentacao = os.path.join(pkg, 'config', f'movimentacao{sufixo}.yaml')
    navegacao = os.path.join(pkg, 'config', f'navegacao{sufixo}.yaml')

    return [
        Node(
            package='robot_motion',
            executable='heading_controller',
            name='heading_controller',
            output='both',
            parameters=[movimentacao, {'use_sim_time': sim}],
        ),
        Node(
            package='robot_motion',
            executable='goal_navigator',
            name='goal_navigator',
            output='both',
            parameters=[navegacao, {'use_sim_time': sim}],
            remappings=[('~/objetivo', topico_objetivo)],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': sim}],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true usa o relógio E os parâmetros '
                                          'da simulação'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='true abre o RViz já configurado '
                                          'para clicar o objetivo'),
        DeclareLaunchArgument(
            'topico_objetivo', default_value='/goal_pose',
            description='de onde vem o ponto de destino (PoseStamped)'),
        OpaqueFunction(function=nos),
    ])
