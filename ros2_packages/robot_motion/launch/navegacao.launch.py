"""Pilha de movimento do robô 2: navegação por cima da movimentação.

    ros2 launch robot_motion navegacao.launch.py           # no robô
    ros2 launch robot_motion navegacao.launch.py sim:=true # no simulador

Espera a base já de pé (`robot_base base.launch.py` ou `sim.launch.py`).

Manda o robô a um ponto assim:

    ros2 topic pub -1 /goal_navigator/objetivo geometry_msgs/msg/PoseStamped \\
      "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 2.0}}}"

⚠️ NÃO desvia de obstáculo. Isso é a fatia B, e depende do Livox.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_motion')
    movimentacao = os.path.join(pkg, 'config', 'movimentacao.yaml')
    navegacao = os.path.join(pkg, 'config', 'navegacao.yaml')

    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true usa o relógio da simulação'),
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
        ),
    ])
