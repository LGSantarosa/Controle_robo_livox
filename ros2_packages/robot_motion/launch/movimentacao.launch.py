"""Sobe a camada de movimentação do robô 2.

    ros2 launch robot_motion movimentacao.launch.py

Espera a base já de pé (`robot_base base.launch.py` no robô, ou
`robot_base sim.launch.py` no simulador): este nó só fala rumo, quem move
roda é o `diff_drive_controller`.

Para o simulador, passar `sim:=true` — sem isso o nó usa o relógio de parede
e o tempo da simulação não bate.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_motion')
    params = os.path.join(pkg, 'config', 'movimentacao.yaml')

    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true usa o relógio da simulação'),
        Node(
            package='robot_motion',
            executable='heading_controller',
            name='heading_controller',
            output='both',
            parameters=[params, {'use_sim_time': sim}],
        ),
    ])
