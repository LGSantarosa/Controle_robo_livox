"""Base completa do robô 2: tração + localização.

É o piso verificado em hardware — o robô liga, aceita comando de velocidade e
sabe onde está. Nada aqui decide para onde ir; a camada de movimentação entra
por cima, publicando em `/hoverboard_base_controller/cmd_vel`.

    ros2 launch robot_base base.launch.py

Requisitos: robô LIGADO, placa hover no serial e Mid-360 na ethernet com os IPs
de `config/MID360_config.json` (ver `config/README.md`).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share = get_package_share_directory('robot_base')

    def incluir(nome):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', nome))
        )

    return LaunchDescription([
        incluir('tracao.launch.py'),
        incluir('localizacao.launch.py'),
    ])
