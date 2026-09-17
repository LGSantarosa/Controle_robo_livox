"""Base completa do robô 2: tração + localização.

É o piso verificado em hardware — o robô liga, aceita comando de velocidade e
sabe onde está. Nada aqui decide para onde ir; a camada de movimentação entra
por cima, publicando em `/hoverboard_base_controller/cmd_vel`.

    ros2 launch robot_base base.launch.py
    ros2 launch robot_base base.launch.py congela_parado:=true   # só o ensaio

Requisitos: robô LIGADO, placa hover no serial e Mid-360 na ethernet com os IPs
de `config/MID360_config.json` (ver `config/README.md`).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('robot_base')

    def incluir(nome, **args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', nome)),
            launch_arguments=args.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'congela_parado', default_value='false',
            description='Trava do AMCL (decisão 050): rodas paradas = '
                        'odom->base_link parado. PADRÃO DESLIGADO até passar '
                        'no robô — ligar só para o ensaio dela.'),
        incluir('tracao.launch.py'),
        # 🔴 DESLIGADA POR PADRÃO até passar no robô (17-09). A trava (decisão
        # 050, rodas paradas = odom parado) subia como `true` aqui, então
        # qualquer `reset --hard origin/main` + `sobe-robo` no NUC a IMPLANTAVA
        # sozinho — código não validado entrando em produção por inércia.
        # Para o ensaio dela, e só para ele:
        #     ros2 launch robot_base base.launch.py congela_parado:=true
        incluir('localizacao.launch.py',
                congela_parado=LaunchConfiguration('congela_parado')),
    ])
