"""Localização do robô 2: Livox Mid-360 + FAST-LIO.

O Mid-360 entrega nuvem 3D (`/livox/lidar`) e IMU (`/livox/imu`); o FAST-LIO
funde os dois e publica a pose em `/Odometry`. É a única fonte de posição do
robô — não há AMCL, não há mapa 2D, não há `/scan`.

Por que LIO e não odometria de roda: a odometria do diff_drive está em
`open_loop` e a roda de hover escorrega; ela serve de referência, não de
localização. O Mid-360 já traz a IMU embutida, então o LIO sai de graça em
termos de hardware.

⚠️ A origem do FAST-LIO zera a cada boot — a pose é relativa ao ponto onde o
robô ligou, não a um mapa global.
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource

_COMO_RESOLVER = (
    'Esses pacotes não são versionados neste repo (upstream de terceiros, em '
    'commits fixados). Traga-os com:\n'
    '    ./setup_livox.sh\n'
    'e depois: source install/setup.bash'
)


def _share(pkg):
    """Share do pacote, com erro que diz o que fazer em vez de só 'not found'."""
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        raise RuntimeError(
            f"Pacote '{pkg}' não encontrado — a camada de localização não está "
            f'instalada.\n{_COMO_RESOLVER}'
        ) from None


def _primeiro_existente(caminhos, oque):
    for c in caminhos:
        if os.path.exists(c):
            return c
    raise RuntimeError(
        f'{oque} não encontrado. Procurei em: {caminhos}\n{_COMO_RESOLVER}'
    )


def generate_launch_description():
    livox_share = _share('livox_ros_driver2')
    fastlio_share = _share('fast_lio')

    # O livox_ros_driver2 instala os launches ora em launch/, ora em
    # launch_ROS2/, dependendo de como o preparo pró-ROS2 rodou. Aceitar os dois
    # evita quebrar por causa do layout do upstream.
    livox_launch = _primeiro_existente([
        os.path.join(livox_share, 'launch_ROS2', 'msg_MID360_launch.py'),
        os.path.join(livox_share, 'launch', 'msg_MID360_launch.py'),
    ], 'launch do Livox Mid-360')

    fastlio_launch = _primeiro_existente([
        os.path.join(fastlio_share, 'launch', 'mapping.launch.py'),
        os.path.join(fastlio_share, 'launch_ROS2', 'mapping.launch.py'),
    ], 'launch do FAST-LIO')

    fastlio_cfg = _primeiro_existente([
        os.path.join(fastlio_share, 'config', 'mid360.yaml'),
        os.path.join(fastlio_share, 'config', 'MID360.yaml'),
    ], 'config mid360.yaml do FAST-LIO')

    return LaunchDescription([
        LogInfo(msg=f'[livox]    {livox_launch}'),
        LogInfo(msg=f'[fast_lio] {fastlio_launch} (cfg: {fastlio_cfg})'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(livox_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_launch),
            launch_arguments={'config_file': fastlio_cfg}.items(),
        ),
    ])
