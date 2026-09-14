"""Dirigir o robô 3 no controle Xbox, pela MEGA, sem Livox.

    ros2 launch robot_nav controle_robo3.launch.py
    ros2 launch robot_nav controle_robo3.launch.py sinal:=-1.0       # inverte frente E giro
    ros2 launch robot_nav controle_robo3.launch.py porta:=/dev/ttyACM1

A cadeia (plano em docs/PLANO_CONTROLE_ROBO3.md):

    joy_node → teleop_twist_joy → twist_mux → cmd_vel_to_wheels → mega_bridge
      → MEGA (firmware/mega_bridge, 50 Hz fixos) → Serial1 → placa

Segure o LB e mexa o analógico esquerdo. RB = turbo. Soltou o LB, o robô para.
LB + direcional cima/baixo = reta pura, sem giro (dpad_reto).

Sem URDF, sem estimador de pose, sem autonomia: o objetivo é só ver o robô
responder. Os números são de partida e estão todos como argumento, para
corrigir sentido no laboratório sem recompilar.
"""
import fcntl
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# _IOR('j', 0x13, char[128]) — o ioctl que devolve o nome do joystick.
JSIOCGNAME_128 = 0x80806A13


def escolhe_joystick():
    """(device_id, motivo). Casa 'xbox' no nome; a numeração jsN muda por boot."""
    achados = []
    for caminho in sorted(glob.glob('/dev/input/js*')):
        try:
            with open(caminho, 'rb') as fh:
                raw = fcntl.ioctl(fh, JSIOCGNAME_128, bytes(128))
        except OSError:
            continue
        nome = raw.rstrip(b'\0').decode('utf-8', 'replace')
        dev_id = int(caminho.rsplit('js', 1)[1])
        if 'xbox' in nome.lower():
            return dev_id, f'{caminho} anuncia {nome!r}'
        achados.append((dev_id, nome))
    if achados:
        dev_id, nome = achados[0]
        return dev_id, (f'NENHUM Xbox; caindo no js{dev_id} ({nome!r}) — '
                        f'confira os botões com scripts/js_mapping.py')
    return 0, 'nenhum /dev/input/js* agora — joy_node vai esperar o controle'


def _monta(contexto, *_a, **_k):
    pkg = get_package_share_directory('robot_nav')
    dev_id, motivo = escolhe_joystick()
    sinal = ParameterValue(LaunchConfiguration('sinal'), value_type=float)

    return [
        LogInfo(msg=f'[robo3] controle em js{dev_id} — {motivo}'),
        LogInfo(msg='[robo3] LB = homem-morto · RB = turbo · analógico esquerdo dirige'),
        Node(
            package='robot_nav', executable='mega_bridge', name='mega_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('porta'),
                'baud': 230400,
            }],
        ),
        Node(
            package='robot_nav', executable='cmd_vel_to_wheels',
            name='cmd_vel_to_wheels', output='screen',
            parameters=[{
                'wheel_base': ParameterValue(
                    LaunchConfiguration('bitola'), value_type=float),
                'linear_scale': ParameterValue(
                    LaunchConfiguration('escala'), value_type=float),
                # Um sinal só para as duas rodas: em 10-09 `speed>0` andou de
                # ré, e inverter as duas inverte speed E steer juntos.
                'left_wheel_sign': sinal,
                'right_wheel_sign': sinal,
                'cmd_vel_topic': 'cmd_vel',
            }],
        ),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            output={'stdout': 'screen', 'stderr': 'log'},
            parameters=[{
                # device_id é o índice do SDL, NÃO o N do /dev/input/jsN: em
                # 14-09 o Xbox era js1 (js0 = mouse falso) e SDL ID 0, e com
                # device_id 1 o /joy ficou mudo. O robô 3 tem um controle só.
                'device_id': 0,
                # Drift de repouso do analógico só morde com o LB apertado —
                # que é justamente quando o robô anda.
                'deadzone': 0.10,
                # O joy_node só publica quando algo muda; analógico parado num
                # ângulo estouraria o timeout do mux com o LB ainda apertado.
                'autorepeat_rate': 20.0,
                # Com sticky o LB vira liga/desliga e o robô anda com ele solto.
                'sticky_buttons': False,
            }],
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            # O nome TEM de casar com a chave de topo do YAML, senão ele é
            # ignorado em silêncio e o homem-morto cai no botão 0.
            name='teleop_twist_joy_node', output='screen',
            parameters=[os.path.join(pkg, 'config', 'teleop_xbox_robo3.yaml')],
            remappings=[('cmd_vel', 'joy_vel')],
        ),
        Node(
            # LB + direcional cima/baixo = reta pura (giro zero), acima do
            # analógico no mux. Velocidades iguais às do teleop.
            package='robot_nav', executable='dpad_reto', name='dpad_reto',
            output='screen',
        ),
        Node(
            package='twist_mux', executable='twist_mux', name='twist_mux',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'twist_mux_robo3.yaml')],
            remappings=[('cmd_vel_out', 'cmd_vel')],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'porta', default_value='/dev/ttyACM0',
            description='USB da MEGA com firmware/mega_bridge'),
        DeclareLaunchArgument(
            'sinal', default_value='1.0',
            description='1.0 (14-09, aprovado pelo dono): frente = a das rodas, '
                        'giro e força bons. -1.0 inverte frente E giro juntos'),
        DeclareLaunchArgument(
            'bitola', default_value='0.3225',
            description='[m] centro a centro das motrizes (URDF do robô 3)'),
        DeclareLaunchArgument(
            'escala', default_value='400.0',
            description='[unidades da placa por m/s] de partida, não calibrada'),
        OpaqueFunction(function=_monta),
    ])
