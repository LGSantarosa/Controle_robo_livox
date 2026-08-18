"""Teleop por controle Xbox Series X|S — o freio de mão físico do robô 2.

    # no robô, com a pilha já de pé (bin/sobe-robo faz isso sozinho):
    ros2 launch robot_motion joystick.launch.py

    # se a auto-detecção errar o device:
    ros2 launch robot_motion joystick.launch.py dev:=1

Sobe dois nós e nada mais:

    joy_node            /dev/input/jsN  →  /joy
    teleop_twist_joy    /joy            →  /joy_vel   (prio 100 no twist_mux)

Segure o **LB** e mexa o analógico esquerdo. Soltou o LB, o robô para — isso é
o homem-morto, e é o motivo de este canal poder ter a prioridade mais alta da
máquina (ver `config/twist_mux.yaml`).  **RB** = turbo.

─── POR QUE UMA LAUNCH SEPARADA, e não dentro da `pilha.launch.py`

Porque o controle é do OPERADOR, não da missão. A pilha sobe e desce por causa
de mapa, localização e Nav2; o joystick tem que sobreviver a tudo isso — e,
principalmente, tem que poder subir SOZINHO, com a pilha inteira morta, para
tirar o robô de um lugar ruim. Amarrar o freio de mão ao ciclo de vida daquilo
de que ele é o freio seria o erro.

⚠️ Ele NÃO é órfão do mux, porém: sem o `twist_mux` de pé, o `/joy_vel` não vai
para atuador nenhum. O que esta launch garante é que a recíproca não vale.

─── DUAS COISAS AQUI QUE PARECEM DETALHE E NÃO SÃO

1. `autorepeat_rate`. O `joy_node` só publica QUANDO ALGO MUDA. Analógico
   segurado parado num ângulo = nenhuma mensagem nova = o `timeout: 0.5` do mux
   expira = o mux devolve o comando para a autonomia com o operador ainda
   segurando o LB. O robô sairia da mão de quem está com ele na mão. O
   `autorepeat_rate` reemite o último estado e é o que fecha esse buraco.

2. `deadzone`. Analógico de Xbox tem drift de repouso. Com o homem-morto isso
   só morde enquanto o LB está apertado — mas é justamente aí que morde. 0,10
   é folgado de propósito: perder 10% de curso fino num robô que já tem giro
   parado em quantum de ~95° (ver `teleop_xbox.yaml`) não custa nada, e um
   robô que anda sozinho com o analógico em repouso custa caro.
"""
import fcntl
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# _IOR('j', 0x13, char[128]) — o ioctl que devolve o nome do joystick.
JSIOCGNAME_128 = 0x80806A13


def nome_do_js(caminho):
    """Nome que o driver anuncia para /dev/input/jsN, ou None se não der."""
    try:
        with open(caminho, 'rb') as fh:
            raw = fcntl.ioctl(fh, JSIOCGNAME_128, bytes(128))
    except OSError:
        return None  # sumiu no meio, ou sem permissão
    return raw.rstrip(b'\0').decode('utf-8', 'replace')


def escolhe_device():
    """Devolve (device_id, nome, motivo) do joystick a usar.

    Por que auto-detectar em vez de cravar js0: o NUC pode ter mais de um
    device de entrada, e a numeração do joydev depende da ORDEM em que as
    coisas conectaram no boot — o mesmo controle nasce js0 num dia e js1 no
    outro. Quem sobe o robô no dia a dia roda `bin/sobe-robo` e mais nada, então
    errar aqui é o robô "não responder ao controle" sem ninguém saber por quê.

    Casa por substring 'xbox' no nome anunciado. Sem nenhum js*, devolve 0 —
    o `joy_node` sobe e fica esperando o device aparecer, que é o
    comportamento certo quando o controle ainda não ligou.
    """
    forcado = os.environ.get('ROBOT_JOY_DEVICE')
    if forcado:
        return int(forcado), None, f'forçado por ROBOT_JOY_DEVICE={forcado}'

    achados = []
    for caminho in sorted(glob.glob('/dev/input/js*')):
        nome = nome_do_js(caminho)
        if nome is None:
            continue
        dev_id = int(caminho.rsplit('js', 1)[1])
        achados.append((dev_id, nome))
        if 'xbox' in nome.lower():
            return dev_id, nome, f'{caminho} anuncia {nome!r}'

    if achados:
        dev_id, nome = achados[0]
        return dev_id, nome, (f'NENHUM device com "Xbox" no nome; caindo no '
                              f'js{dev_id} ({nome!r}) — confira o mapa de '
                              f'botões com scripts/js_mapping.py')
    return 0, None, 'nenhum /dev/input/js* agora — joy_node vai esperar o controle'


def _monta(contexto, *_a, **_k):
    forcado = LaunchConfiguration('dev').perform(contexto)
    if forcado != 'auto':
        dev_id, nome, motivo = int(forcado), None, f'dev:={forcado} na linha de comando'
    else:
        dev_id, nome, motivo = escolhe_device()

    cfg = os.path.join(get_package_share_directory('robot_motion'),
                       'config', 'teleop_xbox.yaml')

    return [
        LogInfo(msg=f'[joystick] usando js{dev_id} — {motivo}'),
        LogInfo(msg=('[joystick] LB = homem-morto (segure para andar) · '
                     'RB = turbo · analógico esquerdo dirige')),
        Node(
            package='joy', executable='joy_node', name='joy_node',
            output='both',
            parameters=[{
                'device_id': dev_id,
                # Ver bloco 2 do cabeçalho.
                'deadzone': 0.10,
                # Ver bloco 1 do cabeçalho — sem isto o mux larga o operador.
                'autorepeat_rate': 20.0,
                # `sticky_buttons` MUITO explicitamente falso: com ele o LB
                # vira liga/desliga em vez de homem-morto, e o robô continua
                # andando com o botão solto. É o oposto do que este canal é.
                'sticky_buttons': False,
            }],
        ),
        Node(
            package='teleop_twist_joy', executable='teleop_node',
            # ⚠️ O nome TEM que casar com a chave de topo do teleop_xbox.yaml.
            # Nome diferente = YAML ignorado em silêncio = escalas e botões
            # nos defaults do pacote (enable_button 0, scale 0.5/1.0), ou seja,
            # homem-morto no botão errado.
            name='teleop_twist_joy_node',
            output='both',
            parameters=[cfg],
            remappings=[('cmd_vel', '/joy_vel')],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'dev', default_value='auto',
            description='ID do /dev/input/jsN, ou "auto" para detectar pelo nome'),
        OpaqueFunction(function=_monta),
    ])
