#!/usr/bin/env python3
"""
Mapeamento por teleop: o robô desenha o próprio mapa da sala onde vai andar.

    ros2 launch robot_motion mapeia.launch.py
    # noutro terminal: bin/robot-key, e dirigir devagar pela sala
    # no fim:  ros2 run nav2_map_server map_saver_cli -f maps/<nome>/<nome>

🔴 POR QUE ESTA LAUNCH EXISTE (13-08, decisão 028)

O passo 6 do roteiro mandava localizar contra `maps/andar3/scan_andar3_ajustado`.
Ela é a versão LIMPA de um mapa do estágio, e a limpeza que dobrou a folga p10
(0,212 → 0,400 m) foi APAGAR A MOBÍLIA. A sala real está cheia de coisa. Medido
no robô, com a pose semeada e depois por busca exaustiva na sala inteira:

    melhor casamento possível    41,4% dos feixes a menos de 0,15 m de parede
    esperado (12-08, simulador)  100%
    erro mediano                 0,300 m

Mapa de planta limpa contra sala mobiliada não fecha em pose nenhuma — não é
pose errada, é mapa errado. O AMCL não tem o que casar.

⚠️ O QUE ESTA LAUNCH **NÃO** SOBE, e cada ausência tem motivo:

- `map_server` e `amcl`: quem publica `/map` aqui é o slam_toolbox, e quem
  publica `map → odom` também. Subir o AMCL junto é o defeito que o cabeçalho
  da `pilha.launch.py` marca em vermelho: dois donos da mesma TF.
- `tf_map_odom`: idem — identidade brigando com a correção do SLAM.
- os servidores do Nav2, o `path_follower` e o `heading_controller`: durante o
  mapeamento quem dirige é o humano. Autonomia enquanto se mapeia é robô
  perseguindo um mapa que está mudando debaixo dele.

✅ O que sobe é só a cadeia de comando do humano, a mesma da `pilha.launch.py`:

    bin/robot-key --/key_vel--> twist_mux --> placa

Com a roda omni o robô passou a andar reto. A compensação medida com a boba
foi removida também do mapeamento (decisão 063).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_motion')
    mux_params = os.path.join(pkg, 'config', 'twist_mux.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'resolucao', default_value='0.05',
            description='célula do mapa [m]. 0,05 é a dos mapas do andar 3 e a '
                        'do `robot_radius` 0,32 — trocar aqui e não trocar no '
                        'nav2.yaml é comparar folga com régua de outro tamanho'),

        LogInfo(msg='[mapeia] SLAM ligado: quem manda em /map e map→odom é o '
                    'slam_toolbox. Nada de AMCL nem tf_map_odom nesta launch.'),

        # ------------------------------------------------------- o SLAM
        #
        # ⚠️ Herdamos os números do `robot_nav/launch/slam.launch.py`, que é do
        # ROBÔ 1 (LD06 planar, skid-steer, SEM IMU) — lá o `minimum_travel_*`
        # pequeno existia para socorrer um seed de yaw de roda ruim no giro.
        # Aqui o seed vem do FAST-LIO, que é bom (018/019). Mantidos mesmo
        # assim, porque scan sobrando não estraga mapa; o que estraga é scan
        # de menos. Se o mapeamento ficar pesado no NUC, é o primeiro knob.
        Node(
            package='slam_toolbox', executable='async_slam_toolbox_node',
            name='slam_toolbox', output='both',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': '/scan',
                'mode': 'mapping',
                'resolution': ParameterValue(LaunchConfiguration('resolucao'),
                                             value_type=float),
                # Casa com o `range_max` do `scan_2d.yaml` (decisão 021): ler
                # mais longe do que a fatia entrega é pedir feixe que não vem.
                'max_laser_range': 20.0,
                'minimum_time_interval': 0.1,     # /scan é 10 Hz (medido: 10,03)
                'transform_publish_period': 0.05,
                'map_update_interval': 1.0,
                'transform_timeout': 0.5,
                'use_scan_matching': True,
                'minimum_travel_distance': 0.15,
                'minimum_travel_heading': 0.10,
                'scan_buffer_size': 20,
                # 🔴 O `slam_toolbox` do Jazzy é lifecycle node SEMPRE, e
                # `use_lifecycle_manager: False` NÃO o faz se auto-ativar:
                # medido hoje, ele nasce `unconfigured` e fica lá, calado —
                # sem erro, sem aviso, e o sintoma é `/map` que nunca aparece.
                # Quem transiciona é o manager abaixo.
                'use_lifecycle_manager': True,
            }],
        ),

        # ⚠️ Lista de UM nó de propósito. Servidor da lista que não responde
        # derruba o bringup inteiro (foi assim que o `collision_monitor` levou
        # a pilha junto em 06-08) — aqui só o SLAM está exposto a isso.
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_mapeia', output='both',
             parameters=[{'autostart': True, 'use_sim_time': False,
                          'node_names': ['slam_toolbox']}]),

        # ------------------------------------------- a cadeia do humano
        # Árbitro de comando. Aqui só o teclado (prioridade 90) e a web (50)
        # têm o que dizer; a autonomia nem sobe. Prioridades em twist_mux.yaml.
        Node(package='twist_mux', executable='twist_mux',
             name='twist_mux', output='both',
             parameters=[mux_params, {'use_sim_time': False}],
             remappings=[('/cmd_vel_out', '/hoverboard_base_controller/cmd_vel')]),
    ])
