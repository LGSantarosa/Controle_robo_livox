"""A pilha inteira do robô 2: Nav2 planeja, nós dirigimos.

    # no simulador, na pista de obstáculos:
    ros2 launch robot_motion pilha.launch.py sim:=true

    # no robô (base já de pé por robot_base base.launch.py):
    ros2 launch robot_motion pilha.launch.py

Espere `PILHA PRONTA` no terminal e clique **2D Goal Pose** no RViz. O robô vai.

A cadeia, e de quem é cada pedaço:

    GUI ou RViz → /goal_pose → bt_navigator → planner_server → /plan
                                                                 ↓
                                                     path_follower  (nosso)
                                                                 ↓
                                       ~/rumo_alvo + ~/velocidade_alvo
                                                                 ↓
                                                 heading_controller  (nosso)
                                                                 ↓
                                                     /auto_vel_raw
                                                                 ↓
                                          collision_monitor  (reflexo)
                                                                 ↓
                                          twist_mux  ← /key_vel, /web_vel
                                                       (humano FURA o reflexo)
                                                                 ↓
                                              /compensador_rumo/cmd_vel
                                                                 ↓
                                                 compensador_rumo  (nosso)
                                                                 ↓
                              /cmd_vel_bruto (sim) ou direto (robô)
                                                                 ↓
                                    /hoverboard_base_controller/cmd_vel

O `compensador_rumo` (decisão 011) é a última camada antes do atuador: ele
existe porque o robô comandado a ir RETO descreve um círculo de 1,22 m de raio
(−0,817 1/m de frente, −0,098 de ré, medidos em 04-08). Corrigir isso é
problema de todo comandante, então mora fora de todos eles.

O `controller_server` do Nav2 sobe junto e é IGNORADO de propósito: a árvore de
comportamento padrão usa `FollowPath` e sem esse servidor ela falha, levando o
replanejamento junto — que é justamente o que queremos do Nav2. O `cmd_vel` dele
sai num tópico que ninguém escuta. Ver `config/nav2.yaml`.

⚠️ **TF `map→odom` fixa e provisória.** A localização é LIO (decisão 003), que
entrega `odom`, e não há nada que case `odom` com o `map` do costmap. Enquanto
for assim, o mapa só serve para o robô nascer onde ele diz — no simulador isso
vale porque mundo e mapa saem da MESMA planta (`tools/mundo/gera_pista.py`).
**No robô real isto não vale**, e é por isso que o `mapa` é argumento: sem mapa
que corresponda ao lugar, o costmap global inventa obstáculo onde não tem.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

RAIZ = os.path.abspath(os.path.join(
    get_package_share_directory('robot_motion'), '..', '..', '..', '..'))
MAPA_PADRAO = os.path.join(RAIZ, 'maps', 'pista_obstaculos.yaml')
MUNDO_PADRAO = os.path.join(RAIZ, 'worlds', 'pista_obstaculos.sdf')


def generate_launch_description():
    pkg = get_package_share_directory('robot_motion')
    nav2_params = os.path.join(pkg, 'config', 'nav2.yaml')
    mux_params = os.path.join(pkg, 'config', 'twist_mux.yaml')
    cm_params = os.path.join(pkg, 'config', 'collision_monitor.yaml')
    mov_params_real = os.path.join(pkg, 'config', 'movimentacao.yaml')
    mov_params_sim = os.path.join(pkg, 'config', 'movimentacao_sim.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'pilha.rviz')
    # Árvore de comportamento SEM recuperação: replaneja a 1 Hz e segue. O
    # porquê está no `config/nav2.yaml` — em resumo, os nós de recuperação do
    # Nav2 são `spin` (pivô, que este robô não faz) e `backup` (a ré que a
    # decisão 009 tirou do Nav2), e ainda seriam no-op porque o comando deles
    # sai pelo tópico ignorado.
    bt_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees', 'navigate_w_replanning_time.xml')

    sim = LaunchConfiguration('sim')
    mapa = LaunchConfiguration('mapa')
    rviz = LaunchConfiguration('rviz')
    placa = LaunchConfiguration('placa')

    # O perfil da movimentação segue o do simulador quando `sim:=true`: os dois
    # arquivos diferem na zona morta suposta, e rodar o controlador pessimista
    # contra a planta otimista mede uma máquina que não existe (nota de 29-07).
    mov_params = PythonExpression(
        ["'", mov_params_sim, "' if '", sim, "' == 'true' else '",
         mov_params_real, "'"])

    servidores = ['map_server', 'planner_server', 'controller_server',
                  'bt_navigator', 'collision_monitor']

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true sobe o Gazebo junto'),
        DeclareLaunchArgument('mapa', default_value=MAPA_PADRAO),
        # `mundo` separado de `mapa` de propósito: é fazendo os dois
        # DISCORDAREM que se testa percepção. Mundo com um obstáculo que o
        # mapa não tem = o desvio só pode vir do sensor. Com os dois iguais
        # (o padrão, os dois saem de `gera_pista.py`) o robô poderia estar
        # desviando de memória e ninguém saberia.
        DeclareLaunchArgument('mundo', default_value=MUNDO_PADRAO),
        # ⚠️ PADRÃO `normal`, e isto é uma CORREÇÃO de 05-08. Esta launch não
        # passava `planta` nenhuma, então o `sim.launch.py` caía no default
        # dele (`lenta`) e a pilha inteira rodava contra a planta
        # DELIBERADAMENTE PESSIMISTA de 27-07 — aceleração angular de
        # 0,3 rad/s² contra 1,5. Sintoma: o pivô comandado por 1,4 s chegava a
        # 0,34 rad/s (0,3 × 1,4 = 0,42, bate), quando o robô real faz 2,33.
        #
        # A `lenta` nasceu quando o `a_dec` era ESTIMADO em ~0,5; hoje ele está
        # medido, e foi a planta `normal` que passou na aceitação de 04-08
        # (pico 2,25 contra 2,33 do robô). Ela continua disponível como teste
        # de estresse — mas não pode ser o que se mede por omissão.
        DeclareLaunchArgument(
            'planta', default_value='normal',
            description='"normal" (a que bate com o robô medido) ou "lenta" '
                        '(pessimista de 27-07, para estressar o controlador)'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'placa', default_value='medido',
            description='modelo do atuador simulado: "medido" (a placa de '
                        '31-07, com patamar), "cru" ou "ideal"'),

        # ---------------------------------------------------- o simulador
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_base'),
                'launch', 'sim.launch.py')),
            condition=IfCondition(sim),
            # Nasce num ponto LIVRE da pista: a origem cai dentro da parede do
            # perímetro, que começa em 0.
            launch_arguments={'mundo': LaunchConfiguration('mundo'),
                              'x': '2.0', 'y': '5.0',
                              'planta': LaunchConfiguration('planta'),
                              'placa': placa}.items(),
        ),

        # ---------------------------------------------------------- Nav2
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='both',
             parameters=[nav2_params, {'yaml_filename': mapa,
                                       'use_sim_time': sim}]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='both',
             parameters=[nav2_params, {'use_sim_time': sim}]),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='log',
             parameters=[nav2_params, {'use_sim_time': sim}],
             # O comando dele morre aqui. Quem dirige é a nossa cadeia.
             remappings=[('/cmd_vel', '/nav2_cmd_vel_ignorado')]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='both',
             parameters=[nav2_params, {'use_sim_time': sim,
                                       'default_nav_to_pose_bt_xml': bt_xml}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_pilha', output='both',
             parameters=[{'autostart': True, 'use_sim_time': sim,
                          'node_names': servidores}]),

        # A TF que falta: sem localização contra o mapa, `map` e `odom` são o
        # mesmo lugar. Provisório, e documentado no cabeçalho.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_map_odom', output='log',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': sim}]),

        # ------------------------------------------------------- os nossos
        #
        # A cadeia de comando, e por que ela tem esta forma (decisão 011):
        #
        #   heading_controller  --/compensador_rumo/cmd_vel-->
        #   compensador_rumo    --/cmd_vel_bruto (sim) ou direto (robô)-->
        #   [placa fingida, só no sim] --> diff_drive_controller
        #
        # O compensador é a ÚLTIMA camada antes do atuador de propósito: ele
        # corrige fidelidade de comando (o robô comandado reto arca −0,82 1/m),
        # e isso vale para QUALQUER comandante. Pôr o Nav2 ou o
        # heading_controller para brigar com o arco sozinhos é o que a fatia 1
        # tornou desnecessário.
        Node(package='robot_motion', executable='heading_controller',
             name='heading_controller', output='both',
             parameters=[mov_params, {'use_sim_time': sim}],
             remappings=[('/hoverboard_base_controller/cmd_vel',
                          '/auto_vel_raw')]),

        # ------------------------------------- o reflexo de colisão (05-08)
        # Filtra SÓ a autonomia (`auto_vel_raw` -> `auto_vel`), e o humano
        # entra DEPOIS dele, no mux: quem está com o teclado atravessa o
        # reflexo de propósito. É a única forma de tirar um robô que o próprio
        # reflexo prendeu contra uma parede.
        #
        # ⚠️ Ele é CEGO para obstáculo baixo e perto — o Mid-360 não vê o chão
        # dentro de ~2 m. Racional inteiro em `config/collision_monitor.yaml`.
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='both',
             parameters=[cm_params, {'use_sim_time': sim}]),

        # ---------------------------------------------- quem manda (05-08)
        # Árbitro de comando. Humano acima da autonomia, sempre. Até hoje a
        # pilha não tinha nenhum: o heading_controller publicava direto no
        # atuador e não havia como tomar o controle de um robô indo para a
        # parede. Racional e prioridades em `config/twist_mux.yaml`.
        Node(package='twist_mux', executable='twist_mux',
             name='twist_mux', output='both',
             parameters=[mux_params, {'use_sim_time': sim}],
             remappings=[('/cmd_vel_out', '/compensador_rumo/cmd_vel')]),

        # ⚠️ No simulador o comando TEM de passar pela placa fingida
        # (`/cmd_vel_bruto`), como já fazia o `navegacao.launch.py`. Esta
        # launch NÃO fazia: publicava direto no controlador e pulava a placa,
        # então toda corrida de pilha no Gazebo até 05-08 mediu um atuador
        # PERFEITO — sem patamar, sem latência, sem assimetria. É o mesmo
        # defeito que o `ensaio.py` tinha e que o `--topico` consertou.
        Node(package='robot_motion', executable='compensador_rumo',
             name='compensador_rumo', output='both',
             parameters=[{'use_sim_time': sim, 'segura_rumo': False}],
             remappings=[('/hoverboard_base_controller/cmd_vel',
                          '/cmd_vel_bruto')],
             condition=IfCondition(sim)),
        Node(package='robot_motion', executable='compensador_rumo',
             name='compensador_rumo', output='both',
             parameters=[{'use_sim_time': sim, 'segura_rumo': False}],
             condition=UnlessCondition(sim)),
        Node(package='robot_motion', executable='path_follower',
             name='path_follower', output='both',
             parameters=[{'use_sim_time': sim}],
             remappings=[('/path_follower/rumo_alvo',
                          '/heading_controller/rumo_alvo'),
                         ('/path_follower/velocidade_alvo',
                          '/heading_controller/velocidade_alvo')]),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': sim}],
             condition=IfCondition(rviz)),

        LogInfo(msg='PILHA subindo — espere os servidores ativarem '
                    '(o Smac leva ~16 s montando a heurística) e então clique '
                    '"2D Goal Pose" no RViz.'),
    ])
