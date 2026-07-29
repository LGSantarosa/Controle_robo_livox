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
                                    /hoverboard_base_controller/cmd_vel

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
from launch.conditions import IfCondition
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
    zona_morta = LaunchConfiguration('zona_morta')

    # O perfil da movimentação segue o do simulador quando `sim:=true`: os dois
    # arquivos diferem na zona morta suposta, e rodar o controlador pessimista
    # contra a planta otimista mede uma máquina que não existe (nota de 29-07).
    mov_params = PythonExpression(
        ["'", mov_params_sim, "' if '", sim, "' == 'true' else '",
         mov_params_real, "'"])

    servidores = ['map_server', 'planner_server', 'controller_server',
                  'bt_navigator']

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true sobe o Gazebo junto'),
        DeclareLaunchArgument('mapa', default_value=MAPA_PADRAO),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'zona_morta', default_value='0.10',
            description='zona morta de RODA da placa simulada [m/s]. CHUTE — '
                        'o valor real sai do tools/banco (BO-3)'),

        # ---------------------------------------------------- o simulador
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_base'),
                'launch', 'sim.launch.py')),
            condition=IfCondition(sim),
            # Nasce num ponto LIVRE da pista: a origem cai dentro da parede do
            # perímetro, que começa em 0.
            launch_arguments={'mundo': MUNDO_PADRAO, 'x': '2.0', 'y': '5.0',
                              'zona_morta': zona_morta}.items(),
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
        Node(package='robot_motion', executable='heading_controller',
             name='heading_controller', output='both',
             parameters=[mov_params, {'use_sim_time': sim}]),
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
