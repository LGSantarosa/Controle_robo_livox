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

⚠️ **TF `map→odom` fixa por padrão.** A localização é LIO (decisão 003), que
entrega `odom`, e por padrão nada casa `odom` com o `map` do costmap: o
`tf_map_odom` publica identidade. Assim o mapa só serve para o robô nascer onde
ele diz — no simulador isso vale porque mundo e mapa saem da MESMA planta
(`tools/mundo/gera_pista.py`).

🗺️ **`localizacao:=amcl` troca isso por localização de verdade** (decisão 022):
o AMCL casa o `/scan` — a fatia 2D da nuvem, decisão 021 — contra o mapa e
publica `map→odom` de fato. É o que destrava mapa grande (corredor, andar), onde
a janela rolante de 20 m da decisão 015 não alcança.

    ros2 launch robot_motion pilha.launch.py \
        mapa:=maps/meu_mapa/meu_mapa.yaml localizacao:=amcl \
        pose_x:=0.0 pose_y:=0.0 pose_yaw:=0.0

🔴 **O AMCL e o `tf_map_odom` NUNCA sobem juntos** — publicam a mesma TF, e
juntos a pose pisca entre "identidade" e a verdade a cada consulta. A launch
garante a exclusão, e há teste.
🔴 **A pose inicial vem por PARÂMETRO porque o NUC não tem tela**: o "2D Pose
Estimate" do RViz não existe no robô real. Sem ela o filtro nasce espalhado pelo
mapa inteiro.

🔴 **NO ROBÔ REAL, RODE COM `mapa:=nenhum`.** O mapa padrão é a planta da pista
SIMULADA — uma sala de 12 × 8 m que não existe em lugar nenhum. Com ele, o
costmap global nasce com parede onde não há nada e livre onde há parede, e desde
a decisão 014 ainda MISTURA isso com marcação real e permanente do Livox. Meio
mapa é mais difícil de diagnosticar do que mapa nenhum.

    ros2 launch robot_motion pilha.launch.py mapa:=nenhum        # o robô
    ros2 launch robot_motion pilha.launch.py sim:=true           # o simulador

Sem mapa: nada de `map_server`, nada de `static_layer`, e o costmap global vira
uma janela de 20 × 20 m que anda com o robô, alimentada só pelo sensor. O
racional está em `config/nav2_sem_mapa.yaml`, e a consequência a ter em mente é
que o robô planeja com MEMÓRIA CURTA — o que ele nunca viu conta como livre.
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

RAIZ = os.path.abspath(os.path.join(
    get_package_share_directory('robot_motion'), '..', '..', '..', '..'))
MAPA_PADRAO = os.path.join(RAIZ, 'maps', 'pista_obstaculos.yaml')
MUNDO_PADRAO = os.path.join(RAIZ, 'worlds', 'pista_obstaculos.sdf')


def _recusa_combinacao_sem_sentido(contexto, *_args, **_kwargs):
    """Combinação que não existe morre AQUI, e não trinta segundos depois.

    `localizacao:=amcl` sem mapa não é uma pilha degradada: é uma pilha que não
    funciona. O AMCL casa scan contra mapa — sem `map_server` ele sobe, fica
    esperando um mapa que nunca vem, não ativa, e como o `tf_map_odom` também
    não sobe (exclusão mútua), a árvore TF fica partida e o Nav2 inteiro não
    ativa. O sintoma seria o bringup abortando, que já apareceu em 06-08 por
    outro motivo e custou uma sessão para diagnosticar.
    """
    mapa = LaunchConfiguration('mapa').perform(contexto)
    loc = LaunchConfiguration('localizacao').perform(contexto)
    if loc not in ('fixa', 'amcl'):
        raise RuntimeError(
            f'localizacao:={loc!r} não existe. Use "fixa" ou "amcl".')
    if loc == 'amcl' and mapa == 'nenhum':
        raise RuntimeError(
            'localizacao:=amcl exige um mapa, e mapa:=nenhum é o default do '
            'robô real (decisão 015/019). Passe mapa:=<caminho.yaml> junto — '
            'ou fique em localizacao:=fixa, que é o perfil sem mapa.')
    return []


def generate_launch_description():
    # Carimbo de tempo da SUBIDA — resolvido aqui, em Python, e não por
    # substituição: `PythonExpression` com `__import__` funciona até o dia
    # em que não funciona, e o preço seria a corrida sem registro.
    CARIMBO = time.strftime('%Y-%m-%d_%H%M%S')
    pkg = get_package_share_directory('robot_motion')
    nav2_params = os.path.join(pkg, 'config', 'nav2.yaml')
    amcl_params = os.path.join(pkg, 'config', 'localizacao_amcl.yaml')
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
    # 🔴 ÁRVORE PRÓPRIA DESDE a decisão 026: a de fábrica não suaviza, e
    # nenhuma das doze do Jazzy chama `SmoothPath`. O plano do Theta* ia cru
    # para o seguidor, com quinas de 23° a 46 cm da porta — dentro da mira de
    # 0,37 m dele, e portanto impossíveis de seguir. A recuperação continua
    # sendo ZERO; o arquivo diz por quê.
    bt_xml = os.path.join(pkg, 'behavior_trees',
                          'replanejamento_com_suavizacao.xml')

    sem_mapa_params = os.path.join(pkg, 'config', 'nav2_sem_mapa.yaml')

    sim = LaunchConfiguration('sim')
    # ⚠️ `ParameterValue(..., value_type=float)` e não a substituição crua: o
    # argumento de launch chega como TEXTO, e o nó declarou `curv_frente` como
    # double. Passar cru derruba o compensador na subida com "parameter type
    # mismatch" — e um compensador que não sobe é o robô arcando 0,82 1/m com
    # a pilha inteira de pé.
    curv = [
        {'curv_frente': ParameterValue(LaunchConfiguration('curv_frente'),
                                       value_type=float),
         'curv_re': ParameterValue(LaunchConfiguration('curv_re'),
                                   value_type=float),
         'curv_medido_em': ParameterValue(LaunchConfiguration('curv_medido_em'),
                                          value_type=str)},
    ]
    # 🔴 GANHO DA CADEIA DE GIRO (decisão 038). A fração do `wz` pedido que o
    # robô entrega — 0,45 medido no Gazebo em 13-08 (`ganho_de_giro.py`), e
    # NÃO medido no robô real, onde o default 1,0 continua valendo.
    #
    # Por que ele mora aqui e não num YAML: o valor é da MÁQUINA, e as duas
    # máquinas deste projeto (Gazebo e robô) usam o mesmo `nav2.yaml` e o mesmo
    # perfil de movimentação. Ficar no launch deixa a diferença visível na
    # linha de comando, que é onde o `curv_medido_em` da 013 também mora.
    #
    #     ros2 launch robot_motion pilha.launch.py ganho_wz:=0.45
    #
    # ⚠️ O mesmo cuidado de tipo do `curv_frente`: argumento de launch chega
    # como TEXTO e o nó declarou double.
    ganho = [
        {'ganho_wz': ParameterValue(LaunchConfiguration('ganho_wz'),
                                    value_type=float)},
    ]
    # ⚠️ Mesmo cuidado do `curv_frente` acima: o nó declara `lookahead_piso`
    # como double, e argumento de launch chega como TEXTO. Cru, o seguidor cai
    # na subida com "parameter type mismatch" — e sem seguidor a pilha sobe
    # inteira sem ninguém dirigindo.
    #
    # Por que este knob virou argumento (12-08, no robô): com o Nav2 dirigindo,
    # a REFERÊNCIA de rumo é que oscila — `rumo_alvo` variou 30,6° e o `yaw`
    # seguiu com 32,9°, ou seja, o laço de rumo obedece e quem serpenteia é o
    # plano (replanejado 8x em 8 s). Mirar a 0,30 m andando a ~0,42 m/s é olhar
    # 0,7 s à frente dentro de uma malha com 0,94 s de tempo morto: o seguidor
    # persegue rabisco de plano que ele não tem tempo de responder.
    olhar = [
        {'lookahead_piso': ParameterValue(
            LaunchConfiguration('lookahead_piso'), value_type=float)},
        # 039 — ver a declaração do argumento para por que ele existe e por
        # que vai a 0.0 na primeira corrida no robô.
        {'k_lat': ParameterValue(
            LaunchConfiguration('k_lat'), value_type=float)},
        {'log_dir': ParameterValue(
            LaunchConfiguration('log_dir'), value_type=str)},
        {'desvio_taxa_deg_s': ParameterValue(
            LaunchConfiguration('desvio_taxa_deg_s'), value_type=float)},
        # 🔴 O FREIO DA RÉ, e ele é do DONO (13-08). A ré da 025 dirige por
        # FORA do reflexo (canal `unstuck_vel`, prioridade 30) — é a única
        # coisa neste robô que anda sem o freio de mão automático. Quem está na
        # sala com ele tem de poder dizer "hoje não recua", e até hoje não
        # tinha como: o nó lê `re_max_seguidas` uma vez, na subida, e não tem
        # callback de parâmetro — `ros2 param set` não chega lá.
        #
        # `re_habilitada:=false` é o knob PRÓPRIO para isso — ele já existia no
        # nó, com mensagem própria e reinício do contador de progresso. Em
        # 13-08 eu usei `re_max_seguidas:=0` por não o ter procurado, e o teto
        # zero matava o seguidor (ver o conserto no `path_follower.entra_na_re`).
        {'re_habilitada': ParameterValue(
            LaunchConfiguration('re_habilitada'), value_type=bool)},
        {'re_max_seguidas': ParameterValue(
            LaunchConfiguration('re_max_seguidas'), value_type=int)},
        {'re_exige_objetivo': ParameterValue(
            LaunchConfiguration('re_exige_objetivo'), value_type=bool)},
    ]
    mapa = LaunchConfiguration('mapa')
    rviz = LaunchConfiguration('rviz')
    placa = LaunchConfiguration('placa')

    # `mapa:=nenhum` liga o perfil sem mapa. O racional inteiro está em
    # `config/nav2_sem_mapa.yaml`; em uma linha: no robô real o mapa da pista
    # SIMULADA é ficção, e desde a decisão 014 o costmap global misturava as
    # paredes fantasma dele com marcação real e permanente do Livox.
    sem_mapa = IfCondition(PythonExpression(["'", mapa, "' == 'nenhum'"]))
    com_mapa = UnlessCondition(PythonExpression(["'", mapa, "' == 'nenhum'"]))

    # `localizacao:=amcl` troca a TF `map → odom` FIXA por localização de
    # verdade contra o mapa. Decisão 021/022.
    #
    # 🔴 EXCLUSÃO MÚTUA, e é o ponto mais fácil de errar aqui: `tf_map_odom` e
    # `amcl` publicam a MESMA transformada. Subir os dois deixa a TF disputada
    # entre um publicador que diz "identidade" e outro que diz a verdade — a
    # pose PISCA entre as duas a cada consulta, e o sintoma (robô que anda em
    # ziguezague no RViz, plano que salta) não aponta para TF nenhuma.
    loc = LaunchConfiguration('localizacao')
    tf_fixa = UnlessCondition(PythonExpression(["'", loc, "' == 'amcl'"]))
    com_mapa_amcl = IfCondition(PythonExpression(
        ["'", mapa, "' != 'nenhum' and '", loc, "' == 'amcl'"]))
    com_mapa_fixa = IfCondition(PythonExpression(
        ["'", mapa, "' != 'nenhum' and '", loc, "' != 'amcl'"]))

    # O perfil da movimentação segue o do simulador quando `sim:=true`: os dois
    # arquivos diferem na zona morta suposta, e rodar o controlador pessimista
    # contra a planta otimista mede uma máquina que não existe (nota de 29-07).
    mov_params = PythonExpression(
        ["'", mov_params_sim, "' if '", sim, "' == 'true' else '",
         mov_params_real, "'"])

    servidores = ['map_server', 'planner_server', 'smoother_server',
                  'controller_server', 'bt_navigator', 'collision_monitor']
    # Sem mapa não há `map_server`, e ele NÃO pode ficar na lista: o
    # `lifecycle_manager` espera cada servidor da lista responder e **aborta o
    # bringup inteiro** se um não vier — foi assim que o `collision_monitor`
    # morreu junto com o Nav2 em 06-08, e o teste D caiu.
    servidores_sem_mapa = [s for s in servidores if s != 'map_server']
    # ...e pelo mesmo motivo, ao contrário: o `amcl` é nó de ciclo de vida e
    # NÃO ativa sozinho. Fora desta lista ele sobe, fica em `unconfigured`, não
    # publica TF nenhuma — e como o `tf_map_odom` também não está lá (exclusão
    # mútua), a árvore fica partida e o Nav2 inteiro não ativa. Silêncio total.
    servidores_com_amcl = ['map_server', 'amcl'] + [
        s for s in servidores if s != 'map_server']

    def nav2_node(pacote, executavel, nome, extras=None, **kwargs):
        """Um servidor do Nav2 em duas versões, com e sem o overlay sem-mapa.

        Os dois são o MESMO nó; muda só a lista de arquivos de parâmetro. Vale
        a repetição porque `parameters` não aceita condição — e a alternativa
        (um segundo `nav2.yaml` completo) é a duplicação que o
        `test_configs_coerentes.py` existe para impedir.
        """
        base = [nav2_params] + list(extras or []) + [{'use_sim_time': sim}]
        overlay = ([nav2_params, sem_mapa_params] + list(extras or [])
                   + [{'use_sim_time': sim}])
        return [
            Node(package=pacote, executable=executavel, name=nome,
                 parameters=base, condition=com_mapa, **kwargs),
            Node(package=pacote, executable=executavel, name=nome,
                 parameters=overlay, condition=sem_mapa, **kwargs),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false',
                              description='true sobe o Gazebo junto'),
        # 🔴 O DEFAULT SEGUE O `sim`, e isto é uma correção de 11-08.
        #
        # A decisão 015 diz que `mapa:=nenhum` é OBRIGATÓRIO no robô real — e
        # deixava isso como algo que o operador tem de digitar. Config que
        # precisa ser digitada é config que vai ser esquecida, e o preço aqui é
        # silencioso: a pilha sobe o mapa da pista SIMULADA (uma sala de
        # 12 × 8 m que não existe), o `global_costmap` põe `StaticLayer` sobre
        # isso e, desde a 014, mistura parede fantasma com marcação real do
        # Livox. O robô recusa caminho livre e ninguém sabe por quê.
        #
        # Quem quiser mapa no robô real (quando houver um mapa REAL) passa
        # `mapa:=<caminho>` explicitamente — que é a ordem certa: o caso
        # perigoso exige intenção, o seguro é o default.
        DeclareLaunchArgument(
            'mapa',
            default_value=PythonExpression(
                ["'", MAPA_PADRAO, "' if '", LaunchConfiguration('sim'),
                 "' == 'true' else 'nenhum'"]),
            description='caminho do .yaml do mapa, ou "nenhum" para o perfil '
                        'SEM MAPA. Default: a pista quando sim:=true, e '
                        '"nenhum" no robô real (decisão 015) — onde o costmap '
                        'global vira janela rolante alimentada só pelo Livox '
                        '(config/nav2_sem_mapa.yaml)'),
        # `mundo` separado de `mapa` de propósito: é fazendo os dois
        # DISCORDAREM que se testa percepção. Mundo com um obstáculo que o
        # mapa não tem = o desvio só pode vir do sensor. Com os dois iguais
        # (o padrão, os dois saem de `gera_pista.py`) o robô poderia estar
        # desviando de memória e ninguém saberia.
        DeclareLaunchArgument('mundo', default_value=MUNDO_PADRAO),

        # ---------------------------------- localização contra mapa (021/022)
        #
        # `fixa` é o default nos DOIS mundos, e por enquanto isso é o certo:
        # nada do AMCL rodou ainda, nem no simulador. Entra opt-in, como o
        # preditor de Smith (06-08) e o estimador do ff (016) entraram.
        #
        # ⚠️ Quando estiver provado, o default deveria SEGUIR O MAPA (a regra da
        # decisão 019: o caso perigoso é que exige intenção). Com mapa e sem
        # AMCL o robô acredita estar na origem do mapa para sempre — hoje isso
        # é seguro só porque o único mapa que sobe por padrão é o da pista
        # simulada, cujo mundo sai da MESMA planta.
        DeclareLaunchArgument(
            'localizacao', default_value='fixa',
            description='"fixa" (map→odom identidade, provisório) ou "amcl" '
                        '(localiza contra o mapa; exige mapa)'),
        # Defaults = o ponto livre da pista simulada, que é onde o robô
        # nascia antes destes argumentos existirem. No robô real com mapa,
        # PASSE os três: o default aqui é da pista, não do prédio.
        DeclareLaunchArgument('pose_x', default_value='2.0'),
        DeclareLaunchArgument('pose_y', default_value='5.0'),
        DeclareLaunchArgument(
            'pose_yaw', default_value='0.0',
            description='pose inicial do AMCL [m, m, rad]. Vem por parâmetro '
                        'porque o NUC não tem tela: "2D Pose Estimate" do '
                        'RViz não existe no robô real'),
        OpaqueFunction(function=_recusa_combinacao_sem_sentido),
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
        # Mesmo raciocínio do `mapa`: no NUC não há tela, e o rviz2 só gasta
        # CPU de um computador que já converte 11 mil pontos por quadro. Quem
        # quiser rviz no robô (por X forwarding) passa `rviz:=true`.
        DeclareLaunchArgument(
            'rviz',
            default_value=PythonExpression(
                ["'true' if '", LaunchConfiguration('sim'),
                 "' == 'true' else 'false'"])),
        # O `sim.launch.py` já tinha `gui:=false` (headless) e esta launch não
        # repassava, então toda corrida de pilha exigia janela. Medir percepção
        # é o que mais pede corrida automatizada: a nuvem sai igual com ou sem
        # render de tela.
        DeclareLaunchArgument('gui', default_value='true',
                              description='false roda o Gazebo headless'),
        DeclareLaunchArgument(
            'placa', default_value='medido',
            description='modelo do atuador simulado: "medido" (a placa de '
                        '31-07, com patamar), "cru" ou "ideal"'),

        # ---- o feedforward do dia (decisão 013, caminho 3, escolhido em 07-08)
        #
        # A curvatura crua deste robô muda 13,5% de um dia para o outro
        # (04-08: −0,8031 · 05-08: −0,9116, faixas que não se tocam) e só 2–3%
        # dentro do mesmo dia. Não existe valor fixo que sirva, e o protocolo
        # passou a ser: três corridas SEM compensador no começo da sessão, e a
        # média entra AQUI. Sem estes argumentos a única via era `ros2 param
        # set`, que o roteiro lista como armadilha (não chega no nó; matar e
        # subir) — ou seja, a medida do dia não tinha para onde ir.
        #
        #   ros2 launch robot_motion pilha.launch.py mapa:=nenhum \
        #       curv_frente:=-0.9116 curv_medido_em:=2026-08-05
        #
        # Os defaults são os do nó, então quem não passa nada não muda nada —
        # e sobe com o log gritando que o ff é herdado.
        # 12-08, no robô: a referência de rumo é que oscila com o Nav2
        # dirigindo. Olhar longe alisa o plano tremido; olhar perto persegue
        # cada rabisco dele. O default é o do nó — quem não passa nada não
        # muda nada.
        DeclareLaunchArgument(
            'lookahead_piso', default_value='0.30',
            description='[m] distância mínima que o seguidor mira à frente. '
                        'Subir alisa a referência de rumo e corta curva '
                        'fechada por dentro'),
        # 🔴 A REALIMENTAÇÃO DO DESVIO LATERAL (039), e ela tem de nascer
        # NEUTRA no robô. `k_lat:=0.0` reproduz exatamente a lei de hoje (só
        # rumo do carrot), e é assim que ela vai para a primeira corrida com o
        # robô LIGADO — mesma regra da 038: mudança grande não vai blind pro
        # robô. No Gazebo o default do nó (1,0) já vale, que é onde ela foi
        # medida.
        # 🔴 DEFAULT 0.0 DESDE A CORRIDA B DE 14-08, e o motivo é medido:
        # com 1,0 o robô fez zigue-zague e PIOROU. Amplitude p90 da referência
        # de rumo foi de 20,0° para 39,8°, o pico de 50° para 88°, e o período
        # encurtou de 1,00 s para 0,70 s.
        #
        # A causa não é só o ganho: o Nav2 republica o plano ~1 Hz e às vezes
        # DESLOCADO de lado. Isso é um DEGRAU em `e_lat`, e o termo o converte
        # direto em degrau de rumo (até o teto de 30°) contra uma placa com
        # ~0,5 s de tempo morto. Já em `k_lat=0` 57% dos degraus grandes de
        # referência caíam logo depois de um plano novo; com 1,0 eles dobraram
        # (7 → 14) e o maior foi de 26° para 62°.
        #
        # ➡️ Ligar de novo exige ANTES limitar a taxa do termo — degrau de
        # plano não pode virar degrau de rumo. Enquanto isso não existir, o
        # default fica em 0,0, que é a lei de antes, exatamente.
        DeclareLaunchArgument(
            'k_lat', default_value='0.0',
            description='[1/s] ganho do desvio lateral do seguidor (039). '
                        'O erro decai com constante de tempo 1/k. ⚠️ 1.0 sem '
                        'limite de taxa foi REPROVADO em 14-08 (zigue-zague). '
                        'O limite existe agora; o ganho só vira default '
                        'depois de uma corrida que o aprove'),
        # 🔴 LOG DE TODA CORRIDA (14-08, pedido do dono): *"quero que toda
        # corrida, tanto Gazebo quanto robô real, salvem log de tudo, assim
        # como o robô 1 faz"*.
        #
        # Vale para os DOIS perfis de propósito. O robô 1 (`Controle_robo_web`)
        # grava `follow_debug.csv` sempre, sem opt-in, e é por isso que lá dá
        # para ler uma corrida depois. Aqui o `path_follower` sabia gravar
        # desde sempre — mas nascia com `csv: ''` e o `grava()` nunca era
        # chamado. Resultado: nenhuma corrida deste projeto foi gravada pelo
        # nó, e em 14-08 a primeira corrida BOA do dia se perdeu porque eu
        # tinha esquecido de subir um `ros2 bag` à mão.
        DeclareLaunchArgument(
            'log_dir', default_value=os.path.join(
                os.path.expanduser('~'), 'logs_robo2'),
            description='diretório dos logs de corrida (CSV do seguidor + bag '
                        'da corrente). Vazio DESLIGA — mas não desligue: '
                        'corrida sem registro não vira número'),
        DeclareLaunchArgument(
            'bag', default_value='true',
            description='grava a corrente inteira em `ros2 bag` junto com o '
                        'CSV. false deixa só o CSV (NUC com disco apertado)'),
        DeclareLaunchArgument(
            'desvio_taxa_deg_s', default_value='15.0',
            description='[°/s] o quanto a correção lateral pode mover a '
                        'referência de rumo por segundo. É o que impede o '
                        'salto do plano no replanejamento de virar tranco'),
        DeclareLaunchArgument(
            're_habilitada', default_value='true',
            description='false DESLIGA a ré de desencalhe (025), o único '
                        'caminho deste robô que dirige por fora do reflexo. '
                        'Este é o knob certo; o `re_max_seguidas` abaixo é '
                        'teto, não interruptor'),
        DeclareLaunchArgument(
            're_max_seguidas', default_value='2',
            description='quantas rés de desencalhe seguidas o seguidor pode '
                        'dar. 0 DESLIGA a ré — o único caminho deste robô que '
                        'dirige por fora do reflexo (025). Pedido do dono em '
                        '13-08, com ele dentro da sala'),
        DeclareLaunchArgument(
            're_exige_objetivo', default_value='true',
            description='true = a ré só acontece com objetivo de navegação '
                        'VIVO (decisão 031). Com o plano retido e o objetivo '
                        'morto, o robô parado recuava sozinho — visto no robô '
                        'em 13-08 e reproduzido no Gazebo em 14-08. Passe '
                        'false só na bancada, onde o seguidor é dirigido por '
                        '/plan cru e não existe ação do Nav2'),
        DeclareLaunchArgument(
            'curv_frente', default_value='-0.817',
            description='curvatura crua indo para a FRENTE [1/m], medida hoje '
                        'sem compensador (medir.py --resumo curvatura)'),
        DeclareLaunchArgument(
            'curv_re', default_value='-0.098',
            description='idem, de ré. Muda menos e raramente se remede'),
        DeclareLaunchArgument(
            'curv_medido_em', default_value='HERDADO',
            description='a data da medida acima. Não entra na conta: entra no '
                        'log, para o número herdado não passar por medido'),
        # No SIMULADOR o default é o valor MEDIDO lá (0,45, 13-08); no robô
        # real é 1,0, que é a identidade — o ganho dele não foi medido, e
        # herdar knob de planta entre máquinas é o erro que o CLAUDE.md proíbe
        # em letras grandes por causa do robô 1.
        DeclareLaunchArgument(
            'ganho_wz',
            default_value=PythonExpression(
                ["'0.45' if '", LaunchConfiguration('sim'),
                 "' == 'true' else '1.0'"]),
            description='fração do wz pedido que a cadeia entrega (038). '
                        '0,45 medido no Gazebo em 13-08 com ganho_de_giro.py; '
                        '1,0 no robô real, onde ninguém mediu ainda'),

        # ---------------------------------------------------- o simulador
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robot_base'),
                'launch', 'sim.launch.py')),
            condition=IfCondition(sim),
            # Nasce num ponto LIVRE: a origem da pista cai dentro da parede do
            # perímetro, que começa em 0.
            #
            # 🔴 O SPAWN E A POSE INICIAL DO AMCL SÃO O MESMO ARGUMENTO, e isso
            # é trava e não conveniência: com dois números separados alguém
            # nasce o robô num lugar e diz ao AMCL que ele está em outro. O
            # filtro então "corrige" uma diferença que não existe no mundo,
            # converge para a pose errada, e o sintoma é mapa e nuvem
            # desalinhados — que se parece exatamente com fatia 2D mal
            # ajustada. Dois defeitos com o mesmo rosto é o que faz perder o
            # dia. Há teste.
            launch_arguments={'mundo': LaunchConfiguration('mundo'),
                              'x': LaunchConfiguration('pose_x'),
                              'y': LaunchConfiguration('pose_y'),
                              'planta': LaunchConfiguration('planta'),
                              'placa': placa,
                              'gui': LaunchConfiguration('gui')}.items(),
        ),

        # ---------------------------------------------------------- Nav2
        #
        # ⚠️ `mapa:=nenhum` NÃO sobe o `map_server` e tira o mapa dos dois
        # costmaps (overlay `nav2_sem_mapa.yaml`). É o perfil do robô real: a
        # localização é LIO (decisão 003) e não existe mapa do lugar onde ele
        # anda. Com mapa, tudo segue como estava — no simulador o mapa é
        # honesto, porque sai da mesma planta do mundo.
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='both',
             parameters=[nav2_params, {'yaml_filename': mapa,
                                       'use_sim_time': sim}],
             condition=com_mapa),
        *nav2_node('nav2_planner', 'planner_server', 'planner_server',
                   output='both'),
        # O suavizador (026). Entra na lista do `lifecycle_manager` e por isso
        # tem de SUBIR: servidor da lista que não responde aborta o bringup
        # inteiro (o defeito de 06-08 que levou o collision_monitor junto).
        *nav2_node('nav2_smoother', 'smoother_server', 'smoother_server',
                   output='both'),
        *nav2_node('nav2_controller', 'controller_server', 'controller_server',
                   output='log',
                   # O comando dele morre aqui. Quem dirige é a nossa cadeia.
                   remappings=[('/cmd_vel', '/nav2_cmd_vel_ignorado')]),
        *nav2_node('nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
                   extras=[{'default_nav_to_pose_bt_xml': bt_xml}],
                   output='both'),
        # ---------------------------------------- localização contra o mapa
        # O AMCL casa o `/scan` (a fatia 2D da nuvem, decisão 021) contra o
        # mapa e publica `map → odom` de verdade. Sobe SÓ com mapa e SÓ quando
        # pedido — e quando ele sobe, o publicador fixo não sobe.
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='both',
             parameters=[nav2_params, amcl_params,
                         {'use_sim_time': sim},
                         # A pose inicial vem por PARÂMETRO porque o NUC não
                         # tem tela: "2D Pose Estimate" do RViz não existe no
                         # robô real. Sem ela o filtro nasce espalhado pelo
                         # mapa inteiro. Ver o YAML.
                         {'initial_pose.x': ParameterValue(
                             LaunchConfiguration('pose_x'), value_type=float),
                          'initial_pose.y': ParameterValue(
                              LaunchConfiguration('pose_y'), value_type=float),
                          'initial_pose.yaw': ParameterValue(
                              LaunchConfiguration('pose_yaw'),
                              value_type=float)}],
             condition=com_mapa_amcl),

        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_pilha', output='both',
             parameters=[{'autostart': True, 'use_sim_time': sim,
                          'node_names': servidores}],
             condition=com_mapa_fixa),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_pilha', output='both',
             parameters=[{'autostart': True, 'use_sim_time': sim,
                          'node_names': servidores_com_amcl}],
             condition=com_mapa_amcl),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_pilha', output='both',
             parameters=[{'autostart': True, 'use_sim_time': sim,
                          'node_names': servidores_sem_mapa}],
             condition=sem_mapa),

        # A TF que falta quando NÃO há localização contra mapa: `map` e `odom`
        # viram o mesmo lugar. Provisório, e documentado no cabeçalho.
        #
        # 🔴 NÃO SOBE COM O AMCL. Os dois publicam `map → odom`; juntos, a pose
        # pisca entre "identidade" e a verdade a cada consulta.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_map_odom', output='log',
             arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
             parameters=[{'use_sim_time': sim}],
             condition=tf_fixa),

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
             parameters=[{'use_sim_time': sim, 'segura_rumo': False}]
                        + curv + ganho,
             remappings=[('/hoverboard_base_controller/cmd_vel',
                          '/cmd_vel_bruto')],
             condition=IfCondition(sim)),
        Node(package='robot_motion', executable='compensador_rumo',
             name='compensador_rumo', output='both',
             parameters=[{'use_sim_time': sim, 'segura_rumo': False}]
                        + curv + ganho,
             condition=UnlessCondition(sim)),
        Node(package='robot_motion', executable='path_follower',
             name='path_follower', output='both',
             parameters=[*olhar, {'use_sim_time': sim}],
             remappings=[('/path_follower/rumo_alvo',
                          '/heading_controller/rumo_alvo'),
                         ('/path_follower/velocidade_alvo',
                          '/heading_controller/velocidade_alvo')]),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
             arguments=['-d', rviz_config],
             parameters=[{'use_sim_time': sim}],
             condition=IfCondition(rviz)),

        # 🔴 GRAVADOR DA CORRENTE, LIGADO POR PADRÃO (14-08).
        #
        # O CSV do seguidor conta o que o SEGUIDOR pensou; este bag conta o que
        # a cadeia inteira fez — plano, pedido do seguidor, o que o reflexo
        # deixou passar, o que o compensador entregou, pose e TF. Foi essa
        # combinação que permitiu ler as oito corridas de 14-08; a diferença é
        # que agora ela não depende de eu lembrar de subir à mão.
        #
        # ⚠️ SEM a nuvem (`/livox/pontos`) e sem os costmaps de propósito: são
        # os dois que fariam o bag inviável no NUC. O que está aqui é leve —
        # ~10 MB por 3 minutos de corrida.
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o',
                 [LaunchConfiguration('log_dir'), f'/corrida_{CARIMBO}'],
                 '/plan', '/plan_smoothed', '/received_global_plan',
                 '/auto_vel_raw', '/auto_vel', '/unstuck_vel', '/key_vel',
                 '/compensador_rumo/cmd_vel', '/cmd_vel_bruto',
                 '/hoverboard_base_controller/cmd_vel',
                 '/Odometry', '/amcl_pose', '/odom',
                 '/collision_monitor_state', '/polygon_stop',
                 '/heading_controller/rumo_alvo',
                 '/heading_controller/velocidade_alvo',
                 '/behavior_tree_log', '/rosout', '/tf', '/tf_static'],
            output='log',
            condition=IfCondition(LaunchConfiguration('bag'))),

        LogInfo(msg='PILHA subindo — espere os servidores ativarem '
                    '(o Smac leva ~16 s montando a heurística) e então clique '
                    '"2D Goal Pose" no RViz.'),
    ])
