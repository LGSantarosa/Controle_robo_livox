"""O validador da etapa 6 julgado sem subir Gazebo — a lógica pura.

Uma sessão de Gazebo custa caro e não se repete de graça: tudo o que decide
APROVADO/REPROVADO tem de estar testável aqui, contra dumps de mentira. O que
sobra para a corrida é observar, não decidir.

Cobre:

  · o `esperados_pilha_robo3.yaml` passando pelo schema do próprio
    `captura.py` — formato errado ali só apareceria com o Gazebo de pé;
  · cada item do `confere.py`, com o caso que passa E o caso que reprova;
  · o `avalia` do `topicos.py`, inclusive o publicador sem mensagem;
  · o wrapper: sem `pkill`, headless, e sem mandar objetivo (isso é o passo 7).
"""
import importlib.util
import os

import pytest
import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.abspath(os.path.join(AQUI, '..', '..'))
ESPERADOS = os.path.join(AQUI, 'esperados_pilha_robo3.yaml')
WRAPPER = os.path.join(RAIZ, 'bin', 'valida-etapa6')


def _modulo(nome, caminho):
    spec = importlib.util.spec_from_file_location(nome, caminho)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope='module')
def confere():
    return _modulo('confere_etapa6', os.path.join(AQUI, 'confere.py'))


# ─── a lista versionada de nós ───────────────────────────────────────────────

def test_a_lista_de_nos_passa_pelo_schema_da_captura():
    """O formato do `volateis` (padrao/quantidade/alias) é fácil de errar, e o
    erro só apareceria com o Gazebo de pé — caro e tarde."""
    captura = _modulo('captura_etapa6',
                      os.path.join(RAIZ, 'tools/linha_de_base/captura.py'))
    cfg = captura.le_configuracao(ESPERADOS)
    assert cfg['nos'], 'lista vazia aprovaria qualquer grafo'


def test_a_lista_nao_tem_amcl_nem_rviz():
    """O AMCL e o `tf_map_odom` publicam a MESMA TF; e a corrida é headless."""
    nos = yaml.safe_load(open(ESPERADOS))['nos']
    assert '/amcl' not in nos
    assert '/rviz2' not in nos
    # E o que a etapa 6 existe para provar TEM de estar lá.
    for obrigatorio in ('/twist_mux', '/placa_simulada', '/path_follower',
                        '/global_costmap/global_costmap',
                        '/local_costmap/local_costmap'):
        assert obrigatorio in nos, obrigatorio


def test_a_contagem_de_listeners_e_tres_sem_rviz():
    """🔧 Corrigido de 4 para 3 depois da primeira corrida: 1 do `scan_2d` + 2 do
    Nav2. O quarto da baseline do robô 2 era do `rviz2`, e aqui `rviz:=false`."""
    vol = yaml.safe_load(open(ESPERADOS))['grafo_somente']['volateis']
    listener = [v for v in vol if 'transform_listener' in v['padrao']]
    assert len(listener) == 1
    assert listener[0]['quantidade'] == 3


def test_a_lista_nao_tem_no_da_fronteira_de_hardware():
    """No Gazebo a cadeia termina no `hoverboard_base_controller` (achado D)."""
    nos = yaml.safe_load(open(ESPERADOS))['nos']
    assert not [n for n in nos
                if n.rsplit('/', 1)[-1] in ('cmd_vel_to_wheels', 'mega_bridge')]


# ─── §4.1 o mux vivo ─────────────────────────────────────────────────────────

def _mux(faixas, stamped=True):
    p = {'use_stamped': stamped}
    for i, (topico, prio) in enumerate(faixas.items()):
        p[f'topics.f{i}.topic'] = topico
        p[f'topics.f{i}.priority'] = prio
    return {'/twist_mux': p}


def test_mux_com_as_quatro_faixas_aprova(confere):
    ok, det = confere.item_mux(_mux(confere.FAIXAS))
    assert ok, det


def test_mux_sem_o_desencalhe_reprova(confere):
    """Faixa que falta não dá erro, dá silêncio: o desencalhe publicaria para o
    vazio com o perfil do robô 3 configurando a ré."""
    faixas = {k: v for k, v in confere.FAIXAS.items() if k != 'unstuck_vel'}
    ok, det = confere.item_mux(_mux(faixas))
    assert not ok
    assert det['faltando'] == ['unstuck_vel']


def test_mux_com_faixa_do_robo2_reprova(confere):
    ok, det = confere.item_mux(_mux({**confere.FAIXAS, 'key_vel': 90}))
    assert not ok
    assert det['sobrando'] == ['key_vel']


def test_mux_com_use_stamped_falso_reprova(confere):
    ok, det = confere.item_mux(_mux(confere.FAIXAS, stamped=False))
    assert not ok and det['use_stamped'] is False


def test_mux_com_use_stamped_inteiro_reprova(confere):
    """Em Python `1 == True`; o parâmetro do nó é booleano, e 1 não é ele."""
    ok, _ = confere.item_mux(_mux(confere.FAIXAS, stamped=1))
    assert not ok


# ─── §4.3 use_sim_time e a lista que não envelhece ───────────────────────────

def test_use_sim_time_todos_true_aprova(confere):
    params = {'/a': {'use_sim_time': True}, '/b': {'use_sim_time': True}}
    ok, _ = confere.item_use_sim_time(params, ['/a', '/b'])
    assert ok


def test_um_no_com_use_sim_time_falso_reprova(confere):
    params = {'/a': {'use_sim_time': True}, '/b': {'use_sim_time': False}}
    ok, det = confere.item_use_sim_time(params, ['/a', '/b'])
    assert not ok and det['use_sim_time_nao_true'] == {'/b': False}


def test_no_fora_da_lista_versionada_reprova(confere):
    """🔴 O ponto do §4.3: sem isto a prova envelhece em silêncio a cada nó
    novo, e um nó no relógio de parede passaria sem ninguém olhar."""
    params = {'/a': {'use_sim_time': True}, '/novo': {'use_sim_time': True}}
    ok, det = confere.item_use_sim_time(params, ['/a'])
    assert not ok and det['fora_da_lista_versionada'] == ['/novo']


def test_no_sem_o_parametro_reprova(confere):
    ok, det = confere.item_use_sim_time({'/a': {}}, ['/a'])
    assert not ok and det['sem_use_sim_time'] == ['/a']


# ─── §4.3 a exceção nominal (e por que ela não é tapete) ─────────────────────

PERMITIDO = [{'no': '/gz_ros_control', 'motivo': 'plugin dentro do Gazebo',
              'origem': 'corrida 20260924_104529'}]


def test_excecao_nominal_usada_aprova(confere):
    params = {'/a': {'use_sim_time': True},
              '/gz_ros_control': {'use_sim_time': False}}
    ok, det = confere.item_use_sim_time(params, ['/a', '/gz_ros_control'],
                                        PERMITIDO)
    assert ok, det
    assert det['excecoes_usadas'] == ['/gz_ros_control']


def test_excecao_nominal_sem_uso_reprova(confere):
    """🔴 A regra que impede a exceção de virar tapete: se o nó passou a estar em
    tempo simulado (ou saiu do grafo), a permissão tem de sair junto — senão ela
    fica autorizando o próximo caso em silêncio."""
    params = {'/a': {'use_sim_time': True},
              '/gz_ros_control': {'use_sim_time': True}}
    ok, det = confere.item_use_sim_time(params, ['/a', '/gz_ros_control'],
                                        PERMITIDO)
    assert not ok
    assert det['excecoes_sem_uso'] == ['/gz_ros_control']


def test_excecao_de_um_no_nao_libera_outro(confere):
    """O `/rosbag2_recorder` é o caso real: ele foi CONSERTADO
    (`--use-sim-time` no bag do robô 3), não dispensado."""
    params = {'/gz_ros_control': {'use_sim_time': False},
              '/rosbag2_recorder': {'use_sim_time': False}}
    ok, det = confere.item_use_sim_time(
        params, ['/gz_ros_control', '/rosbag2_recorder'], PERMITIDO)
    assert not ok
    assert det['use_sim_time_nao_true'] == {'/rosbag2_recorder': False}


def test_a_excecao_do_arquivo_versionado_e_so_o_gz_ros_control():
    """Quem lê o arquivo tem de achar UMA exceção, com motivo e origem — e o
    `/rosbag2_recorder` NÃO pode estar lá: ele foi consertado."""
    cfg = yaml.safe_load(open(ESPERADOS))
    excecoes = cfg['use_sim_time_falso_permitido']
    assert [e['no'] for e in excecoes] == ['/gz_ros_control']
    for e in excecoes:
        assert e['motivo'].strip() and e['origem'].strip()
        # Redação: ele é dirigido pelo passo de atualização do Gazebo. Chamá-lo
        # de publicador do /clock seria descrever outro mecanismo.
        assert '/clock' not in e['motivo'] or 'não' in e['motivo'].lower()


# ─── §4.4 footprint vivo contra o perfil ─────────────────────────────────────

@pytest.fixture(scope='module')
def perfil3(confere):
    """O perfil do robô 3 montado do `share/` de verdade — a mesma função que a
    pilha usa. Se o `robot_base` não estiver instalado, não há o que comparar."""
    ros2 = pytest.importorskip('ament_index_python.packages')
    try:
        base = ros2.get_package_share_directory('robot_base')
        motion = ros2.get_package_share_directory('robot_motion')
    except Exception:  # noqa: BLE001
        pytest.skip('robot_base/robot_motion não instalados')
    return confere.perfil_do_robo3(motion, base)[1]


def _params_dos_costmaps(perfil3):
    params = {}
    for caminho, valor in perfil3['nav2_rewrites'].items():
        params.setdefault(f'/{caminho[0]}/{caminho[1]}', {})[caminho[-1]] = valor
    return params


def test_footprint_vivo_igual_ao_perfil_aprova(confere, perfil3):
    ok, det = confere.item_footprint(_params_dos_costmaps(perfil3), perfil3)
    assert ok, det
    assert det['conferidos'] == 4


def test_footprint_do_robo2_reprova(confere, perfil3):
    """CONTROLE POSITIVO do §4.4: o valor que está hoje no `nav2.yaml` é o do
    robô 2. Ler ISSO com `robo:=3` significa que a reescrita não pegou."""
    params = _params_dos_costmaps(perfil3)
    for no in params:
        params[no]['footprint'] = '[[0.35, 0.2775], [0.35, -0.2775]]'
    ok, det = confere.item_footprint(params, perfil3)
    assert not ok and len(det['diferencas']) == 2


def test_costmap_sem_dump_reprova(confere, perfil3):
    params = _params_dos_costmaps(perfil3)
    params.pop('/local_costmap/local_costmap')
    ok, det = confere.item_footprint(params, perfil3)
    assert not ok and det['costmaps_sem_dump'] == ['/local_costmap/local_costmap']


def test_padding_com_diferenca_de_float_ainda_aprova(confere, perfil3):
    """O double do ROS volta com o ruído do float de 32 bits; o teste não pode
    reprovar por isso, nem aprovar diferença de verdade."""
    params = _params_dos_costmaps(perfil3)
    for no in params:
        params[no]['footprint_padding'] += 1e-10
    ok, _ = confere.item_footprint(params, perfil3)
    assert ok
    for no in params:
        params[no]['footprint_padding'] += 0.001
    ok, _ = confere.item_footprint(params, perfil3)
    assert not ok


# ─── D2 os materializados ────────────────────────────────────────────────────

def test_materializados_aprova_quando_os_dois_nascem(confere, perfil3, tmp_path):
    perfil_mod = _modulo('perfil_m', os.path.join(
        RAIZ, 'ros2_packages/robot_motion/robot_motion/perfil.py'))
    corrida = tmp_path / 'corrida_x'
    perfil_mod.materializa(perfil3, str(corrida))
    (corrida / 'bag').mkdir()
    ok, det = confere.item_materializados(perfil_mod, perfil3, str(corrida))
    assert ok, det


def test_materializados_reprova_sem_o_bag_em_subpasta(confere, perfil3, tmp_path):
    perfil_mod = _modulo('perfil_m2', os.path.join(
        RAIZ, 'ros2_packages/robot_motion/robot_motion/perfil.py'))
    corrida = tmp_path / 'corrida_y'
    perfil_mod.materializa(perfil3, str(corrida))
    ok, det = confere.item_materializados(perfil_mod, perfil3, str(corrida))
    assert not ok and det['bag']['existe'] is False


def test_materializados_reprova_com_yaml_adulterado(confere, perfil3, tmp_path):
    perfil_mod = _modulo('perfil_m3', os.path.join(
        RAIZ, 'ros2_packages/robot_motion/robot_motion/perfil.py'))
    corrida = tmp_path / 'corrida_z'
    escritos = perfil_mod.materializa(perfil3, str(corrida))
    (corrida / 'bag').mkdir()
    alvo = escritos['nav2']
    dados = yaml.safe_load(open(alvo))
    dados['global_costmap']['global_costmap']['ros__parameters']['footprint'] = '[[9, 9]]'
    with open(alvo, 'w') as f:
        yaml.safe_dump(dados, f)
    ok, det = confere.item_materializados(perfil_mod, perfil3, str(corrida))
    assert not ok
    assert det['arquivos']['nav2']['igual_a_aplica_reescritas'] is False


# ─── §4.2 a fronteira de hardware, pela negativa ─────────────────────────────

def test_fronteira_de_hardware_ausente_aprova(confere):
    ok, _ = confere.item_fronteira_de_hardware(
        {'/placa_simulada', '/hoverboard_base_controller', '/twist_mux'})
    assert ok


def test_cmd_vel_to_wheels_no_grafo_reprova(confere):
    ok, det = confere.item_fronteira_de_hardware(
        {'/placa_simulada', '/hoverboard_base_controller', '/cmd_vel_to_wheels'})
    assert not ok and det['proibidos_no_grafo'] == ['/cmd_vel_to_wheels']


def test_placa_simulada_ausente_reprova(confere):
    """Sem ela a cadeia não termina em lugar nenhum: o comando do compensador
    sai em `/cmd_vel_bruto` e ninguém escuta."""
    ok, det = confere.item_fronteira_de_hardware({'/hoverboard_base_controller'})
    assert not ok and det['exigidos_faltando'] == ['/placa_simulada']


# ─── as provas vivas (a parte pura do topicos.py) ────────────────────────────

@pytest.fixture(scope='module')
def topicos():
    pytest.importorskip('rclpy')
    pytest.importorskip('tf2_ros')
    return _modulo('topicos_etapa6', os.path.join(AQUI, 'topicos.py'))


def _observacao_boa():
    return (dict.fromkeys(('/Odometry', '/scan'), 7),
            {'map->base_link': True, 'base_link->livox_frame': True},
            {'/Odometry': ['/ros_gz_bridge'], '/scan': ['/scan_2d'],
             '/tf': ['/robot_state_publisher'], '/tf_static': ['/tf_map_odom']},
            ['/tf_map_odom', '/twist_mux'],
            {'/scan': ['sensor_msgs/msg/LaserScan']})


def test_observacao_completa_aprova(topicos):
    itens = topicos.avalia(*_observacao_boa())
    assert all(ok for ok, _ in itens.values()), itens


def test_publicador_sem_mensagem_reprova(topicos):
    """O caso enganoso: o nó subiu, o tópico existe, e nada trafega."""
    contagem, tfs, pubs, nos, tipos = _observacao_boa()
    contagem['/scan'] = 0
    itens = topicos.avalia(contagem, tfs, pubs, nos, tipos)
    assert not itens['4.5 /scan: publicador e mensagem'][0]


def test_mensagem_sem_publicador_declarado_reprova(topicos):
    contagem, tfs, pubs, nos, tipos = _observacao_boa()
    pubs['/Odometry'] = []
    itens = topicos.avalia(contagem, tfs, pubs, nos, tipos)
    assert not itens['4.5 /Odometry: publicador e mensagem'][0]


def test_tf_que_nao_resolve_reprova(topicos):
    contagem, tfs, pubs, nos, tipos = _observacao_boa()
    tfs.pop('base_link->livox_frame')
    itens = topicos.avalia(contagem, tfs, pubs, nos, tipos)
    assert not itens['4.5 TF base_link->livox_frame'][0]


def test_amcl_e_tf_map_odom_juntos_reprovam(topicos):
    """Os dois publicam `map → odom`: a pose pisca entre identidade e verdade."""
    contagem, tfs, pubs, nos, tipos = _observacao_boa()
    itens = topicos.avalia(contagem, tfs, pubs, nos + ['/amcl'], tipos)
    assert not itens['4.5 publicador único de map->odom'][0]


def test_nenhum_publicador_de_map_odom_reprova(topicos):
    contagem, tfs, pubs, _nos, tipos = _observacao_boa()
    itens = topicos.avalia(contagem, tfs, pubs, ['/twist_mux'], tipos)
    assert not itens['4.5 publicador único de map->odom'][0]


def test_topico_wheelspeeds_reprova(topicos):
    contagem, tfs, pubs, nos, tipos = _observacao_boa()
    tipos['/wheel_speeds'] = ['robot_interfaces/msg/WheelSpeeds']
    itens = topicos.avalia(contagem, tfs, pubs, nos, tipos)
    assert not itens['4.2 nenhum tópico WheelSpeeds no grafo'][0]


# ─── o wrapper ───────────────────────────────────────────────────────────────

def test_o_wrapper_nunca_usa_pkill():
    """`pkill -f` já matou uma sessão ssh neste projeto.

    A trava olha só linha de CÓDIGO: o wrapper explica no cabeçalho por que não
    usa `pkill`, e proibir a palavra proibiria a explicação. Comentário não mata
    processo.
    """
    codigo = [l for l in open(WRAPPER).read().splitlines()
              if not l.lstrip().startswith('#')]
    assert not [l for l in codigo if 'pkill' in l], \
        [l for l in codigo if 'pkill' in l]


def test_o_wrapper_roda_headless_e_sem_rviz():
    texto = open(WRAPPER).read()
    assert 'gui:=false' in texto and 'rviz:=false' in texto


def test_o_wrapper_passa_robo3_no_argv():
    """A seleção do perfil é por argv (decisão 056); só pelo contexto morre."""
    assert 'robo:=3' in open(WRAPPER).read()


def test_o_wrapper_nao_manda_objetivo():
    """🔴 O objetivo curto é o passo 7. Mandar goal aqui misturaria as duas
    provas, e a primeira corrida deixaria de ser sobre subir."""
    texto = open(WRAPPER).read()
    for proibido in ('goal_pose', 'NavigateToPose', 'navigate_to_pose'):
        assert proibido not in texto, proibido


def test_o_wrapper_reconstroi_antes_de_rodar():
    """Arquivo NOVO não tem symlink no install/ até o build — foi assim que o
    mux do robô 3 ficou invisível no passo 5."""
    texto = open(WRAPPER).read()
    assert 'colcon build' in texto
    assert texto.index('colcon build') < texto.index('ros2 launch')


def test_o_wrapper_nao_apaga_a_pasta_de_evidencia():
    """A pasta é a prova, sobretudo quando reprova."""
    texto = open(WRAPPER).read()
    assert 'rm -rf' not in texto
