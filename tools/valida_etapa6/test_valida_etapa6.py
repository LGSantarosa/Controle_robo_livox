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


# ─── a finalização dirigida do gravador (conserto de 24-09) ──────────────────
#
# Na segunda corrida o `ros2 bag record` sobreviveu a 20 s de SIGINT do grupo e
# levou KILL — e morto por KILL ele nunca escreve o `metadata.yaml`. Resultado:
# 229 MB de bag assinado que o `ros2 bag info` recusa. Evidência que não abre é
# pior que evidência que falta, porque parece estar lá.

def _indice(codigo, trecho):
    achados = [i for i, l in enumerate(codigo) if trecho in l]
    assert achados, f'não achei {trecho!r} no código do wrapper'
    return achados


def test_o_gravador_recebe_sigterm_antes_de_o_grupo_cair():
    """A ordem é o conserto: SIGTERM dirigido → espera → só então `limpa`.

    🔧 O sinal era SIGINT até 24-09. Quatro experimentos sem Gazebo mostraram que
    o `ros2 bag record` deste Jazzy NÃO responde a SIGINT (nem no pid, nem no
    grupo, nem com 30 s); com SIGTERM ele sai em 0,42 s e escreve o
    `metadata.yaml`. Não era janela curta, era sinal errado.
    """
    codigo = _codigo_do_wrapper()
    sigint = min(_indice(codigo, 'sinaliza_um'))
    espera = min(_indice(codigo, "processos.py\" vivo"))
    queda = min(i for i, l in enumerate(codigo) if l.strip() == 'limpa')
    assert sigint < espera < queda, (sigint, espera, queda)


def test_a_espera_do_gravador_e_de_30_segundos():
    """300 voltas de 0,1 s. O número importa: a janela do grupo é de 20 s, e foi
    ela que não bastou."""
    texto = open(WRAPPER).read()
    assert 'seq 1 300' in texto and 'sleep 0.1' in texto


def test_o_sinal_dirigido_ao_gravador_e_term_e_nao_int():
    """E o INT não pode voltar por descuido: ele não para o gravador, então um
    validador que o use volta a assinar bag que não abre."""
    codigo = _codigo_do_wrapper()
    dirigidos = [l for l in codigo if 'sinaliza_um' in l]
    assert dirigidos, 'não há encerramento dirigido'
    for l in dirigidos:
        assert ' TERM' in l, l
        assert ' INT' not in l, l


def test_o_gravador_que_nao_fecha_reprova():
    """Timeout não pode virar aviso: se ele não saiu sozinho, o item reprova."""
    texto = open(WRAPPER).read()
    bloco = texto.split('finalização DIRIGIDA', 1)[1].split('derrubar e provar', 1)[0]
    assert 'REPROVADO' in bloco
    assert 'não saiu em 30 s' in bloco


def test_o_reindex_marca_recuperado_e_nao_aprovado():
    """🔴 A regra que o dono fixou: reindex preserva a evidência, mas NÃO
    transforma encerramento defeituoso em corrida aprovada."""
    texto = open(WRAPPER).read()
    assert 'ros2 bag reindex' in texto
    bloco = texto.split('FECHAR SOZINHO', 1)[1]
    assert 'RECUPERADO' in bloco
    # E o RECUPERADO tem de chegar ao código de saída.
    assert 'REPROVADO|RECUPERADO' in texto


def test_o_bag_info_e_exigido_com_codigo_zero():
    assert 'ros2 bag info' in open(WRAPPER).read()


def test_a_politica_de_grupo_das_etapas_4_e_5_nao_muda():
    """🔴 A prova de que o conserto é LOCALIZADO.

    As etapas 4 e 5 são validadores fechados, e corridas já aprovadas dependem
    da política de sinal delas. A finalização dirigida entrou como primitiva
    NOVA na cópia da etapa 6; as funções compartilhadas têm de continuar com o
    mesmo texto, linha por linha.
    """
    import re
    p6 = open(os.path.join(AQUI, 'processos.py')).read()
    p5 = open(os.path.join(RAIZ, 'tools/valida_etapa5/processos.py')).read()

    def corpo(texto, nome):
        """Só o bloco `def`, e nada do que vem depois dele.

        ⚠️ Antes isto ia até o próximo `def`, e engolia o que estivesse no meio —
        o `SINAIS`, por exemplo. O teste então acusava diferença onde a função
        era idêntica, e acusaria IGUALDADE se alguém mexesse na função e
        compensasse fora dela. Corta na primeira linha de topo depois do corpo.
        """
        linhas = texto.splitlines()
        inicio = next(i for i, l in enumerate(linhas)
                      if l.startswith(f'def {nome}('))
        fim = inicio + 1
        while fim < len(linhas) and (not linhas[fim].strip()
                                     or linhas[fim][:1] in (' ', '\t')):
            fim += 1
        return '\n'.join(linhas[inicio:fim]).rstrip()

    for compartilhada in ('sinaliza_grupos', 'sinaliza_marcados',
                          '_mata_se_ainda_for', 'classifica', 'compara'):
        assert corpo(p6, compartilhada) == corpo(p5, compartilhada), compartilhada


def test_a_politica_de_grupo_nao_ganhou_o_sinal_novo():
    """🔴 O SIGTERM é do encerramento DIRIGIDO, e só dele.

    A derrubada do grupo continua INT e depois KILL, igual às etapas 4 e 5: é
    ela que protege corridas já fechadas, e trocar o sinal dela mudaria o
    significado daquelas provas. Quem chama `sinaliza_grupos` é a `lib.sh`.
    """
    lib = open(os.path.join(AQUI, 'lib.sh')).read()
    chamadas = [l for l in lib.splitlines() if 'sinaliza_grupos' in l
                and not l.lstrip().startswith('#')]
    assert chamadas
    for l in chamadas:
        assert 'TERM' not in l, l
    assert 'sinaliza_grupos "$GRUPOS" INT' in lib
    assert 'sinaliza_grupos "$GRUPOS" KILL' in lib


def _roda_processos(args, marca):
    import subprocess
    env = {**os.environ, 'VALIDA_ETAPA4_MARCA': marca}
    return subprocess.run(
        ['python3', os.path.join(AQUI, 'processos.py'), *args],
        capture_output=True, text=True, env=env, timeout=30)


def test_sinaliza_um_recusa_processo_sem_a_marca(tmp_path):
    """🔴 A trava que impede isto de ser um `pkill` disfarçado.

    O alvo é escolhido por argv, e argv qualquer um pode ter. Quem autoriza o
    sinal é a MARCA desta rodada mais o starttime: sem as duas, o validador
    poderia matar um processo do dono que por acaso casa com o padrão.
    """
    import subprocess
    vitima = subprocess.Popen(['sleep', '30'])
    try:
        start = open(f'/proc/{vitima.pid}/stat').read()
        start = start[start.rindex(')') + 2:].split()[19]
        r = _roda_processos(['sinaliza_um', str(vitima.pid), start, 'KILL'],
                            str(tmp_path))
        assert r.returncode != 0, r.stdout
        assert 'sem marca' in r.stdout, r.stdout
        assert vitima.poll() is None, 'o processo sem marca foi morto'
    finally:
        vitima.kill()
        vitima.wait()


def test_sinaliza_um_recusa_starttime_diferente(tmp_path):
    """PID é reusado pelo sistema; (pid, starttime) é que é identidade."""
    import subprocess
    vitima = subprocess.Popen(['sleep', '30'])
    try:
        r = _roda_processos(['sinaliza_um', str(vitima.pid), '1', 'KILL'],
                            str(tmp_path))
        assert r.returncode != 0, r.stdout
        assert 'outro processo' in r.stdout, r.stdout
        assert vitima.poll() is None
    finally:
        vitima.kill()
        vitima.wait()


def test_vivo_diz_nao_para_pid_que_nao_existe(tmp_path):
    r = _roda_processos(['vivo', '999999', '1'], str(tmp_path))
    assert r.returncode == 1


def test_acha_nao_lista_processo_sem_a_marca(tmp_path):
    """O `acha` só enxerga o que é desta rodada — é assim que o SIGINT dirigido
    não alcança um gravador de outra sessão do dono."""
    import subprocess
    outro = subprocess.Popen(['sleep', '30'])
    try:
        r = _roda_processos(['acha', 'sleep 30'], str(tmp_path))
        assert r.returncode == 0
        assert str(outro.pid) not in r.stdout, r.stdout
    finally:
        outro.kill()
        outro.wait()


def test_as_etapas_4_e_5_nao_ganharam_as_primitivas_novas():
    """E a recíproca: as primitivas da etapa 6 NÃO foram para os validadores
    fechados. Cópia é cópia; mexer neles é mexer em evidência congelada."""
    for etapa in ('valida_etapa4', 'valida_etapa5'):
        texto = open(os.path.join(RAIZ, 'tools', etapa, 'processos.py')).read()
        assert 'sinaliza_um' not in texto, etapa
        assert 'acha_marcado' not in texto, etapa


# ─── o manifesto (conserto de 24-09, depois da segunda corrida) ──────────────
#
# Nas duas primeiras corridas o SHA256SUMS era gerado ANTES das últimas linhas do
# resultado, e o `sha256sum -c` da própria pasta falhava em três arquivos. Nada
# estava corrompido — os YAMLs e o bag batiam byte a byte —, mas manifesto que
# não fecha contra a própria pasta faz duvidar também do que está certo.

def _codigo_do_wrapper():
    """As linhas de CÓDIGO, sem comentário: o que o shell executa."""
    return [l for l in open(WRAPPER).read().splitlines()
            if not l.lstrip().startswith('#')]


def _indice_da_geracao(codigo):
    alvos = [i for i, l in enumerate(codigo) if 'sha256sum > SHA256SUMS' in l]
    assert len(alvos) == 1, f'esperava UMA geração do manifesto, achei {alvos}'
    return alvos[0]


def test_nenhuma_chamada_a_anota_depois_da_geracao_do_manifesto():
    """🔴 A trava central do conserto: `anota` escreve no resultado.txt e no
    resultado.csv, e os dois são ASSINADOS. Uma chamada depois da geração
    invalidaria o manifesto de novo — o defeito exato das duas primeiras
    corridas."""
    codigo = _codigo_do_wrapper()
    depois = codigo[_indice_da_geracao(codigo) + 1:]
    assert not [l for l in depois if 'anota ' in l], \
        [l for l in depois if 'anota ' in l]


def test_nada_escreve_no_resultado_depois_da_geracao():
    """Nem por `anota`, nem por redirecionamento direto."""
    codigo = _codigo_do_wrapper()
    depois = codigo[_indice_da_geracao(codigo) + 1:]
    for l in depois:
        assert '>> "$RESULTADO' not in l and '> "$RESULTADO' not in l, l


def test_o_console_e_o_proprio_manifesto_ficam_fora_do_manifesto():
    """O `console.txt` é o log AO VIVO: ele não pode assinar a si mesmo enquanto
    ainda está sendo escrito. É o ÚNICO arquivo de fora, e o conteúdo dele está
    reproduzido no `resultado.txt`, que fica assinado."""
    texto = open(WRAPPER).read()
    assert '! -name console.txt' in texto
    assert '! -name SHA256SUMS' in texto


def test_o_escopo_assinado_inclui_a_si_mesmo():
    """A declaração do que foi assinado não pode ficar fora do manifesto: fora
    dele, é declaração que se troca depois."""
    texto = open(WRAPPER).read()
    assert 'echo "./$(basename "$ESCOPO")" >> "$ESCOPO"' in texto


def test_o_validador_confere_o_proprio_manifesto_e_pode_reprovar():
    """`sha256sum -c` dentro do próprio validador, e a falha FORÇA RC 1.

    Sem isso, "tem SHA256SUMS" passaria por "o SHA256SUMS fecha" — que é
    exatamente a diferença entre a segunda corrida e a terceira.
    """
    codigo = _codigo_do_wrapper()
    checagem = [i for i, l in enumerate(codigo) if 'sha256sum -c SHA256SUMS' in l]
    assert checagem, 'o validador não confere o próprio manifesto'
    # E depois da geração, não antes: conferir um manifesto que ainda vai mudar
    # não prova nada.
    assert min(checagem) > _indice_da_geracao(codigo)
    # A falha tem de chegar ao código de saída.
    saida = [l for l in codigo if 'MANIFESTO' in l and 'exit 1' not in l]
    assert any('MANIFESTO=1' in l for l in saida), saida
    assert any('"$MANIFESTO" -ne 0' in l for l in codigo), \
        'a falha do manifesto não entra na decisão do código de saída'


def test_a_conferencia_previa_e_do_escopo_nao_do_manifesto_feito():
    """Regra 3: confere-se o que VAI ser assinado, antes de assinar — e o que
    importa são os dois YAMLs materializados e o bag."""
    codigo = _codigo_do_wrapper()
    geracao = _indice_da_geracao(codigo)
    # A própria linha de geração LÊ o escopo (é a lista do que assinar), então
    # ela não conta: o que se exige é que o escopo tenha sido escrito e
    # conferido antes dela.
    escopo = [i for i, l in enumerate(codigo) if '"$ESCOPO"' in l and i != geracao]
    assert escopo and max(escopo) < geracao, (escopo, geracao)
    # E a lista assinada é a que o `find` montou, não um `find` refeito na hora
    # de assinar — refazer abriria espaço para assinar coisa que não foi
    # conferida.
    assert 'xargs -a' in codigo[geracao], codigo[geracao]
    texto = open(WRAPPER).read()
    for exigido in ('perfil_nav2.yaml', 'perfil_collision_monitor.yaml', 'bag'):
        assert exigido in texto, exigido
