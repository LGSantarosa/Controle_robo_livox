#!/usr/bin/env python3
"""O montador da evidência do passo 7 (decisão 060) — sem ROS, sem bag, sem Gazebo.

🔴 ESTES TESTES NASCEM VERMELHOS, DE PROPÓSITO: `tools/valida_etapa7/monta.py`
ainda NÃO existe. A decisão 060 foi escrita antes; estes testes definem a
interface pública que o montador vai ter de cumprir:

    monta(brutos) -> (evidencia, falhas)

`brutos` é o que o `bin/valida-etapa7` junta da corrida, já sem ROS:

  · `objetivo_log`       — texto do `ros2 action send_goal` (identifica o UUID);
  · `status`             — linhas do `/navigate_to_pose/_action/status` já
                           extraídas do bag: `t_ns` (instante de GRAVAÇÃO) e
                           `goals`, cada um com `uuid` (hex), `stamp_ns` e
                           `status` (código do `action_msgs/GoalStatus`);
  · `amostras`           — do bag: `topico`, `t_ns` (gravação), `header_ns`
                           (só diagnóstico), `v`, `wz`;
  · `dump_placa`         — texto do `ros2 param dump /placa_simulada`;
  · `dump_controller`    — texto do `ros2 param dump /controller_server`;
  · `nav2_materializado` — texto do `perfil_nav2.yaml` da corrida;
  · `grafo_antes`, `grafo_depois` — texto do
                           `ros2 topic info -v /hoverboard_base_controller/cmd_vel`;
  · `goal`, `pose_inicial`, `pose_final` — `{'x', 'y'}`.

`evidencia` é o dicionário que o `julga.avalia` recebe; `falhas` é a lista de
falhas de COLETA, `(fonte, detalhe)`, que vira item próprio no `resultado.csv`.
Fonte que falhou deixa o campo AUSENTE — nunca preenchido por outra fonte —, e
o resto da evidência continua montado. A reprovação que o juiz dá depois, por
campo ausente, é consequência deliberada, não duplicidade.

Os formatos dos brutos imitam os REAIS: o dump vivo sai achatado com pontos
(`goal_checker.xy_goal_tolerance`, como em
`~/etapa6/20260924_132510/captura/parametros_brutos.yaml`), o materializado sai
aninhado (como o `perfil_nav2.yaml` da corrida `20260924_160148`), e o grafo
segue o `__str__` do `TopicEndpointInfo` do rclpy deste Jazzy.
"""
import importlib.util
import os

import pytest
import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
MONTA = os.path.join(AQUI, 'monta.py')
JULGA = os.path.join(AQUI, 'julga.py')

TOPICO_FINAL = '/hoverboard_base_controller/cmd_vel'
TOPICO_DIAG = '/cmd_vel_bruto'

# Itens do juiz, como no test_valida_etapa7.py.
SUCESSO = ' 7.1 a ação devolveu SUCCEEDED'
TOLERANCIA = ' 7.2 pose final dentro do xy_goal_tolerance vivo'
NASCEU_FORA = ' 7.2 o objetivo não nasceu dentro da tolerância'
PATAMAR = ' 7.3 comando acima do patamar vivo no consumidor final'
TOPOLOGIA = ' 7.3 topologia nominal do tópico observado'
MODELO = ' 7.3 a placa está no modelo medido'
JANELA = ' 7.3 a amostra está dentro da janela do objetivo'
SETE_ITENS = (SUCESSO, TOLERANCIA, NASCEU_FORA, PATAMAR, TOPOLOGIA, MODELO,
              JANELA)

# Fontes das falhas de coleta.
ACAO = 'acao'
TOL = 'tolerancia'
GRAFO = 'grafo'
PLACA = 'placa'

# action_msgs/msg/GoalStatus
ACCEPTED, EXECUTING, SUCCEEDED, CANCELED, ABORTED = 1, 2, 4, 5, 6

S = 1_000_000_000          # 1 s em ns
UUID = '9306d6c30e57402f8d36bf76a846a8d3'   # o formato do objetivo.log real
OUTRO_UUID = '0123456789abcdef0123456789abcdef'

PLACA_VIVA = {'modelo': 'medido', 'deadband_speed': 100.0,
              'escala_real': 0.0372, 'raio': 0.0825, 'bitola': 0.32}
PATAMAR_VIVO = 100.0 * 0.0372 * 0.0825          # 0,3069 m/s


def _carrega(caminho, nome):
    if not os.path.exists(caminho):
        pytest.fail(f'{caminho} ainda não existe — decisão 060, passo 2: os '
                    'testes do montador vêm antes do código.')
    spec = importlib.util.spec_from_file_location(nome, caminho)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


@pytest.fixture(scope='module')
def monta():
    return _carrega(MONTA, 'monta_etapa7').monta


@pytest.fixture(scope='module')
def avalia():
    return _carrega(JULGA, 'julga_etapa7_monta').avalia


# ─── os brutos sintéticos ────────────────────────────────────────────────────

def _log(uuid=UUID, final='SUCCEEDED'):
    linhas = ['Waiting for an action server to become available...',
              'Sending goal:', '     pose:', '  header:', '    frame_id: map']
    if uuid is not None:
        linhas.append(f'Goal accepted with ID: {uuid}')
    linhas += ['', 'Result:', '    error_code: 0', "error_msg: ''", '']
    if final is not None:
        linhas.append(f'Goal finished with status: {final}')
    return '\n'.join(linhas) + '\n'


def _st(t_ns, status, uuid=UUID, stamp_ns=10 * S):
    return {'t_ns': t_ns,
            'goals': [{'uuid': uuid, 'stamp_ns': stamp_ns, 'status': status}]}


def _status_bom():
    """Aceite em 10 s; terminal gravado em 35 s e republicado depois (o
    servidor mantém o terminal na lista por um tempo — só a PRIMEIRA
    ocorrência fecha a janela)."""
    return [_st(10 * S + 1_000_000, ACCEPTED),
            _st(10 * S + 50_000_000, EXECUTING),
            _st(35 * S, SUCCEEDED),
            _st(36 * S, SUCCEEDED)]


def _amostra(topico, t_ns, v, wz=0.0, header_ns=None):
    return {'topico': topico, 't_ns': t_ns,
            'header_ns': t_ns if header_ns is None else header_ns,
            'v': v, 'wz': wz}


def _dump(no, parametros):
    return yaml.safe_dump({no: {'ros__parameters': parametros}})


def _dump_controller(plugins=('goal_checker',), tolerancias=None):
    """Achatado com pontos, como o `ros2 param dump` real."""
    p = {'controller_frequency': 5.0, 'goal_checker_plugins': list(plugins)}
    for ident, tol in (tolerancias or {'goal_checker': 0.25}).items():
        p[f'{ident}.plugin'] = 'nav2_controller::SimpleGoalChecker'
        p[f'{ident}.xy_goal_tolerance'] = tol
    return _dump('/controller_server', p)


def _materializado(tolerancias=None):
    """Aninhado, como o `perfil_nav2.yaml` materializado real."""
    p = {'controller_frequency': 5.0}
    for ident, tol in (tolerancias or {'goal_checker': 0.25}).items():
        p[ident] = {'plugin': 'nav2_controller::SimpleGoalChecker',
                    'xy_goal_tolerance': tol}
    return yaml.safe_dump({'controller_server': {'ros__parameters': p}})


def _endpoint(no, tipo):
    return '\n'.join([
        f'Node name: {no}', 'Node namespace: /',
        'Topic type: geometry_msgs/msg/TwistStamped',
        'Topic type hash: RIHS01_0000', f'Endpoint type: {tipo}',
        'GID: 01.0f.aa.bb', 'QoS profile:', '  Reliability: RELIABLE',
        '  History (Depth): KEEP_LAST (10)', '  Durability: VOLATILE',
        '  Lifespan: Infinite', '  Deadline: Infinite',
        '  Liveliness: AUTOMATIC',
        '  Liveliness lease duration: Infinite', ''])


def _grafo(pubs=('placa_simulada',),
           subs=('hoverboard_base_controller', 'rosbag2_recorder')):
    partes = ['Type: geometry_msgs/msg/TwistStamped', '',
              f'Publisher count: {len(pubs)}', '']
    partes += [_endpoint(n, 'PUBLISHER') for n in pubs]
    partes += [f'Subscription count: {len(subs)}', '']
    partes += [_endpoint(n, 'SUBSCRIPTION') for n in subs]
    return '\n'.join(partes)


def _brutos():
    return {
        'objetivo_log': _log(),
        'status': _status_bom(),
        'amostras': [
            _amostra(TOPICO_FINAL, 12 * S, 0.0),
            _amostra(TOPICO_FINAL, 20 * S, PATAMAR_VIVO),
            _amostra(TOPICO_DIAG, 20 * S, 0.05),
        ],
        'dump_placa': _dump('/placa_simulada', dict(PLACA_VIVA)),
        'dump_controller': _dump_controller(),
        'nav2_materializado': _materializado(),
        'grafo_antes': _grafo(),
        'grafo_depois': _grafo(),
        'goal': {'x': 1.0, 'y': 0.0},
        'pose_inicial': {'x': 0.0, 'y': 0.0},
        'pose_final': {'x': 0.92, 'y': 0.0},
    }


def _fontes(falhas):
    return [f for f, _ in falhas]


def _detalhes(falhas, fonte):
    return ' | '.join(d for f, d in falhas if f == fonte).lower()


def _t_final(evidencia):
    return [a['t'] for a in evidencia['amostras'] if a['topico'] == TOPICO_FINAL]


# ─── caminho feliz ───────────────────────────────────────────────────────────

def test_evidencia_completa_sem_falha_aprova_os_sete(monta, avalia):
    evidencia, falhas = monta(_brutos())
    assert falhas == [], falhas
    itens = avalia(evidencia)
    for item in SETE_ITENS:
        assert itens[item][0], (item, itens[item])


def test_caminho_feliz_monta_os_campos_das_fontes_certas(monta):
    evidencia, _ = monta(_brutos())
    assert evidencia['resultado_acao'] == 'SUCCEEDED'
    # aceite = goal_info.stamp; resultado = gravação do PRIMEIRO terminal
    assert evidencia['janela'] == {'objetivo_aceito': 10.0, 'resultado': 35.0}
    assert evidencia['xy_goal_tolerance'] == 0.25
    assert evidencia['placa'] == PLACA_VIVA
    topo = evidencia['grafo'][TOPICO_FINAL]
    assert topo['publicadores'] == ['/placa_simulada']
    assert '/hoverboard_base_controller' in topo['assinantes']


# ─── o UUID ──────────────────────────────────────────────────────────────────

def _falha_de_acao(monta, avalia, brutos, *pedacos):
    """Falha da ação: item de coleta próprio, janela e resultado AUSENTES, o
    resto da evidência preservado, e o juiz reprova depois por campo ausente."""
    evidencia, falhas = monta(brutos)
    assert ACAO in _fontes(falhas), falhas
    detalhe = _detalhes(falhas, ACAO)
    for p in pedacos:
        assert p.lower() in detalhe, detalhe
    assert 'janela' not in evidencia and 'resultado_acao' not in evidencia
    for campo in ('placa', 'xy_goal_tolerance', 'grafo', 'amostras'):
        assert campo in evidencia, campo
    itens = avalia(evidencia)
    assert not itens[SUCESSO][0] and 'faltou' in itens[SUCESSO][1]
    assert not itens[JANELA][0] and 'faltou' in itens[JANELA][1]
    return falhas


def test_uuid_ausente_no_log_reprova(monta, avalia):
    b = _brutos()
    b['objetivo_log'] = _log(uuid=None)
    _falha_de_acao(monta, avalia, b, 'uuid')


def test_uuid_duplicado_no_log_reprova(monta, avalia):
    b = _brutos()
    b['objetivo_log'] = _log().replace(
        f'Goal accepted with ID: {UUID}',
        f'Goal accepted with ID: {UUID}\nGoal accepted with ID: {OUTRO_UUID}')
    _falha_de_acao(monta, avalia, b, 'uuid')


def test_goal_concorrente_no_status_reprova(monta, avalia):
    b = _brutos()
    linha = _st(20 * S, EXECUTING)
    linha['goals'].append({'uuid': OUTRO_UUID, 'stamp_ns': 19 * S,
                           'status': EXECUTING})
    b['status'].insert(2, linha)
    _falha_de_acao(monta, avalia, b, OUTRO_UUID)


def test_uuid_do_log_nao_encontrado_no_status_reprova(monta, avalia):
    b = _brutos()
    # o status só tem UUID; o log aponta OUTRO_UUID
    b['objetivo_log'] = _log(uuid=OUTRO_UUID)
    _falha_de_acao(monta, avalia, b, OUTRO_UUID)


# ─── stamp, terminal e o log ─────────────────────────────────────────────────

def test_goal_info_stamp_que_muda_reprova(monta, avalia):
    b = _brutos()
    b['status'][1] = _st(10 * S + 50_000_000, EXECUTING, stamp_ns=11 * S)
    _falha_de_acao(monta, avalia, b, 'stamp')


def test_sem_terminal_gravado_reprova(monta, avalia):
    b = _brutos()
    b['status'] = _status_bom()[:2]
    _falha_de_acao(monta, avalia, b, 'terminal')


@pytest.mark.parametrize('depois', (EXECUTING, ABORTED))
def test_terminal_seguido_de_outro_status_reprova(monta, avalia, depois):
    """Terminal seguido de não terminal, ou de OUTRO terminal: transição
    inconsistente. O terminal repetido igual (caminho feliz) é legítimo."""
    b = _brutos()
    b['status'][3] = _st(36 * S, depois)
    _falha_de_acao(monta, avalia, b, 'terminal')


def test_terminal_divergente_do_log_reprova_e_mantem_o_canonico(monta):
    """O log é só conferência cruzada: divergir dele reprova a COLETA, mas o
    status estruturado continua sendo a fonte canônica válida, e o juiz recebe
    os valores dele — como a tolerância viva divergente do materializado."""
    b = _brutos()
    b['objetivo_log'] = _log(final='ABORTED')
    evidencia, falhas = monta(b)
    assert ACAO in _fontes(falhas), falhas
    assert 'objetivo.log' in _detalhes(falhas, ACAO)
    assert evidencia['resultado_acao'] == 'SUCCEEDED'
    assert evidencia['janela'] == {'objetivo_aceito': 10.0, 'resultado': 35.0}


# ─── cronologia ──────────────────────────────────────────────────────────────

def test_aceite_antes_do_terminal_e_valido(monta):
    evidencia, falhas = monta(_brutos())
    assert falhas == [], falhas
    j = evidencia['janela']
    assert j['objetivo_aceito'] < j['resultado']


def test_aceite_igual_ao_terminal_e_valido(monta, avalia):
    """Janela fechada `[t, t]` é legítima no juiz (G4); a coleta não pode
    inventar uma duração mínima que o contrato não tem. Tudo em ns: a
    amostra gravada no MESMO nanossegundo cai na borda."""
    t = 35 * S
    b = _brutos()
    b['status'] = [_st(t, ACCEPTED, stamp_ns=t), _st(t, SUCCEEDED, stamp_ns=t)]
    b['amostras'] = [_amostra(TOPICO_FINAL, t, PATAMAR_VIVO)]
    evidencia, falhas = monta(b)
    assert falhas == [], falhas
    assert evidencia['janela'] == {'objetivo_aceito': 35.0, 'resultado': 35.0}
    itens = avalia(evidencia)
    assert itens[JANELA][0] and itens[PATAMAR][0], itens


def test_aceite_depois_do_terminal_reprova(monta, avalia):
    b = _brutos()
    b['status'] = [_st(r['t_ns'], r['goals'][0]['status'], stamp_ns=40 * S)
                   for r in _status_bom()]
    _falha_de_acao(monta, avalia, b, 'cronologia')


# ─── o tempo da amostra é o do bag ───────────────────────────────────────────

def test_t_da_amostra_e_o_instante_de_gravacao(monta, avalia):
    """`placa_simulada.py:431` faz `fora.header = msg.header`: o carimbo é o do
    comando de ENTRADA. Aqui ele está dentro da janela, mas a gravação veio
    depois do terminal — e é a gravação que vale."""
    b = _brutos()
    b['amostras'] = [_amostra(TOPICO_FINAL, 36 * S, PATAMAR_VIVO,
                              header_ns=20 * S)]
    evidencia, falhas = monta(b)
    assert falhas == [], falhas
    assert _t_final(evidencia) == [36.0]
    itens = avalia(evidencia)
    assert not itens[JANELA][0], itens[JANELA]


def test_header_fora_da_janela_nao_derruba_amostra_gravada_dentro(monta, avalia):
    b = _brutos()
    b['amostras'] = [_amostra(TOPICO_FINAL, 20 * S, PATAMAR_VIVO,
                              header_ns=5 * S)]
    evidencia, falhas = monta(b)
    assert falhas == [], falhas
    assert _t_final(evidencia) == [20.0]
    assert avalia(evidencia)[JANELA][0]


# ─── a tolerância viva ───────────────────────────────────────────────────────

@pytest.mark.parametrize('plugins', ((), ('goal_checker', 'outro_checker')))
def test_zero_ou_varios_goal_checkers_reprova(monta, avalia, plugins):
    b = _brutos()
    b['dump_controller'] = _dump_controller(
        plugins=plugins,
        tolerancias={'goal_checker': 0.25, 'outro_checker': 0.25})
    evidencia, falhas = monta(b)
    assert TOL in _fontes(falhas), falhas
    assert 'goal_checker_plugins' in _detalhes(falhas, TOL)
    assert 'xy_goal_tolerance' not in evidencia
    assert 'janela' in evidencia and 'placa' in evidencia
    itens = avalia(evidencia)
    assert not itens[TOLERANCIA][0] and 'faltou' in itens[TOLERANCIA][1]


def test_um_goal_checker_e_lido_pelo_id_declarado(monta):
    """O id é o que o controller DECLARA, não o nome `goal_checker`. Um
    `goal_checker.xy_goal_tolerance` velho no dump é armadilha: quem redigitar o
    nome lê 9,0."""
    b = _brutos()
    b['dump_controller'] = _dump_controller(
        plugins=('checador_unico',),
        tolerancias={'checador_unico': 0.25, 'goal_checker': 9.0})
    b['nav2_materializado'] = _materializado({'checador_unico': 0.25})
    evidencia, falhas = monta(b)
    assert falhas == [], falhas
    assert evidencia['xy_goal_tolerance'] == 0.25


def test_materializado_divergente_reprova_a_coleta_e_mantem_o_vivo(monta, avalia):
    b = _brutos()
    b['nav2_materializado'] = _materializado({'goal_checker': 0.30})
    evidencia, falhas = monta(b)
    assert TOL in _fontes(falhas), falhas
    detalhe = _detalhes(falhas, TOL)
    assert '0.25' in detalhe and '0.3' in detalhe, detalhe
    # o juiz continua usando o vivo
    assert evidencia['xy_goal_tolerance'] == 0.25
    assert avalia(evidencia)[TOLERANCIA][0]


def test_dump_vivo_ilegivel_nao_cai_no_materializado(monta):
    """Sem fallback: o materializado é conferência, não fonte."""
    b = _brutos()
    b['dump_controller'] = 'isto: [não é dump'
    evidencia, falhas = monta(b)
    assert TOL in _fontes(falhas), falhas
    assert 'xy_goal_tolerance' not in evidencia


# ─── o grafo ─────────────────────────────────────────────────────────────────

def test_capturas_iguais_do_grafo_aprovam(monta):
    evidencia, falhas = monta(_brutos())
    assert GRAFO not in _fontes(falhas), falhas
    assert 'grafo' in evidencia


def test_capturas_diferentes_do_grafo_reprovam(monta, avalia):
    b = _brutos()
    b['grafo_depois'] = _grafo(pubs=('placa_simulada', 'intruso'))
    evidencia, falhas = monta(b)
    assert GRAFO in _fontes(falhas), falhas
    assert 'grafo' not in evidencia
    assert 'janela' in evidencia and 'placa' in evidencia
    itens = avalia(evidencia)
    assert not itens[TOPOLOGIA][0] and 'faltou' in itens[TOPOLOGIA][1]


def test_mesmos_endpoints_em_outra_ordem_sao_o_mesmo_grafo(monta):
    """A comparação é semântica, não textual: a ordem em que o rclpy lista os
    endpoints não é topologia, e não pode desperdiçar uma corrida."""
    b = _brutos()
    b['grafo_depois'] = _grafo(
        subs=('rosbag2_recorder', 'hoverboard_base_controller'))
    evidencia, falhas = monta(b)
    assert GRAFO not in _fontes(falhas), falhas
    assert 'grafo' in evidencia


@pytest.mark.parametrize('qual', ('grafo_antes', 'grafo_depois'))
def test_captura_ilegivel_do_grafo_reprova(monta, qual):
    b = _brutos()
    b[qual] = 'Unknown topic'
    evidencia, falhas = monta(b)
    assert GRAFO in _fontes(falhas), falhas
    assert 'grafo' not in evidencia


# ─── fonte inválida: campo ausente, evidência parcial preservada ─────────────

def test_dump_da_placa_ilegivel_deixa_so_a_placa_ausente(monta, avalia):
    b = _brutos()
    b['dump_placa'] = ''
    evidencia, falhas = monta(b)
    assert _fontes(falhas) == [PLACA], falhas
    assert 'placa' not in evidencia
    for campo in ('janela', 'resultado_acao', 'xy_goal_tolerance', 'grafo',
                  'amostras', 'goal', 'pose_inicial', 'pose_final'):
        assert campo in evidencia, campo
    itens = avalia(evidencia)
    assert itens[SUCESSO][0] and itens[TOLERANCIA][0], itens
    assert not itens[MODELO][0] and 'faltou' in itens[MODELO][1]


def test_falha_de_coleta_vem_separada_do_veredito_do_juiz(monta, avalia):
    """A falha de coleta é devolvida pelo montador, não escondida dentro do
    veredito do juiz: o `avalia` só conhece os seus sete itens."""
    b = _brutos()
    b['objetivo_log'] = _log(uuid=None)
    evidencia, falhas = monta(b)
    assert falhas and all(isinstance(f, tuple) and len(f) == 2
                          for f in falhas), falhas
    itens = avalia(evidencia)
    assert set(itens) == set(SETE_ITENS)
    assert not itens[SUCESSO][0]
