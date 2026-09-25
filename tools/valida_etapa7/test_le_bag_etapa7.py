#!/usr/bin/env python3
"""O extrator do bag do passo 7 (decisão 060) — contra um bag PEQUENO que o
próprio teste escreve, nunca o bag real.

🔴 NASCEM VERMELHOS: `tools/valida_etapa7/le_bag.py` ainda não existe. Interface:

    le(caminho_do_bag) -> {'amostras': [...], 'status': [...] ou None}

no formato exato que o `monta.py` recebe:

  · amostras: `topico`, `t_ns` (instante de GRAVAÇÃO), `header_ns` (só
    diagnóstico), `v`, `wz` — do tópico final e do `/cmd_vel_bruto`;
  · status: `t_ns` (gravação) e `goals`, cada um com `uuid` (32 hex
    minúsculos), `stamp_ns` e `status`. Tópico de status AUSENTE do bag vira
    `None`, não lista vazia: "não foi gravado" e "gravado sem mensagem" são
    falhas diferentes, e o montador diz a primeira.

Precisa de `rosbag2_py` (ROS carregado); sem ele, pula — como os outros testes
do repositório que dependem de ROS.
"""
import importlib.util
import os

import pytest

rosbag2_py = pytest.importorskip('rosbag2_py')
serialization = pytest.importorskip('rclpy.serialization')
action_msgs = pytest.importorskip('action_msgs.msg')
geometry_msgs = pytest.importorskip('geometry_msgs.msg')

AQUI = os.path.dirname(os.path.abspath(__file__))
LE_BAG = os.path.join(AQUI, 'le_bag.py')

TOPICO_FINAL = '/hoverboard_base_controller/cmd_vel'
TOPICO_DIAG = '/cmd_vel_bruto'
STATUS = '/navigate_to_pose/_action/status'
TWIST_STAMPED = 'geometry_msgs/msg/TwistStamped'
STATUS_ARRAY = 'action_msgs/msg/GoalStatusArray'

S = 1_000_000_000
UUID = bytes(range(16))
UUID_HEX = UUID.hex()       # '000102...0f'
ACCEPTED, SUCCEEDED = 1, 4


@pytest.fixture(scope='module')
def le():
    if not os.path.exists(LE_BAG):
        pytest.fail(f'{LE_BAG} ainda não existe — decisão 060, passo 4: o '
                    'teste do extrator vem antes do código.')
    spec = importlib.util.spec_from_file_location('le_bag_etapa7', LE_BAG)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m.le


def _twist(v, wz, header_ns):
    m = geometry_msgs.TwistStamped()
    m.header.stamp.sec, m.header.stamp.nanosec = divmod(header_ns, S)
    m.twist.linear.x = v
    m.twist.angular.z = wz
    return m


def _status(codigo, stamp_ns, uuid=UUID):
    arr = action_msgs.GoalStatusArray()
    g = action_msgs.GoalStatus()
    g.goal_info.goal_id.uuid = list(uuid)
    g.goal_info.stamp.sec, g.goal_info.stamp.nanosec = divmod(stamp_ns, S)
    g.status = codigo
    arr.status_list = [g]
    return arr


def _escreve(caminho, topicos, mensagens):
    """`topicos`: {nome: tipo}; `mensagens`: [(nome, msg, t_gravacao_ns)]."""
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(caminho), storage_id='mcap'),
           rosbag2_py.ConverterOptions('cdr', 'cdr'))
    for i, (nome, tipo) in enumerate(topicos.items()):
        w.create_topic(rosbag2_py.TopicMetadata(
            id=i, name=nome, type=tipo, serialization_format='cdr'))
    for nome, msg, t in mensagens:
        w.write(nome, serialization.serialize_message(msg), t)
    del w
    return str(caminho)


def _bag_bom(tmp_path):
    topicos = {STATUS: STATUS_ARRAY, TOPICO_FINAL: TWIST_STAMPED,
               TOPICO_DIAG: TWIST_STAMPED,
               '/Odometry': 'nav_msgs/msg/Odometry'}
    odom = pytest.importorskip('nav_msgs.msg').Odometry()
    msgs = [
        (STATUS, _status(ACCEPTED, 10 * S), 10 * S + 1_000_000),
        (TOPICO_DIAG, _twist(0.05, 0.0, 19 * S), 20 * S),
        # header herdado do comando de entrada: 19 s; gravado em 20 s
        (TOPICO_FINAL, _twist(0.3069, -0.5, 19 * S), 20 * S + 7),
        ('/Odometry', odom, 21 * S),
        (STATUS, _status(SUCCEEDED, 10 * S), 35 * S),
    ]
    return _escreve(tmp_path / 'bag', topicos, msgs)


def test_amostras_com_instante_de_gravacao_e_header_a_parte(le, tmp_path):
    saida = le(_bag_bom(tmp_path))
    assert saida['amostras'] == [
        {'topico': TOPICO_DIAG, 't_ns': 20 * S, 'header_ns': 19 * S,
         'v': 0.05, 'wz': 0.0},
        {'topico': TOPICO_FINAL, 't_ns': 20 * S + 7, 'header_ns': 19 * S,
         'v': 0.3069, 'wz': -0.5},
    ]


def test_tempos_sao_inteiros_de_nanossegundos(le, tmp_path):
    saida = le(_bag_bom(tmp_path))
    for a in saida['amostras']:
        assert type(a['t_ns']) is int and type(a['header_ns']) is int, a
    for linha in saida['status']:
        assert type(linha['t_ns']) is int, linha
        for g in linha['goals']:
            assert type(g['stamp_ns']) is int and type(g['status']) is int, g


def test_status_com_uuid_hex_stamp_e_gravacao(le, tmp_path):
    saida = le(_bag_bom(tmp_path))
    assert saida['status'] == [
        {'t_ns': 10 * S + 1_000_000,
         'goals': [{'uuid': UUID_HEX, 'stamp_ns': 10 * S,
                    'status': ACCEPTED}]},
        {'t_ns': 35 * S,
         'goals': [{'uuid': UUID_HEX, 'stamp_ns': 10 * S,
                    'status': SUCCEEDED}]},
    ]


def test_topico_de_status_ausente_vira_none(le, tmp_path):
    """O caso do bag real de 20260924_160148: sem `--include-hidden-topics`
    o status não foi gravado."""
    caminho = _escreve(tmp_path / 'bag', {TOPICO_FINAL: TWIST_STAMPED},
                       [(TOPICO_FINAL, _twist(0.3, 0.0, S), S)])
    saida = le(caminho)
    assert saida['status'] is None
    assert len(saida['amostras']) == 1


def test_topico_de_status_gravado_sem_mensagem_vira_lista_vazia(le, tmp_path):
    caminho = _escreve(tmp_path / 'bag', {STATUS: STATUS_ARRAY,
                                          TOPICO_FINAL: TWIST_STAMPED}, [])
    saida = le(caminho)
    assert saida['status'] == [] and saida['amostras'] == []


def test_tipo_inesperado_no_topico_final_reprova_com_mensagem(le, tmp_path):
    """Se o tópico final vier como `Twist`, não há `header` nem o mesmo
    layout: ler assim mesmo seria adivinhar."""
    m = geometry_msgs.Twist()
    caminho = _escreve(tmp_path / 'bag',
                       {TOPICO_FINAL: 'geometry_msgs/msg/Twist'},
                       [(TOPICO_FINAL, m, S)])
    with pytest.raises(ValueError, match=TOPICO_FINAL):
        le(caminho)
