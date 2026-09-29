"""Prova das conferências que abortam uma tentativa do A/B.

Uma conferência que nunca reprova é pior que nenhuma: ela dá a sensação de que
o protocolo foi seguido. Cada teste aqui tem o par — o caso que passa e o que
reprova — e checa que o MOTIVO diz o que houve, porque é ele que vai para o
registro da tentativa.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import prontidao as pr  # noqa: E402
import provas  # noqa: E402


# ─── pose na largada ─────────────────────────────────────────────────────────

def test_pose_certa_na_largada_passa():
    ok, motivo = pr.pose_na_largada(
        {'x': 2.001, 'y': 4.999, 'yaw': 0.001}, provas.POSE_CHEGADA)
    assert ok, motivo


def test_pose_deslocada_aborta():
    ok, motivo = pr.pose_na_largada(
        {'x': 2.3, 'y': 5.0, 'yaw': 0.0}, provas.POSE_CHEGADA)
    assert not ok
    assert '0.300' in motivo and 'tolerância' in motivo, motivo


def test_yaw_torto_aborta_mesmo_com_xy_certos():
    """Entrar na porta 20° torto é justamente o que muda o resultado."""
    ok, motivo = pr.pose_na_largada(
        {'x': 2.0, 'y': 5.0, 'yaw': 0.35}, provas.POSE_CHEGADA)
    assert not ok and 'yaw' in motivo, motivo


def test_yaw_na_virada_do_pi_nao_e_falso_erro():
    """−3,10 e +3,18 rad são 0,04 de diferença, não 6,28."""
    ok, motivo = pr.pose_na_largada(
        {'x': 11.0505, 'y': 1.5577, 'yaw': math.pi - 0.01},
        {'x': 11.0505, 'y': 1.5577, 'yaw': -math.pi + 0.01})
    assert ok, motivo


def test_sem_leitura_de_pose_aborta():
    ok, motivo = pr.pose_na_largada(None, provas.POSE_CHEGADA)
    assert not ok and 'nenhuma leitura' in motivo


# ─── repouso ─────────────────────────────────────────────────────────────────

def test_parado_o_suficiente_passa():
    amostras = [(i * 0.02, 0.0) for i in range(50)]
    ok, motivo = pr.repouso_sustentado(amostras)
    assert ok, motivo


def test_ainda_andando_nao_e_repouso():
    amostras = [(i * 0.02, 0.1) for i in range(50)]
    ok, motivo = pr.repouso_sustentado(amostras)
    assert not ok and 'movendo' in motivo, motivo


def test_janela_curta_demais_nao_conta():
    """Duas amostras zeradas não são 0,30 s parado."""
    ok, motivo = pr.repouso_sustentado([(0.0, 0.0), (0.02, 0.0)])
    assert not ok and 'precisa de' in motivo, motivo


def test_zerou_agora_mas_andava_ha_pouco_nao_conta():
    amostras = [(i * 0.02, 0.2) for i in range(20)]
    amostras += [(0.4 + i * 0.02, 0.0) for i in range(5)]   # só 0,10 s parado
    ok, motivo = pr.repouso_sustentado(amostras)
    assert not ok, motivo


# ─── freio vivo ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize('texto,esperado', [
    ('Boolean value is: True', True),
    ('Boolean value is: False', False),
])
def test_freio_vivo_igual_ao_pedido_passa(texto, esperado):
    ok, motivo = pr.freio_vivo_confere(texto, esperado)
    assert ok, motivo


def test_freio_vivo_diferente_do_pedido_aborta():
    """🔴 O caso que salva a sessão: a pilha subiu na condição errada."""
    ok, motivo = pr.freio_vivo_confere('Boolean value is: True', False)
    assert not ok
    assert 'não está na condição do teste' in motivo, motivo


def test_param_ilegivel_aborta():
    for ruim in (None, '', 'Parameter not set'):
        ok, _ = pr.freio_vivo_confere(ruim, True)
        assert not ok, ruim


# ─── tolerância do goal_checker ──────────────────────────────────────────────

def test_tolerancia_igual_a_do_juiz_passa():
    ok, motivo = pr.tolerancia_confere('0.25')
    assert ok, motivo


def test_tolerancia_diferente_aborta():
    ok, motivo = pr.tolerancia_confere('0.35')
    assert not ok and 'comparáveis' in motivo, motivo


# ─── bag ─────────────────────────────────────────────────────────────────────

EXIGIDOS = ['/Odometry', '/scan', '/tf', '/goal_pose', '/plan', '/auto_vel',
            '/compensador_rumo/cmd_vel', '/cmd_vel_bruto', '/unstuck_vel',
            '/collision_monitor_state']


def test_bag_completo_passa():
    ok, motivo = pr.bag_confere(True, EXIGIDOS, EXIGIDOS)
    assert ok, motivo


def test_sem_metadata_reprova():
    ok, motivo = pr.bag_confere(False, EXIGIDOS, EXIGIDOS)
    assert not ok and 'metadata' in motivo, motivo


def test_unstuck_fora_do_bag_reprova():
    """🔴 Sem `/unstuck_vel` gravado, "zero escapes" não pode ser afirmado."""
    sem = [t for t in EXIGIDOS if t != '/unstuck_vel']
    ok, motivo = pr.bag_confere(True, sem, EXIGIDOS)
    assert not ok and '/unstuck_vel' in motivo, motivo


# ─── os bloqueadores do wrapper (revisão de 29-09, 3ª rodada) ────────────────

ODOM_REAL = """
pose:
  pose:
    position:
      x: 2.0
      y: 5.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
twist:
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
"""


def test_pose_nao_confunde_position_com_orientation():
    """🔴 O parser antigo devolvia x = 0,0 para uma pose em (2,0; 5,0).

    Ele pegava as quatro primeiras ocorrências de `[xyzw]` e o `x` de
    `orientation` sobrescrevia o de `position`. Com isso a conferência de
    largada reprovaria (ou aprovaria) por motivo inventado.
    """
    p = pr.pose_de_yaml(ODOM_REAL)
    assert p is not None
    assert p['x'] == pytest.approx(2.0), p
    assert p['y'] == pytest.approx(5.0), p
    assert p['yaw'] == pytest.approx(0.0, abs=1e-9), p


def test_pose_com_yaw_de_verdade():
    y = ODOM_REAL.replace('      z: 0.0\n      w: 1.0',
                          '      z: -0.7123909035318096\n'
                          '      w: 0.7017828728069189')
    p = pr.pose_de_yaml(y)
    assert p['yaw'] == pytest.approx(-1.5857984884015157, abs=1e-9), p


def test_pose_vazia_ou_lixo_devolve_none():
    for ruim in ('', '   ', 'nao sou yaml: [', None):
        assert pr.pose_de_yaml(ruim) is None, ruim


def test_duas_amostras_isoladas_nao_sao_repouso():
    """🔴 A versão anterior aceitava t=0 e t=0,31 como 0,30 s parado.

    Entre as duas o robô podia ter andado um metro.
    """
    ok, motivo = pr.repouso_sustentado([(0.0, 0.0), (0.31, 0.0)])
    assert not ok, motivo


def test_janela_com_buraco_nao_e_continua():
    amostras = [(0.0, 0.0), (0.02, 0.0), (0.04, 0.0), (0.5, 0.0), (0.52, 0.0)]
    ok, motivo = pr.repouso_sustentado(amostras)
    assert not ok and ('buraco' in motivo or 'amostra' in motivo), motivo


def test_robo_girando_no_lugar_nao_esta_parado():
    """`vx` zero com `wz` alto é giro — e giro muda a pose de entrada."""
    amostras = [(i * 0.02, 0.0, 0.4) for i in range(40)]
    ok, motivo = pr.repouso_sustentado(amostras)
    assert not ok and 'girando' in motivo, motivo


def test_repouso_denso_e_continuo_passa():
    amostras = [(i * 0.02, 0.0, 0.0) for i in range(40)]
    ok, motivo = pr.repouso_sustentado(amostras)
    assert ok, motivo


def test_terminal_le_o_uuid_do_jazzy():
    """🔴 O grep antigo procurava `goal_id=`, que o Jazzy não escreve."""
    texto = ('Waiting for an action server...\n'
             'Goal accepted with ID: 8f1c2d3e4a5b\n'
             'Result:\n\nGoal finished with status: SUCCEEDED')
    aceito, uuid, estado = pr.terminal_da_acao(texto)
    assert aceito and uuid == '8f1c2d3e4a5b' and estado == 'SUCCEEDED'


@pytest.mark.parametrize('texto,esperado', [
    ('Goal accepted with ID: aa\nGoal finished with status: ABORTED', 'ABORTED'),
    ('Goal accepted with ID: aa\nGoal finished with status: CANCELED', 'CANCELED'),
    ('Goal was rejected!', 'REJECTED'),
    ('', 'UNKNOWN'),
])
def test_terminais_sao_classificados(texto, esperado):
    assert pr.terminal_da_acao(texto)[2] == esperado


def test_goal_rejeitado_nao_e_aceite():
    aceito, _, estado = pr.terminal_da_acao('Goal was rejected!')
    assert not aceito and estado == 'REJECTED'


# ─── contagens do bag ────────────────────────────────────────────────────────

def _metadata(pares):
    linhas = ['rosbag2_bagfile_information:', '  topics_with_message_count:']
    for nome, n in pares:
        linhas += [f'  - topic_metadata:', f'      name: {nome}',
                   f'    message_count: {n}']
    return '\n'.join(linhas)


def test_contagens_distinguem_ausente_de_zerado():
    """🔴 `/unstuck_vel` com zero mensagens é "zero escapes"; ausente é "não
    medido". Conferir só nomes perde essa diferença."""
    exigidos = ['/Odometry', '/unstuck_vel']
    ok, _, cont = pr.contagens_do_bag(
        _metadata([('/Odometry', 900), ('/unstuck_vel', 0)]), exigidos)
    assert ok and cont['/unstuck_vel'] == 0

    ok2, motivo2, _ = pr.contagens_do_bag(
        _metadata([('/Odometry', 900)]), exigidos)
    assert not ok2 and '/unstuck_vel' in motivo2


def test_topico_essencial_sem_mensagem_reprova():
    """Odometria assinada e vazia é coleta perdida, não resultado."""
    ok, motivo, _ = pr.contagens_do_bag(
        _metadata([('/Odometry', 0), ('/unstuck_vel', 0)]),
        ['/Odometry', '/unstuck_vel'], exigem_mensagem=['/Odometry'])
    assert not ok and 'sem mensagem' in motivo, motivo


def test_metadata_ausente_reprova():
    ok, motivo, _ = pr.contagens_do_bag(None, ['/Odometry'])
    assert not ok and 'ausente' in motivo
