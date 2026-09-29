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
