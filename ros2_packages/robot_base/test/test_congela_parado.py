"""Trava o congelamento do odom com as rodas paradas (decisão 050)."""

import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'robot_base'))

from congela_parado import CongelaParado  # noqa: E402


def yaw_q(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def perto(a, b, tol=1e-9):
    for x, y in zip(a, b):
        assert abs(x - y) < tol, f'{a} != {b}'


def test_sem_leitura_de_roda_passa_o_lio_direto():
    c = CongelaParado()
    t, q = c.passo(10.0, (1.0, 2.0, 0.0), yaw_q(0.3))
    perto(t, (1.0, 2.0, 0.0))
    perto(q, yaw_q(0.3))


def test_parado_nao_mexe_e_ao_andar_nao_da_degrau():
    c = CongelaParado(espera=0.5)
    c.rodas(0.0, 0.0, 0.0)
    c.passo(0.1, (1.0, 0.0, 0.0), yaw_q(0.0))
    # parado há 1 s: o LIO deriva 30 cm e 0,3 rad, a TF não
    c.rodas(1.0, 0.0, 0.0)
    t, q = c.passo(1.0, (1.3, 0.0, 0.0), yaw_q(0.3))
    perto(t, (1.0, 0.0, 0.0))
    perto(q, yaw_q(0.0))
    # rodas voltam: o LIO anda 1 m para a frente dele; a TF anda 1 m a partir
    # de onde parou, no rumo em que parou
    c.rodas(1.1, 2.0, 2.0)
    t, q = c.passo(1.1, (1.3 + math.cos(0.3), math.sin(0.3), 0.0), yaw_q(0.3))
    perto(t, (2.0, 0.0, 0.0), 1e-9)
    perto(q, yaw_q(0.0))


def test_leitura_velha_nao_congela():
    c = CongelaParado(espera=0.5, validade=0.5)
    c.rodas(0.0, 0.0, 0.0)
    c.passo(0.0, (0.0, 0.0, 0.0), yaw_q(0.0))
    t, _ = c.passo(2.0, (0.5, 0.0, 0.0), yaw_q(0.0))
    perto(t, (0.5, 0.0, 0.0))


def test_uma_roda_girando_nao_congela():
    c = CongelaParado(espera=0.0)
    c.rodas(0.0, 0.0, 1.0)
    c.passo(0.0, (0.0, 0.0, 0.0), yaw_q(0.0))
    t, q = c.passo(0.1, (0.0, 0.0, 0.0), yaw_q(0.2))
    perto(q, yaw_q(0.2))
