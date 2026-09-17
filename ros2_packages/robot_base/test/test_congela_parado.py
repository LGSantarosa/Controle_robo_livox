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


# 🔴 Os dois abaixo são o defeito de 17-09: estado POR RODA. Antes havia um
# `t_roda` só, e o nó chamava com o valor em cache do outro lado (nascido 0,0),
# então uma roda sozinha mantinha a trava armada. Ver `congela_parado.py`.

def test_roda_que_nunca_publicou_nao_deixa_congelar():
    """Direita muda desde sempre: a esquerda mandando zero NÃO pode congelar.

    É o caso perigoso — o robô pode estar sendo tocado pela roda que não
    reporta, e congelar a TF faria a pose mentir com o robô andando.
    """
    c = CongelaParado(espera=0.5)
    for k in range(10):                      # só a esquerda publica, 1 s
        c.roda(k * 0.1, 0, 0.0)
    c.passo(0.0, (0.0, 0.0, 0.0), yaw_q(0.0))
    t, _ = c.passo(1.0, (0.7, 0.0, 0.0), yaw_q(0.0))
    perto(t, (0.7, 0.0, 0.0))                # passou o LIO direto


def test_leitura_velha_de_UMA_roda_nao_deixa_congelar():
    """Esquerda fresca, direita parou de chegar: não congela.

    Driver de um lado caído no meio da operação é o mesmo risco, e a validade
    tem de valer por roda — não para a última mensagem que chegou de qualquer
    uma.
    """
    c = CongelaParado(espera=0.5, validade=0.5)
    c.roda(0.0, 0, 0.0)
    c.roda(0.0, 1, 0.0)
    c.passo(0.0, (0.0, 0.0, 0.0), yaw_q(0.0))
    for k in range(1, 21):                   # só a esquerda segue publicando
        c.roda(k * 0.1, 0, 0.0)
    t, _ = c.passo(2.0, (0.4, 0.0, 0.0), yaw_q(0.0))
    perto(t, (0.4, 0.0, 0.0))                # direita velha -> sem congelar


def test_roda_que_VOLTA_tem_de_reobservar_a_espera():
    """🔴 Regressão de 17-09: o cronômetro tem de zerar no apagão.

    Antes, `parado_desde` sobrevivia à janela de leitura velha. Quando o lado
    sumido voltava com zero, a trava congelava NO PRIMEIRO PACOTE, usando o
    carimbo antigo — sem reobservar os 0,5 s. Perigoso porque o apagão é
    justamente quando não se sabe se o robô andou.
    """
    c = CongelaParado(espera=0.5, validade=0.5)
    for k in range(11):                       # 1 s com as duas paradas
        c.roda(k * 0.1, 0, 0.0)
        c.roda(k * 0.1, 1, 0.0)
    c.passo(1.0, (0.0, 0.0, 0.0), yaw_q(0.0))
    assert c.congelado(1.0), 'com as duas frescas e paradas, tem de congelar'

    for k in range(11, 41):                   # 3 s só com a esquerda
        c.roda(k * 0.1, 0, 0.0)
    assert not c.congelado(4.0), 'leitura velha não pode congelar'

    c.roda(4.0, 1, 0.0)                       # a direita VOLTA, um pacote
    assert not c.congelado(4.0), (
        'congelou no primeiro pacote que voltou: o cronômetro não zerou')

    for k in range(41, 47):                   # 0,5 s com as duas de novo
        c.roda(k * 0.1, 0, 0.0)
        c.roda(k * 0.1, 1, 0.0)
    assert c.congelado(4.6), 'depois de reobservar a espera, tem de congelar'
