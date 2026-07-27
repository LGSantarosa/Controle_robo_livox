"""Testes da navegação ponto a ponto (decisão 006).

Como no teste da lei de rumo: cada caso trava uma propriedade que veio de
medida ou de limite físico, não um detalhe de implementação.
"""
import math

import pytest

from robot_motion.navegacao_ponto import (
    chegou,
    comando_de_navegacao,
    distancia,
    raio_minimo_de_chegada,
    rumo_para,
    velocidade_de_aproximacao,
)

V_MAX = 0.5
A_LIN = 0.5
V_MIN = 0.20
RAIO = 0.15


# ------------------------------------------------------- geometria

def test_rumo_aponta_para_o_ponto():
    assert rumo_para(0, 0, 1, 0) == pytest.approx(0.0)
    assert rumo_para(0, 0, 0, 1) == pytest.approx(math.pi / 2)
    assert rumo_para(0, 0, 1, 1) == pytest.approx(math.pi / 4)


def test_ponto_atras_devolve_meia_volta():
    assert abs(rumo_para(0, 0, -1, 0)) == pytest.approx(math.pi)


def test_rumo_e_recalculado_da_pose_atual():
    """É isso que dissolve o erro lateral sem controlador extra.

    Um robô que saiu da linha vê um rumo alvo diferente — e por isso curva de
    volta sozinho, em vez de seguir paralelo à rota.
    """
    na_linha = rumo_para(0.0, 0.0, 10.0, 0.0)
    deslocado = rumo_para(0.0, -0.66, 10.0, 0.0)
    assert deslocado > na_linha, 'deslocado para -y deveria mirar para cima'


# ------------------------------------------------------- aproximação

def test_nunca_chega_mais_rapido_do_que_consegue_frear():
    """A lei de frenagem, agora em distância. Espelha a decisão 005."""
    for d in [0.5, 1.0, 3.0, 10.0]:
        v = velocidade_de_aproximacao(d, V_MAX, A_LIN, 0.0)
        assert v ** 2 / (2 * A_LIN) <= d + 1e-9


def test_longe_vai_no_teto():
    assert velocidade_de_aproximacao(50.0, V_MAX, A_LIN, V_MIN) == pytest.approx(V_MAX)


def test_aproximacao_nunca_desce_do_minimo_viavel():
    """A zona morta proíbe chegar 'devagarinho'.

    Sem o piso, a lei manda velocidades cada vez menores, a placa engole
    todas e o robô para longe do ponto sem acusar nada — o BO-3 disfarçado
    de 'chegou'.
    """
    for d in [0.001, 0.01, 0.05, 0.1]:
        assert velocidade_de_aproximacao(d, V_MAX, A_LIN, V_MIN) >= V_MIN


def test_a_lin_invalida_e_erro_explicito():
    with pytest.raises(ValueError):
        velocidade_de_aproximacao(1.0, V_MAX, 0.0, V_MIN)


def test_subestimar_a_lin_e_conservador():
    """Mesma regra de ouro da a_dec: errar pra baixo só faz frear antes."""
    for d in [0.3, 1.0, 5.0]:
        assert (velocidade_de_aproximacao(d, V_MAX, A_LIN / 3, 0.0)
                <= velocidade_de_aproximacao(d, V_MAX, A_LIN, 0.0) + 1e-12)


# ------------------------------------------------------- chegada

def test_raio_minimo_cobre_a_parada_a_partir_do_piso():
    """Raio menor que a distância de parada faria o robô orbitar o ponto."""
    minimo = raio_minimo_de_chegada(V_MIN, A_LIN, folga=0.0)
    assert minimo == pytest.approx(V_MIN ** 2 / (2 * A_LIN))
    # com o raio mínimo, parar a partir do piso cabe dentro do raio
    assert V_MIN ** 2 / (2 * A_LIN) <= raio_minimo_de_chegada(V_MIN, A_LIN)


def test_chegada_corta_firme_e_nao_assintotica():
    ch, _, v = comando_de_navegacao(0.0, 0.0, 0.10, 0.0, V_MAX, A_LIN, V_MIN, RAIO)
    assert ch is True
    assert v == 0.0


def test_fora_do_raio_ainda_anda():
    ch, _, v = comando_de_navegacao(0.0, 0.0, 2.0, 0.0, V_MAX, A_LIN, V_MIN, RAIO)
    assert ch is False
    assert v >= V_MIN


def test_o_robo_para_dentro_do_raio_partindo_do_piso():
    """Coerência entre piso, desaceleração e raio — a propriedade que impede
    o robô de entrar e sair do raio para sempre."""
    parada = V_MIN ** 2 / (2 * A_LIN)
    assert parada <= RAIO, (
        f'parando do piso ele percorre {parada:.3f} m, mas o raio é {RAIO} m')


def test_distancia_e_chegou_sao_coerentes():
    assert distancia(0, 0, 3, 4) == pytest.approx(5.0)
    assert chegou(0.10, 0.15) is True
    assert chegou(0.20, 0.15) is False


def test_alvo_atras_navega_sem_pedir_re():
    """Ponto atrás do robô: a navegação pede rumo de meia-volta e velocidade
    positiva. Quem segura o avanço enquanto o rumo está torto é a
    movimentação, com cos(e) — a navegação não conhece atuador."""
    ch, rumo, v = comando_de_navegacao(0.0, 0.0, -2.0, 0.0, V_MAX, A_LIN,
                                       V_MIN, RAIO)
    assert ch is False
    assert abs(rumo) == pytest.approx(math.pi)
    assert v > 0.0
