"""Testes da lei de seguimento de caminho (decisão 008, fatia A).

Cada teste trava uma propriedade que veio de um DEFEITO MEDIDO, não de gosto.
Se um destes cair, é um defeito conhecido voltando.

Fatia A cobre caminho SEM cúspide: carrot, lookahead e o teto de velocidade
pela curva. Cúspide e ré são a fatia B.
"""
import math

import pytest

from robot_motion.lei_de_seguimento import (
    carrot,
    curvatura_adiante,
    indice_mais_proximo,
    lookahead_de,
    rumo_para,
    velocidade_de_seguimento,
)

# Números do perfil pessimista de 29-07, que é o que o robô provavelmente é.
RAIO_MIN = 0.46
WZ_MAX = 1.0
V_MAX = 0.5
A_LIN = 0.3


def reta(n=20, passo=0.1):
    return [(i * passo, 0.0) for i in range(n)]


def arco(raio, total_deg=90.0, passo_deg=5.0):
    pts, ang = [], 0.0
    while ang <= math.radians(total_deg):
        pts.append((raio * math.sin(ang), raio * (1.0 - math.cos(ang))))
        ang += math.radians(passo_deg)
    return pts


# --------------------------------------------------------------- lookahead

def test_lookahead_sai_do_raio_minimo_nao_de_numero_solto():
    """O lookahead é derivado, não escolhido.

    É o parâmetro central deste nó, como o `a_dec` é o da movimentação. Fixá-lo
    num número solto o transformaria em mais um valor herdado sem justificativa
    — que é exatamente como a bitola (0,32) e o raio de roda (0,0825) chegaram
    errados até 29-07. Amarrado ao raio mínimo, ele acompanha a máquina quando a
    zona morta for medida.
    """
    assert lookahead_de(RAIO_MIN, fator=1.5) == pytest.approx(0.69)
    # dobrou o raio da máquina, dobrou o lookahead
    assert lookahead_de(2 * RAIO_MIN, fator=1.5) == pytest.approx(1.38)


def test_lookahead_tem_piso():
    """Raio pequeno não pode produzir lookahead que mira quase nos pés.

    Carrot colado no robô faz o rumo alvo oscilar com qualquer ruído de pose —
    é o mecanismo do ciclo-limite de rumo, o mesmo S de 27-07 entrando por
    outra porta.
    """
    assert lookahead_de(0.01, fator=1.5, piso=0.30) == pytest.approx(0.30)


# ------------------------------------------------------------------ carrot

def test_carrot_anda_pelo_ARCO_do_caminho_nao_pela_linha_reta():
    """O carrot segue o comprimento do caminho, não a distância em linha reta.

    Medido pelo carrot em linha reta: numa curva fechada, o ponto a 0,7 m de
    distância EUCLIDIANA pode estar muito mais à frente no caminho, e o robô
    corta a curva por dentro — que é como se raspa a quina de um vão.
    """
    # Arco LONGO (180°) e passo fino, de propósito: num arco de 90° a corda e o
    # comprimento ainda ficam perto o bastante para as duas contas darem quase
    # o mesmo, e o teste passava com a conta errada. Aqui, para um lookahead de
    # 0,69 m, o arco dá 0,69 e a corda daria ~0,78 — separados com folga.
    pts = arco(0.46, 180.0, passo_deg=2.0)
    _, alvo = carrot(pts, 0, 0.69)
    percorrido = 0.0
    for a, b in zip(pts, pts[1:]):
        percorrido += math.hypot(b[0] - a[0], b[1] - a[1])
        if b == alvo:
            break
    assert percorrido == pytest.approx(0.69, abs=0.03), (
        f'carrot a {percorrido:.3f} m de ARCO — se deu ~0,78, ele voltou a '
        'medir pela linha reta e o robô corta a curva por dentro')


def test_carrot_no_fim_do_caminho_devolve_o_ultimo_ponto():
    pts = reta(10, 0.1)          # 0,9 m de caminho
    i, alvo = carrot(pts, 0, 5.0)
    assert alvo == pts[-1]
    assert i == len(pts) - 1


def test_indice_mais_proximo_acha_onde_o_robo_esta():
    pts = reta(20, 0.1)
    assert indice_mais_proximo(pts, 0.52, 0.03) == 5


def test_rumo_para_o_carrot():
    assert rumo_para(0.0, 0.0, (1.0, 1.0)) == pytest.approx(math.pi / 4)


# ------------------------------------------- o teto de velocidade pela curva

def test_velocidade_tem_TETO_PELA_CURVA():
    """O defeito nº 2 da sessão de 28-07, e o que produziu o balão.

    `raio = v / wz`. Com `wz_max` de 1,0 rad/s e a linear no teto, o robô é
    obrigado a descrever um arco de `v/wz_max` — foi assim que um alvo a 0,43 m
    custou 3,66 m de caminho. A linear tem que ceder para a curva caber.
    """
    v = velocidade_de_seguimento(
        dist_ao_fim=10.0, raio_da_curva=0.30,
        v_max=V_MAX, a_lin=A_LIN, wz_max=WZ_MAX)
    assert v <= 0.30 * WZ_MAX + 1e-9, (
        f'v={v:.3f} exige raio maior que os 0,30 m da curva — o balão voltou')


def test_reta_longa_anda_no_teto():
    """Sem curva e sem fim à vista, não há razão para ir devagar."""
    v = velocidade_de_seguimento(
        dist_ao_fim=10.0, raio_da_curva=float('inf'),
        v_max=V_MAX, a_lin=A_LIN, wz_max=WZ_MAX)
    assert v == pytest.approx(V_MAX)


def test_freia_pela_distancia_ate_o_fim():
    """Mesma lei da decisão 005, agora em distância: `v = sqrt(2·a·d)`.

    Mesmo princípio, mesmo tipo de parâmetro físico — não é ganho ajustado.
    """
    v = velocidade_de_seguimento(
        dist_ao_fim=0.10, raio_da_curva=float('inf'),
        v_max=V_MAX, a_lin=A_LIN, wz_max=WZ_MAX)
    assert v == pytest.approx(math.sqrt(2 * A_LIN * 0.10))


def test_curvatura_adiante_ve_a_curva_ANTES_de_entrar_nela():
    """Olhar só onde o robô está faz ele chegar rápido demais na curva.

    A curvatura é medida na janela do lookahead à frente, não no ponto atual:
    frear em cima da curva é frear tarde, e a lei de frenagem da 005 existe
    justamente porque a máquina tem distância de frenagem.
    """
    pts = reta(10, 0.1) + [(1.0 + 0.46 * math.sin(a), 0.46 * (1 - math.cos(a)))
                           for a in [math.radians(g) for g in range(5, 95, 5)]]
    # robô no começo da reta, curva de 0,46 m começando a 1,0 m dali
    r = curvatura_adiante(pts, 0, janela=1.5)
    assert r < 0.7, f'raio visto {r:.2f} — a curva à frente passou despercebida'


def test_curvatura_adiante_em_reta_e_infinita():
    assert math.isinf(curvatura_adiante(reta(20, 0.1), 0, janela=1.0))
