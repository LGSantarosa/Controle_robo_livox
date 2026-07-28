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
    orcamento_de_re_estourado,
    precisa_recuar,
    raio_necessario,
    raio_minimo_de_chegada,
    rumo_para,
    velocidade_de_aproximacao,
    velocidade_que_a_curva_permite,
)

V_MAX = 0.5
A_LIN = 0.5
V_MIN = 0.20
RAIO = 0.15
WZ_UTIL = 0.5


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
    ch, _, v = comando_de_navegacao(0.0, 0.0, 0.0, 0.10, 0.0, V_MAX, A_LIN, V_MIN, RAIO, WZ_UTIL)
    assert ch is True
    assert v == 0.0


def test_fora_do_raio_ainda_anda():
    ch, _, v = comando_de_navegacao(0.0, 0.0, 0.0, 2.0, 0.0, V_MAX, A_LIN, V_MIN, RAIO, WZ_UTIL)
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
    ch, rumo, v = comando_de_navegacao(0.0, 0.0, 0.0, -2.0, 0.0, V_MAX, A_LIN,
                                       V_MIN, RAIO, WZ_UTIL)
    assert ch is False
    assert abs(rumo) == pytest.approx(math.pi)
    assert v > 0.0


# ------------------------------------------------- não orbitar o alvo

def test_perto_e_de_lado_a_curva_limita_a_velocidade():
    """O defeito que o dono encontrou clicando no RViz.

    Alvo a 0,65 m e ~64° de erro de rumo: ir a 0,5 m/s exige girar a
    v·sen(e)/d = 0,69 rad/s só para manter o bico no alvo. Se a máquina não
    sustenta isso, o ponto escapa pelo lado e o robô orbita — medido, 0,365 m
    de raio de órbita, indefinidamente.
    """
    teto = velocidade_que_a_curva_permite(0.65, math.radians(64), WZ_UTIL)
    assert teto < V_MAX, 'a curva tem que limitar a velocidade aqui'
    # e o giro que sobra é sustentável
    assert teto * math.sin(math.radians(64)) / 0.65 <= WZ_UTIL + 1e-9


def test_apontado_para_o_alvo_a_curva_nao_limita():
    assert velocidade_que_a_curva_permite(0.65, 0.0, WZ_UTIL) == float('inf')


def test_quanto_mais_perto_mais_devagar():
    """É a distância que aperta a curva: perto do alvo, tem que ir devagar."""
    e = math.radians(60)
    tetos = [velocidade_que_a_curva_permite(d, e, WZ_UTIL)
             for d in [2.0, 1.0, 0.5, 0.25]]
    assert tetos == sorted(tetos, reverse=True)


def test_velocidade_final_respeita_os_dois_tetos():
    """Frenagem e curva limitam juntas; vale o menor."""
    _, _, v = comando_de_navegacao(0.0, 0.0, 0.0, 0.28, 0.59, V_MAX, A_LIN,
                                   0.0, RAIO, WZ_UTIL)
    d = math.hypot(0.28, 0.59)
    erro = math.atan2(0.59, 0.28)
    assert v <= velocidade_de_aproximacao(d, V_MAX, A_LIN, 0.0) + 1e-9
    assert v <= velocidade_que_a_curva_permite(d, erro, WZ_UTIL) + 1e-9


# ------------------------------------------------- a ré como manobra

RAIO_MIN = V_MIN / WZ_UTIL          # 0,40 m: o círculo mais fechado que ele faz


def test_raio_necessario_e_a_geometria_da_perseguicao():
    """Ir a um ponto a `d` com erro `e` exige raio `d/(2·sen e)`.

    É a corda do círculo: o robô e o alvo estão os dois sobre ele, separados
    por `d`, com o bico `e` fora da corda.
    """
    assert raio_necessario(1.0, math.pi / 2) == pytest.approx(0.5)
    assert raio_necessario(1.0, math.radians(30)) == pytest.approx(1.0)
    assert raio_necessario(1.0, 0.0) == float('inf')


def test_alvo_muito_perto_e_de_lado_nao_cabe_na_curva():
    """O caso que o dono achou clicando: 0,65 m de lado, e o robô orbitou.

    Sem pivô, o robô não fecha curva mais apertada que `raio_min`. Um ponto
    que exige menos que isso está DENTRO do círculo que ele descreve — e
    círculo que se persegue por dentro nunca se alcança.
    """
    assert precisa_recuar(dist=0.168, erro_rumo=math.pi / 2,
                          raio_min_curva=RAIO_MIN, recuando=False)


def test_alvo_longe_ou_alinhado_nao_pede_re():
    assert not precisa_recuar(dist=3.0, erro_rumo=math.radians(20),
                              raio_min_curva=RAIO_MIN, recuando=False)
    assert not precisa_recuar(dist=0.5, erro_rumo=0.0,
                              raio_min_curva=RAIO_MIN, recuando=False)


def test_quem_pivota_nunca_precisa_de_re():
    """`raio_min_curva = 0` é o robô que vira no próprio eixo.

    Quando a bitola medida permitir o pivô, a ré some sozinha do
    comportamento — sem trocar código, só o parâmetro.
    """
    for d in [0.05, 0.168, 0.5]:
        assert not precisa_recuar(dist=d, erro_rumo=math.pi / 2,
                                  raio_min_curva=0.0, recuando=False)


def test_histerese_impede_tremer_na_fronteira():
    """Sair da ré exige folga; entrar não.

    Sem isso, no ponto exato em que o alvo passa a caber, o robô alterna ré e
    avanço a cada ciclo e não sai do lugar.
    """
    # distância em que o alvo cabe raspando, de lado
    d = 2.0 * RAIO_MIN
    assert not precisa_recuar(d * 1.01, math.pi / 2, RAIO_MIN, recuando=False)
    # já recuando, esse mesmo ponto ainda não é suficiente: falta folga
    assert precisa_recuar(d * 1.01, math.pi / 2, RAIO_MIN, recuando=True)
    # com folga sobrando, solta
    assert not precisa_recuar(d * 1.5, math.pi / 2, RAIO_MIN, recuando=True)


def test_recuar_afasta_o_alvo_e_resolve_a_geometria():
    """A manobra funciona: recuando em linha reta o ponto passa a caber.

    Robô na origem apontando para +x, alvo a 0,20 m em +y (de lado, dentro do
    círculo). Recuando, a distância cresce mais rápido do que o ângulo
    atrapalha, e em algum ponto o alvo cabe.
    """
    alvo = (0.0, 0.20)
    coube = None
    for recuo in [0.1 * i for i in range(1, 30)]:
        x = -recuo
        d = math.hypot(alvo[0] - x, alvo[1])
        erro = math.atan2(alvo[1], alvo[0] - x)     # bico segue em +x
        if not precisa_recuar(d, erro, RAIO_MIN, recuando=True):
            coube = recuo
            break
    assert coube is not None, 'recuar nunca resolveu — a manobra não serve'
    assert coube <= 1.0, f'precisou de {coube:.2f} m, mais que o orçamento'


def test_orcamento_de_re_e_finito():
    """Recuar para sempre é pior que não alcançar: o robô some do laboratório."""
    assert not orcamento_de_re_estourado(recuou=0.3, t_recuando=2.0,
                                         re_max_dist=1.0, re_max_s=8.0)
    assert orcamento_de_re_estourado(recuou=1.2, t_recuando=2.0,
                                     re_max_dist=1.0, re_max_s=8.0)
    assert orcamento_de_re_estourado(recuou=0.3, t_recuando=9.0,
                                     re_max_dist=1.0, re_max_s=8.0)
