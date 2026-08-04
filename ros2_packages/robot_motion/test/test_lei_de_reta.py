"""A malha de reta (decisão 011) tem de segurar o rumo num robô que arca.

O caso de teste central não é unitário, é de FECHAMENTO: uma planta de
brinquedo com o defeito medido do robô (curvatura de viés que a malha não
conhece com exatidão) tem de sair andando reto. Se só os testes de sinal
passassem, a malha poderia estar estável e inútil.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'robot_motion'))

from lei_de_reta import MalhaDeReta, norm_ang  # noqa: E402


# ------------------------------------------------------------ o feedforward

def test_ff_de_frente_corrige_para_a_esquerda():
    """O robô arca para a DIREITA de frente (curv −0,817), então a correção
    tem de ser wz POSITIVO (esquerda) — e proporcional ao v comandado."""
    m = MalhaDeReta(ki=0.0, kp=0.0)
    wz = m.passo(v_cmd=0.25, wz_cmd=0.0, yaw=0.0, dt=0.1)
    assert wz == pytest.approx(0.817 * 0.25, rel=1e-6)


def test_ff_de_re_corrige_para_o_mesmo_lado_e_menos():
    """De ré o arco é do MESMO lado do corpo e 8x menor — o ff acompanha.
    Foi um sinal trocado aqui (na placa) que a aceitação de 04-08 pegou."""
    m = MalhaDeReta(ki=0.0, kp=0.0)
    frente = m.passo(0.25, 0.0, 0.0, 0.1)
    m2 = MalhaDeReta(ki=0.0, kp=0.0)
    re = m2.passo(-0.25, 0.0, 0.0, 0.1)
    assert re == pytest.approx(0.098 * 0.25, rel=1e-6)
    assert frente > re > 0.0


# ------------------------------------------------------- captura e descarte

def test_captura_o_rumo_na_entrada_da_reta_e_segura():
    """A referência é o yaw do INSTANTE em que a reta começou. Se o robô
    derivou para −0,1 rad, o erro é +0,1 e a correção cresce."""
    m = MalhaDeReta(kp=1.0, ki=0.0, curv_frente=0.0)
    assert m.passo(0.25, 0.0, yaw=0.3, dt=0.1) == pytest.approx(0.0)
    assert m.rumo_ref == pytest.approx(0.3)
    wz = m.passo(0.25, 0.0, yaw=0.2, dt=0.1)
    assert wz == pytest.approx(0.1, abs=0.02)


def test_curva_pedida_passa_intocada_e_rearma():
    """Curva é assunto do comandante. E depois dela a referência velha tem de
    morrer — o rumo novo é o que valer quando a reta voltar."""
    m = MalhaDeReta()
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    assert m.rumo_ref is not None
    assert m.passo(0.25, 0.5, yaw=0.7, dt=0.1) == 0.5
    assert m.rumo_ref is None
    m.passo(0.25, 0.0, yaw=1.5, dt=0.1)
    assert m.rumo_ref == pytest.approx(1.5)


def test_parado_nao_segura_rumo_nenhum():
    """Robô parado pode ser girado no chão pelo dono. Referência velha nesse
    caso é pior que nenhuma: mandaria o robô 'de volta' a um rumo morto."""
    m = MalhaDeReta()
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    m.passo(0.0, 0.0, yaw=0.0, dt=0.1)
    assert m.rumo_ref is None and m.integral == 0.0


def test_troca_de_sentido_zera_o_integrador():
    """O viés da frente não é o da ré. Integrador carregado atravessando a
    troca viraria chicote — o robô sairia da ré já esterçando errado."""
    m = MalhaDeReta(kp=0.0, ki=1.0, curv_frente=0.0, curv_re=0.0)
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    for _ in range(20):
        m.passo(0.25, 0.0, yaw=-0.2, dt=0.1)
    assert m.integral > 0.0
    m.passo(-0.25, 0.0, yaw=-0.2, dt=0.1)
    assert m.integral == pytest.approx(0.0, abs=1e-6)


# ----------------------------------------------------------------- os grampos

def test_integrador_tem_teto():
    """Anti-windup: erro que persiste (robô travado, BO-3) não pode encher o
    integrador sem limite — na soltura ele descarregaria tudo de uma vez."""
    m = MalhaDeReta(int_max=0.6)
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    for _ in range(1000):
        m.passo(0.25, 0.0, yaw=-1.0, dt=0.1)
    assert m.integral <= 0.6 + 1e-9


def test_saida_tem_teto_proprio():
    """A correção de regime é ~0,25 rad/s; o grampo é do transitório. Sem
    ele, erro grande na captura pediria giro que o robô transforma em arco
    fechado — o oposto de andar reto."""
    m = MalhaDeReta(wz_max=0.6, kp=10.0)
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    wz = m.passo(0.25, 0.0, yaw=-3.0, dt=0.1)
    assert abs(wz) <= 0.6 + 1e-9


def test_dt_nao_positivo_nao_envenena_o_integrador():
    """Relógio de simulação pode repetir carimbo; a primeira amostra não tem
    dt. Nenhum dos dois pode somar no integrador."""
    m = MalhaDeReta(kp=0.0, ki=1.0, curv_frente=0.0)
    m.passo(0.25, 0.0, yaw=0.0, dt=0.0)
    m.passo(0.25, 0.0, yaw=-0.5, dt=0.0)
    m.passo(0.25, 0.0, yaw=-0.5, dt=-0.1)
    assert m.integral == 0.0


# ------------------------------------------------- o fechamento (o que vale)

def _roda_planta(malha, curv_planta, v_cmd, passos=600, dt=0.02, atraso=13):
    """Planta de brinquedo com o defeito medido: a curvatura ENTREGUE é a
    comandada mais um viés que a malha não conhece — e o wz age com atraso
    (13 passos de 20 ms ≈ 0,27 s, a latência medida do atuador).

    Devolve (curvatura de regime, erro de rumo final). A curvatura é
    calculada como a bancada calcula (giro acumulado / caminho, 2ª metade);
    o erro de rumo é contra o yaw capturado na largada (aqui, 0).
    """
    yaw, x, y = 0.0, 0.0, 0.0
    fila = [0.0] * atraso
    caminho, giro = 0.0, 0.0
    metade = passos // 2
    for i in range(passos):
        wz_cmd = malha.passo(v_cmd, 0.0, yaw, dt)
        fila.append(wz_cmd)
        wz_agindo = fila.pop(0)
        v_real = math.copysign(0.30, v_cmd)          # patamar: v não obedece
        curv_entregue = wz_agindo / abs(v_real) + curv_planta
        dyaw = curv_entregue * abs(v_real) * dt
        if i >= metade:
            giro += dyaw
            caminho += abs(v_real) * dt
        yaw = norm_ang(yaw + dyaw)
        x += v_real * math.cos(yaw) * dt
        y += v_real * math.sin(yaw) * dt
    return giro / caminho, norm_ang(0.0 - yaw)


def test_fecha_a_reta_de_frente_no_criterio_da_011():
    """O teste que importa: planta com o viés MEDIDO (−0,817), malha com os
    ganhos de fábrica → |curvatura| < 0,05 1/m em regime (critério da 011),
    e o rumo assentado em cima da referência (< 1°)."""
    c, e = _roda_planta(MalhaDeReta(), curv_planta=-0.817, v_cmd=0.25)
    assert abs(c) < 0.05, f'curvatura de regime {c:.3f}'
    assert abs(math.degrees(e)) < 1.0, f'rumo assentou {math.degrees(e):.1f}° torto'


def test_fecha_a_re_no_criterio_da_011():
    c, e = _roda_planta(MalhaDeReta(), curv_planta=-0.098, v_cmd=-0.25)
    assert abs(c) < 0.05, f'curvatura de regime {c:.3f}'
    assert abs(math.degrees(e)) < 1.0, f'rumo assentou {math.degrees(e):.1f}° torto'


def test_fecha_mesmo_com_ff_errado_25_por_cento():
    """A dispersão do robô é 21% entre corridas idênticas. O integrador tem
    de comer a diferença entre o ff (calibrado num dia) e a planta (de outro
    dia): reta E rumo em cima, mesmo com o ff 25% otimista."""
    m = MalhaDeReta()                       # ff acha que é −0,817
    c, e = _roda_planta(m, curv_planta=-1.02, v_cmd=0.25)   # planta 25% pior
    assert abs(c) < 0.05, f'curvatura de regime {c:.3f}'
    assert abs(math.degrees(e)) < 1.0, f'rumo assentou {math.degrees(e):.1f}° torto'


def test_sem_integrador_o_rumo_assenta_torto():
    """O papel EXATO do Ki, descoberto por este próprio teste na primeira
    versão: sem ele o P ainda ZERA a curvatura (o robô anda reto!), mas
    andando ~6° fora do rumo capturado — o erro constante que o P precisa
    manter para sustentar a correção. O integrador existe para zerar o RUMO,
    não a curvatura. Com Ki o mesmo caso assenta < 1° (teste acima)."""
    m = MalhaDeReta(ki=0.0)
    c, e = _roda_planta(m, curv_planta=-1.02, v_cmd=0.25)
    assert abs(c) < 0.05, 'a curvatura fecha até sem Ki — não é ela que o justifica'
    assert abs(math.degrees(e)) > 3.0, \
        f'sem Ki o rumo deveria assentar torto, deu {math.degrees(e):.1f}°'
