"""A placa fingida tem de reproduzir o atuador MEDIDO em 31-07, não um ideal.

Estes testes existem porque o modelo antigo ("zona morta em m/s, engole comando
pequeno") descrevia um robô que não é este. O robô real tem uma compensação de
zona morta DENTRO do driver que multiplica as duas rodas até a maior vencer o
deadband — preserva a curva e destrói a magnitude. O simulador que não tiver
isso ajusta a movimentação contra uma máquina que não existe.

Fonte dos números: `docs/MODELO_ROBO2.md` e a 4ª leva de 31-07 no `docs/DIARIO.md`.
"""
import importlib.util
import os
import sys
import types

import pytest

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FONTE = os.path.join(RAIZ, 'robot_base', 'placa_simulada.py')


def _carrega():
    """Importa o nó sem subir ROS: só a matemática interessa aqui."""
    for nome in ('rclpy', 'rclpy.node', 'rclpy.qos', 'geometry_msgs',
                 'geometry_msgs.msg'):
        sys.modules.setdefault(nome, types.ModuleType(nome))
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].QoSProfile = object
    sys.modules['rclpy.qos'].ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    sys.modules['geometry_msgs.msg'].TwistStamped = object
    spec = importlib.util.spec_from_file_location('placa_simulada', FONTE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


placa_mod = _carrega()


class Placa:
    """Dublê com os parâmetros de fábrica do nó, sem o nó."""

    def __init__(self, **kw):
        self.par = dict(
            modelo='medido', deadband_speed=100.0, escala_driver=0.10472,
            escala_real=0.0372, latencia=0.27, assimetria_frente=0.12,
            assimetria_re=0.0, zona_morta_crua=0.30, bitola=0.270, raio=0.080,
            taxa_avisos=2.0)
        self.par.update(kw)

    unidades = placa_mod.PlacaSimulada.unidades
    patamar = placa_mod.PlacaSimulada.patamar
    faixa_do_patamar = placa_mod.PlacaSimulada.faixa_do_patamar
    medido = placa_mod.PlacaSimulada.medido
    cru = placa_mod.PlacaSimulada.cru


def borda(p, v, wz=0.0):
    """Velocidade linear resultante, sem a assimetria (que é ganho à parte)."""
    meia = p.par['bitola'] / 2.0
    ve, vd, _ = p.medido(v - wz * meia, v + wz * meia)
    return (ve + vd) / 2.0


# ------------------------------------------------------------- o patamar

def test_tres_comandos_diferentes_dao_a_mesma_velocidade():
    """O defeito central deste robô, medido: 0,10 e 0,25 m/s de comando deram a
    MESMA velocidade de roda. Se o simulador não reproduzir isto, a movimentação
    é ajustada contra um atuador que não existe."""
    p = Placa()
    vs = [borda(p, v) for v in (0.05, 0.10, 0.25, 0.50, 0.80)]
    assert max(vs) - min(vs) < 1e-9, vs
    assert vs[0] == pytest.approx(0.298, abs=0.005)


def test_o_patamar_cobre_a_faixa_util_inteira():
    """De ~0,008 a ~0,838 m/s. O teto do robô é 1,0, então quase tudo que a
    navegação pede cai dentro — é por isso que o defeito é central e não de
    borda."""
    p = Placa()
    piso, teto = p.faixa_do_patamar()
    assert piso == pytest.approx(0.0084, abs=0.001)
    assert teto == pytest.approx(0.838, abs=0.005)


def test_abaixo_do_piso_nao_anda():
    """`mx <= 1 unidade` é onde o driver deixa de compensar. É este o limiar que
    a bancada mediu (0,095 rad/s no giro) — aritmética do driver, NÃO atrito."""
    p = Placa()
    assert borda(p, 0.004) == 0.0
    assert borda(p, 0.012) > 0.0


def test_acima_do_patamar_o_comando_volta_a_valer():
    """Acima de `deadband_speed` a compensação não age e o comando passa
    proporcional — na escala real, que segue 2,8x menor que a suposta.
    NÃO TESTADO no robô: é extrapolação do código, e está anotado como tal."""
    p = Placa()
    a, b = borda(p, 1.0), borda(p, 2.0)
    assert b == pytest.approx(2 * a, rel=1e-6)


# ------------------------------------------------- a escala que o driver erra

def test_a_escala_real_do_firmware_nao_e_a_que_o_driver_supoe():
    """Medido: 100 unidades entregam 3,72 rad/s de roda, não os 10,47 que o
    driver calcula. É por isso que o robô sempre andou mais rápido do que se
    pediu — e o fator é o mesmo 2,8x visto no patamar."""
    p = Placa()
    assert p.par['escala_driver'] / p.par['escala_real'] == pytest.approx(2.8, abs=0.05)
    assert p.patamar() == pytest.approx(0.298, abs=0.005)


# ------------------------------------------------------------- a assimetria

def test_a_assimetria_e_dependente_de_sentido():
    """Medido (n=2 por sentido): indo à frente a esquerda entrega 11–13% a mais;
    de ré elas empatam. É por isso que o robô puxa para a direita só de frente —
    e é o que mantém a boba como candidata a causa."""
    p = Placa()
    assert p.par['assimetria_frente'] == pytest.approx(0.12, abs=0.02)
    assert p.par['assimetria_re'] == pytest.approx(0.0, abs=0.02)


# ------------------------------------------------------------- os regimes

def test_placa_crua_devolve_proporcionalidade_e_zona_morta_de_verdade():
    """Com a compensação desligada some o patamar e aparece a zona morta física
    — que é o que o banco precisa medir. O valor é CHUTE (só bracketado entre
    0,25 e 0,5), e o teste trava o comportamento, não o número."""
    p = Placa(modelo='cru')
    meia = p.par['bitola'] / 2.0
    ve, vd, engoliu = p.cru(0.10 - 0.0, 0.10 + 0.0)
    assert (ve, vd) == (0.0, 0.0) and engoliu, 'comando pequeno tem de ser engolido'
    ve, vd, engoliu = p.cru(2.0, 2.0)
    assert not engoliu and ve > 0
    ve2, _, _ = p.cru(4.0, 4.0)
    assert ve2 == pytest.approx(2 * ve, rel=1e-6), 'acima do limiar, proporcional'


# --------------------------------------------- a consequência para a 005

def test_a_lei_de_frenagem_da_005_degenera_em_liga_desliga():
    """A consequência que motivou tudo isto. A lei

        wz = sinal(e)·min(wz_max, √(2·a_dec·|e|))

    produz um contínuo de valores. Passados pelo atuador medido, TODOS viram o
    mesmo giro — de 1° a 180° de erro de rumo. A frenagem de rumo da decisão 005
    não existe neste robô enquanto a compensação estiver ligada.
    """
    import math
    p = Placa()
    meia = p.par['bitola'] / 2.0
    entregues = set()
    for graus in (1, 2, 5, 10, 20, 45, 90, 180):
        e = math.radians(graus)
        wz = min(1.0, math.sqrt(2 * 3.05 * e))
        ve, vd, _ = p.medido(-wz * meia, wz * meia)
        entregues.add(round((vd - ve) / p.par['bitola'], 4))
    assert len(entregues) == 1, f'a lei tinha de colapsar num valor só: {entregues}'
