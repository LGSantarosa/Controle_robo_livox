"""A placa fingida tem de reproduzir o atuador MEDIDO em 31-07, não um ideal.

Estes testes existem porque o modelo antigo ("zona morta em m/s, engole comando
pequeno") descrevia um robô que não é este. O robô real tem uma compensação de
zona morta DENTRO do driver que multiplica as duas rodas até a maior vencer o
deadband — preserva a curva e destrói a magnitude. O simulador que não tiver
isso ajusta a movimentação contra uma máquina que não existe.

Fonte dos números: `docs/MODELO_ROBO2.md` e a 4ª leva de 31-07 no `docs/DIARIO.md`.
"""
import ast
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


def _defaults_do_no():
    """Lê os defaults do `declare_parameters` do nó, direto do fonte.

    Antes esta lista era copiada à mão aqui. Copiada, ela DERRAPA: em 04-08 os
    valores de assimetria mudaram no nó, o dublê seguiu com os antigos e os
    testes passariam descrevendo um robô que o simulador já não é. Extrair do
    fonte é o que garante que o que se testa é o que roda.
    """
    arvore = ast.parse(open(FONTE).read())
    for no in ast.walk(arvore):
        if (isinstance(no, ast.Call)
                and getattr(no.func, 'attr', None) == 'declare_parameters'):
            lista = no.args[1]
            return {ast.literal_eval(t.elts[0]): ast.literal_eval(t.elts[1])
                    for t in lista.elts}
    raise AssertionError('declare_parameters não encontrado em ' + FONTE)


PADRAO = _defaults_do_no()


class Placa:
    """Dublê com os parâmetros de fábrica do nó, sem o nó."""

    def __init__(self, **kw):
        self.par = dict(PADRAO)
        self.par.update(kw)

    unidades = placa_mod.PlacaSimulada.unidades
    assimetria = placa_mod.PlacaSimulada.assimetria
    patamar = placa_mod.PlacaSimulada.patamar
    faixa_do_patamar = placa_mod.PlacaSimulada.faixa_do_patamar
    medido = placa_mod.PlacaSimulada.medido
    cru = placa_mod.PlacaSimulada.cru


def borda(p, v, wz=0.0):
    """Velocidade linear resultante, sem a assimetria (que é ganho à parte)."""
    meia = p.par['bitola'] / 2.0
    ve, vd, _ = p.medido(v - wz * meia, v + wz * meia)
    return (ve + vd) / 2.0


def curvatura(p, v_cmd):
    """Curvatura de corpo [1/m] que a placa entrega numa reta comandada.

    ⚠️ **`wz / |v|`, não `wz / v`.** É a convenção da bancada: giro por metro
    PERCORRIDO, com o caminho sem sinal, que é o que o `medir.py curvatura`
    calcula a partir do CSV. As duas convenções concordam de frente e dão
    sinais OPOSTOS na ré — e foi exatamente aí que a primeira tentativa errou,
    com o teste passando verde porque ele usava a convenção errada. Se este
    helper divergir do `medir.py`, o simulador pode arcar para o lado errado
    com a suíte inteira verde.
    """
    ve, vd, _ = p.medido(v_cmd, v_cmd)          # wz=0 -> as duas iguais
    ve *= (1.0 + p.assimetria(ve + vd))
    v = (ve + vd) / 2.0
    wz = (vd - ve) / p.par['bitola']
    return wz / abs(v)


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
    """Derivada, não escrita: positiva de frente, NEGATIVA de ré. Os dois
    sinais juntos é que fazem o corpo girar para o mesmo lado nos dois
    sentidos, que foi o que o robô fez nas quatro corridas do par matched."""
    p = Placa()
    assert p.assimetria(1.0) > 0.0, 'de frente a esquerda entrega MAIS'
    assert p.assimetria(-1.0) < 0.0, 'de ré ela entrega MENOS — não é typo'
    assert abs(p.assimetria(1.0)) > abs(p.assimetria(-1.0))


# ------------------------------------------- o arco do corpo medido em 04-08
#
# `rendimento_giro=1.0` nestes: aqui se testa o MODELO contra o robô. A
# compensação da derrapagem do Gazebo é outro assunto, com teste próprio
# abaixo, e quem a verifica de verdade é a corrida de aceitação no simulador.

def test_o_arco_de_frente_bate_com_o_robo():
    """Alvo: −0,817 1/m (raio 1,22 m), n=2 matched com controle de piso.
    Recomputado do CSV cru dá −0,838; a diferença é de janela de amostras e
    cabe folgada na dispersão medida (21%, faixa −0,73 a −0,90)."""
    p = Placa(rendimento_giro=1.0)
    assert curvatura(p, 0.25) == pytest.approx(-0.817, abs=0.02)


def test_o_arco_de_re_bate_com_o_robo():
    """Alvo: −0,098 1/m (raio 10,19 m). A ré NÃO é reta — desvia 8,3x menos,
    e essa parcela sobrevive aos dois sentidos: é o que consertar a boba
    deixaria para trás, e o que o seguidor tem de fechar em malha fechada."""
    p = Placa(rendimento_giro=1.0)
    assert curvatura(p, -0.25) == pytest.approx(-0.098, abs=0.02)


def test_a_razao_frente_re_e_a_assinatura_da_boba():
    """O número que separa "robô que arca" de "robô torto": ~8x. Um desvio de
    motor ou placa fraca daria a MESMA curvatura nos dois sentidos (razão 1).
    Antes de 04-08 este modelo dava ré perfeitamente reta — razão infinita."""
    p = Placa(rendimento_giro=1.0)
    razao = curvatura(p, 0.25) / curvatura(p, -0.25)
    assert razao == pytest.approx(8.3, abs=1.0)


def test_o_arco_nao_troca_de_sinal_com_o_sentido():
    """O teste de piso de 04-08, virado para dentro do modelo: as duas
    curvaturas têm de ser NEGATIVAS. Caimento de piso é força fixa no mundo e
    trocaria o sinal ao girar o corpo 180°; no robô não trocou, quatro vezes.

    Este é o teste que a primeira tentativa não tinha e que teria pego o erro:
    lá a ré saiu POSITIVA, e só a corrida no Gazebo denunciou.
    """
    for rend in (1.0, 0.80):
        p = Placa(rendimento_giro=rend)
        assert curvatura(p, 0.25) < 0.0, f'frente, rendimento {rend}'
        assert curvatura(p, -0.25) < 0.0, f'ré, rendimento {rend}'


def test_o_arco_nao_depende_do_modulo_do_comando():
    """Consequência do patamar: dentro dele todo comando vira a mesma coisa na
    placa, então a curvatura também é a mesma. Bate com o robô, que arcou igual
    comandado a 0,25 m/s em corridas de comprimento diferente."""
    p = Placa()
    cs = [curvatura(p, v) for v in (0.05, 0.10, 0.25, 0.50, 0.80)]
    assert max(cs) - min(cs) < 1e-9, cs


def test_a_derrapagem_do_gazebo_e_compensada_pedindo_mais():
    """O Gazebo entrega ~80% do giro pedido (29-07, confirmado em 04-08). Para
    o CORPO simulado sair no número do robô, a placa tem de pedir mais — e na
    proporção certa, senão o simulador fica parecido com o robô por acaso."""
    alvo, rend = -0.817, 0.80
    pedido = curvatura(Placa(rendimento_giro=rend), 0.25)
    assert pedido == pytest.approx(alvo / rend, abs=0.02)
    assert abs(pedido) > abs(alvo), 'compensar é pedir MAIS, não menos'


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
