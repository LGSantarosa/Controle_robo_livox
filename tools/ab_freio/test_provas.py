"""Trava do desenho do A/B — as poses e a ordem, antes de qualquer Gazebo.

Desenho errado custa uma sessão inteira de seis subidas para depois descobrir
que as metades não eram comparáveis. Aqui ele é dado, e o dado é conferido.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import provas  # noqa: E402


def test_sao_seis_tentativas_na_ordem_a_b_a():
    t = provas.tentativas()
    assert len(t) == 6
    assert [x['rotulo'] for x in t] == ['A', 'A', 'B', 'B', "A'", "A'"]


def test_o_a_linha_repete_a_condicao_do_a():
    """O controle temporal só vale se A′ for MESMO a condição do A."""
    t = provas.tentativas()
    a = [x for x in t if x['rotulo'] == 'A']
    al = [x for x in t if x['rotulo'] == "A'"]
    assert all(x['freio_linear'] is True for x in a + al)
    for prova in ('chegada', 'porta'):
        ua = next(x for x in a if x['prova'] == prova)
        ub = next(x for x in al if x['prova'] == prova)
        assert ua['pose'] == ub['pose'], prova
        assert ua['goal'] == ub['goal'], prova


def test_o_b_e_o_unico_com_o_freio_desligado():
    t = provas.tentativas()
    desligados = [x['rotulo'] for x in t if x['freio_linear'] is False]
    assert desligados == ['B', 'B']


def test_cada_prova_tem_a_mesma_pose_nas_tres_tentativas():
    """⚠️ Idêntica DENTRO de cada prova — não uma pose comum às seis."""
    t = provas.tentativas()
    for prova in ('chegada', 'porta'):
        poses = [x['pose'] for x in t if x['prova'] == prova]
        assert len(poses) == 3, prova
        assert all(p == poses[0] for p in poses), prova
    # e as duas provas partem de lugares DIFERENTES, senão a da porta não
    # estaria começando perto da porta
    chegada = next(x['pose'] for x in t if x['prova'] == 'chegada')
    porta = next(x['pose'] for x in t if x['prova'] == 'porta')
    assert chegada != porta


def test_a_prova_da_porta_nasce_perto_da_porta():
    """A porta é o aperto em x ≈ 8,9…9,1, e o goal da volta a atravessa.

    Nascer em (2,0; 5,0) faria o robô refazer a ida inteira antes de chegar
    nela — e a pose de entrada voltaria a depender do caminho.
    """
    porta = provas.PROVAS['porta']
    assert porta['pose']['x'] > 10.0, porta['pose']
    # o goal está do outro lado do aperto
    assert porta['goal']['x'] < 3.0, porta['goal']


def test_os_quaternions_batem_com_os_yaws():
    """O executor manda quaternion; o yaw é só para registro e para a launch.

    Se os dois discordarem, o objetivo enviado não é o que o protocolo diz.
    """
    for nome, g in (('ida', provas.GOAL_IDA), ('volta', provas.GOAL_VOLTA)):
        yaw = 2 * math.atan2(g['qz'], g['qw'])
        assert yaw == __import__('pytest').approx(g['yaw'], abs=1e-9), nome


def test_os_quaternions_sao_unitarios():
    for nome, g in (('ida', provas.GOAL_IDA), ('volta', provas.GOAL_VOLTA)):
        assert (g['qz'] ** 2 + g['qw'] ** 2) == __import__('pytest').approx(
            1.0, abs=1e-9), nome


def test_os_identificadores_sao_unicos_e_ordenados():
    """Nunca sobrescrever: cada tentativa tem pasta própria."""
    ids = [x['id'] for x in provas.tentativas()]
    assert len(set(ids)) == 6
    assert ids == sorted(ids), ids
    assert "'" not in ''.join(ids), 'apóstrofo em nome de pasta dá dor de cabeça'


def test_as_tolerancias_estao_declaradas_antes():
    assert provas.TOL_POSE_INICIAL_M > 0
    assert provas.TOL_YAW_INICIAL_RAD > 0
    assert provas.TOLERANCIA_GOAL_ESPERADA == 0.25
