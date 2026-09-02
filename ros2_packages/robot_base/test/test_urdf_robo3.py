"""Trava a geometria do modelo do robô 3.

Mesmo espírito do `test_urdf_robo2.py`: não confere estilo, confere o que, se
derivar, faz o simulador mentir em silêncio. O que muda aqui é o que a máquina
mudou — e cada teste abaixo guarda uma decisão que custou uma leva de medidas.

As medidas e a discussão de cada número estão em `docs/ROBO3_REVISAO_CRUZADA.md`.
"""

import math
import os
import xml.etree.ElementTree as ET

import pytest

try:
    import xacro
except ImportError:  # pragma: no cover
    xacro = None

import yaml

AQUI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(AQUI, 'description', 'robo3.urdf.xacro')
YAML_SIM = os.path.join(AQUI, 'config', 'hoverboard_controllers_sim_robo3.yaml')

pytestmark = pytest.mark.skipif(xacro is None, reason='xacro não disponível')

TOL = 1e-6


@pytest.fixture(scope='module')
def urdf():
    return ET.fromstring(xacro.process_file(XACRO, mappings={'sim': 'false'}).toxml())


@pytest.fixture(scope='module')
def params():
    with open(YAML_SIM) as f:
        return yaml.safe_load(f)['hoverboard_base_controller']['ros__parameters']


def _junta(urdf, nome):
    for j in urdf.findall('joint'):
        if j.get('name') == nome:
            return j
    raise AssertionError(f'junta {nome} não existe')


def _xyz(elem):
    o = elem.find('origin')
    if o is None or o.get('xyz') is None:
        return (0.0, 0.0, 0.0)
    return tuple(float(v) for v in o.get('xyz').split())


def _cilindro(urdf, link_nome, tag='collision'):
    for l in urdf.findall('link'):
        if l.get('name') == link_nome:
            cil = l.find(f'./{tag}/geometry/cylinder')
            assert cil is not None, f'{link_nome} não tem cilindro em {tag}'
            return float(cil.get('radius')), float(cil.get('length'))
    raise AssertionError(f'link {link_nome} não existe')


def _caixa(urdf, link_nome='base_link'):
    for l in urdf.findall('link'):
        if l.get('name') == link_nome:
            box = l.find('./collision/geometry/box')
            origem = tuple(float(v) for v in l.find('./collision/origin').get('xyz').split())
            return tuple(float(v) for v in box.get('size').split()), origem
    raise AssertionError('base_link sem caixa')


# ---------------------------------------------------------------- apoio no chão

@pytest.mark.parametrize('lado', ['left', 'right'])
def test_roda_motriz_toca_o_chao(urdf, lado):
    """Junta a `raio` do chão. Fora disso o robô nasce pulando ou afundado."""
    r, _ = _cilindro(urdf, f'{lado}_wheel')
    assert abs(_xyz(_junta(urdf, f'{lado}_wheel_joint'))[2] - r) < TOL


@pytest.mark.parametrize('lado', ['left', 'right'])
def test_boba_toca_o_chao(urdf, lado):
    pivo_z = _xyz(_junta(urdf, f'{lado}_caster_swivel_joint'))[2]
    queda = _xyz(_junta(urdf, f'{lado}_caster_wheel_joint'))[2]
    r, _ = _cilindro(urdf, f'{lado}_caster_wheel')
    assert abs((pivo_z + queda) - r) < TOL


def test_sao_QUATRO_apoios(urdf):
    """Duas motrizes e DUAS bobas — o robô 2 tinha três apoios, este tem quatro.

    É esta contagem que torna o apoio hiperestático (C3): com quatro pontos num
    chassi rígido e sem mola, uma boba sai do chão em piso irregular e o corpo
    balança entre as diagonais. Se alguém reduzir para três, o modelo deixa de
    reproduzir o defeito que o robô tem.
    """
    contatos = [l.get('name') for l in urdf.findall('link')
                if l.find('./collision/geometry/cylinder') is not None]
    assert sorted(contatos) == sorted([
        'left_wheel', 'right_wheel', 'left_caster_wheel', 'right_caster_wheel'])


# -------------------------------------------------- a inversão da geometria

def test_base_link_esta_NO_EIXO(urdf):
    """Convenção C8: a origem é o centro do eixo motriz, não o centro da caixa.

    Com isto o centro de rotação É a origem e `wz` puro não desloca `x` nem `y`.
    No robô 2 o eixo ficava 8,15 cm à frente da origem, e todo giro carregava
    deslocamento lateral que ninguém escolheu.
    """
    for lado in ('left', 'right'):
        assert abs(_xyz(_junta(urdf, f'{lado}_wheel_joint'))[0]) < TOL


def test_motrizes_ATRAS_e_bobas_na_FRENTE(urdf):
    """O contrário do robô 2, e é a mudança que define o robô 3."""
    (cx, _cy, _cz), (ox, _oy, _oz) = _caixa(urdf)
    eixo_x = _xyz(_junta(urdf, 'left_wheel_joint'))[0]
    boba_x = _xyz(_junta(urdf, 'left_caster_swivel_joint'))[0]
    assert eixo_x < ox < boba_x, 'a caixa tem de ficar ENTRE o eixo e as bobas'
    # E o eixo fica atrás da traseira da caixa em projeção: a roda passa 2 cm.
    traseira_caixa = ox - cx / 2
    r, _ = _cilindro(urdf, 'left_wheel')
    assert (eixo_x - r) < traseira_caixa, 'a roda tem de passar da traseira'
    assert abs((traseira_caixa - (eixo_x - r)) - 0.020) < 1e-3, \
        'os 2 cm de roda passando atrás foram medidos — ver C5'


def test_bobas_nas_QUINAS_da_frente(urdf):
    """Medido: "cada uma na ponta da frente da caixa, uma em cada extremidade"."""
    (cx, cy, _cz), (ox, _oy, _oz) = _caixa(urdf)
    for lado, sinal in (('left', 1), ('right', -1)):
        bx, by, _bz = _xyz(_junta(urdf, f'{lado}_caster_swivel_joint'))
        assert abs(bx - (ox + cx / 2)) < TOL, 'a boba fica na ponta da frente'
        assert abs(by - sinal * cy / 2) < TOL, 'a boba fica na quina lateral'


@pytest.mark.parametrize('lado', ['left', 'right'])
def test_boba_tem_trail_e_ele_DOBROU(urdf, lado):
    """Trail zero = boba ideal = simulador que mente.

    E aqui ele é o dobro do robô 2 (20 mm contra 10), em duas bobas em vez de
    uma: numa inversão de marcha o contato é arrastado 40 mm contra 20 mm, o que
    prevê até 9,1° de perturbação de rumo contra 4,6° (C7). Se este número cair
    para o do robô 2, a previsão some junto e a ré volta a parecer barata.
    """
    trail = -_xyz(_junta(urdf, f'{lado}_caster_wheel_joint'))[0]
    assert trail > 0.015, 'trail do robô 3 é 20 mm, medido'


@pytest.mark.parametrize('lado', ['left', 'right'])
def test_pivo_da_boba_tem_atrito(urdf, lado):
    """Pivô livre demais alinha na hora e o chicote da ré desaparece."""
    d = _junta(urdf, f'{lado}_caster_swivel_joint').find('dynamics')
    assert d is not None
    assert float(d.get('friction')) > 0.0
    assert float(d.get('damping')) > 0.0


def test_caixa_nao_raspa_o_chao(urdf):
    (_cx, _cy, cz), (_ox, _oy, oz) = _caixa(urdf)
    assert oz - cz / 2 > 0.05, 'fundo da caixa medido a 70 mm do chão'


# ------------------------------------------------ envelope: o que passa na porta

def test_a_LARGURA_vem_do_PNEU_e_nao_da_caixa(urdf):
    """No robô 2 as rodas ficavam DENTRO da largura da caixa; aqui não.

    Caixa de 24 cm contra envelope de 47,5 — as motrizes são a parte mais larga,
    com 11,75 cm saindo de cada lado. Construir footprint a partir de `caixa_y`
    é o erro fácil deste robô, e ele daria um footprint 23 cm mais estreito que
    o robô. Ver §5.8.2.
    """
    (_cx, cy, _cz), _o = _caixa(urdf)
    _r, larg_roda = _cilindro(urdf, 'left_wheel')
    y_roda = _xyz(_junta(urdf, 'left_wheel_joint'))[1]
    envelope = 2 * y_roda + larg_roda
    assert envelope > cy, 'a roda tem de ser mais larga que a caixa'
    assert abs(envelope - 0.475) < 1e-3, 'envelope medido: 37,5 interno + 2×5,0'


def test_o_COMPRIMENTO_vem_da_CAIXA_com_a_roda_passando(urdf):
    """Total 33,2 cm: da traseira do pneu à ponta da caixa."""
    (cx, _cy, _cz), (ox, _oy, _oz) = _caixa(urdf)
    r, _ = _cilindro(urdf, 'left_wheel')
    eixo_x = _xyz(_junta(urdf, 'left_wheel_joint'))[0]
    comprimento = (ox + cx / 2) - (eixo_x - r)
    assert abs(comprimento - 0.3315) < 1e-3


def test_passa_na_porta_de_70_cm_em_QUALQUER_angulo(urdf):
    """A conta que o robô 2 não fechava (C6).

    O que aperta numa porta é a DIAGONAL do envelope, porque é a projeção máxima
    do corpo sobre a soleira. O robô 2 tinha 62,8 cm de diagonal contra 70 de
    vão — 3,6 cm por lado, e passar virava sorte. Este robô é mais LARGO que o
    robô 2 (47,5 contra 45,5) e ganha por ser muito mais CURTO.
    """
    (cx, _cy, _cz), (ox, _oy, _oz) = _caixa(urdf)
    r, larg_roda = _cilindro(urdf, 'left_wheel')
    eixo_x = _xyz(_junta(urdf, 'left_wheel_joint'))[0]
    larg = 2 * _xyz(_junta(urdf, 'left_wheel_joint'))[1] + larg_roda
    comp = (ox + cx / 2) - (eixo_x - r)
    diagonal = math.hypot(larg, comp)
    assert diagonal < 0.70, 'não caberia na porta 2 nem de frente'
    assert (0.70 - diagonal) / 2 > 0.05, 'folga do pior caso: 6,1 cm por lado'


# ------------------------------------------------- URDF e controlador em par

def test_separacao_bate_com_o_controlador(urdf, params):
    """Divergir aqui é o controlador comandar um giro e o robô fazer outro.

    Foi exatamente o BO do robô 2 (simulador 0,20 contra robô 0,32, ambos
    herdados e ambos errados, para lados opostos).
    """
    y = _xyz(_junta(urdf, 'left_wheel_joint'))[1]
    assert abs(2 * y - params['wheel_separation']) < TOL


def test_raio_bate_com_o_controlador(urdf, params):
    r, _ = _cilindro(urdf, 'left_wheel')
    assert abs(r - params['wheel_radius']) < TOL


def test_teto_de_giro_cobre_o_patamar_da_placa(params):
    """O corte tem de ficar ACIMA do que a placa entrega, senão desfaz a placa.

    E o patamar em `wz` depende da bitola: a mesma placa gira o robô 3 mais
    devagar que o robô 2, porque a roda tem 57% mais braço para vencer.
    """
    patamar = 2 * (100 * 0.0372 * params['wheel_radius']) / params['wheel_separation']
    assert params['angular.z.max_velocity'] > patamar
    assert params['angular.z.max_velocity'] < 2 * patamar, 'teto solto demais'


# ------------------------------------------------------------------- sensor

def test_o_livox_e_o_ponto_mais_alto(urdf):
    (_cx, _cy, cz), (_ox, _oy, oz) = _caixa(urdf)
    z_livox = _xyz(_junta(urdf, 'livox_joint'))[2]
    r, h = _cilindro(urdf, 'livox_frame', tag='visual')
    assert z_livox + h / 2 > oz + cz / 2, 'o sensor não pode ficar dentro da caixa'


def test_a_zona_cega_que_essa_altura_implica(urdf):
    """O Mid-360 só olha até −7°: o chão começa a ~8,1× a altura do sensor.

    ⚠️ Este teste NÃO trava a altura — ela é chute, porque o Livox não está
    montado (D7). Ele trava a CONSEQUÊNCIA ser conhecida: se alguém subir o
    sensor, a zona cega cresce junto e o teste avisa antes de a bancada avisar.
    """
    z = _xyz(_junta(urdf, 'livox_joint'))[2]
    raio_cego = z / math.tan(math.radians(7.0))
    assert raio_cego < 2.5, (
        f'zona cega de {raio_cego:.2f} m — acima disto o robô perde obstáculo '
        f'baixo e perto, e nenhuma sintonia resolve')
