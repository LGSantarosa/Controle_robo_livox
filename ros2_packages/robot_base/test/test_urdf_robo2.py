"""Trava a geometria e a massa do modelo do robô 2.

Estes testes não conferem estilo — conferem coisas que, se derivarem, fazem o
simulador mentir em silêncio:

- se as rodas não tocarem o chão exatamente, o robô nasce pulando ou afundado;
- se a caixa raspar o chão, aparece um atrito que o robô real não tem;
- se o `trail` da boba for a zero, a boba vira ideal e o problema que estamos
  estudando (traseira jogada pra fora no giro) simplesmente some;
- se a separação das rodas na URDF e no YAML do controlador divergirem, o
  controlador comanda um giro e o robô faz outro.
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
XACRO = os.path.join(AQUI, 'description', 'robo2.urdf.xacro')
YAML_SIM = os.path.join(AQUI, 'config', 'hoverboard_controllers_sim.yaml')

pytestmark = pytest.mark.skipif(xacro is None, reason='xacro não disponível')

TOL = 1e-6


@pytest.fixture(scope='module')
def urdf():
    return ET.fromstring(xacro.process_file(XACRO, mappings={'sim': 'false'}).toxml())


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


def _raio_cilindro(urdf, link_nome, tag='collision'):
    for l in urdf.findall('link'):
        if l.get('name') == link_nome:
            cil = l.find(f'./{tag}/geometry/cylinder')
            assert cil is not None, f'{link_nome} não tem cilindro em {tag}'
            return float(cil.get('radius'))
    raise AssertionError(f'link {link_nome} não existe')


@pytest.mark.parametrize('lado', ['left', 'right'])
def test_roda_motriz_toca_o_chao(urdf, lado):
    """Centro da roda na altura do próprio raio → contato exatamente em z=0."""
    _, _, z = _xyz(_junta(urdf, f'{lado}_wheel_joint'))
    assert abs(z - _raio_cilindro(urdf, f'{lado}_wheel')) < TOL


def test_boba_toca_o_chao(urdf):
    """A boba desce do fundo da caixa até o chão pelo garfo."""
    _, _, z_pivo = _xyz(_junta(urdf, 'caster_swivel_joint'))
    _, _, dz = _xyz(_junta(urdf, 'caster_wheel_joint'))
    centro = z_pivo + dz
    assert abs(centro - _raio_cilindro(urdf, 'caster_wheel')) < TOL


def test_motrizes_na_frente_boba_atras(urdf):
    """Geometria que causa o problema: tração à frente, ponta solta atrás."""
    x_motriz, _, _ = _xyz(_junta(urdf, 'left_wheel_joint'))
    x_pivo, _, _ = _xyz(_junta(urdf, 'caster_swivel_joint'))
    assert x_motriz > 0 > x_pivo


def test_boba_tem_trail_nao_nulo(urdf):
    """Sem deslocamento entre pivô e contato, a boba não é jogada pra fora.

    É o parâmetro que faz este simulador reproduzir o robô em vez de um robô
    ideal. Zerar isso invalida qualquer ajuste feito aqui.

    O limite é RELATIVO ao raio da roda, não absoluto. O piso anterior (10 mm
    fixos) foi escrito quando o modelo supunha uma roda de 100 mm — nessa escala
    10 mm de trail é desprezível e o piso fazia sentido. Medido o robô em
    2026-07-29, a boba cabe em 85,2 mm de vão e a roda é de ~50 mm; aí 10 mm de
    trail passa a ser o valor TÍPICO (15–25% do diâmetro), e o piso absoluto
    reprovava justamente o valor correto. Amarrar à roda vale em qualquer
    tamanho e preserva a intenção: o trail não pode sumir.
    """
    dx, _, _ = _xyz(_junta(urdf, 'caster_wheel_joint'))
    raio = _raio_cilindro(urdf, 'caster_wheel')
    assert abs(dx) > 0.2 * raio, 'trail da boba sumiu — simulador vira robô ideal'


def test_pivo_da_boba_tem_atrito(urdf):
    """Pivô livre demais se alinha na hora e esconde o problema."""
    din = _junta(urdf, 'caster_swivel_joint').find('dynamics')
    assert din is not None, 'pivô da boba sem <dynamics>'
    assert float(din.get('friction')) > 0
    assert float(din.get('damping')) > 0


def test_caixa_nao_raspa_o_chao(urdf):
    """Quem toca o chão são as rodas. Caixa raspando = atrito inventado."""
    for l in urdf.findall('link'):
        if l.get('name') == 'base_link':
            col = l.find('collision')
            _, _, cz = _xyz(col)
            altura = float(col.find('geometry/box').get('size').split()[2])
            assert cz - altura / 2 > 0.05
            return
    raise AssertionError('base_link não existe')


def test_massa_total(urdf):
    """~10,265 kg: o dono descreveu o robô como leve. A inércia que sai daqui
    é o que produz o sobrepasso de rumo — massa errada, ajuste errado.

    Os 10,0 são chassi + rodas + boba (todos ESTIMADOS, item de bancada); os
    0,265 são o Livox Mid-360 da decisão 012, e este é o único número de massa
    do robô que vem de folha de fabricante em vez de chute. Ele entra porque
    fica ALTO (0,27 m) e mexe no centro de massa.
    """
    total = sum(
        float(m.get('value'))
        for m in urdf.findall('./link/inertial/mass')
    )
    assert math.isclose(total, 10.265, abs_tol=0.05), f'massa total = {total}'


def test_inercias_positivas(urdf):
    """Inércia zerada faz o corpo girar sem resistência — some o sobrepasso."""
    for l in urdf.findall('link'):
        i = l.find('inertial/inertia')
        if i is None:
            continue
        for eixo in ('ixx', 'iyy', 'izz'):
            assert float(i.get(eixo)) > 0, f'{l.get("name")}.{eixo} <= 0'


def test_separacao_das_rodas_bate_com_o_controlador(urdf):
    """URDF e YAML têm que concordar: é esse número que converte cmd_vel em
    rad/s de roda. Divergir = comandar um giro e o robô fazer outro."""
    _, y_esq, _ = _xyz(_junta(urdf, 'left_wheel_joint'))
    _, y_dir, _ = _xyz(_junta(urdf, 'right_wheel_joint'))
    separacao_urdf = abs(y_esq - y_dir)

    with open(YAML_SIM) as f:
        cfg = yaml.safe_load(f)
    separacao_yaml = cfg['hoverboard_base_controller']['ros__parameters']['wheel_separation']

    assert math.isclose(separacao_urdf, separacao_yaml, abs_tol=TOL)


def test_raio_da_roda_bate_com_o_controlador(urdf):
    with open(YAML_SIM) as f:
        cfg = yaml.safe_load(f)
    raio_yaml = cfg['hoverboard_base_controller']['ros__parameters']['wheel_radius']
    assert math.isclose(_raio_cilindro(urdf, 'left_wheel'), raio_yaml, abs_tol=TOL)


# ------------------------------------------- o Mid-360 (medido com trena 05-08)
#
# A altura do sensor era SUPOSTA (topo da caixa + 4 cm = 27 cm) e a trena deu
# 42 cm — 55% de erro. Como o Mid-360 quase não olha para baixo (−7°), a zona
# cega escala com ela (`h/tan 7° ≈ 8,1·h`): 2,19 m no valor suposto contra
# 3,40 m no real. O simulador enxergava obstáculo baixo que o robô não enxerga.
# Nada travava esse número — é a lição de 29-07 ("a bancada não tinha teste
# nenhum, foi assim que sobreviveu errada").

def test_o_livox_existe_no_robo_REAL(urdf):
    """A fixture renderiza com `sim:=false`, que é o que o robô carrega desde
    05-08. Sem `livox_frame` aqui não há `base_link → livox_frame`, e o
    `collision_monitor` não tem como trazer a nuvem para o corpo — era esse o
    bloqueio do teste D."""
    assert _junta(urdf, 'livox_joint') is not None
    assert any(l.get('name') == 'livox_frame' for l in urdf.findall('link'))


def test_altura_do_livox_e_a_MEDIDA(urdf):
    """42 cm do chão, trena de 05-08. `base_link` está no nível do chão (a
    junta da roda fica a `roda_raio` acima dele), então a origem da junta é a
    altura de montagem, sem somar mais nada — somar era o que a versão anterior
    fazia, e é onde o erro se escondia."""
    _, y, z = _xyz(_junta(urdf, 'livox_joint'))
    assert math.isclose(z, 0.42, abs_tol=1e-9), (
        f'altura do Mid-360 = {z:.3f} m, medida = 0,42. Trocar aqui reescala a '
        f'zona cega inteira (8,1x a altura)')
    assert math.isclose(y, 0.0, abs_tol=TOL), 'medido CENTRADO no robô'


def test_a_zona_cega_que_essa_altura_implica(urdf):
    """Não é redundante com o teste acima: traduz a altura na grandeza que a
    operação sente. Se alguém mexer na montagem, este teste diz o que muda no
    campo — e o número tem de ir junto para o `collision_monitor.yaml` e para
    o tamanho da caixa do teste D."""
    _, _, h = _xyz(_junta(urdf, 'livox_joint'))
    raio_cego = h / math.tan(math.radians(7.0))
    assert raio_cego == pytest.approx(3.42, abs=0.05), (
        f'raio cego = {raio_cego:.2f} m')
    # A 0,5 m do robô, só aparece o que passar desta altura:
    visivel_a_meio_metro = h - 0.5 * math.tan(math.radians(7.0))
    assert visivel_a_meio_metro == pytest.approx(0.359, abs=0.005), (
        f'a 0,5 m o sensor só vê acima de {visivel_a_meio_metro:.3f} m — a '
        f'caixa do teste D tem de passar disso com folga (50 cm, não 40)')


def test_o_livox_e_o_ponto_mais_ALTO_do_robo(urdf):
    """Trava um fato que o `collision_monitor.yaml` ainda descreve errado: ele
    diz "o robô tem 0,30 m de alto", número do modelo antigo da decisão 004. A
    caixa termina a 0,230 m e o sensor está a 0,42 — é ele quem define o gabarito
    do robô, e é por ele que se decide o `max_height` do reflexo."""
    _, _, z_livox = _xyz(_junta(urdf, 'livox_joint'))
    topo_da_caixa = 0.0852 + 0.145          # altura_solo + caixa_z
    assert z_livox > topo_da_caixa, (
        'o sensor tem de estar acima da caixa; se deixar de estar, o gabarito '
        'do robô muda e o max_height do collision_monitor precisa ser revisto')
    assert topo_da_caixa == pytest.approx(0.230, abs=0.001)
