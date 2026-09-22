"""Trava a geometria do modelo do robô 3.

Mesmo espírito do `test_urdf_robo2.py`: não confere estilo, confere o que, se
derivar, faz o simulador mentir em silêncio. O que muda aqui é o que a máquina
mudou — e cada teste abaixo guarda uma decisão que custou uma leva de medidas.

As medidas e a discussão de cada número estão em `docs/ROBO3_REVISAO_CRUZADA.md`.
"""

import math
import os
import re
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
GEOMETRIA = os.path.join(AQUI, 'config', 'geometria_robo3.yaml')
LAUNCH_CONTROLE = os.path.join(os.path.dirname(AQUI), 'robot_nav', 'launch',
                               'controle_robo3.launch.py')

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


def _link(urdf, nome):
    for l in urdf.findall('link'):
        if l.get('name') == nome:
            return l
    raise AssertionError(f'link {nome} não existe')


def _envolvente_x(urdf):
    """(min x, max x) da envolvente RÍGIDA em repouso: pneus e caixa.

    Por extremos, e não por "frente do pneu menos traseira da caixa": assim a
    conta não sabe para que lado o robô está virado, e fica igual antes e
    depois do giro de 180° — que não muda a envolvente rígida, só a gira.
    As bobas em repouso ficam por baixo da caixa e não estendem nada.
    """
    (cx, _cy, _cz), (ox, _oy, _oz) = _caixa(urdf)
    r, _ = _cilindro(urdf, 'left_wheel')
    eixo_x = _xyz(_junta(urdf, 'left_wheel_joint'))[0]
    return min(eixo_x - r, ox - cx / 2), max(eixo_x + r, ox + cx / 2)


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


# ------------------------------------------ o giro de 180° (etapa 3, 18-09)
#
# Até 17-09 este bloco travava "motrizes ATRÁS, bobas NA FRENTE". Em 16-09 o
# dono decidiu o contrário para a navegação: frente = motrizes, traseira =
# bobas (`docs/PLANO_NAV2_ROBO3.md`, §2 e §3). Os testes abaixo foram
# REESCRITOS para o estado girado — de propósito e com registro, não apagados.
#
# O giro é do CORPO em relação ao `base_link`, que continua no eixo (C8). E ele
# acopla quatro coisas que só fazem sentido juntas: posição em x, direção do
# trail, qual roda física é a esquerda e o yaw do Livox. Inverter x sem o resto
# não é rotação, é um modelo semanticamente intermediário.

def test_base_link_esta_NO_EIXO(urdf):
    """Convenção C8: a origem é o centro do eixo motriz, não o centro da caixa.

    Com isto o centro de rotação É a origem e `wz` puro não desloca `x` nem `y`.
    No robô 2 o eixo ficava 8,15 cm à frente da origem, e todo giro carregava
    deslocamento lateral que ninguém escolheu.
    """
    for lado in ('left', 'right'):
        assert abs(_xyz(_junta(urdf, f'{lado}_wheel_joint'))[0]) < TOL


def test_motrizes_na_FRENTE_e_bobas_ATRAS(urdf):
    """Frente = motrizes (decisão do dono, 16-09). x cresce para as motrizes.

    Reescrito em 18-09: até ali era `test_motrizes_ATRAS_e_bobas_na_FRENTE`.
    """
    (cx, _cy, _cz), (ox, _oy, _oz) = _caixa(urdf)
    eixo_x = _xyz(_junta(urdf, 'left_wheel_joint'))[0]
    boba_x = _xyz(_junta(urdf, 'left_caster_swivel_joint'))[0]
    assert boba_x < ox < eixo_x, 'a caixa tem de ficar ENTRE as bobas (atrás) e o eixo'
    # A roda passa 2 cm da ponta da caixa — agora a ponta da FRENTE.
    frente_caixa = ox + cx / 2
    r, _ = _cilindro(urdf, 'left_wheel')
    assert (eixo_x + r) > frente_caixa, 'a roda tem de passar da frente da caixa'
    assert abs(((eixo_x + r) - frente_caixa) - 0.020) < 1e-3, \
        'os 2 cm de roda passando da caixa foram medidos — ver C5'


def test_bobas_nas_quinas_mas_DEBAIXO_da_caixa(urdf):
    """Medido: "cada uma na ponta da frente da caixa, uma em cada extremidade".

    Ao pé da letra isso põe o PIVÔ em cima da quina — e aí meia rodinha fica
    para fora da lateral, que foi o que o dono viu no Gazebo. A regra que ele
    deu é sobre a SUPERFÍCIE, não sobre o pivô: "as pontas com as pontas".

    Então o que este teste trava é o alinhamento das FACES, e não uma distância
    escolhida a olho — recuo escolhido a olho já errou nos dois sentidos.
    """
    (cx, cy, _cz), (ox, _oy, _oz) = _caixa(urdf)
    _r, larg_boba = _cilindro(urdf, 'left_caster_wheel')
    for lado, sinal in (('left', 1), ('right', -1)):
        bx, by, _bz = _xyz(_junta(urdf, f'{lado}_caster_swivel_joint'))
        # Girado em 18-09: a quina das bobas agora é a TRASEIRA da caixa.
        assert abs(bx - (ox - cx / 2)) < TOL, 'a ponta do garfo é a ponta de trás da caixa'
        assert abs((abs(by) + larg_boba / 2) - cy / 2) < TOL, \
            'a face externa da rodinha é a lateral da caixa'
        assert by * sinal > 0, 'uma de cada lado'


@pytest.mark.parametrize('lado', ['left', 'right'])
def test_boba_tem_trail_e_ele_DOBROU(urdf, lado):
    """Trail zero = boba ideal = simulador que mente.

    E aqui ele é o dobro do robô 2 (20 mm contra 10), em duas bobas em vez de
    uma: numa inversão de marcha o contato é arrastado 40 mm contra 20 mm, o que
    prevê até 9,1° de perturbação de rumo contra 4,6° (C7). Se este número cair
    para o do robô 2, a previsão some junto e a ré volta a parecer barata.
    """
    trail = abs(_xyz(_junta(urdf, f'{lado}_caster_wheel_joint'))[0])
    assert trail > 0.015, 'trail do robô 3 é 20 mm, medido'


@pytest.mark.parametrize('lado', ['left', 'right'])
def test_trail_em_repouso_aponta_PARA_O_EIXO(urdf, lado):
    """A rodinha, parada, fica do lado do eixo motor, debaixo da caixa.

    É a posição física de repouso girada junto com o corpo, e é ela que
    preserva a regra "as pontas com as pontas": a ponta do conjunto é o pivô,
    rente à caixa. Escrito sem sinal de x de propósito — é a direção em relação
    ao eixo que tem significado, não o lado do plano.

    ⚠️ Andando para a frente (motrizes), cada rodinha gira 180° no pivô e passa
    a sair 20 mm da caixa. Isso é a ENVOLVENTE VARRIDA, e não é este teste que
    a cobre (item 5 do §3, com teste geométrico próprio).
    """
    pivo_x = _xyz(_junta(urdf, f'{lado}_caster_swivel_joint'))[0]
    eixo_x = _xyz(_junta(urdf, f'{lado}_wheel_joint'))[0]
    para_o_eixo = eixo_x - pivo_x
    dx = _xyz(_junta(urdf, f'{lado}_caster_wheel_joint'))[0]
    assert dx * para_o_eixo > 0, 'a rodinha tem de estar entre o pivô e o eixo'
    assert abs(abs(dx) - 0.020) < TOL

    # O garfo vai junto: desenho e centro de massa. Sem isto o CONTATO gira e
    # o garfo fica desenhado e pesando do lado antigo, com a suíte verde.
    garfo = _link(urdf, f'{lado}_caster_fork')
    vis_x = _xyz(garfo.find('visual'))[0]
    ine_x = _xyz(garfo.find('inertial'))[0]
    assert vis_x * para_o_eixo > 0, 'o desenho do garfo tem de ir para o lado do eixo'
    assert ine_x * para_o_eixo > 0, 'o centro de massa do garfo tem de ir para o lado do eixo'
    assert abs(vis_x - dx / 2) < TOL and abs(ine_x - dx / 2) < TOL, \
        'o garfo vai do pivô até a rodinha: meio do caminho'


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

    Caixa de 24 cm contra envelope de 37,25 — as motrizes são a parte mais larga,
    com 7,0 cm saindo de cada lado. Construir footprint a partir de `caixa_y` é
    o erro fácil deste robô, e ele daria um footprint 13 cm mais estreito que o
    robô. Ver §5.9.
    """
    (_cx, cy, _cz), _o = _caixa(urdf)
    _r, larg_roda = _cilindro(urdf, 'left_wheel')
    y_roda = _xyz(_junta(urdf, 'left_wheel_joint'))[1]
    envelope = 2 * y_roda + larg_roda
    assert envelope > cy, 'a roda tem de ser mais larga que a caixa'
    assert abs(envelope - 0.3805) < 1e-3, 'bitola medida 32,25 + roda de catálogo 5,8'


def test_o_COMPRIMENTO_vem_da_CAIXA_com_a_roda_passando(urdf):
    """Total 33,2 cm: caixa de 31,1 mais os 2 cm de pneu passando dela.

    Por extremos (`_envolvente_x`) desde 18-09: é verde antes e depois do giro,
    porque girar não muda o comprimento. Quem trava o LADO é
    `test_motrizes_na_FRENTE_e_bobas_ATRAS`.
    """
    x_min, x_max = _envolvente_x(urdf)
    assert abs((x_max - x_min) - 0.3315) < 1e-3


def test_passa_na_porta_de_70_cm_em_QUALQUER_angulo(urdf):
    """A conta que o robô 2 não fechava (C6).

    O que aperta numa porta é a DIAGONAL do envelope, porque é a projeção máxima
    do corpo sobre a soleira. O robô 2 tinha 62,8 cm de diagonal contra 70 de
    vão — 3,6 cm por lado, e passar virava sorte. Este robô é mais LARGO que o
    robô 2 (47,5 contra 45,5) e ganha por ser muito mais CURTO.
    """
    _r, larg_roda = _cilindro(urdf, 'left_wheel')
    larg = 2 * _xyz(_junta(urdf, 'left_wheel_joint'))[1] + larg_roda
    x_min, x_max = _envolvente_x(urdf)
    comp = x_max - x_min
    diagonal = math.hypot(larg, comp)
    assert diagonal < 0.70, 'não caberia na porta 2 nem de frente'
    assert (0.70 - diagonal) / 2 > 0.09, 'folga do pior caso: 10,0 cm por lado'


# ------------------------------- envolvente varrida e footprint (etapa 3, 18-09)

def _varredura_bobas(urdf):
    """[(pivô x, pivô y, raio varrido)] — tudo tirado do URDF.

    A boba gira 360° no pivô. O ponto mais longe do pivô, em planta, é a quina
    da rodinha: `trail + raio` ao longo e `largura/2` de lado. Esse é o raio do
    círculo que ela varre — e é o que o contorno tem de cobrir, não o ponto
    onde ela está parada (§3, item 5).
    """
    r, larg = _cilindro(urdf, 'left_caster_wheel')
    out = []
    for lado in ('left', 'right'):
        px, py, _ = _xyz(_junta(urdf, f'{lado}_caster_swivel_joint'))
        trail = abs(_xyz(_junta(urdf, f'{lado}_caster_wheel_joint'))[0])
        out.append((px, py, math.hypot(trail + r, larg / 2)))
    return out


def _pontos_do_corpo(urdf):
    """Quinas, em planta, de tudo o que é rígido: caixa e os dois pneus."""
    (cx, cy, _cz), (ox, oy, _oz) = _caixa(urdf)
    pts = [(ox + sx * cx / 2, oy + sy * cy / 2) for sx in (1, -1) for sy in (1, -1)]
    r, larg = _cilindro(urdf, 'left_wheel')
    for lado in ('left', 'right'):
        wx, wy, _ = _xyz(_junta(urdf, f'{lado}_wheel_joint'))
        pts += [(wx + sx * r, wy + sy * larg / 2) for sx in (1, -1) for sy in (1, -1)]
    return pts


def _dentro(poligono, x, y):
    """Ponto dentro (ou na borda) de polígono CONVEXO, qualquer sentido."""
    sinais = set()
    n = len(poligono)
    for i in range(n):
        (x1, y1), (x2, y2) = poligono[i], poligono[(i + 1) % n]
        c = (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1)
        if abs(c) > 1e-12:
            sinais.add(c > 0)
    return len(sinais) <= 1


@pytest.fixture(scope='module')
def footprint():
    with open(GEOMETRIA) as f:
        return [tuple(v) for v in yaml.safe_load(f)['footprint']['poligono']]


def test_a_varredura_das_bobas_sai_4_cm_atras_e_nao_alarga(urdf):
    """Raio varrido √((20+20)² + 15²) mm = 42,7 mm em torno de cada pivô.

    Atrás: a traseira passa de −0,2485 (caixa) para −0,2912. De lado: 0,1477,
    menos que a face do pneu (0,190) — quem manda na largura continua o pneu.
    """
    r, larg = _cilindro(urdf, 'left_wheel')
    y_pneu = _xyz(_junta(urdf, 'left_wheel_joint'))[1] + larg / 2
    for px, py, R in _varredura_bobas(urdf):
        assert abs(R - 0.04272) < 1e-4
        assert abs((px - R) - (-0.29122)) < 1e-4
        assert abs(py) + R < y_pneu, 'a varredura não pode alargar o robô'


def test_footprint_canonico_cobre_corpo_e_varredura(urdf, footprint):
    """O polígono de `geometria_robo3.yaml` contém tudo o que o robô ocupa.

    Corpo rígido pelas quinas; varredura amostrada no círculo inteiro de cada
    boba. É o artefato que o perfil Nav2 consome na etapa 4.
    """
    for x, y in _pontos_do_corpo(urdf):
        assert _dentro(footprint, x, y), f'corpo fora do footprint em ({x:.4f}, {y:.4f})'
    for px, py, R in _varredura_bobas(urdf):
        for k in range(360):
            a = math.radians(k)
            x, y = px + R * math.cos(a), py + R * math.sin(a)
            assert _dentro(footprint, x, y), \
                f'varredura da boba fora do footprint em ({x:.5f}, {y:.5f})'


def test_footprint_canonico_NAO_carrega_folga(urdf, footprint):
    """Geometria pura: cada lado encosta no extremo, a menos do arredondamento.

    A folga é `footprint_padding`, classe (c), no perfil. Se ela entrar aqui
    dentro, soma duas vezes e ninguém vê — foi o que aconteceu com o footprint
    do robô 2 (decisão 032: corpo + ~5 cm dentro do polígono).
    """
    pts = _pontos_do_corpo(urdf)
    for px, py, R in _varredura_bobas(urdf):
        pts += [(px - R, py), (px + R, py), (px, py - R), (px, py + R)]
    geo = (min(p[0] for p in pts), max(p[0] for p in pts),
           min(p[1] for p in pts), max(p[1] for p in pts))
    fp = (min(p[0] for p in footprint), max(p[0] for p in footprint),
          min(p[1] for p in footprint), max(p[1] for p in footprint))
    ARRED = 1e-4   # 4 casas, arredondando para FORA
    assert geo[0] - ARRED <= fp[0] <= geo[0] + 1e-9
    assert geo[1] - 1e-9 <= fp[1] <= geo[1] + ARRED
    assert geo[2] - ARRED <= fp[2] <= geo[2] + 1e-9
    assert geo[3] - 1e-9 <= fp[3] <= geo[3] + ARRED


def _raio_varrido_pivo(urdf):
    """Distância máxima ao `base_link` de tudo o que o robô ocupa girando nele.

    Envolvente FÍSICA: quinas do corpo rígido e, para cada boba, o ponto mais
    longe do disco que ela varre (|pivô| + raio varrido). NÃO é a quina do
    footprint retangular — ela é área vazia (0,348 m) e inflaria a folga.
    """
    corpo = max(math.hypot(x, y) for x, y in _pontos_do_corpo(urdf))
    bobas = max(math.hypot(px, py) + R for px, py, R in _varredura_bobas(urdf))
    return max(corpo, bobas)


def test_raio_varrido_pivo_do_artefato_bate_com_o_urdf(urdf, footprint):
    """`raio_varrido_pivo` (etapa 4, passo 4a) é geometria, classe (b).

    Consumidor: o `desencalhe_pivo_folga` do perfil do robô 3 = este raio +
    margem (c) em chave própria. Sem folga aqui, arredondado para FORA como a
    traseira do footprint; quem manda nele hoje são as bobas (0,31249), não o
    corpo rígido (0,2760).
    """
    with open(GEOMETRIA) as f:
        artefato = yaml.safe_load(f)['raio_varrido_pivo']
    geo = _raio_varrido_pivo(urdf)
    assert abs(geo - 0.31249) < 1e-5
    assert geo - 1e-9 <= artefato <= geo + 1e-4, 'fora da envolvente ou com folga'
    assert artefato == 0.3125, 'o valor escolhido no plano: 0,31249 arredondado para fora'
    quina = max(math.hypot(x, y) for x, y in footprint)
    assert artefato < quina - 0.03, 'isto é a quina vazia do retângulo, não o robô'


def test_urdf_girado_exige_frente_negativa_no_controle(urdf):
    """Coerência entre o URDF girado e o `frente:=-1.0` — NÃO observa roda física.

    O nome é modesto de propósito: este teste não sabe qual roda está parafusada
    onde. Ele só garante que as duas metades do giro andam juntas.

    `left_wheel_joint` fica em +y sempre — é convenção do ROS e não gira. O que
    a rotação de 180° troca é QUAL RODA FÍSICA está em +y. No robô 3 esse
    mapeamento vive no `cmd_vel_to_wheels`: a decisão 049 provou que negar só a
    linear (`linear_sign: -1.0`, exposto como `frente:=`) equivale ao giro de
    180° COM a troca de lado.

    Então os dois andam em par: URDF com motrizes na frente exige `frente`
    -1.0 por padrão. Mudar um sem o outro volta ao modelo meio girado — o que
    o simulador mostra deixa de ser o que o robô faz, sem erro nenhum.
    """
    for lado, sinal in (('left', 1), ('right', -1)):
        assert _xyz(_junta(urdf, f'{lado}_wheel_joint'))[1] * sinal > 0, \
            'left_wheel_joint fica em +y (convenção do ROS)'
    eixo_x = _xyz(_junta(urdf, 'left_wheel_joint'))[0]
    boba_x = _xyz(_junta(urdf, 'left_caster_swivel_joint'))[0]
    assert eixo_x > boba_x, 'pré-condição: o URDF está com as motrizes na frente'

    with open(LAUNCH_CONTROLE) as f:
        texto = f.read()
    m = re.search(r"'frente',\s*default_value='([^']+)'", texto)
    assert m, 'controle_robo3.launch.py não declara o argumento `frente`'
    assert float(m.group(1)) == -1.0, \
        'URDF com motrizes na frente exige `frente` -1.0 (decisão 049)'


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

def test_livox_no_centro_da_caixa_olhando_para_a_FRENTE(urdf):
    """Posição: centro da caixa, como decidido em 16-09 (emprestado do robô 2).

    🟡 Yaw 0 é uma CONVENÇÃO NOVA DE MONTAGEM, decidida pelo dono em 18-09 —
    NÃO é consequência do giro: "o x do sensor será alinhado à frente nova (as
    motrizes)". Girar rigidamente o chute antigo daria yaw π. Vale porque o
    sensor ainda não está montado; é PROVISÓRIA até a etapa 7 medir a pose 6D.
    """
    (_cx, _cy, _cz), (ox, _oy, _oz) = _caixa(urdf)
    j = _junta(urdf, 'livox_joint')
    assert abs(_xyz(j)[0] - ox) < TOL, 'o Livox fica no centro da caixa'
    assert abs(_xyz(j)[1]) < TOL
    o = j.find('origin')
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    assert all(abs(a) < TOL for a in rpy), 'yaw 0: x do sensor para a frente (motrizes)'


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
