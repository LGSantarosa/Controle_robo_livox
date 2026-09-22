"""Coerência footprint ↔ reflexo POR PERFIL — etapa 4, passo 5 (plano §9).

Sobre o que os nós leriam: o perfil montado, com `aplica_reescritas` aplicado
aos dois YAMLs, e não os arquivos crus. A comparação é GEOMÉTRICA (vértices
lidos do texto, contenção por semiplano, área), não de texto nem de números
escritos aqui.

- **Robô 2 — a regra da 032, exatamente:** o footprint dos DOIS costmaps é o
  `PolygonStop`. Nada se exige do `PolygonApproach` dele: é o corpo + 3 cm
  (frente 0,2465) e NÃO contém o footprint (frente 0,35). Uma regra única de
  contenção reprovaria o robô 2 — é por isso que a regra é por perfil.
- **Robô 3:** o footprint é a geometria pura da 052, sem folga; cada polígono
  do reflexo (Approach e Stop, separadamente) CONTÉM o footprint inteiro e é
  diferente dele (área maior). Entre Approach e Stop nada se exige: são duas
  políticas distintas (D3).
"""
import importlib
import os

import pytest
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
BASE = os.path.abspath(os.path.join(PKG, '..', 'robot_base'))
COSTMAPS = ('global_costmap', 'local_costmap')
REFLEXO = ('PolygonApproach', 'PolygonStop')
TOL = 1e-9


def _perfil():
    return importlib.import_module('robot_motion.perfil')


def _le(caminho):
    with open(caminho) as f:
        return yaml.safe_load(f)


def _montado(robo):
    p = _perfil()
    par = p.parametros(robo, PKG, **({'share_base': BASE} if robo == 3 else {}))
    nav2 = p.aplica_reescritas(_le(par['nav2']), par['nav2_rewrites'])
    cm = p.aplica_reescritas(_le(par['collision_monitor']),
                             par['collision_monitor_rewrites'])
    footprints = {c: _vertices(nav2[c][c]['ros__parameters']['footprint'])
                  for c in COSTMAPS}
    reflexo = {pol: _vertices(cm['collision_monitor']['ros__parameters'][pol]['points'])
               for pol in REFLEXO}
    return footprints, reflexo


# ─── geometria ───────────────────────────────────────────────────────────────

def _vertices(texto):
    assert isinstance(texto, str), 'o Nav2 exige os pontos como TEXTO'
    pts = yaml.safe_load(texto)
    assert len(pts) >= 3 and all(len(v) == 2 for v in pts), pts
    return [(float(x), float(y)) for x, y in pts]


def _area_com_sinal(pol):
    return sum(x0 * y1 - x1 * y0
               for (x0, y0), (x1, y1) in zip(pol, pol[1:] + pol[:1])) / 2


def _area(pol):
    return abs(_area_com_sinal(pol))


def _convexo(pol):
    n = len(pol)
    cruz = [(pol[(i + 1) % n][0] - pol[i][0]) * (pol[(i + 2) % n][1] - pol[(i + 1) % n][1])
            - (pol[(i + 1) % n][1] - pol[i][1]) * (pol[(i + 2) % n][0] - pol[(i + 1) % n][0])
            for i in range(n)]
    return all(c > TOL for c in cruz) or all(c < -TOL for c in cruz)


def _contem(externo, interno):
    """`interno` ⊆ `externo` (borda conta).

    Exige `externo` convexo: aí basta cada vértice do interno estar do lado de
    dentro de toda aresta.
    """
    assert _convexo(externo), f'contenção por semiplano exige convexo: {externo}'
    s = 1 if _area_com_sinal(externo) > 0 else -1
    arestas = list(zip(externo, externo[1:] + externo[:1]))
    return all(s * ((bx - ax) * (py - ay) - (by - ay) * (px - ax)) >= -TOL
               for px, py in interno for (ax, ay), (bx, by) in arestas)


def _mesmo_poligono(a, b):
    """Mesmo ciclo de vértices, a menos do ponto de partida e do sentido (CW/CCW)."""
    if len(a) != len(b):
        return False
    return any(all(abs(p[0] - q[0]) <= TOL and abs(p[1] - q[1]) <= TOL
                   for p, q in zip(a, c[k:] + c[:k]))
               for c in (b, b[::-1]) for k in range(len(c)))


# ─── os auxiliares mordem (senão um verde não afirma nada) ───────────────────

QUADRADO = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]


def test_contem_aceita_o_menor_e_o_igual_e_recusa_o_que_vaza():
    menor = [(0.5, 0.5), (0.5, -0.5), (-0.5, -0.5), (-0.5, 0.5)]
    assert _contem(QUADRADO, menor)
    assert _contem(QUADRADO, QUADRADO), 'borda conta'
    assert not _contem(menor, QUADRADO)
    vaza_uma_quina = [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.001)]
    assert not _contem(QUADRADO, vaza_uma_quina)


def test_contem_nao_depende_do_sentido_dos_vertices():
    assert _contem(QUADRADO[::-1], [(0.0, 0.0), (0.9, 0.9), (-0.9, -0.9)])
    assert not _contem(QUADRADO[::-1], [(1.1, 0.0), (0.0, 0.0), (0.0, 0.1)])


def test_contem_recusa_externo_nao_convexo():
    seta = [(1.0, 1.0), (0.0, 0.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.0)]
    with pytest.raises(AssertionError, match='convexo'):
        _contem(seta, [(0.0, 0.5)])


def test_mesmo_poligono_e_geometria_e_nao_texto():
    assert _mesmo_poligono(QUADRADO, QUADRADO[1:] + QUADRADO[:1])
    assert _mesmo_poligono(QUADRADO, QUADRADO[::-1]), 'CW e CCW são o mesmo polígono'
    assert not _mesmo_poligono(QUADRADO, [(1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0)]), \
        'mesmos pontos, outro ciclo: é outro polígono (laço)'
    assert not _mesmo_poligono(QUADRADO, [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)])
    assert not _mesmo_poligono(QUADRADO, [(1.0, 1.0), (1.0, -1.0), (-1.0, -1.0), (-1.0, 1.1)])
    assert _vertices('[[1, 1.0], [1.0, -1], [-1.0, -1.0], [-1, 1]]') == QUADRADO


# ─── robô 2: a regra da 032, sem mudar ───────────────────────────────────────

@pytest.mark.parametrize('costmap', COSTMAPS)
def test_robo2_footprint_e_o_polygon_stop(costmap):
    footprints, reflexo = _montado(2)
    assert _mesmo_poligono(footprints[costmap], reflexo['PolygonStop']), (
        f'{costmap}: planner e reflexo discordam sobre o corpo orientado (032)')


# ─── robô 3: o reflexo contém o footprint e não é ele ────────────────────────

def test_robo3_os_dois_costmaps_tem_o_mesmo_footprint():
    footprints, _ = _montado(3)
    assert _mesmo_poligono(footprints['global_costmap'], footprints['local_costmap'])


@pytest.mark.parametrize('costmap', COSTMAPS)
@pytest.mark.parametrize('pol', REFLEXO)
def test_robo3_reflexo_contem_o_footprint(costmap, pol):
    footprints, reflexo = _montado(3)
    assert _contem(reflexo[pol], footprints[costmap]), (
        f'{pol} não cobre o footprint do {costmap}: o reflexo deixaria corpo '
        'de fora')


@pytest.mark.parametrize('costmap', COSTMAPS)
@pytest.mark.parametrize('pol', REFLEXO)
def test_robo3_reflexo_nao_e_o_footprint(costmap, pol):
    footprints, reflexo = _montado(3)
    fp, r = footprints[costmap], reflexo[pol]
    assert not _mesmo_poligono(fp, r), (
        f'{pol} igual ao footprint: no robô 3 o footprint não tem folga, e o '
        'reflexo sem margem só dispara com o corpo já encostando')
    assert _area(r) > _area(fp) + 1e-6, f'{pol} contém o footprint mas não é maior'
