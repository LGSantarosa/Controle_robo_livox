"""Testes da régua da bancada do planner (`mede`).

Esta régua é o que decide a comparação Theta* × Smac Hybrid-A*, e até 29-07 ela
não tinha teste nenhum — foi assim que ela passou meses lendo errado justamente
o caso que a bancada existe para julgar.

O defeito: caminho de Reeds-Shepp com **cúspide** (o ponto onde o robô troca de
sentido e passa a andar de ré). Na cúspide o caminho dobra sobre si mesmo, e
medir curvatura por três pontos vizinhos lê aquilo como uma curva fechadíssima.
Medido no caso real `perto_de_lado` com raio mínimo de 0,46 m configurado:

    raio mínimo relatado ... 0,125 m   (3,7x MENOR que o configurado)
    giro relatado .......... 181°      (giro que o robô não faz: ele inverte)
    inversões relatadas .... 0         (havia DUAS)

Ou seja, as três medidas erravam ao mesmo tempo, e na direção que fazia o
Reeds-Shepp parecer indisciplinado. O caminho estava certo; a régua, não.

As geometrias abaixo saem do caminho de verdade devolvido pelo `planner_server`
(20 pontos, passo de 0,072 m, virada de 10° por passo, cúspides nos pontos 4 e
17) — não são inventadas para o teste passar.
"""
import math

from robot_planning.bancada_planner import mede


class _Pose:
    def __init__(self, x, y):
        self.pose = type('p', (), {'position': type('q', (), {'x': x, 'y': y})})


class _Caminho:
    def __init__(self, pts):
        self.poses = [_Pose(x, y) for x, y in pts]


def arco(x0, y0, rumo0_deg, raio, total_deg, passo_deg=10.0, sentido=+1):
    """Pontos de um arco de raio constante, andando para FRENTE."""
    pts = []
    x, y, rumo = x0, y0, math.radians(rumo0_deg)
    n = int(abs(total_deg) / passo_deg)
    d = 2.0 * raio * math.sin(math.radians(passo_deg) / 2.0)
    giro = math.radians(passo_deg) * (1 if total_deg > 0 else -1)
    for _ in range(n):
        meio = rumo + giro / 2.0
        x += sentido * d * math.cos(meio)
        y += sentido * d * math.sin(meio)
        rumo += giro
        pts.append((x, y))
    return pts, x, y, math.degrees(rumo)


def caminho_com_cuspide(raio=0.46):
    """Frente 30°, INVERTE, ré por 60°, INVERTE, frente 30°.

    É a forma do caminho que o Smac devolve para um alvo a 0,6 m de lado: ele
    não consegue chegar lá só para frente com esse raio, então usa a ré do
    Reeds-Shepp. O robô descreve arcos de `raio` o tempo todo — nunca fecha
    nada mais apertado que isso.
    """
    pts = [(0.0, 0.0)]
    a, x, y, rumo = arco(0.0, 0.0, 0.0, raio, -30.0)
    pts += a
    b, x, y, rumo = arco(x, y, rumo, raio, -60.0, sentido=-1)   # de ré
    pts += b
    c, x, y, rumo = arco(x, y, rumo, raio, +30.0)
    pts += c
    return _Caminho(pts)


def test_cuspide_nao_vira_curva_fechada():
    """O raio medido é o dos ARCOS, não o da dobra da cúspide.

    Sem isto a bancada acusa o Smac de desenhar caminho inseguível quando o
    caminho é perfeitamente seguível — só tem ré no meio.
    """
    m = mede(caminho_com_cuspide(raio=0.46))
    assert m['raio_min'] > 0.35, (
        f"raio medido {m['raio_min']:.3f} — a cúspide voltou a ser lida como "
        'curva fechada')


def test_cuspide_e_contada_como_inversao():
    """Duas trocas de sentido têm que aparecer como duas inversões.

    `inversoes` é a única coluna que mostra a ré do Reeds-Shepp, que é o motivo
    de o Smac estar na disputa. Zerada, a bancada fica cega para o próprio tema.
    """
    m = mede(caminho_com_cuspide())
    assert m['inversoes'] == 2, f"inversões medidas: {m['inversoes']}"


def test_giro_nao_conta_a_inversao_como_virada():
    """O robô não gira 180° na cúspide: ele para e volta.

    Somar a dobra ao giro total infla o número que compara os dois planners,
    e sempre contra quem usa ré.
    """
    m = mede(caminho_com_cuspide())
    # 30 + 60 + 30 = 120° de virada real, com folga para a discretização.
    assert m['giro_deg'] < 150.0, f"giro medido: {m['giro_deg']:.0f}°"


def test_arco_simples_mede_o_raio_certo():
    """Sem cúspide, a régua continua medindo o que media (não regredimos)."""
    pts, *_ = arco(0.0, 0.0, 0.0, 0.50, 90.0, passo_deg=5.0)
    m = mede(_Caminho([(0.0, 0.0)] + pts))
    assert 0.42 < m['raio_min'] < 0.58, f"raio medido: {m['raio_min']:.3f}"
    assert m['inversoes'] == 0


def test_giro_subestima_arco_continuo_VIES_CONHECIDO():
    """O `giro` de um arco contínuo sai CURTO, e isto está por consertar.

    Um arco de 90° é medido como ~50°. A causa é a reamostragem a 0,20 m: o
    giro é somado nos VÉRTICES da poligonal, e as meias-viradas das duas pontas
    (entre a direção real e a primeira/última corda) não têm vértice onde
    aparecer. Quanto mais largo o passo, maior a perda.

    Fica registrado e NÃO corrigido junto com a cúspide, de propósito: são dois
    defeitos independentes, e o viés puxa para o lado CONTRÁRIO ao da cúspide
    (a cúspide inflava o giro do Smac, este o encolhe). Corrigir os dois na
    mesma mudança tornaria impossível saber qual moveu qual número.

    Consequência prática enquanto isto valer: a coluna `giro` serve para
    comparar caminhos entre SI na mesma corrida, não como ângulo absoluto — e
    ela favorece quem faz curva contínua (Smac) contra quem faz canto vivo
    (Theta*, cujos cantos têm vértice e são contados inteiros).
    """
    pts, *_ = arco(0.0, 0.0, 0.0, 0.50, 90.0, passo_deg=5.0)
    m = mede(_Caminho([(0.0, 0.0)] + pts))
    assert 40.0 < m['giro_deg'] < 60.0, (
        f"giro medido {m['giro_deg']:.0f}° — se este teste caiu porque o "
        'número melhorou, o viés foi consertado: reescreva o teste')


def test_tremor_no_alvo_nao_vira_raio_zero():
    """Trecho curto demais para a régua é PULADO, não medido como raio ~0.

    Caso real: `bloco` com raio mínimo de 0,46 m. O Smac desenha 4,86 m limpos
    e depois TREME em cima do alvo — 4 inversões dentro de uma caixa de 9 cm,
    deixando trechos de 8 cm entre cúspides. Medir curvatura ali devolvia
    `raio_min = 0,00 m` e condenava um caminho cujo trecho longo fecha 0,54 m.

    O tremor é do planner e é achado de verdade (fica no relatório); o 0,00 era
    da régua.
    """
    reto, x, y, rumo = arco(0.0, 0.0, 0.0, 0.60, 60.0, passo_deg=5.0)
    pts = [(0.0, 0.0)] + reto
    # o tremor: quatro passos de 8 cm alternando de sentido
    for k in range(4):
        ang = math.radians(rumo) + (math.pi if k % 2 else 0.0)
        x += 0.08 * math.cos(ang)
        y += 0.08 * math.sin(ang)
        pts.append((x, y))
    m = mede(_Caminho(pts))
    assert m['raio_min'] > 0.40, (
        f"raio medido {m['raio_min']:.3f} — o tremor voltou a ser lido como "
        'curva')
    assert m['trechos_curtos'] >= 2, (
        'trechos pulados têm que ser CONTADOS: pular em silêncio é como o '
        'defeito da cúspide sobreviveu tanto tempo')


def test_reta_pura_segue_sem_curva_e_sem_inversao():
    m = mede(_Caminho([(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.5, 0.0)]))
    assert math.isinf(m['raio_min'])
    assert m['inversoes'] == 0
    assert m['giro_deg'] < 1e-6
    assert abs(m['desvio'] - 1.0) < 1e-6
