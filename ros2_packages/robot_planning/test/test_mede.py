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


def test_giro_de_arco_continuo_nao_encolhe():
    """Um arco de 90° tem que medir ~90°.

    Até 29-07 media 50°: o giro era somado nos vértices da poligonal
    REAMOSTRADA a 0,20 m, e as meias-viradas das duas pontas não tinham vértice
    onde aparecer. O erro não era parelho entre os planners — o canto vivo do
    Theta* tem vértice e era contado inteiro, enquanto o arco do Smac perdia
    nas duas pontas. A coluna favorecia o Smac na comparação que ela arbitra.

    Consertado somando nos pontos CRUS, o que só é honesto porque o passo cru é
    limpo (Theta* anda 0,05 m com virada mediana de 0,00°; o Smac 0,086 m com
    virada máxima de 19,9°, que é o arco no raio configurado, não ruído).
    """
    pts, *_ = arco(0.0, 0.0, 0.0, 0.50, 90.0, passo_deg=5.0)
    m = mede(_Caminho([(0.0, 0.0)] + pts))
    assert 80.0 < m['giro_deg'] < 100.0, f"giro medido: {m['giro_deg']:.0f}°"


def test_giro_do_canto_vivo_continua_inteiro():
    """Canto de 90° segue valendo 90° — a correção não podia tirar daqui.

    O risco de somar no cru era o oposto do defeito: inflar o giro com ruído.
    Este teste trava a outra ponta, num caminho que é só reta e um canto.
    """
    pts = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0, 0.5), (1.0, 1.0)]
    m = mede(_Caminho(pts))
    assert 85.0 < m['giro_deg'] < 95.0, f"giro medido: {m['giro_deg']:.0f}°"


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


def test_toco_de_ponta_nao_infla_o_giro():
    """Segmento de milímetros na ponta não define direção.

    O planner cola a pose exata de partida e de chegada no caminho
    discretizado, e sobra um toco fora do arco em cada extremidade. Medido no
    caso real `lado_1m` com raio 0,25: dois tocos de 7,7 mm injetavam ±125,6°
    cada, e o giro de um caminho de 1,19 m saía **447°** — mais de três voltas
    de bico num percurso de um metro.

    A geometria abaixo é a daquele caminho: arco limpo com um toco em cada
    ponta, saindo perpendicular para exagerar o efeito.
    """
    pts, x, y, rumo = arco(0.0, 0.0, 0.0, 0.30, 90.0, passo_deg=15.0)
    corpo = [(0.0, 0.0)] + pts
    perp = math.radians(rumo) + math.pi / 2.0
    caminho = ([(-0.0077, 0.0)] + corpo
               + [(x + 0.0077 * math.cos(perp), y + 0.0077 * math.sin(perp))])
    m = mede(_Caminho(caminho))
    assert m['giro_deg'] < 130.0, (
        f"giro medido {m['giro_deg']:.0f}° — os tocos de ponta voltaram a "
        'contar como virada')


def test_toco_nao_encurta_o_caminho():
    """Costurar o toco fora é medida de ÂNGULO, não amputação.

    `comprimento` e `desvio` continuam saindo dos pontos crus: o robô percorre
    o toco, ele só não define rumo.
    """
    reto = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (1.0050, 0.0)]
    m = mede(_Caminho(reto))
    assert abs(m['comprimento'] - 1.005) < 1e-6, m['comprimento']


class _PoseComRumo:
    def __init__(self, x, y, rumo):
        self.pose = type('p', (), {
            'position': type('q', (), {'x': x, 'y': y}),
            'orientation': type('o', (), {
                'x': 0.0, 'y': 0.0,
                'z': math.sin(rumo / 2.0), 'w': math.cos(rumo / 2.0)})})


class _CaminhoComRumo:
    def __init__(self, pts_rumos):
        self.poses = [_PoseComRumo(x, y, r) for x, y, r in pts_rumos]


def test_cuspide_rasa_e_achada_pela_pose():
    """Inversão que a geometria não vê, a orientação vê.

    Caso real `lado_1m` com raio 0,25: a pose diz `+----------------+` — um
    passo à frente, 16 de ré, um à frente. Nas duas cúspides a dobra geométrica
    mede 147°, passa por baixo do limiar de 150°, e as duas sumiam. Resultado
    relatado: `inv = 0` e **437°** de giro num caminho de 1,19 m.
    """
    # Ré = a pose aponta para um lado e o robô se desloca para o outro. O rumo
    # VARIA ao longo do arco, como no caminho real (faixa de yaw de 115°): é
    # justamente em caminho curvo que a cúspide fica rasa e a geometria falha.
    passo, giro = 0.07, math.radians(-10.0)
    x = y = rumo = 0.0
    pts = [(x, y, rumo)]
    x, y = x + passo * math.cos(rumo), y + passo * math.sin(rumo)
    pts.append((x, y, rumo))                                    # 1 à frente
    for _ in range(8):                                          # 8 de ré
        meio = rumo + giro / 2.0
        x, y = x - passo * math.cos(meio), y - passo * math.sin(meio)
        rumo += giro
        pts.append((x, y, rumo))
    x, y = x + passo * math.cos(rumo), y + passo * math.sin(rumo)
    pts.append((x, y, rumo))                                    # 1 à frente
    m = mede(_CaminhoComRumo(pts))
    assert m['inversoes'] == 2, (
        f"inversões medidas: {m['inversoes']} — a cúspide rasa voltou a passar "
        'por baixo do limiar geométrico')
    # O arco vira 80° de verdade; medem-se 70°, porque os dois trechos de um
    # passo só nas pontas são curtos demais para a régua e entram em `curt`.
    # O que este limite pega é o defeito: sem a correção entravam 2 x ~147° das
    # dobras e o giro passava de 370°.
    assert m['giro_deg'] < 120.0, (
        f"giro medido {m['giro_deg']:.0f}° — a inversão está contando como "
        'virada de novo')


def test_sem_orientacao_cai_na_geometria():
    """Theta* devolve o caminho com orientação zerada (faixa de yaw 0,0° em 144
    pontos, medido em 29-07). Sem pose que consultar, vale a dobra geométrica —
    e não custa nada: planner só-para-frente não tem cúspide para achar.
    """
    m = mede(caminho_com_cuspide())        # _Caminho, sem orientation
    assert m['inversoes'] == 2
