"""Testes da lei de seguimento de caminho (decisão 008, fatia A).

Cada teste trava uma propriedade que veio de um DEFEITO MEDIDO, não de gosto.
Se um destes cair, é um defeito conhecido voltando.

Fatia A cobre caminho SEM cúspide: carrot, lookahead e o teto de velocidade
pela curva. Cúspide e ré são a fatia B.
"""
import math

import pytest

from robot_motion.lei_de_seguimento import (
    carrot,
    curvatura_adiante,
    indice_mais_proximo,
    lookahead_de,
    rumo_para,
    velocidade_de_seguimento,
)

# Números do perfil pessimista de 29-07, que é o que o robô provavelmente é.
RAIO_MIN = 0.46
WZ_MAX = 1.0
V_MAX = 0.5
A_LIN = 0.3


def reta(n=20, passo=0.1):
    return [(i * passo, 0.0) for i in range(n)]


def arco(raio, total_deg=90.0, passo_deg=5.0):
    pts, ang = [], 0.0
    while ang <= math.radians(total_deg):
        pts.append((raio * math.sin(ang), raio * (1.0 - math.cos(ang))))
        ang += math.radians(passo_deg)
    return pts


# --------------------------------------------------------------- lookahead

def test_lookahead_sai_do_raio_minimo_nao_de_numero_solto():
    """O lookahead é derivado, não escolhido.

    É o parâmetro central deste nó, como o `a_dec` é o da movimentação. Fixá-lo
    num número solto o transformaria em mais um valor herdado sem justificativa
    — que é exatamente como a bitola (0,32) e o raio de roda (0,0825) chegaram
    errados até 29-07. Amarrado ao raio mínimo, ele acompanha a máquina quando a
    zona morta for medida.
    """
    assert lookahead_de(RAIO_MIN, fator=1.5) == pytest.approx(0.69)
    # dobrou o raio da máquina, dobrou o lookahead
    assert lookahead_de(2 * RAIO_MIN, fator=1.5) == pytest.approx(1.38)


def test_lookahead_tem_piso():
    """Raio pequeno não pode produzir lookahead que mira quase nos pés.

    Carrot colado no robô faz o rumo alvo oscilar com qualquer ruído de pose —
    é o mecanismo do ciclo-limite de rumo, o mesmo S de 27-07 entrando por
    outra porta.
    """
    assert lookahead_de(0.01, fator=1.5, piso=0.30) == pytest.approx(0.30)


# ------------------------------------------------------------------ carrot

def test_carrot_anda_pelo_ARCO_do_caminho_nao_pela_linha_reta():
    """O carrot segue o comprimento do caminho, não a distância em linha reta.

    Medido pelo carrot em linha reta: numa curva fechada, o ponto a 0,7 m de
    distância EUCLIDIANA pode estar muito mais à frente no caminho, e o robô
    corta a curva por dentro — que é como se raspa a quina de um vão.
    """
    # Arco LONGO (180°) e passo fino, de propósito: num arco de 90° a corda e o
    # comprimento ainda ficam perto o bastante para as duas contas darem quase
    # o mesmo, e o teste passava com a conta errada. Aqui, para um lookahead de
    # 0,69 m, o arco dá 0,69 e a corda daria ~0,78 — separados com folga.
    pts = arco(0.46, 180.0, passo_deg=2.0)
    _, alvo = carrot(pts, 0, 0.69)
    percorrido = 0.0
    for a, b in zip(pts, pts[1:]):
        percorrido += math.hypot(b[0] - a[0], b[1] - a[1])
        if b == alvo:
            break
    assert percorrido == pytest.approx(0.69, abs=0.03), (
        f'carrot a {percorrido:.3f} m de ARCO — se deu ~0,78, ele voltou a '
        'medir pela linha reta e o robô corta a curva por dentro')


def test_carrot_no_fim_do_caminho_devolve_o_ultimo_ponto():
    pts = reta(10, 0.1)          # 0,9 m de caminho
    i, alvo = carrot(pts, 0, 5.0)
    assert alvo == pts[-1]
    assert i == len(pts) - 1


def test_indice_mais_proximo_acha_onde_o_robo_esta():
    pts = reta(20, 0.1)
    assert indice_mais_proximo(pts, 0.52, 0.03) == 5


def test_rumo_para_o_carrot():
    assert rumo_para(0.0, 0.0, (1.0, 1.0)) == pytest.approx(math.pi / 4)


# ------------------------------------------- o teto de velocidade pela curva

def test_velocidade_tem_TETO_PELA_CURVA():
    """O defeito nº 2 da sessão de 28-07, e o que produziu o balão.

    `raio = v / wz`. Com `wz_max` de 1,0 rad/s e a linear no teto, o robô é
    obrigado a descrever um arco de `v/wz_max` — foi assim que um alvo a 0,43 m
    custou 3,66 m de caminho. A linear tem que ceder para a curva caber.
    """
    v = velocidade_de_seguimento(
        dist_ao_fim=10.0, raio_da_curva=0.30,
        v_max=V_MAX, a_lin=A_LIN, wz_max=WZ_MAX)
    assert v <= 0.30 * WZ_MAX + 1e-9, (
        f'v={v:.3f} exige raio maior que os 0,30 m da curva — o balão voltou')


def test_reta_longa_anda_no_teto():
    """Sem curva e sem fim à vista, não há razão para ir devagar."""
    v = velocidade_de_seguimento(
        dist_ao_fim=10.0, raio_da_curva=float('inf'),
        v_max=V_MAX, a_lin=A_LIN, wz_max=WZ_MAX)
    assert v == pytest.approx(V_MAX)


def test_freia_pela_distancia_ate_o_fim():
    """Mesma lei da decisão 005, agora em distância: `v = sqrt(2·a·d)`.

    Mesmo princípio, mesmo tipo de parâmetro físico — não é ganho ajustado.
    """
    v = velocidade_de_seguimento(
        dist_ao_fim=0.10, raio_da_curva=float('inf'),
        v_max=V_MAX, a_lin=A_LIN, wz_max=WZ_MAX)
    assert v == pytest.approx(math.sqrt(2 * A_LIN * 0.10))


def test_curvatura_adiante_ve_a_curva_ANTES_de_entrar_nela():
    """Olhar só onde o robô está faz ele chegar rápido demais na curva.

    A curvatura é medida na janela do lookahead à frente, não no ponto atual:
    frear em cima da curva é frear tarde, e a lei de frenagem da 005 existe
    justamente porque a máquina tem distância de frenagem.
    """
    pts = reta(10, 0.1) + [(1.0 + 0.46 * math.sin(a), 0.46 * (1 - math.cos(a)))
                           for a in [math.radians(g) for g in range(5, 95, 5)]]
    # robô no começo da reta, curva de 0,46 m começando a 1,0 m dali
    r = curvatura_adiante(pts, 0, janela=1.5)
    assert r < 0.7, f'raio visto {r:.2f} — a curva à frente passou despercebida'


def test_curvatura_adiante_em_reta_e_infinita():
    assert math.isinf(curvatura_adiante(reta(20, 0.1), 0, janela=1.0))


# ------------------------------------------------ fatia B: a ré por gatilho
#
# Decisão 009: o plano não dá ré (Dubins), e quem recua é o seguidor, disparado
# por SINTOMA — o robô não está progredindo — e não por geometria que prevê que
# ele não vai progredir. O gatilho geométrico da decisão 007 foi reprovado duas
# vezes com dado, pelo ciclo "ré e anda".

from robot_motion.lei_de_seguimento import (          # noqa: E402
    ProgressoDeAvanco,
    orcamento_de_re,
    re_esgotada,
)


def test_avanco_normal_nao_dispara_re():
    """Robô andando não recua. O óbvio, travado: é o falso positivo que
    produziria o vai-e-volta que a 007 morreu de ter."""
    p = ProgressoDeAvanco(parado_s=1.5, avanco_min=0.05)
    t, d = 0.0, 3.0
    disparou = False
    while t < 10.0:
        d -= 0.02                 # 0,4 m/s a 20 Hz: progride
        t += 0.05
        disparou |= p.atualiza(t, d)
    assert not disparou


def test_dispara_so_DEPOIS_de_parado_o_tempo_todo():
    """O gatilho é tardio de propósito.

    Sintoma dispara raro e tarde; geometria dispara cedo e sempre — e foi a
    geometria que produziu 5 entradas em ré e 1,85 m de caminho para um alvo a
    0,40 m, na bancada de 29-07. Aqui: 1,5 s sem progredir, nem um tick antes.
    """
    p = ProgressoDeAvanco(parado_s=1.5, avanco_min=0.05)
    t, d = 0.0, 1.0
    assert not p.atualiza(t, d)
    primeiro = None
    while t < 5.0:
        t += 0.05                 # distância NÃO cai: emperrado
        if p.atualiza(t, d) and primeiro is None:
            primeiro = t
    assert primeiro == pytest.approx(1.55, abs=0.06), (
        f'disparou em {primeiro} s — o gatilho tem que ser tardio')


def test_progresso_lento_mas_real_nao_dispara():
    """Chegar devagar não é estar preso.

    A aproximação freia por `sqrt(2·a·d)` e fica lenta de propósito perto do
    fim; confundir isso com travamento faria o robô dar ré JUSTO ao chegar.

    0,10 m/s é o "devagar" que este robô CONSEGUE. Escrevi este teste a 3 cm/s
    primeiro e ele reprovou a lei — mas 3 cm/s é abaixo do piso de linear que a
    zona morta obriga (~0,33 m/s no perfil pessimista): nessa faixa o robô não
    anda devagar, ele **não anda**. Ver o teste da taxa mínima logo abaixo.
    """
    p = ProgressoDeAvanco(parado_s=1.5, avanco_min=0.05)
    t, d = 0.0, 0.40
    disparou = False
    while d > 0.02:               # só a APROXIMAÇÃO: parar em cima do alvo é
        d -= 0.005                # assunto da chegada, não do gatilho — lá o
        t += 0.05                 # seguidor desarma o detector (`reinicia`)
        disparou |= p.atualiza(t, d)
    assert not disparou, 'aproximação a 0,10 m/s foi lida como travamento'


def test_chegada_desarma_o_detector():
    """Parado EM CIMA do alvo não é travamento — mas o detector não sabe disso.

    Ele só vê distância que não cai, e no alvo ela não cai mesmo. Quem sabe que
    chegou é o seguidor, e é ele que desarma. Sem isso o robô chega, fica, e
    depois de 1,5 s dá ré para longe do ponto onde acabou de chegar.
    """
    p = ProgressoDeAvanco(parado_s=1.5, avanco_min=0.05)
    t = 0.0
    for _ in range(60):           # 3 s parado no alvo
        t += 0.05
        p.atualiza(t, 0.01)
    assert p.atualiza(t, 0.01), 'sem desarme, o detector acusa — e deve acusar'
    p.reinicia()
    assert not p.atualiza(t + 0.05, 0.01), 'depois de reiniciar, ele cala'


def test_a_taxa_minima_implicita_tem_folga_contra_o_piso_de_linear():
    """`avanco_min / parado_s` é uma velocidade mínima disfarçada.

    Com 0,05 m em 1,5 s, quem se aproximar a menos de 3,3 cm/s é declarado
    preso. Isso é seguro só porque o piso de linear que a zona morta obriga é
    ~10x maior — abaixo dele o robô não anda de verdade, então "aproximação
    lenta" não é um estado que esta máquina ocupa.

    Se a zona morta medida na bancada derrubar muito o piso, este par de
    números volta à mesa: a folga é o que sustenta o gatilho.
    """
    taxa_minima = 0.05 / 1.5
    v_piso_pessimista = 0.15 + 1.0 * 0.270 / 2 + 0.05
    assert taxa_minima < v_piso_pessimista / 5.0, (
        f'taxa mínima {taxa_minima:.3f} m/s perto demais do piso '
        f'{v_piso_pessimista:.3f} m/s — o gatilho vira falso positivo')


def test_o_relogio_zera_quando_o_robo_volta_a_andar():
    p = ProgressoDeAvanco(parado_s=1.5, avanco_min=0.05)
    t, d = 0.0, 1.0
    for _ in range(20):           # 1,0 s emperrado
        t += 0.05
        p.atualiza(t, d)
    d -= 0.20                     # destravou
    t += 0.05
    p.atualiza(t, d)
    for _ in range(20):           # mais 1,0 s emperrado: total 2 s, mas
        t += 0.05                 # nenhum trecho contínuo de 1,5 s
        assert not p.atualiza(t, d)


def test_orcamento_da_re_e_obrigatorio_e_curto():
    """Sem sensor traseiro a ré é CEGA (decisão 009).

    O Mid-360 é 360° e vai ver atrás, mas ainda não está no modelo. Enquanto
    não estiver, recuar é apostar que não tem nada lá — e aposta cega tem que
    ser curta.
    """
    assert orcamento_de_re(vao_traseiro=None) == pytest.approx(0.30)


def test_com_vao_medido_a_re_pode_ir_mais_longe_mas_com_folga():
    """Quando o lidar entrar, o orçamento passa a sair de metros medidos."""
    assert orcamento_de_re(vao_traseiro=1.20, folga=0.30) == pytest.approx(0.90)


def test_vao_menor_que_a_folga_proibe_a_re():
    """Parede atrás = não recua. Zero, não 'um pouquinho'."""
    assert orcamento_de_re(vao_traseiro=0.20, folga=0.30) == 0.0


def test_re_para_ao_gastar_o_orcamento():
    assert not re_esgotada(recuado=0.10, orcamento=0.30, t_na_re=1.0, teto_s=8.0)
    assert re_esgotada(recuado=0.31, orcamento=0.30, t_na_re=1.0, teto_s=8.0)


def test_re_para_no_teto_de_TEMPO_mesmo_sem_ter_recuado():
    """A ré também é cega para si mesma: se a pose não muda, ela nunca gastaria
    o orçamento em metros e recuaria para sempre. O teto de tempo é a defesa —
    é o mesmo BO-3 (comando saindo, robô parado) visto de outro ângulo."""
    assert re_esgotada(recuado=0.0, orcamento=0.30, t_na_re=8.1, teto_s=8.0)
