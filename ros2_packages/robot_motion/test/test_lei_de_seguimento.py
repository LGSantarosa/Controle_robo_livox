"""Testes da lei de seguimento de caminho (decisão 008, fatia A).

Cada teste trava uma propriedade que veio de um DEFEITO MEDIDO, não de gosto.
Se um destes cair, é um defeito conhecido voltando.

Fatia A cobre caminho SEM cúspide: carrot, lookahead e o teto de velocidade
pela curva. Cúspide e ré são a fatia B.
"""
import math

import pytest

from robot_motion.lei_de_seguimento import (
    CorrecaoDeDesvio,
    MiraAdaptativa,
    PassagemEstreita,
    alvo_estavel_de_passagem,
    carrot,
    correcao_de_desvio,
    curvatura_adiante,
    desvio_da_corda,
    desvio_lateral,
    folga_radial,
    indice_mais_proximo,
    lookahead_de,
    mudanca_de_rumo_adiante,
    orcamento_de_re,
    passagens_estreitas,
    rumo_com_desvio,
    rumo_local_do_caminho,
    rumo_para,
    vao_no_corredor_frontal,
    vao_no_corredor_traseiro,
    velocidade_de_seguimento,
)

# Números do perfil pessimista de 29-07, que é o que o robô provavelmente é.
RAIO_MIN = 0.46
WZ_MAX = 1.0
V_MAX = 0.5
A_LIN = 0.3


def reta(n=20, passo=0.1):
    return [(i * passo, 0.0) for i in range(n)]


def test_folga_radial_ignora_leitura_invalida_e_pega_a_quina_mais_perto():
    assert folga_radial([None, float('nan'), float('inf'), -1.0,
                         0.52, 0.341, 0.80], alcance_max=10.0) == 0.341
    assert math.isinf(folga_radial([None, float('inf'), 0.0]))


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


# ------------------------------------- gargalo estável (20-08, repetibilidade)

def _grade_com_porta(x_parede=3.0, y0=1.6, y1=2.4,
                     largura_m=6.0, altura_m=4.0, res=0.05):
    w, h = round(largura_m / res), round(altura_m / res)
    dados = [0] * (w * h)
    c0, c1 = round((x_parede - 0.10) / res), round((x_parede + 0.10) / res)
    for lin in range(h):
        y = (lin + 0.5) * res
        if y0 <= y <= y1:
            continue
        for col in range(c0, c1):
            dados[lin * w + col] = 100
    return dados, w, h, res


def test_detecta_a_porta_pelo_mapa_sem_coordenada_marcada():
    dados, w, h, res = _grade_com_porta()
    caminho = [(0.5 + 0.05 * i, 2.0) for i in range(101)]
    portas = passagens_estreitas(caminho, dados, w, h, res,
                                 largura_min=0.55, largura_max=1.10)
    assert len(portas) == 1
    p = portas[0]
    assert caminho[p.centro][0] == pytest.approx(3.0, abs=0.10)
    assert p.largura == pytest.approx(0.80, abs=0.08)
    assert abs(rumo_local_do_caminho(caminho, p.centro)) < math.radians(1.0)


def test_espaco_aberto_nao_inventa_porta():
    w, h, res = 120, 80, 0.05
    caminho = [(0.5 + 0.05 * i, 2.0) for i in range(101)]
    assert passagens_estreitas(caminho, [0] * (w * h), w, h, res) == []


def test_alvo_do_gargalo_primeiro_centraliza_se_o_atalho_nao_cabe():
    caminho = [(0.1 * i, 0.0) for i in range(41)]
    porta = PassagemEstreita(19, 20, 21, 0.80)
    alvo, fase = alvo_estavel_de_passagem(
        caminho, porta, x=1.0, y=0.50, rumo_atual=0.0, saida=0.60,
        meia_largura=0.2275, margem=0.03)
    assert fase == 'centro'
    assert alvo == caminho[porta.centro]


def test_alvo_do_gargalo_e_um_eixo_FIXO_quando_a_reta_cabe():
    caminho = [(0.1 * i, 0.0) for i in range(41)]
    porta = PassagemEstreita(19, 20, 21, 0.80)
    alvo1, fase1 = alvo_estavel_de_passagem(
        caminho, porta, x=1.0, y=0.05, rumo_atual=0.0, saida=0.60)
    alvo2, fase2 = alvo_estavel_de_passagem(
        caminho, porta, x=1.4, y=0.04, rumo_atual=math.radians(4.0),
        saida=0.60)
    assert fase1 == fase2 == 'eixo'
    assert alvo1 == pytest.approx((2.60, 0.0))
    assert alvo2 == pytest.approx(alvo1)  # pose mudou; referência não


def test_alvo_estavel_funciona_na_volta_pela_mesma_porta():
    caminho = [(4.0 - 0.1 * i, 0.0) for i in range(41)]
    porta = PassagemEstreita(19, 20, 21, 0.80)
    alvo, fase = alvo_estavel_de_passagem(
        caminho, porta, x=3.0, y=0.05, rumo_atual=math.pi, saida=0.60)
    assert fase == 'eixo'
    assert alvo == pytest.approx((1.40, 0.0))


def test_gargalo_nao_libera_eixo_com_rumo_fisico_ainda_torto():
    caminho = [(0.1 * i, 0.0) for i in range(41)]
    porta = PassagemEstreita(19, 20, 21, 0.80)
    alvo, fase = alvo_estavel_de_passagem(
        caminho, porta, x=1.3, y=0.03,
        rumo_atual=math.radians(53.0), saida=1.0)
    assert fase == 'centro'
    assert alvo == caminho[porta.centro]


def test_gargalo_libera_quando_posicao_e_rumo_estao_alinhados():
    caminho = [(0.1 * i, 0.0) for i in range(41)]
    porta = PassagemEstreita(19, 20, 21, 0.80)
    alvo, fase = alvo_estavel_de_passagem(
        caminho, porta, x=1.3, y=0.07,
        rumo_atual=math.radians(9.0), saida=1.0)
    assert fase == 'eixo'
    assert alvo == pytest.approx((3.0, 0.0))


def test_gargalo_nao_volta_ao_centro_so_porque_o_rumo_oscilou():
    """RUMO torto no meio da travessia não desfaz o latch — só desvio desfaz.

    ⚠️ Este teste EXIGIA `y=0,20` e fase `eixo` até 20-08, quando o latch era
    permanente. A medida derrubou aquela versão: com 0,20 m de desvio o corpo
    invade a ombreira (a folga desta porta é 0,1425 m), e foi assim que o robô
    chegou à soleira em `x=7,20` num vão que termina em 7,29. O que continua
    valendo, e é o que este teste guarda, é a razão pela qual o latch existe:
    oscilação de RUMO no meio do vão não pode devolver a referência ao centro.
    """
    caminho = [(0.1 * i, 0.0) for i in range(41)]
    porta = PassagemEstreita(19, 20, 21, 0.80)
    alvo, fase = alvo_estavel_de_passagem(
        caminho, porta, x=1.5, y=0.10,
        rumo_atual=math.radians(30.0), saida=1.0,
        eixo_comprometido=True)
    assert fase == 'eixo'
    assert alvo == pytest.approx((3.0, 0.0))


# ------------------------------------------- a MIRA ADAPTATIVA (040, 14-08)
#
# Vem do robô 1 (`Controle_robo_web`), que mediu antes: carrot de 0,6 m
# amplifica ruído de pose (12 cm = 12°) -> 184 giros no lugar, zigue-zague em
# corredor; a 1,5 m os mesmos 12 cm viram 4,6°. Aqui a mira era 0,37 m — mais
# curta ainda — e a corrida A de 14-08 mediu o mesmo efeito por outro caminho:
# salto do plano de 15,1 cm -> 22° de referência, amplitude p90 de 20,0°.

CURTO, LONGO = 0.37, 1.00


def test_desvio_da_corda_e_zero_em_reta():
    assert desvio_da_corda(reta(20, 0.1), 0, 1.0) == pytest.approx(0.0, abs=1e-9)


def test_desvio_da_corda_e_a_regua_da_geometria():
    """Os limiares saem DESTA tabela, não de varredura.

    ⚠️ Arcos de 180° de propósito: um arco de 90° em raio pequeno tem menos de
    1,0 m de comprimento, a corda acaba antes e o desvio sai artificialmente
    baixo (0,108 em vez de 0,293 no raio 0,37). Foi o que derrubou a primeira
    versão deste teste — e é um caso REAL, no fim do caminho.
    """
    assert desvio_da_corda(arco(0.37, 180.0, 2.0), 0, 1.0) == \
        pytest.approx(0.293, abs=0.01)      # o mais fechado que a máquina faz
    assert desvio_da_corda(arco(1.00, 180.0, 2.0), 0, 1.0) == \
        pytest.approx(0.125, abs=0.01)      # curva de verdade -> encolhe
    assert desvio_da_corda(arco(2.00, 180.0, 2.0), 0, 1.0) == \
        pytest.approx(0.068, abs=0.01)      # suave -> pode esticar


def test_estica_em_reta_e_encolhe_em_curva():
    m = MiraAdaptativa(CURTO, LONGO)
    assert m.passo(reta(40, 0.1), 0) == pytest.approx(LONGO)
    assert m.passo(arco(0.37, 180.0, 2.0), 0) == pytest.approx(CURTO)


def test_o_caminho_ACABANDO_nao_finge_ser_reta():
    """No fim do caminho a corda encurta e o desvio cai — mas ali o carrot
    esticado devolve o último ponto de qualquer jeito (é o destino), então
    esticar não corta nada. O que este teste trava é que não quebra."""
    curto_demais = arco(0.37, 90.0, 2.0)             # ~0,58 m de caminho
    m = MiraAdaptativa(CURTO, LONGO)
    assert m.passo(curto_demais, 0) in (CURTO, LONGO)


def test_a_mira_longa_derruba_a_amplificacao_do_ruido():
    """A conta que justifica o número, com o salto de plano medido em 14-08."""
    salto = 0.151                                   # max medido, plano suave
    assert math.degrees(math.atan2(salto, CURTO)) == pytest.approx(22.2, abs=0.3)
    assert math.degrees(math.atan2(salto, LONGO)) == pytest.approx(8.6, abs=0.3)


def test_HISTERESE_o_carrot_nao_fica_pulando_na_fronteira():
    """Sem isto o limite-ciclo do robô 1 volta por outra porta.

    Caminho cuja mudanca de rumo cai ENTRE 3 e 5 graus: quem ja esticou
    continua esticado, quem nao esticou continua curto. O mesmo caminho, dois
    estados, sem piscar por ruido angular do planner.
    """
    m = MiraAdaptativa(CURTO, LONGO, tol_estica=0.07, tol_encolhe=0.08)
    a = math.radians(4.0)
    meio = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0), (0.6, 0.0),
            (0.8, 0.2 * math.sin(a)),
            (1.0, 0.4 * math.sin(a))]
    mudanca = mudanca_de_rumo_adiante(meio, 0, LONGO)
    assert math.radians(3.0) < mudanca < math.radians(5.0)

    assert m.passo(meio, 0) == pytest.approx(CURTO)  # não estica
    m.passo(reta(40, 0.1), 0)                        # estica numa reta
    assert m.passo(meio, 0) == pytest.approx(LONGO)  # e AGUENTA no mesmo trecho


def test_curva_logo_depois_da_reta_ENCOLHE_antes_do_carrot_cortar_a_quina():
    """Regressao visual da porta em 19-08.

    Mesmo ja esticada, a mira nao pode atravessar a quina que aparece no fim
    do proximo metro. Os ~8 cm de desvio ficavam abaixo do limiar antigo de
    12 cm e o robo virava cedo para o carrot, em vez de cumprir a reta.
    """
    m = MiraAdaptativa(CURTO, LONGO)
    assert m.passo(reta(20, 0.1), 0) == pytest.approx(LONGO)
    reta_e_quina = [(0.0, 0.0), (0.91, 0.0), (0.91, 0.09), (0.91, 0.30)]
    assert desvio_da_corda(reta_e_quina, 0, LONGO) >= 0.08
    assert m.passo(reta_e_quina, 0) == pytest.approx(CURTO)


def test_mira_longa_SO_QUANDO_o_proximo_metro_inteiro_e_reto():
    """A curva futura nao pode puxar o robo enquanto ele cruza a porta."""
    caminho = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0), (0.6, 0.0),
               (0.8, 0.0), (0.9, 0.1), (0.9, 0.3)]
    assert mudanca_de_rumo_adiante(caminho, 0, 1.0) > math.radians(40.0)
    m = MiraAdaptativa(CURTO, LONGO)
    assert m.passo(caminho, 0, vao_frente=2.0) == pytest.approx(CURTO)


def test_meandro_suave_do_plano_nao_e_confundido_com_curva_da_porta():
    """Regressão do S visto no corredor em 20-08.

    Uma barriga suave de poucos graus é ruído de referência a filtrar. Já a
    quina da porta precisa continuar impondo a mira curta.
    """
    m = MiraAdaptativa(CURTO, LONGO, tol_estica=0.15, tol_encolhe=0.20,
                       rumo_estica=math.radians(15.0),
                       rumo_encolhe=math.radians(20.0))
    meandro = [(0.0, 0.0), (0.2, 0.00), (0.4, 0.02), (0.6, 0.04),
               (0.8, 0.05), (1.0, 0.05)]
    assert m.passo(meandro, 0, vao_frente=2.0) == pytest.approx(LONGO)

    porta = [(0.0, 0.0), (0.2, 0.0), (0.4, 0.0), (0.6, 0.0),
             (0.8, 0.0), (0.9, 0.1), (0.9, 0.3)]
    assert m.passo(porta, 0, vao_frente=2.0) == pytest.approx(CURTO)


def test_reta_de_um_metro_CONTINUA_com_mira_longa():
    caminho = reta(20, 0.1)
    assert mudanca_de_rumo_adiante(caminho, 0, 1.0) == pytest.approx(0.0)
    assert MiraAdaptativa(CURTO, LONGO).passo(caminho, 0, 2.0) == \
        pytest.approx(LONGO)


def test_o_gate_do_scan_encolhe_SEM_histerese():
    """Encolher é o lado seguro: hesitar perto de parede é o que não pode."""
    m = MiraAdaptativa(CURTO, LONGO, folga_min=0.60)
    m.passo(reta(40, 0.1), 0)                        # esticada
    assert m.passo(reta(40, 0.1), 0, vao_frente=0.30) == pytest.approx(CURTO)
    assert m.esticada is False


def test_espaco_livre_deixa_esticar():
    m = MiraAdaptativa(CURTO, LONGO, folga_min=0.60)
    assert m.passo(reta(40, 0.1), 0, vao_frente=2.0) == pytest.approx(LONGO)


def test_mira_sem_histerese_e_recusada_na_construcao():
    with pytest.raises(ValueError):
        MiraAdaptativa(CURTO, LONGO, tol_estica=0.10, tol_encolhe=0.10)
    with pytest.raises(ValueError):
        MiraAdaptativa(LONGO, CURTO)


# --------------------------------------- o corredor da FRENTE (gate do scan)

def test_vao_frontal_ve_o_que_esta_na_frente_descontando_o_para_choque():
    # feixe reto à frente a 1,0 m, para-choque a 0,25 m do centro
    d = vao_no_corredor_frontal([1.0], -0.0, 0.1, largura=0.46, avanco=0.25)
    assert d == pytest.approx(0.75)


def test_vao_frontal_ignora_o_que_esta_ATRAS():
    d = vao_no_corredor_frontal([1.0], math.pi, 0.1, largura=0.46, avanco=0.25)
    assert math.isinf(d)


def test_vao_frontal_ignora_o_que_passa_de_LADO_do_corredor():
    # feixe a 60°: y = 1,0·sen(60°) = 0,87 m, fora da meia-largura 0,23
    d = vao_no_corredor_frontal([1.0], math.radians(60.0), 0.1,
                                largura=0.46, avanco=0.25)
    assert math.isinf(d)


def test_vao_frontal_nunca_devolve_negativo():
    """Orçamento negativo somado com folga viraria permissão — igual à irmã."""
    d = vao_no_corredor_frontal([0.10], 0.0, 0.1, largura=0.46, avanco=0.25)
    assert d == 0.0


# --------------------------------------------------- o desvio lateral (039)
#
# O defeito medido em 14-08 no Gazebo, e antes dele em 13-08 no robô real:
# o PLANO cruza a porta quase centrado e o ROBÔ chega 11 cm para o lado,
# comendo 2/3 da margem do vão e disparando o reflexo contra parede mapeada.


def test_desvio_lateral_tem_SINAL_esquerda_positivo():
    """Sem sinal não há realimentação — módulo não sabe para que lado voltar."""
    pts = reta(20, 0.1)                       # caminho ao longo de +x
    assert desvio_lateral(pts, 5, 0.5, +0.10) == pytest.approx(+0.10, abs=1e-6)
    assert desvio_lateral(pts, 5, 0.5, -0.10) == pytest.approx(-0.10, abs=1e-6)


def test_desvio_e_medido_contra_o_SEGMENTO_nao_contra_o_vertice():
    """Ponto a ponto o caminho vem a ~5 cm; a distância ao vértice satura.

    Robô exatamente entre dois pontos do caminho, 10 cm fora: a distância ao
    vértice mais próximo daria 0,112 m e depende de onde ele está no passo. A
    distância ao segmento é 0,10 m, sempre.
    """
    pts = reta(20, 0.1)
    assert desvio_lateral(pts, 5, 0.55, 0.10) == pytest.approx(0.10, abs=1e-9)


def test_desvio_zero_em_cima_do_caminho():
    pts = reta(20, 0.1)
    assert desvio_lateral(pts, 5, 0.5, 0.0) == pytest.approx(0.0, abs=1e-9)


def test_a_lei_NASCE_NEUTRA_ganho_zero_e_o_comportamento_de_hoje():
    """Ligar o termo tem de ser reversível por parâmetro. Mesma regra da 038."""
    for e in (-0.20, -0.05, 0.0, 0.05, 0.20):
        assert rumo_com_desvio(0.7, e, 0.3, k_lat=0.0) == pytest.approx(0.7)


def test_erro_a_esquerda_pede_rumo_a_DIREITA():
    """O sinal errado aqui afasta o robô do caminho em vez de trazer."""
    assert rumo_com_desvio(0.0, +0.10, 0.30, k_lat=1.0) < 0.0
    assert rumo_com_desvio(0.0, -0.10, 0.30, k_lat=1.0) > 0.0


def test_a_correcao_e_a_de_STANLEY_atan_de_k_e_sobre_v():
    e, v, k = 0.10, 0.30, 1.0
    esperado = -math.atan2(k * e, v)
    assert rumo_com_desvio(0.0, e, v, k_lat=k) == pytest.approx(esperado)


def integra_erro(e0, v, k, ate_s, dt=0.0005):
    """Integra o erro lateral sob a correção, cinemática de bicicleta."""
    e, t = e0, 0.0
    while t < ate_s:
        delta = rumo_com_desvio(0.0, e, v, k_lat=k, v_ref=0.0)
        e += v * math.sin(delta) * dt          # rumo negativo reduz e>0
        t += dt
    return e


def test_o_erro_decai_com_constante_de_tempo_1_sobre_k():
    """A propriedade que ESCOLHE o ganho, e ela não depende da velocidade.

    ⚠️ Vale no limite de ângulo pequeno, que é onde a lei deve viver:

        ė = −v·sen(atan(k·e/v)) = −k·e / sqrt(1 + (k·e/v)²)

    O denominador é 1 só enquanto `k·e ≪ v`. Com erro de 10 cm a 0,20 m/s ele
    já vale 1,12 e o decaimento sai ~5% mais lento — não é defeito, é a lei
    saturando de propósito (é o mesmo mecanismo do teto de 30°).
    """
    for v in (0.20, 0.45):
        assert integra_erro(0.02, v, 1.0, ate_s=1.0) == \
            pytest.approx(0.02 / math.e, rel=0.02)


def test_o_decaimento_NAO_depende_da_velocidade():
    """É por isso que o termo tem `v` no denominador, e não é P puro.

    Perto da porta o robô anda devagar. Uma correção proporcional pura ficaria
    fraca justo onde ela precisa ser forte — que é o defeito de 14-08.
    """
    devagar = integra_erro(0.02, 0.20, 1.0, ate_s=1.0)
    rapido = integra_erro(0.02, 0.45, 1.0, ate_s=1.0)
    assert devagar == pytest.approx(rapido, rel=0.02)


# ------------------------------------- o LIMITE DE TAXA (corrida B, 14-08)
#
# A corrida B reprovou a correção SEM limite: amplitude p90 da referência de
# rumo 20,0° -> 39,8°, pico 50,3° -> 88,1°, período 1,00 s -> 0,70 s. A causa
# não é o ganho: é o plano que salta de lado no replanejamento (p90 5,8 cm,
# max 15,1 cm), e um degrau de posição virava degrau de rumo no mesmo ciclo.

DT = 0.1


def test_o_degrau_do_replanejamento_NAO_vira_degrau_de_rumo():
    """O defeito exato da corrida B, reproduzido: 15 cm de salto de plano."""
    c = CorrecaoDeDesvio(k_lat=0.5, taxa_max=math.radians(15.0))
    c.passo(0.0, 0.0, 0.30, DT)                      # em cima do caminho
    rumo = c.passo(0.0, 0.151, 0.30, DT)             # o plano saltou 15,1 cm
    assert abs(rumo) <= math.radians(15.0) * DT + 1e-9
    assert abs(rumo) < math.radians(2.0)             # ~1,5°, não um tranco


def test_sem_limite_o_mesmo_degrau_daria_um_TRANCO():
    """A testemunha do defeito: é isto que a corrida B fez."""
    sem = rumo_com_desvio(0.0, 0.151, 0.30, k_lat=0.5)
    assert abs(sem) > math.radians(12.0)


def test_a_correcao_CHEGA_no_alvo_so_que_devagar():
    """Limitar taxa não pode virar limitar autoridade."""
    c = CorrecaoDeDesvio(k_lat=0.5, taxa_max=math.radians(15.0))
    alvo = correcao_de_desvio(0.151, 0.30, k_lat=0.5)
    rumo = None
    for _ in range(int(3.0 / DT)):                   # 3 s
        rumo = c.passo(0.0, 0.151, 0.30, DT)
    assert -rumo == pytest.approx(alvo, rel=0.02)


def test_o_pior_caso_se_completa_em_dois_segundos():
    """O teto é 30°; a 15°/s isso é 2 s — 4x o tempo morto da placa (0,5 s)."""
    c = CorrecaoDeDesvio(k_lat=5.0, taxa_max=math.radians(15.0))
    for _ in range(int(2.0 / DT) + 1):
        rumo = c.passo(0.0, 1.0, 0.30, DT)
    assert abs(rumo) == pytest.approx(math.radians(30.0), rel=0.01)


def test_a_taxa_pedida_cabe_no_que_a_maquina_fecha():
    """15°/s contra os ~57°/s de `wz_max` 1,0 rad/s: folga de 3,8x."""
    assert math.radians(15.0) < 0.30 * 1.0


def test_ganho_zero_continua_neutro_mesmo_com_o_limite():
    c = CorrecaoDeDesvio(k_lat=0.0)
    for e in (-0.20, 0.0, 0.20):
        assert c.passo(0.7, e, 0.3, DT) == pytest.approx(0.7)


def test_reset_zera_correcao_acumulada():
    """Correção velha em caminho novo é comando sem dono (a ré do nada, 13-08)."""
    c = CorrecaoDeDesvio(k_lat=0.5, taxa_max=math.radians(15.0))
    for _ in range(20):
        c.passo(0.0, 0.15, 0.30, DT)
    assert c.corr != 0.0
    c.reset()
    assert c.passo(0.7, 0.0, 0.30, DT) == pytest.approx(0.7)


def test_taxa_max_invalida_e_recusada_na_construcao():
    with pytest.raises(ValueError):
        CorrecaoDeDesvio(k_lat=0.5, taxa_max=0.0)


def test_com_o_erro_medido_na_porta_o_ganho_1_resolve_em_um_metro():
    """A régua do conserto: 11 cm de desvio (14-08) têm de virar ~1 cm.

    A 0,30 m/s, 3 s de correção são 0,90 m de caminho — cabe folgado na reta
    de aproximação da porta.
    """
    assert integra_erro(0.11, 0.30, 1.0, ate_s=3.0) < 0.015


def test_a_correcao_tem_TETO_e_nao_vira_manobra():
    """Acima de 30° não é mais correção: manobra tem dono (o pivô, a ré)."""
    corr = rumo_com_desvio(0.0, 5.0, 0.30, k_lat=1.0)
    assert corr == pytest.approx(-math.radians(30.0))


def test_velocidade_quase_zero_NAO_pede_giro_de_90_graus():
    """Sem o piso `v_ref` o termo explode e o robô gira parado em cima da linha."""
    corr = rumo_com_desvio(0.0, 0.10, 0.0, k_lat=1.0)
    assert abs(corr) == pytest.approx(math.atan2(0.10, 0.20))
    assert abs(corr) < math.radians(30.0)


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


def test_curvatura_curta_na_porta_NAO_vira_reta_por_falta_de_amostras():
    """Regressao de 19-08: mira de 0,37 m via a quina como `inf`.

    A reamostragem antiga era fixa em 20 cm e descartava o segmento que
    cruzava o fim da janela. Sobravam dois pontos, insuficientes para medir a
    curva, e o robo atacava a porta a 0,5 m/s enquanto ja pedia a guinada.
    """
    quina = [(0.0, 0.0), (0.20, 0.0), (0.20, 0.40)]
    raio = curvatura_adiante(quina, 0, janela=0.37)
    assert not math.isinf(raio)
    assert raio < 0.30


# ------------------------------------------------ fatia B: a ré por gatilho
#
# Decisão 009: o plano não dá ré (Dubins), e quem recua é o seguidor, disparado
# por SINTOMA — o robô não está progredindo — e não por geometria que prevê que
# ele não vai progredir. O gatilho geométrico da decisão 007 foi reprovado duas
# vezes com dado, pelo ciclo "ré e anda".

from robot_motion.lei_de_seguimento import (          # noqa: E402
    ProgressoDeAvanco,
    orcamento_de_re,
    vao_no_corredor_traseiro,
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

    ⚠️ E ela voltou, em 12-08 (decisão 020): a zona morta medida (0,0178 contra
    o chute de 0,15) derrubou o piso de 0,335 para 0,203, e a folga caiu de 10x
    para 6,1x. Ainda passa. Por isso o piso é LIDO do arquivo do robô e não
    copiado para cá — copiado, ele passaria a mentir na próxima vez em vez de
    derrubar o teste.
    """
    import os
    import re

    yaml_robo = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'config', 'movimentacao.yaml')
    with open(yaml_robo) as f:
        conf = {m.group(1): float(m.group(2))
                for m in (re.match(r'^\s*(\w+):\s*([-\d.]+)\s*$',
                                   linha.split('#')[0])
                          for linha in f) if m}

    taxa_minima = 0.05 / 1.5
    v_piso = (conf['zona_morta'] + conf['wz_max'] * conf['bitola'] / 2.0
              + conf['margem_piso'])
    assert taxa_minima < v_piso / 5.0, (
        f'taxa mínima {taxa_minima:.3f} m/s perto demais do piso '
        f'{v_piso:.3f} m/s — o gatilho vira falso positivo. O piso caiu porque '
        'a zona morta do robô caiu; `re_avanco_min`/`re_parado_s` precisam '
        'acompanhar (lei_de_seguimento / path_follower).')


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


# ----------------------------------------------------- fatia C: a chegada
#
# Duas lições da decisão 006, as duas medidas e as duas caras:
#
#   1. não existe "chegar devagarinho" — abaixo do mínimo viável a placa engole
#      o comando e o robô para LONGE do ponto achando que chegou (BO-3
#      disfarçado de sucesso);
#   2. chegando, se ele continua girando, se arrasta para fora: 0,06 m viraram
#      0,27 m na sessão de 27-07.

from robot_motion.lei_de_seguimento import (          # noqa: E402
    chegou,
    comando_de_parada,
    raio_de_chegada_minimo,
)

V_PISO = 0.15 + 1.0 * 0.270 / 2 + 0.05     # perfil pessimista: 0,335 m/s


def test_raio_de_chegada_minimo_e_a_distancia_de_PARADA():
    """Raio menor que a distância de parada faz o robô ORBITAR o ponto.

    Ele não consegue ir mais devagar que o piso, então entra no raio com
    `v_piso` e precisa de `v_piso²/(2·a_lin)` para parar. Se o raio for menor
    que isso, ele atravessa, sai do outro lado, volta — para sempre.
    """
    assert raio_de_chegada_minimo(V_PISO, A_LIN) == pytest.approx(
        V_PISO ** 2 / (2 * A_LIN))
    # com os números de hoje isso dá quase 19 cm — nada desprezível
    assert raio_de_chegada_minimo(V_PISO, A_LIN) > 0.18


def test_piso_maior_exige_raio_de_chegada_maior():
    """A zona morta não encarece só a manobra: ela encarece a PRECISÃO.

    É o argumento que o dono vai querer quando escolher entre pivotar e não:
    piso alto = chegada grosseira, e não há ganho que conserte isso.
    """
    r_baixo = raio_de_chegada_minimo(0.20, A_LIN)
    r_alto = raio_de_chegada_minimo(0.40, A_LIN)
    assert r_alto > 3.5 * r_baixo, 'a precisão piora com o QUADRADO do piso'


def test_chegou_e_so_a_distancia():
    assert chegou(0.10, raio=0.15)
    assert not chegou(0.20, raio=0.15)


def test_parada_zera_o_GIRO_tambem_nao_so_a_linear():
    """O defeito nº 4 de 27-07: chegando, ele continuava girando para acertar o
    rumo e se ARRASTAVA para fora do ponto — 0,06 m viraram 0,27 m.

    Chegou é chegou: para tudo. Rumo no ponto de chegada não é requisito deste
    seguidor, e persegui-lo custa a própria chegada.
    """
    v, wz = comando_de_parada()
    assert v == 0.0
    assert wz == 0.0, 'girar depois de chegar arrasta o robô para fora do ponto'


# ---------------------------------------------------------------------------
# O VÃO TRASEIRO — decisão 025
#
# A ré deixa de ser cega. O que estes testes protegem não é o número: é a
# FORMA da medida. A versão por setor angular já pôs um robô em cima de um
# obstáculo atrás, e o modo de falhar dela é passar em teste de caso feliz.
# ---------------------------------------------------------------------------

A0_GRAUS, INC_GRAUS = -180.0, 1.0


def _varredura(pontos, n=360):
    """Varredura de `n` feixes a 1° com obstáculos em (ÂNGULO°, raio).

    ⚠️ O índice sai do ÂNGULO, não da posição na lista. A primeira versão
    deste helper indexava direto (`ranges[graus]`) e, com `angulo_min` em
    −180°, punha em 0° (a FRENTE) o obstáculo que o teste dizia estar em 180°
    (atrás). Os testes reprovavam a função por um defeito do próprio teste.
    """
    ranges = [float('inf')] * n
    for graus, r in pontos:
        ranges[int(round((graus - A0_GRAUS) / INC_GRAUS)) % n] = r
    return ranges, math.radians(A0_GRAUS), math.radians(INC_GRAUS)


def test_corredor_vazio_da_vao_infinito():
    r, a0, inc = _varredura([])
    assert math.isinf(vao_no_corredor_traseiro(r, a0, inc, 0.50, 0.28))


def test_o_que_esta_atras_no_corredor_desconta_o_para_choque():
    """Obstáculo a 1,00 m do CENTRO, para-choque a 0,28 m: vão de 0,72 m."""
    r, a0, inc = _varredura([(180, 1.00)])
    v = vao_no_corredor_traseiro(r, a0, inc, 0.50, 0.28)
    assert v == pytest.approx(0.72, abs=1e-3)


def test_o_que_esta_na_FRENTE_nao_conta_como_vao_traseiro():
    r, a0, inc = _varredura([(0, 0.40)])
    assert math.isinf(vao_no_corredor_traseiro(r, a0, inc, 0.50, 0.28))


def test_o_que_passa_de_LADO_do_corredor_nao_freia_a_re():
    """Parede paralela a 2 m de lado, atrás: `|y|` fora da meia-largura.

    Contar isso faria o robô se recusar a recuar em qualquer corredor — a ré
    ficaria proibida justamente onde ela é possível.
    """
    r, a0, inc = _varredura([(135, 3.0), (225, 3.0)])   # ~2,1 m de lado
    assert math.isinf(vao_no_corredor_traseiro(r, a0, inc, 0.50, 0.28))


def test_A_QUINA_QUE_O_SETOR_ANGULAR_PERDIA():
    """O caso da batida, e a razão de a medida ser retangular.

    Obstáculo a 0,60 m do centro num feixe de 145° — 35° fora dos 180°, ou
    seja FORA de um cone traseiro de ±30°. Mas `y = 0,60·sin(145°) = 0,34 m`
    e a meia-largura é 0,35: ele está DENTRO do corredor que o corpo varre.

    Um cone diria "livre" e o robô recuaria em cima dele. O retângulo vê.
    """
    r, a0, inc = _varredura([(145, 0.60)])
    v = vao_no_corredor_traseiro(r, a0, inc, 0.70, 0.28)
    assert math.isfinite(v), 'a quina sumiu da medida — é a batida de novo'
    assert v == pytest.approx(0.60 * abs(math.cos(math.radians(145))) - 0.28,
                              abs=1e-3)


def test_encostado_da_ZERO_e_nunca_negativo():
    """Obstáculo mais perto que o para-choque. Vão negativo somado à folga
    daria orçamento POSITIVO — permissão para recuar contra a parede."""
    r, a0, inc = _varredura([(180, 0.10)])
    v = vao_no_corredor_traseiro(r, a0, inc, 0.50, 0.28)
    assert v == 0.0
    assert orcamento_de_re(vao_traseiro=v, folga=0.30) == 0.0


def test_feixe_invalido_nao_vira_vao_livre():
    """`inf`, `nan` e zero são "não mediu", não "está livre". Tratá-los como
    livre é o BO-3: o robô recua confiando numa leitura que não existiu."""
    n = 360
    ranges = [float('nan')] * n
    ranges[180] = 0.0
    ranges[181] = float('inf')
    v = vao_no_corredor_traseiro(ranges, math.radians(-180.0),
                                 math.radians(1.0), 0.50, 0.28)
    assert math.isinf(v), 'feixe inválido virou obstáculo ou virou vão medido'


def test_gargalo_volta_a_centrar_se_derivou_para_fora_da_folga():
    """O latch do eixo tem saída — e ela é medida (20-08).

    Sem isto o robô congelava o primeiro instante em que ficou alinhado e
    seguia derivando: na corrida do corredor real ele chegou à soleira com
    0,195 m de desvio (p50) e a borda do corpo invadiu 14 cm da ombreira,
    apesar de o erro de RUMO estar em 4,1°.
    """
    caminho = [(0.1 * i, 0.0) for i in range(41)]
    porta = PassagemEstreita(19, 20, 21, 0.80)   # folga = 0,40-0,2275-0,03
    alvo, fase = alvo_estavel_de_passagem(
        caminho, porta, x=1.0, y=0.22, rumo_atual=0.0, saida=1.0,
        eixo_comprometido=True)
    assert fase == 'centro'
    assert alvo == caminho[porta.centro]


def test_gargalo_nao_larga_o_eixo_por_ruido_de_pose():
    """A saída do latch é mais larga que a entrada — senão vira o pulinho.

    Entra em eixo com |d| <= 0,08 e só sai acima da folga (0,1425 nesta porta).
    Um desvio de 0,10 m está na banda: já não autorizaria ENTRAR, e não é
    motivo para SAIR de uma travessia em curso.
    """
    caminho = [(0.1 * i, 0.0) for i in range(41)]
    porta = PassagemEstreita(19, 20, 21, 0.80)
    _, fase_com_latch = alvo_estavel_de_passagem(
        caminho, porta, x=1.0, y=0.10, rumo_atual=0.0, saida=1.0,
        eixo_comprometido=True)
    assert fase_com_latch == 'eixo'
    _, fase_sem_latch = alvo_estavel_de_passagem(
        caminho, porta, x=1.0, y=0.10, rumo_atual=0.0, saida=1.0,
        eixo_comprometido=False)
    assert fase_sem_latch == 'centro'
