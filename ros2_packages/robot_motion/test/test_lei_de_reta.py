"""A malha de reta (decisão 011) tem de segurar o rumo num robô que arca.

O caso de teste central não é unitário, é de FECHAMENTO: uma planta de
brinquedo com o defeito medido do robô (curvatura de viés que a malha não
conhece com exatidão) tem de sair andando reto. Se só os testes de sinal
passassem, a malha poderia estar estável e inútil.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'robot_motion'))

from lei_de_reta import MalhaDeReta, norm_ang  # noqa: E402


# ------------------------------------------------------------ o feedforward

def test_ff_de_frente_corrige_para_a_esquerda():
    """O robô arca para a DIREITA de frente (curv −0,817), então a correção
    tem de ser wz POSITIVO (esquerda) — e proporcional ao v comandado."""
    m = MalhaDeReta(ki=0.0, kp=0.0)
    wz = m.passo(v_cmd=0.25, wz_cmd=0.0, yaw=0.0, dt=0.1)
    assert wz == pytest.approx(0.817 * 0.25, rel=1e-6)


def test_ff_de_re_corrige_para_o_mesmo_lado_e_menos():
    """De ré o arco é do MESMO lado do corpo e 8x menor — o ff acompanha.
    Foi um sinal trocado aqui (na placa) que a aceitação de 04-08 pegou."""
    m = MalhaDeReta(ki=0.0, kp=0.0)
    frente = m.passo(0.25, 0.0, 0.0, 0.1)
    m2 = MalhaDeReta(ki=0.0, kp=0.0)
    re = m2.passo(-0.25, 0.0, 0.0, 0.1)
    assert re == pytest.approx(0.098 * 0.25, rel=1e-6)
    assert frente > re > 0.0


# ------------------------------------------------------- captura e descarte

def test_captura_o_rumo_na_entrada_da_reta_e_segura():
    """A referência é o yaw do INSTANTE em que a reta começou. Se o robô
    derivou para −0,1 rad, o erro é +0,1 e a correção cresce."""
    m = MalhaDeReta(kp=1.0, ki=0.0, curv_frente=0.0)
    assert m.passo(0.25, 0.0, yaw=0.3, dt=0.1) == pytest.approx(0.0)
    assert m.rumo_ref == pytest.approx(0.3)
    wz = m.passo(0.25, 0.0, yaw=0.2, dt=0.1)
    assert wz == pytest.approx(0.1, abs=0.02)


def test_curva_pedida_mantem_a_intencao_mas_ganha_o_ff():
    """Curva é assunto do comandante — a MALHA DE RUMO não se mete nela. Mas o
    ff sim: o arco é do CORPO e existe girando também. Antes esta lei devolvia
    a curva crua, e o robô curvava com o viés somado por cima."""
    m = MalhaDeReta()
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    assert m.rumo_ref is not None
    saida = m.passo(0.25, 0.5, yaw=0.7, dt=0.1)
    assert saida == pytest.approx(0.5 + 0.817 * 0.25, rel=1e-6)
    assert m.rumo_ref is None, 'a referência de rumo tem de morrer na curva'
    m.passo(0.25, 0.0, yaw=1.5, dt=0.1)
    assert m.rumo_ref == pytest.approx(1.5)


def test_ff_escala_com_a_velocidade_REAL_e_nao_com_a_pedida():
    """O defeito de 05-08, achado pelo dono olhando o robô pender. O arco é
    curvatura × distância percorrida, e quem decide a distância é o patamar da
    placa. Na pilha o seguidor pedia 0,500 e o robô andava 0,299: o ff saía
    67% grande."""
    m = MalhaDeReta(kp=0.0, ki=0.0)
    com_real = m.passo(0.500, 0.0, yaw=0.0, dt=0.1, v_real=0.299)
    assert com_real == pytest.approx(0.817 * 0.299, rel=1e-6)
    m2 = MalhaDeReta(kp=0.0, ki=0.0)
    sem_real = m2.passo(0.500, 0.0, yaw=0.0, dt=0.1)
    assert sem_real == pytest.approx(0.817 * 0.500, rel=1e-6)
    assert com_real < sem_real, 'sem v_real a lei superestima o arco'


def test_sem_segurar_rumo_sobra_so_o_feedforward():
    """Modo para quando há controlador de rumo ACIMA (a pilha). Os dois
    segurando rumo disputam: este segura o que CAPTUROU, o de cima quer o do
    caminho, e o de cima acaba pivotando para desfazer. Aqui a lei só cancela
    o arco e não escolhe direção nenhuma."""
    m = MalhaDeReta(segura_rumo=False)
    # Mesmo com erro de rumo enorme, a saída é o ff puro: nada de P nem I.
    for _ in range(50):
        saida = m.passo(0.25, 0.0, yaw=-1.0, dt=0.1, v_real=0.30)
    assert saida == pytest.approx(0.817 * 0.30, rel=1e-6)
    assert m.integral == 0.0 and m.rumo_ref is None


def test_parado_nao_segura_rumo_nenhum():
    """Robô parado pode ser girado no chão pelo dono. Referência velha nesse
    caso é pior que nenhuma: mandaria o robô 'de volta' a um rumo morto."""
    m = MalhaDeReta()
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    m.passo(0.0, 0.0, yaw=0.0, dt=0.1)
    assert m.rumo_ref is None and m.integral == 0.0


def test_troca_de_sentido_zera_o_integrador():
    """O viés da frente não é o da ré. Integrador carregado atravessando a
    troca viraria chicote — o robô sairia da ré já esterçando errado."""
    m = MalhaDeReta(kp=0.0, ki=1.0, curv_frente=0.0, curv_re=0.0)
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    for _ in range(20):
        m.passo(0.25, 0.0, yaw=-0.2, dt=0.1)
    assert m.integral > 0.0
    m.passo(-0.25, 0.0, yaw=-0.2, dt=0.1)
    assert m.integral == pytest.approx(0.0, abs=1e-6)


# ----------------------------------------------------------------- os grampos

def test_integrador_tem_teto():
    """Anti-windup: erro que persiste (robô travado, BO-3) não pode encher o
    integrador sem limite — na soltura ele descarregaria tudo de uma vez."""
    m = MalhaDeReta(int_max=0.6)
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    for _ in range(1000):
        m.passo(0.25, 0.0, yaw=-1.0, dt=0.1)
    assert m.integral <= 0.6 + 1e-9


def test_saida_tem_teto_proprio():
    """A correção de regime é ~0,25 rad/s; o grampo é do transitório. Sem
    ele, erro grande na captura pediria giro que o robô transforma em arco
    fechado — o oposto de andar reto."""
    m = MalhaDeReta(wz_max=0.6, kp=10.0)
    m.passo(0.25, 0.0, yaw=0.0, dt=0.1)
    wz = m.passo(0.25, 0.0, yaw=-3.0, dt=0.1)
    assert abs(wz) <= 0.6 + 1e-9


def test_dt_nao_positivo_nao_envenena_o_integrador():
    """Relógio de simulação pode repetir carimbo; a primeira amostra não tem
    dt. Nenhum dos dois pode somar no integrador."""
    m = MalhaDeReta(kp=0.0, ki=1.0, curv_frente=0.0)
    m.passo(0.25, 0.0, yaw=0.0, dt=0.0)
    m.passo(0.25, 0.0, yaw=-0.5, dt=0.0)
    m.passo(0.25, 0.0, yaw=-0.5, dt=-0.1)
    assert m.integral == 0.0


# ------------------------------------------------- o fechamento (o que vale)

def _roda_planta(malha, curv_planta, v_cmd, passos=1500, dt=0.02, atraso=47):
    """Planta de brinquedo com o defeito medido: a curvatura ENTREGUE é a
    comandada mais um viés que a malha não conhece — e o wz age com atraso.

    ⚠️ **O ATRASO ERA 0,26 s E ESTAVA ERRADO — corrigido em 05-08 para 0,94 s**
    (47 passos de 20 ms). O valor antigo era só a latência de LIGA (0,27 s); o
    laço real tem também o atraso de DESLIGA da placa (0,52 s) mais a pose a
    10 Hz e a janela de 0,2 s da velocidade. Os 0,94 s foram confirmados por
    duas rotas independentes: essa soma, e a frequência da oscilação que o robô
    de fato fez (1,277 rad/s com o PI antigo).

    Por que isso importa mais do que parece: **com 0,26 s esta planta não
    oscila com ganho nenhum.** Foi por isso que o S do robô não apareceu em
    lugar nenhum — nem aqui, nem no Gazebo. Com 0,94 s ela reproduz o
    fenômeno, e passa a ser o único lugar do projeto onde a estabilidade do
    rumo pode ser julgada sem o robô.

    O horizonte subiu de 600 para 1500 passos (12 s -> 30 s) porque a malha
    nova é 4x mais lenta de propósito; medir regime em 12 s pegaria o
    transiente dela e reprovaria um projeto correto.

    Devolve (curvatura de regime, erro de rumo final). A curvatura é
    calculada como a bancada calcula (giro acumulado / caminho, 2ª metade);
    o erro de rumo é contra o yaw capturado na largada (aqui, 0).
    """
    yaw, x, y = 0.0, 0.0, 0.0
    fila = [0.0] * atraso
    caminho, giro = 0.0, 0.0
    metade = passos // 2
    for i in range(passos):
        wz_cmd = malha.passo(v_cmd, 0.0, yaw, dt)
        fila.append(wz_cmd)
        wz_agindo = fila.pop(0)
        v_real = math.copysign(0.30, v_cmd)          # patamar: v não obedece
        curv_entregue = wz_agindo / abs(v_real) + curv_planta
        dyaw = curv_entregue * abs(v_real) * dt
        if i >= metade:
            giro += dyaw
            caminho += abs(v_real) * dt
        yaw = norm_ang(yaw + dyaw)
        x += v_real * math.cos(yaw) * dt
        y += v_real * math.sin(yaw) * dt
    return giro / caminho, norm_ang(0.0 - yaw)


def test_fecha_a_reta_de_frente_no_criterio_da_011():
    """O teste que importa: planta com o viés MEDIDO (−0,817), malha com os
    ganhos de fábrica → |curvatura| < 0,05 1/m em regime (critério da 011),
    e o rumo assentado em cima da referência (< 1°)."""
    c, e = _roda_planta(MalhaDeReta(), curv_planta=-0.817, v_cmd=0.25)
    assert abs(c) < 0.05, f'curvatura de regime {c:.3f}'
    assert abs(math.degrees(e)) < 1.0, f'rumo assentou {math.degrees(e):.1f}° torto'


def test_fecha_a_re_no_criterio_da_011():
    c, e = _roda_planta(MalhaDeReta(), curv_planta=-0.098, v_cmd=-0.25)
    assert abs(c) < 0.05, f'curvatura de regime {c:.3f}'
    assert abs(math.degrees(e)) < 1.0, f'rumo assentou {math.degrees(e):.1f}° torto'


def test_fecha_mesmo_com_ff_errado_25_por_cento():
    """A dispersão do robô é 21% entre corridas idênticas, então o ff calibrado
    num dia enfrenta a planta de outro. Com o ff 25% otimista, ele ainda tem de
    ANDAR RETO — que é o critério da 011.

    ⚠️ **A expectativa de rumo mudou em 05-08, e não foi para o teste passar.**
    Antes este teste exigia rumo < 1°, e passava porque os ganhos eram 4x
    maiores. Eles foram reduzidos de propósito para matar uma oscilação
    CRESCENTE medida no robô (ver os ganhos no `compensador_rumo.py`), e o
    preço calculado dessa redução é exatamente este: com o ff bem errado o
    integrador satura em `int_max` e o termo P sustenta o resto **mantendo um
    erro de rumo**.

    O que NÃO se degradou é o que a 011 pede: a curvatura fica em 0,0007 —
    três ordens de grandeza abaixo do limite. O robô anda reto; ele só anda
    reto apontando alguns graus fora da referência que capturou.

    ➡️ O conserto do rumo é o **feedforward** estar certo, não o integrador
    brigar contra tempo morto. Subir `int_max` foi testado e traz o sino de
    volta (9° de segunda excursão) — a tabela está no `compensador_rumo.py`."""
    m = MalhaDeReta()                       # ff acha que é −0,817
    c, e = _roda_planta(m, curv_planta=-1.02, v_cmd=0.25)   # planta 25% pior
    assert abs(c) < 0.05, f'curvatura de regime {c:.3f}'
    assert abs(math.degrees(e)) < 8.0, (
        f'rumo assentou {math.degrees(e):.1f}° torto — acima disso o ff está '
        f'errado demais para o integrador limitado dar conta')


def test_sem_integrador_o_rumo_assenta_torto():
    """O papel EXATO do Ki, descoberto por este próprio teste na primeira
    versão: sem ele o P ainda ZERA a curvatura (o robô anda reto!), mas
    andando ~6° fora do rumo capturado — o erro constante que o P precisa
    manter para sustentar a correção. O integrador existe para zerar o RUMO,
    não a curvatura. Com Ki o mesmo caso assenta < 1° (teste acima)."""
    m = MalhaDeReta(ki=0.0)
    c, e = _roda_planta(m, curv_planta=-1.02, v_cmd=0.25)
    assert abs(c) < 0.05, 'a curvatura fecha até sem Ki — não é ela que o justifica'
    assert abs(math.degrees(e)) > 3.0, \
        f'sem Ki o rumo deveria assentar torto, deu {math.degrees(e):.1f}°'


# ------------------------------- os ganhos são um PROJETO, não um chute (05-08)
#
# Com kp=1,0 e ki=0,5 o robô OSCILAVA e a oscilação CRESCIA 2,07x por
# meio-período (três corridas, `docs/dados/2026-08-05-bancada-robo/`). A causa
# é tempo morto: 0,94 s de atraso efetivo no laço, confirmado por duas rotas
# independentes — a frequência medida da oscilação (1,277 rad/s) e a soma dos
# atrasos de liga (0,27 s) e desliga (0,52 s) da placa mais o sensoriamento.
#
# Os ganhos caíram 4,1x, JUNTOS, para o ganho de laço passar de ~2 para ~0,5.

import ast  # noqa: E402


def _defaults_do_no():
    """Lê os defaults do `declare_parameters` do nó, direto do fonte.

    Duplicar default entre o nó e a lei é como eles derivam. O
    `placa_simulada` já pagou esse preço em 04-08: o dublê do teste ficou com
    valores antigos e a suíte passou verde descrevendo um robô que já não
    existia."""
    fonte = os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'robot_motion', 'compensador_rumo.py')
    arvore = ast.parse(open(fonte).read())
    for no in ast.walk(arvore):
        if (isinstance(no, ast.Call)
                and getattr(no.func, 'attr', None) == 'declare_parameters'):
            return {ast.literal_eval(t.elts[0]): ast.literal_eval(t.elts[1])
                    for t in no.args[1].elts}
    raise AssertionError('declare_parameters não encontrado')


def _picos(malha, curv_planta=-0.9116, v_cmd=0.25, passos=1500, dt=0.02,
           atraso=47):
    """Amplitudes dos extremos sucessivos do rumo. Toca sino ou assenta?"""
    yaw, fila, hist = 0.0, [0.0] * atraso, []
    for _ in range(passos):
        fila.append(malha.passo(v_cmd, 0.0, yaw, dt))
        agindo = fila.pop(0)
        v_real = math.copysign(0.30, v_cmd)
        yaw = norm_ang(yaw + (agindo / abs(v_real) + curv_planta)
                       * abs(v_real) * dt)
        hist.append(math.degrees(yaw))
    return [abs(hist[i]) for i in range(1, len(hist) - 1)
            if (hist[i] - hist[i - 1]) * (hist[i + 1] - hist[i]) < 0]


def test_o_no_e_a_lei_concordam_nos_ganhos():
    par = _defaults_do_no()
    m = MalhaDeReta()
    assert m.kp == par['kp'], 'default do nó e da lei divergiram'
    assert m.ki == par['ki'], 'default do nó e da lei divergiram'


def test_os_ganhos_ANTIGOS_tocam_sino_na_planta_com_o_atraso_MEDIDO():
    """A prova de que o número novo tem razão de ser — e de que a planta de
    brinquedo, com o atraso certo, enxerga o defeito que o robô mostrou.

    Verificado por mutação: com o atraso antigo (13 passos = 0,26 s) este
    teste FALHA, porque lá nem os ganhos velhos oscilam. Era essa a cegueira."""
    picos = _picos(MalhaDeReta(kp=1.0, ki=0.5))
    assert len(picos) >= 5, (
        f'esperava sino com os ganhos antigos, vieram {len(picos)} extremos')
    assert picos[1] > 5.0, (
        f'a segunda excursão foi {picos[1]:.1f}° — sem sino, a planta não '
        f'reproduz o que o robô fez')


def test_os_ganhos_DE_HOJE_assentam_em_uma_excursao():
    """O projeto: uma excursão e acabou. Não é "oscila menos" — é não oscilar."""
    picos = _picos(MalhaDeReta())
    assert len(picos) >= 1
    assert picos[1] < 2.0 if len(picos) > 1 else True, (
        f'segunda excursão de {picos[1]:.1f}° — ainda está tocando sino')
    assert all(p < 1.0 for p in picos[2:]), (
        f'não assentou: extremos seguintes {[round(p, 2) for p in picos[2:5]]}')


def test_a_razao_ki_sobre_kp_foi_preservada():
    """A redução foi de GANHO, não de controlador: os dois caíram juntos, então
    o zero do PI (em ki/kp) fica onde estava. Mexer só num deles muda a forma
    da resposta e invalida a análise de estabilidade que escolheu o número."""
    par = _defaults_do_no()
    assert par['ki'] / par['kp'] == pytest.approx(0.5, rel=0.05), (
        'o zero do PI mudou de lugar — refazer a análise antes de aceitar')


def test_o_integrador_ainda_da_conta_do_residuo_do_ff():
    """Ganho menor não pode virar erro permanente. O ff erra ~0,095 1/m (planta
    de 05-08 contra o ff carregado), o que a 0,30 m/s pede 0,028 rad/s de
    correção contínua. O teto do integrador tem de cobrir isso com folga."""
    par = _defaults_do_no()
    residuo_rad_s = (0.9116 - 0.817) * 0.298
    autoridade = par['ki'] * par['int_max']
    assert autoridade > 2 * residuo_rad_s, (
        f'integrador entrega no máximo {autoridade:.3f} rad/s e o resíduo pede '
        f'{residuo_rad_s:.3f} — sem folga, o rumo fica com erro permanente')
