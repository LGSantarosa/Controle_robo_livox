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

from lei_de_reta import MalhaDeReta, herdado_ff, norm_ang  # noqa: E402


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


# ------------------------------------------- o preditor de Smith (opt-in, 06-08)
#
# A alternativa a baixar o ganho: em vez de responder devagar por causa do
# atraso, DESCONTAR o que já está a caminho. Com o atraso fora de dentro da
# malha, o ganho pode voltar a subir.
#
# ⚠️ Ele depende do modelo (`preditor_atraso`, `preditor_ganho`). Por isso está
# desligado por padrão e por isso os testes abaixo incluem o caso do modelo
# ERRADO — um preditor que só é testado com o modelo certo é um preditor cuja
# pior falha ninguém viu.

def _picos_com_planta(malha, atraso_planta, curv_planta=-0.9116, v_cmd=0.25,
                      passos=1500, dt=0.02):
    """Como `_picos`, mas com o atraso da PLANTA escolhível — para poder pôr o
    modelo do preditor em desacordo com a planta de propósito."""
    yaw, fila, hist = 0.0, [0.0] * atraso_planta, []
    for _ in range(passos):
        fila.append(malha.passo(v_cmd, 0.0, yaw, dt))
        agindo = fila.pop(0)
        v_real = math.copysign(0.30, v_cmd)
        yaw = norm_ang(yaw + (agindo / abs(v_real) + curv_planta)
                       * abs(v_real) * dt)
        hist.append(math.degrees(yaw))
    return [abs(hist[i]) for i in range(1, len(hist) - 1)
            if (hist[i] - hist[i - 1]) * (hist[i + 1] - hist[i]) < 0]


def test_o_preditor_vem_DESLIGADO():
    """O caminho de produção não pode depender de modelo. Ligar é decisão
    explícita de quem está sintonizando, não default."""
    assert MalhaDeReta().preditor is False
    assert _defaults_do_no()['preditor'] is False


def test_desligado_ele_nao_muda_nada():
    """Garantia de que a fatia é inerte por padrão: com `preditor=False` a
    saída tem de ser bit a bit a mesma de antes de ele existir."""
    a, b = MalhaDeReta(), MalhaDeReta(preditor=False)
    for yaw in (0.0, -0.05, -0.12, -0.2, -0.1):
        assert a.passo(0.25, 0.0, yaw, 0.1) == b.passo(0.25, 0.0, yaw, 0.1)


def test_com_preditor_os_ganhos_ANTIGOS_param_de_tocar_sino():
    """O que ele compra. Os ganhos que faziam o robô oscilar (1,0 / 0,5) contra
    a planta com o atraso medido: sem preditor tocam sino, com preditor não."""
    sem = _picos_com_planta(MalhaDeReta(kp=1.0, ki=0.5), 47)
    com = _picos_com_planta(
        MalhaDeReta(kp=1.0, ki=0.5, preditor=True), 47)
    assert sem[1] > 5.0, 'a planta tem de mostrar o sino sem o preditor'
    assert com[1] < sem[1] / 2.0, (
        f'com preditor a 2ª excursão foi {com[1]:.1f}° contra {sem[1]:.1f}° '
        f'sem — esperava pelo menos metade')


def test_ele_preve_so_a_CORRECAO_e_nao_o_feedforward():
    """O detalhe que faz ele ajudar em vez de atrapalhar. O ff é a maior
    parcela da saída e o efeito futuro dele é cancelado pelo arco futuro;
    prever um sem o outro criaria viés do tamanho do ff.

    Com erro de rumo ZERO a correção é zero, então a previsão tem de ser zero
    — mesmo com o ff mandando 0,2 rad/s há vários passos."""
    m = MalhaDeReta(preditor=True)
    for _ in range(30):
        m.passo(0.25, 0.0, yaw=0.0, dt=0.05)     # rumo em cima: correção ~0
    assert m.yaw_efetivo(0.0) == pytest.approx(0.0, abs=1e-9), (
        'o feedforward vazou para a previsão')


def test_a_previsao_e_grampeada():
    """Fila grande não pode deslocar o yaw sem limite: previsão errada com
    autoridade infinita é pior que atraso nenhum."""
    m = MalhaDeReta(preditor=True, preditor_max=0.35, kp=5.0, ki=0.0)
    for _ in range(200):
        m.passo(0.25, 0.0, yaw=-1.0, dt=0.05)
    assert abs(m.yaw_efetivo(-1.0) - (-1.0)) <= 0.35 + 1e-9


def test_modelo_ERRADO_degrada_sem_explodir():
    """O caso que importa para confiar nele. O preditor acha que o atraso é
    0,94 s; a planta entrega 1,4 s (50% pior). Ele tem de degradar — não pode
    ficar pior que não ter preditor nenhum com os mesmos ganhos."""
    ganhos = dict(kp=1.0, ki=0.5)
    sem = _picos_com_planta(MalhaDeReta(**ganhos), 70)          # 1,4 s
    com = _picos_com_planta(MalhaDeReta(preditor=True, **ganhos), 70)
    assert com[1] <= sem[1] * 1.1, (
        f'com modelo 50% errado o preditor PIOROU: 2ª excursão {com[1]:.1f}° '
        f'contra {sem[1]:.1f}° sem ele')


def test_a_fila_morre_quando_a_referencia_morre():
    """Correções emitidas contra um rumo que não existe mais não podem ser
    descontadas na próxima reta — seria descontar giro que ninguém pediu."""
    m = MalhaDeReta(preditor=True)
    for _ in range(20):
        m.passo(0.25, 0.0, yaw=-0.2, dt=0.05)
    assert m.em_transito, 'a fila tinha de ter enchido'
    m.passo(0.0, 0.0, yaw=-0.2, dt=0.05)     # parou: descarta tudo
    assert m.em_transito == []


def test_o_detune_ganha_do_preditor_na_planta_de_hoje():
    """Trava o veredito de 06-08 para que ninguém ligue o preditor por default
    achando que é melhor. Ele mata a divergência, mas deixa ondulação
    SUSTENTADA (~3,6°) onde a redução de ganho assenta abaixo de 0,1°.

    Se um dia esta comparação virar, é porque o modelo mudou — e aí o default
    pode ser revisto COM o número novo na mão, não por preferência."""
    detune = _picos_com_planta(MalhaDeReta(), 47)
    preditor = _picos_com_planta(MalhaDeReta(kp=1.0, ki=0.5, preditor=True), 47)

    # O detune assenta: depois da primeira excursão sobra ruído, e tão pouco
    # que ele nem chega a produzir muitos extremos no horizonte.
    assert all(p < 0.5 for p in detune[1:]), (
        f'o detune deixou de assentar: {[round(p, 2) for p in detune[:5]]}')
    # O preditor ondula: extremos que continuam aparecendo, e grandes.
    assert len(preditor) > len(detune), (
        f'o preditor tinha mais extremos que o detune; agora {len(preditor)} '
        f'contra {len(detune)} — reveja o default')
    assert max(preditor[1:4]) > 2.0, (
        f'o preditor passou a assentar ({[round(p, 2) for p in preditor[:4]]})'
        f' — reveja o default, agora COM o número novo')


# ----------------- o ff é do dia, e o log tem de dizer qual (decisão 013)
#
# A curvatura crua muda 13,5% entre dias (04-08: −0,8031 · 05-08: −0,9116,
# faixas que não se tocam) e 2–3% dentro do dia. O caminho 3 escolhido pelo
# dono em 07-08 é medir no começo da sessão e passar por parâmetro — o que só
# funciona se o valor NÃO medido se denunciar. Estes testes travam a denúncia.

def test_o_default_do_no_se_declara_HERDADO():
    """Quem sobe a pilha sem passar nada tem de ver `HERDADO` no rosout. O
    default de `curv_frente` é o número de 04-08, e ele parece medido: é a
    mesma classe de defeito da bitola (29-07), um valor copiado que envelhece
    sozinho e não dá sintoma."""
    assert _defaults_do_no()['curv_medido_em'] == 'HERDADO'
    assert herdado_ff(_defaults_do_no()['curv_medido_em'])


def test_data_preenchida_conta_como_medida():
    """Quem digitou uma data afirmou ter medido. O log mostra qual é, e cabe
    ao dono desmentir — a régua aqui não tem como saber."""
    assert not herdado_ff('2026-08-05')


@pytest.mark.parametrize('texto', ['', '   ', 'HERDADO', 'herdado 04-08',
                                   'Herdado?'])
def test_a_duvida_cai_para_o_lado_que_AVISA(texto):
    """Vazio e as formas de escrever "herdado" à mão. Errar para o lado de
    avisar custa uma linha de log; errar para o outro deixa a bancada medir um
    robô que não existe."""
    assert herdado_ff(texto)


# --------------------------------------------------------------------------
# O ESTIMADOR DO ff (decisão 013, caminho 2) — 10-08
#
# O que o robô disse em 10-08 e motivou este bloco:
#   · a curvatura crua DERIVA +19% em 5,4 minutos (seis retas idênticas);
#   · com ff velho a envoltória CRESCE 1,65x em corrida de 2,5 m;
#   · com o ff do dia ela cai 2,7x e ainda assim não assenta em 10 s.
#
# ⚠️ O QUE ESTES TESTES NÃO PROVAM: que o robô vai andar reto. A planta de
# brinquedo reproduz a PRIMEIRA excursão de 10-08 quase exata (−13,6° contra
# −13,5° medidos) e erra o sobrepasso por 3,7x (+6,0° contra +22,3°) — ela é
# otimista justamente onde este estimador precisa ser julgado. Aqui se prova o
# MECANISMO: que ele converge, que é lento, que não dá solavanco, que não
# inventa robô novo e que sobrevive à parada. Quem arbitra o valor de
# `adapta_t` é a bancada, com corridas de 2,5 m.


def _roda_com_v_real(malha, curv_planta, dur=90.0, dt=0.02, atraso=47,
                     v=0.26, deriva=0.0):
    """Como `_roda_planta`, mas com três diferenças que importam:

    1. `v_real` é informado à lei — é o que o nó faz, e o feedforward escala
       com a velocidade MEDIDA (05-08: pedir 0,50 e andar 0,30 dava ff 67%
       grande demais);
    2. a planta pode DERIVAR (`deriva` em 1/m por minuto), que é a grandeza
       medida em 10-08: +0,022;
    3. ⚠️ **o ARCO sofre o mesmo atraso que o `wz`.** O `_roda_planta` aplica o
       arco desde o instante zero enquanto o comando chega 0,94 s depois — e
       isso fabrica ~13° de excursão (0,94 s × 0,244 rad/s) em TODA corrida,
       que nenhum feedforward pode evitar. É a ressalva que o `lei_de_reta.py`
       registra no bloco do preditor ("no robô os dois chegam juntos"), e o
       robô confirmou em 10-08: a corrida com o ff fresco começou **sem
       mergulho nenhum**, coisa impossível se o arco agisse antes do comando.
       Robô parado não arca, porque não anda.

    Devolve o traço (t, yaw em graus).
    """
    yaw, t, traco = 0.0, 0.0, []
    fila_wz, fila_v = [0.0] * atraso, [0.0] * atraso
    while t < dur:
        fila_wz.append(malha.passo(0.25, 0.0, yaw, dt, v))
        fila_v.append(v)
        wz_agindo, v_agindo = fila_wz.pop(0), fila_v.pop(0)
        planta = curv_planta + deriva * (t / 60.0)
        yaw = norm_ang(yaw + (wz_agindo + planta * v_agindo) * dt)
        traco.append((t, math.degrees(yaw)))
        t += dt
    return traco


def test_o_estimador_aprende_a_curvatura_que_o_ff_nao_sabia():
    """O caso de 10-08: sobe com o ff do começo da sessão (−0,8275) contra uma
    planta que já está em −0,94. Sem estimador esse déficit fica com o
    integrador para sempre; com ele, vira feedforward."""
    m = MalhaDeReta(curv_frente=-0.8275, adapta=True)
    _roda_com_v_real(m, curv_planta=-0.94)
    assert m.curv_frente == pytest.approx(-0.94, abs=0.02), (
        f'aprendeu {m.curv_frente:.4f}, planta −0,94')
    assert abs(m.integral) < 0.05, (
        f'o integrador ficou com {m.integral:.3f} rad·s — a drenagem não '
        f'chegou ao fim, ou está devolvendo mais do que transferiu')


def test_o_estimador_persegue_a_planta_que_DERIVA():
    """A deriva medida em 10-08 é +0,022 1/m por minuto. Em 3 minutos são
    0,066 1/m — mais que o critério da 011 inteiro. O estimador tem de seguir
    isso, que é a coisa que uma medida por sessão não faz."""
    m = MalhaDeReta(curv_frente=-0.85, adapta=True)
    _roda_com_v_real(m, curv_planta=-0.85, dur=180.0, deriva=-0.022)
    esperado = -0.85 - 0.022 * 3.0
    assert m.curv_frente == pytest.approx(esperado, abs=0.02), (
        f'a planta terminou em {esperado:.4f} e ele ficou em {m.curv_frente:.4f}')


def test_a_transferencia_nao_da_solavanco_no_comando():
    """No instante da drenagem o `wz` de saída NÃO pode mudar: o feedforward
    cresce exatamente o que o termo integral encolhe. Degrau de comando num
    laço com 0,94 s de tempo morto é como se fabrica a oscilação que este
    estimador veio matar."""
    m = MalhaDeReta(curv_frente=-0.8275, adapta=True)
    m.integral = 0.45
    v = 0.26
    antes = -m.curv_frente * v + m.ki * m.integral
    m._drena_para_o_ff(1, v, dt=0.1)
    depois = -m.curv_frente * v + m.ki * m.integral
    assert depois == pytest.approx(antes, abs=1e-12), (
        f'o comando saltou {depois - antes:+.6f} rad/s na transferência')
    assert m.curv_frente < -0.8275, 'não transferiu nada'


def test_o_estimador_nao_trabalha_parado_nem_devagar():
    """`Δcurv` divide por `v_real`: perto de zero, qualquer resíduo do
    integrador viraria curvatura enorme."""
    m = MalhaDeReta(curv_frente=-0.8275, adapta=True)
    m.integral = 0.45
    m._drena_para_o_ff(1, v_ff=0.01, dt=0.1)
    assert m.curv_frente == -0.8275, 'estimou com o robô praticamente parado'
    assert m.integral == 0.45


def test_o_grampo_prende_o_estimador_perto_da_semente():
    """Ele corrige DERIVA DE PLANTA, não inventa um robô novo. Uma referência
    de rumo ruim ou um `/Odometry` travado empurram o integrador para um lado
    só; sem grampo a curvatura iria atrás e ficaria lá."""
    m = MalhaDeReta(curv_frente=-0.85, adapta=True, adapta_desvio_max=0.1)
    for _ in range(5000):
        m.integral = 0.6            # integrador colado no teto, sempre
        m._drena_para_o_ff(1, v_ff=0.26, dt=0.02)
    assert m.curv_frente == pytest.approx(-0.95, abs=1e-9), (
        f'passou do grampo: {m.curv_frente:.4f}')


def test_o_grampo_nao_devolve_ao_integrador_o_que_nao_transferiu():
    """Quando o grampo corta a transferência pela metade, só a metade que virou
    feedforward pode sair do integrador. Devolver o valor cheio deixaria o
    comando com um degrau para baixo — o solavanco pela porta dos fundos."""
    m = MalhaDeReta(curv_frente=-0.85, adapta=True, adapta_desvio_max=0.1)
    m.curv_frente = -0.949        # a um milésimo do grampo (semente −0,85)
    m.integral = 0.6
    v = 0.26
    antes = -m.curv_frente * v + m.ki * m.integral
    m._drena_para_o_ff(1, v, dt=1.0)          # pediria muito mais que 0,001
    depois = -m.curv_frente * v + m.ki * m.integral
    assert m.curv_frente == pytest.approx(-0.95, abs=1e-9), 'furou o grampo'
    assert depois == pytest.approx(antes, abs=1e-12), (
        f'o grampo produziu um degrau de {depois - antes:+.6f} rad/s')


def test_a_parada_apaga_o_integrador_mas_NAO_o_aprendido():
    """É a razão de existir do estimador. O integrador é rumo acumulado e some
    na parada (referência velha é pior que nenhuma); a curvatura é propriedade
    do robô e fica. Sem isso, cada corrida recomeça reaprendendo — que é
    exatamente o mergulho seguido de sobrepasso medido em 10-08."""
    m = MalhaDeReta(curv_frente=-0.8275, adapta=True)
    _roda_com_v_real(m, curv_planta=-0.94, dur=60.0)
    aprendido = m.curv_frente
    assert aprendido < -0.90, 'não aprendeu nada para haver o que preservar'
    m.passo(0.0, 0.0, 0.0, 0.02, 0.0)         # parou
    assert m.integral == 0.0 and m.rumo_ref is None
    assert m.curv_frente == aprendido, 'a parada apagou o que ele aprendeu'


def test_a_corrida_seguinte_ja_comeca_corrigida():
    """O ganho prático: a corrida seguinte não repete o aprendizado.

    ⚠️ A janela COMEÇA em 1,5 s, e a razão é um defeito conhecido da planta de
    brinquedo: ela aplica o arco desde o instante zero enquanto o `wz` chega
    com 0,94 s de atraso, então TODA corrida nasce com ~13° de excursão
    (0,94 s × 0,244 rad/s) que nenhum feedforward pode evitar. No robô parado
    não há arco, porque não há movimento — a corrida de 10-08 com o ff fresco
    começou sem mergulho nenhum. Medir a partir de 1,5 s tira o artefato e
    deixa o que este teste quer julgar: o que a malha faz DEPOIS.
    """
    m = MalhaDeReta(curv_frente=-0.8275, adapta=True)
    primeira = _roda_com_v_real(m, curv_planta=-0.94, dur=30.0)
    m.passo(0.0, 0.0, 0.0, 0.02, 0.0)         # parada entre corridas
    segunda = _roda_com_v_real(m, curv_planta=-0.94, dur=30.0)
    janela = lambda tr: max(abs(y) for t, y in tr if 1.5 <= t <= 10.0)
    p1, p2 = janela(primeira), janela(segunda)
    assert p2 < 0.5 * p1, (
        f'a segunda corrida excursionou {p2:.1f}° contra {p1:.1f}° da '
        f'primeira — o aprendido não está valendo na largada')


def test_o_estimador_vem_DESLIGADO_de_fabrica():
    """Entra como o preditor entrou: opt-in. A condição de controle da próxima
    bancada é o comportamento de hoje, senão não há com o que comparar."""
    assert MalhaDeReta().adapta is False
    assert _defaults_do_no()['adapta'] is False


def test_o_no_e_a_lei_concordam_nos_parametros_do_estimador():
    """Default duplicado é default que deriva — o `placa_simulada` já pagou."""
    par = _defaults_do_no()
    m = MalhaDeReta()
    assert m.adapta_t == par['adapta_t']
    assert m.adapta_desvio_max == par['adapta_desvio_max']
    assert m.adapta_v_min == par['adapta_v_min']


def test_a_drenagem_e_mais_lenta_que_o_laco():
    """Dois integradores em série com escalas parecidas oscilam juntos. O laço
    assenta em ~9,6 s (planta de brinquedo, ganhos de 06-08); a drenagem tem de
    ser da mesma ordem ou mais lenta, nunca mais rápida."""
    assert MalhaDeReta().adapta_t >= 8.0, (
        'drenagem rápida demais: ela passa a perseguir o transiente do laço '
        'em vez do viés de planta')
