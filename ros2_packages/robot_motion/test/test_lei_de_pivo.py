"""O pivô por corte previsto (fatia 3 da 011) num robô que não modula giro.

O teste que importa não é unitário: é fechar a manobra contra uma PLANTA de
brinquedo com o atuador medido — latência de liga, rampa, um único `wz`
entregue, e sobra depois do corte. E fechar **mesmo com o `a_dec` errado**,
porque a curva real do robô difere da do simulador por origem (config lá,
inércia aqui) e a lei não pode depender dela.
"""
import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'robot_motion'))

from lei_de_pivo import (  # noqa: E402
    ASSENTANDO, DESISTIU, GIRANDO, PRONTO, PivoPorCorte, norm_ang)


class Planta:
    """O atuador medido, em miniatura.

    Números de 04-08: latência de liga 0,27 s; sobe a ~1,6 rad/s²; a placa
    entrega um patamar de ~2,2 rad/s; depois do corte segue empurrando e então
    desacelera. `a_real` é o que a lei NÃO conhece.

    🔴 **O QUE ESTA PLANTA ERRAVA ATÉ A 4ª LEVA DE 12-08, e era o mesmo ponto
    cego que o Gazebo teve em 06-08**: depois do corte ela CONGELAVA o `wz` por 0,2 s e
    só então desacelerava. A placa não faz isso. O `placa_simulada` segura a
    **saída cheia** (o patamar) decaindo em rampa por `atraso_desliga`, e a
    docstring dele diz o que isso significa com todas as letras: *"entre o
    corte e o pico de `wz` passam 0,40–0,60 s, e nesse trecho o robô ainda
    ACELERA"*.

    A diferença não é de grau. Congelando, a sobra depois do corte é menor que
    a velocidade de corte manda supor, e a lei — que erra `a_dec` para baixo de
    propósito — chega por baixo e converge por pulsos. Acelerando depois do
    corte, a sobra explode e a manobra passa do alvo TODA VEZ. Foi este o
    ciclo-limite medido no Gazebo em 12-08: 28 pivôs disparados com erro entre
    35° e 81°, e uma varredura pós-corte de 93–101° (n=5, repetível).
    """

    def __init__(self, a_real=1.0, latencia=0.27, sobe=1.6, patamar=2.2,
                 atraso_desliga=0.52, piso_tempo=0.0):
        self.a_real = a_real
        self.latencia = latencia
        self.sobe = sobe
        self.patamar = patamar
        self.atraso_desliga = atraso_desliga
        self.piso_tempo = piso_tempo   # tempo mínimo ligado para sair do lugar
        self.wz = 0.0
        self.yaw = 0.0
        self.t_lig = None
        self.t_desl = None
        self.retido = 0.0              # o que a placa segue empurrando
        self.t = 0.0

    def passo(self, wz_cmd, dt):
        self.t += dt
        if abs(wz_cmd) > 1e-9:
            if self.t_lig is None:
                self.t_lig = self.t
            self.t_desl = None
            ligado = self.t - self.t_lig
            if ligado >= self.latencia and ligado >= self.piso_tempo:
                alvo = math.copysign(self.patamar, wz_cmd)
                self.retido = alvo
                self.wz += math.copysign(min(self.sobe * dt, abs(alvo - self.wz)),
                                         alvo - self.wz)
        else:
            self.t_lig = None
            if self.t_desl is None:
                self.t_desl = self.t
            passado = self.t - self.t_desl
            if passado < self.atraso_desliga:
                # A placa ainda empurra — a saída retida DECAI em rampa, e é a
                # cheia, não a de agora. Enquanto ela for maior que o `wz`
                # atual, o robô ACELERA depois do corte.
                alvo = self.retido * (1.0 - passado / self.atraso_desliga)
                self.wz += math.copysign(min(self.sobe * dt, abs(alvo - self.wz)),
                                         alvo - self.wz)
            else:
                self.retido = 0.0
                d = self.a_real * dt
                self.wz = (0.0 if abs(self.wz) <= d
                           else self.wz - math.copysign(d, self.wz))
        self.yaw = norm_ang(self.yaw + self.wz * dt)
        return self.wz, self.yaw


def gira(lei, alvo, planta, dt=0.02, passos=2000):
    """Roda a manobra até acabar. Devolve (erro final, estado, pulsos)."""
    estado = GIRANDO
    for _ in range(passos):
        erro = norm_ang(alvo - planta.yaw)
        wz_cmd, estado = lei.passo(erro, planta.wz, dt)
        planta.passo(wz_cmd, dt)
        if estado in (PRONTO, DESISTIU):
            break
    return norm_ang(alvo - planta.yaw), estado, lei.pulsos


# ------------------------------------------------------------- o critério

def test_recusa_tolerancia_abaixo_do_pivo_minimo():
    """Abaixo de ~0,4 s ligado o robô não sai do lugar (04-08): pedir 2° é
    programar um laço que nunca fecha. A lei recusa na construção."""
    with pytest.raises(ValueError):
        PivoPorCorte(tolerancia=math.radians(2.0))


def test_a_sobra_e_a_lei_da_005_invertida():
    """A 005 caiu como COMANDO e volta como CRITÉRIO DE CORTE: a mesma
    `wz²/(2·a_dec)` de 27-07, agora usada como gatilho."""
    lei = PivoPorCorte(a_dec=1.0)
    assert lei.sobra_prevista(2.0) == pytest.approx(2.0)
    assert lei.sobra_prevista(0.0) == 0.0


# ------------------------------------------------- fechar a manobra
#
# ⚠️ LEIA ANTES DE MEXER (12-08, 4ª leva). Os testes desta seção rodam contra
# `Planta(atraso_desliga=0.0)` — um atuador que PARA quando mandam parar. Não é
# o robô e não é o simulador: é a máquina que a lei SUPÕE, e é contra ela que
# faz sentido julgar a aritmética do corte.
#
# Contra a placa de verdade (a seção seguinte) a manobra não fecha em ângulo
# nenhum, e isso não é defeito de implementação — é a premissa da lei que não
# vale ali. Separar as duas coisas é o que permite dizer QUAL das duas quebrou.

ATUADOR_QUE_OBEDECE = dict(atraso_desliga=0.0)


@pytest.mark.parametrize('graus', [20, 45, 90, 135, 180, -45, -90, -170])
def test_fecha_o_pivo_em_varios_angulos(graus):
    """A manobra tem de fechar dentro da tolerância em toda a faixa útil,
    para os dois lados."""
    lei = PivoPorCorte()
    erro, estado, _ = gira(lei, math.radians(graus), Planta(**ATUADOR_QUE_OBEDECE))
    assert estado == PRONTO, f'{graus}° terminou em {estado}'
    assert abs(erro) <= lei.tolerancia + 1e-9, \
        f'{graus}° sobrou {math.degrees(erro):.1f}°'


def test_nao_sobrepassa_alem_da_tolerancia():
    """O defeito que a lei existe para evitar: varrer além do alvo. Com corte
    conservador ele tem de chegar POR BAIXO, não por cima."""
    lei = PivoPorCorte()
    p = Planta(**ATUADOR_QUE_OBEDECE)
    erro, _, _ = gira(lei, math.radians(90), p)
    assert math.degrees(p.yaw) <= 90 + math.degrees(lei.tolerancia)


@pytest.mark.parametrize('a_real', [0.7, 1.0, 1.5, 2.0, 3.0])
def test_fecha_com_qualquer_a_dec_ACIMA_do_suposto(a_real):
    """O teste que justifica a aproximação sucessiva, e o lado SEGURO da
    condição `a_lei ≤ a_real`. A lei supõe 0,6; a planta entrega de 0,7 a 3,0
    — a faixa medida entre robô (0,83) e simulador (0,67–1,46), com folga.
    Em todos, a manobra tem de FECHAR, não só 'quase'."""
    lei = PivoPorCorte()                      # a_dec 0,6, o padrão
    erro, estado, _ = gira(lei, math.radians(120),
                           Planta(a_real=a_real, **ATUADOR_QUE_OBEDECE))
    assert estado == PRONTO, f'a_real={a_real} terminou em {estado}'
    assert abs(erro) <= lei.tolerancia + 1e-9


# ------------------------------------------ e a placa de verdade, que RETÉM
#
# Decisão 023. O que está travado aqui é o motivo de o pivô ter saído do
# caminho normal do seguidor — e é um resultado NEGATIVO, do tipo que costuma
# não ser escrito e voltar a custar caro.

def test_a_manobra_NAO_FECHA_contra_a_retencao_da_placa():
    """O ciclo-limite de 12-08, reproduzido sem Gazebo.

    Medido no Gazebo (`docs/dados/2026-08-12-sim-meu-mapa/`): 28 pivôs
    disparados com erro entre 35° e 81°, varredura pós-corte de 93–101°
    (n=5), 1321° de giro em 32,6 s e 0,20 m de deslocamento. O robô girava no
    lugar e nunca saía.

    Com a retenção da placa no lugar, a manobra falha na faixa inteira. Se
    algum dia ela passar a fechar, alguém consertou o mecanismo — e aí a 023
    precisa ser relida, não este teste apagado.
    """
    falharam = []
    for graus in (20, 45, 90, 135, 180, -45, -90, -170):
        _, estado, _ = gira(PivoPorCorte(), math.radians(graus), Planta())
        if estado != PRONTO:
            falharam.append(graus)
    assert len(falharam) >= 6, (
        f'a manobra fechou em quase toda a faixa ({falharam} falharam) — '
        'a retenção da placa deixou de derrubar o pivô, e a decisão 023 '
        'perdeu a premissa')


@pytest.mark.parametrize('a_dec', [0.6, 0.4, 0.3, 0.2, 0.15, 0.10, 0.05])
def test_NENHUM_a_dec_salva_o_corte_contra_a_retencao(a_dec):
    """E o conserto óbvio não é conserto — o argumento é estrutural.

    A sobra PREVISTA é `wz²/(2·a_dec)`: uma parábola no `wz` do corte. A sobra
    REAL contra esta placa é a retenção, `patamar·atraso/2` ≈ 33°, que **não
    depende do wz do corte** — a placa segura a saída CHEIA, não a de agora.
    Parábola não casa com constante em valor nenhum de `a_dec`, e a varredura
    mostra exatamente isso: o acerto vira sorteio por ângulo, não tendência.

    É por isso que a 023 não baixou `pivo_a_dec` e mexeu em quem CHAMA o pivô.
    """
    fechou = []
    for graus in (20, 45, 90, 135, 180):
        _, estado, _ = gira(PivoPorCorte(a_dec=a_dec), math.radians(graus),
                            Planta())
        fechou.append(estado == PRONTO)
    assert not all(fechou), (
        f'a_dec={a_dec} fechou a faixa inteira contra a retenção — se isto '
        'passar a valer, o corte VOLTA a ser alavanca e a 023 muda')


def test_a_sobra_da_retencao_nao_depende_de_quando_se_corta():
    """O número que sustenta o teste acima, medido na própria planta.

    Cortar cedo (wz baixo) ou tarde (wz alto) tem de dar quase a mesma
    varredura — é isso que torna o corte uma alavanca inútil.
    """
    varreduras = []
    for ligado_s in (0.4, 0.8, 1.2):
        p = Planta()
        dt, t = 0.02, 0.0
        while t < 6.0:
            p.passo(1.0 if t < p.latencia + ligado_s else 0.0, dt)
            t += dt
            if t > p.latencia + ligado_s and abs(p.wz) < 1e-6:
                break
        varreduras.append(math.degrees(p.yaw))
    depois_do_corte = [v - varreduras[0] for v in varreduras]
    espalho = max(varreduras) - min(varreduras)
    assert espalho < 0.75 * max(varreduras), (
        f'varreduras {varreduras} — se elas se separarem, o instante do corte '
        f'voltou a decidir o ângulo ({depois_do_corte})')


def test_a_dec_otimista_degrada_e_DELATA_em_vez_de_oscilar():
    """O lado ERRADO da desigualdade, e o motivo de o padrão ser baixo.

    Com a lei supondo 1,5 contra 0,5 real, a sobra prevista fica 3x menor que
    a real: o corte vem tarde, ele passa do alvo, volta, passa de novo. É o S
    de 27-07 no pivô. Foi assim que o padrão 0,8 reprovou e virou 0,6.

    O que se exige aqui não é fechar — é **não fingir que fechou**: a lei tem
    de gastar o orçamento e sair com motivo escrito.
    """
    lei = PivoPorCorte(a_dec=1.5)
    erro, estado, _ = gira(lei, math.radians(120),
                           Planta(a_real=0.5, **ATUADOR_QUE_OBEDECE))
    assert estado == DESISTIU, 'violar a condição não pode terminar em PRONTO'
    assert lei.motivo, 'desistir calado é o BO-3'


def test_errar_a_dec_para_baixo_custa_pulso_e_nao_sobrepasso():
    """O achado de 27-07 medido aqui: com `a_dec` MUITO conservador ele corta
    cedo e precisa de mais mordidas — mas não passa do alvo. É o lado seguro
    do erro, e é por isso que o padrão é baixo.

    ⚠️ O lado seguro tem preço e ele é o ORÇAMENTO: contra o atuador que
    obedece, 120° custam 3 pulsos a 1,7x de conservadorismo, 4 a 2,5x, 6 a
    3,8x — e a 5x a manobra estoura os `max_pulsos` e sai por DESISTIU. Errar
    para baixo é de graça em sobrepasso, não em mordidas.
    """
    p = Planta(a_real=1.0, **ATUADOR_QUE_OBEDECE)
    lei = PivoPorCorte(a_dec=0.4)          # 2,5x conservador
    erro, estado, pulsos = gira(lei, math.radians(120), p)
    assert estado == PRONTO and pulsos > 1, 'esperado mais de uma mordida'
    assert math.degrees(p.yaw) <= 120 + math.degrees(lei.tolerancia)


# ------------------------------------------------------- o delator (BO-3)

def test_delata_comando_sem_movimento_em_vez_de_insistir_calado():
    """BO-3 na dimensão do giro: comando saindo, robô imóvel, log limpo. A
    planta aqui exige 5 s ligado para sair do lugar — nunca sai. A lei tem de
    DESISTIR e dizer por quê, não ficar presa."""
    lei = PivoPorCorte()
    _, estado, _ = gira(lei, math.radians(90), Planta(piso_tempo=5.0))
    assert estado == DESISTIU
    assert 'não se mexeu' in lei.motivo and 'BO-3' in lei.motivo


def test_orcamento_de_tempo_termina_a_manobra():
    """Teto de tempo, como o da ré na decisão 007: manobra que não fecha tem
    de acabar, e acabar falando."""
    lei = PivoPorCorte(teto_tempo=2.0)
    _, estado, _ = gira(lei, math.radians(180), Planta(a_real=0.05))
    assert estado == DESISTIU and 'orçamento' in lei.motivo


def test_desiste_depois_de_gastar_os_pulsos():
    lei = PivoPorCorte(max_pulsos=2, tolerancia=math.radians(4.0))
    _, estado, pulsos = gira(lei, math.radians(150), Planta(a_real=0.3))
    if estado == DESISTIU:
        assert pulsos <= 2 and lei.motivo


# ---------------------------------------------------------- casos de borda

def test_ja_esta_no_alvo_nao_manda_comando():
    """Erro dentro da tolerância na largada: não existe manobra. Mandar giro
    aqui gastaria os ~113° de sobra para nada."""
    lei = PivoPorCorte()
    wz, estado = lei.passo(math.radians(2.0), 0.0, 0.02)
    assert wz == 0.0 and estado == ASSENTANDO
    wz, estado = lei.passo(math.radians(2.0), 0.0, 0.02)
    assert wz == 0.0 and estado == PRONTO


def test_o_comando_tem_o_sinal_do_erro():
    lei = PivoPorCorte()
    assert lei.passo(math.radians(90), 0.0, 0.02)[0] > 0
    lei.reset()
    assert lei.passo(math.radians(-90), 0.0, 0.02)[0] < 0


def test_corta_quando_a_sobra_alcanca_o_erro():
    """O coração da lei: girando a 2,0 rad/s com a_dec 1,0, a sobra é 2,0 rad.
    Erro de 1,5 rad já está DENTRO dela — corta agora."""
    lei = PivoPorCorte(a_dec=1.0)
    assert lei.passo(1.5, 2.0, 0.02) == (0.0, ASSENTANDO)
    lei.reset()
    # Erro de 3,0 rad ainda é maior que a sobra: segue girando.
    assert lei.passo(3.0, 2.0, 0.02)[1] == GIRANDO
