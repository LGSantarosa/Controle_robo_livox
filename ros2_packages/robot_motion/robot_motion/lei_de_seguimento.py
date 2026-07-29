#!/usr/bin/env python3
"""Lei de seguimento de caminho — a camada entre o Nav2 e a movimentação.

Puro, sem ROS, testável sozinho — mesma forma da `lei_de_rumo.py`, e pelo mesmo
motivo: o que decide o comportamento do robô tem que poder ser exercitado sem
subir simulador nenhum.

O papel desta camada, e o que ela deliberadamente NÃO faz:

    o Nav2 diz POR ONDE ir (o `/plan`, Smac Hybrid-A* — decisão 008)
    esta lei diz PARA ONDE OLHAR e QUÃO RÁPIDO (rumo alvo + velocidade alvo)
    a `lei_de_rumo` diz O QUE O ATUADOR AGUENTA (decisão 005)

Ela não conhece zona morta, não conhece `a_dec` de giro e não fala com roda.
Tudo isso é da movimentação, onde já está caracterizado. Aqui só entra a
geometria do caminho.

Os três números que ela produz vêm de defeito medido, não de gosto:

1. **carrot** — mira um ponto à frente NO CAMINHO, não o destino lá no fim.
   Sem isso o robô corta a curva por dentro e raspa a quina do vão, em vez de
   seguir a forma que o planner desenhou.
2. **lookahead derivado do raio mínimo**, não escolhido. É o parâmetro central
   deste nó, e número solto aqui viraria mais um valor herdado sem
   justificativa — como a bitola 0,32 e o raio de roda 0,0825, que chegaram
   errados até a trena de 29-07.
3. **teto de velocidade pela curva**. `raio = v/wz`: com o giro no teto e a
   linear no teto, o robô é OBRIGADO a descrever um arco de `v/wz_max`. Foi
   esse o "balão" que fez um alvo a 0,43 m custar 3,66 m de caminho e 57 s na
   sessão de 28-07. A linear cede para a curva caber.
"""
import math

RETO = float('inf')


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def lookahead_de(raio_min, fator=1.5, piso=0.30):
    """Distância do carrot [m], derivada do raio que a máquina fecha.

    Amarrado ao raio mínimo para acompanhar a máquina quando a zona morta for
    medida: hoje o raio realizado é 0,370 m no perfil otimista e 0,463 m no
    pessimista, e um lookahead fixo estaria certo para um e errado para o outro.

    O PISO existe porque carrot colado no robô faz o rumo alvo oscilar com
    qualquer ruído de pose — é o ciclo-limite de rumo (o S de 27-07) entrando
    por outra porta.
    """
    return max(piso, fator * raio_min)


def indice_mais_proximo(caminho, x, y):
    """Onde o robô está, em índice do caminho."""
    melhor, melhor_d = 0, float('inf')
    for i, (px, py) in enumerate(caminho):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < melhor_d:
            melhor, melhor_d = i, d
    return melhor


def carrot(caminho, i0, lookahead):
    """Ponto a `lookahead` à frente de `i0`, medido pelo ARCO do caminho.

    Pelo arco e não pela distância em linha reta: numa curva fechada o ponto a
    0,7 m de distância euclidiana está bem mais adiante no caminho, e mirar
    nele corta a curva por dentro. Devolve `(índice, ponto)`; se o caminho
    acabar antes, devolve o último ponto — que é o destino.
    """
    andado = 0.0
    for k in range(i0, len(caminho) - 1):
        andado += math.hypot(caminho[k + 1][0] - caminho[k][0],
                             caminho[k + 1][1] - caminho[k][1])
        if andado >= lookahead:
            return k + 1, caminho[k + 1]
    return len(caminho) - 1, caminho[-1]


def rumo_para(x, y, alvo):
    """Rumo do robô até um ponto [rad]."""
    return math.atan2(alvo[1] - y, alvo[0] - x)


def curvatura_adiante(caminho, i0, janela):
    """Raio da curva mais fechada dentro de `janela` metros à frente [m].

    Olhada ADIANTE, não no ponto atual: a máquina tem distância de frenagem
    (é a premissa inteira da decisão 005), então frear em cima da curva é frear
    tarde. Devolve `inf` quando o trecho é reto.

    Reamostra a 0,20 m antes de medir, pelo mesmo motivo anotado na bancada do
    planner: o caminho do Smac vem com pontos a ~5 cm, e três pontos vizinhos
    assim descrevem o passo, não a curva.
    """
    ralos = [caminho[i0]]
    andado = 0.0
    for k in range(i0, len(caminho) - 1):
        andado += math.hypot(caminho[k + 1][0] - caminho[k][0],
                             caminho[k + 1][1] - caminho[k][1])
        if andado > janela:
            break
        p = caminho[k + 1]
        if math.hypot(p[0] - ralos[-1][0], p[1] - ralos[-1][1]) >= 0.20:
            ralos.append(p)

    raio = RETO
    for a, b, c in zip(ralos, ralos[1:], ralos[2:]):
        r1 = math.atan2(b[1] - a[1], b[0] - a[0])
        r2 = math.atan2(c[1] - b[1], c[0] - b[0])
        d = abs(norm_ang(r2 - r1))
        passo = math.hypot(c[0] - b[0], c[1] - b[1])
        if d > 1e-6 and passo > 1e-6:
            raio = min(raio, passo / (2.0 * math.sin(min(d, math.pi) / 2.0)))
    return raio


def velocidade_de_seguimento(dist_ao_fim, raio_da_curva, v_max, a_lin, wz_max):
    """Velocidade de cruzeiro [m/s]: o menor entre três limites.

    1. **o teto da máquina** (`v_max`);
    2. **a frenagem até o fim do caminho**, `sqrt(2·a_lin·d)` — mesma lei da
       decisão 005, agora em distância. Parâmetro físico, não ganho ajustado;
    3. **o teto pela curva**, `raio · wz_max` — se a linear passar disso, o
       arco que o robô descreve fica mais aberto que a curva do plano e ele sai
       por fora. É o balão de 28-07.

    O piso da zona morta NÃO entra aqui: ele é da movimentação, que conhece a
    zona morta medida. Esta camada pede; a de baixo defende (BO-3).
    """
    v = min(v_max, math.sqrt(max(0.0, 2.0 * a_lin * dist_ao_fim)))
    if not math.isinf(raio_da_curva):
        v = min(v, raio_da_curva * wz_max)
    return v


# --------------------------------------------------------- a ré por gatilho
#
# Decisão 009. O plano não dá ré (o planner roda em Dubins), então quem recua é
# esta camada — e o gatilho é o SINTOMA, não a geometria.
#
# Por que sintoma, se a decisão 007 o havia descartado: lá o argumento era que
# esperar o sintoma "gasta N segundos de órbita toda vez". É verdade, e é o
# preço certo a pagar. O gatilho geométrico age cedo e SEMPRE que a conta diz
# que não cabe — inclusive em situações que se resolveriam sozinhas — e é assim
# que nasce o ciclo "ré e anda", reprovado duas vezes com dado: 28-07 no clique
# do dono (12 entradas, período 2,10 s) e 29-07 na bancada (5 entradas, 1,85 m
# de caminho para um alvo a 0,40 m).
#
# Sintoma dispara raro e tarde. É a propriedade que se quer.


class ProgressoDeAvanco:
    """Diz se o robô parou de se aproximar do objetivo.

    Mede aproximação, não velocidade: robô que anda em círculo tem velocidade e
    não tem progresso, e é exatamente o caso que interessa (a órbita).

    Só acusa depois de `parado_s` CONTÍNUOS sem ganhar `avanco_min` metros. O
    relógio zera a cada avanço real — dois travamentos curtos separados não
    somam para virar um disparo.
    """

    def __init__(self, parado_s=1.5, avanco_min=0.05):
        self.parado_s = parado_s
        self.avanco_min = avanco_min
        self.melhor = None      # menor distância já vista neste objetivo
        self.desde = None       # instante em que a melhor marca parou de cair

    def reinicia(self):
        self.melhor = None
        self.desde = None

    def atualiza(self, t, dist_ao_objetivo):
        """Devolve True enquanto o robô estiver emperrado."""
        if self.melhor is None or dist_ao_objetivo <= self.melhor - self.avanco_min:
            self.melhor = dist_ao_objetivo
            self.desde = t
            return False
        return (t - self.desde) > self.parado_s


def orcamento_de_re(vao_traseiro=None, folga=0.30, cego=0.30):
    """Quantos metros de ré são permitidos [m].

    Sem `vao_traseiro` a ré é **cega**: o Mid-360 é 360° e vai enxergar atrás,
    mas ainda não está no modelo do simulador. Aposta cega tem que ser curta —
    daí o `cego` valer o mesmo que a folga, e não mais.

    Com vão medido, recua o que há descontando a folga. Vão menor que a folga
    **proíbe** a ré: é zero, não "um pouquinho", porque encostar devagar também
    é bater.
    """
    if vao_traseiro is None:
        return cego
    return max(0.0, vao_traseiro - folga)


def re_esgotada(recuado, orcamento, t_na_re, teto_s=8.0):
    """A manobra acabou — por metros ou por tempo.

    O teto de TEMPO não é redundante com o de metros: se a pose não mudar (roda
    patinando, comando engolido pela zona morta), o orçamento em metros nunca é
    gasto e a ré duraria para sempre. É o BO-3 visto de outro ângulo — comando
    saindo, robô parado — e a defesa é a mesma: não confiar que o movimento
    aconteceu só porque foi pedido.
    """
    return recuado >= orcamento or t_na_re > teto_s
