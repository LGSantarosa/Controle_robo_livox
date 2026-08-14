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


def desvio_da_corda(caminho, i0, distancia):
    """Maior desvio do caminho até a corda, em `distancia` m à frente [m].

    A régua de "o trecho à frente é reto?". Corda e não curvatura porque é a
    corda que diz quanto se PERDE mirando longe: o carrot esticado mira o fim
    do trecho, e o desvio da corda é exatamente o quanto o caminho real se
    afasta dessa mira.
    """
    fim = None
    andado = 0.0
    pontos = [caminho[i0]]
    for k in range(i0, len(caminho) - 1):
        andado += math.hypot(caminho[k + 1][0] - caminho[k][0],
                             caminho[k + 1][1] - caminho[k][1])
        pontos.append(caminho[k + 1])
        if andado >= distancia:
            fim = caminho[k + 1]
            break
    if fim is None:
        fim = caminho[-1]
    ax, ay = pontos[0]
    bx, by = fim
    vx, vy = bx - ax, by - ay
    n = math.hypot(vx, vy)
    if n < 1e-9:
        return 0.0
    pior = 0.0
    for (px, py) in pontos:
        # distância do ponto à RETA que liga início e fim
        pior = max(pior, abs(vx * (py - ay) - vy * (px - ax)) / n)
    return pior


def vao_no_corredor_frontal(distancias, angulo_min, incremento, largura,
                            avanco, alcance_max=None):
    """Vão livre à FRENTE do para-choque, em metros [m].

    Espelho exato do `vao_no_corredor_traseiro`, e pela mesma razão de forma:
    retângulo (a forma do robô) e não setor angular, porque cone é cego para a
    quina — o argumento inteiro está na irmã de trás, e ele não muda de lado.

    Existe para o gate da mira adaptativa: esticar o carrot só faz sentido com
    ESPAÇO à frente. Passagem apertada é parede perto por definição.
    """
    meia = largura / 2.0
    vao = RETO
    for i, r in enumerate(distancias):
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        if alcance_max is not None and r > alcance_max:
            continue
        a = angulo_min + i * incremento
        x, y = r * math.cos(a), r * math.sin(a)
        if x <= 0.0 or abs(y) > meia:
            continue
        vao = min(vao, max(0.0, x - avanco))
    return vao


class MiraAdaptativa:
    """O carrot que ESTICA em reta e encolhe em curva. Uma vez por ciclo::

        la = mira.passo(caminho, i0, vao_frente)

    🔴 DE ONDE VEM — o robô 1 (`Controle_robo_web`), que passou por isto antes
    e mediu (`path_follower.py`, 07-08): *"carrot 0.6 amplifica ruído de pose
    (12 cm lateral = 12°) -> 184 giros no lugar, 127 <10°, zigue-zague em
    corredor. A 1.5 m os mesmos 12 cm = ~4,6° -> segue reto."*

    É a mesma lei que a corrida A de 14-08 mediu aqui, por outro caminho: o
    plano salta de lado no replanejamento (p90 5,8 cm, max 15,1 cm) e o rumo
    pedido muda por `atan(salto / lookahead)`. Com a mira de 0,37 m que este
    robô usava, 15 cm viram **22°** — e a amplitude p90 medida da referência
    foi 20,0°. Com 1,0 m os mesmos 15 cm viram 8,6°.

    ⚠️ E o esticão é CONDICIONAL, que é a parte que o robô 1 aprendeu errando:
    mira longa corta curva por dentro. Estica só quando o trecho à frente é
    reto E há espaço medido pelo SCAN — passagem apertada é parede perto por
    definição, e ali a mira curta é que segura a linha.

    ⚠️ A HISTERESE é sobre a decisão estica/encolhe, não sobre girar. Sem ela
    o carrot fica pulando entre 0,37 e 1,0 m na mesma fronteira, e isso é o
    limite-ciclo do robô 1 (*"girava e parava no MESMO limiar -> pulinhos"*)
    entrando por outra porta. Encolher pede desvio maior do que esticar pediu.

    Os dois limiares saem da GEOMETRIA, medidos com `desvio_da_corda` sobre
    1,0 m de caminho — não de varredura:

        R = 0,37 m (o mais fechado que a máquina faz)   desvio 0,293
        R = 1,00 m                                             0,125
        R = 1,50 m                                             0,090
        R = 2,00 m                                             0,068
        reta                                                   0,000

    `tol_estica = 0,07` = estica só em curva de raio >= ~2 m (suave de
    verdade); `tol_encolhe = 0,12` = encolhe em raio <= ~1 m (curva de
    verdade). A banda entre os dois é a histerese, e ela vale 1,7x.
    """

    def __init__(self, curto, longo, tol_estica=0.07, tol_encolhe=0.12,
                 folga_min=0.60):
        if not longo > curto:
            raise ValueError('a mira longa tem de ser maior que a curta')
        if not tol_encolhe > tol_estica:
            raise ValueError('sem histerese: tol_encolhe tem de ser > tol_estica')
        self.curto = curto
        self.longo = longo
        self.tol_estica = tol_estica
        self.tol_encolhe = tol_encolhe
        self.folga_min = folga_min
        self.esticada = False

    def reset(self):
        self.esticada = False

    def passo(self, caminho, i0, vao_frente=None):
        # O gate do espaço NÃO tem histerese: encolher é o lado seguro do erro,
        # e hesitar em encolher perto de parede é exatamente o que não pode.
        if vao_frente is not None and vao_frente < self.folga_min:
            self.esticada = False
            return self.curto
        desvio = desvio_da_corda(caminho, i0, self.longo)
        if self.esticada:
            if desvio > self.tol_encolhe:
                self.esticada = False
        elif desvio <= self.tol_estica:
            self.esticada = True
        return self.longo if self.esticada else self.curto


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


def desvio_lateral(caminho, i0, x, y, janela=6):
    """Distância do robô ao caminho, COM SINAL [m]: + à esquerda, − à direita.

    O sinal é o que faltava. `rumo_para(x, y, carrot)` já corrige desvio de
    forma implícita — mirando o carrot o robô volta para o caminho — mas só
    depois que o desvio virou ângulo, e num vão de porta esse atraso é o
    defeito inteiro. Sem o sinal não dá para realimentar: módulo não sabe para
    que lado corrigir.

    Mede contra o SEGMENTO mais próximo numa janela em torno de `i0`, e não
    contra o ponto mais próximo: ponto a ponto o caminho do planner vem a ~5 cm,
    e a distância ao vértice mais próximo satura em ~2,5 cm mesmo com o robô
    bem fora da linha.
    """
    a = max(0, i0 - janela)
    b = min(len(caminho) - 1, i0 + janela)
    melhor, sinal = float('inf'), 0.0
    for k in range(a, b):
        ax, ay = caminho[k]
        bx, by = caminho[k + 1]
        vx, vy = bx - ax, by - ay
        n2 = vx * vx + vy * vy
        if n2 < 1e-12:
            continue
        u = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / n2))
        px, py = ax + u * vx, ay + u * vy
        d = math.hypot(x - px, y - py)
        if d < melhor:
            melhor = d
            # Produto vetorial do segmento com o vetor até o robô: positivo
            # quando o robô está à ESQUERDA de quem percorre o caminho.
            sinal = math.copysign(1.0, vx * (y - ay) - vy * (x - ax))
    return 0.0 if math.isinf(melhor) else melhor * sinal


def correcao_de_desvio(e_lat, v, k_lat, v_ref=0.20, teto=math.radians(30.0)):
    """Só o ÂNGULO da correção de Stanley [rad], sem aplicá-lo.

    Separado de `rumo_com_desvio` porque o limite de taxa (`CorrecaoDeDesvio`)
    precisa do alvo da correção para poder persegui-lo devagar.
    """
    if k_lat == 0.0:
        return 0.0
    corr = math.atan2(k_lat * e_lat, max(abs(v), v_ref))
    return max(-teto, min(teto, corr))


class CorrecaoDeDesvio:
    """A correção de Stanley com limite de TAXA. Uma vez por ciclo::

        rumo_alvo = corr.passo(rumo_carrot, e_lat, v, dt)

    🔴 POR QUE O LIMITE EXISTE — medido na corrida B de 14-08, que REPROVOU a
    versão sem ele. O Nav2 republica o plano a 0,31 Hz, e entre um plano e o
    seguinte o ponto que o seguidor mira salta de lado:

        salto do alvo (plano suavizado)   p50 2,9 cm   p90 5,8 cm   max 15,1 cm

    Salto de posição é DEGRAU em `e_lat`, e sem limite de taxa o termo o
    converte em degrau de rumo no mesmo ciclo — contra uma placa que tem ~0,5 s
    de tempo morto. Resultado medido com `k_lat=1,0` e sem limite:

        amplitude p90 da referência   20,0° -> 39,8°     (dobrou)
        pico                          50,3° -> 88,1°
        período p50                    1,00 s -> 0,70 s

    Com o limite, o mesmo degrau vira RAMPA: a correção anda no máximo
    `taxa_max` por segundo, então ela se completa em ~2 s no pior caso (teto de
    30°). Isso é 4× o tempo morto da placa — a malha enxerga a correção como
    movimento lento, que é a condição para não oscilar — e fica bem abaixo dos
    ~57°/s que a máquina fecha com `wz_max` de 1,0 rad/s.

    ⚠️ O limite é sobre a CORREÇÃO, não sobre o rumo alvo inteiro. O rumo do
    carrot também dá degrau no replanejamento (57% dos degraus grandes da
    corrida A, que rodou com `k_lat=0`), mas isso é defeito PRÉ-EXISTENTE e de
    outra natureza — limitar o rumo inteiro atrasaria também o pivô da 036 e a
    saída da ré, que precisam ser rápidos.
    """

    def __init__(self, k_lat=0.0, v_ref=0.20, teto=math.radians(30.0),
                 taxa_max=math.radians(15.0)):
        if taxa_max <= 0.0:
            raise ValueError('taxa_max tem de ser positiva — é rad/s')
        self.k_lat = k_lat
        self.v_ref = v_ref
        self.teto = teto
        self.taxa_max = taxa_max
        self.corr = 0.0

    def reset(self):
        """Zera a correção acumulada.

        Chamado quando o seguidor para ou troca de objetivo: correção velha
        aplicada a caminho novo é comando sem dono, e foi assim que a ré da 025
        nasceu do nada em 13-08.
        """
        self.corr = 0.0

    def passo(self, rumo_carrot, e_lat, v, dt):
        if self.k_lat == 0.0:
            self.corr = 0.0
            return norm_ang(rumo_carrot)
        alvo = correcao_de_desvio(e_lat, v, self.k_lat, self.v_ref, self.teto)
        passo_max = self.taxa_max * dt
        self.corr += max(-passo_max, min(passo_max, alvo - self.corr))
        # Erro à ESQUERDA (e_lat > 0) pede rumo à DIREITA: subtrai.
        return norm_ang(rumo_carrot - self.corr)


def rumo_com_desvio(rumo_carrot, e_lat, v, k_lat, v_ref=0.20,
                    teto=math.radians(30.0)):
    """Rumo alvo do carrot corrigido pelo desvio lateral [rad].

    O termo é o de Stanley: `atan2(k·e, v)`. A propriedade que o justifica é a
    dinâmica que ele impõe ao erro — com a correção aplicada,

        ė = −v·sen(δ) ≈ −k·e

    ou seja, **o erro lateral decai com constante de tempo 1/k segundos,
    independente da velocidade**. É o que se quer aqui: perto da porta o robô
    anda devagar, e uma correção proporcional pura (sem o `v` no denominador)
    ficaria fraca justo onde ela precisa ser forte.

    Com `k_lat = 0` esta função devolve o `rumo_carrot` intacto — a lei nasce
    NEUTRA, e ligá-la é uma mudança de parâmetro que se desfaz sozinha. Mesmo
    cuidado da 038.

    - `v_ref` é PISO do denominador, não a velocidade real: em `v → 0` o termo
      pediria 90° e o robô giraria parado em cima do caminho.
    - `teto` limita o pedido a 30°: acima disso a correção deixa de ser
      correção e vira manobra, e manobra tem dono (o pivô, a ré).
    """
    if k_lat == 0.0:
        return norm_ang(rumo_carrot)
    corr = math.atan2(k_lat * e_lat, max(abs(v), v_ref))
    corr = max(-teto, min(teto, corr))
    # Erro à ESQUERDA (e_lat > 0) pede rumo para a DIREITA: subtrai.
    return norm_ang(rumo_carrot - corr)


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


def vao_no_corredor_traseiro(distancias, angulo_min, incremento, largura,
                             recuo, alcance_max=None):
    """Vão livre atrás do PARA-CHOQUE, em metros [m].

    Mede no **corredor retangular que o corpo varre dando ré** — largura do
    robô, para trás — e não num setor angular.

    🔴 A FORMA IMPORTA, E ISSO CUSTOU UMA BATIDA. A versão por setor (um cone
    de ±30° atrás) é **cega para a quina**: um obstáculo encostado no canto
    traseiro aparece num feixe cujo ângulo cai FORA do cone, e a checagem diz
    "livre" enquanto o corpo já vai raspar nele. Um cone só cobre o corpo se
    for mais largo que o robô a TODA distância, e nenhum ângulo fixo faz isso —
    perto ele é estreito demais, longe é largo demais e para por causa de
    parede que não está no caminho.

    O retângulo é a forma certa porque é a forma do robô: o que importa é `|y|`
    dentro da meia-largura, qualquer que seja o ângulo em que o feixe chegou.

    Convenção: `+x` é a frente do robô, `recuo` é a distância do centro ao
    para-choque traseiro. Devolve `inf` quando não há nada no corredor, e
    **0,0** quando já há coisa encostada — nunca negativo, porque orçamento
    negativo somado com folga viraria permissão.
    """
    meia = largura / 2.0
    vao = RETO
    for i, r in enumerate(distancias):
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue                       # feixe inválido não é vão livre
        if alcance_max is not None and r > alcance_max:
            continue                       # além do alcance útil: não conta
        a = angulo_min + i * incremento
        x, y = r * math.cos(a), r * math.sin(a)
        if x >= 0.0 or abs(y) > meia:
            continue                       # não está no corredor de trás
        vao = min(vao, max(0.0, -x - recuo))
    return vao


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


# ------------------------------------------------------------- a chegada
#
# Duas lições medidas da decisão 006, e as duas custaram corrida.


def raio_de_chegada_minimo(v_piso, a_lin):
    """Menor raio de chegada que não faz o robô orbitar o ponto [m].

    Ele não sabe ir mais devagar que `v_piso` — abaixo disso a placa engole o
    comando (BO-3). Então ele entra no raio com essa velocidade e precisa de
    `v_piso²/(2·a_lin)` para parar. Raio menor que isso: atravessa, sai do outro
    lado, volta, para sempre.

    Com os números de hoje (piso 0,335 m/s no perfil pessimista, `a_lin` 0,3)
    isso dá ~0,19 m. Ou seja: **a zona morta não encarece só a manobra, encarece
    a PRECISÃO**, e com o quadrado. Não há ganho que conserte.
    """
    return v_piso * v_piso / (2.0 * a_lin)


def chegou(dist_ao_objetivo, raio):
    return dist_ao_objetivo <= raio


def comando_de_parada():
    """O comando de quem chegou: zero linear e **zero giro**.

    O giro zerado não é detalhe. Na sessão de 27-07 o robô chegava e continuava
    girando para acertar o rumo, arrastando-se para fora do ponto: 0,06 m de
    erro viravam 0,27 m. Rumo na chegada não é requisito deste seguidor, e
    persegui-lo custa a própria chegada.
    """
    return 0.0, 0.0
