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
from typing import NamedTuple

RETO = float('inf')


class PassagemEstreita(NamedTuple):
    """Trecho do caminho que cruza um gargalo conhecido no mapa.

    Os índices pertencem ao caminho que foi analisado. ``inicio`` e ``fim``
    cobrem as amostras que de fato ficam entre duas paredes; ``centro`` é uma
    amostra no meio desse trecho e serve para congelar o eixo da travessia.
    A largura é a distância entre as primeiras células ocupadas dos dois
    lados, medida perpendicularmente ao caminho.
    """
    inicio: int
    centro: int
    fim: int
    largura: float


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def rumo_local_do_caminho(caminho, i, janela=0.10):
    """Tangente do caminho em ``i``, usando arco dos dois lados [rad].

    Usar pontos vizinhos crus mede o serrilhado de 5 cm do planner. A pequena
    janela bilateral preserva o eixo de uma porta de 20 cm de espessura sem
    puxar para dentro dela a curva de aproximação que vem antes.
    """
    if len(caminho) < 2:
        return 0.0
    i = max(0, min(int(i), len(caminho) - 1))
    a = i
    andado = 0.0
    while a > 0 and andado < janela:
        andado += math.dist(caminho[a], caminho[a - 1])
        a -= 1
    b = i
    andado = 0.0
    while b < len(caminho) - 1 and andado < janela:
        andado += math.dist(caminho[b], caminho[b + 1])
        b += 1
    if a == b:
        return 0.0
    return math.atan2(caminho[b][1] - caminho[a][1],
                      caminho[b][0] - caminho[a][0])


def passagens_estreitas(caminho, dados, largura_grade, altura_grade,
                        resolucao, origem=(0.0, 0.0, 0.0),
                        largura_min=0.55, largura_max=1.10,
                        ocupado_min=65):
    """Detecta gargalos que o *caminho aceito* cruza no mapa estático.

    Para cada ponto, lança dois raios perpendiculares à tangente local. Duas
    paredes, uma de cada lado, com distância total entre ``largura_min`` e
    ``largura_max`` formam uma passagem. Amostras contíguas viram uma única
    :class:`PassagemEstreita`.

    A função não desloca o caminho para o centro calculado pelas células. O
    mapa é rasterizado e pode ganhar/perder um pixel na borda; o plano
    suavizado já escolheu a linha navegável. O mapa só responde *"aqui é
    estreito"*, enquanto o próprio plano continua respondendo *"por onde"*.

    Célula desconhecida e fora do mapa contam como ocupadas. Isso torna uma
    ausência de mapa conservadora, nunca uma licença para inventar um vão.
    """
    if (len(caminho) < 2 or resolucao <= 0.0 or largura_grade <= 0
            or altura_grade <= 0
            or len(dados) != largura_grade * altura_grade
            or not largura_max > largura_min > 0.0):
        return []

    ox, oy, oyaw = origem
    co, so = math.cos(oyaw), math.sin(oyaw)

    def ocupada(x, y):
        # mundo -> referencial da grade (a origem de OccupancyGrid pode girar)
        dx, dy = x - ox, y - oy
        gx = co * dx + so * dy
        gy = -so * dx + co * dy
        col = math.floor(gx / resolucao)
        lin = math.floor(gy / resolucao)
        if not (0 <= col < largura_grade and 0 <= lin < altura_grade):
            return True
        valor = dados[lin * largura_grade + col]
        return valor < 0 or valor >= ocupado_min

    # Meio pixel evita pular uma parede fina situada entre duas amostras. O
    # teto individual precisa ser ``largura_max`` (não a metade): caminho
    # fora do centro ainda deve reconhecer a mesma porta.
    passo = resolucao / 2.0

    def primeiro_obstaculo(p, angulo):
        d = passo
        while d <= largura_max + 1e-9:
            if ocupada(p[0] + d * math.cos(angulo),
                       p[1] + d * math.sin(angulo)):
                return d
            d += passo
        return None

    candidatos = []
    for i, p in enumerate(caminho):
        if ocupada(*p):
            continue
        rumo = rumo_local_do_caminho(caminho, i)
        esquerda = primeiro_obstaculo(p, rumo + math.pi / 2.0)
        direita = primeiro_obstaculo(p, rumo - math.pi / 2.0)
        if esquerda is None or direita is None:
            continue
        largura = esquerda + direita
        if largura_min <= largura <= largura_max:
            candidatos.append((i, largura))

    if not candidatos:
        return []

    grupos = []
    for candidato in candidatos:
        # Um pixel de falha no encontro raio/parede não pode quebrar a mesma
        # porta em duas. Três índices ainda são só ~15 cm no plano de 5 cm.
        if not grupos or candidato[0] - grupos[-1][-1][0] > 3:
            grupos.append([candidato])
        else:
            grupos[-1].append(candidato)

    saida = []
    for grupo in grupos:
        inicio, fim = grupo[0][0], grupo[-1][0]
        meio = (inicio + fim) / 2.0
        # Escolhe a amostra central entre as de largura mínima. Isso põe o
        # eixo no meio da espessura da parede, não na primeira face vista.
        menor = min(x[1] for x in grupo)
        quase_minimos = [x for x in grupo if x[1] <= menor + resolucao]
        indice, largura = min(quase_minimos,
                              key=lambda x: abs(x[0] - meio))
        saida.append(PassagemEstreita(inicio, indice, fim, largura))
    return saida


def alvo_estavel_de_passagem(caminho, passagem, x, y, rumo_atual,
                             saida=1.00, meia_largura=0.2275, margem=0.03,
                             tolerancia_lateral=0.08,
                             tolerancia_rumo=math.radians(10.0),
                             eixo_comprometido=False):
    """Alvo congelado para entrar no gargalo e atravessá-lo sem trocar de lado.

    O eixo é a tangente do plano no centro detectado. O alvo normal é um ponto
    fixo ``saida`` metros depois do vão. Antes de adotá-lo, exige três coisas:
    a reta projetada cabe, o centro do robô já está perto do eixo E o rumo
    físico já acompanha esse eixo. Até lá mira o centro da passagem. Isso é
    importante nesta máquina: trocar a referência só porque a *reta virtual*
    cabe deixou a placa executar por mais 0,52 s a curva anterior e entrar na
    porta ainda a 53 graus.

    Devolve ``(alvo, fase)`` onde fase é ``'centro'`` ou ``'eixo'``. Não há
    ``eixo_comprometido`` é o latch explícito da cola ROS: depois que a fase
    eixo começou, uma oscilação de pose não a devolve ao centro — mas uma
    DERIVA devolve, se o desvio passar da folga que o vão ainda oferece
    (20-08; ver o bloco comentado abaixo). Para a mesma pose, passagem e latch
    a decisão continua determinística.
    """
    if not caminho:
        raise ValueError('caminho vazio')
    i = max(0, min(passagem.centro, len(caminho) - 1))
    cx, cy = caminho[i]
    rumo = rumo_local_do_caminho(caminho, i)
    tx, ty = math.cos(rumo), math.sin(rumo)
    nx, ny = -ty, tx
    alvo_eixo = (cx + saida * tx, cy + saida * ty)

    px, py = x - cx, y - cy
    s = px * tx + py * ty
    d = px * nx + py * ny
    folga = passagem.largura / 2.0 - meia_largura - margem
    if folga <= 0.0:
        return (cx, cy), 'centro'

    # Interseção da reta pose->alvo_eixo com s=0. Para s>=0 o ponto mais
    # estreito já ficou para trás e mirar o centro faria o alvo ir para trás.
    if s >= 0.0:
        return alvo_eixo, 'eixo'

    # 🔴 20-08: O LATCH GANHA SAÍDA, e é por medida. Na primeira corrida do
    # corredor real no Gazebo (`docs/dados/2026-08-20-andar3-gargalo-ligado`) o
    # rumo ficou ótimo — |erro| p50 4,1° contra 14,6° sem o modo — e mesmo
    # assim ele não passou: chegou à soleira em `x = 7,20` num vão que termina
    # em 7,29, com a borda do corpo invadindo **14 cm**. O desvio p50 era
    # 0,195 m contra uma tolerância de entrada de 0,08.
    #
    # O que aconteceu: a fase `centro` durou 27 amostras e a `eixo` 2161. Ele
    # entrou em eixo num instante em que estava alinhado e DERIVOU depois, e o
    # latch — permanente até aqui — não tinha como devolvê-lo ao centro. Um
    # latch que nunca solta não protege de oscilação: ele congela o primeiro
    # acerto e ignora todo o resto da aproximação.
    #
    # A saída é o CORPO, não a pose: sai do eixo só quando o desvio passa da
    # folga que sobra no vão (`largura/2 - meia_largura - margem`), ou seja
    # quando seguir reto deixa de caber. Isso já é histerese: entra com
    # `|d| <= tolerancia_lateral` (0,08) e só sai acima de `folga` (0,1075 no
    # vão de 0,73 m) — banda de 2,7 cm, larga o bastante para ruído de pose e
    # estreita o bastante para pegar deriva de 22 cm.
    if eixo_comprometido:
        return (alvo_eixo, 'eixo') if abs(d) <= folga else ((cx, cy), 'centro')

    if s < 0.0:
        d_no_centro = d * saida / (saida - s)
        if abs(d_no_centro) > folga:
            return (cx, cy), 'centro'

    alinhado_lateral = abs(d) <= max(0.0, tolerancia_lateral)
    alinhado_rumo = abs(norm_ang(rumo_atual - rumo)) <= max(
        0.0, tolerancia_rumo)
    if not (alinhado_lateral and alinhado_rumo):
        return (cx, cy), 'centro'
    return alvo_eixo, 'eixo'


def folga_radial(distancias, alcance_max=None):
    """Menor distância válida do LiDAR ao redor inteiro do robô [m].

    É a régua do pivô: transladar varre um retângulo, mas girar no lugar
    varre o círculo da quina física. ``inf`` significa que nenhum retorno
    válido foi visto; o chamador ainda precisa conferir frescor do scan.
    """
    melhor = RETO
    for r in distancias:
        if r is None or not math.isfinite(r) or r <= 0.0:
            continue
        if alcance_max is not None and r > alcance_max:
            continue
        melhor = min(melhor, r)
    return melhor


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
    verdade); `tol_encolhe = 0,08` = ao aparecer curva logo depois da reta,
    encolhe antes que o carrot de 1 m atravesse a quina. A banda estreita de
    1 cm ainda impede comutacao por ruido, sem sustentar a mira longa nos
    8--11 cm que fizeram o robo cortar a porta em 19-08.
    """

    def __init__(self, curto, longo, tol_estica=0.07, tol_encolhe=0.08,
                 folga_min=0.60, rumo_estica=math.radians(3.0),
                 rumo_encolhe=math.radians(5.0), rumo_passo=0.20):
        if not longo > curto:
            raise ValueError('a mira longa tem de ser maior que a curta')
        if not tol_encolhe > tol_estica:
            raise ValueError('sem histerese: tol_encolhe tem de ser > tol_estica')
        if not rumo_encolhe > rumo_estica >= 0.0:
            raise ValueError('limiares de rumo precisam de histerese valida')
        self.curto = curto
        self.longo = longo
        self.tol_estica = tol_estica
        self.tol_encolhe = tol_encolhe
        self.folga_min = folga_min
        self.rumo_estica = rumo_estica
        self.rumo_encolhe = rumo_encolhe
        if not rumo_passo > 0.0:
            raise ValueError('rumo_passo tem de ser positivo')
        # Menos de tres pontos na janela e a medida devolve 0,0 sempre — e um
        # zero que vem de nao ter medido autoriza a mira longa em curva.
        if longo / rumo_passo < 2.5:
            raise ValueError(
                'rumo_passo grosso demais para a mira longa: sobram menos de '
                'tres pontos na janela e a medida degenera em zero')
        self.rumo_passo = rumo_passo
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
        # A mira longa serve para filtrar ruido de uma RETA, nao para prever a
        # proxima manobra. O desvio da corda sozinho deixa uma curva no fim da
        # janela parecer suave e o carrot atravessa a quina. Qualquer mudanca
        # de direcao relevante no proximo metro obriga a cumprir o plano perto.
        mudanca = mudanca_de_rumo_adiante(caminho, i0, self.longo,
                                          passo=self.rumo_passo)
        if self.esticada:
            if desvio > self.tol_encolhe or mudanca > self.rumo_encolhe:
                self.esticada = False
        elif desvio <= self.tol_estica and mudanca <= self.rumo_estica:
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


def mudanca_de_rumo_adiante(caminho, i0, distancia, passo=0.20):
    """Maior mudanca de direcao no caminho dentro da janela [rad].

    Reamostra em trechos de `passo` para ignorar o serrilhado de 5 cm do
    planner. Compara cada trecho com o primeiro: uma curva no fim do proximo
    metro aparece mesmo quando o desvio da corda ainda e pequeno.

    ⚠️ 0,20 NAO FILTRAVA O BASTANTE, e era ESTA a regra que prendia a mira no
    curto — nao o `tol_estica`, que eu culpei primeiro. Medido em 20-08 sobre
    os planos reais de `/plan_smoothed` no `sala_andar3` (219 amostras):

        criterio                       passa
        desvio_da_corda <= 0,07         77,2%
        mudanca_de_rumo <= 3,0 graus    14,2%   <- o gargalo

    Com o serrilhado dentro da medida, a mudanca mediana no proximo metro dava
    6,4 graus contra um limiar de 3,0. Varrendo o passo, com TODOS os limiares
    intocados:

        passo    p50 da mudanca    estica em
        0,20 m       6,4 graus        14,2%
        0,40 m       3,9 graus        39,3%
        0,60 m       0,0 graus        77,2%   <- DEGENERADO, ver abaixo

    ⚠️ 0,60 NAO E O MELHOR, e sim invalido: com janela de 1,0 m sobra uma unica
    amostra intermediaria, `rumos` fica com menos de dois elementos e a funcao
    devolve 0,0 SEMPRE. Ela deixaria de medir e passaria a autorizar a mira
    longa em qualquer curva — exatamente o defeito de 19-08, que fez o carrot
    cortar a porta. Um zero que vem de nao ter medido nao e um plano reto.

    0,40 e o maior passo que ainda deixa tres pontos na janela. Ataca a MEDIDA
    e nao a protecao: `tol_estica`, `rumo_estica` e o gate de folga frontal
    seguem valendo com os numeros medidos da 042.
    """
    pontos = [caminho[i0]]
    d = passo
    while d <= distancia + 1e-9:
        _, p = carrot(caminho, i0, d)
        if math.dist(p, pontos[-1]) > 1e-6:
            pontos.append(p)
        d += passo
    # ⚠️ O FIM DA JANELA ENTRA SEMPRE, e não só quando calha de ser múltiplo do
    # passo. Com 0,20 em 1,0 m o último passo caía exatamente em 1,00 e isso
    # escondia a dependência; com 0,40 a varredura para em 0,80 e a curva do
    # último trecho sumia — a função devolvia 0,0 para um caminho que vira 90°
    # em 0,9 m, que é o caso de regressão da porta (19-08). Amostrar o fim
    # custa um ponto e é o que torna o passo uma escolha de FILTRO, não de
    # horizonte.
    _, fim_janela = carrot(caminho, i0, distancia)
    if math.dist(fim_janela, pontos[-1]) > 1e-6:
        pontos.append(fim_janela)
    if math.dist(pontos[-1], caminho[-1]) > 1e-6:
        # So inclui o fim se ele estiver dentro da janela; nao olha depois do
        # horizonte que esta decidindo a mira.
        comprimento = 0.0
        for a, b in zip(caminho[i0:], caminho[i0 + 1:]):
            comprimento += math.dist(a, b)
            if comprimento > distancia:
                break
        if comprimento <= distancia:
            pontos.append(caminho[-1])
    rumos = [math.atan2(b[1] - a[1], b[0] - a[0])
             for a, b in zip(pontos, pontos[1:])
             if math.dist(a, b) > 1e-6]
    if len(rumos) < 2:
        return 0.0
    inicial = rumos[0]
    return max(abs(norm_ang(r - inicial)) for r in rumos[1:])


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
    # Primeiro recorta a polilinha EXATAMENTE na janela. A implementacao
    # anterior simplesmente descartava o segmento que cruzava o limite. Com a
    # mira curta de 0,37 m e a reamostragem de 0,20 m isso deixava apenas dois
    # pontos; como curvatura precisa de tres, uma quina de 90 graus virava
    # `inf` (reta) e o seguidor entrava nela a v_max.
    trecho = [caminho[i0]]
    andado = 0.0
    for k in range(i0, len(caminho) - 1):
        a, b = caminho[k], caminho[k + 1]
        ds = math.hypot(b[0] - a[0], b[1] - a[1])
        if ds <= 1e-9:
            continue
        restante = janela - andado
        if restante <= 0.0:
            break
        if ds >= restante:
            f = restante / ds
            trecho.append((a[0] + f * (b[0] - a[0]),
                           a[1] + f * (b[1] - a[1])))
            andado = janela
            break
        trecho.append(b)
        andado += ds

    if andado <= 1e-9:
        return RETO

    # Reamostra a 20 cm para nao medir o serrilhado de 5 cm do planner. Em
    # janelas menores que 40 cm, reduz o passo apenas o suficiente para ainda
    # haver as tres amostras que a geometria exige.
    passo_amostra = min(0.20, andado / 2.0)
    distancias = []
    d = 0.0
    while d < andado - 1e-9:
        distancias.append(d)
        d += passo_amostra
    distancias.append(andado)

    ralos = []
    seg = 0
    inicio_seg = 0.0
    for alvo in distancias:
        while seg < len(trecho) - 2:
            ds = math.dist(trecho[seg], trecho[seg + 1])
            if inicio_seg + ds >= alvo - 1e-9:
                break
            inicio_seg += ds
            seg += 1
        a, b = trecho[seg], trecho[seg + 1]
        ds = math.dist(a, b)
        f = 0.0 if ds <= 1e-9 else max(0.0, min(1.0,
                                                (alvo - inicio_seg) / ds))
        ralos.append((a[0] + f * (b[0] - a[0]),
                      a[1] + f * (b[1] - a[1])))

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
