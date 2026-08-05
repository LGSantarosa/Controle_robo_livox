"""Quanto o robô passou perto da parede — lógica pura, sem ROS.

A régua de "ele bateu?". Existe porque bater é o defeito que o dono vê e que o
número não mostrava: até 05-08 a única evidência de colisão era o olho dele no
RViz, e olho não compara duas sintonias.

Mede contra o MAPA, não contra o sensor: o mapa é a verdade da pista (mundo e
mapa saem da mesma planta, `tools/mundo/gera_pista.py`), enquanto o sensor tem
zona cega de 2 m e diria "livre" justamente onde o robô raspa.

    folga = distância do centro do robô até a célula ocupada mais próxima

    folga < raio_robo   ->  o corpo INVADIU o obstáculo = bateu
    folga < raio + margem -> passou raspando
"""
import math


class Grid:
    """Recorte do mapa do Nav2, o mínimo para consultar ocupação."""

    def __init__(self, dados, largura, altura, resolucao, ox, oy, limiar=65):
        self.dados = dados          # 0..100, linha a linha, origem embaixo
        self.largura = largura
        self.altura = altura
        self.resolucao = resolucao
        self.ox = ox
        self.oy = oy
        self.limiar = limiar

    def ocupada(self, col, lin):
        if not (0 <= col < self.largura and 0 <= lin < self.altura):
            # Fora do mapa é DESCONHECIDO, e desconhecido não é obstáculo.
            # Tratar como ocupado faria toda corrida perto da borda parecer
            # colisão; tratar como livre é o lado seguro do erro para ESTA
            # medida (ela mede o que o mapa conhece, e diz isso).
            return False
        return self.dados[lin * self.largura + col] >= self.limiar


def carrega_pgm(caminho_pgm, resolucao, ox, oy, limiar_ocupado=0.65):
    """Lê um PGM binário (P5) do `map_server` e devolve um `Grid`.

    No PGM do Nav2 o valor é OCUPAÇÃO INVERTIDA: 0 = preto = ocupado,
    255 = branco = livre. A conversão para 0..100 é a mesma do `map_server`
    com `mode: trinary`.
    """
    with open(caminho_pgm, 'rb') as f:
        bruto = f.read()
    # Cabeçalho: P5, largura altura, maxval — comentários começam com '#'.
    campos, i = [], 0
    while len(campos) < 4:
        while i < len(bruto) and bruto[i:i + 1].isspace():
            i += 1
        if bruto[i:i + 1] == b'#':
            while i < len(bruto) and bruto[i:i + 1] != b'\n':
                i += 1
            continue
        j = i
        while j < len(bruto) and not bruto[j:j + 1].isspace():
            j += 1
        campos.append(bruto[i:j])
        i = j
    if campos[0] != b'P5':
        raise ValueError('só PGM binário (P5)')
    largura, altura = int(campos[1]), int(campos[2])
    i += 1                                   # o único byte branco após maxval
    pixels = bruto[i:i + largura * altura]

    # PGM vem de cima para baixo; o OccupancyGrid conta de baixo para cima.
    dados = [0] * (largura * altura)
    for lin in range(altura):
        base = (altura - 1 - lin) * largura
        for col in range(largura):
            v = pixels[base + col]
            dados[lin * largura + col] = 0 if v > 255 * limiar_ocupado else 100
    return Grid(dados, largura, altura, resolucao, ox, oy)


def folga(grid, x, y, alcance=1.5):
    """Distância [m] de (x,y) até a célula ocupada mais próxima.

    Busca em anéis crescentes e para no primeiro achado — o custo é o da folga
    real, não o do `alcance`. Devolve `alcance` quando não há nada perto: a
    medida é "pelo menos isto", e o chamador precisa saber que é um piso e não
    uma distância exata.
    """
    res = grid.resolucao
    c0 = int(math.floor((x - grid.ox) / res))
    l0 = int(math.floor((y - grid.oy) / res))
    passos = int(alcance / res) + 1
    melhor = None
    for r in range(passos + 1):
        # Só o PERÍMETRO do quadrado de raio r: o interior já foi visto nos
        # anéis anteriores.
        for dc in range(-r, r + 1):
            for dl in ((-r, r) if abs(dc) != r else range(-r, r + 1)):
                c, l = c0 + dc, l0 + dl
                if not grid.ocupada(c, l):
                    continue
                # Distância do PONTO ao CENTRO da célula, não diferença de
                # índices: truncar o ponto para índice antes de medir errava
                # meia célula por eixo, e num grid de 5 cm isso é 3,5 cm — a
                # mesma ordem da folga que decide se bateu.
                cx = grid.ox + (c + 0.5) * res
                cy = grid.oy + (l + 0.5) * res
                d = math.hypot(x - cx, y - cy)
                if melhor is None or d < melhor:
                    melhor = d
        # Achou no anel r, mas o anel r+1 ainda pode ter célula mais PERTO em
        # distância real (o anel é quadrado, a distância é redonda). Um anel
        # a mais fecha isso.
        if melhor is not None and melhor <= r * res:
            break
    return min(melhor, alcance) if melhor is not None else alcance


def perfil_de_folga(grid, trajetoria, raio_robo, margem=0.05, alcance=1.5):
    """Percorre a trajetória e resume o quanto ela chegou perto.

    Devolve (folga_minima, n_invasoes, n_raspoes, pior_ponto). "Invasão" é
    folga abaixo do raio do robô — o corpo ocupou a mesma célula que o
    obstáculo, ou seja BATEU. "Raspão" é passar dentro da margem.
    """
    pior, ponto, invasoes, raspoes = alcance, None, 0, 0
    for x, y in trajetoria:
        f = folga(grid, x, y, alcance)
        if f < raio_robo:
            invasoes += 1
        elif f < raio_robo + margem:
            raspoes += 1
        if f < pior:
            pior, ponto = f, (x, y)
    return pior, invasoes, raspoes, ponto
