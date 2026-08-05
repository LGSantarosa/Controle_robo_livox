"""O que o LiDAR enxerga, e onde ele é cego — lógica pura, sem ROS.

Separada do nó pelo padrão da casa (`lei_de_rumo`, `lei_de_reta`): a conta
que responde "este obstáculo apareceria?" tem de ser testável sem subir
simulador nenhum, e tem de rodar igual na nuvem do Gazebo e na do Mid-360 de
verdade.

Existe porque a primeira medição do campo de visão saiu errada por causa da
régua, não do modelo (05-08). Ela pegava "os pontos de z mínimo" como se
fossem o chão; z mínimo era **artefato de rasância a 11 m** — pontos abaixo
do plano do chão, que não existem — e a zona cega saiu 2,27 m em vez de
2,04 m. É a mesma forma de erro da régua do planner (29-07) e do critério de
aceitação do 12b (04-08): a régua respondendo outra pergunta.
"""
import math


def zona_cega(altura, depressao_rad):
    """Raio [m] a partir do qual o CHÃO começa a aparecer.

    O Mid-360 quase não olha para baixo (−7°). Montado a `altura`, o feixe
    mais baixo só encontra o chão a `altura/tan(depressão)`. Dentro desse
    raio o chão é invisível — e com ele qualquer coisa deitada nele.
    """
    if altura <= 0.0:
        raise ValueError('altura tem de ser positiva')
    if depressao_rad <= 0.0:
        # Sensor que não olha para baixo nenhum nunca vê o chão.
        return float('inf')
    return altura / math.tan(depressao_rad)


def altura_minima_visivel(altura, depressao_rad, distancia):
    """Altura [m] que um objeto a `distancia` precisa ter para ser visto.

    Abaixo da linha do feixe mais baixo o objeto não devolve nada. Vale zero
    (ou menos) a partir da `zona_cega`, onde o feixe já encontrou o chão.

    É esta função, e não a zona cega, que responde a pergunta operacional:
    "o robô enxerga aquele degrau/caixa/pé de mesa a tantos metros?"
    """
    return max(0.0, altura - distancia * math.tan(depressao_rad))


def separa_chao(pontos, altura, tol=0.02):
    """Divide a nuvem em (chão, acima, abaixo) no referencial do SENSOR.

    `abaixo` NÃO é geometria: são retornos abaixo do plano do chão, que não
    podem existir. Em rasância a longa distância a precisão de profundidade
    degrada e alguns pontos "afundam". Contá-los à parte é o que impede que
    eles sequestrem qualquer estatística de mínimo — foi exatamente isso que
    estragou a primeira medida.
    """
    chao, acima, abaixo = [], [], []
    for p in pontos:
        z = p[2]
        if z < -altura - tol:
            abaixo.append(p)
        elif z <= -altura + tol:
            chao.append(p)
        else:
            acima.append(p)
    return chao, acima, abaixo


def raio_do_chao_visto(pontos, altura, tol=0.02):
    """Menor distância horizontal com retorno de CHÃO — a zona cega medida.

    Devolve None se não houver chão na nuvem (sala pequena demais, sensor
    apontado para cima, nuvem vazia). None é resposta, não falha: significa
    "esta nuvem não mede isso".
    """
    chao, _, _ = separa_chao(pontos, altura, tol)
    if not chao:
        return None
    return min(math.hypot(p[0], p[1]) for p in chao)


def perfil_de_cegueira(pontos, altura, faixas):
    """Para cada faixa de distância, a menor ALTURA DO CHÃO com retorno.

    É a leitura que vira decisão de projeto: se na faixa de 0,5–1,0 m o ponto
    mais baixo está a +0,15 m do chão, então obstáculo mais baixo que isso,
    ali, é invisível — e nenhuma sintonia de costmap resolve.
    """
    fora = []
    for lo, hi in faixas:
        sub = [p for p in pontos if lo <= math.hypot(p[0], p[1]) < hi]
        if not sub:
            fora.append((lo, hi, None, 0))
            continue
        fora.append((lo, hi, min(p[2] for p in sub) + altura, len(sub)))
    return fora
