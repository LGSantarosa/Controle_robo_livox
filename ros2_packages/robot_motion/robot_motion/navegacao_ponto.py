"""Ir a um ponto — funções puras, sem ROS.

A navegação ponto a ponto do robô 2. Ela NÃO comanda roda: decide para onde
apontar e a que velocidade ir, e entrega isso à camada de movimentação
(`lei_de_rumo.py`), que sabe o que o atuador aguenta.

Duas ideias sustentam o arquivo, e as duas vêm de coisa medida:

1. **O rumo alvo é recalculado todo ciclo** como a direção até o ponto. Isso
   dissolve o erro lateral sem controlador extra: se o robô sai da linha, a
   direção até o alvo muda e ele curva de volta sozinho. Medido: o controlador
   de rumo sozinho trava o rumo mas segue paralelo à rota, 66 cm deslocado
   depois de uma meia-volta.

2. **A aproximação é a mesma lei da frenagem**, aplicada a distância:
   `v = √(2·a_lin·dist)` — nunca chegar mais rápido do que se consegue frear.
   Com uma ressalva que a zona morta impõe: não dá para desacelerar
   suavemente até zero, porque abaixo do mínimo viável a placa ignora o
   comando e o robô para ANTES de chegar. A aproximação tem piso, e o corte
   na chegada é firme.
"""
import math


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def distancia(x, y, alvo_x, alvo_y):
    return math.hypot(alvo_x - x, alvo_y - y)


def rumo_para(x, y, alvo_x, alvo_y):
    """Direção do robô até o ponto, no referencial do /Odometry [rad]."""
    return math.atan2(alvo_y - y, alvo_x - x)


def velocidade_de_aproximacao(dist, v_max, a_lin, v_min_viavel):
    """Velocidade de avanço rumo ao ponto.

    `√(2·a_lin·dist)` é a mesma conta da lei de rumo, com distância no lugar
    de erro angular: é a maior velocidade da qual ainda se consegue parar
    dentro do que falta.

    O piso `v_min_viavel` é o que a zona morta obriga. Sem ele a lei manda
    velocidades cada vez menores na chegada, a placa engole todas, e o robô
    para longe do ponto **sem erro nenhum** — a falha silenciosa do BO-3
    outra vez, agora disfarçada de "chegou".
    """
    if a_lin <= 0.0:
        raise ValueError('a_lin tem que ser positiva — é desaceleração física')
    return max(min(v_max, math.sqrt(2.0 * a_lin * max(0.0, dist))), v_min_viavel)


def velocidade_que_a_curva_permite(dist, erro_rumo, wz_util):
    """Teto de velocidade para o alvo continuar ALCANÇÁVEL.

    Perseguir um ponto a `dist` com erro de rumo `e` exige girar a
    `v·sen(e)/dist` só para manter o bico apontado nele — e quanto mais perto,
    mais rápido é preciso girar. Se o robô não entrega esse giro, o ponto
    escapa pelo lado e ele passa a **orbitá-lo**, a uma distância estável igual
    ao próprio raio de curva.

    Medido: com o alvo a 0,65 m o robô orbitou a 0,365 m indefinidamente,
    com raio de curva de 0,27 m. Nenhum piso de velocidade estava ativo — o
    problema não é zona morta, é ir rápido demais para a curva necessária.

    Invertendo a relação: `v <= wz_util·dist/sen(e)`.

    `wz_util` não é o teto de giro: é o giro que a máquina **sustenta de fato**
    numa curva, que sai do ensaio `curva` do banco. Usar o teto aqui seria
    otimismo, e otimismo neste ponto vira órbita.
    """
    seno = abs(math.sin(erro_rumo))
    if seno < 1e-6:
        return float('inf')          # apontado para o alvo: a curva não limita
    return wz_util * dist / seno


def raio_minimo_de_chegada(v_min_viavel, a_lin, folga=0.02):
    """Menor raio de chegada coerente com o piso de velocidade.

    Se o robô só consegue andar a `v_min_viavel`, ele ainda percorre
    `v_min²/(2·a_lin)` depois do corte. Aceitar um raio menor que isso é
    pedir uma precisão que o atuador não entrega — e o robô ficaria orbitando
    o ponto, entrando e saindo do raio para sempre.
    """
    return v_min_viavel ** 2 / (2.0 * a_lin) + folga


def chegou(dist, raio_chegada):
    return dist <= raio_chegada


def comando_de_navegacao(x, y, yaw, alvo_x, alvo_y, v_max, a_lin,
                         v_min_viavel, raio_chegada, wz_util):
    """(chegou, rumo_alvo, velocidade) para a camada de movimentação.

    A velocidade é o menor de dois tetos, cada um cuidando de um jeito de não
    chegar: o da **frenagem** (não passar do ponto) e o da **curva** (não
    orbitar o ponto). O piso do mínimo viável vem por último, porque abaixo
    dele a placa engole o comando.

    Na chegada devolve zero — corte firme, não desaceleração assintótica.
    """
    d = distancia(x, y, alvo_x, alvo_y)
    rumo = rumo_para(x, y, alvo_x, alvo_y)
    if chegou(d, raio_chegada):
        return True, rumo, 0.0

    erro = norm_ang(rumo - yaw)
    v = min(velocidade_de_aproximacao(d, v_max, a_lin, 0.0),
            velocidade_que_a_curva_permite(d, erro, wz_util))
    return False, rumo, max(v, v_min_viavel)
