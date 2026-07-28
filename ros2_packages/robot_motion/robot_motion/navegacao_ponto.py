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


def raio_necessario(dist, erro_rumo):
    """Raio da curva que leva ao alvo sem largar o alvo: `d/(2·sen e)`.

    Robô e alvo estão os dois sobre a mesma circunferência, separados pela
    corda `d`, com o bico `e` fora dela. Alinhado (`sen e = 0`) o raio é
    infinito: segue reto.
    """
    seno = abs(math.sin(erro_rumo))
    if seno < 1e-6:
        return float('inf')
    return dist / (2.0 * seno)


def precisa_recuar(dist, erro_rumo, raio_min_curva, recuando, folga=1.3):
    """O alvo está dentro do círculo que o robô não consegue fechar?

    Sem pivô o robô tem um raio mínimo de curva, e um ponto que exige menos
    que isso fica **dentro** do círculo que ele descreve. Perseguir um ponto
    por dentro do próprio círculo não converge: o robô o orbita para sempre —
    medido, 0,168 m de um alvo com raio de chegada de 0,15 m.

    A saída é geométrica e é a manobra de baliza: recuando, `d` cresce, o raio
    necessário abre, e o ponto volta a caber. Por isso a decisão sai daqui e
    não de um detector de órbita: dá para saber ANTES, pela geometria, em vez
    de descobrir depois de orbitar.

    `raio_min_curva = 0` é o robô que pivota — para ele nenhum ponto está
    dentro de círculo nenhum, e a ré nunca acontece. Quando a bitola medida
    liberar o pivô, este comportamento some sozinho, pelo parâmetro.

    A `folga` é histerese, e ela não é enfeite: no ponto exato em que o alvo
    passa a caber, sem folga o robô alterna ré e avanço a cada ciclo e não sai
    do lugar.
    """
    if raio_min_curva <= 0.0:
        return False
    limite = raio_min_curva * (folga if recuando else 1.0)
    return raio_necessario(dist, erro_rumo) < limite


def orcamento_de_re_estourado(recuou, t_recuando, re_max_dist, re_max_s):
    """A ré tem fim — em metros e em segundos.

    Uma manobra que não converge tem que parar e ser dita em voz alta. Recuar
    indefinidamente é o pior dos mundos: o robô sai do lugar onde alguém o
    procura, e o log continua limpo.
    """
    return recuou >= re_max_dist or t_recuando >= re_max_s


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
