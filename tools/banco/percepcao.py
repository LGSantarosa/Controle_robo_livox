#!/usr/bin/env python3
"""O costmap está marcando o que o SENSOR vê, ou repetindo o que o MAPA diz?

    python3 tools/banco/percepcao.py                 # local_costmap
    python3 tools/banco/percepcao.py --qual global_costmap
    python3 tools/banco/percepcao.py --csv saida.csv

Um robô que desvia de obstáculo tem duas explicações, e elas se parecem de
fora: ele viu com o lidar, ou o mapa estático já dizia que tinha algo ali.
Enquanto mundo e mapa forem gerados da MESMA planta (`tools/mundo/gera_pista.py`
sem `--surpresa`), as duas são indistinguíveis, e a segunda não é percepção
nenhuma.

Esta ferramenta separa as duas: pega o costmap (pelo SERVIÇO `get_costmap`, não
pelo tópico — o Nav2 só publica costmap quando alguém está assinando, e um
medidor que aparece e some perde a janela) e o mapa estático, e reporta as
células **letais no costmap onde o mapa diz LIVRE**. Essas só podem ter vindo
do sensor.

Agrupa em manchas conexas e imprime o centróide de cada uma, para conferir
contra a posição conhecida do obstáculo que se pôs no mundo de propósito.

⚠️ Lida com o robô PARADO. Com o robô andando o costmap local rola e as manchas
se movem; para julgar desvio, o que vale é o plano, não este número.
"""
import argparse
import sys

import rclpy
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

# ⚠️ 254 e NÃO 253. No costmap cru do Nav2, 254 é LETHAL_OBSTACLE (alguém
# marcou aquela célula) e **253 é INSCRIBED_INFLATED_OBSTACLE — inflação**, que
# a `InflationLayer` pinta ao redor de todo obstáculo até o raio inscrito do
# robô. Contar 253 como obstáculo faz cada parede aparecer 0,32 m mais gorda
# (o `robot_radius`), o que se lê como "o sensor está marcando coisa que não
# existe". Custou meia hora de investigação na primeira corrida desta
# ferramenta, com a nuvem crua conferida ponto a ponto — que estava certa.
LETAL = 254          # no costmap cru; o mapa vem em 0..100
MAPA_LIVRE = 50      # abaixo disto o mapa considera livre


class Percepcao(Node):
    def __init__(self, qual):
        super().__init__('percepcao')
        self.qual = qual
        self.mapa = None
        latched = QoSProfile(
            depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.pega_mapa, latched)

    def pega_mapa(self, msg):
        self.mapa = msg

    def espera_mapa(self, segundos=10.0):
        fim = self.get_clock().now().nanoseconds + segundos * 1e9
        while self.mapa is None and self.get_clock().now().nanoseconds < fim:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self.mapa is not None

    def pega_costmap(self, segundos=10.0):
        cli = self.create_client(GetCostmap, f'/{self.qual}/get_costmap')
        if not cli.wait_for_service(timeout_sec=segundos):
            return None
        fut = cli.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=segundos)
        return fut.result().map if fut.done() and fut.result() else None


def livre_no_mapa(mapa, x, y, orla=0.0):
    """O mapa estático diz que este ponto do mundo está livre?

    `orla` (em metros) exige que TODA a vizinhança também esteja livre. Sem
    isso as faces das paredes conhecidas entram no relatório: o sensor marca a
    superfície com 2 cm de ruído sobre grade de 5 cm, e a célula marcada cai
    logo FORA da parede rasterizada. Medido na primeira corrida: parede real em
    x = 0,20, mancha em x = 0,23. Uma orla de 0,10 m (duas células) descarta
    isso sem esconder obstáculo de verdade, que tem de estar longe de tudo o
    que o mapa já conhece para valer como prova de percepção.

    Fora do mapa conta como livre: o que está além da planta não pode ter sido
    lembrado dela.
    """
    res = mapa.info.resolution
    passo = max(1, int(round(orla / res)))
    c0 = int((x - mapa.info.origin.position.x) / res)
    r0 = int((y - mapa.info.origin.position.y) / res)
    for dc in range(-passo, passo + 1):
        for dr in range(-passo, passo + 1):
            c, r = c0 + dc, r0 + dr
            if not (0 <= c < mapa.info.width and 0 <= r < mapa.info.height):
                continue
            v = mapa.data[r * mapa.info.width + c]
            if v >= MAPA_LIVRE or v < 0:
                return False
    return True


def manchas(celulas):
    """Componentes conexas (8-vizinhos) de um conjunto de células (col, lin)."""
    restantes = set(celulas)
    saida = []
    while restantes:
        fila = [restantes.pop()]
        grupo = []
        while fila:
            c, r = fila.pop()
            grupo.append((c, r))
            for dc in (-1, 0, 1):
                for dr in (-1, 0, 1):
                    viz = (c + dc, r + dr)
                    if viz in restantes:
                        restantes.discard(viz)
                        fila.append(viz)
        saida.append(grupo)
    return sorted(saida, key=len, reverse=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--qual', default='local_costmap',
                   choices=['local_costmap', 'global_costmap'])
    p.add_argument('--min-celulas', type=int, default=4,
                   help='mancha menor que isto é ruído e não entra no relatório')
    p.add_argument('--orla', type=float, default=0.10,
                   help='descarta marcação a menos disto (m) de obstáculo que '
                        'o mapa já conhece — é a face das paredes')
    p.add_argument('--csv', help='grava as manchas em CSV')
    args = p.parse_args()

    rclpy.init()
    no = Percepcao(args.qual)
    if not no.espera_mapa():
        print('o /map não chegou — o map_server está de pé?')
        return 1
    grade = no.pega_costmap()
    if grade is None:
        print(f'o serviço /{args.qual}/get_costmap não respondeu')
        return 1

    res = grade.metadata.resolution
    ox = grade.metadata.origin.position.x
    oy = grade.metadata.origin.position.y
    w, h = grade.metadata.size_x, grade.metadata.size_y

    letais, do_sensor = 0, []
    for i, v in enumerate(grade.data):
        if v < LETAL:
            continue
        letais += 1
        c, r = i % w, i // w
        x, y = ox + (c + 0.5) * res, oy + (r + 0.5) * res
        if livre_no_mapa(no.mapa, x, y, args.orla):
            do_sensor.append((c, r))

    print(f'{args.qual}: {w}x{h} @ {res:.3f} m/célula, '
          f'origem ({ox:.2f}, {oy:.2f})')
    print(f'  células letais           : {letais}')
    print(f'  onde o mapa diz LIVRE    : {len(do_sensor)}  '
          f'({len(do_sensor) * res * res:.3f} m²)  <- só o sensor explica '
          f'(orla de {args.orla:.2f} m descontada)')

    grupos = [g for g in manchas(do_sensor) if len(g) >= args.min_celulas]
    if not grupos:
        print('  nenhuma mancha acima do limiar — o costmap está repetindo o mapa')
    linhas = []
    for n, g in enumerate(grupos, 1):
        cx = ox + (sum(c for c, _ in g) / len(g) + 0.5) * res
        cy = oy + (sum(r for _, r in g) / len(g) + 0.5) * res
        area = len(g) * res * res
        print(f'  mancha {n}: centro ({cx:.2f}, {cy:.2f}) m, '
              f'{len(g)} células, {area:.3f} m²')
        linhas.append((n, cx, cy, len(g), area))

    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('mancha,centro_x,centro_y,celulas,area_m2\n')
            for n, cx, cy, k, area in linhas:
                f.write(f'{n},{cx:.4f},{cy:.4f},{k},{area:.4f}\n')
        print(f'  -> {args.csv}')

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
