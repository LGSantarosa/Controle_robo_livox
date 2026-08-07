#!/usr/bin/env python3
"""Pede um caminho ao Nav2 e mede se ele passa por cima de um obstáculo.

    python3 tools/banco/plano.py --alvo 2.0 7.2 --obstaculo 2.0 6.5 0.25

Chama a ação `compute_path_to_pose` — **planeja sem mover o robô**, que é o que
permite julgar o planejador com a bateria parada (ou, aqui, sem robô nenhum).

A pergunta que ele responde: o plano contorna o obstáculo ou atravessa? Isso
separa "o costmap LOCAL viu" de "o planejador SOUBE". As duas coisas são
diferentes e é fácil confundi-las: o local costmap alimenta quem dirige, o
global alimenta quem planeja. Obstáculo que só entra no local produz um robô
que vai reto até o obstáculo e para lá — o reflexo salva, mas ninguém replaneja.
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node


def pose(x, y, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--alvo', nargs=2, type=float, required=True,
                    metavar=('X', 'Y'))
    ap.add_argument('--de', nargs=2, type=float, metavar=('X', 'Y'),
                    help='partida; omitido usa a pose atual do robô')
    ap.add_argument('--obstaculo', nargs=3, type=float,
                    metavar=('X', 'Y', 'RAIO'),
                    help='mede a folga do plano até este ponto')
    ap.add_argument('--csv', help='grava o caminho ponto a ponto')
    args = ap.parse_args()

    rclpy.init()
    no = Node('mede_plano')
    cli = ActionClient(no, ComputePathToPose, 'compute_path_to_pose')
    if not cli.wait_for_server(timeout_sec=10.0):
        print('o planner_server não respondeu — ele está ativo?')
        return 1

    meta = ComputePathToPose.Goal()
    meta.goal = pose(*args.alvo)
    meta.use_start = args.de is not None
    if args.de:
        meta.start = pose(*args.de)

    fut = cli.send_goal_async(meta)
    rclpy.spin_until_future_complete(no, fut, timeout_sec=15.0)
    handle = fut.result()
    if handle is None or not handle.accepted:
        print('o planejador recusou o pedido')
        return 1
    res = handle.get_result_async()
    rclpy.spin_until_future_complete(no, res, timeout_sec=30.0)
    if not res.done():
        print('o planejador não devolveu resultado')
        return 1
    caminho = res.result().result.path.poses
    if not caminho:
        print('plano VAZIO — o planejador não achou caminho')
        return 1

    pts = [(p.pose.position.x, p.pose.position.y) for p in caminho]
    comp = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
    reta = math.dist(pts[0], pts[-1])
    print(f'plano: {len(pts)} pontos, {comp:.2f} m de caminho para '
          f'{reta:.2f} m de reta ({comp / reta:.2f}x)')
    print(f'  de ({pts[0][0]:.2f}, {pts[0][1]:.2f}) '
          f'até ({pts[-1][0]:.2f}, {pts[-1][1]:.2f})')

    if args.obstaculo:
        ox, oy, raio = args.obstaculo
        folga = min(math.dist(p, (ox, oy)) for p in pts)
        print(f'  folga mínima até ({ox:.2f}, {oy:.2f}): {folga:.3f} m '
              f'(o obstáculo tem raio {raio:.2f})')
        if folga < raio:
            print('  ➡️  O PLANO ATRAVESSA o obstáculo: quem planeja não sabe '
                  'que ele existe.')
        else:
            print('  ➡️  o plano CONTORNA: a informação chegou ao planejador.')

    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('i,x,y\n')
            for i, (x, y) in enumerate(pts):
                f.write(f'{i},{x:.4f},{y:.4f}\n')
        print(f'  -> {args.csv}')

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
