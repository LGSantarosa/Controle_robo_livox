#!/usr/bin/env python3
"""Bancada de suavizadores — pede o MESMO plano aos três e mede.

Mesmo papel (e mesmo formato) da bancada de planners: comparar em corridas
separadas é comparar fotos de dias diferentes. Aqui o plano é pedido UMA vez e
entregue aos três suavizadores, então a única coisa que difere é o suavizador.

**Não move o robô.** Usa `compute_path_to_pose` e `smooth_path`, que são
serviços de planejamento — serve com a bateria parada e sem Gazebo andando.

As três réguas, e cada uma existe por um defeito medido:

1. **maior quina** depois de reamostrar a 0,20 m. É a régua que importa porque
   o seguidor tem mira de 0,37 m: quina grande dentro da mira é caminho que
   ele não consegue seguir. No plano cru da porta (12-08): 36,9° de máximo e
   23,0° a 0,46 m do vão;
2. **folga contra o MAPA** (não contra o sensor, que tem 2 m de zona cega e
   diria "livre" onde o robô raspa). Suavizar pode empurrar o caminho para
   dentro da ombreira, e num vão de 0,90 m para um corpo de 0,63 m isso é o
   oposto do que se quer. Suavizador que ganha em quina e perde em folga
   REPROVA;
3. **comprimento**, para não trocar quina por desvio caro.

Uso:
    python3 tools/banco/suavizador.py --de 2.0 5.0 --alvo 6.0 1.5
"""
import argparse
import math
import sys


def reamostra(pontos, passo=0.20):
    """Um ponto a cada `passo` metros. O plano vem com pontos a 5 cm, e três
    vizinhos assim descrevem o passo da grade, não a curva."""
    if not pontos:
        return []
    ralos = [pontos[0]]
    for p in pontos[1:]:
        if math.hypot(p[0] - ralos[-1][0], p[1] - ralos[-1][1]) >= passo:
            ralos.append(p)
    return ralos


def quinas(pontos, passo=0.20):
    """Viradas (graus) entre trechos consecutivos do caminho reamostrado."""
    ralos = reamostra(pontos, passo)
    saida = []
    for a, b, c in zip(ralos, ralos[1:], ralos[2:]):
        r1 = math.atan2(b[1] - a[1], b[0] - a[0])
        r2 = math.atan2(c[1] - b[1], c[0] - b[0])
        d = math.atan2(math.sin(r2 - r1), math.cos(r2 - r1))
        saida.append((b, abs(math.degrees(d))))
    return saida


def comprimento(pontos):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pontos, pontos[1:]))


def resume(nome, pontos, grade, raio_robo, medir_folga=None):
    """Uma linha por suavizador: quina, folga e comprimento."""
    if not pontos:
        return f'{nome:10s} SEM CAMINHO'
    qs = quinas(pontos)
    maior = max((d for _, d in qs), default=0.0)
    folgas = ([medir_folga(grade, x, y) for x, y in pontos]
              if (grade is not None and medir_folga) else [])
    pior = min(folgas) if folgas else float('nan')
    invadiu = sum(1 for f in folgas if f < raio_robo)
    return (f'{nome:10s} quina máx {maior:5.1f}°   '
            f'folga mín {pior:5.3f} m   '
            f'invasões {invadiu:3d}   '
            f'caminho {comprimento(pontos):5.2f} m')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--de', nargs=2, type=float, metavar=('X', 'Y'),
                    help='partida; sem isto usa a pose atual do robô')
    ap.add_argument('--alvo', nargs=2, type=float, required=True,
                    metavar=('X', 'Y'))
    ap.add_argument('--frame', default='map')
    ap.add_argument('--raio-robo', type=float, default=0.315,
                    help='meia-largura do corpo [m], para contar invasão')
    ap.add_argument('--csv', help='grava os caminhos, um arquivo por plugin')
    # ⚠️ SEM ISTO O SERVIDOR ABORTA NA HORA. A primeira versão desta bancada
    # não mandava `max_smoothing_duration`, ele chegou ZERO, e o smoother
    # respondeu "Smoothing time exceeded allowed duration of -0.00" — e a
    # bancada imprimiu os quatro caminhos IDÊNTICOS como se fosse medida.
    # Instrumento que devolve número sem ter medido é pior que instrumento
    # quebrado: este chegou a sugerir que suavizar não fazia diferença.
    ap.add_argument('--teto-s', type=float, default=1.0,
                    help='orçamento de tempo do suavizador [s]')
    args = ap.parse_args()

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav2_msgs.action import ComputePathToPose, SmoothPath
    from nav_msgs.msg import OccupancyGrid
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy)

    sys.path.insert(0, __file__.rsplit('/', 1)[0])
    try:
        from folga import Grid, folga
    except ImportError:
        Grid = folga = None

    rclpy.init()
    no = Node('bancada_suavizador')

    # ---- o mapa, para a régua de folga
    grade = [None]
    if Grid is not None:
        def cb_mapa(msg):
            if grade[0] is None:
                grade[0] = Grid(list(msg.data), msg.info.width, msg.info.height,
                                msg.info.resolution,
                                msg.info.origin.position.x,
                                msg.info.origin.position.y)
        no.create_subscription(
            OccupancyGrid, '/map', cb_mapa,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def pose(x, y):
        p = PoseStamped()
        p.header.frame_id = args.frame
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        p.pose.orientation.w = 1.0
        return p

    def espera(cliente, meta, nome):
        if not cliente.wait_for_server(timeout_sec=10.0):
            print(f'{nome}: servidor não respondeu em 10 s', file=sys.stderr)
            return None
        fut = cliente.send_goal_async(meta)
        rclpy.spin_until_future_complete(no, fut, timeout_sec=15.0)
        h = fut.result()
        if h is None or not h.accepted:
            print(f'{nome}: objetivo recusado', file=sys.stderr)
            return None
        fr = h.get_result_async()
        rclpy.spin_until_future_complete(no, fr, timeout_sec=20.0)
        return fr.result().result if fr.result() else None

    # ---- o plano, UMA vez
    plan = ActionClient(no, ComputePathToPose, 'compute_path_to_pose')
    meta = ComputePathToPose.Goal()
    meta.goal = pose(*args.alvo)
    meta.use_start = args.de is not None
    if args.de:
        meta.start = pose(*args.de)
    res = espera(plan, meta, 'planner')
    if res is None or not res.path.poses:
        print('sem plano — nada a suavizar', file=sys.stderr)
        return 1
    cru = res.path

    for _ in range(30):            # deixa o mapa chegar
        rclpy.spin_once(no, timeout_sec=0.1)
        if grade[0] is not None:
            break
    if grade[0] is None:
        print('⚠️  sem /map: a régua de FOLGA não roda, e ela é a que impede '
              'trocar quina por raspão. Só a quina será medida.',
              file=sys.stderr)

    caminhos = [('cru', [(q.pose.position.x, q.pose.position.y)
                         for q in cru.poses])]

    from builtin_interfaces.msg import Duration
    suave = ActionClient(no, SmoothPath, 'smooth_path')
    for plugin in ('suave', 'simples', 'savgol'):
        m = SmoothPath.Goal()
        m.path = cru
        m.smoother_id = plugin
        m.check_for_collisions = True
        m.max_smoothing_duration = Duration(
            sec=int(args.teto_s),
            nanosec=int((args.teto_s % 1.0) * 1e9))
        r = espera(suave, m, plugin)
        # Resultado que não COMPLETOU não vira linha na tabela. O servidor
        # devolve o caminho de entrada quando aborta, e imprimi-lo ao lado dos
        # outros é fabricar um empate que não aconteceu.
        if r is None or not getattr(r, 'was_completed', False):
            cod = getattr(r, 'error_code', '?') if r is not None else '?'
            print(f'{plugin:10s} NÃO COMPLETOU (error_code={cod}) — '
                  'não entra na comparação', file=sys.stderr)
            caminhos.append((plugin, []))
            continue
        caminhos.append((plugin,
                         [(q.pose.position.x, q.pose.position.y)
                          for q in r.path.poses]))

    print(f'\nplano de ({cru.poses[0].pose.position.x:.2f} · '
          f'{cru.poses[0].pose.position.y:.2f}) '
          f'até ({args.alvo[0]:.2f} · {args.alvo[1]:.2f})\n')
    for nome, pts in caminhos:
        print('  ' + resume(nome, pts, grade[0], args.raio_robo, folga))
    print('\n  quina máx: o seguidor mira a 0,37 m — quina grande dentro da '
          'mira é caminho que ele não segue.')
    print('  invasões: pontos do caminho a menos de um raio de corpo do '
          'obstáculo, contra o MAPA.')

    if args.csv:
        import csv
        for nome, pts in caminhos:
            alvo = args.csv.replace('.csv', f'-{nome}.csv')
            with open(alvo, 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['x', 'y'])
                w.writerows(pts)
            print(f'  -> {alvo}')

    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
