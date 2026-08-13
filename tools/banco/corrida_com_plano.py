#!/usr/bin/env python3
"""Grava a corrida COM o plano: pose, plano vigente, erro de trajeto e canais.

O `corrida_nav.py` grava a corrente de velocidade mas não guarda o `/plan`, e
sem ele não dá para separar "o plano passa perto da parede" de "o robô sai do
plano". Este grava os dois e calcula, a cada amostra:

    erro_trajeto  distância do robô ao segmento mais próximo do plano
    folga         distância do robô à parede mais próxima (do mapa)
    rumo_plano    para onde o plano aponta ali
"""
import argparse
import csv
import math
import sys
import time
from collections import deque

import rclpy
import yaml
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def campo_de_distancia(mapa_yaml):
    m = yaml.safe_load(open(mapa_yaml))
    res = m['resolution']
    ox, oy = m['origin'][:2]
    import os
    pgm = os.path.join(os.path.dirname(mapa_yaml), m['image'])
    f = open(pgm, 'rb')
    assert f.readline().strip() == b'P5'
    l = f.readline()
    while l.startswith(b'#'):
        l = f.readline()
    w, h = map(int, l.split())
    f.readline()
    d = f.read(w * h)
    INF = 10 ** 9
    dist = [[INF] * w for _ in range(h)]
    q = deque()
    for r in range(h):
        for c in range(w):
            if d[r * w + c] < 100:
                dist[r][c] = 0
                q.append((r, c))
    while q:
        r, c = q.popleft()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == dc == 0:
                    continue
                rr, cc = r + dr, c + dc
                if 0 <= rr < h and 0 <= cc < w and dist[rr][cc] == INF:
                    dist[rr][cc] = dist[r][c] + 1
                    q.append((rr, cc))

    def folga(x, y):
        c = int((x - ox) / res)
        r = h - 1 - int((y - oy) / res)
        if not (0 <= c < w and 0 <= r < h):
            return float('nan')
        return dist[r][c] * res
    return folga


def dist_ao_plano(plano, x, y):
    """Distância ao segmento mais próximo, e o rumo desse segmento."""
    melhor, rumo = float('inf'), float('nan')
    for i in range(len(plano) - 1):
        ax, ay = plano[i]
        bx, by = plano[i + 1]
        vx, vy = bx - ax, by - ay
        n = vx * vx + vy * vy
        t = 0.0 if n == 0 else max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / n))
        px, py = ax + t * vx, ay + t * vy
        d = math.hypot(x - px, y - py)
        if d < melhor:
            melhor, rumo = d, math.atan2(vy, vx)
    return melhor, rumo


class Gravador(Node):
    def __init__(self, alvo, csv_saida, teto, folga_fn):
        super().__init__('gravador')
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.folga_fn = folga_fn
        self.plano = []
        self.n_planos = 0
        self.pose = None
        self.raw = (0.0, 0.0)
        self.saida = (0.0, 0.0)
        self.desencalhe = 0.0
        self.ativo = False
        self.linhas = []
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, q)
        self.create_subscription(Path, '/plan', self.cb_plano, q)
        self.create_subscription(TwistStamped, '/auto_vel_raw',
                                 lambda m: setattr(self, 'raw', (m.twist.linear.x, m.twist.angular.z)), q)
        self.create_subscription(TwistStamped, '/auto_vel',
                                 lambda m: setattr(self, 'saida', (m.twist.linear.x, m.twist.angular.z)), q)
        self.create_subscription(TwistStamped, '/unstuck_vel',
                                 lambda m: setattr(self, 'desencalhe', m.twist.linear.x), q)
        self.create_subscription(GoalStatusArray, 'navigate_to_pose/_action/status',
                                 self.cb_status, q)
        self.alvo = alvo
        self.csv_saida = csv_saida
        self.teto = teto
        self.t0 = None
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def cb_status(self, msg):
        self.ativo = any(s.status in (1, 2, 3) for s in msg.status_list)

    def cb_odom(self, msg):
        self.pose = msg

    def cb_plano(self, msg):
        self.plano = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.n_planos += 1

    def manda(self):
        g = PoseStamped()
        g.header.frame_id = 'map'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = self.alvo[0]
        g.pose.position.y = self.alvo[1]
        g.pose.orientation.w = 1.0
        for _ in range(5):
            self.pub_goal.publish(g)
            rclpy.spin_once(self, timeout_sec=0.1)
        self.t0 = time.time()

    def amostra(self):
        if self.pose is None or self.t0 is None:
            return
        p = self.pose.pose.pose
        x, y = p.position.x, p.position.y
        rumo = yaw_de(p.orientation)
        et, rp = dist_ao_plano(self.plano, x, y) if len(self.plano) > 1 else (float('nan'), float('nan'))
        self.linhas.append({
            't': round(time.time() - self.t0, 3), 'x': round(x, 4), 'y': round(y, 4),
            'yaw': round(rumo, 4), 'erro_trajeto': round(et, 4),
            'rumo_plano': round(rp, 4), 'folga': round(self.folga_fn(x, y), 3),
            'raw_v': round(self.raw[0], 3), 'raw_wz': round(self.raw[1], 3),
            'saida_v': round(self.saida[0], 3), 'saida_wz': round(self.saida[1], 3),
            'desencalhe': round(self.desencalhe, 3), 'planos': self.n_planos,
            'objetivo_ativo': int(self.ativo),
            'dist_alvo': round(math.hypot(self.alvo[0] - x, self.alvo[1] - y), 3),
        })

    def grava(self):
        with open(self.csv_saida, 'w', newline='') as f:
            wtr = csv.DictWriter(f, fieldnames=list(self.linhas[0].keys()))
            wtr.writeheader()
            wtr.writerows(self.linhas)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alvo', nargs=2, type=float, required=True)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--teto-s', type=float, default=90.0)
    ap.add_argument('--mapa', required=True)
    ap.add_argument('--so-escuta', action='store_true',
                    help='não manda objetivo; começa a gravar quando o 1º plano chegar')
    a = ap.parse_args()
    rclpy.init()
    n = Gravador(a.alvo, a.csv, a.teto_s, campo_de_distancia(a.mapa))
    # espera pose
    t0 = time.time()
    while n.pose is None and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.1)
    if a.so_escuta:
        t0 = time.time()
        while n.n_planos == 0 and time.time() - t0 < 40:
            rclpy.spin_once(n, timeout_sec=0.1)
        n.t0 = time.time()
    else:
        n.manda()
    while time.time() - n.t0 < a.teto_s:
        rclpy.spin_once(n, timeout_sec=0.02)
        n.amostra()
        if n.linhas and n.linhas[-1]['dist_alvo'] < 0.25:
            break
        time.sleep(0.03)
    chegou = bool(n.linhas) and n.linhas[-1]['dist_alvo'] < 0.25
    if not n.linhas:
        print('NENHUMA amostra — o objetivo não foi aceito ou não houve pose')
        rclpy.shutdown(); sys.exit(2)
    n.grava()
    L = n.linhas
    cam = sum(math.hypot(L[i + 1]['x'] - L[i]['x'], L[i + 1]['y'] - L[i]['y'])
              for i in range(len(L) - 1))
    reta = math.hypot(L[-1]['x'] - L[0]['x'], L[-1]['y'] - L[0]['y'])
    cortes = sum(1 for r in L if abs(r['raw_v']) > 1e-3 and abs(r['saida_v']) < 1e-6)
    res = sum(1 for r in L if r['desencalhe'] < -1e-6)
    et = sorted(r['erro_trajeto'] for r in L if r['erro_trajeto'] == r['erro_trajeto'])
    fo = sorted(r['folga'] for r in L if r['folga'] == r['folga'])
    print(f"{'CHEGOU' if chegou else 'NAO CHEGOU'}  em {L[-1]['t']:.1f} s, "
          f"a {L[-1]['dist_alvo']:.2f} m do alvo")
    print(f"  caminho {cam:.2f} m / reta {reta:.2f} m = {cam / max(reta, 1e-3):.2f}x | "
          f"{n.n_planos} planos")
    print(f"  reflexo cortou {cortes / len(L) * 100:.0f}% das amostras | ré ativa "
          f"{res / len(L) * 100:.0f}%")
    if et:
        print(f"  erro de trajeto  p50 {et[len(et)//2]:.3f}  p90 {et[int(len(et)*0.9)]:.3f}  max {et[-1]:.3f} m")
    if fo:
        print(f"  folga da parede  min {fo[0]:.2f}  p10 {fo[len(fo)//10]:.2f} m")
    print(f"  -> {a.csv}")
    rclpy.shutdown()
    sys.exit(0 if chegou else 1)


if __name__ == '__main__':
    main()
