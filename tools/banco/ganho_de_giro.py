#!/usr/bin/env python3
"""Ganho de giro ponta a ponta — quanto do `wz` pedido o robô entrega.

    # robô num ponto com ~1,5 m livres em volta, pilha DE PÉ:
    python3 tools/banco/ganho_de_giro.py --sim \
        --csv docs/dados/AAAA-MM-DD-.../ganho.csv

## Por que isto existe

Em 13-08, na corrida da porta, a curva SUAVE não acontecia: pedido de +0,15
rad/s virava −0,03 real, e o robô chegava torto na porta e precisava de ré.

A primeira suspeita — "o compensador desliga na curva" — estava errada: a
`lei_de_reta` soma o feedforward também na curva pedida (o arco é do corpo, não
do comando). Refazendo a conta COM o `ff` somado, sobra uma explicação só:

    real = g·(pedido + ff) + arco,  com ff = −curv·v  e  arco = curv·v
         = g·pedido − (1 − g)·|curv|·v

Ou seja, se o atuador entrega **g < 1** do `wz` comandado, o cancelamento do
arco chega atenuado e sobra `(1 − g)·|curv|·v` de arco permanente. Com os
números de 13-08 isso dava g ≈ 0,4–0,55 e um resíduo de ≈ −0,12 rad/s, que é do
tamanho exato da curva suave que sumia.

Isto aqui MEDE o g em vez de supô-lo. A regressão devolve os dois:

    inclinação  = g                      (quanto do pedido chega)
    intercepto  = −(1 − g)·|curv|·v      (o arco que sobra sem cancelar)

e os dois têm de contar a mesma história. Se contarem histórias diferentes, o
modelo está errado e o número não vale — é o critério de aceitação do ensaio.

## O que cada corrida faz

    1. para e assenta
    2. comanda (v, wz) por `t_cmd` segundos
    3. mede a taxa de giro REAL na janela de regime (descontando a latência
       de 0,27 s da placa, medida em 31-07)
    4. desfaz o arco andando de ré com o mesmo `wz`, para a próxima caber

⚠️ Publica em `/key_vel` (prioridade 90 no mux): passa PELO compensador, que é
o que queremos — o ganho que interessa é o da cadeia inteira, do que se pede ao
que o robô faz. Um ganho medido no atuador cru não responderia a pergunta.

⚠️ `--sim` é obrigatório no Gazebo (relógio de simulação). Sem ele o comando é
descartado por velho e o robô não se mexe, sem erro na tela.
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bancada(Node):
    def __init__(self, topico, sim):
        super().__init__('ganho_de_giro',
                         parameter_overrides=[Parameter(
                             'use_sim_time', Parameter.Type.BOOL, bool(sim))])
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, topico, q)
        self.create_subscription(Odometry, '/Odometry', self.cb, q)
        self.pos = None
        self.yaw = None
        self.acumulado = 0.0      # yaw acumulado: 200° não somem no wrap
        self.percurso = 0.0
        self._ultimo_yaw = None
        self._ultimo_pos = None

    def cb(self, msg):
        p = msg.pose.pose.position
        y = yaw_de(msg.pose.pose.orientation)
        if self._ultimo_yaw is not None:
            self.acumulado += wrap(y - self._ultimo_yaw)
        if self._ultimo_pos is not None:
            dx, dy = p.x - self._ultimo_pos[0], p.y - self._ultimo_pos[1]
            self.percurso += dx * math.cos(y) + dy * math.sin(y)
        self._ultimo_yaw = y
        self._ultimo_pos = (p.x, p.y)
        self.pos = (p.x, p.y)
        self.yaw = y

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def manda(self, v, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(wz)
        self.pub.publish(m)

    def mantem(self, v, wz, dur):
        t0 = self.agora()
        while self.agora() - t0 < dur:
            self.manda(v, wz)
            rclpy.spin_once(self, timeout_sec=0.02)

    def espera(self, dur):
        t0 = self.agora()
        while self.agora() - t0 < dur:
            self.manda(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def corrida(self, v, wz, t_cmd, latencia):
        """Devolve (wz real de regime, percurso, yaw varrido na janela)."""
        self.espera(2.0)
        # fase 1: latência da placa — o comando ainda não virou movimento
        self.mantem(v, wz, latencia)
        # fase 2: REGIME, e é só isto que entra na conta
        t0, y0 = self.agora(), self.acumulado
        p0 = self.percurso
        self.mantem(v, wz, t_cmd)
        t1, y1 = self.agora(), self.acumulado
        p1 = self.percurso
        # fase 3: solta e deixa a inércia acabar FORA da janela de medida
        self.espera(2.5)
        dt = t1 - t0
        return ((y1 - y0) / dt if dt > 1e-6 else float('nan'),
                (p1 - p0) / dt if dt > 1e-6 else float('nan'),
                y1 - y0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--sim', action='store_true')
    ap.add_argument('--topico', default='/key_vel')
    ap.add_argument('--v', type=float, default=0.5,
                    help='v comandado (a placa entrega ~0,298 de qualquer jeito)')
    ap.add_argument('--wz', nargs='*', type=float,
                    default=[0.0, 0.15, -0.15, 0.30, -0.30, 0.50, -0.50],
                    help='os wz a varrer; o 0,0 mede o arco natural sozinho')
    ap.add_argument('--t-cmd', type=float, default=1.5,
                    help='[s] janela de regime')
    ap.add_argument('--latencia', type=float, default=0.30,
                    help='[s] latência da placa, descartada antes da janela')
    a = ap.parse_args()

    rclpy.init()
    n = Bancada(a.topico, a.sim)
    t0 = time.time()
    while n.pos is None and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pos is None:
        print('sem /Odometry — a pilha está de pé?')
        rclpy.shutdown()
        sys.exit(2)

    linhas = []
    print(f'{"wz pedido":>10} {"wz real":>9} {"v real":>8} {"varreu":>8}')
    for wz in a.wz:
        real, v_real, varrido = n.corrida(a.v, wz, a.t_cmd, a.latencia)
        print(f'{wz:10.2f} {real:9.3f} {v_real:8.3f} '
              f'{math.degrees(varrido):7.1f}°')
        linhas.append({'wz_pedido': wz, 'wz_real': round(real, 4),
                       'v_real': round(v_real, 4),
                       'varrido_deg': round(math.degrees(varrido), 2),
                       'x': round(n.pos[0], 3), 'y': round(n.pos[1], 3)})
        # desfaz o arco: mesma curva, de ré
        n.mantem(-a.v, wz, a.t_cmd + a.latencia)
        n.espera(2.0)

    with open(a.csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
        w.writeheader()
        w.writerows(linhas)
    print(f'\n-> {a.csv}')

    # regressão simples: wz_real = g·wz_pedido + b
    xs = [r['wz_pedido'] for r in linhas]
    ys = [r['wz_real'] for r in linhas]
    nn = len(xs)
    mx, my = sum(xs) / nn, sum(ys) / nn
    den = sum((x - mx) ** 2 for x in xs)
    g = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / den if den else float('nan')
    b = my - g * mx
    v_med = sum(r['v_real'] for r in linhas) / nn
    print(f'\nGANHO g = {g:.3f}   (1,0 seria o robô entregando o que se pede)')
    print(f'resíduo de arco b = {b:+.3f} rad/s a v_real {v_med:.3f} m/s')
    # o teste do modelo: b tem de valer −(1−g)·|curv|·v
    for curv in (0.817,):
        previsto = -(1.0 - g) * curv * v_med
        print(f'previsto pelo modelo com curv {curv:.3f}: {previsto:+.3f} rad/s'
              f'   -> {"BATE" if abs(previsto - b) < 0.06 else "NÃO BATE"}')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
