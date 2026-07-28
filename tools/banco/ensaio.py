#!/usr/bin/env python3
"""Banco de ensaios de movimentação — mede os limites do robô e grava CSV.

Roda IGUAL no robô real e no simulador: os dois falam
`/hoverboard_base_controller/cmd_vel` (TwistStamped, SI) e publicam pose em
`/Odometry` (FAST-LIO no robô, pose verdadeira do Gazebo no simulador). Por
isso os números dos dois são diretamente comparáveis — é o que permite dizer
o quanto o simulador mente, com medida em vez de opinião.

    ros2 run ... não: é script solto, roda direto.
    python3 ensaio.py --ensaio zona_morta_linear --csv zm_lin.csv

VELOCIDADE MEDIDA VEM DA POSE, não do campo `twist`. O twist do publicador de
odometria do Gazebo mostrou-se ruidoso (marcava 0,077 rad/s com o rumo
parado); a pose é limpa nos dois lados. O twist é gravado assim mesmo, em
coluna separada, para quem quiser comparar.

SEGURANÇA (o robô é real e pesa 10 kg):
  --espaco   distância máxima da origem, em metros. Estourou, para tudo.
  --dur      teto de tempo. Estourou, para tudo.
  Ctrl-C, exceção ou fim do ensaio: publica zero antes de sair, sempre.
"""
import argparse
import csv
import math
import sys
from collections import deque

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Janela da diferenciação da pose. Curta demais amplifica ruído, longa demais
# atrasa a medida e estraga justamente o que queremos medir (a rampa de giro).
JANELA_S = 0.2


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


class Ensaio(Node):
    def __init__(self, cfg):
        super().__init__('ensaio')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, cfg.sim)])
        self.cfg = cfg

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            TwistStamped, '/hoverboard_base_controller/cmd_vel', qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_pose, qos)
        self.create_subscription(
            Odometry, '/hoverboard_base_controller/odom', self.cb_roda, qos)

        self.pose = None
        self.roda = None
        self.hist = deque()          # (t, x, y, yaw) para diferenciar
        self.t0 = None
        self.p0 = None
        self.t_ant = -1.0
        self.linhas = []
        self.fim = False
        self.motivo = 'concluído'
        self.create_timer(1.0 / cfg.taxa, self.passo)

    def cb_pose(self, msg):
        self.pose = msg

    def cb_roda(self, msg):
        self.roda = msg

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def derivada(self, t, x, y, yaw):
        """Velocidade linear e de guinada tiradas da POSE, numa janela curta."""
        self.hist.append((t, x, y, yaw))
        while len(self.hist) > 1 and t - self.hist[0][0] > JANELA_S:
            self.hist.popleft()
        if len(self.hist) < 2:
            return 0.0, 0.0
        t1, x1, y1, yaw1 = self.hist[0]
        dt = t - t1
        if dt < 1e-6:
            return 0.0, 0.0
        return math.hypot(x - x1, y - y1) / dt, norm_ang(yaw - yaw1) / dt

    # ---------------- os ensaios ----------------

    def comando(self, te):
        """Devolve (v, wz) do ensaio no instante te. Um lugar só."""
        c = self.cfg
        e = c.ensaio

        if e == 'zona_morta_linear':
            # Rampa lenta de linear até a roda sair do lugar. O que sai daqui
            # é o v mínimo que produz movimento — a zona morta em m/s.
            return c.rampa_ate * te / c.dur, 0.0

        if e == 'zona_morta_giro':
            # Mesma ideia, girando parado: o wz mínimo que tira o robô do
            # lugar. É o pior caso da zona morta (as duas rodas pequenas).
            return 0.0, c.rampa_ate * te / c.dur

        if e == 'degrau_giro':
            # 2 s reto, 2 s de giro constante, resto SEM comando de giro.
            # a_dec = wz² / (2·Δrumo depois do corte).
            if te < 2.0:
                return c.v, 0.0
            if te < 4.0:
                return c.v, c.wz
            return c.v, 0.0

        if e == 'curva':
            # Curva sustentada: mede o wz REALIZADO contra o comandado e a
            # derrapada (odometria de roda contra pose). Rodar em várias
            # velocidades é o que responde "quanto ele curva a x, 2x, 3x".
            return (c.v, c.wz) if te >= 2.0 else (c.v, 0.0)

        if e == 'reta':
            # Reta com CUTUCÃO: anda, leva um pulso de giro de 0,5 s e SOLTA.
            # `--v` aceita NEGATIVO, e é assim que se mede a ré — de ré a boba
            # passa a ser a roda da frente, e boba na frente é a configuração
            # geometricamente instável (o carrinho de supermercado).
            #
            # O pulso não é enfeite: sem ele o ensaio não mede nada. O robô
            # simulado é perfeitamente simétrico num plano liso, então reta
            # pura dá desvio ZERO EXATO nos dois sentidos (medido: 0,0° e
            # 0,0 cm em 4 m, ida e ré). Instabilidade é bifurcação — só se vê
            # perturbando e olhando se o desvio volta ou cresce.
            #
            # `--wz 0` desliga o pulso, para quem quiser a reta crua no robô
            # real, onde a assimetria de verdade perturba sozinha.
            if te < 2.0:
                return 0.0, 0.0
            if 5.0 <= te < 5.5:
                return c.v, c.wz
            return c.v, 0.0

        if e == 'aceleracao_linear':
            # Degrau de linear e corte: acelera e desacelera de fato quanto?
            if te < 2.0:
                return 0.0, 0.0
            if te < 6.0:
                return c.v, 0.0
            return 0.0, 0.0

        raise SystemExit(f'ensaio desconhecido: {e}')

    # ---------------- laço ----------------

    def passo(self):
        if self.pose is None:
            return
        t = self.agora()
        if self.t0 is None:
            self.t0 = t
            self.p0 = (self.pose.pose.pose.position.x,
                       self.pose.pose.pose.position.y)
        te = t - self.t0

        if te < self.t_ant - 1e-6:
            self.parar('relógio andou pra trás (mais de uma fonte de /clock?)')
            return
        self.t_ant = te

        p = self.pose.pose.pose
        x, y = p.position.x, p.position.y
        yaw = yaw_de(p.orientation)

        # Trava de espaço: o robô é real e o laboratório tem parede.
        if math.hypot(x - self.p0[0], y - self.p0[1]) > self.cfg.espaco:
            self.parar(f'estourou o espaço de {self.cfg.espaco:.1f} m')
            return
        if te > self.cfg.dur:
            self.parar('concluído')
            return

        v_pose, wz_pose = self.derivada(t, x, y, yaw)
        cv, cw = self.comando(te)
        self.publica(cv, cw)

        self.linhas.append({
            't': round(te, 3),
            'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4),
            'cmd_v': round(cv, 4), 'cmd_wz': round(cw, 4),
            'v_pose': round(v_pose, 4), 'wz_pose': round(wz_pose, 4),
            'v_twist': round(self.pose.twist.twist.linear.x, 4),
            'wz_twist': round(self.pose.twist.twist.angular.z, 4),
            'x_roda': round(self.roda.pose.pose.position.x, 4) if self.roda else '',
            'y_roda': round(self.roda.pose.pose.position.y, 4) if self.roda else '',
            'yaw_roda': (round(yaw_de(self.roda.pose.pose.orientation), 4)
                         if self.roda else ''),
        })

    def parar(self, motivo):
        self.motivo = motivo
        self.publica(0.0, 0.0)
        self.fim = True

    def publica(self, v, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(wz)
        self.pub.publish(m)

    def grava(self):
        # Zerar de novo na saída: se o nó morreu no meio, o robô não pode
        # continuar andando com o último comando.
        for _ in range(5):
            self.publica(0.0, 0.0)
        if not self.linhas:
            print('sem dados — o /Odometry chegou?', file=sys.stderr)
            return
        with open(self.cfg.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.linhas[0].keys()))
            w.writeheader()
            w.writerows(self.linhas)
        print(f'{len(self.linhas)} amostras -> {self.cfg.csv} ({self.motivo})',
              file=sys.stderr)


ENSAIOS = ['zona_morta_linear', 'zona_morta_giro', 'degrau_giro', 'curva',
           'aceleracao_linear', 'reta']


def main():
    ap = argparse.ArgumentParser(
        description='Banco de ensaios de movimentação (robô real ou simulador)')
    ap.add_argument('--ensaio', choices=ENSAIOS, required=True)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--dur', type=float, default=20.0, help='teto de tempo [s]')
    ap.add_argument('--espaco', type=float, default=4.0,
                    help='distância máxima da origem [m] — trava de segurança')
    ap.add_argument('--v', type=float, default=0.3, help='linear do ensaio [m/s]')
    ap.add_argument('--wz', type=float, default=0.5, help='giro do ensaio [rad/s]')
    ap.add_argument('--rampa-ate', dest='rampa_ate', type=float, default=0.35,
                    help='valor final da rampa nos ensaios de zona morta')
    ap.add_argument('--taxa', type=float, default=50.0, help='malha do banco [Hz]')
    ap.add_argument('--sim', action='store_true',
                    help='usar tempo de simulação (no robô real, NÃO passar)')
    cfg = ap.parse_args()

    rclpy.init()
    no = Ensaio(cfg)
    try:
        while rclpy.ok() and not no.fim:
            rclpy.spin_once(no, timeout_sec=0.1)
    except KeyboardInterrupt:
        no.motivo = 'interrompido no teclado'
    finally:
        no.grava()
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
