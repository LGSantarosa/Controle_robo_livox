#!/usr/bin/env python3
"""Amostra `/Odometry` continuamente e diz quando o robô assentou.

    amostra_odom.py <saida.csv> <prazo_s> [--exige-repouso]

Existe porque `ros2 topic echo --once` repetido em laço de shell produz
exatamente o que a conferência de repouso reprova: amostras esparsas, com
buracos de centenas de milissegundos entre elas. Assinar uma vez e escutar dá
uma série contínua, que é o que `repouso_sustentado` sabe julgar.

Sai com 0 quando o repouso foi confirmado, 1 quando o prazo acabou sem ele, e
2 em erro de ROS. Grava o CSV nos dois primeiros casos — o dado de uma espera
que falhou é justamente o que explica por que falhou.
"""
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import prontidao as pr  # noqa: E402


def main():
    if len(sys.argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    saida, prazo = sys.argv[1], float(sys.argv[2])
    exige = '--exige-repouso' in sys.argv

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    amostras = []

    class Amostrador(Node):
        def __init__(self):
            super().__init__('ab_freio_amostrador')
            qos = QoSProfile(depth=50,
                             reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Odometry, '/Odometry', self.cb, qos)
            self.t0 = self.get_clock().now().nanoseconds / 1e9
            self.pronto = False

        def cb(self, m):
            t = self.get_clock().now().nanoseconds / 1e9
            amostras.append((t - self.t0,
                             m.twist.twist.linear.x,
                             m.twist.twist.angular.z,
                             m.pose.pose.position.x,
                             m.pose.pose.position.y))
            if exige and not self.pronto:
                ok, _ = pr.repouso_sustentado(
                    [(a[0], a[1], a[2]) for a in amostras])
                self.pronto = ok

    rclpy.init()
    no = Amostrador()
    try:
        while rclpy.ok():
            rclpy.spin_once(no, timeout_sec=0.1)
            agora = no.get_clock().now().nanoseconds / 1e9 - no.t0
            if no.pronto or agora >= prazo:
                break
    finally:
        with open(saida, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'vx', 'wz', 'px', 'py'])
            w.writerows(amostras)
        no.destroy_node()
        rclpy.try_shutdown()

    ok, motivo = pr.repouso_sustentado([(a[0], a[1], a[2]) for a in amostras])
    print(motivo)
    if not exige:
        return 0
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
