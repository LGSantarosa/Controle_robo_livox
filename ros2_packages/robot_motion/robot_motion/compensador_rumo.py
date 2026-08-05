#!/usr/bin/env python3
"""Compensador de rumo — a camada que faz "reto" significar reto.

Este robô comandado a ir reto descreve um círculo de 1,22 m de raio
(−0,817 1/m de frente; de ré −0,098, medido em 04-08). Este nó fica entre
quem comanda e o atuador: entra o `cmd_vel` desejado, sai o `cmd_vel`
corrigido, com o yaw do `/Odometry` fechando a malha. A lei (feedforward
medido + PI) vive em `lei_de_reta.py`, pura e testada sem ROS; o racional
com os números é a decisão 011.

Serve QUALQUER comandante — bancada, teleop, seguidor, Nav2 — porque
corrigir fidelidade de comando é problema de todos eles e não deve ser
resolvido dentro de nenhum.

Tópicos:
    entra  ~/cmd_vel     geometry_msgs/TwistStamped   o comando desejado
           /Odometry     nav_msgs/Odometry            pose (LIO ou Gazebo)
    sai    /hoverboard_base_controller/cmd_vel        o comando corrigido
           (no simulador, remapear a saída para /cmd_vel_bruto, como o
            heading_controller já faz no navegacao.launch.py)

⚠️ Sem pose fresca o nó NÃO corrige: passa o comando adiante INTOCADO e
avisa alto. A alternativa (segurar o comando) violaria a prioridade do
humano; corrigir sem sensor é impossível; e fazer qualquer um dos dois em
silêncio é o defeito do BO-3. O robô volta a arcar — mas dizendo por quê.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from robot_motion.heading_controller import yaw_de
from robot_motion.lei_de_reta import MalhaDeReta


class CompensadorRumo(Node):
    def __init__(self):
        super().__init__('compensador_rumo')

        p = self.declare_parameters('', [
            # A curvatura MEDIDA do robô comandado reto (bancada 04-08) —
            # com sinal, na convenção da bancada (Δyaw/caminho).
            ('curv_frente', -0.817),
            ('curv_re', -0.098),
            ('kp', 1.0),            # [1/s]  wz por rad de erro de rumo
            ('ki', 0.5),            # [1/s²] wz por rad·s acumulado
            ('wz_max', 0.6),        # [rad/s] grampo da correção
            ('int_max', 0.6),       # [rad·s] anti-windup
            ('limiar_curva', 0.05),  # [rad/s] acima disso é curva: passa
            # False quando há controlador de rumo ACIMA (a pilha): aí este nó
            # só cancela o arco e não disputa a direção. Ver `lei_de_reta`.
            ('segura_rumo', True),
            # Pose mais velha que isto = sem sensor: passa reto e grita.
            ('validade_pose', 0.5),  # [s]
        ])
        par = {x.name: x.value for x in p}

        self.malha = MalhaDeReta(
            curv_frente=par['curv_frente'], curv_re=par['curv_re'],
            kp=par['kp'], ki=par['ki'], wz_max=par['wz_max'],
            int_max=par['int_max'], limiar_curva=par['limiar_curva'],
            segura_rumo=par['segura_rumo'])
        self.validade_pose = par['validade_pose']

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            TwistStamped, '/hoverboard_base_controller/cmd_vel', qos)
        self.create_subscription(TwistStamped, '~/cmd_vel', self.cb_cmd, qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, qos)

        self.yaw = None
        self.t_pose = None
        self.t_passo = None
        self.v_real = 0.0
        self.hist = []      # (t, x, y) para medir a velocidade de verdade

        self.get_logger().warn(
            f"compensador de rumo (decisão 011): ff {par['curv_frente']:+.3f} "
            f"1/m frente, {par['curv_re']:+.3f} ré; kp={par['kp']:.2f} "
            f"ki={par['ki']:.2f}, correção limitada a ±{par['wz_max']:.2f} "
            f"rad/s. Curva pedida (|wz|≥{par['limiar_curva']:.2f}) passa "
            f"intocada.")

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def cb_odom(self, msg):
        self.yaw = yaw_de(msg.pose.pose.orientation)
        self.t_pose = self.agora()
        # Velocidade MEDIDA, da pose. O feedforward precisa dela e não do
        # comando: o arco é curvatura × distância percorrida, e quem decide a
        # distância é o patamar da placa. Janela de 0,2 s para não derivar
        # ruído; fonte é a pose, nunca o campo `twist` (`ensaio.py`).
        p = msg.pose.pose.position
        self.hist.append((self.t_pose, p.x, p.y))
        while len(self.hist) > 2 and self.t_pose - self.hist[0][0] > 0.2:
            self.hist.pop(0)
        if len(self.hist) >= 2:
            (t0, x0, y0), (t1, x1, y1) = self.hist[0], self.hist[-1]
            if t1 - t0 > 1e-4:
                self.v_real = math.hypot(x1 - x0, y1 - y0) / (t1 - t0)

    def cb_cmd(self, msg):
        v = msg.twist.linear.x
        wz = msg.twist.angular.z
        t = self.agora()

        sem_pose = (self.yaw is None
                    or t - self.t_pose > self.validade_pose)
        if sem_pose:
            # Nunca em silêncio (BO-3): o robô vai arcar, e o log diz.
            if abs(v) > 1e-9 or abs(wz) > 1e-9:
                self.get_logger().error(
                    'SEM POSE FRESCA no /Odometry — comando passa SEM '
                    'correção de rumo, o robô vai arcar como sempre arcou',
                    throttle_duration_sec=1.0)
            self.malha._descarta()
            return self.publica(msg, v, wz)

        dt = 0.0 if self.t_passo is None else t - self.t_passo
        self.t_passo = t
        self.publica(msg, v,
                     self.malha.passo(v, wz, self.yaw, dt, self.v_real))

    def publica(self, msg, v, wz):
        fora = TwistStamped()
        fora.header = msg.header
        fora.twist.linear.x = float(v)
        fora.twist.angular.z = float(wz)
        self.pub.publish(fora)


def main():
    rclpy.init()
    no = CompensadorRumo()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
