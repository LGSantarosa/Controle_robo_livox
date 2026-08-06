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
            # ⚠️ GANHOS REDUZIDOS 4,1x EM 05-08, e o número saiu do robô.
            #
            # Com kp=1,0 e ki=0,5 o robô OSCILA, e a oscilação CRESCE — não é
            # transiente. Medido nas três corridas da bancada de 05-08
            # (`docs/dados/2026-08-05-bancada-robo/A2-com-comp-frente-*.csv`):
            #
            #     1ª excursão  −2,97 / −6,47 / −5,43 graus
            #     2ª excursão  +6,46 / +12,04 / +11,84
            #     cresce 2,07x por meio-período, meio-período 2,46 s
            #
            # É o S que o dono viu a olho ("de frente ele faz um pequeno S para
            # tentar compensar o erro"). A causa é TEMPO MORTO, e o número
            # fecha por duas rotas independentes:
            #
            #   · a frequência da oscilação (1,277 rad/s) com este PI exige
            #     0,94 s de atraso puro no laço;
            #   · a placa MEDIDA tem 0,27 s de atraso de liga (01-08) mais
            #     0,52 s de desliga (04-08) = 0,79 s, mais pose a 10 Hz e a
            #     janela de 0,2 s da velocidade -> ~0,94 s.
            #
            # Crescer 2,07x por meio-período põe o ganho de laço em ~2,07 na
            # travessia de fase; ele precisa ficar abaixo de 1. Reduzindo os
            # dois JUNTOS por 4,1x (a razão ki/kp não muda — é ganho a menos,
            # não controlador diferente) o laço vai para |L| ~ 0,5, ou seja
            # margem de ganho 2x.
            #
            # PREÇO, calculado: o feedforward erra ~0,095 1/m (planta de 05-08
            # em −0,9116 contra ff −0,817), o que pede 0,028 rad/s de correção
            # contínua a 0,30 m/s. Com kp=0,25 isso vira ~6,5° de erro de rumo
            # enquanto o integrador não o come, contra 1,6° antes. Trocar ±12°
            # CRESCENDO por 6,5° assentando é o negócio.
            #
            # ⚠️ NÃO foi possível verificar isto no simulador, e a razão está
            # em `docs/dados/2026-08-05-ki-no-simulador/`: lá o feedforward
            # cancela o arco por construção (a placa fingida e este nó usam o
            # MESMO −0,817), e a boba é um patim (BO-4). O simulador não
            # oscila em ganho nenhum — varrer ki de 1,0 a 0,0 não mudou uma
            # única inversão. O projeto veio da planta medida, que é o que
            # sobra quando o simulador não pode arbitrar.
            #
            # ⏳ A CONFERIR NO ROBÔ (`docs/PLANO_SINTONIA_RUMO.md`): a mesma
            # corrida tem de mostrar a amplitude DECAINDO em vez de crescer.
            # Se ela continuar crescendo com estes ganhos, o S não é do laço e
            # o caminho passa a ser o BO-4.
            ('kp', 0.25),           # [1/s]  wz por rad de erro de rumo
            ('ki', 0.12),           # [1/s²] wz por rad·s acumulado
            ('wz_max', 0.6),        # [rad/s] grampo da correção
            # ⚠️ FICA EM 0,6, e isto foi DECIDIDO com medida em 05-08, não
            # herdado. Ao baixar `ki` 4,1x eu subi este teto para 2,5 achando
            # que o que precisava ser preservado era a AUTORIDADE do
            # integrador (`ki · int_max`). O teste de fechamento derrubou:
            #
            #   int_max   ff certo: 2ª excursão | curvatura    ff 25% errado: curv
            #     0,6          0,4°  | −0,0012                      0,0007
            #     1,0          7,1°  | +0,0061                     −0,0026
            #     2,5          9,0°  | +0,0076                     +0,0053
            #
            # Com tempo morto, `int_max` não é só teto de autoridade: é a
            # proteção contra WINDUP, que é justamente o que produz sobrepasso.
            # Subir para 2,5 trouxe o sino de volta (9° de segunda excursão) —
            # desfazendo a mudança que veio matá-lo.
            #
            # Em 0,6 a curvatura fica em 0,0007 e −0,0012 nos dois casos, três
            # ordens de grandeza abaixo do critério de 0,05 da 011. O preço é
            # que, com o ff 25% errado, o rumo assenta ~6,8° fora da referência
            # capturada (o integrador satura e o P sustenta o resto com erro).
            # ➡️ O conserto disso é o FEEDFORWARD estar certo, não o integrador
            # brigar — ver `curv_frente` acima.
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
