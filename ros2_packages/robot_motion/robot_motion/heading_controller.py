#!/usr/bin/env python3
"""Controlador de rumo do robô 2 — a camada de movimentação.

Recebe um **rumo alvo** (rad, no referencial do `/Odometry`) e uma velocidade
de cruzeiro, e entrega `cmd_vel` em SI para o `diff_drive_controller`. Ele
responde por RUMO: leva o bico até a direção pedida e o segura lá. Distância
lateral até uma rota é problema da navegação, que conhece a rota — este nó
segura o rumo e segue paralelo.

A lei está em `lei_de_rumo.py`, testável sem ROS. O racional inteiro, com os
números que o justificam, está em
`docs/decisoes/005-lei-de-frenagem-de-rumo.md`.

Tópicos:
    entra  ~/rumo_alvo        std_msgs/Float64      [rad]
           ~/velocidade_alvo  std_msgs/Float64      [m/s]  (opcional)
           /Odometry          nav_msgs/Odometry     pose (FAST-LIO ou Gazebo)
    sai    /hoverboard_base_controller/cmd_vel   geometry_msgs/TwistStamped

⚠️ Os parâmetros de fábrica são CONSERVADORES E NÃO MEDIDOS. Ver o BO-3 no
`ESTADO_PROJETO.md` e o protocolo em `tools/banco/README.md`.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from robot_motion.lei_de_rumo import (
    comando,
    comando_de_re,
    norm_ang,
    pivo_disponivel,
    wz_minimo_parado,
)


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class HeadingController(Node):
    def __init__(self):
        super().__init__('heading_controller')

        # ---- parâmetros, todos em SI, todos num lugar só ----
        p = self.declare_parameters('', [
            # Desaceleração angular que a máquina entrega. O parâmetro central:
            # o sobrepasso do controlador velho era wz²/(2·a_dec).
            # NÃO MEDIDO — banco de ensaios, ensaio `degrau_giro`.
            # Regra de ouro medida: errar pra BAIXO é de graça (sobrepasso
            # zero, 0,3 s a mais); errar pra cima traz o S de volta.
            ('a_dec', 0.3),
            ('wz_max', 1.0),
            ('v_max', 0.5),
            # Zona morta da RODA. NÃO MEDIDA — ensaios `zona_morta_*`.
            # Chute alto de propósito: o piso que sai daqui é o que impede o
            # robô de ficar plantado no chão (BO-3).
            ('zona_morta', 0.15),
            # Bitola. NÃO MEDIDA neste robô — herdada do YAML do controlador.
            ('bitola', 0.32),
            ('margem_piso', 0.05),
            ('tolerancia_rumo', 0.02),
            ('taxa', 20.0),
            # Sem alvo novo por este tempo, o robô para. Alvo velho é alvo
            # perigoso.
            ('timeout_alvo', 1.0),
            # Detector de plantão: comando saindo e pose sem mudar.
            ('plantao_s', 0.5),
        ])
        self.par = {x.name: x.value for x in p}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(
            TwistStamped, '/hoverboard_base_controller/cmd_vel', qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, qos)
        self.create_subscription(Float64, '~/rumo_alvo', self.cb_rumo, qos)
        self.create_subscription(
            Float64, '~/velocidade_alvo', self.cb_velocidade, qos)

        self.pose = None
        self.t_pose = None
        self.hist = []          # (t, x, y) para estimar a direção do movimento
        self.rumo_alvo = None
        self.t_alvo = None
        self.v_alvo = self.par['v_max']

        # estado do detector de plantão
        self.pose_ref = None
        self.t_ref = None
        self.ja_reclamou = False

        self.create_timer(1.0 / self.par['taxa'], self.passo)
        self.avisa_de_saida()

    def avisa_de_saida(self):
        """Diz na subida com que números está trabalhando e o que eles custam.

        Parâmetro não medido que ninguém vê é parâmetro que vira verdade por
        esquecimento.
        """
        wz_min = wz_minimo_parado(self.par['zona_morta'], self.par['bitola'])
        self.get_logger().warn(
            'parâmetros NÃO MEDIDOS neste robô — '
            f"a_dec={self.par['a_dec']} rad/s², "
            f"zona_morta={self.par['zona_morta']} m/s, "
            f"bitola={self.par['bitola']} m. "
            'Rodar tools/banco/ e corrigir (BO-3).')
        if pivo_disponivel(self.par['zona_morta'], self.par['bitola'],
                           self.par['margem_piso'], self.par['wz_max']):
            self.get_logger().info(
                f'pivô DISPONÍVEL acima de {wz_min:.2f} rad/s — o robô '
                'consegue virar no próprio eixo')
        else:
            self.get_logger().error(
                f'pivô INDISPONÍVEL: girar parado exigiria mais de '
                f"{2*(self.par['zona_morta']+self.par['margem_piso'])/self.par['bitola']:.2f}"
                f" rad/s, e o teto é {self.par['wz_max']:.2f}. O robô só faz "
                'arcos, e pontos perto dele ficam INALCANÇÁVEIS. '
                'Bitola maior ou zona morta menor resolvem — medir com trena.')

    def cb_odom(self, msg):
        self.pose = msg
        self.t_pose = self.get_clock().now().nanoseconds * 1e-9
        p = msg.pose.pose.position
        self.hist.append((self.t_pose, p.x, p.y))
        while len(self.hist) > 1 and self.t_pose - self.hist[0][0] > 0.3:
            self.hist.pop(0)

    def direcao_do_movimento(self):
        """Para onde o robô ANDA de fato — não para onde aponta.

        Este robô escorrega: motriz na frente, boba atrás. Em curva fechada a
        diferença entre as duas passou de 37° em ensaio, e foi ela que
        sustentou uma órbita infinita em volta de um ponto. Devolve None
        quando o robô está lento demais para a estimativa valer.
        """
        if len(self.hist) < 2:
            return None
        t0, x0, y0 = self.hist[0]
        t1, x1, y1 = self.hist[-1]
        dt = t1 - t0
        if dt < 1e-3:
            return None
        if math.hypot(x1 - x0, y1 - y0) / dt < 0.05:
            return None
        return math.atan2(y1 - y0, x1 - x0)

    def cb_rumo(self, msg):
        self.rumo_alvo = float(msg.data)
        self.t_alvo = self.get_clock().now().nanoseconds * 1e-9

    def cb_velocidade(self, msg):
        # Velocidade NEGATIVA é o pedido de ré, e é assim que a navegação
        # pede a manobra — sem tópico novo, sem modo escondido. O sinal já
        # diz tudo: quem manda -0,2 quer recuar a 0,2 m/s.
        self.v_alvo = max(-self.par['v_max'],
                          min(float(msg.data), self.par['v_max']))

    def passo(self):
        agora = self.get_clock().now().nanoseconds * 1e-9

        # Sem pose não há controle de rumo possível: parar é a única resposta
        # honesta. Andar às cegas com o último rumo conhecido é o que faz robô
        # entrar em parede.
        if self.pose is None or self.t_pose is None:
            return self.para('sem /Odometry')
        if agora - self.t_pose > self.par['timeout_alvo']:
            return self.para('/Odometry parou de chegar')
        if self.rumo_alvo is None or self.t_alvo is None:
            return self.para(None)          # ainda não mandaram alvo: quieto
        if agora - self.t_alvo > self.par['timeout_alvo']:
            return self.para('alvo de rumo venceu')

        # ---- modo ré: reta, sem giro, e sem passar pela lei de rumo ----
        if self.v_alvo < 0.0:
            v, wz = comando_de_re(self.v_alvo, self.par['zona_morta'],
                                  self.par['margem_piso'], self.par['v_max'])
            self.get_logger().info(f'RÉ a {abs(v):.2f} m/s (manobra)',
                                   throttle_duration_sec=2.0)
            self.publica(v, wz)
            self.plantao(agora, v, wz)
            return

        yaw = yaw_de(self.pose.pose.pose.orientation)
        erro = norm_ang(self.rumo_alvo - yaw)

        # Quanto o MOVIMENTO está fora do rumo pedido. É ele que decide se
        # vale a pena avançar ou se é hora de parar e virar.
        direcao = self.direcao_do_movimento()
        erro_mov = None if direcao is None else norm_ang(self.rumo_alvo - direcao)

        v, wz = comando(
            erro,
            v_max=self.v_alvo,
            a_dec=self.par['a_dec'],
            wz_max=self.par['wz_max'],
            zona_morta=self.par['zona_morta'],
            bitola=self.par['bitola'],
            margem_piso=self.par['margem_piso'],
            tolerancia=self.par['tolerancia_rumo'],
            erro_do_movimento=erro_mov,
        )
        self.publica(v, wz)
        self.plantao(agora, v, wz)

    def plantao(self, agora, v, wz):
        """Delata comando saindo com robô imóvel.

        O prejuízo da zona morta não é o robô parar — é ninguém saber por quê.
        Nó vivo, tópico publicando, log limpo, máquina parada. Já custou horas
        numa competição. Ver BO-3.
        """
        p = self.pose.pose.pose
        atual = (p.position.x, p.position.y, yaw_de(p.orientation))

        if abs(v) < 1e-3 and abs(wz) < 1e-3:
            self.pose_ref, self.t_ref, self.ja_reclamou = None, None, False
            return

        if self.pose_ref is None:
            self.pose_ref, self.t_ref = atual, agora
            return

        andou = math.hypot(atual[0] - self.pose_ref[0], atual[1] - self.pose_ref[1])
        girou = abs(norm_ang(atual[2] - self.pose_ref[2]))
        if andou > 0.01 or girou > 0.01:
            self.pose_ref, self.t_ref, self.ja_reclamou = atual, agora, False
            return

        if agora - self.t_ref > self.par['plantao_s'] and not self.ja_reclamou:
            self.ja_reclamou = True
            self.get_logger().error(
                f'ROBÔ PARADO COM COMANDO SAINDO há {agora - self.t_ref:.1f} s '
                f'— pedindo v={v:.3f} m/s, wz={wz:.3f} rad/s e a pose não mexe. '
                f"Suspeita: zona morta (parâmetro={self.par['zona_morta']} m/s, "
                'não medido). Rodar tools/banco/, ensaios de zona morta.')

    def para(self, motivo):
        self.publica(0.0, 0.0)
        if motivo:
            self.get_logger().warn(motivo, throttle_duration_sec=5.0)

    def publica(self, v, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(wz)
        self.pub.publish(m)


def main():
    rclpy.init()
    no = HeadingController()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            no.publica(0.0, 0.0)
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
