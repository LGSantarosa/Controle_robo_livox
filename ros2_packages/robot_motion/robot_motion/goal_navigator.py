#!/usr/bin/env python3
"""Navegação ponto a ponto do robô 2 — a fatia "ir a um ponto".

Recebe um objetivo e conduz o robô até ele. **Não desvia de obstáculo** — isso
é a fatia B, depende do Livox e de percepção que este repo ainda não tem.

Fala com a movimentação pelos MESMOS tópicos que qualquer outro cliente usaria
(`rumo_alvo` e `velocidade_alvo`), sem atalho por dentro: a navegação decide
para onde ir, a movimentação decide o que o atuador aguenta. Se a navegação
pedir velocidade que não sustenta a curva, é a movimentação que corta o giro —
e ela já sabe fazer isso (decisão 005).

Tópicos:
    entra  ~/objetivo   geometry_msgs/PoseStamped   ponto de destino
           /Odometry    nav_msgs/Odometry           pose
    sai    /heading_controller/rumo_alvo        std_msgs/Float64  [rad]
           /heading_controller/velocidade_alvo  std_msgs/Float64  [m/s]

Racional em `docs/decisoes/006-navegacao-ponto-a-ponto.md`.
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from robot_motion.navegacao_ponto import (
    comando_de_navegacao,
    distancia,
    norm_ang,
    orcamento_de_re_estourado,
    precisa_recuar,
    raio_minimo_de_chegada,
    raio_necessario,
)


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')

        p = self.declare_parameters('', [
            ('v_max', 0.5),
            # Desaceleração LINEAR que a máquina entrega [m/s²].
            # NÃO MEDIDA — ensaio `aceleracao_linear` do tools/banco.
            # Conservadora de propósito, mesma regra da a_dec angular:
            # subestimar só faz o robô começar a frear antes.
            ('a_lin', 0.5),
            # Menor velocidade que tira a roda do lugar [m/s]. Sai da zona
            # morta medida; abaixo disso a placa ignora e o robô para longe
            # do ponto sem acusar nada (BO-3).
            ('v_min_viavel', 0.20),
            # Raio de chegada [m]. Se ficar menor que o mínimo coerente com
            # o piso, o nó corrige e avisa — robô orbitando o alvo é defeito
            # difícil de ler no log.
            ('raio_chegada', 0.15),
            # Giro que a máquina SUSTENTA numa curva [rad/s] — não o teto.
            # NÃO MEDIDO: sai do ensaio `curva` do tools/banco.
            # Limita a velocidade de aproximação: perto do alvo, ir rápido
            # demais faz o ponto escapar pelo lado e o robô orbitá-lo.
            ('wz_util', 0.5),
            # Raio da curva mais fechada que o robô descreve [m]. Ponto que
            # exige menos que isso está DENTRO do círculo dele, e não se
            # alcança perseguindo — só recuando (decisão 007).
            #   negativo = deduzir de v_min_viavel/wz_util (o padrão)
            #   ZERO     = o robô pivota; nunca dá ré
            ('raio_min_curva', -1.0),
            # Teto de velocidade da ré [m/s]. Não é a velocidade da manobra:
            # ela recua no mínimo viável, que é o mais devagar que ainda tira
            # a roda do lugar. Este número existe para DELATAR quando o
            # mínimo viável já é mais rápido do que gostaríamos de recuar.
            ('v_re_max', 0.20),
            # Orçamento da manobra: recuar sem fim é pior que não alcançar.
            ('re_max_dist', 1.0),
            ('re_max_s', 8.0),
            ('taxa', 20.0),
            ('timeout_pose', 1.0),
        ])
        self.par = {x.name: x.value for x in p}

        minimo = raio_minimo_de_chegada(self.par['v_min_viavel'],
                                        self.par['a_lin'])
        if self.par['raio_chegada'] < minimo:
            self.get_logger().warn(
                f"raio_chegada={self.par['raio_chegada']:.3f} m é menor que o "
                f'mínimo coerente ({minimo:.3f} m) para v_min_viavel='
                f"{self.par['v_min_viavel']} e a_lin={self.par['a_lin']}. "
                'O robô ficaria orbitando o ponto. Usando o mínimo.')
            self.par['raio_chegada'] = minimo

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_rumo = self.create_publisher(
            Float64, '/heading_controller/rumo_alvo', qos)
        self.pub_vel = self.create_publisher(
            Float64, '/heading_controller/velocidade_alvo', qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, qos)
        self.create_subscription(PoseStamped, '~/objetivo', self.cb_objetivo, qos)

        self.pose = None
        self.t_pose = None
        self.objetivo = None
        self.anunciou_chegada = False

        # estado da manobra de ré
        self.recuando = False
        self.p_re0 = None
        self.t_re0 = None
        self.desistiu = False

        self.create_timer(1.0 / self.par['taxa'], self.passo)
        self.get_logger().warn(
            'a_lin e v_min_viavel NÃO foram medidos neste robô — '
            'rodar tools/banco/ e corrigir (BO-3)')
        self.anuncia_a_re()

    def anuncia_a_re(self):
        """Diz na subida se este robô vai precisar recuar, e por quê."""
        if self.par['raio_min_curva'] < 0.0:
            self.par['raio_min_curva'] = (self.par['v_min_viavel']
                                          / self.par['wz_util'])
        if self.par['raio_min_curva'] <= 0.0:
            self.get_logger().info(
                'raio_min_curva = 0: este robô pivota, e a manobra de ré '
                'nunca será acionada')
            return
        self.get_logger().info(
            f"raio mínimo de curva {self.par['raio_min_curva']:.2f} m "
            f"(v_min_viavel/wz_util). Alvo que exigir menos que isso está "
            f'dentro do círculo do robô: ele RECUA até caber, reto, no '
            f'mínimo viável, com teto de '
            f"{self.par['re_max_dist']:.2f} m e {self.par['re_max_s']:.0f} s.")
        if self.par['v_min_viavel'] > self.par['v_re_max']:
            self.get_logger().warn(
                f"a ré vai a {self.par['v_min_viavel']:.2f} m/s, acima do "
                f"teto desejado de {self.par['v_re_max']:.2f} m/s: o mínimo "
                'viável manda, senão a placa engole o comando e a manobra '
                'trava calada (BO-3). Zona morta menor baixaria isso.')

    def cb_odom(self, msg):
        self.pose = msg
        self.t_pose = self.get_clock().now().nanoseconds * 1e-9

    def cb_objetivo(self, msg):
        # O objetivo tem que estar no mesmo referencial da pose. Aceitar
        # frame diferente calado levaria o robô para o lugar errado com toda
        # a confiança do mundo.
        quadro = self.pose.header.frame_id if self.pose else 'odom'
        if msg.header.frame_id and msg.header.frame_id != quadro:
            self.get_logger().error(
                f"objetivo em '{msg.header.frame_id}' mas a pose está em "
                f"'{quadro}' — ignorado. Não há transformação aqui.")
            return
        self.objetivo = (msg.pose.position.x, msg.pose.position.y)
        self.anunciou_chegada = False
        # Objetivo novo zera a desistência: a manobra falhou para o alvo
        # anterior, não para sempre.
        self.desistiu = False
        self.encerra_re()
        d = self.distancia_atual()
        self.get_logger().info(
            f'objetivo ({self.objetivo[0]:.2f}, {self.objetivo[1]:.2f})'
            + (f' — {d:.2f} m daqui' if d is not None else ''))

    def distancia_atual(self):
        if self.pose is None or self.objetivo is None:
            return None
        p = self.pose.pose.pose.position
        return distancia(p.x, p.y, *self.objetivo)

    def passo(self):
        agora = self.get_clock().now().nanoseconds * 1e-9

        if self.pose is None or self.t_pose is None:
            return
        if agora - self.t_pose > self.par['timeout_pose']:
            # Sem pose fresca não há navegação: parar de mandar alvo faz a
            # movimentação parar o robô sozinha, pelo timeout dela.
            self.get_logger().warn('/Odometry parou de chegar',
                                   throttle_duration_sec=5.0)
            return
        if self.objetivo is None:
            return

        p = self.pose.pose.pose.position
        chegou, rumo, vel = comando_de_navegacao(
            p.x, p.y, yaw_de(self.pose.pose.pose.orientation),
            self.objetivo[0], self.objetivo[1],
            v_max=self.par['v_max'],
            a_lin=self.par['a_lin'],
            v_min_viavel=self.par['v_min_viavel'],
            raio_chegada=self.par['raio_chegada'],
            wz_util=self.par['wz_util'],
        )

        if chegou:
            if not self.anunciou_chegada:
                self.anunciou_chegada = True
                self.get_logger().info(
                    f'chegou — {self.distancia_atual():.2f} m do ponto '
                    f"(raio {self.par['raio_chegada']:.2f} m)")
            # Em cima do ponto, a direção ATÉ ele gira sozinha (o vetor fica
            # curto e qualquer deriva o faz varrer). Mandar esse rumo depois
            # de chegar punha o robô girando no lugar para sempre, e o giro
            # arrastava a traseira para fora do raio — medido, 0,06 m viraram
            # 0,27 m em 4 s. O rumo alvo passa a ser o rumo ATUAL: erro zero,
            # robô quieto.
            self.encerra_re()
            self.publica(yaw_de(self.pose.pose.pose.orientation), 0.0)
            return

        yaw = yaw_de(self.pose.pose.pose.orientation)
        erro = norm_ang(rumo - yaw)
        d = self.distancia_atual()

        if self.desistiu:
            # Já gritou uma vez; fica parado esperando objetivo novo em vez de
            # repetir a manobra que não converge.
            self.publica(yaw, 0.0)
            return

        if precisa_recuar(d, erro, self.par['raio_min_curva'], self.recuando):
            self.recua(agora, yaw, d, erro)
            return

        if self.recuando:
            self.get_logger().info(
                f'alvo cabe na curva agora (precisa de '
                f'{raio_necessario(d, erro):.2f} m de raio, tenho '
                f"{self.par['raio_min_curva']:.2f}) — recuei "
                f'{self.recuo_atual():.2f} m. Seguindo.')
            self.encerra_re()

        self.publica(rumo, vel)

    def recua(self, agora, yaw, d, erro):
        """Ré reta até o alvo caber na curva — com orçamento e com voz.

        O rumo alvo publicado é o rumo ATUAL: erro zero, giro zero. É o que
        faz a ré ser reta sem a movimentação precisar saber de manobra
        nenhuma; para ela é só uma velocidade negativa.
        """
        p = self.pose.pose.pose.position
        if not self.recuando:
            self.recuando = True
            self.p_re0 = (p.x, p.y)
            self.t_re0 = agora
            self.get_logger().warn(
                f'alvo a {d:.2f} m e {math.degrees(erro):.0f}° exige raio de '
                f'{raio_necessario(d, erro):.2f} m, e o meu mínimo é '
                f"{self.par['raio_min_curva']:.2f} m — ele está DENTRO do meu "
                'círculo. Recuando para abrir a curva.')

        if orcamento_de_re_estourado(self.recuo_atual(), agora - self.t_re0,
                                     self.par['re_max_dist'],
                                     self.par['re_max_s']):
            self.get_logger().error(
                f'RÉ ABORTADA: recuei {self.recuo_atual():.2f} m em '
                f'{agora - self.t_re0:.1f} s e o alvo continua sem caber '
                f'(precisa de {raio_necessario(d, erro):.2f} m de raio). '
                'Parando aqui — o objetivo NÃO foi alcançado. '
                'Mandar objetivo novo, ou medir bitola e zona morta para '
                'liberar o pivô (BO-3).')
            self.desistiu = True
            self.encerra_re()
            self.publica(yaw, 0.0)
            return

        self.publica(yaw, -self.par['v_min_viavel'])

    def recuo_atual(self):
        if self.p_re0 is None or self.pose is None:
            return 0.0
        p = self.pose.pose.pose.position
        return math.hypot(p.x - self.p_re0[0], p.y - self.p_re0[1])

    def encerra_re(self):
        self.recuando = False
        self.p_re0 = None
        self.t_re0 = None

    def publica(self, rumo, vel):
        self.pub_rumo.publish(Float64(data=float(rumo)))
        self.pub_vel.publish(Float64(data=float(vel)))


def main():
    rclpy.init()
    no = GoalNavigator()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
