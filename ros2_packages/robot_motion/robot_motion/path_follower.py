#!/usr/bin/env python3
"""Seguidor de caminho do robô 2 — executa o que o Nav2 planejou.

    o Nav2 diz POR ONDE ir          (/plan, Smac Hybrid-A* em Dubins)
    este nó diz PARA ONDE OLHAR     (~/rumo_alvo, ~/velocidade_alvo)
    a movimentação diz O QUE CABE   (lei_de_rumo, decisão 005)

A lei está em `lei_de_seguimento.py`, testável sem ROS; aqui é a cola de I/O e a
máquina de dois estados (SEGUINDO / RÉ). O racional, com os números que o
justificam, está em `docs/decisoes/008-nav2-planeja-nos-seguimos.md` e
`docs/decisoes/009-re-por-gatilho-nao-por-plano.md`.

Tópicos:
    entra  /plan              nav_msgs/Path      quem pede é a GUI, via
                                                 bt_navigator; o replanejamento
                                                 vem dele, não daqui
           /Odometry          nav_msgs/Odometry  pose (FAST-LIO ou Gazebo)
    sai    ~/rumo_alvo        std_msgs/Float64   [rad]
           ~/velocidade_alvo  std_msgs/Float64   [m/s] — NEGATIVA aciona a ré na
                                                 movimentação, sem tópico novo

⚠️ Este nó NÃO conhece zona morta nem `a_dec` de giro, e não fala com roda. Isso
é da movimentação, onde já está caracterizado. O único número dela que entra
aqui é o `v_piso`, e só para calcular o raio de chegada — ver o aviso de subida.
"""
import csv
import math

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from robot_motion.lei_de_seguimento import (
    ProgressoDeAvanco,
    carrot,
    chegou,
    comando_de_parada,
    curvatura_adiante,
    indice_mais_proximo,
    lookahead_de,
    orcamento_de_re,
    raio_de_chegada_minimo,
    re_esgotada,
    rumo_para,
    velocidade_de_seguimento,
)


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        p = self.declare_parameters('', [
            # Raio que a MÁQUINA fecha [m]. Governa o lookahead. Medido em
            # 29-07: 0,370 no perfil otimista de zona morta, 0,463 no
            # pessimista. Mesmo número que o planner recebe.
            ('raio_min_curva', 0.37),
            ('lookahead_fator', 1.5),
            ('lookahead_piso', 0.30),
            ('v_max', 0.5),
            ('a_lin', 0.3),
            ('wz_max', 1.0),
            # Piso de linear da MOVIMENTAÇÃO. Não é usado para comandar — ela
            # se defende sozinha — mas sem ele não dá para saber se o raio de
            # chegada pedido é possível. TEM QUE BATER com o que a movimentação
            # calcula: zona_morta + wz_max·bitola/2 + margem.
            ('v_piso', 0.335),
            ('raio_chegada', 0.25),
            # --- ré por gatilho (decisão 009) ---
            ('re_parado_s', 1.5),
            ('re_avanco_min', 0.05),
            ('re_orcamento_cego', 0.30),
            ('re_teto_s', 8.0),
            ('taxa', 20.0),
            # Sem plano novo por este tempo, para. Plano velho é plano perigoso
            # — mesma regra do `timeout_alvo` da movimentação.
            ('timeout_plano', 2.0),
            ('csv', ''),
        ])
        self.par = {x.name: x.value for x in p}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_rumo = self.create_publisher(Float64, '~/rumo_alvo', qos)
        self.pub_vel = self.create_publisher(Float64, '~/velocidade_alvo', qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, qos)
        self.create_subscription(Path, '/plan', self.cb_plano, qos)

        self.pose = None
        self.plano = []
        self.t_plano = None
        self.estado = 'ocioso'
        self.progresso = ProgressoDeAvanco(self.par['re_parado_s'],
                                           self.par['re_avanco_min'])
        self.re_desde = None
        self.re_origem = None

        self.linhas = []
        self.create_timer(1.0 / self.par['taxa'], self.passo)
        self.avisa_de_saida()

    # ------------------------------------------------------------- subida
    def avisa_de_saida(self):
        """Diz com que números está trabalhando e o que eles impedem.

        O raio de chegada é o caso onde isso morde: pedir um raio menor que a
        distância de parada a partir do piso faz o robô ORBITAR o ponto para
        sempre — e o sintoma parece defeito de controle, não de configuração.
        Melhor gritar na subida do que descobrir isso rodando.
        """
        minimo = raio_de_chegada_minimo(self.par['v_piso'], self.par['a_lin'])
        la = lookahead_de(self.par['raio_min_curva'],
                          self.par['lookahead_fator'],
                          self.par['lookahead_piso'])
        self.get_logger().info(
            f"seguidor de pé — raio da máquina {self.par['raio_min_curva']:.2f} m, "
            f'lookahead {la:.2f} m, piso de linear {self.par["v_piso"]:.3f} m/s')
        if self.par['raio_chegada'] < minimo:
            self.get_logger().error(
                f"raio_chegada={self.par['raio_chegada']:.3f} m é MENOR que a "
                f'distância de parada a partir do piso ({minimo:.3f} m). O robô '
                'vai ORBITAR o ponto sem nunca fechar. Suba o raio de chegada '
                'ou meça a zona morta — ela é quem manda no piso.')
        else:
            self.get_logger().info(
                f'raio de chegada {self.par["raio_chegada"]:.2f} m '
                f'(mínimo viável {minimo:.2f} m)')

    # --------------------------------------------------------- callbacks
    def cb_odom(self, msg):
        self.pose = msg

    def cb_plano(self, msg):
        novo = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
        if len(novo) < 2:
            return
        self.plano = novo
        self.t_plano = self.agora()
        if self.estado == 'ocioso':
            self.estado = 'seguindo'
            self.progresso.reinicia()

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publica(self, rumo, v):
        m = Float64()
        m.data = float(rumo)
        self.pub_rumo.publish(m)
        m = Float64()
        m.data = float(v)
        self.pub_vel.publish(m)

    # -------------------------------------------------------------- ciclo
    def passo(self):
        if self.pose is None or not self.plano:
            return
        t = self.agora()
        if self.t_plano is not None and t - self.t_plano > self.par['timeout_plano']:
            self.para('plano velho')
            return

        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        rumo = yaw_de(self.pose.pose.pose.orientation)
        objetivo = self.plano[-1]
        dist = math.hypot(objetivo[0] - x, objetivo[1] - y)

        if chegou(dist, self.par['raio_chegada']):
            self.para('chegou')
            return

        if self.estado == 're':
            self.passo_de_re(t, x, y, rumo, dist)
            return

        # --- seguindo ---
        i0 = indice_mais_proximo(self.plano, x, y)
        la = lookahead_de(self.par['raio_min_curva'],
                          self.par['lookahead_fator'],
                          self.par['lookahead_piso'])
        _, alvo = carrot(self.plano, i0, la)
        rumo_alvo = rumo_para(x, y, alvo)
        raio = curvatura_adiante(self.plano, i0, janela=la)
        v = velocidade_de_seguimento(dist, raio, self.par['v_max'],
                                     self.par['a_lin'], self.par['wz_max'])
        self.publica(rumo_alvo, v)
        self.registra(t, x, y, rumo, rumo_alvo, v, dist, raio)

        if self.progresso.atualiza(t, dist):
            self.entra_na_re(t, x, y, dist)

    def entra_na_re(self, t, x, y, dist):
        orcamento = orcamento_de_re(vao_traseiro=None,
                                    cego=self.par['re_orcamento_cego'])
        if orcamento <= 0.0:
            self.get_logger().warn('emperrado e sem vão para recuar — parado')
            return
        self.estado = 're'
        self.re_desde = t
        self.re_origem = (x, y)
        self.get_logger().warn(
            f'EMPERRADO a {dist:.2f} m do objetivo — ré de até {orcamento:.2f} m '
            '(CEGA: sem sensor traseiro no modelo)')

    def passo_de_re(self, t, x, y, rumo, dist):
        recuado = math.hypot(x - self.re_origem[0], y - self.re_origem[1])
        orcamento = orcamento_de_re(vao_traseiro=None,
                                    cego=self.par['re_orcamento_cego'])
        if re_esgotada(recuado, orcamento, t - self.re_desde,
                       self.par['re_teto_s']):
            self.get_logger().warn(
                f'fim da ré: recuou {recuado:.2f} m em {t - self.re_desde:.1f} s')
            self.estado = 'seguindo'
            self.progresso.reinicia()
            return
        # Ré RETA (decisão 009): mantém o rumo e anda para trás. Velocidade
        # negativa é o que aciona a ré na movimentação — sem tópico novo.
        self.publica(rumo, -self.par['v_piso'])
        self.registra(t, x, y, rumo, rumo, -self.par['v_piso'], dist, float('inf'))

    def para(self, motivo):
        v, _ = comando_de_parada()
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        rumo = yaw_de(self.pose.pose.pose.orientation)
        # Rumo alvo = o rumo ATUAL: pedir outro faria a movimentação girar, e
        # girar depois de chegar arrasta o robô para fora do ponto (0,06 m
        # viraram 0,27 m em 27-07).
        self.publica(rumo, v)
        if self.estado != 'ocioso':
            self.get_logger().info(f'parado ({motivo})')
            self.estado = 'ocioso'
            self.progresso.reinicia()

    # ------------------------------------------------------------ registro
    def registra(self, t, x, y, rumo, rumo_alvo, v, dist, raio):
        """CSV de diagnóstico — o dono só roda, os números vêm por ssh.

        `rumo_alvo` está aqui de propósito: o plano salta entre replanejamentos,
        e seguidor que persegue esse salto oscila. Se isso aparecer neste robô,
        quero o número na mão em vez de adivinhar — e só então decidir se cabe
        filtro, não antes.
        """
        if not self.par['csv']:
            return
        self.linhas.append({
            't': round(t, 3), 'estado': self.estado,
            'x': round(x, 4), 'y': round(y, 4),
            'rumo': round(rumo, 4), 'rumo_alvo': round(rumo_alvo, 4),
            'erro_rumo': round(math.atan2(math.sin(rumo_alvo - rumo),
                                          math.cos(rumo_alvo - rumo)), 4),
            'v_alvo': round(v, 4), 'dist': round(dist, 4),
            'raio_curva': ('inf' if math.isinf(raio) else round(raio, 4)),
        })

    def grava(self):
        if not self.par['csv'] or not self.linhas:
            return
        with open(self.par['csv'], 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.linhas[0].keys()))
            w.writeheader()
            w.writerows(self.linhas)
        self.get_logger().info(
            f"{len(self.linhas)} amostras -> {self.par['csv']}")


def main():
    rclpy.init()
    no = PathFollower()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.grava()
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
