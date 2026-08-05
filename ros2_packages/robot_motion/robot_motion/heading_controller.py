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

from robot_motion.lei_de_pivo import DESISTIU, PRONTO, PivoPorCorte
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
            # Bitola MEDIDA com trena em 2026-07-29 (era 0.32, herdada).
            ('bitola', 0.270),
            ('margem_piso', 0.05),
            ('tolerancia_rumo', 0.02),
            ('taxa', 20.0),
            # Sem alvo novo por este tempo, o robô para. Alvo velho é alvo
            # perigoso.
            ('timeout_alvo', 1.0),
            # Detector de plantão: comando saindo e pose sem mudar.
            ('plantao_s', 0.5),

            # ---- PIVÔ (fatia 3 da decisão 011) ----
            # Erro de rumo acima do qual vale PARAR e virar no eixo em vez de
            # arcar. Medido em 05-08: arcando, o robô realiza raio de 0,82 m
            # contra os 0,37 m que o Smac planeja — virada grande feita em arco
            # não tem como seguir o plano.
            # ⚠️ 0,26 rad (~15°) desde 05-08, era 0,70 (~40°). Com 40° ele
            # arcava até o erro ficar grande, e o arco já tinha tirado o robô
            # da rota — caminho de 1,60x. O pivô é BARATO neste robô (fecha em
            # ~2 s com resíduo abaixo de 0,1°) e o arco é CARO: virar andando
            # realiza raio de 0,82 m contra os 0,37 m que o Smac planeja.
            #
            # O piso não pode descer muito: a tolerância do pivô é ~6° e o
            # pivô mínimo da máquina é ~4°, então limiar perto disso faria a
            # manobra disparar sem parar e o robô nunca andaria. 15° dá folga
            # de 2,5x sobre a tolerância.
            ('limiar_pivo', 0.26),          # rad (~15°)
            # ⚠️ Este a_dec é do PIVÔ e NÃO é o `a_dec` acima. Ele entra numa
            # desigualdade de segurança (`a_dec da lei ≤ a_dec real`): baixo
            # demais custa pulsos, alto demais traz sobrepasso. Ver
            # `lei_de_pivo.py`.
            ('pivo_a_dec', 0.6),
            ('pivo_tolerancia', 0.105),     # rad (~6°), o piso medido é ~4°
            ('pivo_max_pulsos', 6),
            ('pivo_teto_tempo', 20.0),
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
        self.hist_yaw = []      # (t, yaw) para estimar o giro realizado
        self.wz_real = 0.0
        self.pivo = None        # manobra em curso, quando houver
        self.t_passo = None
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

        # Giro medido, para a lei do pivô decidir a hora de cortar. Sai da
        # POSE e não do campo `twist`, pela mesma razão anotada no `ensaio.py`:
        # o twist do publicador é ruidoso e a pose é limpa. Janela curta
        # (0,1 s) porque derivar duas amostras coladas amplifica ruído, e é
        # justamente `wz` ao quadrado que entra no critério de corte.
        yaw = yaw_de(msg.pose.pose.orientation)
        self.hist_yaw.append((self.t_pose, yaw))
        while len(self.hist_yaw) > 2 and self.t_pose - self.hist_yaw[0][0] > 0.1:
            self.hist_yaw.pop(0)
        if len(self.hist_yaw) >= 2:
            (t0, y0), (t1, y1) = self.hist_yaw[0], self.hist_yaw[-1]
            if t1 - t0 > 1e-4:
                self.wz_real = norm_ang(y1 - y0) / (t1 - t0)

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

        yaw = yaw_de(self.pose.pose.pose.orientation)
        erro = norm_ang(self.rumo_alvo - yaw)

        # ---- modo ré: reta, sem giro, e sem passar pela lei de rumo ----
        #
        # ⚠️ O PIVÔ TEM PRIORIDADE SOBRE A RÉ, e isso foi medido em 05-08.
        # A ré da decisão 009 dispara por SINTOMA ("não progrediu"), e durante
        # um pivô o robô legitimamente não progride — ele está girando no
        # lugar. Resultado com a ordem antiga (ré primeiro):
        #
        #     PIVÔ -45° -> RÉ -> RÉ -> PIVÔ DESISTIU
        #     PIVÔ +40° -> RÉ -> RÉ -> PIVÔ DESISTIU
        #     PIVÔ -47° -> (sem ré) -> pivô fechado
        #
        # A ré atropelava a manobra em 2 de 3 casos, e o pivô então acusava
        # "o robô não se mexeu (BO-3)" — alarme FALSO: ele mandava girar e
        # quem estava publicando era a ré. Recuar para consertar rumo também é
        # a manobra errada: recuar reto NÃO muda o rumo (medido em 29-07, 7ª
        # leva). Quem conserta rumo é o pivô; a ré serve para abrir geometria.
        if self.v_alvo < 0.0 and self.pivo is None:
            v, wz = comando_de_re(self.v_alvo, self.par['zona_morta'],
                                  self.par['margem_piso'], self.par['v_max'])
            self.get_logger().info(f'RÉ a {abs(v):.2f} m/s (manobra)',
                                   throttle_duration_sec=2.0)
            self.publica(v, wz)
            self.plantao(agora, v, wz)
            return

        # ---- PIVÔ: parar e virar no eixo, quando arcar não serve ----
        #
        # Medido em 05-08 com a pilha inteira: arcando, o robô realiza raio de
        # 0,82 m contra os 0,37 m que o Smac planeja, e o caminho realizado não
        # tem como coincidir com o planejado — o seguidor passa a corrigir um
        # erro que ele mesmo gera (1,35x de caminho, 0,4% de giro parado).
        #
        # A manobra é BANG-BANG por necessidade, não por escolha: a placa
        # entrega um `wz` só, então a única alavanca é quando cortar
        # (`lei_de_pivo.py`). Uma vez começada ela vai até o fim — trocar de
        # modo no meio deixaria o robô girando sem ninguém responsável pelo
        # corte, e a sobra é de ~113°.
        # Com velocidade ZERO pedida, arcar não existe: o robô só pode girar
        # parado. É o caso da fase 2 da chegada (o seguidor manda `v=0` e o
        # ângulo do objetivo). Sem esta condição o erro pequeno cairia na lei
        # de arco, que aplica piso de linear e ARRASTA o robô para fora do
        # ponto — exatamente o defeito de 27-07 que a decisão 006 evitou
        # cortando o rumo de chegada. Aqui o limiar é a própria tolerância do
        # pivô, porque abaixo dela não há manobra possível.
        parado = abs(self.v_alvo) < 1e-6
        precisa_pivo = abs(erro) > (self.par['pivo_tolerancia'] if parado
                                    else self.par['limiar_pivo'])
        if self.pivo is None and precisa_pivo:
            self.pivo = PivoPorCorte(
                a_dec=self.par['pivo_a_dec'],
                tolerancia=self.par['pivo_tolerancia'],
                wz_comando=self.par['wz_max'],
                max_pulsos=self.par['pivo_max_pulsos'],
                teto_tempo=self.par['pivo_teto_tempo'])
            self.get_logger().info(
                f'PIVÔ: {math.degrees(erro):+.0f}° de erro — parando para '
                f'virar no eixo (arcar daria raio de ~0,8 m)')

        if self.pivo is not None:
            # dt limitado a alguns ciclos: se a manobra ficar suspensa (outro
            # modo assumiu, o nó travou), o intervalo acumulado entraria de uma
            # vez nos cronômetros e a lei acusaria BO-3 sem ter comandado nada
            # naquele tempo. Foi assim que o alarme falso de 05-08 apareceu.
            dt = (0.0 if self.t_passo is None
                  else min(agora - self.t_passo, 5.0 / self.par['taxa']))
            self.t_passo = agora
            wz, estado = self.pivo.passo(erro, self.wz_real, dt)
            if estado == PRONTO:
                self.get_logger().info(
                    f'pivô fechado, sobrou {math.degrees(erro):+.1f}°')
                self.pivo = None
            elif estado == DESISTIU:
                # Nunca em silêncio: o motivo é o que separa "não deu" de
                # "parou sozinho e ninguém viu" (BO-3).
                self.get_logger().error(f'PIVÔ DESISTIU — {self.pivo.motivo}')
                self.pivo = None
            else:
                self.publica(0.0, wz)     # linear ZERO: é giro no eixo
                self.plantao(agora, 0.0, wz)
                return
        self.t_passo = agora

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
