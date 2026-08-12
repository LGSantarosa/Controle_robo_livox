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
            # ⚠️ 1,0 desde 05-08, era 1,5. Com 1,5 o lookahead dava 0,555 m —
            # mais da metade do vão da porta (0,90 m). A cenoura caía DEPOIS
            # da porta e o robô cortava a quina para alcançá-la: o pior ponto
            # das quatro corridas de 05-08 caiu sempre na ombreira, com folga
            # de 0,29 m contra os 0,314 do corpo.
            #
            # 1,5 vinha de um seguidor que NÃO pivotava e precisava de mira
            # longa para não oscilar. Este pivota (fatia 3 da 011) e tem o
            # compensador cancelando o arco: mira curta deixou de custar
            # oscilação. Com 1,0 o lookahead vira 0,37 m — cabe dentro do vão.
            ('lookahead_fator', 1.0),
            ('lookahead_piso', 0.30),
            ('v_max', 0.5),
            ('a_lin', 0.3),
            ('wz_max', 1.0),
            # Piso de linear da MOVIMENTAÇÃO. Não é usado para comandar — ela
            # se defende sozinha — mas sem ele não dá para saber se o raio de
            # chegada pedido é possível. TEM QUE BATER com o que a movimentação
            # calcula: zona_morta + wz_max·bitola/2 + margem.
            #
            # 0,0178 + 1,0·0,270/2 + 0,05 = 0,203, com a zona morta MEDIDA
            # (decisão 020). Era 0,335, que vinha do chute de 0,15 — e o "tem
            # que bater" acima era só comentário: os dois arquivos andaram
            # separados por doze dias porque nada conferia. Agora confere
            # (`test_o_piso_do_seguidor_sai_da_movimentacao`).
            ('v_piso', 0.203),
            ('raio_chegada', 0.25),
            # --- chegada em DUAS FASES (05-08) ---
            # A decisão 006 tirou o rumo de chegada com esta razão: "girar
            # depois de chegar arrasta o robô para fora do ponto (0,06 m
            # viraram 0,27 m em 27-07)". Aquilo era verdade para um robô que
            # só sabia ARCAR: girar significava andar em círculo.
            #
            # Com o pivô (fatia 3 da 011) girar parado custa v=0 — o robô não
            # sai do lugar. Então a chegada volta a ter duas fases: vai até o
            # ponto reto, e SÓ LÁ acerta o ângulo. É também o que torna o
            # Theta* utilizável: o plano não precisa mais chegar apontado.
            ('aponta_no_fim', True),
            # Folga sobre a tolerância do pivô (~6°): pedir mais fino que a
            # manobra consegue entregar é laço que não fecha.
            ('tolerancia_rumo_final', 0.15),
            # --- ré por gatilho (decisão 009) ---
            #
            # ⚠️ DESLIGADA POR PADRÃO desde 05-08, e isto revisa a premissa da
            # própria 009. Ela escolheu ré-por-sintoma com estas palavras: "o
            # que sustenta a ré por gatilho é o PIVÔ, e o robô 1 pivota. O robô
            # 2 não, com os parâmetros de hoje." Essa premissa CAIU: com a lei
            # do pivô (fatia 3 da 011) o robô 2 pivota — 3 manobras iniciadas,
            # 3 fechadas, resíduo de 0,0°/0,0°/0,1°.
            #
            # E a ré nunca resolveu o problema que a disparava: ela dispara por
            # "não progrediu", que num robô mal-apontado significa RUMO errado
            # — e recuar RETO não muda rumo (medido em 29-07, 7ª leva:
            # "recuar NÃO salva o plano, hipótese testada e derrubada").
            # Pior, ela atropelava o pivô: em 05-08, 2 de 3 manobras morreram
            # assim, com o pivô acusando BO-3 por não conseguir girar.
            #
            # Fica no código, com orçamento e voz, para o caso que só ela
            # resolve: geometria fechada de verdade (nariz contra a parede,
            # sem espaço para girar). Ligar com `re_habilitada:=true`.
            ('re_habilitada', False),
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
        self.rumo_objetivo = None

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
        # O ângulo de chegada sai do ÚLTIMO ponto do plano, e não de uma
        # assinatura própria de `/goal_pose`.
        #
        # A primeira versão assinava `/goal_pose` e ERA FRÁGIL: nó que publica
        # o alvo e sai pode ser descoberto pelo `bt_navigator` e não por este
        # nó, e então o robô navega para o ponto certo e gira para o ângulo do
        # alvo ANTERIOR. Aconteceu na demonstração de 05-08 — chegou a 0,09 m
        # do ponto com 98° de erro, e o log mostrava "ângulo acertado" porque
        # ele acertou o objetivo velho.
        #
        # Ler do plano elimina a corrida: sem plano este nó não faz nada
        # mesmo, então a informação chega junto com o trabalho.
        #
        # ⚠️ São só os pontos INTERMEDIÁRIOS do Theta* que vêm com orientação
        # zerada (29-07). O ÚLTIMO carrega o rumo pedido — conferido em 05-08:
        # alvo de +45,0° e último ponto do plano com +45,0°.
        self.rumo_objetivo = yaw_de(msg.poses[-1].pose.orientation)
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
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        rumo = yaw_de(self.pose.pose.pose.orientation)
        objetivo = self.plano[-1]
        dist = math.hypot(objetivo[0] - x, objetivo[1] - y)

        # ⚠️ A CHEGADA VEM ANTES DO FRESCOR DO PLANO, e a ordem é o conserto de
        # 05-08. O Nav2 declara `Goal succeeded` assim que o robô entra no raio
        # e PARA de replanejar; o plano vence 2 s depois. Com a checagem de
        # plano velho antes desta, o seguidor entrava em `parado (plano velho)`
        # e ficava trancado lá — nunca alcançava a fase de apontar, e o robô
        # parava no ponto com o ângulo errado. Medido: 2 de 3 alvos chegaram a
        # 0,10–0,20 m do ponto com 84–90° de erro de rumo.
        if chegou(dist, self.par['raio_chegada']) or self.estado == 'apontando':
            if self.aponta(rumo):
                return
            self.para('chegou')
            return

        if (self.t_plano is not None
                and t - self.t_plano > self.par['timeout_plano']):
            self.para('plano velho')
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

    def aponta(self, rumo):
        """Fase 2 da chegada: no ponto, acerta o ângulo. Devolve True se ainda
        está trabalhando nisso.

        Quem gira é o PIVÔ da movimentação (v=0), então o robô não sai do
        lugar — é isso que torna esta fase possível sem repetir o defeito de
        27-07, quando girar depois de chegar arrastava 0,06 m para 0,27 m.
        """
        if not self.par['aponta_no_fim'] or self.rumo_objetivo is None:
            return False
        erro = math.atan2(math.sin(self.rumo_objetivo - rumo),
                          math.cos(self.rumo_objetivo - rumo))
        if abs(erro) <= self.par['tolerancia_rumo_final']:
            if self.estado == 'apontando':
                self.get_logger().info(
                    f'ângulo acertado: {math.degrees(erro):+.1f}° do pedido')
            return False
        if self.estado != 'apontando':
            self.get_logger().info(
                f'no ponto — girando {math.degrees(erro):+.0f}° para acertar '
                f'o ângulo pedido')
            self.estado = 'apontando'
        # Velocidade ZERO: é o pivô que responde por isto.
        self.publica(self.rumo_objetivo, 0.0)
        return True

    def entra_na_re(self, t, x, y, dist):
        if not self.par['re_habilitada']:
            # Quem conserta rumo agora é o pivô, lá na movimentação. Falar uma
            # vez a cada 5 s é o suficiente: se o robô ficar de fato emperrado
            # com a ré desligada, isto é a pista.
            self.get_logger().warn(
                f'sem progresso a {dist:.2f} m do objetivo — ré DESLIGADA '
                '(o pivô responde por rumo desde 05-08); se ele não sair '
                'daqui, a geometria é fechada e a ré precisa voltar',
                throttle_duration_sec=5.0)
            self.progresso.reinicia()
            return
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
