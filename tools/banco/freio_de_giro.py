#!/usr/bin/env python3
"""Freio de giro por CONTRA-PULSO — quanto tempo de giro contrário mata a inércia.

    # robô num ponto LIVRE (ele gira no lugar, não anda):
    python3 tools/banco/freio_de_giro.py --csv docs/dados/AAAA-MM-DD-.../freio.csv

## Por que isto existe

Esta máquina **não tem freio**. Zerar o comando não para nada: a placa segura a
saída cheia por `atraso_desliga` (0,52 s medido no robô em 04-08) e o robô varre
93–101° DEPOIS do corte — número que não depende do `wz` do corte (decisão 023).

O dono, vendo o robô querer girar 90° e entregar 180°: *"ele chega no 90, mas
chega rápido, aí solta o motor, mas a inércia joga ele mais 90 graus até parar
de verdade"*. Está certo, e a conclusão dele também: *"se o freio for feito ele
não precisa dar essa ré que hoje ele dá pra passar a porta"*.

A única forma de tirar energia deste atuador é **torque contrário**: comandar o
giro oposto por um instante. Mas a retenção vale para o contra-pulso TAMBÉM, e
por isso existe uma duração ótima — curta demais não mata a inércia, longa
demais e ele volta girando para o outro lado. Não dá para calcular contra um
modelo que já errou duas vezes neste projeto; mede-se.

## O que cada corrida faz

    1. wz = +wz_cmd por `t_giro`      (a placa entrega o módulo dela, sempre o mesmo)
    2. wz = -wz_cmd por T             <- o CONTRA-PULSO, a variável do ensaio
    3. wz = 0                         e espera assentar

E grava, por T:

    giro_total   quanto o robô varreu do início ao fim
    sobra        quanto ele ainda varreu DEPOIS do último comando (o alvo é 0)
    giro_util    quanto ele varreu durante a fase 1 (para saber o que se paga)

⚠️ Publica em `/key_vel` — o canal do teclado, prioridade 90 no mux, que fura o
reflexo de propósito. É bancada, não navegação: o robô fica sem a guarda de
colisão durante o ensaio. Ponto livre, sempre.

⚠️ NÃO publique em `/cmd_vel_bruto` com a pilha de pé: o `compensador_rumo`
publica lá continuamente e os dois se intercalam. Foi assim que a 1ª leva de
14-08 mediu 0,1° de giro na fase comandada — número impossível, e o sinal de
que o ensaio estava brigando com a cadeia em vez de dirigi-la.
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bancada(Node):
    def __init__(self, topico, sim):
        # 🔴 `use_sim_time` NÃO É DETALHE (14-08). Nó de bancada carimbando
        # relógio de PAREDE num mundo em tempo de SIMULAÇÃO tem o comando
        # descartado por velho, e o robô simplesmente não se mexe. Foram QUATRO
        # varreduras inválidas por isto — uma delas eu cheguei a publicar como
        # resultado. O sintoma é traiçoeiro: não dá erro, dá zero.
        super().__init__('freio_de_giro',
                         parameter_overrides=[Parameter(
                             'use_sim_time', Parameter.Type.BOOL, bool(sim))])
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, topico, q)
        self.create_subscription(Odometry, '/Odometry', self.cb, q)
        self.yaw = None
        self.pos = None
        # Yaw ACUMULADO: sem isto um giro de 200° some no wrap de ±180°.
        self.acumulado = 0.0
        self._ultimo = None
        # wz MEDIDO, derivado da pose. Nunca do campo `twist`: o do publicador
        # do Gazebo é ruidoso e o `ensaio.py` já registrou isso.
        self.wz_medido = 0.0
        self._t_ultimo = None

    def cb(self, msg):
        y = yaw_de(msg.pose.pose.orientation)
        agora = self.agora()
        if self._ultimo is not None:
            d = wrap(y - self._ultimo)
            self.acumulado += d
            if self._t_ultimo is not None:
                dt = agora - self._t_ultimo
                if dt > 1e-4:
                    # filtro de 1a ordem: a derivada da pose é degrau a degrau
                    self.wz_medido = 0.7 * self.wz_medido + 0.3 * (d / dt)
        self._ultimo = y
        self._t_ultimo = agora
        self.yaw = y
        self.pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def manda(self, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.angular.z = float(wz)
        self.pub.publish(m)

    def gira(self, wz, dur, taxa=20.0):
        """Mantém o comando por `dur` segundos, publicando a `taxa` Hz."""
        t0 = self.agora()
        while self.agora() - t0 < dur:
            self.manda(wz)
            rclpy.spin_once(self, timeout_sec=0.02)

    def espera(self, dur):
        t0 = self.agora()
        while self.agora() - t0 < dur:
            self.manda(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)


    def freia(self, wz_cmd, limiar, teto=3.0):
        """FREIO DE MALHA FECHADA: contra-torque enquanto |wz| for alto.

        Comanda o giro CONTRÁRIO ao sentido medido e solta quando `|wz_medido|`
        cai abaixo de `limiar`. Soltar em zero seria tarde demais — a placa
        segura o contra-comando por `atraso_desliga` (0,52 s) depois de soltar,
        e o robô inverteria o giro. `limiar` é exatamente esse "quanto antes".

        Devolve (tempo de freio, wz no instante de soltar).
        """
        # ⚠️ SÓ SOLTA DEPOIS DO PICO. O `wz` sobe DEPOIS do corte (medido no
        # robô real em 06-08: pico 2,5x o comandado, `t_parar` 1,9 s), então um
        # laço que solta no primeiro `|wz| <= limiar` solta na SUBIDA e não freia
        # nada — foi o que a 1ª varredura de malha fechada fez, com `freou por
        # 0,00 s` em toda linha. `PICO_MIN` é o "a inércia já apareceu".
        PICO_MIN = 0.6   # o pico medido no Gazebo é ~1,1 rad/s (perfil de 14-08)
        t0 = self.agora()
        maior = 0.0
        sentido = 0.0
        while self.agora() - t0 < teto:
            wz = self.wz_medido
            if abs(wz) > maior:
                maior = abs(wz)
            if sentido == 0.0 and abs(wz) > 0.2:
                sentido = math.copysign(1.0, wz)
            # ⚠️ COMPONENTE NO SENTIDO ORIGINAL, não o módulo. Testar `|wz|`
            # nunca solta: depois que o freio inverte o giro, o módulo volta a
            # subir e o laço segura o contra-torque até o teto — foi o que a 2ª
            # varredura fez, com −300° de giro total. Com o sinal, `limiar`
            # significa "ainda girando para o lado de origem a esta taxa".
            if maior >= PICO_MIN and wz * sentido <= limiar:
                break
            # o contra-torque segue o sentido do PICO, não o do instante: perto
            # do zero o sinal medido oscila e o freio ficaria batendo palma
            alvo = sentido if sentido != 0.0 else math.copysign(1.0, wz or 1.0)
            self.manda(-alvo * wz_cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.agora() - t0, self.wz_medido


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--sim', action='store_true',
                    help='usa o relógio da simulação (OBRIGATÓRIO no Gazebo)')
    ap.add_argument('--topico', default='/key_vel',
                    help='canal do ensaio. `/key_vel` (prioridade 90 no mux, o '
                         'do teclado) é o certo com a pilha DE PÉ: publicar em '
                         '/cmd_vel_bruto briga com o compensador_rumo, que '
                         'publica lá o tempo todo — a 1ª leva de 14-08 saiu '
                         'com giro de 0,1° na fase comandada por causa disso')
    ap.add_argument('--wz', type=float, default=1.0,
                    help='wz comandado (a placa entrega o módulo dela)')
    ap.add_argument('--t-giro', type=float, default=0.6,
                    help='[s] duração da fase de giro')
    ap.add_argument('--contra', nargs='*', type=float,
                    default=[0.0, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40],
                    help='[s] durações de contra-pulso a varrer')
    ap.add_argument('--assenta', type=float, default=3.0,
                    help='[s] espera depois do último comando')
    ap.add_argument('--malha-fechada', nargs='*', type=float, default=None,
                    help='[rad/s] varre LIMIARES de soltar o contra-torque, em '
                         'vez de durações fixas. É o freio de verdade: '
                         'contra-torque enquanto |wz medido| passar do limiar')
    a = ap.parse_args()

    rclpy.init()
    n = Bancada(a.topico, a.sim)
    t0 = time.time()
    while n.yaw is None and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.yaw is None:
        print('sem /Odometry — a pilha está de pé?')
        rclpy.shutdown()
        sys.exit(2)

    linhas = []
    g = math.degrees

    if a.malha_fechada is not None:
        print(f'{"limiar":>8} {"giro total":>11} {"freou por":>10} '
              f'{"wz ao soltar":>13} {"SOBRA":>8}')
        for lim in a.malha_fechada:
            n.espera(1.5)
            ini = n.acumulado
            n.gira(a.wz, a.t_giro)
            # espera a inércia APARECER: o pico vem depois do corte (medido no
            # robô real em 06-08, t_parar de 1,9 s e pico 2,5x o comandado)
            dur, wz_solta = n.freia(a.wz, lim)
            antes = n.acumulado
            n.espera(a.assenta)
            total = n.acumulado - ini
            sobra = n.acumulado - antes
            print(f'{lim:8.2f} {g(total):11.1f} {dur:10.2f} '
                  f'{wz_solta:13.2f} {g(sobra):8.1f}')
            linhas.append({'limiar': lim, 'giro_total_deg': round(g(total), 2),
                           'freou_s': round(dur, 3),
                           'wz_ao_soltar': round(wz_solta, 3),
                           'sobra_deg': round(g(sobra), 2),
                           'x': round(n.pos[0], 3), 'y': round(n.pos[1], 3)})
        with open(a.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
            w.writeheader()
            w.writerows(linhas)
        print(f'\n-> {a.csv}')
        z = min(linhas, key=lambda r: abs(r['sobra_deg']))
        print(f"\nmenor sobra: {z['sobra_deg']:+.1f}° soltando em "
              f"{z['limiar']:.2f} rad/s")
        rclpy.shutdown()
        return

    print(f'{"T contra":>9} {"giro fase 1":>12} {"giro total":>11} '
          f'{"SOBRA":>8}   (graus)')
    for T in a.contra:
        n.espera(1.0)
        ini = n.acumulado
        n.gira(a.wz, a.t_giro)
        fase1 = n.acumulado - ini
        if T > 0.0:
            n.gira(-a.wz, T)
        antes_de_soltar = n.acumulado
        n.espera(a.assenta)
        total = n.acumulado - ini
        sobra = n.acumulado - antes_de_soltar
        print(f'{T:9.2f} {g(fase1):12.1f} {g(total):11.1f} {g(sobra):8.1f}')
        linhas.append({'t_contra': T, 'giro_fase1_deg': round(g(fase1), 2),
                       'giro_total_deg': round(g(total), 2),
                       'sobra_deg': round(g(sobra), 2),
                       'x': round(n.pos[0], 3), 'y': round(n.pos[1], 3)})

    with open(a.csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
        w.writeheader()
        w.writerows(linhas)
    print(f'\n-> {a.csv}')

    zero = min(linhas, key=lambda r: abs(r['sobra_deg']))
    print(f"\nmenor sobra: {zero['sobra_deg']:+.1f}° com contra-pulso de "
          f"{zero['t_contra']:.2f} s")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
