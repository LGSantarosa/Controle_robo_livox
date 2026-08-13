#!/usr/bin/env python3
"""Freio LINEAR por contra-torque — quanto o robô ainda ANDA depois do corte.

    # robô num trecho reto com 3 m livres à frente:
    python3 tools/banco/freio_linear.py --sim \
        --csv docs/dados/AAAA-MM-DD-.../freio-linear.csv --malha-fechada 0.30 0.20 0.10

## Por que isto existe, e por que é o eixo que faltava

A 037 mediu e resolveu o freio de GIRO. O eixo LINEAR ficou de fora, e é ele
que estava batendo. Medido em 13-08 na corrida da porta gravada
(`docs/dados/2026-08-13-re-na-porta/volta-pra-sala.csv`):

    t=21,85 s   o reflexo ZERA a saída       folga 0,30 m
    t=22,31 s   o robô para                  folga 0,20 m
    ------------------------------------------------------
    andou +0,10 m com o comando em ZERO, e 0,20 < 0,2275 (meia-largura) = BATEU

O reflexo funcionou: cortou o comando a 0,30 m da parede. O que não existe é o
freio — a mesma retenção de `atraso_desliga` (0,52 s, medida no robô em 04-08)
que segura o giro segura a marcha, e 0,52 s a 0,30 m/s são 0,16 m de corpo
entrando na parede.

A física é idêntica à da 037 e a conclusão também: a placa entrega **um módulo
só** e não sabe desacelerar, então a única forma de tirar energia é **torque
contrário** — comandar a marcha oposta por um instante.

## O número que manda: QUANDO SOLTAR

Igual ao giro: a retenção vale para o contra-comando também. Soltar com o robô
já parado deixa 0,52 s de empurrão reverso sobrando, e ele sai andando para
trás — que é a ré que o dono não quer. Por isso o freio solta **cedo**, com o
robô ainda andando para a frente. `--malha-fechada` varre esse limiar, em m/s.

## O que cada corrida faz

    1. v = +v_cmd por `t_anda`      (a placa entrega o módulo dela: 0,298 m/s)
    2. v = -v_cmd até |v medido| no sentido de origem cair abaixo do limiar
    3. v = 0                        e espera assentar

E grava, por limiar:

    anda_util   quanto ele avançou na fase 1 (o que se paga pelo freio)
    SOBRA       quanto ele ainda avançou DEPOIS do último comando — o alvo é 0
    recuou      negativo em `sobra` significa que o freio inverteu a marcha

⚠️ Publica em `/key_vel` (prioridade 90 no mux, o canal do teclado), que fura o
reflexo de propósito — é bancada, não navegação. **Trecho livre, sempre**: o
robô sai do lugar de verdade nesta bancada, ao contrário da do giro, que gira
parado. Entre corridas ele volta sozinho para perto do ponto de partida, mas a
volta também arca (−0,098 1/m de ré contra −0,817 de frente), então confira a
deriva pelas colunas `x` e `y` de cada linha.

⚠️ `--sim` NÃO É DETALHE. Nó de bancada com relógio de parede num mundo em tempo
de simulação tem o comando descartado por velho e o robô não se mexe — sem erro
na tela, só zero. Foram quatro varreduras inválidas assim em 14-08.
"""
import argparse
import csv
import math
import sys
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Bancada(Node):
    """Mede deslocamento e velocidade pela POSE, nunca pelo campo `twist`.

    Mesma razão da bancada do giro: o `twist` publicado pelo Gazebo é ruidoso,
    e aqui a pergunta é "quantos centímetros de parede ele comeu", que só a
    pose responde.
    """

    def __init__(self, topico, sim):
        super().__init__('freio_linear',
                         parameter_overrides=[Parameter(
                             'use_sim_time', Parameter.Type.BOOL, bool(sim))])
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, topico, q)
        self.create_subscription(Odometry, '/Odometry', self.cb, q)
        self.pos = None
        self.yaw = None
        # Percurso ACUMULADO COM SINAL: projeção do passo no rumo do robô. O
        # módulo puro somaria a ré como se fosse avanço e a `sobra` nunca
        # poderia ficar negativa — perderíamos justamente o sinal de "o freio
        # passou do ponto e ele voltou andando".
        self.percurso = 0.0
        self.v_medido = 0.0
        self._ultimo = None
        self._t_ultimo = None

    def cb(self, msg):
        p = msg.pose.pose.position
        y = yaw_de(msg.pose.pose.orientation)
        agora = self.agora()
        if self._ultimo is not None:
            dx = p.x - self._ultimo[0]
            dy = p.y - self._ultimo[1]
            # sinal pela projeção no rumo ATUAL: andar de ré num robô que arca
            # não é "menos x", é "menos projeção"
            d = dx * math.cos(y) + dy * math.sin(y)
            self.percurso += d
            if self._t_ultimo is not None:
                dt = agora - self._t_ultimo
                if dt > 1e-4:
                    # mesmo filtro de 1a ordem da bancada do giro: a derivada
                    # da pose vem em degraus
                    self.v_medido = 0.7 * self.v_medido + 0.3 * (d / dt)
        self._ultimo = (p.x, p.y)
        self._t_ultimo = agora
        self.pos = (p.x, p.y)
        self.yaw = y

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def manda(self, v):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(v)
        self.pub.publish(m)

    def anda(self, v, dur):
        t0 = self.agora()
        while self.agora() - t0 < dur:
            self.manda(v)
            rclpy.spin_once(self, timeout_sec=0.02)

    def espera(self, dur):
        t0 = self.agora()
        while self.agora() - t0 < dur:
            self.manda(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)

    def freia(self, v_cmd, limiar, teto=3.0):
        """Contra-torque enquanto a marcha no sentido de origem passar do limiar.

        Devolve (tempo de freio, v no instante de soltar).

        ⚠️ SÓ SOLTA DEPOIS DO PICO, como no giro: a velocidade sobe DEPOIS do
        corte (a retenção da placa é um degrau que ainda está subindo quando o
        comando zera), então um laço que solta no primeiro `|v| <= limiar`
        solta na subida e não freia nada.
        """
        PICO_MIN = 0.12   # m/s — "a inércia já apareceu"; o patamar da placa
        #                   é 0,298 m/s, então 0,12 é ~40% dele
        t0 = self.agora()
        maior = 0.0
        sentido = 0.0
        while self.agora() - t0 < teto:
            v = self.v_medido
            if abs(v) > maior:
                maior = abs(v)
            if sentido == 0.0 and abs(v) > 0.05:
                sentido = math.copysign(1.0, v)
            # COMPONENTE NO SENTIDO ORIGINAL, não o módulo: depois que o freio
            # inverte a marcha o módulo volta a subir e o laço seguraria o
            # contra-torque até o teto (o erro que a 2ª varredura do giro fez).
            if maior >= PICO_MIN and v * sentido <= limiar:
                break
            alvo = sentido if sentido != 0.0 else math.copysign(1.0, v or 1.0)
            self.manda(-alvo * v_cmd)
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.agora() - t0, self.v_medido

    def retorna(self, alvo_percurso, v_cmd, teto=6.0):
        """Volta para perto de onde a corrida começou, para a próxima caber.

        Sem isto a varredura anda ~0,4 m por linha sempre no mesmo sentido e a
        quinta corrida acontece dentro da parede. Não é medida: é arrumação de
        bancada, e por isso usa velocidade e não tem critério fino.
        """
        t0 = self.agora()
        while self.agora() - t0 < teto:
            falta = alvo_percurso - self.percurso
            if abs(falta) < 0.08:
                break
            self.manda(math.copysign(v_cmd, falta))
            rclpy.spin_once(self, timeout_sec=0.02)
        self.espera(2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', required=True)
    ap.add_argument('--sim', action='store_true',
                    help='usa o relógio da simulação (OBRIGATÓRIO no Gazebo)')
    ap.add_argument('--topico', default='/key_vel',
                    help='canal do ensaio; /key_vel fura o reflexo (prio 90)')
    ap.add_argument('--v', type=float, default=0.5,
                    help='v comandado (a placa entrega 0,298 m/s de qualquer '
                         'jeito, entre 0,008 e 0,838 — decisão 020)')
    ap.add_argument('--t-anda', type=float, default=1.2,
                    help='[s] duração da fase de marcha')
    ap.add_argument('--malha-fechada', nargs='*', type=float,
                    default=[0.30, 0.25, 0.20, 0.15, 0.10],
                    help='[m/s] limiares de soltar o contra-torque')
    ap.add_argument('--assenta', type=float, default=3.0,
                    help='[s] espera depois do último comando')
    ap.add_argument('--sem-volta', action='store_true',
                    help='não retorna ao ponto de partida entre corridas')
    a = ap.parse_args()

    rclpy.init()
    n = Bancada(a.topico, a.sim)
    t0 = time.time()
    while n.pos is None and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    if n.pos is None:
        print('sem /Odometry — a pilha está de pé?')
        rclpy.shutdown()
        sys.exit(2)

    linhas = []
    print(f'{"limiar":>8} {"anda util":>10} {"freou por":>10} '
          f'{"v ao soltar":>12} {"SOBRA":>8}   (m, s, m/s)')
    for lim in a.malha_fechada:
        n.espera(1.5)
        base = n.percurso
        n.anda(a.v, a.t_anda)
        util = n.percurso - base
        dur, v_solta = n.freia(a.v, lim)
        antes = n.percurso
        n.espera(a.assenta)
        total = n.percurso - base
        sobra = n.percurso - antes
        print(f'{lim:8.2f} {util:10.3f} {dur:10.2f} '
              f'{v_solta:12.3f} {sobra:8.3f}')
        linhas.append({'limiar': lim, 'anda_util_m': round(util, 4),
                       'freou_s': round(dur, 3),
                       'v_ao_soltar': round(v_solta, 3),
                       'sobra_m': round(sobra, 4),
                       'total_m': round(total, 4),
                       'x': round(n.pos[0], 3), 'y': round(n.pos[1], 3)})
        if not a.sem_volta:
            n.retorna(base, a.v)

    with open(a.csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
        w.writeheader()
        w.writerows(linhas)
    print(f'\n-> {a.csv}')

    z = min(linhas, key=lambda r: abs(r['sobra_m']))
    print(f"\nmenor sobra: {z['sobra_m']:+.3f} m soltando em "
          f"{z['limiar']:.2f} m/s   (sem freio, a corrida de 13-08 sobrou "
          f"+0,100 m e isso bastou para bater)")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
