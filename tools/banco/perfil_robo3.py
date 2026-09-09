#!/usr/bin/env python3
"""Executa UM perfil seguro do robô 3 e grava todas as testemunhas.

Este instrumento não supõe que ``cmd_vel`` seja velocidade. Ele apenas aplica
um perfil curto e registra, no mesmo relógio, o que foi pedido, o que a placa
mandou às rodas, os encoders, o LIO e a odometria diferencial. Isso permite
identificar patamar, latência, retenção, derrapagem e assimetria sem herdar o
modelo do robô 2.

Perfis:

``parado``
    Não move. Mede deriva e ruído do LIO.
``suspenso``
    Pulsos curtos de frente, ré e giro para conferir placa, fiação e encoders
    com as rodas fora do chão.
``pulso``
    Repouso, comando constante por ``--liga`` e cauda sem comando.
``inversao``
    Repouso, ``(v,wz)``, pausa, ``(-v,-wz)`` e cauda. É o teste das bobas.

Ctrl-C, falha, perda de dado ou qualquer trava publicam zero repetidamente
antes da saída. A energia das rodas continua precisando de corte físico.
"""

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64


ASSENTA_S = 2.0
CAUDA_S = 3.0
BITOLA_ROBO3 = 0.3225
RAIO_ROBO3 = 0.0825


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def duracao_perfil(cfg):
    if cfg.perfil == 'parado':
        return cfg.dur
    if cfg.perfil == 'suspenso':
        # assenta + 4 pulsos de 0,35 s separados por 1,0 s + cauda
        return ASSENTA_S + 4 * 0.35 + 3 * 1.0 + CAUDA_S
    if cfg.perfil == 'pulso':
        return ASSENTA_S + cfg.liga + CAUDA_S
    if cfg.perfil == 'inversao':
        return ASSENTA_S + 2 * cfg.liga + cfg.pausa + CAUDA_S
    raise ValueError(cfg.perfil)


def comando_perfil(cfg, t):
    """Função pura da linha do tempo: devolve ``(fase, v, wz)``."""
    if cfg.perfil == 'parado':
        return 'parado', 0.0, 0.0

    if cfg.perfil == 'suspenso':
        t -= ASSENTA_S
        if t < 0:
            return 'assenta', 0.0, 0.0
        sequencia = [
            ('frente', 0.08, 0.0),
            ('re', -0.08, 0.0),
            ('giro_esquerda', 0.0, 0.15),
            ('giro_direita', 0.0, -0.15),
        ]
        for nome, v, wz in sequencia:
            if t < 0.35:
                return nome, v, wz
            t -= 0.35
            if nome != sequencia[-1][0]:
                if t < 1.0:
                    return 'pausa', 0.0, 0.0
                t -= 1.0
        return 'cauda', 0.0, 0.0

    t -= ASSENTA_S
    if t < 0:
        return 'assenta', 0.0, 0.0
    if cfg.perfil == 'pulso':
        if t < cfg.liga:
            return 'pulso', cfg.v, cfg.wz
        return 'cauda', 0.0, 0.0
    if cfg.perfil == 'inversao':
        if t < cfg.liga:
            return 'ida', cfg.v, cfg.wz
        t -= cfg.liga
        if t < cfg.pausa:
            return 'pausa', 0.0, 0.0
        t -= cfg.pausa
        if t < cfg.liga:
            return 'volta', -cfg.v, -cfg.wz
        return 'cauda', 0.0, 0.0
    raise ValueError(cfg.perfil)


class Perfil(Node):
    CAMPOS_ESCALARES = {
        'vel_esq': '/hoverboard/left_wheel/velocity',
        'vel_dir': '/hoverboard/right_wheel/velocity',
        'pos_esq': '/hoverboard/left_wheel/position',
        'pos_dir': '/hoverboard/right_wheel/position',
        'cmd_esq': '/hoverboard/left_wheel/cmd',
        'cmd_dir': '/hoverboard/right_wheel/cmd',
        'corrente_esq': '/hoverboard/left_wheel/dc_current',
        'corrente_dir': '/hoverboard/right_wheel/dc_current',
        'tensao': '/hoverboard/battery_voltage',
        'temperatura': '/hoverboard/temperature',
    }

    def __init__(self, cfg):
        super().__init__('perfil_robo3')
        self.cfg = cfg
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, cfg.topico, qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_pose, qos)
        self.create_subscription(
            Odometry, '/hoverboard_base_controller/odom', self.cb_odom, qos)

        self.escalares = {nome: None for nome in self.CAMPOS_ESCALARES}
        self.t_escalares = {nome: None for nome in self.CAMPOS_ESCALARES}
        for nome, topico in self.CAMPOS_ESCALARES.items():
            self.create_subscription(
                Float64, topico,
                lambda msg, n=nome: self.cb_escalar(n, msg), qos)

        self.pose = None
        self.odom = None
        self.t_pose = None
        self.t_odom = None
        self.pose_ant = None
        self.v_pose = 0.0
        self.wz_pose = 0.0
        self.v_hist = deque(maxlen=3)
        self.wz_hist = deque(maxlen=3)
        self.trajeto = 0.0
        self.origem = None
        self.linhas = []
        self.abortado = None
        self.violacoes = {'v': 0, 'wz': 0, 'roda': 0}

    def agora(self):
        return time.monotonic()

    def cb_escalar(self, nome, msg):
        self.escalares[nome] = float(msg.data)
        self.t_escalares[nome] = self.agora()

    def cb_pose(self, msg):
        t = self.agora()
        p = msg.pose.pose.position
        yaw = yaw_de(msg.pose.pose.orientation)
        atual = (t, p.x, p.y, p.z, yaw)
        if self.pose_ant is not None:
            ta, xa, ya, _za, yawa = self.pose_ant
            dt = t - ta
            if dt > 1e-4:
                dx, dy = p.x - xa, p.y - ya
                self.trajeto += math.hypot(dx, dy)
                # Projeção no corpo: ao contrário de hypot(), preserva a ré.
                v = (dx * math.cos(yawa) + dy * math.sin(yawa)) / dt
                wz = norm_ang(yaw - yawa) / dt
                self.v_hist.append(v)
                self.wz_hist.append(wz)
                self.v_pose = sorted(self.v_hist)[len(self.v_hist) // 2]
                self.wz_pose = sorted(self.wz_hist)[len(self.wz_hist) // 2]
        self.pose_ant = atual
        self.pose = msg
        self.t_pose = t

    def cb_odom(self, msg):
        self.odom = msg
        self.t_odom = self.agora()

    def publica(self, v, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(wz)
        self.pub.publish(msg)

    def zera(self):
        for _ in range(20):
            self.publica(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.01)

    def espera_pronto(self):
        limite = self.agora() + self.cfg.timeout_pronto
        while rclpy.ok() and self.agora() < limite:
            rclpy.spin_once(self, timeout_sec=0.05)
            tem_pose = self.pose is not None
            tem_atuador = (self.cfg.perfil == 'parado' or
                           self.pub.get_subscription_count() > 0)
            tem_encoder = (self.cfg.permitir_sem_encoders or
                           (self.escalares['vel_esq'] is not None and
                            self.escalares['vel_dir'] is not None))
            if tem_pose and tem_atuador and tem_encoder:
                self.origem = (self.pose.pose.pose.position.x,
                               self.pose.pose.pose.position.y)
                self.pose_ant = None
                self.trajeto = 0.0
                return True
        faltam = []
        if self.pose is None:
            faltam.append('/Odometry')
        if self.cfg.perfil != 'parado' and self.pub.get_subscription_count() == 0:
            faltam.append('ouvinte de cmd_vel')
        if (not self.cfg.permitir_sem_encoders and
                (self.escalares['vel_esq'] is None or
                 self.escalares['vel_dir'] is None)):
            faltam.append('encoders esquerdo/direito')
        self.abortado = 'pré-voo sem ' + ', '.join(faltam)
        return False

    def _viola(self, nome, condicao, limite, valor):
        self.violacoes[nome] = self.violacoes[nome] + 1 if condicao else 0
        if self.violacoes[nome] >= 2 and self.abortado is None:
            self.abortado = f'trava {nome}: {valor:.3f} > {limite:.3f}'

    def seguranca(self, agora):
        if self.t_pose is None or agora - self.t_pose > self.cfg.sem_pose:
            self.abortado = f'/Odometry calou por mais de {self.cfg.sem_pose:.1f} s'
            return
        p = self.pose.pose.pose.position
        afast = math.hypot(p.x - self.origem[0], p.y - self.origem[1])
        if afast > self.cfg.espaco:
            self.abortado = f'afastamento {afast:.2f} m excedeu {self.cfg.espaco:.2f} m'
            return
        if self.trajeto > self.cfg.trajeto_max:
            self.abortado = (f'trajeto {self.trajeto:.2f} m excedeu '
                              f'{self.cfg.trajeto_max:.2f} m')
            return
        self._viola('v', abs(self.v_pose) > self.cfg.v_real_max,
                    self.cfg.v_real_max, abs(self.v_pose))
        self._viola('wz', abs(self.wz_pose) > self.cfg.wz_real_max,
                    self.cfg.wz_real_max, abs(self.wz_pose))
        vr = max(abs(self.escalares['vel_esq'] or 0.0),
                 abs(self.escalares['vel_dir'] or 0.0))
        self._viola('roda', vr > self.cfg.roda_max,
                    self.cfg.roda_max, vr)

    @staticmethod
    def _valor(v):
        return '' if v is None else round(v, 6)

    def registra(self, t, fase, cv, cw):
        p = self.pose.pose.pose
        o = self.odom.pose.pose if self.odom else None
        ot = self.odom.twist.twist if self.odom else None
        agora = self.agora()
        linha = {
            't': round(t, 4), 'fase': fase,
            'cmd_v': round(cv, 5), 'cmd_wz': round(cw, 5),
            'x': round(p.position.x, 6), 'y': round(p.position.y, 6),
            'z': round(p.position.z, 6), 'yaw': round(yaw_de(p.orientation), 6),
            'v_pose': round(self.v_pose, 6), 'wz_pose': round(self.wz_pose, 6),
            'trajeto_pose': round(self.trajeto, 6),
            'pose_age': round(agora - self.t_pose, 4),
            'x_roda': self._valor(o.position.x if o else None),
            'y_roda': self._valor(o.position.y if o else None),
            'yaw_roda': self._valor(yaw_de(o.orientation) if o else None),
            'v_odom': self._valor(ot.linear.x if ot else None),
            'wz_odom': self._valor(ot.angular.z if ot else None),
        }
        for nome in self.CAMPOS_ESCALARES:
            linha[nome] = self._valor(self.escalares[nome])
        self.linhas.append(linha)

    def executa(self):
        if not self.espera_pronto():
            return False
        total = duracao_perfil(self.cfg)
        inicio = self.agora()
        periodo = 1.0 / self.cfg.taxa
        proximo = inicio
        while rclpy.ok():
            agora = self.agora()
            t = agora - inicio
            if t >= total or self.abortado:
                break
            rclpy.spin_once(self, timeout_sec=min(0.01, max(0.0, proximo - agora)))
            fase, cv, cw = comando_perfil(self.cfg, t)
            self.publica(cv, cw)
            self.seguranca(self.agora())
            self.registra(t, fase, cv, cw)
            proximo += periodo
            sobra = proximo - self.agora()
            if sobra > 0:
                time.sleep(min(sobra, periodo))
        return self.abortado is None

    def grava(self):
        os.makedirs(os.path.dirname(os.path.abspath(self.cfg.csv)), exist_ok=True)
        if self.linhas:
            with open(self.cfg.csv, 'w', newline='') as arq:
                w = csv.DictWriter(arq, fieldnames=list(self.linhas[0]))
                w.writeheader()
                w.writerows(self.linhas)
        meta = {
            'perfil': self.cfg.perfil,
            'v': self.cfg.v,
            'wz': self.cfg.wz,
            'liga_s': self.cfg.liga,
            'pausa_s': self.cfg.pausa,
            'bitola_m': self.cfg.bitola,
            'raio_roda_m': self.cfg.raio,
            'status': 'abortado' if self.abortado else 'concluido',
            'motivo': self.abortado or 'perfil completo',
            'amostras': len(self.linhas),
            'trajeto_m': self.trajeto,
            'topico_cmd': self.cfg.topico,
            'commit': subprocess.run(
                ['git', 'rev-parse', '--short', 'HEAD'], capture_output=True,
                text=True).stdout.strip(),
        }
        with open(os.path.splitext(self.cfg.csv)[0] + '.json', 'w') as arq:
            json.dump(meta, arq, indent=2, ensure_ascii=False)
            arq.write('\n')


def argumentos(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--perfil', required=True,
                    choices=['parado', 'suspenso', 'pulso', 'inversao'])
    ap.add_argument('--csv', required=True)
    ap.add_argument('--v', type=float, default=0.0)
    ap.add_argument('--wz', type=float, default=0.0)
    ap.add_argument('--liga', type=float, default=2.0)
    ap.add_argument('--pausa', type=float, default=1.0)
    ap.add_argument('--dur', type=float, default=20.0,
                    help='duração do perfil parado [s]')
    ap.add_argument('--bitola', type=float, default=BITOLA_ROBO3)
    ap.add_argument('--raio', type=float, default=RAIO_ROBO3)
    ap.add_argument('--topico', default='/hoverboard_base_controller/cmd_vel')
    ap.add_argument('--taxa', type=float, default=50.0)
    ap.add_argument('--timeout-pronto', type=float, default=12.0)
    ap.add_argument('--sem-pose', type=float, default=1.0)
    ap.add_argument('--permitir-sem-encoders', action='store_true')
    ap.add_argument('--espaco', type=float, default=2.5)
    ap.add_argument('--trajeto-max', type=float, default=5.0)
    ap.add_argument('--v-real-max', type=float, default=0.8)
    ap.add_argument('--wz-real-max', type=float, default=3.0)
    ap.add_argument('--roda-max', type=float, default=18.0,
                    help='corta se algum encoder exceder este |rad/s|')
    return ap.parse_args(argv)


def main(argv=None):
    cfg = argumentos(argv)
    rclpy.init()
    no = Perfil(cfg)
    ok = False
    try:
        ok = no.executa()
    except KeyboardInterrupt:
        no.abortado = 'interrompido pelo operador'
    except Exception as exc:
        no.abortado = f'exceção: {exc}'
        raise
    finally:
        no.zera()
        no.grava()
        print(f'{len(no.linhas)} amostras -> {cfg.csv}')
        print('STATUS:', no.abortado or 'perfil completo')
        no.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 2


if __name__ == '__main__':
    sys.exit(main())
