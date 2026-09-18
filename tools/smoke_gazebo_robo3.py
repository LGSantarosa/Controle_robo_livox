#!/usr/bin/env python3
"""Smoke test do robô 3 no Gazebo: a CADEIA DE SOFTWARE sobe inteira?

Chamado pelo `bin/smoke-gazebo-robo3`, com o simulador já de pé. Escuta por
uma janela fixa e classifica seis itens em PASSOU / FALHOU / INCONCLUSIVO:

    1. modelo assentado     (/Odometry: z, roll, pitch, e parado sem comando)
    2. /livox/pontos        (nuvem chega, ~10 Hz, não vazia)
    3. /scan                (fatia chega, ~10 Hz, com alcance finito)
    4. /Odometry            (~50 Hz, odom -> base_link)
    5. TF                   (odom->base_link, base_link->livox_frame e rodas)
    6. controladores ativos (joint_state_broadcaster e hoverboard_base_controller)

A taxa é medida pelo carimbo do cabeçalho (tempo de SIMULAÇÃO), não pelo
relógio da parede: se o Gazebo rodar a 0,5× do tempo real, a nuvem sai a 5 Hz
na parede e continua certa. O fator de tempo real vai junto no resultado.

🔴 O QUE ESTE TESTE NÃO VALIDA: bitola, geometria, massas, curvatura, zona
morta, dinâmica da placa, nem o `frente:=-1.0` — este último nem está na
cadeia do simulador (quem o aplica é o `cmd_vel_to_wheels`, e o sim usa o
`diff_drive_controller`). O Gazebo usa os números que nós demos: concordar
consigo mesmo não é evidência sobre o robô.

    python3 tools/smoke_gazebo_robo3.py <pasta_de_saida> [janela_s]
"""
import csv
import math
import os
import sys
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener

PASSOU, FALHOU, INCONC = 'PASSOU', 'FALHOU', 'INCONCLUSIVO'

# Assentado: base_link é o chão sob o eixo das motoras (URDF, C8), então
# assentado é z ≈ 0 e corpo nivelado. Tolerâncias largas de propósito: isto é
# smoke, não medida — o que se caça é robô afundado, tombado ou quicando.
Z_TOL = 0.010          # m
INCLINACAO_TOL = 2.0   # graus, roll e pitch
DERIVA_TOL = 0.005     # m, quanto ele pode andar sozinho sem comando na janela
SPAWN_Z = 0.05         # o `-z` do spawn no sim_robo3.launch.py

TFS = [('odom', 'base_link'), ('base_link', 'livox_frame'),
       ('base_link', 'left_wheel'), ('base_link', 'right_wheel')]
CONTROLADORES = ['joint_state_broadcaster', 'hoverboard_base_controller']


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def rpy(q):
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x ** 2 + q.y ** 2))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    return math.degrees(roll), math.degrees(pitch)


class Escuta(Node):
    def __init__(self):
        super().__init__('smoke_gazebo_robo3',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.nuvem, self.scan, self.odom = [], [], []
        self.create_subscription(PointCloud2, '/livox/pontos',
                                 lambda m: self.nuvem.append(m), qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan',
                                 lambda m: self.scan.append(m), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry',
                                 lambda m: self.odom.append(m), 10)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        # O SERVIÇO, e não o `ros2 control list_controllers`: o CLI vem do
        # `ros2controlcli`, que não está instalado no PC de dev (18-09). Pela
        # linha de comando o erro virava "ausente" e o item dava falso vermelho.
        self.cli_controladores = self.create_client(
            ListControllers, '/controller_manager/list_controllers')

    def limpa(self):
        self.nuvem.clear()
        self.scan.clear()
        self.odom.clear()


def taxa(msgs):
    if len(msgs) < 2:
        return None
    dt = stamp_s(msgs[-1]) - stamp_s(msgs[0])
    return (len(msgs) - 1) / dt if dt > 0 else None


def confere_taxa(nome, msgs, alvo, minimo):
    t = taxa(msgs)
    if not msgs:
        return FALHOU, f'{nome}: nenhuma mensagem'
    if len(msgs) < minimo or t is None:
        return INCONC, f'{nome}: só {len(msgs)} mensagens'
    if not (0.7 * alvo <= t <= 1.3 * alvo):
        return FALHOU, f'{t:.1f} Hz (esperado ~{alvo:.0f})'
    return PASSOU, f'{t:.1f} Hz'


def item_assentado(odom):
    if len(odom) < 10:
        return INCONC, f'só {len(odom)} amostras de /Odometry'
    zs = [m.pose.pose.position.z for m in odom]
    inc = [rpy(m.pose.pose.orientation) for m in odom]
    p0, p1 = odom[0].pose.pose.position, odom[-1].pose.pose.position
    deriva = math.hypot(p1.x - p0.x, p1.y - p0.y)
    z_med = sum(zs) / len(zs)
    z_var = max(zs) - min(zs)
    roll = max(abs(r) for r, _ in inc)
    pitch = max(abs(p) for _, p in inc)
    txt = (f'z={z_med:+.4f} m (variou {z_var * 1000:.1f} mm), '
           f'|roll|max={roll:.2f}°, |pitch|max={pitch:.2f}°, '
           f'deriva xy={deriva * 1000:.1f} mm')
    if z_var > 0.002 or deriva > DERIVA_TOL:
        return FALHOU, txt + ' — não está parado'
    if roll > INCLINACAO_TOL or pitch > INCLINACAO_TOL:
        return FALHOU, txt + ' — inclinado'
    if abs(z_med) <= Z_TOL:
        return PASSOU, txt
    if abs(z_med + SPAWN_Z) <= Z_TOL:
        # O OdometryPublisher pode estar dando a pose RELATIVA ao spawn. Aí z
        # não diz nada sobre o chão, e a palavra final é do olho do dono.
        return INCONC, txt + ' — z parece relativo ao spawn; confirmar na tela'
    return FALHOU, txt + ' — altura errada'


def item_nuvem(nuvem):
    v, txt = confere_taxa('/livox/pontos', nuvem, 10.0, 5)
    if v != PASSOU:
        return v, txt
    pts = nuvem[-1].width * nuvem[-1].height
    frame = nuvem[-1].header.frame_id
    txt += f', {pts} pontos, frame {frame}'
    if pts == 0:
        return FALHOU, txt + ' — nuvem vazia'
    if frame != 'livox_frame':
        return FALHOU, txt + ' — frame errado'
    return PASSOU, txt


def item_scan(scan):
    v, txt = confere_taxa('/scan', scan, 10.0, 5)
    if v != PASSOU:
        return v, txt
    ult = scan[-1]
    finitos = sum(1 for r in ult.ranges if math.isfinite(r))
    txt += f', {finitos}/{len(ult.ranges)} feixes com alcance finito, frame {ult.header.frame_id}'
    if ult.header.frame_id != 'base_link':
        return FALHOU, txt + ' — frame errado'
    if finitos < 10:
        # A pista tem paredes de 0,6 m dentro da fatia (0,15–1,00 m): quase
        # tudo devia voltar finito. Pouco finito é fatia ou pose errada.
        return FALHOU, txt + ' — fatia não enxerga as paredes'
    return PASSOU, txt


def item_odometria(odom):
    v, txt = confere_taxa('/Odometry', odom, 50.0, 20)
    if v != PASSOU:
        return v, txt
    m = odom[-1]
    txt += f', {m.header.frame_id} -> {m.child_frame_id}'
    if (m.header.frame_id, m.child_frame_id) != ('odom', 'base_link'):
        return FALHOU, txt + ' — frames errados'
    return PASSOU, txt


def item_tf(no):
    faltam, ok = [], []
    for pai, filho in TFS:
        if no.tf.can_transform(pai, filho, Time()):
            ok.append(f'{pai}->{filho}')
        else:
            faltam.append(f'{pai}->{filho}')
    if faltam:
        return FALHOU, 'faltam: ' + ', '.join(faltam)
    return PASSOU, ', '.join(ok)


def item_controladores(no):
    if not no.cli_controladores.wait_for_service(timeout_sec=10.0):
        return INCONC, '/controller_manager/list_controllers não apareceu em 10 s'
    fut = no.cli_controladores.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(no, fut, timeout_sec=10.0)
    if not fut.done() or fut.result() is None:
        return INCONC, '/controller_manager/list_controllers não respondeu em 10 s'
    estado = {c.name: c.state for c in fut.result().controller}
    txt = ', '.join(f'{c}={estado.get(c, "ausente")}' for c in CONTROLADORES)
    if all(estado.get(c) == 'active' for c in CONTROLADORES):
        return PASSOU, txt
    return FALHOU, txt


def main():
    saida = sys.argv[1]
    janela = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
    rclpy.init()
    no = Escuta()

    # Espera o robô existir (primeira /Odometry), dá 3 s para assentar e só
    # então começa a janela — o quique do spawn não é defeito.
    t0 = time.monotonic()
    while not no.odom and time.monotonic() - t0 < 90:
        rclpy.spin_once(no, timeout_sec=0.2)
    if not no.odom:
        print('   ⚠️  90 s sem /Odometry — o robô não apareceu; medindo mesmo assim')
    fim = time.monotonic() + 3.0
    while time.monotonic() < fim:
        rclpy.spin_once(no, timeout_sec=0.1)

    no.limpa()
    parede0, sim0 = time.monotonic(), None
    while time.monotonic() - parede0 < janela:
        rclpy.spin_once(no, timeout_sec=0.1)
        if sim0 is None and no.odom:
            sim0 = stamp_s(no.odom[0])
    parede = time.monotonic() - parede0
    rtf = ((stamp_s(no.odom[-1]) - sim0) / parede) if (sim0 is not None and no.odom) else None

    itens = [
        ('1 modelo assentado', *item_assentado(no.odom)),
        ('2 /livox/pontos', *item_nuvem(no.nuvem)),
        ('3 /scan', *item_scan(no.scan)),
        ('4 /Odometry', *item_odometria(no.odom)),
        ('5 TF', *item_tf(no)),
        ('6 controladores ativos', *item_controladores(no)),
    ]

    icone = {PASSOU: '🟢', FALHOU: '🔴', INCONC: '🟡'}
    print(f'   janela: {parede:.1f} s de parede, fator de tempo real '
          f'{rtf:.2f}' if rtf else f'   janela: {parede:.1f} s de parede, fator de tempo real ?')
    for nome, v, txt in itens:
        print(f'   {icone[v]} {nome:<24} {v:<13} {txt}')

    with open(os.path.join(saida, 'resultado.csv'), 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['item', 'veredito_auto', 'detalhe', 'veredito_dono'])
        for nome, v, txt in itens:
            w.writerow([nome, v, txt, ''])
        w.writerow(['fator_tempo_real', '', f'{rtf:.3f}' if rtf else '?', ''])

    no.destroy_node()
    rclpy.shutdown()
    return 0 if all(v == PASSOU for _, v, _ in itens) else 1


if __name__ == '__main__':
    sys.exit(main())
