#!/usr/bin/env python3
"""Bancada de Gazebo: uma corrida só, várias manobras, CSV para ler depois.

Complementa o `ensaio.py`, não o substitui. O `ensaio.py` caracteriza a
máquina em malha ABERTA (quanto ela aguenta) e roda igual no robô real. Esta
bancada é do simulador e responde outra pergunta: **a pilha montada obedece?**

    python3 corrida_gazebo.py --perfil sim
    python3 corrida_gazebo.py --perfil real

Sobe o Gazebo headless, roda as duas fases em sequência dentro da MESMA
simulação e derruba tudo no fim.

FASE A — aferição em malha aberta.
    Publica direto em `/hoverboard_base_controller/cmd_vel`, contornando a
    placa fingida de propósito: sem zona morta no caminho, o que sobra é só a
    conversão comando→roda. Mede REALIZADO ÷ COMANDADO. Esta fase existe por
    causa da trena de 2026-07-29: a bitola aparece em quatro arquivos, e se
    duas camadas discordarem a razão foge de 1. É o cheque de que a medida
    chegou inteira na pilha.

FASE B — roteiro em malha fechada.
    Com a navegação de pé, manda cinco objetivos em `/goal_pose` e vê se o
    robô chega: reta, 90° de lado, 180° para trás, um ponto PERTO E DE LADO
    (o caso que exige a ré da decisão 007) e a volta à origem.

O QUE O CSV GRAVA, e por quê:
    `/cmd_vel_bruto` é o que o controlador PEDIU; o cmd_vel do controlador de
    tração é o que a placa fingida DEIXOU PASSAR. Gravar os dois é o que torna
    a zona morta visível em número em vez de "o robô não andou" — o defeito
    sem sintoma que custou horas na competição de 2025 (BO-3).

Velocidade medida sai da POSE, nunca do campo `twist` — mesma razão anotada no
`ensaio.py`: o twist do publicador do Gazebo é ruidoso, a pose é limpa.
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

JANELA_S = 0.2          # janela de diferenciação da pose (igual à do ensaio.py)
DESCARTE_S = 1.5        # começo de cada trecho, jogado fora: é a aceleração

# ------------------------------------------------------------------ fase A
# (nome, v [m/s], wz [rad/s], duração [s])
# Giro para os dois lados de propósito: assimetria esquerda/direita é defeito
# de conversão ou de modelo, e só aparece comparando os dois sentidos.
#
# A VARREDURA de wz (0,3 a 1,0) não é capricho. A primeira corrida mostrou o
# giro puro saindo 15% abaixo do comandado enquanto a reta sai exata — o que
# exclui a conversão e sobra a boba raspando. Se o déficit for uma RAZÃO, ele
# acompanha o comando e o teto de 1,0 rad/s entrega bem menos que 1,0; se for
# um OFFSET, ele importa pouco lá em cima. A conta do pivô depende de qual é.
AFERICAO = [
    ('giro_030', 0.0, 0.3, 5.0),
    ('giro_esq', 0.0, 0.5, 5.0),
    ('giro_dir', 0.0, -0.5, 5.0),
    ('giro_080', 0.0, 0.8, 5.0),
    ('giro_100', 0.0, 1.0, 5.0),
    ('reta', 0.3, 0.0, 5.0),
    ('arco', 0.3, 0.4, 6.0),
]

# ------------------------------------------------------------------ fase B
# (nome, x, y, teto [s], giro esperado [°]) — coordenadas RELATIVAS à pose do
# robô no início da fase B, não ao `odom`.
#
# Relativas porque a fase A deixa o robô longe da origem e virado (ela gira de
# propósito). Com alvos absolutos, a primeira corrida mandou o robô girar 159°
# num trecho chamado "reta_2m" e 173° num chamado "lado_90": os nomes mentiam,
# e um resultado com nome errado é pior que resultado nenhum. Ancorando no
# quadro do robô, cada trecho é a manobra que o nome diz.
#
# O giro esperado é geometria pura (o rumo do alvo visto do trecho anterior).
# Serve para separar "girou o que precisava" de "girou à toa".
ROTEIRO = [
    ('reta_2m', 2.0, 0.0, 40.0, 0.0),          # só linear, sem girar
    ('lado_90', 2.0, 1.5, 40.0, 90.0),         # 90° à esquerda e andar
    ('atras_180', 0.5, 1.5, 50.0, 180.0),      # virar por completo
    ('perto_de_lado', 0.5, 1.15, 60.0, 90.0),  # perto E de lado: a ré
    ('volta_origem', 0.0, 0.0, 50.0, 90.0),    # diagonal, de volta ao começo
]


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


class Bancada(Node):
    def __init__(self, saida):
        super().__init__('bancada_gazebo')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.saida = saida

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        # Fase A fala direto com a tração (sem placa); fase B fala com a
        # navegação, que fala com a movimentação, que fala com a placa.
        self.pub_cmd = self.create_publisher(
            TwistStamped, '/hoverboard_base_controller/cmd_vel', qos)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', qos)

        self.create_subscription(Odometry, '/Odometry', self.cb_pose, qos)
        self.create_subscription(TwistStamped, '/cmd_vel_bruto',
                                 self.cb_bruto, qos)
        self.create_subscription(TwistStamped,
                                 '/hoverboard_base_controller/cmd_vel',
                                 self.cb_placa, qos)

        self.pose = None
        self.bruto = (0.0, 0.0)
        self.placa = (0.0, 0.0)
        self.hist = deque()
        self.linhas = []

    # ---------------------------------------------------------- callbacks
    def cb_pose(self, msg):
        self.pose = msg

    def cb_bruto(self, msg):
        self.bruto = (msg.twist.linear.x, msg.twist.angular.z)

    def cb_placa(self, msg):
        self.placa = (msg.twist.linear.x, msg.twist.angular.z)

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def estado(self):
        p = self.pose.pose.pose
        return p.position.x, p.position.y, yaw_de(p.orientation)

    def derivada(self, t, x, y, yaw):
        self.hist.append((t, x, y, yaw))
        while len(self.hist) > 1 and t - self.hist[0][0] > JANELA_S:
            self.hist.popleft()
        if len(self.hist) < 2:
            return 0.0, 0.0
        t1, x1, y1, yaw1 = self.hist[0]
        dt = t - t1
        if dt < 1e-6:
            return 0.0, 0.0
        # Linear COM SINAL: de ré o robô anda para trás e hypot esconderia
        # isso, justamente na manobra que queremos ver (decisão 007).
        d = math.hypot(x - x1, y - y1)
        avanco = (x - x1) * math.cos(yaw) + (y - y1) * math.sin(yaw)
        return math.copysign(d, avanco) / dt, norm_ang(yaw - yaw1) / dt

    # ------------------------------------------------------------ registro
    def registra(self, fase, trecho, te, extra):
        x, y, yaw = self.estado()
        v_pose, wz_pose = self.derivada(self.agora(), x, y, yaw)
        linha = {
            'fase': fase, 'trecho': trecho, 't': round(te, 3),
            'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4),
            'v_pose': round(v_pose, 4), 'wz_pose': round(wz_pose, 4),
            'cmd_v_bruto': round(self.bruto[0], 4),
            'cmd_wz_bruto': round(self.bruto[1], 4),
            'cmd_v_placa': round(self.placa[0], 4),
            'cmd_wz_placa': round(self.placa[1], 4),
        }
        linha.update(extra)
        self.linhas.append(linha)
        return linha

    def publica(self, v, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(wz)
        self.pub_cmd.publish(m)

    def gira(self, seg=0.05):
        """Deixa o ROS respirar por `seg` de tempo de SIMULAÇÃO."""
        alvo = self.agora() + seg
        while rclpy.ok() and self.agora() < alvo:
            rclpy.spin_once(self, timeout_sec=0.05)

    # -------------------------------------------------------------- fase A
    def roda_afericao(self):
        print('\n== FASE A — aferição em malha aberta ==', file=sys.stderr)
        resumo = []
        for nome, v, wz, dur in AFERICAO:
            t0 = self.agora()
            amostras = []
            while rclpy.ok() and self.agora() - t0 < dur:
                te = self.agora() - t0
                self.publica(v, wz)
                linha = self.registra('afericao', nome, te,
                                      {'cmd_v': round(v, 4),
                                       'cmd_wz': round(wz, 4),
                                       'alvo_x': '', 'alvo_y': '',
                                       'dist': ''})
                if te > DESCARTE_S:
                    amostras.append((linha['v_pose'], linha['wz_pose']))
                rclpy.spin_once(self, timeout_sec=0.05)

            # Freia e espera parar de verdade antes do próximo trecho, senão a
            # inércia do trecho anterior vaza para dentro da medida seguinte.
            for _ in range(20):
                self.publica(0.0, 0.0)
                self.gira(0.1)

            if not amostras:
                continue
            v_med = sum(a[0] for a in amostras) / len(amostras)
            wz_med = sum(a[1] for a in amostras) / len(amostras)
            resumo.append((nome, v, wz, v_med, wz_med, len(amostras)))
            print(f'  {nome:10s} cmd v={v:+.3f} wz={wz:+.3f}  ->  '
                  f'medido v={v_med:+.3f} wz={wz_med:+.3f}', file=sys.stderr)
        return resumo

    # -------------------------------------------------------------- fase B
    def manda_objetivo(self, x, y):
        g = PoseStamped()
        g.header.frame_id = 'odom'
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.orientation.w = 1.0
        self.pub_goal.publish(g)

    def roda_roteiro(self, raio_chegada):
        print('\n== FASE B — roteiro em malha fechada ==', file=sys.stderr)
        # Quadro de referência da fase B: onde o robô está AGORA, olhando para
        # onde está olhando agora. Todo alvo do roteiro é lido aqui dentro.
        bx, by, byaw = self.estado()
        c, s = math.cos(byaw), math.sin(byaw)
        print(f'  (quadro da fase B: x={bx:.2f} y={by:.2f} '
              f'rumo={math.degrees(byaw):.1f}°)', file=sys.stderr)

        resultado = []
        for nome, rx, ry, teto, giro_esp in ROTEIRO:
            ax = bx + rx * c - ry * s
            ay = by + rx * s + ry * c
            x0, y0, yaw0 = self.estado()
            # Repete o objetivo algumas vezes: publicador RELIABLE recém-criado
            # contra assinante recém-criado ainda perde mensagem.
            for _ in range(3):
                self.manda_objetivo(ax, ay)
                self.gira(0.1)

            t0 = self.agora()
            chegou = False
            d_min = float('inf')
            re_vista = False
            while rclpy.ok() and self.agora() - t0 < teto:
                te = self.agora() - t0
                x, y, yaw = self.estado()
                d = math.hypot(ax - x, ay - y)
                d_min = min(d_min, d)
                linha = self.registra('roteiro', nome, te,
                                      {'cmd_v': '', 'cmd_wz': '',
                                       'alvo_x': ax, 'alvo_y': ay,
                                       'dist': round(d, 4)})
                # Ré = comando linear negativo pedido pela navegação.
                if linha['cmd_v_bruto'] < -0.01:
                    re_vista = True
                if d <= raio_chegada:
                    chegou = True
                    break
                rclpy.spin_once(self, timeout_sec=0.05)

            gasto = self.agora() - t0
            x, y, yaw = self.estado()
            giro = abs(math.degrees(norm_ang(yaw - yaw0)))
            resultado.append({
                'alvo': nome, 'chegou': chegou, 'd_final': math.hypot(ax - x, ay - y),
                'd_min': d_min, 's': gasto, 'giro_deg': giro,
                'giro_esperado': giro_esp, 're': re_vista,
            })
            marca = 'OK  ' if chegou else 'FALHOU'
            print(f'  {marca} {nome:14s} d_final={resultado[-1]["d_final"]:.3f} m  '
                  f'd_min={d_min:.3f}  {gasto:5.1f} s  '
                  f'giro={giro:5.1f}° (esperado {giro_esp:5.1f}°)'
                  f'{"  [DEU RÉ]" if re_vista else ""}', file=sys.stderr)

            # Solta o robô antes do próximo alvo.
            self.gira(1.0)
        return resultado

    def grava(self):
        if not self.linhas:
            print('sem dados — o /Odometry chegou?', file=sys.stderr)
            return
        campos = list(self.linhas[0].keys())
        with open(self.saida, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=campos)
            w.writeheader()
            w.writerows(self.linhas)
        print(f'\n{len(self.linhas)} amostras -> {self.saida}', file=sys.stderr)


# ------------------------------------------------------------ orquestração

class Processos:
    """Sobe e derruba as pilhas. Cada uma em seu grupo, para morrer inteira."""

    def __init__(self):
        self.filhos = []

    def sobe(self, cmd, nome):
        print(f'[subindo] {nome}', file=sys.stderr)
        p = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.STDOUT, start_new_session=True)
        self.filhos.append((p, nome))
        return p

    def derruba(self):
        for p, nome in reversed(self.filhos):
            if p.poll() is not None:
                continue
            print(f'[derrubando] {nome}', file=sys.stderr)
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except ProcessLookupError:
                continue
        time.sleep(3.0)
        for p, _ in reversed(self.filhos):
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        # O Gazebo às vezes sobrevive ao launch que o criou.
        subprocess.run(['pkill', '-9', '-f', 'gz sim'], check=False)


def espera_odometria(no, seg=90.0):
    """Sem pose não há bancada: espera o simulador nascer antes de medir."""
    fim = time.time() + seg
    while time.time() < fim:
        rclpy.spin_once(no, timeout_sec=0.2)
        if no.pose is not None:
            return True
    return False


def main():
    ap = argparse.ArgumentParser(description='Bancada de manobras no Gazebo')
    ap.add_argument('--perfil', choices=['sim', 'real'], required=True,
                    help='"sim" = zona morta 0,10 (otimista); "real" = 0,15, '
                         'o chute pessimista do robô, planta E controlador')
    ap.add_argument('--csv', default=None)
    ap.add_argument('--planta', default='lenta')
    cfg = ap.parse_args()

    # A zona morta entra nos DOIS lados de propósito. Pôr o chute pessimista
    # só no controlador, com a planta otimista, mede uma máquina que não
    # existe: o controlador se defenderia de um defeito que a planta não tem.
    # 31-07: a placa deixou de ser "zona morta em m/s" e passou a ser um modelo
    # de atuador. O perfil otimista vira a placa IDEAL (obedece tudo) e o
    # pessimista vira a placa MEDIDA no robô — que é bem pior que a zona morta
    # de 0,15 que se supunha, porque nela o comando nem controla módulo.
    placa = 'ideal' if cfg.perfil == 'sim' else 'medido'
    sufixo = '_sim' if cfg.perfil == 'sim' else ''
    csv_saida = cfg.csv or (
        f'docs/dados/{time.strftime("%Y-%m-%d")}-bancada-gazebo-'
        f'{cfg.perfil}.csv')

    from ament_index_python.packages import get_package_share_directory
    pkg_motion = get_package_share_directory('robot_motion')
    p_mov = os.path.join(pkg_motion, 'config', f'movimentacao{sufixo}.yaml')
    p_nav = os.path.join(pkg_motion, 'config', f'navegacao{sufixo}.yaml')

    procs = Processos()
    rclpy.init()
    no = Bancada(csv_saida)
    codigo = 0
    try:
        procs.sobe(['ros2', 'launch', 'robot_base', 'sim.launch.py',
                    'gui:=false', f'planta:={cfg.planta}',
                    f'placa:={placa}'], 'gazebo + tração')
        if not espera_odometria(no):
            print('ERRO: /Odometry nunca chegou — o simulador não subiu.',
                  file=sys.stderr)
            return 1
        # A física precisa assentar o robô nas rodas antes de qualquer medida.
        no.gira(3.0)

        no.roda_afericao()

        # A movimentação só entra AGORA: durante a fase A ela brigaria pelo
        # mesmo cmd_vel e a aferição mediria os dois somados.
        procs.sobe(['ros2', 'run', 'robot_motion', 'heading_controller',
                    '--ros-args', '--params-file', p_mov,
                    '-p', 'use_sim_time:=true',
                    '-r', '/hoverboard_base_controller/cmd_vel:=/cmd_vel_bruto',
                    '-r', '__node:=heading_controller'], 'movimentação')
        procs.sobe(['ros2', 'run', 'robot_motion', 'goal_navigator',
                    '--ros-args', '--params-file', p_nav,
                    '-p', 'use_sim_time:=true',
                    '-r', '__node:=goal_navigator',
                    '-r', '~/objetivo:=/goal_pose'], 'navegação')
        no.gira(5.0)

        raio = 0.15
        no.roda_roteiro(raio)
    except KeyboardInterrupt:
        print('interrompido no teclado', file=sys.stderr)
        codigo = 130
    finally:
        no.grava()
        no.destroy_node()
        rclpy.shutdown()
        procs.derruba()
    return codigo


if __name__ == '__main__':
    sys.exit(main())
