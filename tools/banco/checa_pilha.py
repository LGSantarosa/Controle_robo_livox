#!/usr/bin/env python3
"""Pré-voo da pilha de navegação — 30 s, e o robô NÃO se mexe.

    python3 tools/banco/checa_pilha.py
    python3 tools/banco/checa_pilha.py --csv docs/dados/AAAA-MM-DD-.../preflight.csv

Roda com a base e a pilha de pé e responde, item a item, se o que foi escrito
sem robô sobreviveu ao robô. Cada linha traz o veredito, o número que o produziu
e — quando falha — **o que fazer**, para não ter de sair procurando no roteiro
com a bateria correndo.

Ele não comanda velocidade nenhuma. Pode rodar com o robô no chão e ligado.

O que ele NÃO faz, e precisa de você:
  · empurrar o robô com a mão para ver a TF mudar (ele mede a TF parada);
  · qualquer coisa que ande — isso é `ensaio.py`, `plano.py`, `homem_morto.py`.

Ordem de leitura da saída: o primeiro ❌ costuma explicar os de baixo. Conserte
de cima para baixo.
"""
import argparse
import math
import subprocess
import sys
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

# Altura do Livox medida com trena em 05-08. O URDF tem 0,42; se a TF discordar,
# a nuvem inteira sai deslocada em altura e o filtro de chão passa a mentir.
LIVOX_Z = 0.42
TOLERANCIA_Z = 0.02

SERVIDORES = ['planner_server', 'controller_server', 'bt_navigator',
              'collision_monitor']

# Uma pilha só. Em 07-31 três `fastlio_mapping` órfãos produziram saltos de
# 1,35 m e custaram horas; em 07-08 três `ros2 launch` empilhados abortaram o
# bringup com cara de bug no código.
#
# `so_no_robo` marca o que NÃO existe no simulador: lá a pose é do Gazebo e a
# nuvem é de um `gpu_lidar`. No robô, zero desses processos é falha grave — e
# um script que dissesse ✅ para "0 fastlio" no robô seria pior que script
# nenhum, porque daria confiança errada.
UNICOS = {
    'fastlio_mapping': ('localização', True),
    'livox_ros_driver2_node': ('driver do lidar', True),
    'ros2 launch robot_motion': ('a pilha', False),
    'nav2_planner/planner_server': ('planner', False),
}

# Tópicos da cadeia de comando, na ordem em que o comando desce. O teleop não
# publicar nada em `/key_vel` (06-08) é o tipo de silêncio que só um contador de
# mensagens revela.
#
# ⚠️ **`/auto_vel` calado NÃO é defeito quando a entrada é só zero.** O
# `collision_monitor` **não republica comando nulo** — medido no simulador em
# 07-08: 3 s de zeros em `/auto_vel_raw` produzem 0 mensagens em `/auto_vel`, e
# 3 s de 0,05 m/s produzem 150. Com o robô parado o `heading_controller` publica
# só zeros, então o reflexo cala **por construção**.
#
# Isto corrige a leitura de 06-08, que registrou "recebe e não publica nem zero"
# como evidência de que faltava a TF. O sintoma era esperado; a TF faltava pelo
# que o Nav2 e o lifecycle mostraram, não por isto.
CADEIA = [
    ('/auto_vel_raw', 'heading_controller → reflexo', True),
    ('/auto_vel', 'reflexo → mux (SAÍDA do collision_monitor)', 'se_nao_nulo'),
    ('/key_vel', 'teclado → mux (só com o robot-key rodando)', False),
    ('/compensador_rumo/cmd_vel', 'mux → compensador', False),
    ('/hoverboard_base_controller/cmd_vel', 'compensador → ATUADOR', False),
]


class Checagem:
    def __init__(self):
        self.linhas = []

    def diz(self, nome, ok, detalhe, conserto=''):
        self.linhas.append((nome, ok, detalhe, conserto))
        marca = {True: '✅', False: '❌', None: '⚠️ '}[ok]
        print(f'{marca} {nome:<34} {detalhe}')
        if ok is not True and conserto:
            print(f'      ↳ {conserto}')
        return ok


def quantos(padrao):
    """Processos vivos que casam, LENDO a linha de comando de cada um.

    `pgrep -c` conta o próprio shell que o executou — mordeu duas vezes neste
    projeto. Aqui se lê o `ps` e se descarta o que for o próprio comando.
    """
    saida = subprocess.run(['ps', '-eo', 'cmd'], capture_output=True, text=True)
    return sum(1 for linha in saida.stdout.splitlines()
               if padrao in linha and 'ps -eo' not in linha
               and 'checa_pilha' not in linha)


class Preflight(Node):
    def __init__(self):
        super().__init__('checa_pilha')
        self.buffer = Buffer()
        self.ouvinte = TransformListener(self.buffer, self)
        self.nuvens = []
        self.pontos = 0
        self.mapa = None
        self.create_subscription(PointCloud2, '/livox/lidar', self._nuvem,
                                 qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, '/map', self._mapa,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))

    def _nuvem(self, msg):
        self.nuvens.append((time.time(), msg.header.stamp, msg.header.frame_id))
        self.pontos = msg.width * msg.height

    def _mapa(self, msg):
        self.mapa = msg

    def gira(self, segundos):
        fim = time.time() + segundos
        while time.time() < fim:
            rclpy.spin_once(self, timeout_sec=0.1)

    def estado_de(self, servidor, espera=3.0):
        cli = self.create_client(GetState, f'/{servidor}/get_state')
        if not cli.wait_for_service(timeout_sec=espera):
            return None
        fut = cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=espera)
        return fut.result().current_state.label if fut.done() else None

    def costmap(self, qual, espera=5.0):
        cli = self.create_client(GetCostmap, f'/{qual}/get_costmap')
        if not cli.wait_for_service(timeout_sec=espera):
            return None
        fut = cli.call_async(GetCostmap.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=espera)
        return fut.result().map if fut.done() and fut.result() else None


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--segundos', type=float, default=8.0,
                    help='janela de escuta da nuvem')
    ap.add_argument('--csv', help='grava o resultado')
    args = ap.parse_args()

    rclpy.init()
    no = Preflight()
    c = Checagem()
    print('\n== PRÉ-VOO DA PILHA ==  (o robô não se mexe)\n')

    # ---------------------------------------------------- 1. uma pilha só
    no_simulador = quantos('gz sim') > 0
    if no_simulador:
        print('   (Gazebo de pé: os itens de hardware não se aplicam)\n')
    for padrao, (papel, so_no_robo) in UNICOS.items():
        n = quantos(padrao)
        nome = f'um só {padrao.split("/")[-1][:22]}'
        if so_no_robo and no_simulador:
            c.diz(nome, None, f'{n} — simulador, não se aplica')
            continue
        c.diz(nome, n == 1, f'{n} processo(s) — {papel}',
              'matar por PID os extras. ⚠️ `ros2 launch` NÃO morre com os nós: '
              'mate o launch também' if n > 1 else
              f'{papel} NÃO está de pé' if n == 0 else '')

    print()
    no.gira(args.segundos)

    # ------------------------------------------------------ 2. a nuvem
    if not no.nuvens:
        c.diz('nuvem em /livox/lidar', False, 'NADA chegou',
              'driver do Livox caído; power-cycle no lidar já resolveu antes')
    else:
        dt = no.nuvens[-1][0] - no.nuvens[0][0]
        hz = (len(no.nuvens) - 1) / dt if dt > 0 else 0
        frame = no.nuvens[-1][2]
        c.diz('nuvem em /livox/lidar', hz > 5,
              f'{hz:.1f} Hz, {no.pontos} pontos/quadro, frame `{frame}`',
              'abaixo de 5 Hz o costmap vence o expected_update_rate (0,5 s) '
              'e PARA de atualizar')

    # -------------------------------------------------------- 3. as TFs
    ok_odom = False
    try:
        t = no.buffer.lookup_transform('odom', 'base_link', Time())
        p = t.transform.translation
        ok_odom = True
        c.diz('TF odom → base_link', True,
              f'({p.x:.3f}, {p.y:.3f}) — EMPURRE o robô e rode de novo: tem de mudar')
    except Exception as e:
        c.diz('TF odom → base_link', False, f'{type(e).__name__}',
              'é o `tf_odom` (decisão de 06-08). Veja o nome do frame que ele '
              'reclama no log e passe -p frame_da_pose:=<esse frame>')

    try:
        t = no.buffer.lookup_transform('base_link', 'livox_frame', Time())
        z = t.transform.translation.z
        bate = abs(z - LIVOX_Z) < TOLERANCIA_Z
        c.diz('TF base_link → livox_frame', bate, f'z = {z:.3f} m (trena: {LIVOX_Z})',
              'a nuvem inteira sai deslocada em altura e o filtro de chão mente. '
              'Provável URDF antigo no install/ — colcon build')
    except Exception as e:
        c.diz('TF base_link → livox_frame', False, f'{type(e).__name__}',
              'o robot_state_publisher não subiu, ou o URDF é o do diffbot')

    # ---------------------- 4. a nuvem é transformável? (o que o costmap faz)
    if no.nuvens and ok_odom:
        bons = 0
        for _, carimbo, frame in no.nuvens:
            try:
                no.buffer.lookup_transform('odom', frame, Time.from_msg(carimbo))
                bons += 1
            except Exception:
                pass
        frac = bons / len(no.nuvens)
        c.diz('nuvem transformável p/ odom', frac > 0.8,
              f'{bons}/{len(no.nuvens)} quadros ({frac * 100:.0f}%)',
              'o costmap descarta o que não transforma, o buffer envelhece e a '
              'camada fica não-current — costmap não-current PARA')

    print()
    # ------------------------------------------------- 5. os lifecycle
    for s in SERVIDORES:
        estado = no.estado_de(s)
        c.diz(f'lifecycle {s}', estado == 'active', estado or 'não respondeu',
              'o lifecycle_manager aborta o bringup INTEIRO se um servidor da '
              'lista não vier — foi assim que o collision_monitor morreu em '
              '06-08. `ros2 lifecycle set /<nó> activate` tira do buraco')

    # ------------------------------------------- 6. o perfil, com ou sem mapa
    print()
    tem_map_server = no.estado_de('map_server', espera=1.5) is not None
    c.diz('perfil SEM MAPA (decisão 015)', not tem_map_server,
          'sem map_server' if not tem_map_server else 'map_server DE PÉ',
          'no robô, suba com `mapa:=nenhum`. O mapa padrão é a planta da pista '
          'SIMULADA: parede onde não há nada, livre onde há parede')

    # --------------------------------------------------- 7. os costmaps
    for qual in ('local_costmap', 'global_costmap'):
        g = no.costmap(qual)
        if g is None:
            c.diz(f'{qual}', False, 'o serviço get_costmap não respondeu',
                  'o servidor dono dele não está ativo — veja o lifecycle acima')
            continue
        letais = sum(1 for v in g.data if v >= 254)
        res = g.metadata.resolution
        c.diz(f'{qual} marcando', letais > 0,
              f'{letais} células letais ({letais * res * res:.2f} m²), '
              f'{g.metadata.size_x}x{g.metadata.size_y} @ {res:.3f}',
              'ZERO obstáculo com o robô numa sala é a camada de obstáculo não '
              'recebendo nuvem. Confira a faixa de altura (0,10–0,50 m): o '
              'Mid-360 a 0,42 m só vê acima de 0,36 m a meio metro')

    # ------------------------------- 8. a cadeia de comando fala? (06-08)
    print()
    contagem = {t: 0 for t, _, _ in CADEIA}
    nao_nulas = {t: 0 for t, _, _ in CADEIA}

    def conta(msg, t):
        contagem[t] += 1
        v = msg.twist
        if abs(v.linear.x) + abs(v.angular.z) > 1e-6:
            nao_nulas[t] += 1

    for topico, _, _ in CADEIA:
        no.create_subscription(TwistStamped, topico,
                               lambda m, t=topico: conta(m, t), 10)
    no.gira(4.0)
    for topico, papel, exigido in CADEIA:
        n, vivas = contagem[topico], nao_nulas[topico]
        detalhe = f'{n} msgs em 4 s ({vivas} não-nulas) — {papel}'
        if exigido == 'se_nao_nulo':
            # O reflexo só fala quando há o que filtrar. Sem comando não-nulo
            # entrando, o silêncio dele é a resposta certa.
            entrou = nao_nulas['/auto_vel_raw']
            if not entrou:
                c.diz(topico[:32], None, detalhe + ' — entrada só zero',
                      'o collision_monitor NÃO republica comando nulo (medido '
                      '07-08). Para exercitar o reflexo é preciso comandar, e '
                      'isso é o teste D — com o "pode" do dono')
            else:
                c.diz(topico[:32], n > 0, detalhe,
                      f'entraram {entrou} comandos não-nulos e o reflexo calou: '
                      'ele não está conseguindo transformar a nuvem')
        elif exigido:
            c.diz(topico[:32], n > 0, detalhe,
                  'sem isto a autonomia não tem o que dizer ao reflexo — o '
                  'heading_controller não subiu')
        else:
            c.diz(topico[:32], True if n else None, detalhe,
                  'silêncio esperado se a fonte não está rodando')

    print()
    print('Move o robô? Nada aqui. Os que movem, e cada um espera o "pode":')
    print('  ensaio.py (curva crua)   plano.py (planeja parado)   homem_morto.py')

    if args.csv:
        with open(args.csv, 'w') as f:
            f.write('item,veredito,detalhe\n')
            for nome, ok, detalhe, _ in c.linhas:
                v = {True: 'PASSOU', False: 'FALHOU', None: 'ATENCAO'}[ok]
                f.write(f'"{nome}",{v},"{detalhe}"\n')
        print(f'\n-> {args.csv}')

    ruins = sum(1 for _, ok, _, _ in c.linhas if ok is False)
    print(f'\n{len(c.linhas) - ruins}/{len(c.linhas)} passaram.'
          + (f'  ⚠️ Conserte o PRIMEIRO ❌ e rode de novo — ele costuma '
             'explicar os de baixo.' if ruins else '  Pilha pronta.'))
    rclpy.shutdown()
    return 1 if ruins else 0


if __name__ == '__main__':
    sys.exit(main())
