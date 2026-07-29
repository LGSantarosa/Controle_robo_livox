#!/usr/bin/env python3
"""Varredura de raio mínimo na bancada do planner: a conclusão depende dele?

    python3 tools/planner/varredura_raio.py

Roda o MESMO conjunto de casos contra Theta* e Smac Hybrid-A*, uma vez para
cada raio mínimo de curva da lista. Sem RViz, sem cliques, sem robô, sem
simulador — a bancada de `robot_planning` já não move nada; esta aqui só troca
o dedo do humano por um roteiro e repete tudo por raio.

POR QUE ELA EXISTE
    O `minimum_turning_radius` não é mais um parâmetro dessa comparação: ele
    **é** o argumento dela. O Smac Hybrid-A* está na mesa contra o Theta*
    justamente porque respeita raio de curva; se o raio informado for otimista,
    o Smac ganha desenhando curvas que a máquina não fecha, e a decisão 008
    sairia assinada em cima de um robô que não existe.

    E o número certo não é sabido. A corrida de 29-07 (`corrida_gazebo.py`) mede
    raio realizado de 0,370 m no perfil otimista de zona morta e 0,463 m no
    pessimista, contra os 0,25 m que a bancada tinha configurado. Qual dos dois
    perfis é o robô, só a zona morta medida na bancada real dirá.

    Daí varrer em vez de escolher: **se o ranking não mudar com o raio, a
    conclusão está imune à medida que falta** e o dono pode julgar hoje. Se
    mudar, isso se sabe ANTES de assinar a decisão, e a zona morta ganha uma
    segunda razão de peso na fila da bancada com o robô.

O QUE ELA NÃO RESPONDE
    Se o robô consegue seguir o caminho — igual à bancada de origem, ela
    desenha e não dirige. E o Theta* não tem raio nenhum: ele é a testemunha do
    experimento, e sair igual em todos os raios é o cheque de que a varredura
    mexeu só no que deveria.
"""
import argparse
import csv
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

# A régua é IMPORTADA da bancada, não copiada: dois `mede()` que divergem
# fazem a varredura comparar medidas de réguas diferentes e ninguém percebe.
from robot_planning.bancada_planner import mede

RAIZ = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# ------------------------------------------------------------------ os raios
# 0,25 é o que a bancada tinha (justificado por uma curva de 28-07); os outros
# três saem da corrida de 29-07 — o que o controlador PEDE e o que a máquina
# REALIZA, em cada perfil de zona morta. O realizado é maior porque o giro
# entrega 79% do comandado, e o raio abre na mesma proporção.
RAIOS = [
    (0.25, 'configurado até 29-07'),
    (0.34, 'pedido pelo controlador, perfil zona morta 0,15'),
    (0.37, 'realizado (p5), perfil zona morta 0,10'),
    (0.46, 'realizado (p5), perfil zona morta 0,15'),
]

# ------------------------------------------------------------------ os casos
# (nome, x0, y0, rumo0 [graus], x1, y1) na pista de `tools/mundo/gera_pista.py`:
# sala de 12 x 8 m, porta de 0,90 m em x=4, bloco solto em (6,5..8 · 4..5,5),
# aperto de 0,80 m em x=9 e beco sem saída no canto sudeste.
#
# Cada caso isola UMA pergunta. Misturar dois obstáculos no mesmo caso faz o
# resultado ficar sem dono quando ele piora.
CASOS = [
    # o vão de 0,90 m contra um robô de 0,50 m: passa ou contorna?
    ('porta',          2.0, 5.0,   0.0,  6.0, 1.5),
    # bloco solto em campo aberto: contorna pelo lado curto ou dá a volta?
    ('bloco',          5.0, 4.75,  0.0,  8.5, 4.75),
    # o aperto de 0,80 m é a ÚNICA passagem em x=9: aceita ou desiste?
    ('aperto',         7.0, 6.5,   0.0, 10.5, 6.5),
    # destino dentro do beco: entra pela boca certa ou tenta atravessar?
    ('beco',           9.5, 6.0, -90.0, 11.4, 1.0),
    # o caso que trouxe o Nav2 para a conversa: alvo PERTO e DE LADO. É onde o
    # raio mínimo morde mais forte, e onde a ré do Reeds-Shepp deve aparecer.
    ('perto_de_lado',  5.0, 6.0,   0.0,  5.0, 6.6),
    # o mesmo de lado, com folga: separa "curto demais" de "de lado".
    ('lado_1m',        5.0, 6.0,   0.0,  5.0, 6.9),
]

PLANNERS = ['theta', 'hibrido']


def pose(x, y, graus=0.0, quadro='map'):
    p = PoseStamped()
    p.header.frame_id = quadro
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(math.radians(graus) / 2.0)
    p.pose.orientation.w = math.cos(math.radians(graus) / 2.0)
    return p


class Varredura(Node):
    def __init__(self):
        super().__init__('varredura_raio')
        self.cliente = ActionClient(self, ComputePathToPose,
                                    'compute_path_to_pose')
        # Mesmo motivo anotado na bancada: pedir caminho antes de o costmap ter
        # RECEBIDO o mapa devolve caminho vazio com código de SUCESSO — ou seja,
        # mentindo. Sem esperar por ele a primeira linha de cada raio seria lixo.
        self.costmap_ok = False
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            lambda _m: setattr(self, 'costmap_ok', True),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def espera_pilha(self, seg=90.0):
        fim = time.time() + seg
        while time.time() < fim and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.costmap_ok and self.cliente.server_is_ready():
                return True
        return False

    def pede(self, planner, ini, dest, espera=20.0):
        """Um caminho. Devolve (medida, ms) ou (None, motivo)."""
        objetivo = ComputePathToPose.Goal()
        objetivo.start = ini
        objetivo.goal = dest
        objetivo.planner_id = planner
        objetivo.use_start = True     # planeja entre os dois pontos do roteiro

        fut = self.cliente.send_goal_async(objetivo)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=espera)
        if fut.result() is None:
            return None, 'sem resposta do servidor'
        handle = fut.result()
        if not handle.accepted:
            return None, 'pedido recusado'

        fr = handle.get_result_async()
        rclpy.spin_until_future_complete(self, fr, timeout_sec=espera)
        if fr.result() is None:
            return None, 'resultado não chegou'
        res = fr.result().result
        if res.error_code != 0:
            return None, f'erro {res.error_code}: {res.error_msg or "sem msg"}'
        if len(res.path.poses) == 0:
            return None, 'caminho VAZIO'
        ms = res.planning_time.sec * 1000.0 + res.planning_time.nanosec / 1e6
        return mede(res.path), ms


class Pilha:
    """Sobe e derruba a bancada headless. Uma pilha por raio, sem exceção."""

    def __init__(self):
        self.p = None

    def sobe(self, params):
        self.p = subprocess.Popen(
            ['ros2', 'launch', 'robot_planning', 'bancada_planner.launch.py',
             'rviz:=false', f'params:={params}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT,
            start_new_session=True)

    def derruba(self):
        if self.p is None or self.p.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(self.p.pid), signal.SIGINT)
        except ProcessLookupError:
            return
        for _ in range(30):
            if self.p.poll() is not None:
                break
            time.sleep(0.2)
        if self.p.poll() is None:
            try:
                os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


def yaml_do_raio(base, raio, destino, modelo=None):
    """Copia o YAML da bancada trocando o raio mínimo (e o modelo, se pedido).

    Reescrita por LINHA, de propósito: carregar e reserializar o YAML jogaria
    fora todos os comentários, que neste repo são metade do valor do arquivo.

    `modelo` troca o `motion_model_for_search`. REEDS_SHEPP deixa o planner usar
    ré; DUBIN a proíbe. Existe porque a ré PLANEJADA é contestada: em robô com
    Nav2 ela tende a ficar tentando entrar e sair de trechos de ré, e o custo de
    proibi-la no plano não era conhecido — esta bancada mede.
    """
    trocou = False
    with open(base) as f:
        linhas = f.readlines()
    saida = []
    for linha in linhas:
        if linha.strip().startswith('minimum_turning_radius:'):
            indent = linha[:len(linha) - len(linha.lstrip())]
            saida.append(f'{indent}minimum_turning_radius: {raio}\n')
            trocou = True
        elif modelo and linha.strip().startswith('motion_model_for_search:'):
            indent = linha[:len(linha) - len(linha.lstrip())]
            saida.append(f'{indent}motion_model_for_search: "{modelo}"\n')
        else:
            saida.append(linha)
    if not trocou:
        raise SystemExit(f'ERRO: `minimum_turning_radius` não achado em {base}')
    with open(destino, 'w') as f:
        f.writelines(saida)
    return destino


def main():
    ap = argparse.ArgumentParser(description='Varredura de raio mínimo')
    ap.add_argument('--csv', default=os.path.join(
        RAIZ, 'docs', 'dados',
        f'{time.strftime("%Y-%m-%d")}-varredura-raio-planner.csv'))
    ap.add_argument('--tmp', default='/tmp')
    ap.add_argument('--modelo', default=None,
                    choices=['REEDS_SHEPP', 'DUBIN'],
                    help='troca o motion_model_for_search (padrão: o do YAML). '
                         'REEDS_SHEPP deixa o planner usar ré; DUBIN proíbe.')
    cfg = ap.parse_args()

    from ament_index_python.packages import get_package_share_directory
    base = os.path.join(get_package_share_directory('robot_planning'),
                        'config', 'bancada_planner.yaml')

    linhas = []
    rclpy.init()
    try:
        for raio, porque in RAIOS:
            print(f'\n{"=" * 74}\nRAIO {raio:.2f} m — {porque}\n{"=" * 74}',
                  file=sys.stderr)
            params = yaml_do_raio(
                base, raio, os.path.join(cfg.tmp, f'bancada_r{raio:.2f}.yaml'),
                modelo=cfg.modelo)
            pilha = Pilha()
            no = Varredura()
            try:
                pilha.sobe(params)
                if not no.espera_pilha():
                    print('  ERRO: a pilha não ficou pronta — pulando este raio',
                          file=sys.stderr)
                    continue
                print(f'  {"caso":<15}{"planner":<10}{"compr":>8}{"desvio":>8}'
                      f'{"giro":>7}{"raio_min":>10}{"inv":>5}{"curt":>6}'
                      f'{"ms":>7}',
                      file=sys.stderr)
                for nome, x0, y0, g0, x1, y1 in CASOS:
                    ini, dest = pose(x0, y0, g0), pose(x1, y1)
                    for planner in PLANNERS:
                        medida, extra = no.pede(planner, ini, dest)
                        reg = {'raio': raio, 'caso': nome, 'planner': planner}
                        if medida is None:
                            reg.update({'ok': 0, 'motivo': extra})
                            print(f'  {nome:<15}{planner:<10}  SEM CAMINHO '
                                  f'({extra})', file=sys.stderr)
                        else:
                            r = medida['raio_min']
                            reg.update({
                                'ok': 1, 'motivo': '',
                                'comprimento': round(medida['comprimento'], 3),
                                'reta': round(medida['reta'], 3),
                                'desvio': round(medida['desvio'], 3),
                                'giro_deg': round(medida['giro_deg'], 1),
                                'raio_min': ('inf' if math.isinf(r)
                                             else round(r, 3)),
                                'inversoes': medida['inversoes'],
                                'curtos': medida['trechos_curtos'],
                                'pontos': medida['pontos'],
                                'ms': round(extra, 1),
                            })
                            rtxt = '  reto' if math.isinf(r) else f'{r:6.2f}'
                            print(f'  {nome:<15}{planner:<10}'
                                  f'{medida["comprimento"]:8.2f}'
                                  f'{medida["desvio"]:8.2f}'
                                  f'{medida["giro_deg"]:7.0f}'
                                  f'{rtxt:>10}'
                                  f'{medida["inversoes"]:5d}'
                                  f'{medida["trechos_curtos"]:6d}'
                                  f'{extra:7.0f}',
                                  file=sys.stderr)
                        linhas.append(reg)
            finally:
                no.destroy_node()
                pilha.derruba()
                time.sleep(2.0)
    except KeyboardInterrupt:
        print('interrompido no teclado', file=sys.stderr)
    finally:
        rclpy.shutdown()

    if not linhas:
        print('nada medido', file=sys.stderr)
        return 1
    campos = ['raio', 'caso', 'planner', 'ok', 'comprimento', 'reta', 'desvio',
              'giro_deg', 'raio_min', 'inversoes', 'curtos', 'pontos', 'ms',
              'motivo']
    with open(cfg.csv, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=campos, extrasaction='ignore')
        w.writeheader()
        w.writerows(linhas)
    print(f'\n{len(linhas)} linhas -> {cfg.csv}', file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
