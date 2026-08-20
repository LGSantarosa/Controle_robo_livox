#!/usr/bin/env python3
"""Protocolo contínuo dos dois vãos da pista, com resultado por perna.

Mantém a mesma pilha/Gazebo vivos e alterna objetivos entre os dois lados da
pista. Isso mede a propriedade pedida pelo dono em 20-08: não basta uma
corrida bonita; o robô precisa repetir ida e volta sem bater nem depender de
reset favorável.

O bag completo é responsabilidade de ``pilha.launch.py``. Este instrumento
acrescenta um CSV de pose por perna e ``resumo.json``, atualizado depois de
cada objetivo para uma interrupção nunca apagar as pernas já concluídas.
"""
import argparse
import csv
from datetime import datetime, timezone
import json
import math
import os
import time


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ciclos', type=int, default=3,
                    help='quantas idas+voltas sem reiniciar a pilha')
    ap.add_argument('--ida', nargs=2, type=float, default=(10.65, 6.60),
                    metavar=('X', 'Y'))
    ap.add_argument('--volta', nargs=2, type=float, default=(2.00, 5.00),
                    metavar=('X', 'Y'))
    ap.add_argument('--teto-s', type=float, default=120.0)
    ap.add_argument('--saida', required=True)
    a = ap.parse_args()
    if a.ciclos <= 0:
        ap.error('--ciclos precisa ser positivo')
    os.makedirs(a.saida, exist_ok=True)

    import rclpy
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.msg import CollisionMonitorState
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TwistStamped
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Float64

    class Repeticao(Node):
        def __init__(self):
            super().__init__('repeticao_portas')
            self.pose = None
            self.alvo = None
            self.t0 = None
            self.amostras = []
            self.paradas_reflexo = 0
            self.reflexo_em_stop = False
            self.reflexo_desde = None
            self.tempo_reflexo_stop = 0.0
            self.res = 0
            self.em_re = False
            self.re_desde = None
            self.tempo_re = 0.0
            self.recuperacoes = 0
            self.recuperacao_ativa = False
            self.recuperacao_desde = None
            self.tempo_recuperacao = 0.0
            self.tipos_recuperacao = set()
            self.cliente = ActionClient(self, NavigateToPose,
                                        'navigate_to_pose')
            self.create_subscription(Odometry, '/Odometry', self.cb_odom,
                                     qos_profile_sensor_data)
            self.create_subscription(
                CollisionMonitorState, '/collision_monitor_state',
                self.cb_reflexo, 10)
            self.create_subscription(
                Float64, '/heading_controller/velocidade_alvo',
                self.cb_velocidade, 10)
            self.create_subscription(
                TwistStamped, '/unstuck_vel', self.cb_desencalhe, 10)

        def agora(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def cb_reflexo(self, msg):
            if self.alvo is None:
                return
            agora = self.agora()
            parou = msg.action_type == CollisionMonitorState.STOP
            if parou and not self.reflexo_em_stop:
                self.paradas_reflexo += 1
                self.reflexo_desde = agora
            elif not parou and self.reflexo_em_stop:
                self.tempo_reflexo_stop += max(
                    0.0, agora - self.reflexo_desde)
                self.reflexo_desde = None
            self.reflexo_em_stop = parou

        def cb_velocidade(self, msg):
            if self.alvo is None:
                return
            agora = self.agora()
            re = msg.data < -1e-4
            if re and not self.em_re:
                self.res += 1
                self.re_desde = agora
            elif not re and self.em_re:
                self.tempo_re += max(0.0, agora - self.re_desde)
                self.re_desde = None
            self.em_re = re

        def cb_desencalhe(self, msg):
            if self.alvo is None:
                return
            agora = self.agora()
            v = msg.twist.linear.x
            wz = msg.twist.angular.z
            ativo = abs(v) > 1e-4 or abs(wz) > 1e-4
            if ativo:
                if v < -1e-4:
                    self.tipos_recuperacao.add('re')
                elif v > 1e-4:
                    self.tipos_recuperacao.add('frente')
                if abs(wz) > 1e-4:
                    self.tipos_recuperacao.add('pivo')
            if ativo and not self.recuperacao_ativa:
                self.recuperacoes += 1
                self.recuperacao_desde = agora
            elif not ativo and self.recuperacao_ativa:
                self.tempo_recuperacao += max(
                    0.0, agora - self.recuperacao_desde)
                self.recuperacao_desde = None
            self.recuperacao_ativa = ativo

        def cb_odom(self, msg):
            self.pose = msg
            if self.alvo is None:
                return
            p = msg.pose.pose
            agora = self.agora()
            if self.t0 is None:
                self.t0 = agora
            self.amostras.append({
                't': round(agora - self.t0, 4),
                'x': round(p.position.x, 5),
                'y': round(p.position.y, 5),
                'yaw': round(yaw_de(p.orientation), 5),
                'dist': round(math.hypot(self.alvo[0] - p.position.x,
                                         self.alvo[1] - p.position.y), 5),
            })

        def salva_csv(self, nome):
            caminho = os.path.join(a.saida, nome)
            if not self.amostras:
                return caminho
            with open(caminho, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=self.amostras[0].keys())
                w.writeheader()
                w.writerows(self.amostras)
            return caminho

        def executa(self, numero, nome, alvo):
            self.alvo = tuple(alvo)
            self.t0 = None
            self.amostras = []
            self.paradas_reflexo = 0
            self.reflexo_em_stop = False
            self.reflexo_desde = None
            self.tempo_reflexo_stop = 0.0
            self.res = 0
            self.em_re = False
            self.re_desde = None
            self.tempo_re = 0.0
            self.recuperacoes = 0
            self.recuperacao_ativa = False
            self.recuperacao_desde = None
            self.tempo_recuperacao = 0.0
            self.tipos_recuperacao = set()
            if not self.cliente.wait_for_server(timeout_sec=15.0):
                return {'perna': numero, 'nome': nome, 'alvo': list(alvo),
                        'status': 'sem_servidor', 'sucesso': False,
                        'aprovada': False}

            g = NavigateToPose.Goal()
            g.pose.header.frame_id = 'map'
            g.pose.header.stamp = self.get_clock().now().to_msg()
            g.pose.pose.position.x = float(alvo[0])
            g.pose.pose.position.y = float(alvo[1])
            g.pose.pose.orientation.w = 1.0

            enviado = self.cliente.send_goal_async(g)
            rclpy.spin_until_future_complete(self, enviado, timeout_sec=15.0)
            alca = enviado.result() if enviado.done() else None
            if alca is None or not alca.accepted:
                self.alvo = None
                return {'perna': numero, 'nome': nome, 'alvo': list(alvo),
                        'status': 'recusado', 'sucesso': False,
                        'aprovada': False}

            resultado = alca.get_result_async()
            inicio_real = time.monotonic()
            while (rclpy.ok() and not resultado.done()
                   and time.monotonic() - inicio_real < a.teto_s):
                rclpy.spin_once(self, timeout_sec=0.05)

            estourou = not resultado.done()
            if estourou:
                cancelar = alca.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancelar,
                                                 timeout_sec=5.0)
            status_num = (None if estourou else resultado.result().status)
            sucesso = status_num == GoalStatus.STATUS_SUCCEEDED
            duracao = time.monotonic() - inicio_real
            agora = self.agora()
            if self.reflexo_em_stop and self.reflexo_desde is not None:
                self.tempo_reflexo_stop += max(
                    0.0, agora - self.reflexo_desde)
                self.reflexo_desde = agora
            if self.em_re and self.re_desde is not None:
                self.tempo_re += max(0.0, agora - self.re_desde)
                self.re_desde = agora
            if (self.recuperacao_ativa
                    and self.recuperacao_desde is not None):
                self.tempo_recuperacao += max(
                    0.0, agora - self.recuperacao_desde)
                self.recuperacao_desde = agora
            final = None
            dist_final = None
            if self.pose is not None:
                p = self.pose.pose.pose.position
                final = [p.x, p.y]
                dist_final = math.hypot(alvo[0] - p.x, alvo[1] - p.y)
            csv_nome = f'perna_{numero:02d}_{nome}.csv'
            self.salva_csv(csv_nome)
            # O dono pediu recuperação ilimitada enquanto houver objetivo:
            # ré, avanço e giro são movimentos válidos e ficam contabilizados,
            # não são reprovação. O que jamais pode acontecer é o reflexo ter
            # de evitar contato — uma perna só passa com zero PolygonStop.
            aprovado = sucesso and self.paradas_reflexo == 0
            resposta = {
                'perna': numero, 'nome': nome, 'alvo': list(alvo),
                'status': ('timeout' if estourou else int(status_num)),
                'sucesso': sucesso, 'aprovada': aprovado,
                'duracao_s': round(duracao, 3),
                'final': final, 'dist_final_m': dist_final,
                'paradas_reflexo': self.paradas_reflexo,
                'tempo_reflexo_stop_s': round(self.tempo_reflexo_stop, 3),
                'res': self.res, 'tempo_re_s': round(self.tempo_re, 3),
                'recuperacoes': self.recuperacoes,
                'tipos_recuperacao': sorted(self.tipos_recuperacao),
                'tempo_recuperacao_s': round(self.tempo_recuperacao, 3),
                'amostras_odom': len(self.amostras), 'csv': csv_nome,
            }
            self.alvo = None
            return resposta

    resumo = {
        'inicio_utc': datetime.now(timezone.utc).isoformat(),
        'ciclos_pedidos': a.ciclos,
        'teto_por_perna_s': a.teto_s,
        'ida': list(a.ida), 'volta': list(a.volta),
        'pernas': [],
    }
    resumo_path = os.path.join(a.saida, 'resumo.json')

    def salva_resumo():
        resumo['sucessos'] = sum(x['sucesso'] for x in resumo['pernas'])
        resumo['falhas'] = len(resumo['pernas']) - resumo['sucessos']
        resumo['aprovadas'] = sum(x.get('aprovada', False)
                                 for x in resumo['pernas'])
        with open(resumo_path, 'w') as f:
            json.dump(resumo, f, indent=2, ensure_ascii=False)

    rclpy.init()
    no = Repeticao()
    try:
        numero = 0
        for _ in range(a.ciclos):
            for nome, alvo in (('ida', a.ida), ('volta', a.volta)):
                numero += 1
                print(f'PERNA {numero}/{2 * a.ciclos}: {nome} -> {alvo}',
                      flush=True)
                resultado = no.executa(numero, nome, alvo)
                resumo['pernas'].append(resultado)
                salva_resumo()
                print(json.dumps(resultado, ensure_ascii=False), flush=True)
                if not resultado['aprovada']:
                    print('NÃO APROVADA: objetivo, reflexo ou ré reprovou a '
                          'perna; protocolo interrompido e dados preservados.',
                          flush=True)
                    return 2
                # Deixa a parada de chegada assentar antes de inverter o rumo.
                fim = time.monotonic() + 1.0
                while time.monotonic() < fim:
                    rclpy.spin_once(no, timeout_sec=0.05)
        resumo['fim_utc'] = datetime.now(timezone.utc).isoformat()
        salva_resumo()
        return 0
    except KeyboardInterrupt:
        resumo['interrompido'] = True
        salva_resumo()
        return 130
    finally:
        no.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
