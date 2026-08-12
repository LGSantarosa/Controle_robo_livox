#!/usr/bin/env python3
"""Uma corrida de navegação NO ROBÔ: manda um objetivo e grava a corrente toda.

    # robô LIGADO, base + pilha de pé, espaço livre à frente:
    python3 tools/banco/corrida_nav.py --alvo 2.0 0.0 \
        --csv docs/dados/AAAA-MM-DD-nav/objetivo-a.csv

    python3 tools/banco/corrida_nav.py --alvo 2.0 0.0 --teto-s 40   # menos corda

## Por que isto existe

O `corrida_gazebo.py` responde a mesma pergunta, mas sobe o Gazebo — não serve
no robô. E sem instrumento a corrida de navegação vira "o dono olha e relata",
que é exatamente o que este projeto não faz: o dono só roda, e os números vêm
por ssh.

O que ele grava é a CORRENTE INTEIRA, e cada elo separa uma hipótese:

    /plan                    o Nav2 planejou? replanejou quantas vezes?
    /auto_vel_raw            o nosso seguidor pediu o quê
    /auto_vel                o que o REFLEXO deixou passar   <- ele agiu?
    /compensador_rumo/cmd_vel  o que o MUX entregou
    /key_vel                 o humano meteu a mão?           <- corrida MISTA
    /Odometry                a pose aguentou 10 Hz ANDANDO?  <- a pergunta do CPU

Parar por reflexo e parar por ter chegado se parecem de fora. Só a comparação
`raw` × `saída` separa as duas — é por isso que as duas pontas são gravadas.

## Segurança

- manda o objetivo pela AÇÃO `navigate_to_pose` (e não por `/goal_pose`) para
  poder **cancelar**: Ctrl-C e o teto de tempo cancelam o objetivo antes de sair,
  senão o robô continua tentando com o instrumento morto;
- o cancelamento NÃO é freio. Quem para o robô na emergência é o teclado
  (`bin/robot-key`, prioridade 90 no mux, fura o reflexo de propósito). Tenha
  ele rodando num terminal separado ANTES de mandar o objetivo.
"""
import argparse
import csv
import math
import sys


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alvo', nargs=2, type=float, required=True,
                    metavar=('X', 'Y'))
    ap.add_argument('--rumo', type=float, default=None,
                    help='rumo final em GRAUS; omitido, chega em qualquer um')
    ap.add_argument('--frame', default='map')
    ap.add_argument('--csv', required=True)
    ap.add_argument('--teto-s', type=float, default=60.0,
                    help='corda máxima; estourou, cancela o objetivo')
    # O raio do seguidor (`path_follower.raio_chegada`). Aqui é só para o
    # VEREDITO — quem decide chegada no robô é o nó, não este instrumento.
    ap.add_argument('--raio', type=float, default=0.25)
    a = ap.parse_args()

    # rclpy fica DENTRO do main: assim `leitura_nav.py` (que é quem julga) roda
    # nos testes sem ROS instalado, e este arquivo continua importável.
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry, Path
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data

    sys.path.insert(0, __file__.rsplit('/', 1)[0])
    import leitura_nav

    class Corrida(Node):
        def __init__(self):
            super().__init__('corrida_nav')
            self.alvo = (a.alvo[0], a.alvo[1])
            self.linhas = []
            self.ts_pose = []
            self.t0 = None
            self.pose = None
            self.plano_n = 0
            self.cmd = {'raw': (0.0, 0.0), 'saida': (0.0, 0.0),
                        'mux': (0.0, 0.0), 'key': (0.0, 0.0)}

            qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Odometry, '/Odometry', self.cb_odom,
                                     qos_profile_sensor_data)
            self.create_subscription(Path, '/plan', self.cb_plano, qos)
            for topico, campo in (('/auto_vel_raw', 'raw'),
                                  ('/auto_vel', 'saida'),
                                  ('/compensador_rumo/cmd_vel', 'mux'),
                                  ('/key_vel', 'key')):
                self.create_subscription(
                    TwistStamped, topico,
                    lambda m, c=campo: self.cb_cmd(c, m), qos)

            self.cliente = ActionClient(self, NavigateToPose, 'navigate_to_pose')
            self.objetivo = None
            self.create_timer(1.0 / 20.0, self.passo)

        # ------------------------------------------------------- entradas
        def agora(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def cb_odom(self, msg):
            self.pose = msg
            self.ts_pose.append(self.agora())

        def cb_plano(self, msg):
            self.plano_n = len(msg.poses)

        def cb_cmd(self, campo, msg):
            self.cmd[campo] = (msg.twist.linear.x, msg.twist.angular.z)

        # --------------------------------------------------------- corrida
        def manda(self):
            print('esperando o servidor `navigate_to_pose`...')
            if not self.cliente.wait_for_server(timeout_sec=10.0):
                print('🔴 o bt_navigator não respondeu em 10 s. A pilha está de '
                      'pé? (tools/banco/checa_pilha.py)')
                return False
            objetivo = NavigateToPose.Goal()
            objetivo.pose.header.frame_id = a.frame
            objetivo.pose.header.stamp = self.get_clock().now().to_msg()
            objetivo.pose.pose.position.x = a.alvo[0]
            objetivo.pose.pose.position.y = a.alvo[1]
            rumo = math.radians(a.rumo) if a.rumo is not None else 0.0
            objetivo.pose.pose.orientation.z = math.sin(rumo / 2.0)
            objetivo.pose.pose.orientation.w = math.cos(rumo / 2.0)
            self.futuro = self.cliente.send_goal_async(objetivo)
            print(f'objetivo mandado: ({a.alvo[0]:.2f} · {a.alvo[1]:.2f}) '
                  f'no frame {a.frame}. TECLADO NA MÃO.')
            return True

        def passo(self):
            if self.pose is None:
                return
            t = self.agora()
            if self.t0 is None:
                self.t0 = t
            p = self.pose.pose.pose
            self.linhas.append({
                't': round(t - self.t0, 3),
                'x': round(p.position.x, 4), 'y': round(p.position.y, 4),
                'yaw': round(yaw_de(p.orientation), 4),
                'dist': round(leitura_nav.distancia(
                    p.position.x, p.position.y, self.alvo), 4),
                'plano_n': self.plano_n,
                'raw_v': round(self.cmd['raw'][0], 4),
                'raw_wz': round(self.cmd['raw'][1], 4),
                'saida_v': round(self.cmd['saida'][0], 4),
                'saida_wz': round(self.cmd['saida'][1], 4),
                'mux_v': round(self.cmd['mux'][0], 4),
                'mux_wz': round(self.cmd['mux'][1], 4),
                'key_v': round(self.cmd['key'][0], 4),
            })

        def cancela(self):
            """Tira o objetivo de pé antes de sair. NÃO é freio — ver o
            cabeçalho: quem freia é o teclado."""
            try:
                alca = self.futuro.result()
                if alca is not None and alca.accepted:
                    alca.cancel_goal_async()
                    print('objetivo CANCELADO.')
            except Exception:
                pass

        def fecha(self):
            if not self.linhas:
                print('nada gravado — /Odometry chegou? a base está de pé?')
                return
            with open(a.csv, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=list(self.linhas[0].keys()))
                w.writeheader()
                w.writerows(self.linhas)
            print(f'\n{len(self.linhas)} amostras -> {a.csv}\n')
            m = leitura_nav.metricas(self.linhas, self.alvo, a.raio,
                                     self.ts_pose)
            for linha in leitura_nav.veredito(m, a.raio):
                print(linha)

    rclpy.init()
    no = Corrida()
    try:
        if no.manda():
            fim = no.agora() + a.teto_s
            while rclpy.ok() and no.agora() < fim:
                rclpy.spin_once(no, timeout_sec=0.1)
            print(f'\nteto de {a.teto_s:.0f} s atingido.')
    except KeyboardInterrupt:
        print('\ninterrompido.')
    finally:
        no.cancela()
        no.fecha()
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
