#!/usr/bin/env python3
"""Bancada do planner: dois cliques, dois caminhos, uma tabela de números.

O robô NÃO se move — nem existe. Você marca de onde sai ("2D Pose Estimate") e
para onde vai ("2D Goal Pose"), e este nó pede o caminho a CADA planner
configurado, publica os dois para o RViz desenhar e mede os dois.

    ros2 launch robot_planning bancada_planner.launch.py

Por que existe: o defeito que motivou trazer o Nav2 ("ele dá um puta balão
para chegar num goal do lado") nasceu na camada de movimentação, não no
planejamento. Julgar o planner junto com quem o executa mistura as culpas.
Aqui só o desenho do caminho está em jogo.

Tópicos:
    entra  /initialpose   geometry_msgs/PoseWithCovarianceStamped   de onde sai
           /goal_pose     geometry_msgs/PoseStamped                 para onde vai
    sai    ~/plano_<id>   nav_msgs/Path                             um por planner

As medidas de cada caminho são impressas no log e servem para comparar sem
depender de "achei mais bonito":

  comprimento   quanto ele anda de fato [m], contra a linha reta
  desvio        comprimento / distância em linha reta (1,0 = reta perfeita)
  giro total    soma dos ângulos de virada [graus] — quanto ele mexe o bico
  raio mínimo   curva mais fechada do caminho [m]; abaixo do que a máquina
                faz, o caminho é bonito e inseguível
  inversões     quantas vezes o caminho troca de sentido (ré do Reeds-Shepp)
  tempo         quanto o planner levou [ms]
"""
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


def mede(caminho):
    """Números de um caminho, para comparar planners sem depender de gosto."""
    pts = [(p.pose.position.x, p.pose.position.y) for p in caminho.poses]
    if len(pts) < 3:
        # Caminho de 1 ou 2 pontos é reta pura — o planner achou o destino
        # trivial. É resultado válido, não falha: relatar como falha aqui
        # fazia a tabela dizer "SEM CAMINHO" para uma reta de 1 m.
        comp = (math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
                if len(pts) == 2 else 0.0)
        return {'pontos': len(pts), 'comprimento': comp, 'reta': comp,
                'desvio': 1.0, 'giro_deg': 0.0, 'raio_min': float('inf'),
                'inversoes': 0}

    comp = sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pts, pts[1:]))
    reta = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])

    # Curvatura tem que ser medida em passo LONGO. O Smac entrega o caminho
    # suavizado com pontos a ~5 cm, e três pontos vizinhos assim medem ruído
    # de arredondamento, não a curva: com raio mínimo de 0,25 m configurado no
    # planner, a conta ponto-a-ponto acusava 0,01 m. Reamostrado a 0,20 m o
    # número volta a descrever a geometria.
    ralos = [pts[0]]
    for p in pts[1:]:
        if math.hypot(p[0] - ralos[-1][0], p[1] - ralos[-1][1]) >= 0.20:
            ralos.append(p)
    if len(ralos) < 3:
        ralos = pts

    giro = 0.0
    raio_min = float('inf')
    inversoes = 0
    rumo_ant = None
    for a, b, c in zip(ralos, ralos[1:], ralos[2:]):
        r1 = math.atan2(b[1] - a[1], b[0] - a[0])
        r2 = math.atan2(c[1] - b[1], c[0] - b[0])
        d = abs(norm_ang(r2 - r1))
        giro += d
        # Inversão de sentido: o caminho "dobra sobre si mesmo" (>150°). É
        # assim que a ré do Reeds-Shepp aparece na geometria do caminho.
        if rumo_ant is not None and abs(norm_ang(r1 - rumo_ant)) > math.radians(150):
            inversoes += 1
        rumo_ant = r1
        # Raio da curva por três pontos: dois segmentos e o ângulo entre eles.
        passo = math.hypot(c[0] - b[0], c[1] - b[1])
        if d > 1e-6 and passo > 1e-6:
            raio_min = min(raio_min, passo / (2.0 * math.sin(min(d, math.pi) / 2.0)))

    return {
        'pontos': len(pts),
        'comprimento': comp,
        'reta': reta,
        'desvio': comp / reta if reta > 1e-6 else float('inf'),
        'giro_deg': math.degrees(giro),
        'raio_min': raio_min,
        'inversoes': inversoes,
    }


class BancadaPlanner(Node):
    def __init__(self):
        super().__init__('bancada_planner')
        p = self.declare_parameters('', [
            # Ids dos planners configurados no planner_server. A ordem manda
            # na ordem da tabela do log.
            ('planners', ['theta', 'hibrido']),
            # Raio de curva que a máquina entrega [m]. Só para o log dizer se
            # o caminho é seguível — MEDIDO em 28-07 no simulador, e refém do
            # teto de giro, que não foi medido.
            ('raio_da_maquina', 0.23),
            ('quadro', 'map'),
        ])
        self.par = {x.name: x.value for x in p}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pubs = {pid: self.create_publisher(Path, f'~/plano_{pid}', qos)
                     for pid in self.par['planners']}
        self.cliente = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose',
                                 self.cb_inicio, qos)
        self.create_subscription(PoseStamped, '/goal_pose', self.cb_destino, qos)

        # O costmap tem que ter RECEBIDO o mapa antes do primeiro pedido. Sem
        # esperar por ele, a primeira requisição depois da subida volta com
        # caminho VAZIO e código de sucesso — medido nas duas primeiras
        # corridas desta bancada, sempre no primeiro caso e nunca nos
        # seguintes. Parece defeito do planner e é corrida de inicialização.
        self.costmap_ok = False
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap',
            lambda _m: setattr(self, 'costmap_ok', True),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.inicio = None
        self.resultados = {}
        self.pedidos = 0

        # Só anuncia depois que o planner_server está de pé. O Smac Hybrid
        # leva ~16 s montando a tabela de heurística na subida, e clicar antes
        # disso devolve um erro que parece defeito da bancada.
        self.get_logger().info(
            'esperando o planner_server e o costmap (o Smac demora ~16 s '
            'montando a heurística)...')
        self.create_timer(1.0, self.checa_servidor)
        self.pronta = False

    def checa_servidor(self):
        if self.pronta:
            return
        if not self.cliente.server_is_ready() or not self.costmap_ok:
            return
        self.pronta = True
        self.get_logger().info(
            'BANCADA PRONTA. No RViz: "2D Pose Estimate" marca a PARTIDA, '
            '"2D Goal Pose" marca o DESTINO. O robô não se move — só o '
            f"caminho é desenhado. Planners: {', '.join(self.par['planners'])}")

    def cb_inicio(self, msg):
        self.inicio = PoseStamped()
        self.inicio.header = msg.header
        self.inicio.pose = msg.pose.pose
        self.get_logger().info(
            f'PARTIDA em ({self.inicio.pose.position.x:.2f}, '
            f'{self.inicio.pose.position.y:.2f}), '
            f'{math.degrees(yaw_de(self.inicio.pose.orientation)):.0f}°')

    def cb_destino(self, msg):
        if self.inicio is None:
            self.get_logger().warn(
                'marque a PARTIDA primeiro ("2D Pose Estimate") — sem ela não '
                'há de onde planejar, e não existe robô para perguntar')
            return
        if not self.cliente.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                'planner_server não respondeu. Ele subiu e está ATIVO? '
                '(lifecycle_manager com autostart)')
            return

        self.resultados = {}
        self.pedidos = len(self.par['planners'])
        self.destino = msg
        self.fila = list(self.par['planners'])
        self.get_logger().info(
            f'DESTINO em ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
            f" — pedindo caminho a {self.pedidos} planner(s)")
        self.pede_proximo()

    def pede_proximo(self):
        """UM pedido por vez.

        O `planner_server` atende um objetivo de cada vez: mandar os dois
        juntos faz o segundo PREEMPTAR o primeiro, e o preemptado volta com
        caminho vazio e código de SUCESSO — ou seja, mentindo. Foi assim que a
        bancada acusou "Theta* sem caminho" no primeiro clique de cada corrida
        (frio, ele demorava mais e era atropelado pelo Smac; quente, escapava).
        """
        if not self.fila:
            return
        pid = self.fila.pop(0)
        objetivo = ComputePathToPose.Goal()
        objetivo.start = self.inicio
        objetivo.goal = self.destino
        objetivo.goal.header.frame_id = self.par['quadro']
        objetivo.start.header.frame_id = self.par['quadro']
        objetivo.planner_id = pid
        objetivo.use_start = True         # planeja entre os DOIS cliques
        fut = self.cliente.send_goal_async(objetivo)
        fut.add_done_callback(lambda f, pid=pid: self.cb_aceito(f, pid))

    def cb_aceito(self, futuro, pid):
        handle = futuro.result()
        if not handle.accepted:
            self.get_logger().error(f'{pid}: pedido RECUSADO pelo planner_server')
            self.chegou(pid, None, None)
            return
        handle.get_result_async().add_done_callback(
            lambda f, pid=pid: self.cb_resultado(f, pid))

    def cb_resultado(self, futuro, pid):
        res = futuro.result().result
        if res.error_code != 0:
            self.get_logger().error(
                f'{pid}: FALHOU — código {res.error_code} '
                f'({res.error_msg or "sem mensagem"})')
            self.chegou(pid, None, None)
            return
        ms = (res.planning_time.sec * 1000.0
              + res.planning_time.nanosec / 1e6)
        res.path.header.frame_id = self.par['quadro']
        self.pubs[pid].publish(res.path)
        self.chegou(pid, mede(res.path), ms)

    def chegou(self, pid, medida, ms):
        self.resultados[pid] = (medida, ms)
        if len(self.resultados) < self.pedidos:
            self.pede_proximo()
            return
        self.tabela()

    def tabela(self):
        """Imprime os dois caminhos lado a lado. Sem veredito — a escolha é do dono."""
        linhas = ['', 'planner      compr.  desvio   giro   raio min  inv  pts   tempo',
                  '-----------------------------------------------------------------']
        for pid in self.par['planners']:
            medida, ms = self.resultados.get(pid, (None, None))
            if medida is None:
                linhas.append(f'{pid:<12} SEM CAMINHO (o planner recusou este par)')
                continue
            r = medida['raio_min']
            if math.isinf(r):
                raio, marca = '  reto', ''
            else:
                raio = f'{r:6.2f}'
                marca = '' if r >= self.par['raio_da_maquina'] else '  <-- APERTADO'
            linhas.append(
                f"{pid:<12} {medida['comprimento']:5.2f}m  "
                f"{medida['desvio']:5.2f}x  {medida['giro_deg']:5.0f}°  "
                f"{raio}m  {medida['inversoes']:3d} {medida['pontos']:4d}  "
                f'{ms:5.0f}ms{marca}')
        reta = next((m['reta'] for m, _ in self.resultados.values() if m), 0.0)
        linhas.append(f'(reta pura: {reta:.2f} m · a máquina fecha '
                      f"{self.par['raio_da_maquina']:.2f} m de raio)")
        self.get_logger().info('\n'.join(linhas))


def main():
    rclpy.init()
    no = BancadaPlanner()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
