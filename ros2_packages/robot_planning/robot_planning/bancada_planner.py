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
  curtos        trechos entre cúspides curtos demais para medir curvatura —
                não entram no raio nem no giro, e são contados para não sumir
                em silêncio. Muitos deles = o planner está tremendo no alvo.
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


def _rumo(a, b):
    """Direção do segmento a→b, ou None se os pontos coincidem."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    if math.hypot(dx, dy) < 1e-9:
        return None
    return math.atan2(dy, dx)


def cuspides(pts, yaws=None):
    """Índices onde o caminho TROCA DE SENTIDO (a ré do Reeds-Shepp).

    Duas maneiras, e a boa depende do planner:

    **Pela POSE**, quando o caminho traz orientação: projeta cada passo no rumo
    da pose e vê o sinal. Andar de ré é passo com projeção negativa, e a troca
    de sinal é a cúspide. É o critério certo porque é o que a palavra significa.

    **Pela geometria** (dobra > 150°), quando não traz. O Theta* devolve o
    caminho inteiro com orientação zerada — medido em 29-07: faixa de yaw de
    0,0° em 144 pontos — então para ele não há pose que consultar. Não custa
    nada: ele é planner só-para-frente, não tem cúspide para achar.

    Por que não ficar só na geometria: ela erra nas pontas. No caso `lado_1m`
    com raio 0,25 o caminho é `+----------------+` pela pose — um passo à
    frente, 16 de ré, um à frente, DUAS cúspides. A dobra geométrica nelas mede
    147°, passa por baixo do limiar de 150°, e as duas sumiam. O preço aparecia
    no `giro`: 2 x 147° de virada que o robô não faz entravam na conta, e um
    caminho de 1,19 m era relatado com **437°** de giro.
    """
    idx = []
    if yaws is not None and len(yaws) == len(pts) and (
            max(yaws) - min(yaws)) > math.radians(1.0):
        sinal_ant = None
        for i in range(1, len(pts)):
            dx, dy = pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]
            if math.hypot(dx, dy) < 1e-9:
                continue
            proj = dx * math.cos(yaws[i]) + dy * math.sin(yaws[i])
            sinal = proj >= 0.0
            if sinal_ant is not None and sinal != sinal_ant:
                idx.append(i - 1)
            sinal_ant = sinal
        return idx

    for i in range(1, len(pts) - 1):
        r1, r2 = _rumo(pts[i - 1], pts[i]), _rumo(pts[i], pts[i + 1])
        if r1 is None or r2 is None:
            continue
        if abs(norm_ang(r2 - r1)) > math.radians(150.0):
            idx.append(i)
    return idx


def _yaws_de(poses):
    """Rumos das poses, ou None se o caminho não traz orientação utilizável."""
    try:
        return [yaw_de(p.pose.orientation) for p in poses]
    except AttributeError:
        return None


def _sem_tocos(pts, minimo):
    """Tira os segmentos curtos demais para definir direção.

    Mantém sempre o primeiro e o último ponto: o toco é costurado fora, o
    caminho não é encurtado. Comprimento e desvio seguem saindo dos pontos
    CRUS — só a medida de ângulo usa esta lista.
    """
    if minimo <= 0.0:
        return pts
    limpos = [pts[0]]
    for p in pts[1:-1]:
        if math.hypot(p[0] - limpos[-1][0], p[1] - limpos[-1][1]) >= minimo:
            limpos.append(p)
    if len(pts) > 1:
        while (len(limpos) > 1
               and math.hypot(pts[-1][0] - limpos[-1][0],
                              pts[-1][1] - limpos[-1][1]) < minimo):
            limpos.pop()
        limpos.append(pts[-1])
    return limpos


def mede(caminho):
    """Números de um caminho, para comparar planners sem depender de gosto.

    ⚠️ Cúspides são tratadas à parte. Medir curvatura por três pontos em cima de
    uma cúspide lê a dobra como curva fechadíssima e soma ~180° de giro que o
    robô não faz — ele inverte, não vira. Medido no caso real `perto_de_lado`
    com raio mínimo de 0,46 m: a régua acusava raio de 0,125 m, 181° de giro e
    ZERO inversões, as três erradas ao mesmo tempo e todas contra quem usa ré.
    Como a ré do Reeds-Shepp é o motivo de o Smac estar na disputa, isso cegava
    a bancada exatamente no assunto dela. Agora o caminho é PARTIDO nas
    cúspides e cada trecho é medido sozinho.
    """
    pts = [(p.pose.position.x, p.pose.position.y) for p in caminho.poses]
    yaws = _yaws_de(caminho.poses)
    if len(pts) < 3:
        # Caminho de 1 ou 2 pontos é reta pura — o planner achou o destino
        # trivial. É resultado válido, não falha: relatar como falha aqui
        # fazia a tabela dizer "SEM CAMINHO" para uma reta de 1 m.
        comp = (math.hypot(pts[1][0] - pts[0][0], pts[1][1] - pts[0][1])
                if len(pts) == 2 else 0.0)
        return {'pontos': len(pts), 'comprimento': comp, 'reta': comp,
                'desvio': 1.0, 'giro_deg': 0.0, 'raio_min': float('inf'),
                'inversoes': 0, 'trechos_curtos': 0}

    passos = sorted(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(pts, pts[1:]))
    comp = sum(passos)
    reta = math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1])
    meio_passo = passos[len(passos) // 2] / 2.0

    # O caminho é partido nas cúspides: cada trecho é um sentido de marcha, e
    # medir através da dobra é o defeito descrito no docstring.
    cortes = cuspides(pts, yaws)
    trechos = []
    ini = 0
    for c in cortes:
        trechos.append(pts[ini:c + 1])
        ini = c
    trechos.append(pts[ini:])

    # --- giro: somado nos pontos CRUS, dentro de cada trecho ---------------
    # Medido nos reamostrados, o giro de curva CONTÍNUA saía curto: um arco de
    # 90° virava 50°, porque as meias-viradas das duas pontas não têm vértice
    # onde aparecer. E o erro não era parelho entre os dois planners — o canto
    # vivo do Theta* tem vértice e era contado inteiro, enquanto o arco do Smac
    # perdia nas duas pontas. A coluna que mede "quanto ele mexe o bico"
    # favorecia o Smac, justo na comparação que ela existe para arbitrar.
    #
    # Somar no cru só é honesto se o passo cru for limpo, e ele é: medido em
    # 29-07, o Theta* anda 0,0500 m por passo (a resolução do mapa) com virada
    # MEDIANA de 0,00°, e o Smac 0,086 m com virada máxima de 19,9° — que dá
    # raio 0,249 m, o teto configurado, não ruído. A justificativa antiga da
    # reamostragem ("três pontos vizinhos medem ruído de arredondamento") não
    # se sustentou: o 0,01 m que ela dizia consertar era a cúspide.
    #
    # A exceção são os TOCOS DE PONTA: o planner cola a pose exata de partida e
    # de chegada no caminho discretizado, e sobra um segmento de poucos
    # milímetros em cada extremidade, fora do arco. Direção tirada de um toco
    # desses é lixo — no caso `lado_1m` com raio 0,25, os dois tocos (7,7 mm
    # cada) injetavam ±125,6° e o giro de um caminho de 1,19 m saía 447°.
    # Segmento abaixo de METADE do passo típico do caminho não define direção e
    # é costurado fora. Limite relativo, não absoluto: o passo do Theta* (0,05,
    # a resolução do mapa) e o do Smac (~0,08) são diferentes, e um número fixo
    # serviria a um e não ao outro.
    giro = 0.0
    for trecho in trechos:
        limpos = _sem_tocos(trecho, meio_passo)
        for a, b, c in zip(limpos, limpos[1:], limpos[2:]):
            r1, r2 = _rumo(a, b), _rumo(b, c)
            if r1 is None or r2 is None:
                continue
            giro += abs(norm_ang(r2 - r1))

    # --- raio mínimo: segue na poligonal reamostrada ------------------------
    # ⚠️ NÃO mexido nesta mudança, de propósito (uma correção por vez). Mas o
    # levantamento acima expôs um problema nele: a reamostragem a 0,20 m passa
    # POR CIMA do canto vivo do Theta* e devolve 0,37–0,39 m onde a virada real
    # é um canto — curvatura infinita, que robô sem pivô não segue de jeito
    # nenhum. Ou seja, o raio faz o Theta* parecer MAIS seguível do que ele é,
    # e o veredito de 29-07 (Smac ganha) é conservador, não otimista. Tratar
    # canto e arco como coisas diferentes é a próxima correção da régua.
    raio_min = float('inf')
    curtos = 0
    for trecho in trechos:
        if len(trecho) < 3:
            curtos += 1
            continue
        # Curvatura tem que ser medida em passo LONGO. O Smac entrega o caminho
        # suavizado com pontos a ~5 cm, e três pontos vizinhos assim medem ruído
        # de arredondamento, não a curva: com raio mínimo de 0,25 m configurado
        # no planner, a conta ponto-a-ponto acusava 0,01 m. Reamostrado a 0,20 m
        # o número volta a descrever a geometria.
        ralos = [trecho[0]]
        for p in trecho[1:]:
            if math.hypot(p[0] - ralos[-1][0], p[1] - ralos[-1][1]) >= 0.20:
                ralos.append(p)
        if len(ralos) < 3:
            # Trecho curto demais para a régua: menos de 0,40 m entre duas
            # cúspides. Antes daqui caía um `ralos = trecho`, que media
            # curvatura nos pontos CRUS e devolvia o ruído que a reamostragem
            # existe para evitar — no caso `bloco` com raio 0,46 isso virou
            # `raio_min = 0,00 m`, num caminho cujo trecho longo fecha 0,54 m.
            # Não há curvatura para ler num arco menor que a resolução: o
            # trecho é PULADO e contado em `trechos_curtos`, porque pular em
            # silêncio é como o defeito anterior sobreviveu tanto tempo.
            curtos += 1
            continue
        for a, b, c in zip(ralos, ralos[1:], ralos[2:]):
            r1, r2 = _rumo(a, b), _rumo(b, c)
            if r1 is None or r2 is None:
                continue
            d = abs(norm_ang(r2 - r1))
            # Raio da curva por três pontos: dois segmentos e o ângulo entre eles.
            passo = math.hypot(c[0] - b[0], c[1] - b[1])
            if d > 1e-6 and passo > 1e-6:
                raio_min = min(raio_min,
                               passo / (2.0 * math.sin(min(d, math.pi) / 2.0)))

    inversoes = len(cortes)

    return {
        'pontos': len(pts),
        'comprimento': comp,
        'reta': reta,
        'desvio': comp / reta if reta > 1e-6 else float('inf'),
        'giro_deg': math.degrees(giro),
        'raio_min': raio_min,
        'inversoes': inversoes,
        'trechos_curtos': curtos,
    }


class BancadaPlanner(Node):
    def __init__(self):
        super().__init__('bancada_planner')
        p = self.declare_parameters('', [
            # Ids dos planners configurados no planner_server. A ordem manda
            # na ordem da tabela do log.
            ('planners', ['theta', 'hibrido']),
            # Raio de curva que a máquina entrega [m]. Só para o log dizer se
            # o caminho é seguível.
            #
            # Era 0,23 até 29-07, tirado de uma curva solta de 28-07. A corrida
            # de bancada de 29-07 mediu o realizado (p5) em 0,370 m com zona
            # morta 0,10 e 0,463 m com 0,15 — o valor DEPENDE da zona morta,
            # que ainda não foi medida no robô. Fica no otimista; quem quiser o
            # pessimista passa o parâmetro. A varredura
            # (`tools/planner/varredura_raio.py`) é que responde direito: ela
            # roda a comparação inteira nos dois extremos.
            ('raio_da_maquina', 0.37),
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
        linhas = ['', 'planner      compr.  desvio   giro   raio min  inv  curt  pts   tempo',
                  '-----------------------------------------------------------------------']
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
                f"{raio}m  {medida['inversoes']:3d} "
                f"{medida['trechos_curtos']:4d}  {medida['pontos']:4d}  "
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
