#!/usr/bin/env python3
"""freeze_capture — coletor de diagnóstico do "robô burro / congela perto do goal".

Por que existe (2026-06-24): perto do ponto / diante de obstáculo o robô vai RETO
mesmo com o planner mandando contornar, gira só no lugar, e só para porque o
collision manda. Grava DOIS CSVs (controle_web/logs/) pra eu (assistente) ler
DEPOIS — nunca ao vivo:

⚠️ 20-08: OS TÓPICOS E O TIPO SÃO PARÂMETROS, e o default é a cadeia do ROBÔ 2.
Este nó veio do robô 1 e chegava mudo aqui (tipo `Twist` numa cadeia
`TwistStamped`, metade dos tópicos inexistente) — o porquê está no `__init__`.

1) freeze_capture.csv — a CADEIA de velocidade + odom + estados, 1 linha por msg:
     t_wall, topic, vx, wz, px, py, extra
     auto_vel_raw    : o que o heading_controller PEDIU (pré-reflexo)
     auto_vel        : o que SOBROU do collision_monitor
     compensador_rumo/cmd_vel : o que o twist_mux repassou (pós-humano)
     hoverboard_base_controller/cmd_vel : o que foi ao ATUADOR (robô)
     cmd_vel_bruto   : idem, no simulador (a placa fingida)
     joy_vel/key_vel/web_vel : o humano, se ele interferiu
     odom            : o que o robô FAZ (twist) + pose (px,py) — `/Odometry`
     collision_state : transições do reflexo, `AÇÃO:polígono` na col. extra
     goal_active     : transições (valor na col. extra)
   → orçamento do tempo parado (07-03): `bin/pause_budget.py freeze_capture.csv`
     atribui cada segundo parado-com-goal a uma causa (guard, collision, giro
     engolido, zona-morta, unstuck, follower quieto...) pra achar o vilão.

2) freeze_diag.csv — métricas DERIVADAS a ~5 Hz pra provar planner-vs-controller:
     t_wall, px, py, yaw_deg, plan_rel_deg, front_obst_m, cmd_nav_vx, cmd_nav_wz
     yaw_deg      : heading do robô no frame map (via TF map→base_link)
     plan_rel_deg : ângulo do /plan a ~0.5 m à frente RELATIVO ao heading do robô
                    (+ = planner quer virar à esquerda, − à direita; ~0 = seguindo).
                    Grande + robô indo reto = "planner grita contornar, robô força reto".
     front_obst_m : obstáculo mais próximo num setor ±15° à frente (do /scan)
     cmd_nav_vx/wz: último comando do controller (o que ele tenta fazer)

Sobe sozinho no nav2.launch.py. Read-only: só assina + grava arquivo.
"""
import os
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       QoSDurabilityPolicy)
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist, TwistStamped
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _norm(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class FreezeCapture(Node):
    def __init__(self):
        super().__init__('freeze_capture')
        out_dir = self.declare_parameter(
            'out_dir', 'controle_web/logs').get_parameter_value().string_value
        self.lookahead = self.declare_parameter('plan_lookahead', 0.5).value
        self.front_sector_deg = self.declare_parameter('front_sector_deg', 15.0).value
        p_chain, p_diag = self._open_csvs(out_dir)

        # 🔴 20-08 (robô 2) — ESTE NÓ NASCEU NO ROBÔ 1 E CHEGAVA MUDO AQUI.
        # Dois desencontros, os dois falhando em SILÊNCIO (nó vivo, CSV com
        # cabeçalho e nenhuma linha, que se lê como "não aconteceu nada"):
        #
        #   (a) a cadeia do robô 2 é `TwistStamped` inteira — `twist_mux.yaml`
        #       roda com `use_stamped: true` e o `collision_monitor` com
        #       `enable_stamped_cmd_vel: true`. Assinar `Twist` num publisher
        #       `TwistStamped` não é erro de DDS: é tipo diferente, e o
        #       casamento simplesmente não acontece. Zero mensagem, zero aviso;
        #   (b) metade dos tópicos do robô 1 não existe aqui: `follow_vel`,
        #       `auto_vel_pre`, `unstuck_vel`, `motion_guard/state`. Neste robô
        #       o seguidor fala rumo+velocidade em `Float64` para o
        #       `heading_controller`, e é ELE quem abre a cadeia de twist.
        #
        # A cadeia real, e o default dos parâmetros abaixo:
        #
        #     path_follower ──(rumo_alvo, velocidade_alvo)──▶ heading_controller
        #        ──/auto_vel_raw──▶ collision_monitor ──/auto_vel──▶ twist_mux
        #        ──/compensador_rumo/cmd_vel──▶ compensador ──▶ atuador
        #
        # 🔴 A PERGUNTA QUE ELE EXISTE PARA RESPONDER, aberta desde 20-08: na
        # porta a lei pede giro com 50° de erro e o robô não gira; e ele fica
        # 32 s com `v_alvo` em 0,50 andando 0,01 m/s. O CSV do seguidor grava o
        # que ele PEDE, nunca o que sai — então não havia como separar "a
        # movimentação não converteu" de "o reflexo cortou" de "a placa não
        # obedeceu". Com esta cadeia gravada, `bin/pause_budget.py` atribui
        # cada segundo parado a UMA camada.
        stamped = self.declare_parameter('stamped', True).value
        tipo = TwistStamped if stamped else Twist
        topicos = self.declare_parameter('topicos', [
            '/auto_vel_raw',                     # o que o heading_controller pediu
            '/auto_vel',                         # o que SOBROU do reflexo
            '/compensador_rumo/cmd_vel',         # o que o mux repassou
            '/hoverboard_base_controller/cmd_vel',   # o que foi ao atuador
            '/cmd_vel_bruto',                    # idem, no simulador
            '/joy_vel', '/key_vel', '/web_vel',  # o humano, se interferiu
        ]).value

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        for topic in topicos:
            self.create_subscription(tipo, topic, self._mk_twist(topic), qos)
        self.create_subscription(
            Odometry, self.declare_parameter('odom_topic', '/Odometry').value,
            self._on_odom, qos)
        self.create_subscription(
            Path, self.declare_parameter('plan_topic', '/plan').value,
            self._on_plan, qos)
        self.create_subscription(
            LaserScan, self.declare_parameter('scan_topic', '/scan').value,
            self._on_scan, qos)
        # O estado do reflexo é a testemunha direta: `polygon_name` diz QUAL
        # caixa disparou e `action_type` o que ela mandou fazer. Sem isto,
        # `auto_vel` zerado é indistinguível de `auto_vel_raw` já ter vindo
        # zerado num ciclo em que ninguém publicou.
        self.create_subscription(
            CollisionMonitorState,
            self.declare_parameter('estado_colisao_topic',
                                   '/collision_monitor_state').value,
            self._on_collision_state, qos)
        # estados (latched nos publishers) + goal ativo -> col. extra
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, 'follow_state',
            lambda m: self._log_extra('follow_state', m.data), latched)
        self.create_subscription(
            String, 'motion_guard/state',
            lambda m: self._log_extra('guard_state', m.data), latched)
        self._goal_active = {}
        for topic in ('navigate_to_pose/_action/status',
                      'navigate_through_poses/_action/status'):
            self.create_subscription(
                GoalStatusArray, topic,
                lambda m, t=topic: self._on_goal_status(t, m), 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # estado p/ o diag derivado
        self._plan = []           # [(x,y), ...] no frame map
        self._front = math.inf    # obstáculo à frente (m)
        self._cmd_nav = (0.0, 0.0)  # último cmd_vel_nav (vx, wz)

        self.create_timer(0.2, self._diag_tick)   # 5 Hz
        self.create_timer(2.0, self._flush)
        self.get_logger().info(f'freeze_capture: chain={p_chain} diag={p_diag}')

    def _open_csvs(self, out_dir):
        try:
            os.makedirs(out_dir, exist_ok=True)
            base = out_dir
        except OSError:
            base = '/tmp'
        self._f = open(os.path.join(base, 'freeze_capture.csv'), 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow(['t_wall', 'topic', 'vx', 'wz', 'px', 'py', 'extra'])
        self._fd = open(os.path.join(base, 'freeze_diag.csv'), 'w', newline='')
        self._wd = csv.writer(self._fd)
        self._wd.writerow(['t_wall', 'px', 'py', 'yaw_deg', 'plan_rel_deg',
                           'front_obst_m', 'cmd_nav_vx', 'cmd_nav_wz'])
        return (os.path.join(base, 'freeze_capture.csv'),
                os.path.join(base, 'freeze_diag.csv'))

    # ---- cadeia de velocidade (CSV 1) ----
    def _mk_twist(self, topic):
        def cb(m):
            t = m.twist if hasattr(m, 'twist') else m      # stamped ou cru
            if topic == 'cmd_vel_nav':
                self._cmd_nav = (t.linear.x, t.angular.z)
            self._w.writerow([f'{time.time():.3f}', topic,
                              f'{t.linear.x:.4f}', f'{t.angular.z:.4f}',
                              '', '', ''])
        return cb

    # Só as TRANSIÇÕES: o collision_monitor publica o estado a cada ciclo, e
    # gravar 20 linhas por segundo de "DO_NOTHING" afogaria o CSV justamente
    # nos trechos em que nada acontece.
    _ACOES = {0: 'DO_NOTHING', 1: 'STOP', 2: 'SLOWDOWN', 3: 'APPROACH',
              4: 'LIMIT'}

    def _on_collision_state(self, m):
        estado = (f'{self._ACOES.get(m.action_type, m.action_type)}'
                  f':{m.polygon_name or "-"}')
        if estado == getattr(self, '_estado_colisao', None):
            return
        self._estado_colisao = estado
        self._log_extra('collision_state', estado)

    def _on_odom(self, m):
        t = m.twist.twist
        p = m.pose.pose.position
        self._w.writerow([f'{time.time():.3f}', 'odom',
                          f'{t.linear.x:.4f}', f'{t.angular.z:.4f}',
                          f'{p.x:.3f}', f'{p.y:.3f}', ''])

    def _log_extra(self, topic, value):
        self._w.writerow([f'{time.time():.3f}', topic, '', '', '', '', value])

    def _on_goal_status(self, topic, m):
        # ACTIVE_STATUSES do nav2 = 1,2,3 (aceito/executando/cancelando)
        active = any(s.status in (1, 2, 3) for s in m.status_list)
        if self._goal_active.get(topic) != active:
            self._goal_active[topic] = active
            self._log_extra('goal_active',
                            '1' if any(self._goal_active.values()) else '0')

    # ---- entradas p/ o diag (CSV 2) ----
    def _on_plan(self, m):
        self._plan = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]

    def _on_scan(self, m):
        lim = math.radians(self.front_sector_deg)
        best = math.inf
        a = m.angle_min
        for r in m.ranges:
            if -lim <= a <= lim and m.range_min < r < m.range_max and math.isfinite(r):
                if r < best:
                    best = r
            a += m.angle_increment
        self._front = best

    def _plan_rel(self, px, py, yaw):
        pts = self._plan
        if len(pts) < 2:
            return math.nan
        di = min(range(len(pts)), key=lambda i: (pts[i][0] - px) ** 2 + (pts[i][1] - py) ** 2)
        acc = 0.0
        j = di
        while j + 1 < len(pts) and acc < self.lookahead:
            acc += math.hypot(pts[j + 1][0] - pts[j][0], pts[j + 1][1] - pts[j][1])
            j += 1
        tx, ty = pts[j]
        if math.hypot(tx - px, ty - py) < 1e-3:
            return math.nan
        return _norm(math.atan2(ty - py, tx - px) - yaw)

    def _diag_tick(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        px = tf.transform.translation.x
        py = tf.transform.translation.y
        yaw = _yaw_from_quat(tf.transform.rotation)
        rel = self._plan_rel(px, py, yaw)
        front = '' if math.isinf(self._front) else f'{self._front:.3f}'
        rel_s = '' if (rel is None or math.isnan(rel)) else f'{math.degrees(rel):.1f}'
        self._wd.writerow([f'{time.time():.3f}', f'{px:.3f}', f'{py:.3f}',
                           f'{math.degrees(yaw):.1f}', rel_s, front,
                           f'{self._cmd_nav[0]:.4f}', f'{self._cmd_nav[1]:.4f}'])

    def _flush(self):
        self._f.flush()
        self._fd.flush()

    def destroy_node(self):
        for f in (getattr(self, '_f', None), getattr(self, '_fd', None)):
            try:
                if f:
                    f.flush()
                    f.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = FreezeCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
