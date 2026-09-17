#!/usr/bin/env python3
"""Publica a TF `odom → base_link` a partir do `/Odometry` do FAST-LIO.

    ros2 run robot_base tf_odom

## Por que isto existe

O FAST-LIO publica a pose como **mensagem** em `/Odometry`, e não como
transformada. No simulador quem publicava `odom → base_link` era o Gazebo; no
robô real **ninguém publica**, e a árvore TF fica partida em dois pedaços
desconexos:

    (URDF)      base_link → livox_frame → ...
    (fast_lio)  camera_init → body

Medido no robô em 06-08:

    odom -> base_link        : "frame does not exist" / árvores desconectadas
    base_link -> livox_frame : OK, 0,420 m
    /livox/lidar             : 8 Hz, nuvem viva

O estrago é em cascata e não parece um problema de TF quando acontece:

1. o `planner_server` do Nav2 não ativa (`map → base_link` impossível);
2. o `lifecycle_manager` **aborta o bringup inteiro** quando um nó falha;
3. o `collision_monitor` está na mesma lista e **nunca é ativado** — ativado na
   mão, ele recebe em `/auto_vel_raw` e não publica **nem zero** em `/auto_vel`,
   porque precisa transformar a nuvem para o corpo (`base_shift_correction`);
4. o robô fica parado sem uma linha de erro que diga "falta uma TF".

Foi assim que o teste D caiu em 06-08. Ver `docs/DIARIO.md` (06-08, 3ª leva).

## A composição, que é a única parte não-óbvia

⚠️ **`/Odometry` NÃO é necessariamente a pose do `base_link`.** O FAST-LIO
localiza o **sensor**, e o Mid-360 está a 42 cm do chão (medido com trena em
05-08). Publicar a pose do sensor como se fosse a do corpo embutiria esse
offset em toda a navegação — e o erro seria silencioso, porque nada reclama de
uma TF *presente e errada*.

Então:

    T(odom → base_link)  =  T(odom → pose)  ∘  T(pose → base_link)
                            ↑ da mensagem     ↑ do URDF, via tf2

O `child_frame_id` da própria mensagem diz de que frame é a pose. Se ele já for
o `base_link`, a segunda parcela é identidade e a composição não faz nada — o
código trata os dois casos sem precisar saber de antemão qual é.

🛑 **Se a transformada intermediária não existir, este nó NÃO publica.** Uma TF
ausente trava a navegação de um jeito visível; uma TF errada deixa tudo rodar
entregando lugar errado. O segundo é pior, então o silêncio aqui é deliberado —
e vem com log dizendo exatamente o que faltou.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformBroadcaster, TransformListener

from robot_base.congela_parado import CongelaParado
from robot_base.transformadas import compoe, inverte


class TfOdom(Node):
    def __init__(self):
        super().__init__('tf_odom')
        p = self.declare_parameters('', [
            ('topico', '/Odometry'),
            ('frame_pai', 'odom'),
            ('frame_filho', 'base_link'),
            # Vazio = confiar no child_frame_id da mensagem. Preencher só se o
            # FAST-LIO vier com um child_frame_id que não existe na árvore do
            # URDF (já aconteceu de upstream mandar 'body').
            ('frame_da_pose', ''),
            # Robô 2 só (decisão 050): com as rodas paradas a TF não se mexe,
            # e o AMCL não reamostra com o robô parado — o pulo no mapa.
            ('congela_parado', False),
        ])
        self.par = {x.name: x.value for x in p}
        self.congela = None
        if self.par['congela_parado']:
            self.congela = CongelaParado()
            self.v = [0.0, 0.0]
            for i, lado in enumerate(('left', 'right')):
                self.create_subscription(
                    Float64, f'/hoverboard/{lado}_wheel/velocity',
                    lambda m, i=i: self.roda(i, m.data), 10)
            self.get_logger().warn(
                'congela_parado: rodas paradas = odom->base_link parado.')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.br = TransformBroadcaster(self)
        self.avisou = set()
        self.n = 0
        self.create_subscription(
            Odometry, self.par['topico'], self.passo,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().warn(
            f"tf_odom: {self.par['topico']} -> TF "
            f"{self.par['frame_pai']} -> {self.par['frame_filho']}. "
            'A pose do LIO é do SENSOR; a composição com o URDF é o que impede '
            'os 42 cm do Mid-360 de virarem erro silencioso de navegação.')

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def roda(self, i, v):
        self.v[i] = v
        self.congela.rodas(self.agora(), *self.v)

    def uma_vez(self, chave, msg):
        """Loga uma vez por causa — este callback roda a 10 Hz."""
        if chave not in self.avisou:
            self.avisou.add(chave)
            self.get_logger().error(msg)

    def passo(self, m):
        pose_frame = self.par['frame_da_pose'] or m.child_frame_id
        alvo = self.par['frame_filho']
        if not pose_frame:
            self.uma_vez('sem_child', (
                'A mensagem não traz child_frame_id e o parâmetro '
                '`frame_da_pose` está vazio — não dá para saber de que frame é '
                'esta pose. Passe -p frame_da_pose:=<frame>.'))
            return

        t = (m.pose.pose.position.x, m.pose.pose.position.y,
             m.pose.pose.position.z)
        q = (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
             m.pose.pose.orientation.z, m.pose.pose.orientation.w)

        if pose_frame != alvo:
            # T(pose → base_link), do URDF. Estática, mas buscada a cada ciclo
            # de propósito: se o URDF mudar, isto acompanha.
            try:
                tr = self.buffer.lookup_transform(
                    pose_frame, alvo, rclpy.time.Time()).transform
            except Exception as e:  # noqa: BLE001 - tf2 lança várias classes
                self.uma_vez('sem_tf', (
                    f'NÃO EXISTE TF {pose_frame} -> {alvo}: {e}\n'
                    'Sem ela a pose do sensor viraria a pose do corpo, com o '
                    'offset embutido e SEM sintoma. Este nó não vai publicar. '
                    'Confira se o robot_state_publisher está de pé e se o '
                    f"URDF descreve '{pose_frame}'."))
                return
            t_s = (tr.translation.x, tr.translation.y, tr.translation.z)
            q_s = (tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w)
            t, q = compoe(t, q, t_s, q_s)

            # 🔴 E AGORA O `odom` VAI PARA O CHÃO (10-08).
            #
            # Compor só à direita põe o `base_link` no lugar certo em relação
            # ao sensor, mas deixa a ORIGEM do `odom` onde o LIO a criou: em
            # cima do Mid-360, a 0,42 m. Medido no robô:
            #
            #     tf2_echo odom base_link  ->  z = −0,477 m
            #
            # ou seja, o robô 48 cm ABAIXO da origem do próprio odom (os 42 cm
            # da trena mais a deriva de z do LIO). A TF está "certa" no sentido
            # de descrever a geometria, e errada como convenção: o chão passa a
            # ficar em z ≈ −0,42.
            #
            # Quem quebra com isso é a percepção, em dois lugares independentes
            # do Nav2, os dois no frame GLOBAL:
            #   · `min_obstacle_height`/`max_obstacle_height` (0,10–0,50 m)
            #     rejeitam a nuvem inteira, que chega em −0,42 ± altura;
            #   · a `VoxelLayer` descarta o que está abaixo do `origin_z` dela.
            # Sintoma: costmap com ZERO células letais e nuvem perfeita.
            #
            # Pré-compor com a inversa põe a origem do `odom` onde o base_link
            # estava na largada — o chão volta para z ≈ 0. Para o robô no plano
            # isso é tirar os 42 cm; a forma geral (com a inversa de verdade)
            # existe porque em rampa a mesma conta não é uma subtração.
            t_i, q_i = inverte(t_s, q_s)
            t, q = compoe(t_i, q_i, t, q)

        if self.congela is not None:
            t, q = self.congela.passo(self.agora(), t, q)

        msg = TransformStamped()
        msg.header.stamp = m.header.stamp
        msg.header.frame_id = self.par['frame_pai']
        msg.child_frame_id = alvo
        msg.transform.translation.x, msg.transform.translation.y, \
            msg.transform.translation.z = t
        msg.transform.rotation.x, msg.transform.rotation.y, \
            msg.transform.rotation.z, msg.transform.rotation.w = q
        self.br.sendTransform(msg)

        self.n += 1
        if self.n == 1:
            self.get_logger().warn(
                f"primeira TF publicada (pose vinha de '{pose_frame}'"
                + (', composta com o URDF)' if pose_frame != alvo
                   else ', já era o corpo — sem composição)'))


def main():
    rclpy.init()
    no = TfOdom()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
