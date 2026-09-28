#!/usr/bin/env python3
"""Auxiliar do reprodutor do SIGSEGV do collision_monitor (decisão 057).

Roteiro fixo, relativo ao início deste nó:
  · sempre: TF estático base_link→livox_frame e nuvem em livox_frame a 10 Hz
    com carimbo atual (a fonte fica válida para o source_timeout);
  · 0–3 s: TF odom→base_link a 50 Hz;
  · 3,0 s: o TF odom→base_link PARA — linha TF_PARADO;
  · 3,5 s: UM TwistStamped em /repro/cmd_vel_in — linha CMD_PUBLICADO.

Com o TF parado, o process() dessa callback fica no laço do canTransform
esperando base_link no instante corrente. Encerra no SIGINT.
"""
import time

import rclpy
from geometry_msgs.msg import TransformStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

T_TF_PARA = 3.0
T_CMD = 3.5


class Auxiliar(Node):
    def __init__(self):
        super().__init__('repro_auxiliar')
        self.t0 = time.monotonic()
        self.tf_ligado = True
        self.cmd_enviado = False
        qos_static = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_static = self.create_publisher(TFMessage, '/tf_static', qos_static)
        self.pub_tf = self.create_publisher(TFMessage, '/tf', 100)
        self.pub_nuvem = self.create_publisher(PointCloud2, '/repro/pontos', 10)
        self.pub_cmd = self.create_publisher(TwistStamped, '/repro/cmd_vel_in', 1)
        self.pub_static.publish(TFMessage(transforms=[
            self._tf('base_link', 'livox_frame', self.get_clock().now().to_msg(), z=0.3)]))
        self.create_timer(0.02, self._passo_tf)
        self.create_timer(0.1, self._passo_nuvem)
        self._log('AUX_INICIO')

    def _log(self, evento):
        print(f'{evento} t={time.monotonic() - self.t0:.3f} wall={time.time():.6f}', flush=True)

    @staticmethod
    def _tf(pai, filho, stamp, z=0.0):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = pai
        t.child_frame_id = filho
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0
        return t

    def _passo_tf(self):
        dt = time.monotonic() - self.t0
        if self.tf_ligado and dt >= T_TF_PARA:
            self.tf_ligado = False
            self._log('TF_PARADO')
        if self.tf_ligado:
            self.pub_tf.publish(TFMessage(transforms=[
                self._tf('odom', 'base_link', self.get_clock().now().to_msg())]))
        if not self.cmd_enviado and dt >= T_CMD:
            self.cmd_enviado = True
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = 'base_link'
            m.twist.linear.x = 0.1
            self.pub_cmd.publish(m)
            self._log('CMD_PUBLICADO')

    def _passo_nuvem(self):
        h = Header(stamp=self.get_clock().now().to_msg(), frame_id='livox_frame')
        self.pub_nuvem.publish(point_cloud2.create_cloud_xyz32(
            h, [(3.0, 0.0, 0.0), (3.0, 0.5, 0.0), (3.0, -0.5, 0.0)]))


def main():
    rclpy.init()
    no = Auxiliar()
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    no._log('AUX_FIM')
    no.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
