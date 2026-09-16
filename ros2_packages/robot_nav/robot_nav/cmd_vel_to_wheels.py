#!/usr/bin/env python3
"""
Converts geometry_msgs/Twist (/cmd_vel) into wheel_msgs/WheelSpeeds
(/wheel_vel_setpoints for the hoverboard driver).

Cinemática diferencial amarrada à geometria do robô:
  v_left  = linear * linear_sign - angular * wheel_base / 2     (m/s)
  v_right = linear * linear_sign + angular * wheel_base / 2     (m/s)
  cmd_left  = v_left  * linear_scale * left_wheel_sign
  cmd_right = v_right * linear_scale * right_wheel_sign

`linear_scale` é a constante única que converte m/s nas unidades internas
do driver (~ -1000..1000). `wheel_base` é a bitola física — assim
`angular = 1 rad/s` produz, depois da conversão em /odom, exatamente
1 rad/s medido (sem mais magic number `angular_scale`).

`left_wheel_sign` / `right_wheel_sign` calibram polaridade. Devem casar
com os mesmos parâmetros do `odom_publisher` — se inverter aqui sem
inverter lá (ou vice-versa) o AMCL/EKF enxerga divergência entre comando
e feedback. Por isso a inversão antiga "fios trocados" foi removida.

`linear_sign` = -1.0 é o robô DE COSTAS: um giro de 180° do base_link.
Só a linear troca de sinal; o giro fica como está, e para quem dirige
esquerda continua esquerda — porque virar o robô troca também qual roda
é a esquerda, e as duas inversões se cancelam no `angular`.

NÃO confundir com inverter os DOIS `*_wheel_sign` juntos: aquilo é um
ESPELHO, inverte frente E giro (e por isso o giro sai trocado para quem
dirige). Rotação ≠ reflexão. Ver decisão 049.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from wheel_msgs.msg import WheelSpeeds

from .utils import spin_node


class CmdVelToWheels(Node):

    def __init__(self):
        super().__init__('cmd_vel_to_wheels')

        self.declare_parameter('wheel_base', 0.50)
        self.declare_parameter('linear_scale', 400.0)
        self.declare_parameter('left_wheel_sign', 1.0)
        self.declare_parameter('right_wheel_sign', 1.0)
        # -1.0 = robô de costas (frente ↔ ré), sem mexer no giro. Default 1.0:
        # quem não passa o parâmetro não sente diferença nenhuma.
        self.declare_parameter('linear_sign', 1.0)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('max_output', 1000.0)

        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.linear_scale = float(self.get_parameter('linear_scale').value)
        self.left_sign = float(self.get_parameter('left_wheel_sign').value)
        self.right_sign = float(self.get_parameter('right_wheel_sign').value)
        self.linear_sign = float(self.get_parameter('linear_sign').value)
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.max_output = float(self.get_parameter('max_output').value)

        self.sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self._cmd_vel_callback,
            10
        )

        self.pub = self.create_publisher(WheelSpeeds, 'wheel_vel_setpoints', 10)

        self.get_logger().info(
            f'CmdVelToWheels: listening on /{self.cmd_vel_topic} '
            f'| wheel_base={self.wheel_base} m '
            f'| linear_scale={self.linear_scale} units/(m/s) '
            f'| signs L={self.left_sign} R={self.right_sign} '
            f'| linear_sign={self.linear_sign}'
            f'{" (DE COSTAS: frente ↔ ré)" if self.linear_sign < 0 else ""}'
        )

    def _cmd_vel_callback(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z

        if not (math.isfinite(linear) and math.isfinite(angular)):
            self.get_logger().warn(
                f'cmd_vel não-finito (linear={linear}, angular={angular}); ignorando',
                throttle_duration_sec=1.0,
            )
            return

        # O 180°: só a linear vira. O `angular` NÃO — ver o cabeçalho.
        linear *= self.linear_sign

        v_left = linear - angular * self.wheel_base / 2.0
        v_right = linear + angular * self.wheel_base / 2.0

        left = v_left * self.linear_scale * self.left_sign
        right = v_right * self.linear_scale * self.right_sign

        peak = max(abs(left), abs(right))
        if peak > self.max_output:
            scale = self.max_output / peak
            left *= scale
            right *= scale

        wheels = WheelSpeeds()
        wheels.left_wheel = float(left)
        wheels.right_wheel = float(right)
        self.pub.publish(wheels)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToWheels()
    try:
        spin_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
