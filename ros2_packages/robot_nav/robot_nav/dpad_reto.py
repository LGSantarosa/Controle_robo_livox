"""Direcional do Xbox = reta pura (giro zero), para o robô 3.

O dono, 14-09: no analógico dá para escapar um pouco para o lado sem querer, e
aí não se sabe se o desvio foi da mão ou do robô. Com o LB segurado, direcional
para cima anda reto e para baixo dá ré, com `angular.z = 0` exato.

Publica em `dpad_vel`, que o `twist_mux_robo3.yaml` põe ACIMA do analógico
(`joy_vel`): enquanto o direcional está apertado, ele manda. Soltou (ou soltou o
LB), publica um zero e emudece; o mux volta ao analógico no timeout.

Eixos medidos em 14-09 no notebook (`joy_dpad_204304.csv`): direcional
cima/baixo = eixo 7 (+1 cima), esquerda/direita = eixo 6 (+1 esquerda).
LB = botão 6, RB = botão 7 (turbo), iguais ao `teleop_xbox_robo3.yaml`.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


def reta_do_direcional(axes, buttons, eixo, botao_lb, botao_turbo,
                       vel, vel_turbo):
    """Velocidade linear pedida pelo direcional, ou None se ele não está ativo.

    Pura (sem ROS) para ser testada fora do robô.
    """
    def botao(i):
        return i < len(buttons) and buttons[i] == 1

    if not botao(botao_lb) or eixo >= len(axes):
        return None
    valor = axes[eixo]
    if abs(valor) < 0.5:
        return None
    escala = vel_turbo if botao(botao_turbo) else vel
    return escala if valor > 0 else -escala


class DpadReto(Node):
    def __init__(self):
        super().__init__('dpad_reto')
        self.declare_parameter('eixo', 7)
        self.declare_parameter('botao_lb', 6)
        self.declare_parameter('botao_turbo', 7)
        # Mesmas velocidades do analógico (scale_linear / _turbo do teleop).
        self.declare_parameter('vel', 0.30)
        self.declare_parameter('vel_turbo', 0.50)
        self.p = {k: self.get_parameter(k).value
                  for k in ('eixo', 'botao_lb', 'botao_turbo', 'vel', 'vel_turbo')}
        self.ativo = False
        self.pub = self.create_publisher(Twist, 'dpad_vel', 10)
        self.create_subscription(Joy, 'joy', self._joy, 10)
        self.get_logger().info(f'dpad_reto: {self.p}')

    def _joy(self, msg):
        v = reta_do_direcional(list(msg.axes), list(msg.buttons), **self.p)
        if v is None:
            if self.ativo:
                self.pub.publish(Twist())  # um zero ao soltar, depois cala
                self.ativo = False
            return
        t = Twist()
        t.linear.x = float(v)
        self.pub.publish(t)
        self.ativo = True


def main(args=None):
    rclpy.init(args=args)
    node = DpadReto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
