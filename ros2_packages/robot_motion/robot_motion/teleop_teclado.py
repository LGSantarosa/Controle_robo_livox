#!/usr/bin/env python3
"""Teleop de teclado do robô 2 — o freio de mão do humano.

Publica `TwistStamped` em `~/cmd_vel` (a launch remapeia para `/key_vel`), que
entra no `twist_mux` com prioridade ACIMA da autonomia. É o caminho pelo qual
uma pessoa toma o controle de um robô que está indo para a parede.

    ros2 run robot_motion teleop_teclado          # ou: bin/robot-key

        w / s     frente / ré
        a / d     girar esquerda / direita
        espaço    PARA agora
        q         sai (publicando zero)

## Duas coisas que este teleop faz diferente, e as duas vêm de medida

1. **NÃO tem passo de velocidade.** O `teleop_twist_keyboard` de fábrica tem
   teclas para subir e descer a velocidade; aqui elas seriam mentira. A
   compensação de zona morta do driver entrega um PATAMAR (~0,30 m/s de borda):
   comandar 0,10 ou 0,50 dá a mesma coisa na placa, medido em 31-07 e 04-08.
   O que o comando escolhe é SENTIDO, não módulo.

2. **É homem-morto.** Sem tecla nova por `solta` segundos o comando vai a zero
   sozinho. Terminal não avisa quando a tecla é SOLTA, então um teleop que
   repete o último comando para sempre é um robô que continua andando depois
   de a pessoa largar o teclado — o oposto de um freio de mão. Soltar tudo é o
   estado seguro, e é para onde ele cai sozinho.

⚠️ Sem tecla nenhuma este nó publica ZERO continuamente, e não silêncio: no
`twist_mux`, fonte que cala é fonte que some, e some devolveria o robô à
autonomia. Zero explícito é o que segura o robô parado enquanto o humano
estiver com a mão nele.
"""
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

TECLAS = {
    'w': (1.0, 0.0),
    's': (-1.0, 0.0),
    'a': (0.0, 1.0),
    'd': (0.0, -1.0),
    ' ': (0.0, 0.0),
}


class TeleopTeclado(Node):
    def __init__(self):
        super().__init__('teleop_teclado')
        p = self.declare_parameters('', [
            # Um valor só, e qualquer um serve dentro do patamar — ver o
            # cabeçalho. Fica parametrizado porque no dia em que a compensação
            # do driver mudar (item aberto do ESTADO) o módulo volta a existir.
            ('v', 0.25),
            ('wz', 0.6),
            ('solta', 0.4),      # [s] sem tecla -> para sozinho
            ('taxa', 20.0),
        ])
        self.par = {x.name: x.value for x in p}
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, '~/cmd_vel', qos)
        self.cmd = (0.0, 0.0)
        self.t_tecla = None
        self.create_timer(1.0 / self.par['taxa'], self.passo)
        print(__doc__.split('## ')[0].split('\n\n', 1)[1])
        print('>>> homem-morto: soltou o teclado, o robô para.\n')

    def le_tecla(self):
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return None

    def passo(self):
        t = self.get_clock().now().nanoseconds * 1e-9
        k = self.le_tecla()
        if k == 'q':
            self.publica(0.0, 0.0)
            raise KeyboardInterrupt
        if k in TECLAS:
            self.cmd = TECLAS[k]
            self.t_tecla = t
        elif self.t_tecla is not None and t - self.t_tecla > self.par['solta']:
            self.cmd = (0.0, 0.0)
        self.publica(self.cmd[0] * self.par['v'], self.cmd[1] * self.par['wz'])

    def publica(self, v, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(wz)
        self.pub.publish(m)


def main():
    ajustes = termios.tcgetattr(sys.stdin)
    rclpy.init()
    no = TeleopTeclado()
    try:
        tty.setcbreak(sys.stdin.fileno())
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        # Restaurar o terminal ANTES de derrubar o nó: se o spin morrer com
        # exceção, terminal em cbreak deixa o ssh do dono inutilizável.
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, ajustes)
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
