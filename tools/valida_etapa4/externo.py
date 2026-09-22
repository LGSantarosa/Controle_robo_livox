#!/usr/bin/env python3
"""Um nó ROS de verdade com o NOME pedido e nada mais — etapa 4, passo 7.

    exec -a /opt/ros/jazzy/lib/joy/joy_node python3 externo.py joy_node

Faz o papel do robô 2 ou do simulador no mesmo PC (§10.5): tem o nome de nó e
o argv que o `sobe-robo3` antigo mataria por nome. Não publica nem assina
nada; só existe no grafo do seu domínio até receber INT/TERM.
"""
import signal
import sys

import rclpy
from rclpy.node import Node


def main():
    nome = sys.argv[1]
    rclpy.init()
    no = Node(nome)
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(no)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
