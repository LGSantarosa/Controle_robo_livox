#!/usr/bin/env python3
"""Nós fingidos para testar a captura sem Gazebo (usado só pelo test_captura).

    python3 nos_fingidos.py --comum /ns/no --ciclo /a/b:ativo --grafo /auxiliar

Cada nó declara parâmetros de tipos variados (aninhado, bool, lista de int);
os lifecycle declaram `footprint_padding`, como os costmaps do Nav2. Roda até
receber SIGINT (o teste manda SIGINT: o rclpy o trata e sai limpo).
"""
import argparse

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.lifecycle import Node as LifecycleNode
from rclpy.node import Node


def separa(completo):
    ns, nome = completo.rsplit('/', 1)
    return nome, ns or '/'


def declara(n):
    n.declare_parameter('a.b', 1)
    n.declare_parameter('flag', True)
    n.declare_parameter('lista', [1, 2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--comum', action='append', default=[])
    ap.add_argument('--ciclo', action='append', default=[])
    ap.add_argument('--grafo', action='append', default=[],
                    help='nó de grafo sem serviços de parâmetros')
    a = ap.parse_args()

    rclpy.init()
    ex = SingleThreadedExecutor()
    nos = []
    for c in a.comum:
        nome, ns = separa(c)
        n = Node(nome, namespace=ns)
        declara(n)
        nos.append(n)
    for c in a.ciclo:
        completo, alvo = c.split(':')
        nome, ns = separa(completo)
        n = LifecycleNode(nome, namespace=ns)
        declara(n)
        n.declare_parameter('footprint_padding', 0.01)
        if alvo in ('inativo', 'ativo'):
            n.trigger_configure()
        if alvo == 'ativo':
            n.trigger_activate()
        nos.append(n)
    for c in a.grafo:
        nome, ns = separa(c)
        nos.append(Node(nome, namespace=ns, start_parameter_services=False))
    for n in nos:
        ex.add_node(n)

    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
