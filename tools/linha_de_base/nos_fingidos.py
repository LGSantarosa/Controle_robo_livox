#!/usr/bin/env python3
"""Nós fingidos para testar a captura sem Gazebo (usado só pelo test_captura).

    python3 nos_fingidos.py --comum /ns/no --ciclo /a/b:ativo --grafo /auxiliar \
        --param-falso /falso:ilegivel

`--param-falso <nó>:<modo>` sobe um nó com os SEIS serviços de parâmetro
escritos à mão, imitando o `get_parameters` do rclcpp, que é tudo-ou-nada:
se um nome pedido falha, ele devolve ZERO valores para o lote inteiro (foi o
`collision_monitor` em 21-09, com `Polygon*.max_points`). Modos:
  ilegivel    lista use_sim_time, a, ruim; `ruim` zera o lote e, sozinho,
              volta PARAMETER_NOT_SET;
  lote_falha  lista use_sim_time, a, b; lote com >1 nome volta vazio, mas
              cada nome sozinho responde;
  vazio       lista nada.

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


class ParametrosFalsos:
    """Os seis serviços de parâmetro de um nó, com defeito escolhido."""

    def __init__(self, node, modo):
        from rcl_interfaces import srv
        from rcl_interfaces.msg import ParameterType, ParameterValue
        self.T, self.V = ParameterType, ParameterValue
        self.modo = modo
        self.valores = {
            'use_sim_time': ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=False),
            'a': ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=1),
            'b': ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=0.5),
        }
        self.nomes = {'ilegivel': ['use_sim_time', 'a', 'ruim'],
                      'lote_falha': ['use_sim_time', 'a', 'b'],
                      'vazio': []}[modo]
        base = node.get_fully_qualified_name()
        node.create_service(srv.ListParameters, f'{base}/list_parameters', self.lista)
        node.create_service(srv.GetParameters, f'{base}/get_parameters', self.le)
        for tipo, sufixo in ((srv.SetParameters, 'set_parameters'),
                             (srv.SetParametersAtomically, 'set_parameters_atomically'),
                             (srv.DescribeParameters, 'describe_parameters'),
                             (srv.GetParameterTypes, 'get_parameter_types')):
            node.create_service(tipo, f'{base}/{sufixo}', lambda req, resp: resp)

    def lista(self, req, resp):
        resp.result.names = list(self.nomes)
        return resp

    def le(self, req, resp):
        nomes = list(req.names)
        if self.modo == 'lote_falha' and len(nomes) > 1:
            return resp
        if 'ruim' in nomes:
            if len(nomes) == 1:
                resp.values = [self.V(type=self.T.PARAMETER_NOT_SET)]
            return resp
        resp.values = [self.valores[n] for n in nomes]
        return resp


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--comum', action='append', default=[])
    ap.add_argument('--ciclo', action='append', default=[])
    ap.add_argument('--grafo', action='append', default=[],
                    help='nó de grafo sem serviços de parâmetros')
    ap.add_argument('--param-falso', action='append', default=[])
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
    falsos = []
    for c in a.param_falso:
        completo, modo = c.split(':')
        nome, ns = separa(completo)
        n = Node(nome, namespace=ns, start_parameter_services=False)
        falsos.append(ParametrosFalsos(n, modo))
        nos.append(n)
    for n in nos:
        ex.add_node(n)

    try:
        ex.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
