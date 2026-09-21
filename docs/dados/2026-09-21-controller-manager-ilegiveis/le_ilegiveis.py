#!/usr/bin/env python3
"""Evidência dos ilegíveis do /controller_manager (21-09), SÓ LEITURA.

Roda contra a pilha do robô 2 de pé no Gazebo (bin/linha-de-base-robo2):
lista os parâmetros, lê em lote e relê sozinho cada nome que não vem com
exatamente um valor legível. Não escreve parâmetro nenhum.
"""
import rclpy
from rclpy.parameter_client import AsyncParameterClient

rclpy.init()
n = rclpy.create_node('_evidencia_ilegiveis')
cli = AsyncParameterClient(n, '/controller_manager')
print('serviços:', cli.wait_for_services(5.0))


def espera(f):
    rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    return f.result()


nomes = sorted(espera(cli.list_parameters(depth=None)).result.names)
lote = espera(cli.get_parameters(nomes))
print(f'list_parameters: {len(nomes)} nomes; get_parameters em lote: {len(lote.values)} valores')
for x in nomes:
    um = espera(cli.get_parameters([x]))
    vals = list(um.values) if um else []
    if len(vals) != 1 or vals[0].type == 0:
        tipo = f', tipo {vals[0].type} (0 = PARAMETER_NOT_SET)' if vals else ''
        print(f'ILEGIVEL /controller_manager:{x} -> {len(vals)} valor(es){tipo}')
n.destroy_node()
rclpy.shutdown()
