#!/usr/bin/env python3
"""Extrai do bag da corrida o que o montador precisa (decisão 060).

    le(caminho_do_bag) -> {'amostras': [...], 'status': [...] ou None}

Só lê e converte; não julga. O formato de saída é exatamente o que o
`monta.py` recebe (ver o `test_le_bag_etapa7.py`):

  · amostras do tópico final e do `/cmd_vel_bruto`: `t_ns` é o instante de
    GRAVAÇÃO (o bag é gravado com `--use-sim-time`, então é tempo simulado);
    `header_ns` vai só como diagnóstico, porque a placa copia o header do
    comando de entrada (`placa_simulada.py:431`);
  · status da ação por mensagem gravada, com o UUID em hex.

Tópico de status ausente do bag vira `None`, não `[]`: "não foi gravado" (o
caso do bag de 20260924_160148, sem `--include-hidden-topics`) e "gravado sem
mensagem" são falhas diferentes. Tipo inesperado num tópico lido é erro com
mensagem — ler outro layout seria adivinhar.

Precisa do ROS carregado (`rosbag2_py`, `rclpy`).
"""
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICO_FINAL = '/hoverboard_base_controller/cmd_vel'
TOPICO_DIAG = '/cmd_vel_bruto'
STATUS = '/navigate_to_pose/_action/status'

ESPERADOS = {
    TOPICO_FINAL: 'geometry_msgs/msg/TwistStamped',
    TOPICO_DIAG: 'geometry_msgs/msg/TwistStamped',
    STATUS: 'action_msgs/msg/GoalStatusArray',
}

_S = 1_000_000_000


def _ns(stamp):
    return int(stamp.sec) * _S + int(stamp.nanosec)


def _amostra(topico, msg, t_ns):
    return {'topico': topico, 't_ns': int(t_ns),
            'header_ns': _ns(msg.header.stamp),
            'v': float(msg.twist.linear.x), 'wz': float(msg.twist.angular.z)}


def _status(msg, t_ns):
    return {'t_ns': int(t_ns),
            'goals': [{'uuid': bytes(bytearray(g.goal_info.goal_id.uuid)).hex(),
                       'stamp_ns': _ns(g.goal_info.stamp),
                       'status': int(g.status)}
                      for g in msg.status_list]}


def le(caminho):
    leitor = rosbag2_py.SequentialReader()
    leitor.open(rosbag2_py.StorageOptions(uri=str(caminho), storage_id='mcap'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    tipos = {t.name: t.type for t in leitor.get_all_topics_and_types()}
    for topico, tipo in tipos.items():
        if topico in ESPERADOS and tipo != ESPERADOS[topico]:
            raise ValueError(f'{topico} gravado como {tipo}, esperado '
                             f'{ESPERADOS[topico]}')
    lidos = [t for t in ESPERADOS if t in tipos]
    classes = {t: get_message(tipos[t]) for t in lidos}
    leitor.set_filter(rosbag2_py.StorageFilter(topics=lidos))

    amostras, status = [], ([] if STATUS in tipos else None)
    while lidos and leitor.has_next():
        topico, dados, t_ns = leitor.read_next()
        msg = deserialize_message(dados, classes[topico])
        if topico == STATUS:
            status.append(_status(msg, t_ns))
        else:
            amostras.append(_amostra(topico, msg, t_ns))
    return {'amostras': amostras, 'status': status}
