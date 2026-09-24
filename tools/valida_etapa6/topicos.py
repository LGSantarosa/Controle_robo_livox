#!/usr/bin/env python3
"""As provas VIVAS da etapa 6 que não cabem num dump de parâmetro — §4.5 e §4.2.

    topicos.py <saida.yaml> [--prazo 60]

Quatro coisas, e nenhuma delas se prova lendo arquivo:

  1. `/Odometry` e `/scan` com PUBLICADOR e ao menos uma MENSAGEM recebida.
     Publicador sem mensagem é o caso comum e enganoso: o nó subiu, o tópico
     existe, e nada trafega — foi assim que a nuvem do robô 2 "existiu" por três
     semanas com o tipo errado (decisão 017).
  2. As TFs que a pilha precisa: `map → base_link` (a pose) e
     `base_link → livox_frame` (onde o sensor está montado).
  3. Publicador ÚNICO de `map → odom`. O `tf_map_odom` e o AMCL publicam a mesma
     transformada, e juntos a pose PISCA entre "identidade" e a verdade a cada
     consulta — o sintoma (ziguezague no RViz, plano que salta) não aponta para
     TF nenhuma.
     ⚠️ LIMITE HONESTO DESTA PROVA: o tf2 não diz quem publicou o quê. O que se
     confere é (a) que a transformada existe e (b) que só UM dos dois candidatos
     está no grafo, com a lista de publicadores de `/tf` e `/tf_static`
     registrada como evidência. Não é farejar o `/tf`; é o que dá para afirmar.
  4. A fronteira do atuador real fora do grafo, pelo lado do TÓPICO: nenhum
     tópico de tipo `WheelSpeeds`. No Gazebo a cadeia termina no
     `hoverboard_base_controller`, e `WheelSpeeds` não tem consumidor nenhum —
     se ele aparecer, alguém publicou comando por um caminho que ninguém mede.

O nó é OCULTO (`_valida_etapa6`): nome começando com `_` fica fora do
`ros2 node list` e das comparações de grafo, então esta ferramenta não se conta
na lista versionada que ela mesma ajuda a provar.
"""
import argparse
import sys
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

NOME = '_valida_etapa6'
# Os dois publicadores possíveis de `map → odom`; exatamente um pode existir.
CANDIDATOS_MAP_ODOM = ('/tf_map_odom', '/amcl')
TFS = (('map', 'base_link'), ('base_link', 'livox_frame'))


def _qos_sensor():
    """O perfil do sensor: o `/scan` do pointcloud_to_laserscan é best effort.

    Assinar RELIABLE um publicador BEST_EFFORT é incompatibilidade de QoS — a
    assinatura sobe, o tópico aparece com publicador, e nenhuma mensagem chega.
    Seria um REPROVADO falso, e do tipo que custa horas para entender.
    """
    return QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE,
                      history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class Prova(Node):
    def __init__(self):
        super().__init__(NOME)
        # Tempo simulado: a pilha inteira roda nele, e um `lookup_transform`
        # feito no relógio de parede compararia carimbos de dois mundos.
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.contagem = {'/Odometry': 0, '/scan': 0}
        self.create_subscription(Odometry, '/Odometry',
                                 lambda _m: self._conta('/Odometry'), _qos_sensor())
        self.create_subscription(LaserScan, '/scan',
                                 lambda _m: self._conta('/scan'), _qos_sensor())
        self.buffer = Buffer()
        self.ouvinte = TransformListener(self.buffer, self)

    def _conta(self, topico):
        self.contagem[topico] += 1


def colhe(no, prazo):
    """Roda o nó até as duas condições básicas baterem, ou o prazo vencer."""
    fim = time.time() + prazo
    tfs = {}
    while time.time() < fim:
        rclpy.spin_once(no, timeout_sec=0.2)
        for alvo, origem in TFS:
            if f'{alvo}->{origem}' in tfs:
                continue
            if no.buffer.can_transform(alvo, origem, rclpy.time.Time()):
                tfs[f'{alvo}->{origem}'] = True
        if all(no.contagem.values()) and len(tfs) == len(TFS):
            break
    return tfs


def avalia(contagem, tfs, publicadores, nos, tipos_por_topico):
    """A parte pura: dado o que se observou, aprovar ou reprovar cada item."""
    itens = {}
    for topico in sorted(contagem):
        pubs = publicadores.get(topico, [])
        itens[f'4.5 {topico}: publicador e mensagem'] = (
            bool(pubs) and contagem[topico] > 0,
            {'publicadores': pubs, 'mensagens': contagem[topico]})
    for alvo, origem in TFS:
        chave = f'{alvo}->{origem}'
        itens[f'4.5 TF {chave}'] = (chave in tfs, {'resolveu': chave in tfs})
    presentes = [n for n in CANDIDATOS_MAP_ODOM if n in nos]
    itens['4.5 publicador único de map->odom'] = (
        len(presentes) == 1,
        {'candidatos_no_grafo': presentes,
         'publicadores_de_tf': publicadores.get('/tf', []),
         'publicadores_de_tf_static': publicadores.get('/tf_static', [])})
    wheel = sorted(t for t, tipos in tipos_por_topico.items()
                   if any('WheelSpeeds' in tp for tp in tipos))
    itens['4.2 nenhum tópico WheelSpeeds no grafo'] = (
        not wheel, {'topicos': wheel})
    return itens


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('saida')
    ap.add_argument('--prazo', type=float, default=60.0)
    a = ap.parse_args(argv)

    rclpy.init()
    no = Prova()
    try:
        tfs = colhe(no, a.prazo)
        tipos = {t: tps for t, tps in no.get_topic_names_and_types()}
        publicadores = {}
        for topico in list(no.contagem) + ['/tf', '/tf_static']:
            publicadores[topico] = sorted(
                f'{ns.rstrip("/")}/{n}' if ns not in ('', '/') else f'/{n}'
                for n, ns in
                [(i.node_name, i.node_namespace)
                 for i in no.get_publishers_info_by_topic(topico)])
        nos = sorted(f'{ns.rstrip("/")}/{n}' if ns not in ('', '/') else f'/{n}'
                     for n, ns in no.get_node_names_and_namespaces())
        itens = avalia(dict(no.contagem), tfs, publicadores, nos, tipos)
    finally:
        no.destroy_node()
        rclpy.shutdown()

    veredito = 'APROVADO' if all(ok for ok, _ in itens.values()) else 'REPROVADO'
    with open(a.saida, 'w') as f:
        yaml.safe_dump(
            {'veredito': veredito, 'nos_vistos': nos,
             'itens': {nome: {'veredito': 'APROVADO' if ok else 'REPROVADO',
                              **det}
                       for nome, (ok, det) in itens.items()}},
            f, sort_keys=True, allow_unicode=True, width=1000)
    for nome, (ok, _) in sorted(itens.items()):
        print(f'{"APROVADO" if ok else "REPROVADO"}  {nome}')
    print(f'veredito: {veredito}  ({a.saida})')
    return 0 if veredito == 'APROVADO' else 1


if __name__ == '__main__':
    sys.exit(main())
