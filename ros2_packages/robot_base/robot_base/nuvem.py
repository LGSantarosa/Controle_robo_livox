"""Empacotar pontos do Livox como `PointCloud2` — lógica pura, sem ROS.

Está separada do nó (`nuvem_pontos.py`) pelo padrão da casa: aqui mora o que
precisa ser testável na máquina de dev, onde as mensagens do
`livox_ros_driver2` **não existem** (o pacote só é compilado no NUC).

## Por que este módulo existe

Medido no robô em 10-08:

    /livox/lidar   publisher:  livox_ros_driver2/msg/CustomMsg
    costmaps, collision_monitor e o pré-voo assinam  sensor_msgs/PointCloud2

O FAST-LIO funciona porque lê CustomMsg — por isso a localização ia bem e a
percepção era ZERO, com a nuvem medida a 9,96 Hz o tempo todo. No simulador o
lidar é um `gpu_lidar` que publica PointCloud2 no MESMO nome de tópico, então a
decisão 014 passou lá e nunca foi exercitada de verdade: **o nome batia e o
contrato não**.
"""
import math
import struct

# x, y, z, intensidade — todos float32. Quatro campos de 4 bytes.
#
# `intensity` não é usada por nenhum consumidor nosso (costmap e reflexo só
# querem geometria), mas entra porque é o layout que as ferramentas de
# visualização esperam encontrar, e porque um ponto de 16 bytes fica alinhado.
PASSO_PONTO = 16
CAMPOS = (
    # (nome, deslocamento, datatype FLOAT32 = 7, quantidade)
    ('x', 0, 7, 1),
    ('y', 4, 7, 1),
    ('z', 8, 7, 1),
    ('intensity', 12, 7, 1),
)

_PONTO = struct.Struct('<ffff')


def ponto_valido(x, y, z, raio_cego=0.0):
    """O Mid-360 emite (0,0,0) quando o raio não volta.

    ⚠️ Deixar esses pontos passar não é ruído inocente: (0,0,0) no frame do
    sensor é o PRÓPRIO ROBÔ. A camada de obstáculo marcaria célula letal em
    cima dele a cada quadro, e o planejador passaria a recusar todo caminho por
    estar "dentro" de um obstáculo — com a nuvem aparentemente perfeita.

    ⚠️ **E a origem exata não era o único ponto do próprio robô** — medido no
    robô em 12-08, com ele PARADO no meio da sala e o `collision_monitor`
    disparando 928 vezes:

        0 a 2 pontos por quadro, sempre no MESMO lugar, no frame do sensor:
            x ≈ +0,04 m   y ≈ +0,09 m   z ≈ 0,00 a 0,05 m
            raio horizontal 0,10 m       (`min_points` do reflexo é 2)

    É peça do próprio robô a 10 cm do Mid-360 e na altura dele. Como oscilava
    entre 1 e 2 pontos, o reflexo ligava e desligava sozinho o tempo todo, e
    durante a primeira navegação autônoma **vetou 26 dos 156 comandos (17%)**,
    cada veto zerando o comando por ~0,1 s dentro de uma malha com 0,94 s de
    tempo morto. Ver a decisão 027.

    Por isso `raio_cego` corta um CILINDRO em torno do eixo do sensor, e não uma
    esfera: o corpo do robô fica ABAIXO do lidar, e o que o cega é a estrutura
    ao longo de todo o z, não uma vizinhança da origem.

    ⚠️ **O custo, e ele é real**: obstáculo de verdade a menos de `raio_cego`
    do eixo do sensor fica invisível. Com 0,15 m isso cai inteiramente dentro
    do chassi (o polígono de parada vai de −0,28 a +0,49 em x e ±0,26 em y), ou
    seja, para haver algo ali ele já teria de estar encostado no robô. Subir
    este número troca segurança por silêncio.
    """
    if x == 0.0 and y == 0.0 and z == 0.0:
        return False
    return math.hypot(x, y) >= raio_cego


def empacota(pontos, raio_cego=0.0):
    """`pontos` iterável de (x, y, z, intensidade) -> (bytes, quantos, descartados).

    Devolve o buffer no layout de `CAMPOS`, já sem os pontos inválidos. Conta os
    descartados porque essa contagem é diagnóstico: nuvem que chega com metade
    dos pontos zerados é sensor sujo ou obstruído, e isso tem de aparecer no
    log em vez de virar silêncio.

    `raio_cego` em metros: pontos a menos disso do EIXO do sensor são do próprio
    robô e saem junto (ver `ponto_valido` e a decisão 027). Zero mantém o
    comportamento anterior — é o que se usa para MEDIR se a peça que estava ao
    lado do sensor era mesmo a culpada.
    """
    buf = bytearray()
    n = 0
    fora = 0
    for x, y, z, i in pontos:
        if not ponto_valido(x, y, z, raio_cego):
            fora += 1
            continue
        buf += _PONTO.pack(x, y, z, i)
        n += 1
    return bytes(buf), n, fora


def desempacota(buf):
    """Volta de bytes para tuplas — existe para o teste conferir o que foi
    escrito, e para diagnóstico à mão."""
    return [_PONTO.unpack_from(buf, o)
            for o in range(0, len(buf) - PASSO_PONTO + 1, PASSO_PONTO)]
