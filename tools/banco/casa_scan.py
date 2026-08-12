#!/usr/bin/env python3
"""O `/scan` casa com o mapa? — a pergunta que precede confiar no AMCL.

    # pilha de pé, robô PARADO (não move nada):
    python3 tools/banco/casa_scan.py --mapa maps/meu_mapa/meu_mapa.yaml
    python3 tools/banco/casa_scan.py --mapa ... --pose amcl --csv saida.csv

Projeta cada feixe do `/scan` no frame do mapa e mede a distância do ponto de
impacto até a parede mais próxima da planta. É a primeira coisa a rodar antes de
deixar o robô navegar com mapa, e responde em 10 s **sem mover o robô**.

## Por que isto existe

Localização contra mapa falha de duas formas que se parecem de fora: a fatia 2D
está errada (decisão 021 — altura, alcance, frame) ou a pose está errada. As
duas dão "mapa e nuvem desalinhados" no RViz.

Este instrumento separa as duas porque mede com a pose que VOCÊ escolhe:

    --pose odom   onde o robô acha que está por odometria (o default)
    --pose amcl   onde o AMCL diz que ele está

⚠️ **`--pose amcl` é parcialmente circular** e não serve de prova sozinho: o
AMCL escolheu justamente a pose que maximiza esse casamento. Ele responde outra
pergunta — "o filtro convergiu para um lugar que faz sentido?" — e a diferença
entre os dois números é que é informativa. Casar mal no `odom` e bem no `amcl`
significa que o filtro está trabalhando; casar mal nos dois significa que a
fatia 2D ou o mapa estão errados.

## O que o número significa

Com o mundo do Gazebo GERADO a partir do mapa, o casamento é o limite da
discretização: 100% dentro de 0,15 m e erro mediano de uma célula. No robô real
vai ser pior — móvel, vidro, porta aberta e gente não estão na planta — e é
exatamente por isso que o número precisa existir antes, e não depois.

Não usa `scipy`: a busca é limitada a uma janela em volta de cada impacto
(`--teto`), o que basta para a pergunta e roda em qualquer Python. Bancada que
não roda no robô não serve.
"""
import argparse
import csv
import math
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(AQUI)), 'bin'))


def carrega_mapa(caminho_yaml):
    """Mapa de ocupação, pelo MESMO leitor que gera o mundo do Gazebo.

    Reusar o `map2world.py` não é economia de linhas: é garantia de que o que
    esta régua chama de "parede" é o que virou parede no simulador. Duas
    leituras independentes do mesmo PGM divergiriam em silêncio — e a divergência
    apareceria como erro de casamento, culpando a fatia 2D.
    """
    sys.path.insert(0, os.path.join(
        os.path.dirname(os.path.dirname(AQUI)), 'bin'))
    import importlib.util
    p = os.path.join(os.path.dirname(os.path.dirname(AQUI)), 'bin',
                     'map2world.py')
    spec = importlib.util.spec_from_file_location('map2world', p)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    meta, w, h, maxval, dados = m.load_map(caminho_yaml)
    return meta, w, h, m.occupied_grid(meta, w, h, maxval, dados)


def ponto_do_feixe(x, y, yaw, angulo, alcance):
    """Onde o feixe bate, no frame do mapa."""
    a = yaw + angulo
    return x + alcance * math.cos(a), y + alcance * math.sin(a)


def celula(meta, h, x, y):
    """(coluna, linha) do PGM para um ponto do mundo.

    ⚠️ A linha 0 do PGM é o TOPO do mapa, ou seja o **maior** y do mundo. É a
    mesma inversão que o `map2world.py` aplica ao extrudar as paredes; errá-la
    espelha o mapa verticalmente e o casamento vira ruído sem nenhum aviso.
    """
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    col = int(round((x - ox) / res))
    lin = int(round(h - 1 - (y - oy) / res))
    return col, lin


def distancia_ate_parede(meta, w, h, grid, x, y, teto=0.5):
    """Metros até a parede mais próxima, ou None se não houver dentro do teto.

    Busca em janela limitada em vez de transformada de distância no mapa
    inteiro: são poucas centenas de feixes, o teto responde a pergunta ("casou
    ou não"), e assim não entra dependência que o NUC talvez não tenha.
    """
    res = float(meta['resolution'])
    col, lin = celula(meta, h, x, y)
    raio = int(math.ceil(teto / res))
    melhor = None
    for dl in range(-raio, raio + 1):
        l = lin + dl
        if not (0 <= l < h):
            continue
        for dc in range(-raio, raio + 1):
            c = col + dc
            if not (0 <= c < w) or not grid[l][c]:
                continue
            d = math.hypot(dc, dl) * res
            if d <= teto and (melhor is None or d < melhor):
                melhor = d
    return melhor


def resumo(distancias, fora_do_mapa, faixas=(0.05, 0.10, 0.15, 0.25, 0.50)):
    """As linhas do veredito. `None` = não achou parede dentro do teto."""
    casou = [d for d in distancias if d is not None]
    n = len(distancias)
    L = [f'{n} feixes válidos ({fora_do_mapa} caíram fora do mapa)']
    if not n:
        return L + ['🔴 nenhum feixe válido — o /scan está publicando?']
    for lim in faixas:
        pc = 100.0 * sum(1 for d in casou if d <= lim) / n
        L.append(f'  a menos de {lim:.2f} m de parede da planta: {pc:5.1f}%')
    if casou:
        ordenado = sorted(casou)
        meio = len(ordenado) // 2
        mediana = (ordenado[meio] if len(ordenado) % 2
                   else (ordenado[meio - 1] + ordenado[meio]) / 2.0)
        L.append(f'  erro mediano {mediana:.3f} m')
    perdidos = n - len(casou)
    if perdidos:
        L.append(f'  🔴 {perdidos} feixes ({100.0*perdidos/n:.1f}%) sem parede '
                 'nenhuma perto: o que eles viram não está na planta')
    return L


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mapa', required=True, help='o .yaml do map_server')
    ap.add_argument('--pose', choices=['odom', 'amcl'], default='odom',
                    help='de onde vem a pose do robô; "amcl" é parcialmente '
                         'circular — ver o cabeçalho')
    ap.add_argument('--teto', type=float, default=0.5,
                    help='até onde procurar parede [m]')
    ap.add_argument('--csv', default='')
    a = ap.parse_args()

    meta, w, h, grid = carrega_mapa(a.mapa)

    # rclpy fica DENTRO do main para as funções acima rodarem nos testes sem ROS.
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    no = Node('casa_scan')
    dados = {}
    no.create_subscription(LaserScan, '/scan',
                           lambda m: dados.setdefault('scan', m),
                           qos_profile_sensor_data)
    if a.pose == 'amcl':
        # ⚠️ TRANSIENT_LOCAL, e não o QoS padrão. O AMCL só publica `/amcl_pose`
        # quando o filtro ATUALIZA (a cada `update_min_d` de deslocamento) —
        # com o robô parado, que é como esta régua é usada, nunca vem
        # mensagem nova e o instrumento morre em "faltou a pose" reclamando de
        # uma pilha que está perfeita. Com durabilidade transitória ele recebe
        # a última publicada.
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        qos_ultima = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        no.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                               lambda m: dados.__setitem__('pose', m.pose.pose),
                               qos_ultima)
    else:
        no.create_subscription(Odometry, '/Odometry',
                               lambda m: dados.__setitem__('pose', m.pose.pose),
                               qos_profile_sensor_data)

    t0 = no.get_clock().now().nanoseconds
    while (('scan' not in dados or 'pose' not in dados)
           and no.get_clock().now().nanoseconds - t0 < 30e9):
        rclpy.spin_once(no, timeout_sec=0.2)

    if 'scan' not in dados or 'pose' not in dados:
        print('🔴 faltou /scan ou a pose. A pilha está de pé? '
              '(tools/banco/checa_pilha.py)')
        rclpy.shutdown()
        raise SystemExit(1)

    s, p = dados['scan'], dados['pose']
    q = p.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    print(f'robô em ({p.position.x:.2f}, {p.position.y:.2f}) '
          f'rumo {math.degrees(yaw):+.1f}°  [pose: {a.pose}]')

    distancias, fora, linhas = [], 0, []
    for i, r in enumerate(s.ranges):
        if not math.isfinite(r) or r < s.range_min or r > s.range_max:
            continue
        ang = s.angle_min + i * s.angle_increment
        x, y = ponto_do_feixe(p.position.x, p.position.y, yaw, ang, r)
        col, lin = celula(meta, h, x, y)
        if not (0 <= col < w and 0 <= lin < h):
            fora += 1
            continue
        d = distancia_ate_parede(meta, w, h, grid, x, y, a.teto)
        distancias.append(d)
        linhas.append({'i': i, 'angulo': round(ang, 4), 'alcance': round(r, 3),
                       'x': round(x, 3), 'y': round(y, 3),
                       'ate_parede': ('' if d is None else round(d, 3))})

    print()
    for linha in resumo(distancias, fora):
        print(linha)

    if a.csv and linhas:
        with open(a.csv, 'w', newline='') as f:
            escritor = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
            escritor.writeheader()
            escritor.writerows(linhas)
        print(f'\n{len(linhas)} feixes -> {a.csv}')

    no.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
