#!/usr/bin/env python3
"""Transforma um mapa de SLAM em mundo do Gazebo — e num mapa que BATE com ele.

⚠️ ESCRITO EM 20-08 E NAO USADO POR NADA. Leia isto antes de gastar tempo com
ele. O `worlds/sala_andar3.sdf` que o projeto usa vem de um mesh `.obj` e e
melhor que a saida daqui em todos os aspectos que importam. Eu escrevi este
script sem antes conferir que aquele mundo ja existia, e cheguei a
SOBRESCREVE-LO — restaurado do git na mesma sessao.

Fica no repo porque a conversao mapa->mundo pode ser util para um andar que
ainda nao tenha mesh, e porque as duas licoes de desempenho abaixo (um modelo
so, e o cuidado com o numero de colisoes) custaram uma sessao para aparecer.
Se for usar: confira antes se ja existe um `.obj` para aquele lugar.

    python3 tools/mundo/gera_mundo_de_mapa.py --mapa maps/sala_andar3/sala_andar3.yaml

Escreve `worlds/<nome>.sdf`, `maps/<nome>_sim/<nome>_sim.pgm` e o `.yaml`.

## Por que não dá para converter o PGM direto

O mapa do robô é um scan, não uma planta. O `sala_andar3` medido em 18-08 é
**83% desconhecido**, com paredes fragmentadas e ruído de um pixel. Emitir uma
caixa por célula ocupada produz um mundo cheio de frestas — e o robô escapa por
elas, ou pior: o LiDAR simulado enxerga através, o AMCL diverge, e passamos a
depurar um defeito que só existe porque o mundo está furado.

O que este script faz é o inverso: descreve o **espaço navegável** e fecha o
contorno dele. A região livre do scan é limpa (fecha buraco, descarta ilha
solta), fica só a componente conexa que o robô de fato percorreu, e a parede
nasce como a casca em volta dessa região. O resultado é um mundo estanque com a
geometria real das portas, que é o que interessa aqui.

⚠️ **O que este mundo NÃO é**: a planta do prédio. Onde o LiDAR não passou, não
há parede — há o fim do mundo. Um corredor que o robô nunca percorreu vira uma
sala fechada. Serve para ensaiar a passagem pelas portas medidas; não serve para
concluir nada sobre lugares que o scan não viu.

## Por que o mapa também é gerado

Mesma razão do `gera_pista.py`: se o mundo vier do contorno limpo e o mapa
continuar sendo o PGM ruidoso, os dois descrevem lugares diferentes. O AMCL
casaria o scan simulado contra paredes que o mundo não tem, e o erro apareceria
como "localização ruim" quando na verdade é fonte divergente. A `origin` do YAML
original é preservada, então as coordenadas continuam as do robô real.
"""
import argparse
import os

import numpy as np
from scipy import ndimage

ALTURA = 0.6               # m — mesma da pista, acima do plano do lidar
ESPESSURA_PADRAO = 4       # células de parede (4 x 0,05 = 0,20 m)


def le_mapa(caminho_yaml):
    """Lê o par .yaml/.pgm do Nav2 e devolve (imagem, resolução, origem)."""
    import re
    texto = open(caminho_yaml).read()

    def campo(nome, padrao=None):
        m = re.search(rf'^{nome}\s*:\s*(.+)$', texto, re.M)
        return m.group(1).strip() if m else padrao

    res = float(campo('resolution'))
    origem = [float(v) for v in campo('origin').strip('[]').split(',')]
    imagem = campo('image')
    ocup = float(campo('occupied_thresh', '0.65'))
    livre = float(campo('free_thresh', '0.196'))
    pgm = os.path.join(os.path.dirname(caminho_yaml), imagem)

    from PIL import Image
    arr = np.array(Image.open(pgm))
    # Convenção do map_server: valor ALTO = livre, BAIXO = ocupado, meio =
    # desconhecido. Os limiares vêm em probabilidade de ocupação.
    return arr, res, origem, (1.0 - ocup) * 255.0, (1.0 - livre) * 255.0


def regiao_navegavel(arr, corte_livre, fecha=3, area_min=200):
    """A maior componente conexa de espaço livre, com os buracos fechados.

    `fecha` tapa fresta de até esse tanto de células — buraco de scan, não
    porta. Uma porta de 0,80 m são 16 células, então ela nunca é fechada por
    engano. `area_min` descarta ilha de ruído longe da área percorrida.
    """
    livre = arr > corte_livre
    livre = ndimage.binary_closing(livre, np.ones((fecha, fecha), bool))
    rotulos, n = ndimage.label(livre)
    if n == 0:
        raise SystemExit('mapa sem espaço livre — conferir os limiares')
    tamanhos = ndimage.sum(livre, rotulos, range(1, n + 1))
    maior = int(np.argmax(tamanhos)) + 1
    principal = rotulos == maior
    # Buraco INTERNO da região navegável é obstáculo de verdade (um pilar, uma
    # mesa): preencher aqui o apagaria do mundo. Só os pequenos somem.
    buracos = ndimage.binary_fill_holes(principal) & ~principal
    rot_b, nb = ndimage.label(buracos)
    for k in range(1, nb + 1):
        if (rot_b == k).sum() < area_min:
            principal |= rot_b == k
    return principal


def casca(navegavel, espessura):
    """A parede: o anel de `espessura` células em volta do navegável."""
    grosso = ndimage.binary_dilation(
        navegavel, np.ones((3, 3), bool), iterations=espessura)
    return grosso & ~navegavel


def retangulos(mascara):
    """Cobre a máscara com retângulos grandes — um modelo SDF por retângulo.

    Guloso: varre em ordem, e de cada célula ainda descoberta estica o maior
    retângulo possível. Não é a cobertura ótima (esse problema é NP-difícil),
    mas leva ~2 mil células de parede a algumas centenas de caixas, que é o
    que separa um mundo que abre de um que engasga o Gazebo.
    """
    livre = mascara.copy()
    H, W = livre.shape
    saida = []
    for j in range(H):
        for i in range(W):
            if not livre[j, i]:
                continue
            # estica em x enquanto a linha inteira estiver descoberta
            i2 = i
            while i2 + 1 < W and livre[j, i2 + 1]:
                i2 += 1
            # estica em y enquanto a faixa [i, i2] inteira estiver descoberta
            j2 = j
            while j2 + 1 < H and livre[j2 + 1, i:i2 + 1].all():
                j2 += 1
            livre[j:j2 + 1, i:i2 + 1] = False
            saida.append((i, j, i2 - i + 1, j2 - j + 1))
    return saida


def sdf(nome, caixas, res, origem, altura_img):
    """Monta o SDF. `altura_img` é o número de linhas: o PGM cresce para baixo
    e o mundo cresce para cima, então a linha vira `origem_y + (H - j) * res`.

    ⚠️ TUDO NUM MODELO SÓ, e isto não é estética. A primeira versão emitia um
    `<model>` por retângulo: 253 modelos no andar 3, cada um uma entidade com
    física própria e um alvo a mais para o raycast do lidar. O Gazebo abriu, mas
    andando a ~0,5 do tempo real — o `hoverboard_base_controller` recusava todo
    comando por chegar 0,50 s atrasado, e a pilha inteira ficava inerte com o
    mundo carregado na tela. Um modelo estático com N `<collision>` descreve a
    mesma geometria e roda em tempo real.
    """
    partes = [f'''<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="{nome}">
    <physics name="passo_fino" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <light type="directional" name="sol">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.5 0.3 -0.9</direction>
    </light>
    <model name="chao">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          <material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material>
        </visual>
      </link>
    </model>''']
    partes.append('''
    <model name="paredes">
      <static>true</static>
      <link name="link">''')
    for n, (i, j, w, h) in enumerate(caixas):
        cx = origem[0] + (i + w / 2.0) * res
        cy = origem[1] + (altura_img - (j + h / 2.0)) * res
        partes.append(f'''
        <collision name="c{n:04d}">
          <pose>{cx:.3f} {cy:.3f} {ALTURA/2:.3f} 0 0 0</pose>
          <geometry><box><size>{w*res:.3f} {h*res:.3f} {ALTURA:.3f}</size></box></geometry>
        </collision>
        <visual name="v{n:04d}">
          <pose>{cx:.3f} {cy:.3f} {ALTURA/2:.3f} 0 0 0</pose>
          <geometry><box><size>{w*res:.3f} {h*res:.3f} {ALTURA:.3f}</size></box></geometry>
          <material><ambient>0.4 0.4 0.45 1</ambient><diffuse>0.5 0.5 0.55 1</diffuse></material>
        </visual>''')
    partes.append('''
      </link>
    </model>
  </world>
</sdf>
''')
    return ''.join(partes)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mapa', required=True, help='caminho do .yaml do Nav2')
    ap.add_argument('--nome', help='nome de saída (padrão: o do mapa)')
    ap.add_argument('--espessura', type=int, default=ESPESSURA_PADRAO,
                    help='parede em células (padrão 4 = 0,20 m)')
    ap.add_argument('--fecha', type=int, default=3,
                    help='fecha fresta de até N células (padrão 3 = 0,15 m)')
    args = ap.parse_args()

    arr, res, origem, corte_ocup, corte_livre = le_mapa(args.mapa)
    nome = args.nome or os.path.splitext(os.path.basename(args.mapa))[0]
    H, W = arr.shape

    nav = regiao_navegavel(arr, corte_livre, fecha=args.fecha)
    parede = casca(nav, args.espessura)
    caixas = retangulos(parede)

    print(f'{nome}: {W}x{H} px = {W*res:.1f} x {H*res:.1f} m')
    print(f'  navegável  {nav.sum():6d} células = {nav.sum()*res*res:7.1f} m²')
    print(f'  parede     {parede.sum():6d} células -> {len(caixas)} caixas')

    raiz = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    destino_sdf = os.path.join(raiz, 'worlds', f'{nome}.sdf')
    with open(destino_sdf, 'w') as fh:
        fh.write(sdf(nome, caixas, res, origem, H))
    print(f'  -> {destino_sdf}')

    # O mapa da MESMA fonte: parede ocupada, navegável livre, resto
    # desconhecido. Origem preservada, então as coordenadas continuam as do
    # robô real e dá para comparar corrida de simulador com corrida de campo.
    saida = np.full(arr.shape, 205, np.uint8)     # desconhecido
    saida[nav] = 254                              # livre
    saida[parede] = 0                             # ocupado
    dir_mapa = os.path.join(raiz, 'maps', f'{nome}_sim')
    os.makedirs(dir_mapa, exist_ok=True)
    from PIL import Image
    Image.fromarray(saida).save(os.path.join(dir_mapa, f'{nome}_sim.pgm'))
    with open(os.path.join(dir_mapa, f'{nome}_sim.yaml'), 'w') as fh:
        fh.write(f'''# GERADO por tools/mundo/gera_mundo_de_mapa.py a partir de
# {args.mapa} — não editar à mão.
image: {nome}_sim.pgm
mode: trinary
resolution: {res}
origin: [{origem[0]}, {origem[1]}, {origem[2]}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
''')
    print(f'  -> {dir_mapa}/')

    # Um ponto de spawn que caiba de verdade: o livre mais longe de parede.
    dist = ndimage.distance_transform_edt(nav)
    j, i = np.unravel_index(int(np.argmax(dist)), dist.shape)
    print(f'\n  spawn sugerido (ponto mais folgado, {dist[j, i]*res:.2f} m '
          f'de parede):\n    pose_x:={origem[0] + i*res:.2f} '
          f'pose_y:={origem[1] + (H - j)*res:.2f}')


if __name__ == '__main__':
    main()
