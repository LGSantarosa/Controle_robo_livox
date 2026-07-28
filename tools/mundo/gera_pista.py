#!/usr/bin/env python3
"""Gera a pista de obstáculos: mundo do Gazebo E mapa do Nav2, da MESMA fonte.

    python3 tools/mundo/gera_pista.py

Escreve `worlds/pista_obstaculos.sdf`, `maps/pista_obstaculos.pgm` e
`maps/pista_obstaculos.yaml`.

**Por que gerar os dois juntos:** o costmap do Nav2 e o mundo do simulador
precisam descrever o mesmo lugar. Mantidos à mão eles divergem — e um mapa que
não bate com o mundo produz plano bonito que o robô não consegue seguir, com o
erro aparecendo no lugar errado (culpa-se o seguidor por um obstáculo que o
planner não sabia que existia). Aqui a planta é UMA lista de retângulos, e as
duas saídas caem dela.

Enquanto o modelo 3D do robô não chega, só o MAPA é usado: a bancada do planner
julga caminho sem sensor e sem robô. O mundo fica pronto para quando o lidar
entrar no modelo.

A pista foi desenhada para responder perguntas sobre o planner, não para ser
bonita. Cada obstáculo tem um propósito, anotado na planta abaixo.
"""
import os

RES = 0.05                 # m/píxel do mapa
LARG, ALT = 12.0, 8.0      # m — a sala inteira
ESP = 0.2                  # m — espessura de parede
ALTURA_PAREDE = 0.6        # m — alto o bastante para o lidar ver, baixo para
                           # não virar caixa fechada na visualização

# ---------------------------------------------------------------- a planta
#
# Cada item: (nome, x0, y0, x1, y1) em metros, cantos do retângulo.
# O robô tem 0,50 m de largura — os vãos abaixo são medidos contra isso.

PAREDES = [
    # -- perímetro --
    ('borda_sul',   0.0, 0.0, LARG, ESP),
    ('borda_norte', 0.0, ALT - ESP, LARG, ALT),
    ('borda_oeste', 0.0, 0.0, ESP, ALT),
    ('borda_leste', LARG - ESP, 0.0, LARG, ALT),

    # -- PORTA: vão de 0,90 m (robô 0,50) --
    # Pergunta que responde: o planner passa pela porta ou contorna? E o
    # caminho encosta na quina ou sai centrado?
    ('divisoria_sul',   3.9, ESP, 4.1, 2.05),
    ('divisoria_norte', 3.9, 2.95, 4.1, ALT - ESP),

    # -- BLOCO SOLTO: obstáculo para contornar em campo aberto --
    # Pergunta: ele contorna pelo lado curto (certo) ou dá a volta grande?
    ('bloco', 6.5, 4.0, 8.0, 5.5),

    # -- APERTO: vão de 0,80 m --
    # Pergunta: com a inflação escolhida, este vão é aceito ou recusado? É o
    # caso que separa "planner corajoso" de "planner que não cabe".
    ('aperto_sul',   8.9, ESP, 9.1, 3.2),
    ('aperto_norte', 8.9, 4.0, 9.1, ALT - ESP),

    # -- BECO SEM SAÍDA: parece atalho, não é --
    # Pergunta: ele entra e volta (plano ruim) ou já sai pelo caminho bom?
    # A boca fica no canto NORDESTE (de x=11,0 até a parede leste): dá para
    # entrar, não dá para atravessar. Fechado dos dois lados isto virava uma
    # caixa lacrada e os planners recusavam o destino — corretamente, o que
    # não testa nada.
    ('beco_lateral', 10.0, ESP, 10.2, 2.5),
    ('beco_fundo',   10.0, 2.3, 11.0, 2.5),
]

# ---------------------------------------------------------------- mundo SDF

CABECALHO = f'''<?xml version="1.0"?>
<!--
  GERADO por tools/mundo/gera_pista.py — não editar à mão.
  Edite a planta no script e rode de novo; o mapa do Nav2 sai do mesmo lugar.

  Pista de obstáculos: porta de 0,90 m, bloco solto, aperto de 0,80 m e um beco
  sem saída. Sala de {LARG:.0f} x {ALT:.0f} m. O robô tem 0,50 m de largura.
-->
<sdf version="1.10">
  <world name="pista_obstaculos">

    <physics name="passo_fino" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.8</gravity>

    <light type="directional" name="sol">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.3 -0.9</direction>
    </light>

    <model name="chao">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <surface>
            <friction>
              <ode><mu>1.0</mu><mu2>1.0</mu2></ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry>
          <material>
            <ambient>0.75 0.75 0.75 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''

PAREDE_SDF = '''
    <model name="{nome}">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material>
            <ambient>0.55 0.55 0.58 1</ambient>
            <diffuse>0.62 0.62 0.66 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
'''


def escreve_mundo(caminho):
    partes = [CABECALHO]
    for nome, x0, y0, x1, y1 in PAREDES:
        partes.append(PAREDE_SDF.format(
            nome=nome, cx=(x0 + x1) / 2, cy=(y0 + y1) / 2, cz=ALTURA_PAREDE / 2,
            sx=x1 - x0, sy=y1 - y0, sz=ALTURA_PAREDE))
    partes.append('\n  </world>\n</sdf>\n')
    with open(caminho, 'w') as f:
        f.write(''.join(partes))


# ---------------------------------------------------------------- mapa PGM

def escreve_mapa(pgm, yaml):
    """Rasteriza a mesma planta em PGM (0 = ocupado, 254 = livre).

    A linha 0 do PGM é o TOPO da imagem, que corresponde ao MAIOR y do mundo —
    daí a inversão. Errar isso produz um mapa espelhado que parece plausível e
    manda o robô para o lado errado.
    """
    w, h = int(LARG / RES), int(ALT / RES)
    grade = bytearray([254]) * (w * h)
    for _, x0, y0, x1, y1 in PAREDES:
        c0, c1 = int(x0 / RES), int(x1 / RES)
        r0, r1 = int((ALT - y1) / RES), int((ALT - y0) / RES)
        for r in range(max(0, r0), min(h, r1)):
            for c in range(max(0, c0), min(w, c1)):
                grade[r * w + c] = 0
    with open(pgm, 'wb') as f:
        f.write(f'P5\n# gerado por tools/mundo/gera_pista.py\n{w} {h}\n255\n'
                .encode())
        f.write(bytes(grade))
    with open(yaml, 'w') as f:
        f.write(f'''# GERADO por tools/mundo/gera_pista.py — não editar à mão.
image: {os.path.basename(pgm)}
mode: trinary
resolution: {RES}
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
''')
    return w, h


def main():
    raiz = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))))
    mundo = os.path.join(raiz, 'worlds', 'pista_obstaculos.sdf')
    pgm = os.path.join(raiz, 'maps', 'pista_obstaculos.pgm')
    yaml = os.path.join(raiz, 'maps', 'pista_obstaculos.yaml')
    os.makedirs(os.path.dirname(pgm), exist_ok=True)
    escreve_mundo(mundo)
    w, h = escreve_mapa(pgm, yaml)
    print(f'mundo -> {mundo}')
    print(f'mapa  -> {pgm} ({w}x{h} px, {RES} m/px)')
    print(f'        {yaml}')
    print(f'{len(PAREDES)} obstáculos; sala {LARG:.0f}x{ALT:.0f} m; '
          'vãos: porta 0,90 m e aperto 0,80 m (robô 0,50 m)')


if __name__ == '__main__':
    main()
