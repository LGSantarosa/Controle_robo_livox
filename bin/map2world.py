#!/usr/bin/env python3
"""map2world.py — converte um mapa de ocupação (PGM+YAML do map_server) num mundo SDF.

Uso: bin/map2world.py maps/sala_real/sala.yaml worlds/sala_real.sdf [--height 0.5]

Cada célula ocupada vira parede; células adjacentes são fundidas em retângulos
(greedy por linhas + merge vertical) pra não gerar milhares de caixinhas.
O frame do mundo == frame do mapa (origin do YAML respeitado) → o robô
localiza de cara com o mesmo mapa no AMCL.

Reescrito 2026-07-06 (a versão original de 06-27 vivia em scratchpad efêmero e
foi perdida — por isso agora está versionada aqui).
"""
import argparse
import os
import sys

import yaml


def load_map(yaml_path):
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    pgm_path = os.path.join(os.path.dirname(yaml_path), meta["image"])
    with open(pgm_path, "rb") as f:
        magic = _token(f)
        if magic != b"P5":
            sys.exit(f"só PGM binário (P5); achei {magic!r}")
        w, h, maxval = (_numero(f, nome) for nome in ("largura", "altura",
                                                      "maxval"))
        data = f.read(w * h)
    return meta, w, h, maxval, data


def _token(f):
    """Próximo campo do cabeçalho PGM, pulando espaço em branco e comentários.

    Lido TOKEN A TOKEN, e não linha a linha, porque a especificação do PGM
    separa os campos por espaço em branco *qualquer* — e os formatos convivem
    no mesmo diretório de mapas:

        P5\\n973 808\\n255\\n      largura e altura na MESMA linha
        P5\\n1468\\n399\\n255\\n   uma por linha

    A versão anterior lia uma linha e exigia os dois números nela. O mapa do
    segundo formato derrubava o conversor com `not enough values to unpack`,
    que não diz nada sobre mapa nenhum — e mapa é o insumo de toda esta
    ferramenta.

    ⚠️ O espaço em branco que FECHA o token é consumido aqui, de propósito: a
    especificação manda exatamente um separador entre o `maxval` e o primeiro
    byte de dado, e é este `read` que o come.
    """
    while True:
        byte = f.read(1)
        if not byte:
            return b""
        if byte == b"#":                       # comentário: até o fim da linha
            while byte and byte not in b"\r\n":
                byte = f.read(1)
            continue
        if byte.isspace():
            continue
        token = byte
        while True:
            byte = f.read(1)
            if not byte or byte.isspace():
                return token
            token += byte


def _numero(f, nome):
    token = _token(f)
    if not token.isdigit():
        sys.exit(f"cabeçalho PGM truncado ou inválido: esperava {nome}, "
                 f"achei {token!r}")
    return int(token)


def occupied_grid(meta, w, h, maxval, data):
    """True = parede. Convenção map_server: negate=0 → escuro = ocupado."""
    occ_thresh = float(meta.get("occupied_thresh", 0.65))
    negate = int(meta.get("negate", 0))
    grid = [[False] * w for _ in range(h)]
    for y in range(h):
        row = grid[y]
        base = y * w
        for x in range(w):
            p = data[base + x] / maxval
            occ = (1.0 - p) if negate == 0 else p
            row[x] = occ > occ_thresh
    return grid


def drop_small_blobs(grid, w, h, min_cells):
    """Remove blobs ocupados com menos de min_cells células (chuvisco do SLAM).
    Flood-fill 8-conexo em python puro pra não depender de scipy."""
    seen = [[False] * w for _ in range(h)]
    removed_blobs = removed_cells = 0
    for sy in range(h):
        for sx in range(w):
            if not grid[sy][sx] or seen[sy][sx]:
                continue
            stack = [(sy, sx)]
            seen[sy][sx] = True
            blob = []
            while stack:
                y, x = stack.pop()
                blob.append((y, x))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        ny, nx = y + dy, x + dx
                        if 0 <= ny < h and 0 <= nx < w and grid[ny][nx] and not seen[ny][nx]:
                            seen[ny][nx] = True
                            stack.append((ny, nx))
            if len(blob) < min_cells:
                for y, x in blob:
                    grid[y][x] = False
                removed_blobs += 1
                removed_cells += len(blob)
    return removed_blobs, removed_cells


def merge_rects(grid, w, h):
    """Funde células ocupadas em retângulos: runs horizontais + merge vertical
    de runs idênticos (mesmo x0/x1) em linhas consecutivas."""
    runs = []  # (y, x0, x1) inclusivos
    for y in range(h):
        x = 0
        row = grid[y]
        while x < w:
            if row[x]:
                x0 = x
                while x < w and row[x]:
                    x += 1
                runs.append([y, y, x0, x - 1])  # y0, y1, x0, x1
            else:
                x += 1
    # merge vertical
    merged = []
    open_runs = {}  # (x0,x1) -> rect ainda crescendo
    for rect in runs:
        y0, y1, x0, x1 = rect
        key = (x0, x1)
        prev = open_runs.get(key)
        if prev is not None and prev[1] == y0 - 1:
            prev[1] = y1
        else:
            if prev is not None:
                merged.append(prev)
            open_runs[key] = rect
    merged.extend(open_runs.values())
    return merged


def downsample(grid, w, h, factor):
    """Reduz a resolução: célula nova ocupada se QUALQUER célula original dentro
    dela for ocupada (parede engorda, nunca some)."""
    nw, nh = (w + factor - 1) // factor, (h + factor - 1) // factor
    g2 = [[False] * nw for _ in range(nh)]
    for y in range(h):
        row = grid[y]
        gy = g2[y // factor]
        for x in range(w):
            if row[x]:
                gy[x // factor] = True
    return g2, nw, nh


def rects_to_obj(rects, meta, w, h, wall_height, obj_path):
    """Extruda os retângulos num único mesh OBJ (12 triângulos por caixa).
    Milhares de <box> de colisão não escalam no DART (custo por shape a cada
    passo, RTF<0.2 mesmo com o mundo parado — vale tanto pra N models quanto
    pra N shapes num link); um trimesh único vira BVH no FCL = barato, e no
    render é 1 draw call em vez de milhares."""
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    out = ["# gerado por map2world.py"]
    # normais dos 6 lados (reaproveitadas por todas as caixas)
    out += ["vn 1 0 0", "vn -1 0 0", "vn 0 1 0", "vn 0 -1 0", "vn 0 0 1", "vn 0 0 -1"]
    nv = 0
    for (y0, y1, x0, x1) in rects:
        # PGM: linha 0 = topo do mapa = y máximo no mundo
        xa, xb = ox + x0 * res, ox + (x1 + 1) * res
        ya, yb = oy + (h - 1 - y1) * res, oy + (h - y0) * res
        for x in (xa, xb):
            for y in (ya, yb):
                for z in (0.0, wall_height):
                    out.append(f"v {x:.3f} {y:.3f} {z:.3f}")
        # vértices locais 1..8 na ordem binária (x,y,z); winding pra fora
        b = nv
        quads = [
            (5, 7, 8, 6, 1),   # +x
            (1, 2, 4, 3, 2),   # -x
            (3, 4, 8, 7, 3),   # +y
            (1, 5, 6, 2, 4),   # -y
            (2, 6, 8, 4, 5),   # +z
            (1, 3, 7, 5, 6),   # -z
        ]
        for v1, v2, v3, v4, n in quads:
            out.append(f"f {b+v1}//{n} {b+v2}//{n} {b+v3}//{n}")
            out.append(f"f {b+v1}//{n} {b+v3}//{n} {b+v4}//{n}")
        nv += 8
    with open(obj_path, "w") as f:
        f.write("\n".join(out) + "\n")


def walls_model_boxes(rects, meta, w, h, wall_height):
    """As mesmas caixas, mas como `<box>` PRIMITIVAS de colisão.

    🔴 20-08 — POR QUE ISTO EXISTE, e é defeito medido, não preferência. O
    mesh único economiza CPU (ver `rects_to_obj`) e **não segura o robô**:
    colisão de malha triangular côncava no gz-sim deixa o robô ESCALAR a
    parede. Na prova do corredor do andar 3 ele emperrou na garganta da porta
    com `z = 0,068` — 7 cm acima do chão, montado em cima da parede — e o
    sintoma na navegação era idêntico ao defeito real que se queria estudar:
    chega na porta, não passa, dá ré, repete. Quatro corridas foram gastas
    acreditando nisso.

    Primitiva é o que o `pista_obstaculos.sdf` (o mundo que sempre funcionou)
    usa: 22 `<box>`. Cada caixa vira um `<collision>` no mesmo link, e o custo
    por shape do DART é real — por isso o par natural desta opção é o
    `--recorte`, que reduz o mundo à região sob prova.
    """
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    partes = []
    for i, (y0, y1, x0, x1) in enumerate(rects):
        xa, xb = ox + x0 * res, ox + (x1 + 1) * res
        ya, yb = oy + (h - 1 - y1) * res, oy + (h - y0) * res
        cx, cy = (xa + xb) / 2.0, (ya + yb) / 2.0
        sx, sy = xb - xa, yb - ya
        geom = (f"<geometry><box><size>{sx:.3f} {sy:.3f} "
                f"{wall_height:.3f}</size></box></geometry>")
        pose = f"<pose>{cx:.3f} {cy:.3f} {wall_height / 2.0:.3f} 0 0 0</pose>"
        partes.append(f'<collision name="c{i}">{pose}{geom}</collision>'
                      f'<visual name="v{i}">{pose}{geom}'
                      '<material><ambient>0.6 0.55 0.5 1</ambient>'
                      '<diffuse>0.6 0.55 0.5 1</diffuse></material></visual>')
    return ("\n    <model name=\"walls\">\n      <static>true</static>\n"
            "      <link name=\"link\">\n        " + "\n        ".join(partes) +
            "\n      </link>\n    </model>")


def recorta(grid, w, h, meta, caixa):
    """Zera tudo fora do retângulo `xmin ymin xmax ymax` (metros do mapa).

    Mundo menor = menos shapes, que é o que torna a colisão primitiva viável.
    Não mexe na origem nem na resolução: as coordenadas continuam as do mapa,
    então mapa e mundo seguem casados célula a célula.
    """
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    xmin, ymin, xmax, ymax = caixa
    mantidas = 0
    for y in range(h):
        ymundo = oy + (h - 1 - y) * res
        for x in range(w):
            if not grid[y][x]:
                continue
            xmundo = ox + x * res
            if xmin <= xmundo <= xmax and ymin <= ymundo <= ymax:
                mantidas += 1
            else:
                grid[y][x] = False
    return mantidas


def walls_model_sdf(obj_name):
    geom = f"<geometry><mesh><uri>{obj_name}</uri></mesh></geometry>"
    return f"""
    <model name="walls">
      <static>true</static>
      <link name="link">
        <collision name="col">{geom}</collision>
        <visual name="vis">{geom}
          <material><ambient>0.6 0.55 0.5 1</ambient><diffuse>0.6 0.55 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>"""


SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="{name}">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="col"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="vis"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material></visual>
      </link>
    </model>
{walls}
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml")
    ap.add_argument("out_sdf")
    ap.add_argument("--height", type=float, default=0.5)
    ap.add_argument("--min-blob", type=int, default=1,
                    help="descarta blobs ocupados com menos células que isso (chuvisco do SLAM)")
    ap.add_argument("--downsample", type=int, default=1,
                    help="engrossa a célula por esse fator (5cm→15cm com 3); "
                         "menos caixas, paredes engordam até 1 célula")
    ap.add_argument("--caixas", action="store_true",
                    help="colisão por <box> PRIMITIVAS em vez de mesh único. "
                         "Mesh não segura o robô (ele escala a parede); use "
                         "isto para qualquer mundo em que o robô vá encostar")
    ap.add_argument("--recorte", nargs=4, type=float,
                    metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                    help="mantém só as paredes dentro deste retângulo [m], nas "
                         "coordenadas do mapa. Par natural do --caixas")
    args = ap.parse_args()

    meta, w, h, maxval, data = load_map(args.map_yaml)
    grid = occupied_grid(meta, w, h, maxval, data)
    if args.min_blob > 1:
        nb, nc = drop_small_blobs(grid, w, h, args.min_blob)
        print(f"filtro min-blob {args.min_blob}: removeu {nb} blobs ({nc} células)")
    if args.downsample > 1:
        grid, w, h = downsample(grid, w, h, args.downsample)
        meta = dict(meta, resolution=float(meta["resolution"]) * args.downsample)
        print(f"downsample {args.downsample}x: célula {meta['resolution'] * 100:.0f}cm, grade {w}x{h}")
    if args.recorte:
        mantidas = recorta(grid, w, h, meta, args.recorte)
        print(f"recorte {args.recorte}: {mantidas} células ocupadas mantidas")
    rects = merge_rects(grid, w, h)
    n_occ = sum(sum(r) for r in grid)
    name = os.path.splitext(os.path.basename(args.out_sdf))[0]
    if args.caixas:
        walls = walls_model_boxes(rects, meta, w, h, args.height)
        extra = f"{len(rects)} caixas PRIMITIVAS (colisão confiável)"
    else:
        obj_path = os.path.splitext(args.out_sdf)[0] + ".obj"
        rects_to_obj(rects, meta, w, h, args.height, obj_path)
        walls = walls_model_sdf(os.path.basename(obj_path))
        extra = (f"{len(rects)} caixas num mesh único "
                 f"({os.path.basename(obj_path)}) — ⚠️ mesh NÃO segura o robô "
                 f"em contato; veja --caixas")
    with open(args.out_sdf, "w") as f:
        f.write(SDF_TEMPLATE.format(name=name, walls=walls))
    print(f"{args.out_sdf}: {w}x{h} @{meta['resolution']}m, "
          f"{n_occ} células ocupadas → {extra}")


if __name__ == "__main__":
    main()
