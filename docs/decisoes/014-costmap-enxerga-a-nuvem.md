# 014 — O costmap passa a enxergar a nuvem do Mid-360

**Data:** 2026-08-07
**Estado:** implementada e medida no simulador; **nunca rodou no robô**
**Depende de:** [012 — LiDAR 3D no simulador](012-lidar-3d-no-simulador.md),
[003 — base ros2_control e LIO](003-base-ros2control-e-lio.md)

## O buraco que estava aberto, e há quanto tempo

O robô 2 tem um sensor externo, o Mid-360. Ele publica `PointCloud2` em
`/livox/lidar` desde 24-07 no robô (driver real) e desde 04-08 no simulador
(decisão 012). Até esta sessão, **o único consumidor dessa nuvem no projeto
inteiro era o `collision_monitor`**.

Os dois costmaps do Nav2 rodavam com `static_layer` e mais nada. O comentário
que sobrou no `nav2.yaml` datava disso:

    # Só o mapa estático por enquanto: o robô simulado ainda não tem lidar.

A frase estava velha havia três dias, e a consequência não era pequena: **o robô
não desviava de obstáculo novo — ele PARAVA na frente dele.** A fatia B do
`ESTADO_PROJETO` (desviar de obstáculo) estava bloqueada por configuração, não
por hardware nem por sensor.

## O que entra

Uma camada de obstáculo alimentada pela nuvem, **nos dois costmaps**, com a
mesma faixa de altura que o reflexo de colisão já usa (0,10 a 0,50 m).

### Por que `VoxelLayer` e não `ObstacleLayer` — é a geometria do sensor

O Mid-360 varre de **−7° a +52°**: ele olha para cima. Com o sensor a 0,42 m
(trena de 05-08), a altura que ele enxerga a uma distância `d` é

    z(d) = 0,42 − 0,123·d

Uma caixa de 0,30 m é vista a 1–2 m e **some quando o robô chega a 0,5 m dela**
— o feixe mais baixo passa a 0,36 m, por cima da caixa. Ou seja: o robô perde
de vista o obstáculo exatamente quando vai manobrar perto dele.

A `ObstacleLayer` projeta tudo em 2D e limpa por raytrace 2D. O raio que passa
por cima da caixa **apaga a célula da própria caixa** — o robô esquece o que
acabou de ver, no pior momento. A `VoxelLayer` limpa por voxel em altura: os
voxels altos por onde o raio passou são liberados e o voxel baixo, onde a caixa
está, continua marcado.

⚠️ **Isto é o oposto do que o robô 1 decidiu**, e a divergência é deliberada. O
`robot_nav/config/nav2_params_pi.yaml` troca `VoxelLayer` por `ObstacleLayer`
com o argumento *"sem câmera RGB-D o eixo Z do voxel é desperdício puro"*. Lá o
sensor é um LD06 **planar**, e com nuvem plana o eixo z é mesmo desperdício.
Aqui ele é o que separa "passa por baixo" de "bate". **Não portar aquela troca
para cá** — é o tipo de correção boa do robô 1 que piora este robô.

### Por que nos DOIS costmaps, e não só no local

Esta foi a parte que o experimento corrigiu. A primeira versão pôs a camada só
no `local_costmap`, com o raciocínio de que o local é quem dirige. Medido no
simulador com o mundo `pista_surpresa.sdf` (obstáculo em 2,00 · 6,50 que o mapa
**não** tem), pedindo um caminho de (2,0 · 5,0) para (2,0 · 7,2):

```
                        caminho / reta   folga do centro do obstáculo
camada só no local       2,20 / 2,20      0,035 m   ATRAVESSA
camada nos dois          2,74 / 2,20      0,530 m   CONTORNA
```

Quem **dirige** via a caixa; quem **planeja** não. E o efeito prático disso não
é "desvio pior": é um robô que vai reto até o obstáculo e trava lá. O
`collision_monitor` salva a máquina, o replanejamento roda a 1 Hz e devolve
**para sempre o mesmo plano ruim**. Obstáculo que entra só no local costmap não
produz desvio — produz travamento educado.

### Os números, e de onde cada um saiu

```
min/max_obstacle_height  0,10 / 0,50   iguais aos do reflexo (05-08: a nuvem
                                       erra até 3,8 cm em incidência rasante;
                                       acima de 0,50 o robô passa por baixo)
obstacle_max_range       2,0 (local)   a janela local é 4×4 m: marcar além é
                         3,0 (global)  produzir célula que a janela descarta.
                                       O global escolhe ROTA e precisa ver longe
raytrace_max_range       3,0 / 4,0     limpar pode ir além do que marca
z_resolution × z_voxels  0,05 × 16     coluna de 0,80 m: cobre o sensor a 0,42
mark_threshold           2             ponto solto é ruído — mesmo raciocínio
                                       do `min_points: 2` do reflexo
expected_update_rate     0,50          ⚠️ CORRIGIDO por medida — ver abaixo
```

## O que a máquina corrigiu do que eu tinha escrito

**`expected_update_rate` 0,30 → 0,50 s.** O primeiro valor era conta de padaria
(10 Hz mais 3 quadros de folga) e não sobreviveu à primeira corrida:

    The /livox/lidar observation buffer has not been updated for 0.43 seconds,
    and it should be updated every 0.30 seconds

E a nuvem estava **perfeita**: 9,7 Hz medidos, carimbo de 100,0 ms com mediana,
p90, p99 e máximo idênticos — sem cauda nenhuma (281 quadros). O atraso é do
**consumidor**: são 20 000 pontos por quadro para transportar e transformar.

Isso não é cosmético. Buffer vencido deixa a camada **não-current**, e costmap
não-current **para de atualizar**. Errar este número para baixo desliga a
percepção tendo um aviso amarelo por único sintoma. Com 0,50 s, zero avisos na
corrida de validação.

## Como se prova que é percepção, e não memória

Um robô que desvia tem duas explicações que se parecem de fora: ele viu com o
lidar, ou o mapa estático já dizia que havia algo ali. Enquanto mundo e mapa
saírem da **mesma planta** (`gera_pista.py`), as duas são indistinguíveis — e a
segunda não é percepção nenhuma.

Entram duas ferramentas para separá-las:

- **`tools/mundo/gera_pista.py`** passa a escrever também `pista_surpresa.sdf`:
  o mesmo mundo com dois obstáculos que **nunca entram no mapa**. A
  `pilha.launch.py` já separava `mundo` de `mapa` como argumento esperando
  exatamente este arquivo;
- **`tools/banco/percepcao.py`** compara o costmap com o mapa e relata as
  células letais onde o mapa diz LIVRE — as que só o sensor explica;
- **`tools/banco/plano.py`** pede um caminho pela ação `compute_path_to_pose`
  (**planeja sem mover o robô**) e mede a folga até um obstáculo dado.

Medido, robô parado em (2,0 · 5,0):

```
local_costmap   1 mancha: centro (2,00 · 6,28), 0,075 m²
global_costmap  2 manchas: (2,00 · 6,28) e (3,79 · 2,64)
```

A primeira é a face **sul** da caixa que está em x 1,75–2,25 / y 6,25–6,75 — a
única que o lidar enxerga daquela posição, e o centróide bate em x exatamente.
A segunda é o obstáculo do vão da porta, a 3,5 m: o alcance de 3,0 m do global
fazendo efeito.

⚠️ **O robô só conhece a face que viu.** A caixa inteira nunca vira obstáculo
no costmap; vira a fatia de 0,075 m² que o sensor tocou. No caso medido o plano
passou a 0,50 m da face oeste (folga real de corpo ~0,18 m), mas isso é sorte
da geometria, não garantia — o planejador está contornando uma lasca do
obstáculo, não o obstáculo.

## Ressalvas que ficam abertas

1. **No `global_costmap` a marcação é PERMANENTE.** Não há janela rolante: o
   que for marcado fica até o raytrace limpar. Com a TF `map→odom` fixa e
   provisória, **toda deriva do LIO no robô real vira obstáculo fantasma
   acumulado**. É seguro no simulador (mundo e mapa saem da mesma planta) e é
   dívida no robô — o mesmo buraco que já obriga o mapa a ser argumento. A
   saída é a mesma: localização que case `map` com `odom`.
2. **O simulador enxerga melhor que o robô**, e a decisão 012 já avisa por quê:
   o `gpu_lidar` varre em raster e o Mid-360 varre em roseta não repetitiva.
   Obstáculo fino que a roseta pode perder num quadro aqui aparece sempre.
   Nada nesta decisão prova taxa de detecção de obstáculo fino no robô.
3. **A zona cega continua inteira.** Nenhuma camada de costmap conserta
   geometria: obstáculo baixo a menos de ~0,5 m é invisível, e o reflexo é cego
   para ele (`collision_monitor.yaml`). O que muda é que agora existe MEMÓRIA
   do que foi visto de longe — que era a saída apontada lá.
4. **Nada disto rodou no robô.** É a mesma ressalva da decisão 012.

## Referências

- `docs/DIARIO.md`, 07-08 — a sessão que implementou e mediu
- `ros2_packages/robot_motion/config/nav2.yaml` — o racional inteiro, por número
- `ros2_packages/robot_motion/test/test_configs_coerentes.py` — o que ficou travado
