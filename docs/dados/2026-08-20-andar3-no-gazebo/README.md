# 2026-08-20 — o corredor do andar 3 roda no Gazebo, e o defeito aparece

Primeira vez que o cenário REAL é reproduzido fora do robô. Mundo gerado do
mapa do robô (`bin/map2world.py` sobre `andar3todoalterado`), mesmo mapa no
Nav2, spawn no corredor.

```
ros2 launch robot_motion pilha.launch.py sim:=true gui:=false rviz:=false \
  bag:=false \
  mundo:=$PWD/worlds/andar3todoalterado.sdf \
  mapa:=$PWD/maps/andar3todoalterado/andar3todoalterado.yaml \
  pose_x:=6.14 pose_y:=10.0 pose_yaw:=1.5708
```

Objetivo em `(6,80 · 20,30)` — do outro lado da porta 2.

## O que aconteceu

```
subiu o corredor de y=10,0 ate y=18,79        8,7 m, seguindo o plano
NAO passou a porta (o vao esta em y=19,0)
7 episodios de re
|erro de rumo| na aproximacao (y>18,3)   p50 14,6°   p90 59,5°   max 83,7°
```

**No robô, 20-08**: travessia 1 p50 23,6°, travessia 2 p50 13,9°, e também não
passou. Mesma ordem de grandeza, mesmo desfecho.

## ⚠️ POR QUE O GAZEBO "NÃO ERA EFICAZ" ATÉ AQUI

Porque as provas rodavam no `pista_obstaculos`, cujas portas têm **0,90 m**.
A porta 2 real tem **0,70 m** no mapa. Com o footprint do `nav2.yaml`
(0,555 m de largura × 0,617 m de comprimento) o limite de erro de rumo para
caber é:

```
largura projetada = 0,555·cos θ + 0,617·sin θ

  porta de 0,90 m   ->  cabe ate  θ ≈ 32°
  porta de 0,70 m   ->  cabe ate  θ ≈ 12,4°     <- a porta REAL
```

**O simulador estava testando um problema quase três vezes mais folgado.** Com
o mundo certo, o mesmo defeito do robô aparece na primeira corrida.

## Arquivos

- `seguidor_*.csv` — a corrida (CSV do seguidor)
- `freeze_capture.csv` — a cadeia de comando (decisão 044), primeira gravação
  útil dela: dá para rodar `bin/pause_budget.py` em cima
