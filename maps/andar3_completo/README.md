# Andar 3 completo — trazido do repo do robô 3 (01-10)

Cópia **byte a byte** do `maps/andar3_robo3/mapa_3_andar_completo.{pgm,yaml}`
de `LGSantarosa/Controle_robo_livox_robo3`, commit `01f81a2`: o mapa que o
robô 3 desenhou em 30-09 com Mid-360 → FAST-LIO → `/scan` → `slam_toolbox`.
Só o nome mudou, para seguir a convenção `maps/<nome>/<nome>.yaml` do
`bin/sobe-robo`. O conteúdo do `.pgm` é idêntico (SHA-256 conferido).

```
tamanho   391 x 1158 células a 5 cm  (~19,6 x 57,9 m)
origem    [-2.202, -12.239, 0]
(0, 0)    a largada do SLAM do robô 3 = a MARCA DE FITA (sala de baixo)
```

Pedido do dono em 01-10: é o mapa grande que vale para o robô 2, por ser
bem melhor que o `andar3todo`. Virou o padrão do `bin/sobe-robo`.

⚠️ O `sobe-robo` nasce o AMCL em `0, 0, 0`. O robô 2 tem de estar **na marca
de fita, virado para a mesma direção da largada do robô 3**, senão o AMCL
nasce mentindo.

⚠️ Nunca foi validado para navegação: no robô 3, a primeira subida do Nav2
contra ele saturou a CPU do notebook antes de localizar. No robô 2 (NUC),
esta é a primeira vez.
