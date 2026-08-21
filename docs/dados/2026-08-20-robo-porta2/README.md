# 2026-08-20 — a corrida da porta 2, no ROBÔ

`seguidor_2026-08-20_183507.csv` — 21 233 amostras, 2 358 s. É o CSV que
sustenta o handoff de 20-08 **e a correção dele** (DIARIO, 2ª leva; decisão
044). Fica versionado porque duas leituras diferentes já foram tiradas dele.

## O que a corrida é

```
t=  57- 208   travessia 1   y máximo 19,02 em x=6,94   5 rés na porta 2
t= 194- 209   o dono carrega o robô de volta (19,3 m em 15,4 s)
t= 762- 874   travessia 2   y máximo 18,92 em x=6,99   3 rés na porta 2
t= 859- 874   o dono carrega o robô de volta (19,1 m em 15,0 s)
t=2101-2117   não saiu da origem
```

`dist` (ao fim do plano) **nunca desceu de 4,94 m**: nesta corrida a porta 2
nunca foi vencida. A passagem única que o dono relata (*"passou 1 vez, foi
quando chegou no objetivo pela primeira vez"*) está em outro CSV, no robô.

## Os números que já foram tirados dele

```
v real com v_alvo = 0,50, fora dos trechos carregados (n=2247)
    p10 0,00   p50 0,26   p90 0,32 m/s        a máquina não entrega o v_max

na porta, travessia 1 (32 s)     v real p50 +0,01 m/s
                                 |erro rumo| p50 23,6°  max 51,5°
                                 |wz| real   p50  2,9 °/s
                                 rumo p50 69°   (o corredor é 90°)
na porta, travessia 2 (16 s)     rumo p50 81°, 2,3 s parado com v_alvo +0,50

|d rumo_alvo|/dt   corredor livre p90 38-40 °/s
                   PORTA 2        p90 37-40 °/s      <- IGUAL: a mira não é
                   (mira estica 20% do tempo no corredor e 0% na porta)
```

## ⚠️ Armadilhas deste arquivo

- **`v_alvo` é PEDIDO, não velocidade.** Velocidade real só derivando a pose —
  e com janela de pelo menos 0,3 s: a pose distinta chega a ~4,5 Hz enquanto o
  CSV grava a 20 Hz, então linhas consecutivas repetem a mesma posição e
  qualquer diferença sobre 0,05 s explode.
- **Derive COM SINAL** (projeção no rumo). Foi tomar módulo numa virada
  ré→frente que produziu os "0,55 m/s" do handoff.
- **Os trechos a 1,3 m/s não são salto de LIO** — são 19 m contínuos em 15 s:
  o robô no colo do dono, voltando ao início.
- **Conte EPISÓDIOS, não amostras** (o CSV é a 20 Hz).
