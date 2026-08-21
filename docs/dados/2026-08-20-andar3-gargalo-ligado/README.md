# 2026-08-20 — o alinhamento LIGADO na porta real de 0,70 m

Mesma prova de `2026-08-20-andar3-no-gazebo`, mudando UMA coisa:
`gargalo:=true` (`passagem_estreita_habilitada`, decisão 033, que estava
desligada). Primeira vez que esse modo roda no cenário para o qual ele foi
escrito — o teste que o reprovou em 20-08 rodou num mundo de portas 0,90 m,
onde alinhar não decide nada.

## 🟢 O detector achou a porta

```
path_follower: 1 gargalo(s) fixado(s) no plano aceito: 0.73 m
```

0,73 m contra os 0,70 m que a varredura do mapa dá. O alvo que ele calcula
está certo: `(6,99 · 19,01)` na fase de centro e `(6,99 · 20,01)` na de eixo,
contra um centro real de vão em **6,96**. Três centímetros.

## 🟢 O RUMO foi resolvido

```
                        |erro de rumo| na aproximacao (y > 18,3)
gargalo DESLIGADO         p50 14,6°   p90 59,5°     (0,7 cm de folga)
gargalo LIGADO            p50  4,1°   p90 16,2°     (4,7 cm de folga)
```

Era exatamente o que a conta da porta pedia: abaixo de 12,4° cabe, abaixo de
5° cabe com margem. **O modo faz o que promete.**

## 🔴 E ele continua sem passar — por outro motivo, agora medido

```
x do robo na aproximacao   p10 7,13   p50 7,20   p90 7,28
o vao vai de 6,64 a 7,29   ->  centro 6,96
```

➡️ **Ele chega 24 cm à direita do centro.** Com meia-largura de 0,2775 m, a
borda direita dele fica em **7,48** — e a parede está em **7,29**. Rumo certo,
posição errada. `|desvio lateral|` p50 0,195 m, p90 0,282 m.

O alvo NÃO está errado (ele mira 6,99). O robô mira o centro e não chega lá,
porque fica parado tentando. `bin/pause_budget.py` sobre a mesma corrida, dos
311 s parados com objetivo vivo:

```
movimentacao_muda[-]                 205,1 s   ninguem cortou: a lei nao pediu
collision                             47,2 s   o reflexo cortou comando vivo
movimentacao_muda[STOP:PolygonStop]   24,2 s
```

## ⚠️ A suspeita para a próxima mudança (não testada ainda)

A repartição das fases:

```
gargalo_centro     27 amostras
gargalo_eixo     2161 amostras
```

A fase de CENTRAGEM dura quase nada e ele se compromete com o eixo — e
`eixo_comprometido` não volta atrás. `passagem_alinha_lateral: 0,08` diz que
ele deveria centrar até 8 cm antes de assumir o eixo; ele assumiu com 22.

**É uma coisa só para testar da próxima vez**, e ela tem número de aprovação:
`x` na soleira dentro de 6,96 ± 0,07 m.
