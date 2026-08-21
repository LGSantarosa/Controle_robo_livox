# 2026-08-20 — o latch do eixo ganha saída: 24 cm → 11 cm de descentragem

Terceira corrida da mesma prova. Muda UMA coisa em relação a
`2026-08-20-andar3-gargalo-ligado`: `alvo_estavel_de_passagem` deixa a fase
`eixo` e volta a centrar quando o desvio passa da folga do vão.

## As três corridas, lado a lado

```
                        x na soleira   |erro rumo| p50   fases centro/eixo
sem gargalo               7,20 (p50)        14,6°            -
gargalo ligado            7,20              4,1°          27 / 2161
gargalo + latch com saida 7,07              5,4°         167 /  418
                          ^^^^
alvo: 6,96 +/- 0,07 m   (a folga da CAIXA do reflexo, ver abaixo)
```

**Ganho real: 13 cm de centragem.** A fase de centro passou de 27 para 167
amostras — o robô agora volta a centrar em vez de congelar o primeiro
alinhamento. **E ainda não passou** (y máximo 18,81 contra a porta em 19,0).

## 🔴 Quem segura agora: o REFLEXO, e a conta explica

`bin/pause_budget.py`, dos 151 s parados com objetivo vivo:

```
collision                             65,6 s   (43,4%)   <- era 15,2%
movimentacao_muda[-]                  29,2 s   (19,3%)
movimentacao_muda[DO_NOTHING:-]       23,5 s   (15,5%)
movimentacao_muda[STOP:PolygonStop]   15,6 s   (10,3%)
```

Conforme o robô centra e insiste, o gargalo migra de "a lei não pediu" para
**"o reflexo cortou"**. E é geometria pura:

```
vao da porta 2                 0,700 m
CORPO real                     0,455 m   -> folga 12,2 cm por lado
caixa PolygonStop do reflexo   0,555 m   -> folga  7,2 cm por lado
   (points [[0.35, 0.2775], ...] no collision_monitor.yaml)

robo a 11 cm do centro  ->  a CAIXA invade a ombreira em ~4 cm  ->  STOP
```

➡️ **O corpo caberia; a caixa não cabe.** O critério `6,96 ± 0,07 m` não é
arbitrário: 7 cm é exatamente a folga que a caixa do reflexo deixa. Enquanto a
centragem for pior que isso, o reflexo veta a travessia — corretamente, do
ponto de vista dele.

## As duas saídas para a próxima mudança

1. **centrar mais 4 cm** — o latch com saída trouxe 13; falta pouco;
2. **estreitar a caixa durante a travessia** — é a discussão da decisão 041
   (*"a quina de trás do reflexo come a porta"*), e tem preço: a caixa também
   responde por distância de parada.

Uma de cada vez, com o mesmo critério de aprovação.
