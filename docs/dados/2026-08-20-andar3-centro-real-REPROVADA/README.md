# 2026-08-20 — mirar o meio das paredes: REPROVADA, e revertida

Hipótese: o alvo da travessia é o ponto do PLANO, e na porta 2 o plano passa
**3 cm** fora do meio das duas paredes — 40% da folga de 7,2 cm que a caixa do
reflexo deixa. Mirar o meio medido deveria recuperar esses 3 cm.

Implementado (`desvio_do_centro` na detecção + alvo deslocado, com limite para
erro de rasterização não jogar o alvo na parede), 374 testes passando, e
rodado na mesma prova.

## O veredito

```
corrida                          x p50   x p10-p90   |rumo| p50   collision
sem gargalo                       7,20      -           14,6°       15%
gargalo ligado                    7,20   7,13-7,28       4,1°       15%
+ latch com saida                 7,07   6,99-7,09       5,4°       43%
+ centro real do vao              7,03   6,83-7,21       8,5°       49%   <-
```

**4 cm de ganho na mediana, pagos com quase o dobro da dispersão** (0,10 →
0,38 m entre p10 e p90) **e o rumo indo de 5,4° para 8,5°**. A fase `centro`
saltou de 167 para 908 amostras: mirar um ponto a menos de 1 m faz o rumo
oscilar, que é o mesmo mecanismo do carrot curto. Ele passou a caçar o centro
em vez de atravessar.

11 rés contra 7. Não passou.

➡️ **Revertida.** Um ganho de 4 cm na mediana não paga 28 cm de dispersão.

## O que fica desta tentativa

A **medida** continua verdadeira e vale registrar: o plano passa ~3 cm fora do
meio do vão. Só que perseguir isso com o alvo custa mais do que rende. Se um
dia valer a pena recuperar esses 3 cm, o lugar é o PLANO (o gradiente de
inflação que centra o caminho), não o alvo do seguidor.

## E o que a corrida mostrou de mais importante

`collision` chegou a **49,3%** do tempo parado, e o robô ATINGE `x = 6,83` em
p10 — ou seja, ele consegue ficar centrado, e mesmo assim não passa.

```
vao 0,700   corpo 0,455 (folga 12,2 cm/lado)   caixa do reflexo 0,555 (7,2 cm)
```

**O limite deixou de ser a centragem e passou a ser a caixa.** Centrar mais não
resolve mais nada: é a saída 2 (a caixa na travessia, decisão 041) que decide.
