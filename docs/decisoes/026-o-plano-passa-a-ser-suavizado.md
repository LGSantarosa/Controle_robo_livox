# 026 — O plano passa a ser suavizado, e isso NÃO bastou

**Data**: 2026-08-12 (7ª leva; dev, corridas no Gazebo)
**Status**: aceita e implementada — **e o resultado principal é NEGATIVO**
**Toca**: `config/nav2.yaml`, `launch/pilha.launch.py`, `setup.py`,
`behavior_trees/replanejamento_com_suavizacao.xml` (novo),
`tools/banco/suavizador.py` (novo)
**Vem de**: o critério do dono — *"o reflexo só deve parar em surpresas; o que
é parado deve ser desviado previamente pela navegação"*

## O problema

Depois da 025 o robô atravessa a porta, mas **usando a ré**, e o dono nomeou
isso corretamente: a ré é **band-aid**. Ele vai errado em direção à parede
desde o começo e precisa do recuo para não bater.

A medida que separa planner de seguidor:

```
porta: parede em x=4,0 · vão de y=2,05 a 2,95 · CENTRO 2,50 · corpo 0,63 m

o PLANO do Theta*      cruza em y=2,483   (−0,017 m do centro)  sobra +0,118 m/lado
o CAMINHO REALIZADO    cruza em y=2,401   (−0,099 m do centro)  sobra +0,036 m/lado
```

O plano está bem POSICIONADO. Mas não é **seguível**: reamostrado a 0,20 m ele
tem 14 viradas acima de 2°, com **23,0° a 0,46 m da porta** e 15,7° a 0,23 m —
dentro da mira de 0,37 m do seguidor, onde a cenoura salta por cima da quina.

## A decisão

Entra `smoother_server`, e com ele a **primeira árvore de comportamento própria
do projeto** — porque nenhuma das doze de fábrica do Jazzy chama `SmoothPath`.
A árvore preserva a propriedade de 29-07: **recuperação zero**.

⚠️ `SmoothPath` vive dentro de um `RecoveryNode` com `AlwaysSuccess`: falha do
suavizador **não derruba a navegação**, o `FollowPath` segue com o plano cru.
Suavização é melhoria, não pré-requisito.

### A escolha do suavizador saiu de medida, e derrubou o favorito

`tools/banco/suavizador.py` pede o plano UMA vez e entrega aos três — comparar
em corridas separadas é comparar fotos de dias diferentes. n=4:

```
              quina máx (4 corridas)              mediana   folga mín
cru           45,0 · 23,3 · 30,1 · 36,9            33,5°     0,450
simples       19,9 · 22,6 · 22,8 · 26,4            22,7°     0,454
savgol        24,2 · 33,5 · 33,6 · 35,6            33,6°     0,452
suave         NÃO COMPLETOU nas 4 (error_code 504)
```

**`simples` (SimpleSmoother) ganha**, e o ganho maior não é a mediana: é o
**espalho**, que cai de 23–45° para 20–26°. Folga contra o mapa nunca piorou e
invasões foram 0 nas quatro — suavizar **não** empurrou o caminho para a
ombreira, que era o risco real num vão de 0,90 m.

⚠️ **O `ConstrainedSmoother` era o favorito a priori** — é o único que olha o
costmap e respeita raio de curva — e **falhou nas quatro**. Fica no `nav2.yaml`
para a bancada continuar comparando: ele está fora por não funcionar, não por
ter perdido.

🔧 **E a bancada mentiu na primeira versão.** Ela não mandava
`max_smoothing_duration`, o campo chegou ZERO, o servidor abortou com
*"Smoothing time exceeded allowed duration of −0.00"* — e ela imprimiu os
quatro caminhos **idênticos** como se fosse medida, sugerindo que suavizar não
fazia diferença. Agora ela recusa resultado que não completou. **Instrumento
que devolve número sem ter medido é pior que instrumento quebrado.**

## 🔴 O RESULTADO PRINCIPAL É NEGATIVO

O suavizador agiu 30 vezes na corrida da porta, sem falhar. E não mudou o
comportamento:

```
                       antes do suavizador    com SimpleSmoother
chegada                    0,082 m                0,070 m
tortuosidade               1,30                   1,31
rés                        1                      1
reflexo                    2× em t=12,0 s         2× em t=12,3 s

erro de trajeto p50        0,174 m                0,128 m
                p90        0,297 m                0,321 m
                máx        0,325 m                0,347 m
```

A mediana do erro de trajeto cai 26%, **mas p90 e máximo pioram**, e nada no
comportamento muda. **O critério de aceitação do dono não foi atingido**: o
reflexo continua disparando contra um obstáculo que está no mapa.

➡️ **A leitura honesta**: a alavanca do lado do Nav2 está quase esgotada. O
plano já cruza a 1,7 cm do centro e a quina caiu de 33,5° para 22,7°; o robô
segue perdendo 10 a 17 cm. **O termo dominante é o seguidor**, e a suspeita
ordenada é:

1. **a lei é só de RUMO** — `rumo_para(x, y, carrot)` corrige desvio lateral só
   implicitamente, com atraso. Pure pursuit tem desvio permanente conhecido em
   curva, e a porta vem logo depois de uma;
2. **`lookahead_fator` 1,0 (mira de 0,37 m)** foi escolhido em 05-08 contra um
   seguidor que PIVOTAVA. Sem pivô (023), mira curta faz virar tarde;
3. **sem pivô ele arca para dentro** em vez de se esquadrejar antes do vão.

## O que isto NÃO resolve

- **Nada disto foi ao robô.** As quinas medidas são do Theta* sobre o mapa da
  pista; o mapa do prédio (`meu_mapa`) tem outra geometria.
- **`max_smoothing_duration` = 0,2 s na árvore** não foi varrido. Se o
  suavizador estourar o orçamento em mapa maior, ele devolve o plano cru — em
  silêncio para o robô, mas com WARN no `rosout`.
