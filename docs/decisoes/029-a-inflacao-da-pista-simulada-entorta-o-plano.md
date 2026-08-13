# 029 — A inflação da pista simulada entorta o plano na sala real

**Data**: 2026-08-13 (robô ligado, duas corridas de controle)
**Status**: aceita, implementada e **medida no robô**
**Toca**: `robot_motion/config/nav2.yaml` (os dois costmaps)
**Vem de**: a primeira navegação COM MAPA deste robô (decisão 028, diário 13-08)

## O problema, visto a olho antes de estar no CSV

Primeira corrida com o mapa próprio, alvo a 2,5 m, `inflation_radius: 0.50` — a
de produção. O robô **chegou sem encostar em nada**, e mesmo assim estava
errado. O dono, olhando: *"não ta nada parecido com aquele ponto reto com nav2
que ficou perfeito, ele fica perdidinho até chegar no goal"*.

O CSV, fase de movimento (0 → 26,1 s):

```
referência de rumo (rumo_alvo)   83,8° de excursão   (desvio 16,4°)
yaw do robô                     103,2°
saída de velocidade não-nula     41,9% do tempo  -> parado 58%
caminho                          3,40 m para 2,35 m de reta  = 1,45x
plano                            mudou 11x em 26 s
```

**Quem passeia é o PLANO, não o robô**: a referência excursiona 83,8° e o yaw
obedece. É o mesmo veredito de 12-08 (lá foram 30,6° de referência para 32,9°
de yaw), agora muito pior — e o que mudou entre as duas foi o mapa entrar.

## A causa

A `inflation_radius: 0.50` foi escolhida em **05-08 para a porta de 0,90 m da
pista SIMULADA**, e lá ela é a virtude que centra o robô no vão. A sala real do
andar 3 tem 6,5 × 4,4 m e mobília. No mapa que o robô desenhou hoje:

```
células livres a menos do robot_radius (0,32 m) de alguma coisa:  35,4%
```

Com inflação de 0,50 m em cima disso, **quase toda a sala fica cara**. E o
efeito não é o que o roteiro previa (`Start occupied`, recusa de planejar): é
pior de diagnosticar, porque o planejador **aceita** e devolve caminho torto,
costurado entre gradientes de custo que mudam a cada replanejamento.

⚠️ Isto reclassifica a dívida nº 2 do `ESTADO_PROJETO.md` ("a inflação pode
proibir planejar"). Ela não proibiu: com 0,50 o `plano.py` planejou 1,00× reto
**com o robô parado**. Costmap parado e costmap sendo atualizado a 1 Hz com o
robô andando são coisas diferentes, e só a segunda entorta.

## A decisão

`inflation_radius: 0.50 → 0.20` nos **dois** costmaps.

## A previsão, escrita ANTES da corrida, e o que deu

> "Se a inflação for a culpada, a excursão da referência cai muito (para a casa
> de 20–30°) e a tortuosidade cai de 1,83. Se não cair, a culpa é do seguidor e
> a inflação está inocente — e aí eu mudo de suspeito."

Mesmo alvo, mesma pilha, mesma pose de partida, única mudança a inflação:

```
                   chegou    referência    yaw     v não-nula   caminho/reta
inflação 0,50      26,1 s      83,8°      103,2°      41,9%        1,45x
inflação 0,20       8,1 s      23,4°       14,0°      87,1%        1,02x
12-08, SEM mapa     9,5 s      14,7°       10,0°        —          1,09x
```

**23,4°.** E 1,02× é reta. Com mapa e AMCL o robô ficou igual à corrida sem mapa
— e mais rápido que ela. O dono, olhando a mesma corrida: *"seguiu
indiscutivelmente bem"*.

## As alternativas descartadas

**Mexer no `cost_scaling_factor` em vez do raio.** Descartada por ora, não por
ser ruim: ela é a outra metade da mesma paisagem de custo (o YAML já registra
que fator baixo é o que centra o robô no vão). Mexer nas duas juntas deixaria a
corrida sem previsão separadora — o erro de 11-08 com o estimador do ff. Fica
como próxima leva **se** a 0,20 mostrar problema de centralização em vão
apertado.

**Aceitar o passeio e compensar no seguidor** (mira mais longa, ganho menor).
Descartada: o seguidor está obedecendo corretamente uma referência ruim. Sintonizar
o seguidor para seguir mal um plano torto é esconder a causa — e foi exatamente
o que 12-08 já tinha descoberto ao falsificar a hipótese do `lookahead`.

**Deixar 0,50 e alargar a sala.** Não é piada: era a alternativa real de tirar
móveis do caminho. Descartada porque o critério do projeto é o robô desviar do
que é parado e está no mapa, não o mundo se ajustar ao robô.

## O que esta decisão NÃO resolve

A `0,20` é menor que o raio inscrito de trabalho que centrava o robô na porta de
0,90 m da pista simulada. **Se voltarmos a passar por vão apertado, este número
volta à mesa** — e aí a leva certa é `cost_scaling_factor`, com corrida de
controle. Anotado no próprio YAML.

## Referências

- decisão 028 (o mapa próprio) — sem ele esta medida não existiria;
- diário 12-08 (8ª a 12ª leva) — a referência que oscila e o `a_dec` 0,1;
- `ESTADO_PROJETO.md`, dívida nº 2 — reclassificada por esta decisão.
