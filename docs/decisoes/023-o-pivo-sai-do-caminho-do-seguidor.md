# 023 — O pivô sai do caminho do seguidor: a placa não entrega módulo

**Data**: 2026-08-12 (4ª leva; dev, robô desligado)
**Status**: aceita e implementada
**Toca**: `heading_controller.py`, `path_follower.py`,
`config/movimentacao_sim.yaml`, `test/test_lei_de_pivo.py`,
`test/test_configs_coerentes.py`
**Vem de**: as quatro corridas de `docs/dados/2026-08-12-sim-meu-mapa/`, relidas
**Revisa**: a fatia 3 da decisão 011 (o pivô por corte), sem apagá-la

## O problema, e o que ele NÃO era

O `ESTADO_PROJETO` de 12-08 abria dizendo que o robô *"não percorre o plano.
Nunca percorreu — nem no simulador, nem no robô"*, e apontava a lei de
velocidade do seguidor como suspeita.

**As duas coisas estavam erradas, e a segunda vinha de um artefato de leitura.**

O robô percorreu o plano duas vezes, as duas registradas no diário:

| quando | evidência |
|---|---|
| 29-07 (9ª leva) | alvo pela porta de 0,90 m: chegou em 12,3 s, 5,55 m de caminho / 5,32 reta = **1,04×** |
| 05-08 (6ª leva) | as batidas: **2509 → 0 invasões**, folga mínima 0,449 m contra 0,45 teórico, *"passou perfeitamente no meio"* |

E o *"erro de rumo é ZERO e ele não pede velocidade"* que motivou a suspeita da
lei de velocidade é aritmética de amostra morta. Em `pista-fixa-a.csv`:

```
amostras de erro_rumo               1494
zeros exatos                         847
amostras depois de o robô congelar   847   (1500 − 653)
```

Batem na unidade. Todos os zeros vêm do trecho em que o seguidor **já tinha
parado** — e `para()` publica `rumo_alvo = rumo atual` de propósito, o que
fabrica erro zero. A mediana estava medindo o robô desligado. Na fase viva,
`|erro| > 15°` em **530 de 964** amostras.

## O que os dados dizem de verdade

Nas quatro corridas, a mesma assinatura: **o robô gira no lugar e não sai.**

```
                fase viva   giro total   deslocamento   raw_v≠0
pista-fixa-a      32,6 s      1321°         0,20 m       1/1500
pista-limiar45    31,9 s      1260°         0,06 m      14/1800
pista-amcl-a      15,1 s       647°         0,11 m       0/1500
objetivo-a         8,3 s       382°         0,23 m       0/2563
```

Um ciclo, à resolução do CSV, com o `wz` derivado do yaw:

```
 t     yaw°   wz_real   erro°   raw_wz
 6,00   41,5   +0,00    +44,7    1,0   dispara o pivô
 6,40   41,6   +0,05    +50,6    1,0   0,40 s de comando sem o robô se mexer
 7,05   60,0   +0,72    +32,2    0,0   CORTA — a lei prevê 25–32° de sobra
 7,50   91,8   +1,74     −0,3    0,0   em cima do alvo, ainda ACELERANDO
 8,75  153,0   +0,00    −50,5    0,0   varreu 93° depois do corte
 8,80  153,0   +0,00    −50,5   −1,0   dispara para o outro lado
```

Ciclo-limite sustentado, período ~5,5 s, amplitude ±50 a 60°.

**Os 28 disparos das quatro corridas tiveram erro entre 35° e 81°.** A varredura
pós-corte foi de **93–101°** (n=5, repetível). Nenhum dos 28 era executável.

## A causa

A placa entrega **um único módulo de giro, sempre o mesmo**. A aritmética é a do
`hoverboard_driver.cpp`, reproduzida no `placa_simulada`: a compensação de zona
morta escala as duas rodas pelo mesmo `k` até a maior vencer o `deadband_speed`.
Para um pivô:

```
pedido 0,10 rad/s  ->  entrega 2,204 rad/s
pedido 0,30 rad/s  ->  entrega 2,204 rad/s
pedido 1,00 rad/s  ->  entrega 2,204 rad/s
                       medido no CSV de 12-08: pico 2,152 rad/s
```

E depois do corte ela **segura a saída cheia** por `atraso_desliga` (0,52 s,
medido no robô em 04-08, n=3), decaindo em rampa. A docstring do próprio
`placa_simulada` já dizia o que isso significa: *"entre o corte e o pico de `wz`
passam 0,40–0,60 s, e nesse trecho o robô ainda ACELERA"*.

Daí sai o resultado central, e ele é estrutural, não de sintonia:

> A sobra **prevista** é `wz²/(2·a_dec)` — uma parábola no `wz` do corte.
> A sobra **real** é a retenção, `patamar·atraso/2` ≈ 33° no piso e 93–101° como
> de fato acontece — e ela **não depende do `wz` do corte**.
> Parábola não casa com constante em valor nenhum de `a_dec`.

Varrido: `a_dec` de 0,60 a 0,05, alvos de 20° a 180°. O acerto vira sorteio por
ângulo, não tendência. **A premissa da lei — *"a única alavanca é decidir quando
cortar"* — não vale contra esta placa.**

## Por que só apareceu agora

A linha do tempo fecha por git, e o commit da corrida boa é **ancestral** do
commit que quebrou:

```
9f00854  05-08  batidas do Nav2: 2509 -> 0 invasões, passa no meio da porta
a87cc9a  05-08  reflexo de colisão
9b8ce08  05-08  plano de teste no robô
77423da  05-08  bancada 05-08: o pivô não é dirigível
6aa347a  05-08  simulador: as duas pontas da placa   <- a retenção entra aqui
```

A porta foi atravessada contra um simulador cuja placa **ainda não retinha**. A
retenção entrou quatro commits depois, e a aceitação daquela mesma leva registra
*"O pivô, n=3 por ponto — **NÃO passou**"*. O defeito estava escrito no diário
desde 05-08; o que faltou foi ligá-lo à navegação, e entre 05-08 e 12-08 a
corrente inteira não voltou a rodar ponta a ponta.

⚠️ **E a planta de brinquedo do teste tinha o mesmo ponto cego** — parente exato
do achado de 06-08, quando a planta do rumo modelava 0,26 s de atraso contra os
0,94 s medidos. Aqui ela **congelava** o `wz` por 0,2 s depois do corte em vez de
segurar a saída cheia por 0,52 s. Com a retenção no lugar, **10 dos testes do
pivô caem na hora**.

## A decisão

**O pivô por corte sai do caminho do seguidor.** Quem responde por erro de rumo
volta a ser a lei contínua da 005 (`lei_de_rumo.comando`), em toda a faixa.

```
heading_controller  limiar_pivo   0,26 -> 3,20 rad   (acima de π: nunca dispara)
movimentacao_sim    limiar_pivo   0,79 -> 3,20 rad
heading_controller  o caso `parado` sai de `precisa_pivo`
path_follower       aponta_no_fim True -> False
```

### Por que a lei contínua sobrevive à mesma placa

Medido offline, malha fechada contra a placa **com** retenção:

```
alvo   excursão   |erro| nos últimos 10 s   andou (v>0)
 20°      20,0°            0,5°               99,5%
 45°      45,0°            0,6°               99,5%
 90°      90,0°            0,6°               98,8%
135°     135,0°            0,5°               98,1%
180°     180,0°            0,5°               97,5%
```

Assenta em 0,5–0,6° de qualquer ângulo, sem sobrepasso, andando quase o tempo
todo. E a razão cabe numa linha:

> A compensação do driver escala as duas rodas **juntas**: preserva a RAZÃO e
> destrói o MÓDULO. **Arco é razão** — a placa entrega o raio pedido (0,515 m
> pedido, 0,515 m entregue, conferido de 20° a 75°). **Pivô é módulo** — e é
> exatamente o que ela não sabe entregar.

É a mesma frase da decisão 020 (*"`cmd_vel` escolhe o RAIO, e a velocidade quem
escolhe é a placa"*), aplicada a uma manobra que só tem módulo.

### Por que `aponta_no_fim` cai junto

A fase 2 da chegada pede `v=0` mais um ângulo, e quem entregava isso era o pivô.
Sem ele, a lei contínua assume e **não arrasta** — o medo escrito no
`heading_controller` (*"aplica piso de linear e ARRASTA o robô para fora do
ponto"*, o defeito de 27-07) não se materializa, e a conta é de uma linha: a lei
recebe `v_max = v_alvo`, e com ele em zero o `v_teto` fecha a saída "por cima".
Medido de 2° a 150°, nos dois perfis: `v` sai **0,000** em toda a faixa.

Mas ela também não gira abaixo de 90° no perfil do simulador (zona morta crida
0,10). Deixar ligado seria pendurar a chegada esperando um ângulo que a máquina
não fecha — **parado, em silêncio, que é o BO-3**. E o requisito nunca foi do
seguidor: `comando_de_parada` já diz que *"rumo na chegada não é requisito deste
seguidor, e persegui-lo custa a própria chegada"*, e o `nav2.yaml` já roda com
`use_final_approach_orientation: false`.

## Alternativas descartadas

**Baixar `pivo_a_dec`** (o conserto óbvio, e o que a própria lei sanciona ao
dizer *"errar para baixo é de graça"*). Descartado **com medida**: a varredura de
`a_dec` de 0,60 a 0,05 não fecha a faixa em valor nenhum, pela razão estrutural
acima. A condição de segurança da lei (`a_dec suposto ≤ a_dec real`) está violada
por ~3,7× e continuaria violada, porque o termo que domina não é desaceleração.

**Subir mais o `limiar_pivo`, guardando o pivô para virada grande.** Foi a
tentativa de 12-08 (15° → 45°) e falhou: 11 disparos viraram 10 e o robô
continuou parado. E os testes mostram que a manobra também não fecha a 135° nem
a 180° — não existe faixa segura para guardá-la.

**Modelar a retenção dentro do critério de corte.** É o conserto de mérito e fica
registrado como caminho: a sobra passaria a ser `retido·atraso/2 + wz_pico²/(2·a)`
em vez de `wz²/(2·a_dec)`. Não entra hoje porque exige a lei conhecer a placa —
o oposto do que a fatia 3 buscou — e porque **destravar a navegação não depende
disso**. Reabre quando houver motivo de campo para o robô virar no eixo.

## O que isto NÃO resolve

- **Não foi ao Gazebo.** Tudo aqui é offline: aritmética da placa, malha fechada
  contra planta de brinquedo, e releitura de CSV. A planta de brinquedo acerta a
  estrutura e erra escala — em 05-08 ela errou o pivô por ~1,6×.
- **Não foi ao robô.** A retenção de 0,52 s é medida no robô real (04-08), e a
  compensação do driver é a mesma lá, então a previsão vale para os dois — mas
  vale como previsão.
- **A velocidade segue não-comandável** (dívida aberta desde a 020): o robô
  percorre qualquer arco a ~0,30 m/s porque a placa escolhe o módulo. Isso não
  impede navegar; impede frear na curva e chegar devagar.

## Previsão falsificável

Uma corrida no Gazebo, na pista, mesmo alvo das corridas de 12-08:

> **`raw_v` não-nulo em mais de 80% das amostras, e a distância ao alvo caindo
> de forma monótona até o raio de chegada.**

Se o robô voltar a girar no lugar, a causa **não** é o pivô — ele não pode mais
disparar — e a suspeita seguinte passa a ser o plano do Theta* saltando entre
replanejamentos (o `rumo_alvo` já está no CSV do seguidor para isso).
