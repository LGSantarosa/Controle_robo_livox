# 009 — A ré é por gatilho, e o plano não dá ré

**Data:** 2026-07-29
**Estado:** aceita (pelo dono, 2026-07-29)
**Revisa:** [008](008-nav2-planeja-nos-seguimos.md), seção 3

---

## Contexto

A decisão 008 fechou horas atrás dizendo que **a ré passa a nascer do
planejamento**: com `motion_model_for_search: REEDS_SHEPP`, o caminho inteiro já
sabe onde a ré entra, em vez de ser manobra decidida no susto. O argumento era
bom no papel e tinha 48 planos por trás.

O dono derrubou com razão de campo: **ré planejada em robô com Nav2 é
problemática — o robô fica tentando entrar e seguir os trechos de ré.** Foi por
isso que ele tirou a ré do planejamento no robô 1 e criou lá a ré por gatilho.

Isso não é opinião contra dado: é dado de operação contra dado de geometria. E a
bancada do planner, por construção, **não pode** arbitrar — ela desenha, não
dirige. O que ela pode fazer é medir o custo de cada saída, e foi o que se fez
antes de decidir.

## O que foi medido antes de decidir

### 1. Encarecer a ré não a elimina

`reverse_penalty` de **2 → 10 → 40** (20×), no raio de produção (0,37 m):

| caso | inversões com 2 | com 10 | com 40 |
|---|---|---|---|
| porta | 2 | 2 | 2 |
| bloco | 3 | 4 | 2 |
| perto_de_lado | 2 | 2 | 2 |
| lado_1m | 2 | 2 | 2 |

Praticamente nada muda. A ré não é oportunismo do planner que um preço corrige:
nesses casos ela é **geometricamente necessária**. A opção do meio não existe.

### 2. Proibir a ré no plano tem custo, e é assimétrico

Mesma varredura com `DUBIN` (só-para-frente), 4 raios × 6 casos
(`docs/dados/2026-07-29-varredura-raio-planner-dubin.csv`):

| caso | Reeds-Shepp | Dubins |
|---|---|---|
| porta · aperto · beco | 1,05–2,32 | 1,05–2,25 — **igual** |
| bloco | 1,39–1,54 | **NO_VALID_PATH nos 4 raios** |
| perto_de_lado | 1,50–2,12 | **3,61 → 5,66** |
| lado_1m | 1,32–1,77 | **2,24 → 4,10** |

Metade dos casos não sente. A outra metade sente muito: um alvo a 0,60 m passa a
custar 2,9–3,4 m de caminho com ~350° de giro — a volta inteira. E é literalmente
o **balão** que motivou trazer o Nav2 para a conversa, agora nascendo do plano.

### 3. Recuar NÃO salva o plano do Dubins — a hipótese foi testada e caiu

A tese natural da ré por gatilho é: o robô recua, pede plano novo, e o caso caro
deixa de existir. Medido, com `DUBIN` e raio 0,37, recuando reto antes de
planejar:

| | recuo 0 | 0,3 m | 0,5 m |
|---|---|---|---|
| perto_de_lado | 2,92 m | 2,99 m | 3,20 m |
| lado_1m | 3,31 m | 3,16 m | 3,30 m |
| bloco | NO_VALID_PATH | NO_VALID_PATH | NO_VALID_PATH |

O comprimento **não cai**. A razão é geométrica e vale para qualquer recuo:
recuar reto não muda o RUMO. O alvo continua a 90° do bico, e um carro
só-para-frente precisa da mesma volta, saia de onde sair.

### 4. O que sustenta a ré por gatilho é o PIVÔ, não a ré

O robô 1 se dá bem com ela porque **pivota**: recua, gira no lugar, e o alvo de
lado vira alvo de frente. É o chassi skid-steer, que só sabe girar parado.

O robô 2 não pivota com os parâmetros de hoje — medido em 29-07, **zero amostras
de giro parado** em toda a fase B do perfil pessimista de zona morta. Sem pivô,
recuar não abre a geometria, e é exatamente essa parede que as decisões 006 e
007 bateram, duas vezes.

## Decisão

### 1. O planner não dá ré: `motion_model_for_search: DUBIN`

Vale a experiência de operação. O custo geométrico está medido e aceito, e a
patologia que ele evita — o robô oscilando para entrar e sair de trecho de ré,
com as cúspides mudando de lugar a cada replanejamento — é justamente a que a
bancada não sabe medir e o dono já pagou.

Some junto, de graça, o **tremor em cima do alvo** que a varredura de 29-07 achou
com Reeds-Shepp: 4 inversões dentro de uma caixa de 9 cm no caso `bloco`, depois
de 4,86 m limpos. Era a mesma doença, e eu a tinha arquivado como problema do
seguidor.

### 2. A ré é do SEGUIDOR, disparada por SINTOMA

O robô recua quando **não está progredindo**, não quando a geometria prevê que
não vai progredir. Isso inverte o gatilho da decisão 007, que era geométrico e
antecipado — e cujo ciclo "ré e anda" foi reprovado duas vezes com dado (28-07
no clique do dono, 29-07 na bancada: 5 entradas em ré e 1,85 m de caminho para um
alvo a 0,40 m).

Por que o sintoma agora, se a 007 o descartou: lá o argumento era que sintoma
"gasta N segundos de órbita toda vez". É verdade, e é **o preço certo a pagar** —
o gatilho geométrico age cedo demais e entra em ré em situações que se
resolveriam sozinhas, que é como nasce o vai-e-volta. Sintoma dispara raro e
tarde; geometria dispara cedo e sempre.

### 3. A ré é reta, com orçamento, e cega por enquanto

Reta pela razão da 007, que continua de pé: andando para trás a boba deixa de ser
arrastada e passa a ser empurrada, e não existe medida nossa dessa configuração
(BO-4).

**Cega** porque hoje não há sensor traseiro no robô simulado: o Mid-360 é 360° e
vai enxergar atrás, mas ainda não está no modelo. Enquanto não estiver, a ré anda
com **orçamento curto e obrigatório** em metros e segundos, e grita ao estourar.
Quando o lidar entrar, o orçamento ganha um vão medido em metros por trás, e não
antes.

## O que fica em aberto, e o que reabre esta decisão

**A zona morta.** Ela decide se o robô pivota, e o pivô é o que faz a ré por
gatilho funcionar de verdade:

- **se o robô pivotar** — esta decisão é confortavelmente a certa: recua, gira,
  e o alvo de lado vira alvo de frente, como no robô 1;
- **se não pivotar** — proibir a ré no plano custa ~3 m de caminho para um alvo
  a 0,6 m, e o caso `bloco` fica sem solução. Aí a 008 seção 3 volta à mesa,
  **com número e não com opinião**.

É o item nº 1 da bancada com o robô desde 27-07, e agora tem uma terceira razão
de peso.

## Alternativas descartadas

**Manter Reeds-Shepp e encarecer a ré.** Medido: 20× no `reverse_penalty` não
muda as inversões. A ré nesses casos não é escolha do planner, é geometria.

**Manter Reeds-Shepp e resolver a oscilação no seguidor.** É apostar que
consertamos no seguidor uma patologia que o dono já enfrentou em campo e que a
nossa bancada não sabe reproduzir. Custo de errar: alto e tardio — aparece só com
o robô andando.

**Reeds-Shepp só para os casos que o Dubins não resolve.** Dois planners no ar
com regra de troca é mais uma máquina de estados sem medida por trás, e a regra
de troca seria exatamente o gatilho geométrico que a 007 já reprovou.

## Referências

- `docs/dados/2026-07-29-varredura-raio-planner-dubin.csv` — os 48 planos Dubins
- `docs/dados/2026-07-29-varredura-raio-planner.csv` — os 48 Reeds-Shepp da 008
- Entrada de 29-07 (7ª leva) do `docs/DIARIO.md` — a varredura de penalidade e o
  teste de recuo, que não viraram CSV por serem corridas de uma pergunta só
