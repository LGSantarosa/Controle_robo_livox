# 008 — O Nav2 planeja com Smac Hybrid-A\*, e o seguidor é nosso

**Data:** 2026-07-29
**Estado:** proposta — aguardando o dono
**Depende de:** [003](003-base-ros2control-e-lio.md), [005](005-lei-de-frenagem-de-rumo.md)
**Revisa:** [006](006-navegacao-ponto-a-ponto.md), [007](007-re-como-manobra.md)

---

## Contexto

Em 28-07 o dono dirigiu o robô no simulador clicando no RViz e reprovou o
resultado. O número que resume a sessão: **0 amostras de giro parado em 2714** —
ele nunca virou no próprio eixo, e um alvo a 0,43 m custou 3,66 m de caminho e
57 s. O CSV está em `docs/dados/2026-07-28-cliques-movimentacao.csv`.

Duas causas distintas, e só uma era da ré:

- o ciclo "ré e anda" (12 entradas, período 2,10 s) é da histerese da decisão
  007, que solta a manobra assim que o alvo cabe *naquele instante*;
- o **balão** é da movimentação: `raio = v/wz`, com o teto de giro em 1,0 rad/s
  e o piso de linear que a zona morta obriga. Girando a 1,0 rad/s o robô é
  obrigado a andar a 0,23 m/s.

O dono decidiu ali: **aposentar a navegação ponto a ponto** (decisão 006 e a ré
da 007), trazer o **Nav2**, e por ora julgar **só o planner** — porque o defeito
nasceu na movimentação, e julgar planner junto com quem executa mistura as
culpas.

Esta decisão fecha esse julgamento.

## O que foi medido

`ros2_packages/robot_planning/` roda o `planner_server` do Nav2 com dois
planners no ar ao mesmo tempo, respondendo ao MESMO par (partida, destino), numa
pista com porta de 0,90 m, bloco solto, aperto de 0,80 m e beco sem saída
(`tools/mundo/gera_pista.py`, que gera mapa do Nav2 e mundo do Gazebo da mesma
planta). Sem robô, sem simulador, sem sensor.

`tools/planner/varredura_raio.py` rodou **48 planos**: 6 casos × 4 raios mínimos
× 2 planners. Dados em `docs/dados/2026-07-29-varredura-raio-planner.csv`.

### Por que quatro raios, e não um

O `minimum_turning_radius` não é um parâmetro qualquer desta comparação: ele
**é** o argumento dela. O Smac está na disputa justamente por respeitar raio de
curva; informá-lo otimista dá a vitória ao Smac num robô que não existe.

E o número certo não é sabido. A corrida de bancada de 29-07
(`tools/banco/corrida_gazebo.py`) mediu o raio realizado (p5) em **0,370 m** com
zona morta 0,10 e **0,463 m** com 0,15 — e o realizado abre em relação ao pedido
porque o giro entrega só 79% do comandado. Qual dos dois vale depende da zona
morta, que segue sem medir no robô.

Varrer em vez de escolher era a única saída honesta: se o ranking não mudasse
com o raio, a conclusão estaria imune à medida que falta.

### O resultado

**Seguível** = o raio do caminho cabe no que a máquina fecha (com 10% de folga
para a discretização). Reta lateral conta como não-seguível: ela só serve a robô
que pivota, e este não pivota com a zona morta pessimista — medido em 29-07,
**zero amostras de giro parado** em toda a fase B do perfil real.

| raio que a máquina fecha | Theta\* | Smac Hybrid-A\* |
|---|---|---|
| 0,25 m | 4/6 | **6/6** |
| 0,34 m | 3/6 | 5/6 |
| 0,37 m | 3/6 | **6/6** |
| 0,46 m | **0/6** | **6/6** |

O Theta\* sai **idêntico nos quatro raios** — ele não conhece raio. É a
testemunha de que a varredura mexeu só no que devia. Quem se move é a linha que
ele precisa cruzar: os caminhos dele fecham 0,28 m (`aperto`) e 0,37–0,39 m nos
demais, então com a máquina em 0,46 m **nenhum** é seguível.

O Smac cobra por isso, e o preço sobe com o raio:

| caso | desvio do Smac (comprimento ÷ reta) |
|---|---|
| alvo perto e de lado | 1,50× → **2,12×** |
| alvo a 1 m de lado | 1,32× → 1,77× |
| porta, bloco, beco | 1,05–1,54×, praticamente estáveis |

E ele mexe mais o bico em todos os casos com obstáculo (`porta` 144–247° contra
192° fixos do Theta\*; `aperto` 320–465° contra 271°) — é o custo de curvar em
vez de pivotar.

## Decisão

### 1. O planner global é o Smac Hybrid-A\*, em Reeds-Shepp

Ele é o único que entrega caminho seguível em toda a faixa de raio plausível.
A vantagem não depende da zona morta: **o ranking não vira entre os extremos,
ele se acentua** — no pior caso para o robô (0,46 m), o Theta\* vai a zero e o
Smac fica em 6/6.

O `reverse_penalty: 2.0` fica: a ré é permitida no plano, mas nunca de graça.

### 2. O Theta\* sai, e o motivo não é ele ser ruim

Ele é o planner do robô 1 e lá funciona — porque **aquele** chassi (skid-steer
4 rodas) só sabe girar parado, vira ~3% do comando em arco, e um caminho reto
com cantos é exatamente o que ele consegue seguir. Este robô é o oposto: faz
curva boa e não pivota. Herdar o Theta\* seria herdar a solução do problema do
outro robô — é o erro que a decisão 000 nos mandou não cometer.

O sintoma disso está nos dois casos "de lado": o Theta\* desenha reta lateral,
**giro zero**, para um robô que teria de pivotar. Giro zero ali não é virtude,
é o planner supondo uma máquina que não temos.

### 3. A ré passa a nascer do PLANEJAMENTO, não do susto

Isto revisa a decisão 007. Lá a ré era manobra de emergência decidida no
instante, com gatilho geométrico, histerese e orçamento — e foi a histerese que
produziu o ciclo "ré e anda" que o dono reprovou, duas vezes (28-07 no clique,
29-07 na bancada: 5 entradas em ré, 1,85 m de caminho para um alvo a 0,40 m).

Com o Reeds-Shepp, o caminho inteiro **já sabe onde a ré entra** antes de o robô
sair do lugar. Medido: 2 inversões nos casos de lado, em todos os quatro raios —
consistente, não oportunista.

O `goal_navigator` e a ré da 007 ficam aposentados. O código sai numa fatia
própria; os dados e os registros ficam.

### 4. O seguidor é NOSSO, e a movimentação da decisão 005 continua

O Nav2 planeja; quem executa é a nossa camada. Motivo: o defeito que trouxe o
Nav2 para a conversa nasceu na movimentação, e ela é o único lugar do projeto
onde temos física medida — a lei de frenagem de rumo `wz = sinal(e)·min(wz_max,
√(2·a_dec·|e|))` da decisão 005, o piso de linear que defende do BO-3, e os
parâmetros em SI que a trena e a bancada estão preenchendo um a um.

Trocar isso por um controlador do Nav2 seria jogar fora a única parte
caracterizada com número, para ganhar uma caixa com ganhos que teríamos de
sintonizar do zero — contra uma planta cujo simulador ainda tem a boba do BO-4.

Fica para a fatia seguinte, com carrot no plano, como no robô 1.

## O que fica em aberto

- **A zona morta segue sem medir.** Ela não muda esta decisão, mas define o
  `minimum_turning_radius` que o planner vai receber em produção. Enquanto isso
  ele fica em **0,37 m**, o realizado medido no perfil otimista — e o pessimista
  (0,46 m) está a um parâmetro de distância.
- **A bancada não responde se o robô consegue SEGUIR o caminho.** Ela desenha,
  não dirige. Isso só fecha com o seguidor de pé e o robô andando.
- **O costmap vem de mapa estático**, porque o robô simulado ainda não tem
  lidar. Quando o modelo 3D entrar com o Mid-360, a fonte dos obstáculos muda e
  o resto continua igual.
- **O Smac treme em cima do alvo com raio grande** — no `bloco` com 0,46 m são
  4 inversões dentro de uma caixa de 9 cm, depois de 4,86 m limpos. É achado
  medido, não suposição, e cai no colo do seguidor.

## Alternativas descartadas

**Manter o Theta\* e resolver na movimentação.** Foi o que tentamos: a decisão
006 (mirar no alvo, frear pela distância) e a 007 (ré como manobra) são
exatamente isso, e as duas foram reprovadas pelo dono clicando. O limite não é
de ganho, é geométrico — sem pivô existe raio mínimo, e ponto dentro dele não
converge.

**Usar o seguidor do Nav2 junto (RPP, MPPI).** Descartado por ora, não por
mérito: seria trocar a única camada com física medida por ganhos a sintonizar,
antes de termos a zona morta e o `a_dec` do robô. Reavaliável depois que o
seguidor próprio estiver de pé e houver com o que comparar — e aí vira resultado
de artigo, que é o BO-2.

**Smac Lattice em vez de Hybrid.** Precisa de tabela de primitivas de movimento
gerada para o robô, que exige exatamente os números que ainda não temos. Custo
alto agora, benefício que só aparece depois da caracterização.

**Escolher um raio e julgar com ele.** É o que a bancada fazia até 29-07 (0,25 m,
justificado por uma curva solta). Repetiria a forma de erro da bitola: um número
herdado e não medido, com a bancada parecendo boa e o robô piorando.

## Referências

- `docs/dados/2026-07-29-varredura-raio-planner.csv` — os 48 planos
- `docs/dados/2026-07-28-cliques-movimentacao.csv` — a sessão que aposentou a 006
- `docs/dados/2026-07-29-bancada-gazebo-{sim,real}.csv` — de onde saem os raios
- `ros2_packages/robot_planning/README.md` — como rodar e como ler
- Entradas de 28-07 (2ª leva) e 29-07 (2ª, 3ª e 4ª levas) do `docs/DIARIO.md`
