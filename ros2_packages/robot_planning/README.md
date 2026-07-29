# Bancada do planner — julgar o caminho, sem mover o robô

```bash
ros2 launch robot_planning bancada_planner.launch.py
```

Espere a linha **`BANCADA PRONTA`** no terminal (o Smac Hybrid leva ~16 s
montando a tabela de heurística na subida). Então, no RViz:

1. **`2D Pose Estimate`** — clique e arraste onde o robô *estaria*, apontando
   para onde ele estaria virado. É a partida.
2. **`2D Goal Pose`** — clique o destino.

Os dois caminhos aparecem juntos, e a tabela com os números sai no terminal.
**Nada se move**: não há robô, não há simulador, não há sensor. Pode clicar à
vontade, em qualquer ordem, quantas vezes quiser.

## Por que uma bancada só de planner

O defeito que trouxe o Nav2 para a conversa — *"eu mando um goal próximo dele e
ele dá um puta balão"* — **nasceu na movimentação**, não no planejamento: é
`v/wz`, com o teto de giro em 1,0 rad/s e o piso de linear que a zona morta
obriga. Medido em 28-07: raio de curva mediano de 0,37 m e **zero** amostras de
giro parado em 2714.

Julgar o planner junto com quem executa o caminho mistura as culpas. Aqui só o
desenho está em jogo. Quando houver seguidor, ele responde por seguir; o
planner responde por desenhar.

## Os dois planners, e por que estes dois

| | Theta\* | Smac Hybrid-A\* |
|---|---|---|
| caminho | reto com cantos | curvas com raio mínimo |
| supõe que o robô | vira no lugar | curva andando |
| ré | não existe | **não** (Dubins, decisão 009) |
| é o do | robô 1 | candidato para este chassi |

O robô 1 usa Theta\* porque **aquele** chassi (skid-steer 4 rodas) só sabe girar
parado — vira ~3% do comando em arco. Este robô é o oposto: faz curva boa. Daí
o Hybrid ser candidato de verdade aqui, e não só curiosidade.

O `motion_model_for_search` está em **`DUBIN`**: o plano nunca dá ré
(decisão 009). Foi `REEDS_SHEPP` por algumas horas em 29-07 e caiu por
experiência de operação — ré planejada faz o robô ficar tentando entrar e sair
dos trechos de ré, com as cúspides mudando de lugar a cada replanejamento. Quem
cobre os casos que o Dubins não resolve é a **ré por gatilho**, no seguidor.

Para medir de novo o custo dessa escolha:
`python3 tools/planner/varredura_raio.py --modelo REEDS_SHEPP`.

## Lendo a tabela

```
planner      compr.  desvio   giro   raio min  inv  curt  pts   tempo
theta         1.00m   1.00x      0°     retom    0     0   21      0ms
hibrido       1.33m   1.33x    104°     0.30m    2     2   21     12ms
```

- **compr.** quanto o robô andaria de fato
- **desvio** comprimento ÷ linha reta. 1,00 = reta perfeita; 1,33 = anda 33% a mais
- **giro** soma de todas as viradas do caminho, em graus
- **raio min** curva mais fechada do caminho, medida DENTRO de cada trecho
  entre cúspides. Vem marcado `<-- APERTADO` quando é menor que o raio que a
  máquina fecha (`raio_da_maquina`, hoje 0,37 m — o realizado medido em 29-07
  com zona morta otimista; com a pessimista são 0,46 m) — caminho bonito que o
  robô não consegue seguir
- **inv** quantas vezes o caminho inverte o sentido: é a ré aparecendo
- **curt** trechos entre cúspides curtos demais para medir curvatura. Não
  entram no raio nem no giro, e aparecem aqui para não sumir calados. Vários
  deles = o planner está TREMENDO em cima do alvo (visto no `bloco` com raio
  0,46: 4 inversões dentro de uma caixa de 9 cm)
- **pts** pontos do caminho, útil para saber se veio suavizado ou cru

O exemplo acima é o caso "alvo a 1 m de lado": o Theta\* manda ir reto de lado
(pressupõe pivô), o Hybrid desenha a curva que o robô de fato conseguiria
fazer, 33% mais longa. Qual dos dois é o certo aqui é exatamente a pergunta que
esta bancada existe para você responder.

## A pista

Gerada por `tools/mundo/gera_pista.py`, que escreve **o mapa do Nav2 e o mundo
do Gazebo da mesma planta** — mantidos à mão os dois divergem, e mapa que não
bate com o mundo produz plano bonito que o robô não consegue seguir.

Sala de 12 × 8 m, e cada obstáculo responde uma pergunta:

- **porta de 0,90 m** — ele passa ou contorna? O caminho sai centrado ou raspa a quina?
- **bloco solto** — contorna pelo lado curto ou dá a volta grande?
- **aperto de 0,80 m** — com a inflação escolhida, aceita o vão ou recusa?
- **beco sem saída** — entra e volta, ou já sai pelo caminho bom?

O robô tem 0,50 m de largura; `robot_radius` está em 0,36 m e a inflação em
0,45 m. **A inflação é a alavanca principal** do que ele aceita: aumentar faz
recusar o aperto e dar a volta — o que pode ser a decisão certa.

## O que esta bancada NÃO responde

- **Se o robô consegue seguir o caminho.** Ela desenha, não dirige.
- **Como fica com obstáculo que o sensor vê na hora.** Aqui o costmap vem de um
  mapa estático, porque o robô simulado ainda não tem lidar. Quando o modelo 3D
  chegar com o Mid-360, a fonte dos obstáculos muda e o resto continua igual.
- **Localização.** A TF `map→odom→base_link` da launch é fixa e provisória, só
  para o costmap parar de exigir pose de robô. O caminho é calculado entre os
  dois pontos clicados (`use_start`), então ela não influencia o resultado.

## Detalhe de implementação que custou tempo

O `planner_server` atende **um objetivo de cada vez**. Pedir os dois caminhos
em paralelo faz o segundo preemptar o primeiro, e o preemptado volta com
caminho **vazio e código de sucesso** — mentindo. Isso apareceu como "Theta\*
sem caminho" no primeiro clique de cada corrida (frio ele demora mais e era
atropelado; quente, escapava). A bancada pede um de cada vez.

## Varredura de raio: a conclusão depende do raio mínimo?

```bash
python3 tools/planner/varredura_raio.py
```

Roda os mesmos casos contra os dois planners, uma vez por raio candidato, sem
RViz e sem cliques. Existe porque o `minimum_turning_radius` não é um parâmetro
qualquer desta comparação — ele **é** o argumento dela: o Smac está na disputa
justamente por respeitar raio de curva, e informá-lo otimista dá a vitória ao
Smac num robô que não existe.

**Resultado de 29-07** (`docs/dados/2026-07-29-varredura-raio-planner.csv`,
48 planos): o ranking **não vira** entre 0,25 m e 0,46 m — ele se acentua. O
Theta\* desenha sempre o mesmo caminho (não conhece raio), então quem se move é
a linha que ele precisa cruzar: com 0,25 m ele entrega 4 dos 6 casos seguíveis,
com 0,46 m entrega **zero**. O Smac fica em 5–6 de 6 em toda a faixa, cobrando
por isso um caminho mais longo que cresce com o raio (no alvo perto e de lado,
de 1,50× para 2,12× a linha reta).

Ou seja: **a decisão 008 pode ser tomada sem esperar a zona morta**. A medida
que falta muda o tamanho da vantagem, não quem vence.

⚠️ O Theta\* falha nos dois casos "de lado" em QUALQUER raio, e por outro
motivo: ele desenha uma reta lateral, que só é seguível por um robô que pivota
— e este não pivota com a zona morta pessimista.
