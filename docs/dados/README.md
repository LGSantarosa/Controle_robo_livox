# Dados medidos

CSVs de corridas que sustentam afirmações do `ESTADO_PROJETO.md`, do
`docs/DIARIO.md` e das decisões. Ficam aqui porque número citado sem o dado ao
lado vira folclore — e este projeto vira artigo.

| arquivo | o que é |
|---|---|
| `2026-07-28-cliques-movimentacao.csv` | sessão do dono dirigindo o robô no simulador pelo RViz, 20 Hz. Colunas: pose verdadeira, alvo clicado, rumo e velocidade pedidos pela navegação (negativo = manobra de ré), comando da movimentação e o que a placa fingida deixou passar. É a corrida que aposentou a navegação ponto a ponto: 0 amostras de giro parado em 2714, raio de curva mediano 0,37 m, e o ciclo "ré e anda" com período de 2,10 s. |
| `2026-07-28-cliques-movimentacao-alvos.csv` | os 4 objetivos clicados nessa sessão, com o instante de cada um |
| `2026-07-29-bancada-gazebo-sim.csv` | corrida de `tools/banco/corrida_gazebo.py --perfil sim` (zona morta 0,10, o chute otimista), já com a bitola medida 0,270. Fase A = aferição em malha aberta com a placa fingida contornada; fase B = 5 alvos em malha fechada. Colunas: pose verdadeira, velocidade e giro derivados da POSE, o que a movimentação PEDIU (`*_bruto`) e o que a placa DEIXOU PASSAR (`*_placa`), mais alvo e distância na fase B. |
| `2026-07-29-bancada-gazebo-real.csv` | a mesma corrida com `--perfil real` (zona morta 0,15, o chute pessimista, posto na planta E no controlador). O par com o de cima é o que mostra o pivô existindo só no perfil otimista e o ciclo "ré e anda" só no pessimista. |

Os dois de 29-07 sustentam, juntos: giro entregue a 79–86% do comandado em malha
aberta (reta a 100,4%), 10 de 10 alvos alcançados, zero rodas pedidas dentro da
banda morta, e o raio de curva realizado (p5) de 0,370 m no perfil sim e
0,463 m no real — o número que reabriu o `minimum_turning_radius` da bancada do
planner.

| arquivo | o que é |
|---|---|
| `2026-07-29-varredura-raio-planner.csv` | 48 planos de `tools/planner/varredura_raio.py`: 6 casos × 4 raios mínimos (0,25 · 0,34 · 0,37 · 0,46) × Theta\* e Smac Hybrid-A\*, na pista de `tools/mundo/gera_pista.py`. Sem robô, sem simulador, sem sensor. Colunas: comprimento, desvio sobre a linha reta, giro, raio mínimo do caminho, inversões (a ré do Reeds-Shepp), trechos curtos demais para medir e tempo do planner. O Theta\* sai IDÊNTICO nos quatro raios — é a testemunha de que a varredura mexeu só no que devia. |

## Bancada no robô real — 2026-07-31

Primeiros números medidos no robô 2. Todos com `--fonte roda` (o `/Odometry` do
LIO estava inutilizável: yaw invertido e ruído de 0,033 rad/s com o robô
parado). Todo CSV traz as **duas** odometrias no mesmo instante — é a coluna
`yaw_roda` contra a `yaw` que denuncia o LIO.

| arquivo | o que é |
|---|---|
| `2026-07-31-zona-morta-giro.csv` | **INVÁLIDO, guardado de propósito.** Gatilho no LIO. Ele fabricou 143,8° de excursão e 2,9 rad/s num pivô que os encoders mediram como 8,7°, e marcou 0,75 rad/s durante a pausa com comando ZERO. É a evidência do defeito do LIO, no mesmo arquivo que a odometria de roda correta. Lido pelo LIO dá 0,068 rad/s com um dente destoando 79%. |
| `2026-07-31-zona-morta-giro-roda.csv` | **INVÁLIDO, guardado de propósito.** Gatilho na roda, mas comparando `wz` cru contra o limiar da reta — ~7x sensível demais. Devolveu 0,032 rad/s com desvio de 0,003 e quantização 15x abaixo do limiar: parecia medida boa, e o dono viu o robô **parado**. Era rastejo de eixo (2° por dente, folga mecânica); a borda da roda nunca cruzou 0,006 m/s. O CSV que mostra que dispersão baixa não é validade. |
| `2026-07-31-zona-morta-giro-borda.csv` | **VÁLIDO.** Gatilho em velocidade de borda de roda (`wz·L/2`). Giro líquido de 43° a 47° por dente, sentido alternando, confirmado a olho pelo dono. **Zona morta de giro = 0,131 rad/s** (queda 0,106). |
| `2026-07-31-zona-morta-linear-roda.csv` | **VÁLIDO.** Avanço de 4,7 a 6,6 cm por dente, 2 à frente e 2 de ré, confirmado a olho. **Zona morta linear = 0,021 m/s** (queda 0,014). |

> ⚠️ **RETRATADO em 31-07 (4ª leva).** Os rótulos VÁLIDO/INVÁLIDO acima estão
> errados. As duas corridas ditas válidas foram medidas por `--fonte roda`, e
> `/hoverboard_base_controller/odom` está em `open_loop`: devolve o comando
> integrado, não medida. Os números 0,021 m/s e 0,131 rad/s eram o próprio
> limiar de detecção ecoado.
>
> Relidos pelo LIO, por deslocamento de pose (o LIO é bom: 0,05° de deriva em
> 20 s parado), os quatro CSV dão:
>
> | corrida | giro por dente | zona morta |
> |---|---|---|
> | `-giro.csv` | 117° a 137° | 0,091 rad/s |
> | `-giro-roda.csv` | **0,03° a 0,21° — não moveu** | — |
> | `-giro-borda.csv` | 266° a 608° | 0,088 rad/s |
> | `-linear-roda.csv` | 0,66 a 0,78 m | 0,023 m/s |
>
> `-giro-roda.csv` é a corrida em que o dono disse "no giro ele não se mexeu": o
> LIO concorda com ele. Ver DIARIO 07-31 4ª leva.
