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
