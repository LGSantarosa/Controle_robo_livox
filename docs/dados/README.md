# Dados medidos

CSVs de corridas que sustentam afirmações do `ESTADO_PROJETO.md`, do
`docs/DIARIO.md` e das decisões. Ficam aqui porque número citado sem o dado ao
lado vira folclore — e este projeto vira artigo.

| arquivo | o que é |
|---|---|
| `2026-07-28-cliques-movimentacao.csv` | sessão do dono dirigindo o robô no simulador pelo RViz, 20 Hz. Colunas: pose verdadeira, alvo clicado, rumo e velocidade pedidos pela navegação (negativo = manobra de ré), comando da movimentação e o que a placa fingida deixou passar. É a corrida que aposentou a navegação ponto a ponto: 0 amostras de giro parado em 2714, raio de curva mediano 0,37 m, e o ciclo "ré e anda" com período de 2,10 s. |
| `2026-07-28-cliques-movimentacao-alvos.csv` | os 4 objetivos clicados nessa sessão, com o instante de cada um |
