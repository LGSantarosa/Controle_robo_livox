# 063 — O compensador de rumo sai do robô real com roda omni

**Data**: 2026-10-02 (dev, sem NUC e sem robô)
**Status**: implementada e coberta por teste; falta validar no chão e ler os
logs da primeira corrida
**Toca**: `robot_motion/launch/pilha.launch.py`,
`robot_motion/launch/mapeia.launch.py`, `bin/sobe-robo` e os instrumentos que
observam a saída do mux no robô
**Vem de**: troca da boba por roda omni em 30-09 e constatação do dono de que
o robô agora anda reto

## Problema

A leitura anterior de que o Nav2 já estava sem o compensador estava errada.
`segura_rumo: false` desligava apenas o PI que tenta manter um rumo capturado;
o feedforward continuava ativo em qualquer curva. Além disso, o
`bin/sobe-robo` ainda passava `curv_frente:=-0.8365`, medida em 20-08 com a
roda boba.

O log da corrida de 01-10 confirma o nó real com ff −0,8365 1/m. Como a lei
soma `-curv_frente * |v_real|` ao giro pedido, isso acrescentava giro positivo
sempre que o robô avançava: cerca de +0,25 rad/s a 0,30 m/s. Num trecho do CSV,
por exemplo, entraram −0,8365 rad/s e saíram −0,6673 rad/s. Portanto ele
fortalecia curvas num sentido e enfraquecia no outro, mesmo com o PI desligado.

Esse feedforward compensava o arco da roda boba. Com a omni, a premissa que o
justificava deixou de existir.

## Alternativas

1. Manter o nó no robô com curvatura zero. Viraria uma camada identidade:
   `ganho_wz` real já é 1, o PI estava desligado e o freio linear também.
   Descartada por deixar um processo e um ponto de falha sem função.
2. **Desviar o nó no robô real** — escolhida. O mux passa a publicar direto no
   controlador tanto no Nav2 quanto no mapeamento.
3. Remover também do Gazebo. Descartada: lá o nó ainda converte o giro pelo
   `ganho_wz=0,45` medido na planta simulada. Sua curvatura já é zero desde a
   decisão 062.

## Decisão

- Com `sim:=false`, `twist_mux` publica diretamente em
  `/hoverboard_base_controller/cmd_vel`; `compensador_rumo` não sobe.
- `mapeia.launch.py` usa a mesma cadeia direta.
- `bin/sobe-robo` deixa de injetar a curvatura da boba.
- Com `sim:=true`, o compensador continua entre o mux e a placa simulada,
  somente para o ganho de giro do Gazebo, com curvatura zero.
- A velocidade linear não muda nesta decisão. Primeiro serão lidos os logs do
  robô sem o viés; só depois se decide se o valor que pareceu alto em 01-10
  precisa baixar.

## Consequência esperada e limite

Teoricamente a resposta deve ficar mais simétrica: a omni não recebe mais uma
correção da boba somada a toda curva. Isso não garante que a velocidade
aprovada em 01-10 continuará ideal. A sintonia final daquela sessão foi vista
com o viés ativo, então precisa ser repetida no chão e julgada pelos logs nos
dois sentidos de curva.

## Teste

`test_configs_coerentes.py` trava as duas cadeias mutuamente exclusivas, a
ausência do compensador no mapeamento, a retirada de `CURV` do script de
produção e o tópico direto no gravador da próxima corrida. A lei do
compensador continua testada porque ainda pertence ao simulador. Suíte
completa: **1320 testes passam** com o `install` deste repo carregado.
