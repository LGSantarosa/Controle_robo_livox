# 057 — O fim do objetivo para o seguidor e apaga o plano

**Data**: 2026-10-01 (dev, sem Gazebo e sem robô)
**Status**: implementada e coberta por teste unitário; falta validar no Gazebo
antes de levar ao NUC
**Toca**: `path_follower.py`, `test_objetivo_encerrado.py`
**Vem de**: revisão de código A2; também elimina o mecanismo de A6

## Problema

O `path_follower` sabia, pelos status das actions do Nav2, se havia objetivo
vivo, mas usava essa informação apenas para autorizar a ré. Cancelado,
abortado ou concluído o objetivo, o plano ficava retido e ainda podia dirigir
o robô até vencer `timeout_plano` (7 s).

Parar só a cadeia normal também seria incompleto: durante uma ré ou um pivô de
escape, o comando sai por `/unstuck_vel`, com prioridade 30 e depois do
`collision_monitor`.

## Decisão

`cb_status` observa o estado agregado de `NavigateToPose` e
`NavigateThroughPoses`. Somente na transição **havia objetivo vivo → não há
objetivo vivo**, o seguidor:

1. publica zero em `/unstuck_vel`;
2. publica parada na cadeia normal e volta para `ocioso`, inclusive se estava
   em `re` ou `pivo_escape`;
3. esvazia `self.plano`.

O agregado evita parar uma action enquanto a outra ainda estiver ativa.

Não foi criado parâmetro novo. Na bancada sem action o estado nunca passa de
verdadeiro para falso, portanto um `/plan` cru continua sendo seguido como
antes. Checar `tem_objetivo()` em todo ciclo foi descartado porque quebraria
esse uso.

Efeito colateral desejável: sem o plano antigo, reenviar o mesmo ponto depois
de mover o robô à mão deixa de cair na comparação por proximidade de destino;
o plano novo entra como novo. Isso elimina o mecanismo descrito em A6.

## Limite conhecido e validação pendente

Status da action e plano vêm de tópicos diferentes. É possível que o status de
encerramento do objetivo anterior seja processado depois do plano do próximo e
o apague. A consequência esperada é ficar parado até o próximo replanejamento,
não movimento órfão. Antes do NUC, validar no Gazebo:

- cancelar no meio do caminho;
- cancelar durante uma ré e durante um pivô de escape;
- clicar outro ponto com o robô andando;
- trocar de waypoint numa rota;
- depois da correção web, acionar o STOP durante a navegação.

## Testes de desenvolvimento

`test_objetivo_encerrado.py` cobre seguimento, ré, pivô de escape, as duas
actions simultâneas e o caso de bancada sem action. Junto dos testes próximos
do seguidor: **29 passed**. Suíte completa com o overlay local carregado:
**1267 passed**.
