# 058 — O STOP da web cancela todo objetivo, inclusive o que ainda não foi aceito

**Data**: 2026-10-01 (dev, sem Gazebo e sem robô)
**Status**: implementada e coberta por teste unitário; falta validar no Gazebo
antes de levar ao NUC
**Toca**: `controle_web/map_service.py`, `controle_web/test_map_service_stop.py`
**Vem de**: revisão de código A1 e A7; completa a 057

## Problema

- **A1**: o STOP (`stop_waypoints`) cancelava só o handle do executor de rotas.
  O clique-para-ir entra por `/goal_pose`, o `bt_navigator` cria o goal por
  dentro e a web não guarda handle dele. STOP com objetivo de clique não
  cancelava nada.
- **A7**: o handle de rota só existe depois do aceite. STOP entre o envio e o
  aceite (~0,5 s a cada waypoint) encontrava `None`; o goal era aceito em
  seguida e executado.

## Decisão

1. O STOP chama o serviço `navigate_to_pose/_action/cancel_goal` com `goal_id`
   zerado e `stamp` = agora: pela semântica do `action_msgs/CancelGoal`, isso
   cancela todo goal aceito até esse instante, venha de onde vier.
2. Cada stop incrementa uma geração (`_wp_gen`). O goal enviado guarda a
   geração do envio; se ele for aceito numa geração vencida, é cancelado no
   próprio aceite e não vira o handle corrente.
3. `start_waypoints` para a rota anterior **sem** o cancelamento geral. Senão o
   pedido assíncrono poderia chegar ao servidor depois do aceite do primeiro
   goal da rota nova e cancelá-lo.

Com a 057, o cancelamento passa a parar o robô na hora: o seguidor zera a cadeia
normal e `/unstuck_vel` e apaga o plano.

## Alternativas descartadas

- **Guardar o handle do clique** trocando `/goal_pose` pela action: cobriria só
  objetivos criados pela web, não os do RViz nem de outro cliente.
- **Stamp zero** (cancelar todos, sem limite de tempo): um STOP seguido de rota
  nova poderia cancelar o goal novo, se o pedido chegasse atrasado.

## Limite conhecido

Um clique enviado milissegundos antes do STOP e aceito depois do instante do
cancelamento escapa do item 1. A janela é a latência de aceite do
`bt_navigator`. O item 2 não cobre esse caso, porque o clique não passa pelo
executor de rotas.

## Testes de desenvolvimento

`test_map_service_stop.py`: o STOP cancela todos (goal_id zerado, stamp
atual); o reinício de rota não cancela tudo; o goal aceito depois do STOP é
cancelado no aceite; o goal aceito sem STOP segue normal. Suíte completa com
o overlay carregado: **1271 passed**.
