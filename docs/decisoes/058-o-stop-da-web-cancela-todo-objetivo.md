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
2. Cada stop liga `_wp_stop` e incrementa uma geração (`_wp_gen`) juntos,
   sob o `_wp_lock`. O `_send` do runner confere o STOP e lê a geração no mesmo
   trecho travado e entrega essa geração ao envio. Um goal aceito numa geração
   vencida é cancelado no próprio aceite e não vira o handle corrente.
   (Na primeira versão a geração era lida fora do lock, depois da conferência:
   um STOP entre as duas dava ao goal a geração nova e ele escapava. A
   revisão do Codex reproduziu essa ordem; corrigido no mesmo dia.)
3. `start_waypoints` para a rota anterior **sem** o cancelamento geral. Senão o
   pedido assíncrono poderia chegar ao servidor depois do aceite do primeiro
   goal da rota nova e cancelá-lo.

Com a 057, o cancelamento faz o seguidor zerar a cadeia normal e
`/unstuck_vel` e apagar o plano. **Isso não é "na hora":** o seguidor conta
`CANCELING` (3) como objetivo vivo e só para quando chega o estado terminal
`CANCELED`. A demora é a do `bt_navigator` processar o cancelamento, sem teto
explícito. Fecha junto da A3: o zero da web em prioridade alta é hoje `Twist`
num mux `TwistStamped` e não chega.

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

Outro limite: o pedido de cancelamento é só enfileirado (`call_async`). Se
o serviço estiver indisponível, há aviso no log, mas a web segue respondendo
`ok`. Também fica para depois.

## Testes de desenvolvimento

`test_map_service_stop.py`: o STOP cancela todos (goal_id zerado, stamp
atual); o reinício de rota não cancela tudo; o goal aceito depois do STOP é
cancelado no aceite; o goal aceito sem STOP segue normal; STOP durante a
espera do runner não envia goal; STOP logo depois da conferência cancela no
aceite (a ordem que a revisão reproduziu). Suíte completa com o overlay
carregado: **1273 passed**.
