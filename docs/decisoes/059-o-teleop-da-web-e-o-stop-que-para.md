# 059 — O teleop da web fala o tipo do mux, e o STOP manda zero

**Data**: 2026-10-01 (dev, sem Gazebo e sem robô)
**Status**: implementada e coberta por teste unitário; falta validar no Gazebo
antes de levar ao NUC
**Toca**: `controle_web/controllers/robot_controller.py`, `controle_web/app.py`,
`templates/index.html`, `static/js/client.js`, `static/js/gamepad.js`,
`robot_motion/path_follower.py`
**Vem de**: revisão de código A3 e o item 2 da revisão do Codex sobre a 058;
completa a 057 e a 058

## Problema

1. A web publicava `geometry_msgs/Twist` em `/web_vel`, mas o `twist_mux` do
   robô 2 roda com `use_stamped: true` e assina `TwistStamped`. Com tipos
   diferentes, o DDS não conecta os dois lados: a WASD, o gamepad e o "Space =
   stop" da web morriam sem erro.
2. A velocidade manual era a do robô 1: giro base de 6,0 rad/s (autoridade
   anti-skid) e multiplicador até 4×.
3. O STOP só cancelava goals (058). O seguidor contava `CANCELING` como
   objetivo vivo e só parava no `CANCELED`, uma demora sem teto. Nenhum zero
   em prioridade alta segurava o robô nesse meio-tempo.

## Decisão

1. `/web_vel` passa a ser `TwistStamped`, com stamp atual e `frame_id =
   base_link`. Teste de contrato dos dois lados (web e `twist_mux.yaml`).
2. Velocidade manual: base = o normal do Xbox (0,30 m/s e 1,25 rad/s). Tetos
   **separados**: `linear = min(0,30·m; 0,50)` e `angular = min(1,25·m; 1,25)`,
   com `SPEED_MULT_MAX = 0,5/0,3`. Um multiplicador comum limitado a 1× não
   deixaria o linear chegar a 0,5. O slider, os presets e a escala do gamepad
   acompanham.
3. O STOP:
   - o seguidor deixa de contar `CANCELING` como objetivo vivo
     (`ATIVOS = {1, 2}`). O agregado das duas actions continua valendo;
   - `parada_web()` publica zero em `/web_vel` por 1,0 s a 20 Hz, **mesmo com
     `WEB_TELEOP=off`**, porque é exclusivamente parada e nunca publica valor
     diferente de zero;
   - um lock serializa toda publicação no `/web_vel`. Dentro da janela, nenhum
     comando não-zero da própria web sai: teclado, gamepad e republicador são
     recusados;
   - teclas e eixos de gamepad retidos são limpos. Evento de teclado ou de
     gamepad que chega **dentro** da janela é descartado sem ser guardado: a
     checagem e a atualização do estado ficam sob o mesmo lock. (Na primeira
     versão o evento era recusado na publicação, mas guardado, e o
     republicador voltava a mover quando a janela acabava. Isso valia até
     para pacote enviado antes do STOP e processado depois. A revisão do Codex
     achou; corrigido antes do push.) Depois da janela, só um comando **novo**
     move o robô;
   - o handler do STOP chama a parada **antes** e independente do
     `map_bridge`.

## Alcance — o que este STOP NÃO é

`/web_vel` tem prioridade 50: vence a autonomia (10) e o desencalhe (30), mas
**não** o Xbox (100) nem o teclado (90). Quem segura o LB continua mandando.
Não é E-STOP global. O freio de mão físico continua sendo o Xbox.

## Alternativas descartadas

- **Esperar o `CANCELED`:** a demora depende do `bt_navigator`, sem teto.
- **Zero só com `WEB_TELEOP=on`:** o modo padrão de visualização ficaria sem
  parada.
- **Multiplicador único limitado a 1×:** prenderia o linear em 0,30.

## Limites conhecidos

- O `freeze_capture` ainda registra 1/2/3 como "goal ativo". É só registro.
- O pedido de cancelamento da 058 segue sem conferência de resposta.

## Testes de desenvolvimento

`test_web_vel_contrato.py`, `test_web_velocidade.py`, `test_web_parada.py` e
três casos novos em `test_objetivo_encerrado.py` (`CANCELING` para; `CANCELING`
numa action com a outra viva não para; `ATIVOS = {1, 2}`). A parada é testada
sem espera real (relógio e disparo injetados). Suíte com overlay:
**1295 passed**.
