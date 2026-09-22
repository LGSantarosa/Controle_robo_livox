# Plano da etapa 5 — um contrato de comando só (`TwistStamped`), sem Gazebo

> Etapa 5 do `docs/PLANO_NAV2_ROBO3.md` (§4 e §8): "unificar o contrato de
> mensagens e testar a cadeia **sem Gazebo** — comando atravessa de ponta a
> ponta, sem simulador para confundir". A etapa 4 travou os dois contratos
> **como estão** e deixou esta fronteira explícita (`PLANO_ETAPA4_ROBO3.md`
> §9). Estado: **plano aprovado pelo dono em 22-09, com seis correções
> (incorporadas abaixo e listadas no §6).** Começa pela caracterização.

---

## 0. O que a leitura do código mostrou

| cadeia | hoje | onde |
|---|---|---|
| robô 2 (`pilha`) | **`TwistStamped` de ponta a ponta**: `joy_vel`/`key_vel`/`web_vel`/`unstuck_vel`/`auto_vel` → `twist_mux` (`use_stamped: true`) → `/compensador_rumo/cmd_vel` → `compensador_rumo` → `/hoverboard_base_controller/cmd_vel` (diff_drive) | `robot_motion/config/twist_mux.yaml:42`, `teleop_xbox.yaml:74`, `compensador_rumo.py:237` |
| robô 3 (`controle_robo3`) | **`Twist` cru**: teleop (`publish_stamped_twist: false`) → `joy_vel`; `dpad_reto` → `dpad_vel` → `twist_mux` (`use_stamped: false`, config `twist_mux_robo3.yaml`) → `/cmd_vel` → `cmd_vel_to_wheels` → `wheel_vel_setpoints` → `mega_bridge` | `teleop_xbox_robo3.yaml:39`, `twist_mux_robo3.yaml:15`, `dpad_reto.py:18`, `cmd_vel_to_wheels.py:36,66` |

Três achados que mudam o desenho:

1. **O `cmd_vel_to_wheels` não é só do robô 3.** O `robot_nav/launch/robot.launch.py`
   (subido pelo `launch.sh:402`) usa o mesmo nó, em `Twist`, com o
   `robot_nav/config/twist_mux.yaml` (`use_stamped: false`). O DIARIO (l.1331)
   e a decisão 049 já tratam esse caminho como compartilhado com o robô 2.
   **Mudar o tipo padrão dele quebraria esse caminho em silêncio** (tipo
   diferente = nó que não recebe nada, sem erro).
2. **O `bin/robot-key` publica `TwistStamped` e hoje NÃO entra no mux do robô 3**
   (`twist_mux_robo3.yaml:9`). Unificar o contrato é o que torna isso possível —
   mas incluir o teclado é escopo novo (D3).
3. **Ninguém na cadeia do robô 3 produz zero quando o comando some.** O
   `twist_mux` não repete nem publica zero: só encaminha o que recebe da faixa
   ativa. O `cmd_vel_to_wheels` não guarda estado: publica um setpoint por
   mensagem recebida e, sem mensagem, simplesmente deixa de publicar. O
   `mega_bridge` só manda frame quando chega setpoint. Então, se o `/joy`
   desaparece com o LB apertado, **os frames cessam na hora e o último
   `FT_SET_SPEED` fica não-zero**; só o **watchdog do firmware da MEGA**
   produz a parada física (sem comando → zera, observado em 15-09). O
   homem-morto do LB funciona porque o teleop publica **um zero ao soltar**.
   O timeout do mux é outra coisa: ele decide quando uma faixa de prioridade
   maior deixa de bloquear a menor. A etapa 5 testa os três separados e diz o
   que o teste **não** alcança (o firmware).

## 1. Escopo

**Entra:**
- a cadeia do robô 3 passa a falar **`TwistStamped` até a fronteira do atuador**:
  teleop (`publish_stamped_twist: true`), `dpad_reto` (publica `TwistStamped`),
  mux do robô 3 (`use_stamped: true`);
- a fronteira escolhida em D1, **testada**;
- **um mux só** no bringup do robô 3, com o mesmo contrato do mux da `pilha`
  (tipo, convenção de nome do nó), para a etapa 6 poder trocar um pelo outro;
- os testes do §4 do plano geral: prioridade, timeout, sinal de `v` e `wz` com
  o `linear_sign`, nenhum nó duplicado.

**Não entra (fronteira explícita):**
- a `pilha` com `robo:=3` (etapa 6); nenhum costmap nem Nav2 do robô 3;
- `compensador_rumo` no robô 3: não há curvatura medida (etapa 8). Até lá a
  cadeia do robô 3 termina no `cmd_vel_to_wheels`, sem compensador;
- nada da cadeia do robô 2: nenhum arquivo de `robot_motion/` nem do
  `robot.launch.py` muda (§3.4 trava isso);
- os dois achados laterais da etapa 4 ("caindo no js0" e o teto do
  `corrida_nav.py`);
- watchdog novo no `cmd_vel_to_wheels` ou no `mega_bridge` (achado 3): se o
  teste mostrar que falta, vira decisão própria, não entra escondido aqui.

## 2. Decisões que dependem do dono

| # | decisão | opções | recomendação |
|---|---|---|---|
| **D1** | a fronteira `TwistStamped → WheelSpeeds` | **(i)** nó adaptador `TwistStamped → Twist` antes do `cmd_vel_to_wheels`; **(ii)** o `cmd_vel_to_wheels` aceita `TwistStamped` por parâmetro | **(ii)**, com parâmetro `use_stamped` (mesmo nome do `twist_mux`), **default `false`** — o `robot.launch.py` segue igual sem tocar nele; o `controle_robo3` passa `true`. Um nó a menos na cadeia, sem salto a medir. O custo: o nó tem dois caminhos de assinatura, e os dois precisam de teste |
| **D2** | o `frame_id` do `TwistStamped` que o `dpad_reto` publica | `base_link`; vazio; o mesmo do teleop | o **mesmo do teleop** (o parâmetro `frame` do `teleop_twist_joy`, lido e registrado no teste): as duas faixas humanas iguais; ninguém na cadeia do robô 3 usa o `frame_id` hoje |
| **D3** | o `bin/robot-key` (`key_vel`) entra no mux do robô 3? | sim, prioridade 90 como no robô 2; não, fica para depois | **não nesta etapa**: é uma faixa humana nova no robô 3, com risco próprio (segundo comandante). A unificação só a torna **possível**; registrar |
| **D4** | perda do controle com o LB apertado: o último frame fica não-zero até o firmware zerar | aceitar e registrar; watchdog no `cmd_vel_to_wheels` | **caracterizar antes de decidir**. Um watchdog seria mudança de comportamento com decisão própria |

## 3. Testes — vermelho antes, e sem Gazebo

### 3.1 Estáticos (sem ROS de pé)

- os tipos do contrato: `teleop_xbox_robo3.yaml` com `publish_stamped_twist:
  true`; `twist_mux_robo3.yaml` com `use_stamped: true`; o `controle_robo3`
  passa `use_stamped: true` ao `cmd_vel_to_wheels`; o `dpad_reto` publica
  `TwistStamped` (AST do fonte, como os testes do `path_follower`);
- **coerência dos dois lados de cada tópico**: para cada faixa do mux do robô
  3, quem publica e o mux concordam no tipo (o erro que hoje só aparece como
  "robô parado em silêncio"). Travado para o robô 2 também, sem mudar nada nele;
- `test_controle_robo3_launch.py` estendido: os seis nós + RSP, uma vez cada;
  cinco argumentos intactos.

### 3.2 Unitários (rclpy, sem launch)

- `cmd_vel_to_wheels` com `use_stamped: true` e `false`: a mesma conta nos dois,
  inclusive `linear_sign = −1` e o corte em `max_output`;
- tipo trocado **não** recebe — com **controle positivo** e **barreira de
  descoberta**: primeiro o publicador do tipo certo é descoberto e entrega
  (prova de que a descoberta aconteceu), só então o do tipo errado publica e
  nada chega. E o tipo da assinatura é conferido **no grafo** (`ros2 topic
  info -v` / `get_subscriptions_info_by_topic`), não só pelo efeito;
- `dpad_reto`: `TwistStamped` com stamp e o `frame_id` de D2; o zero ao soltar.

### 3.3 Bancada real, sem Gazebo (a do passo 7, estendida)

O `bin/valida-etapa4` é **evidência congelada** da etapa 4 e não muda. A
etapa 5 tem o seu: `bin/valida-etapa5` e `tools/valida_etapa5/`, com a
infraestrutura de segurança **copiada** (marca, sinal só com identidade,
limpeza em `EXIT`, consulta ao grafo com erro reprova, pré-condição do udev) —
copiar é aceitável; refatorar a da etapa 4 mudaria uma prova já encerrada.
A mesma bancada roda **duas vezes**: na cadeia `Twist` de hoje
(caracterização) e na `TwistStamped` (depois do corte), e as duas são
comparadas.

| prova | como | esperado |
|---|---|---|
| regressão byte a byte | as duas rodadas do passo 7 (`reproducao_14_09`, `padrao_hoje`) | **os mesmos frames** do passo 7: −120 / 97 / 0 e −120 / −256 / 0 |
| `linear_sign` **isolado** | um par idêntico em tudo, variando **só** `frente:=1.0` / `frente:=-1.0` (as rodadas históricas mudam também sinal, bitola e eixo de giro, e não isolam nada) | o `speed` inverte e o `steer` fica igual |
| prioridade | LB + direcional cima **junto** com o analógico de giro | o frame é o do direcional (reta pura, `steer 0`), não o do giro |
| timeout real do mux | solta o direcional mantendo LB + analógico | depois de **0,3 s** (o timeout da faixa `direcional`) a faixa analógica volta a passar |
| homem-morto | solta o LB | um frame zero |
| perda do controle | o `/joy` **para de chegar** com o LB apertado | frames **cessam na hora**; o último frame é **não-zero** (caracterização do D4). **Não** prova o zero do firmware |
| nó duplicado | lista de nós + `uniq -d` | um mux, um `cmd_vel_to_wheels` |

### 3.4 O robô 2 não muda

- **gate de fechamento** (não teste permanente, porque dependeria do Git):
  `git diff 0c70691 --stat` sem nenhum arquivo de `robot_motion/`,
  `robot_base/`, `robot.launch.py` nem `robot_nav/config/twist_mux.yaml`;
- o `cmd_vel_to_wheels` com o default (`false`) **ainda assina `Twist`** (no
  grafo) e produz **exatamente os mesmos valores de `WheelSpeeds`** para a
  mesma entrada — o fonte compartilhado muda, então "o mesmo nó" não seria
  verdade; a garantia é de comportamento;
- se algum arquivo do robô 2 precisar mudar, o passo para e volta ao dono — e
  aí o cenário `robo2-gazebo` roda de novo.

## 4. Ordem — uma mudança por commit, vermelho antes

Branch própria `etapa5-contrato` a partir da `main` (`0c70691`); a `main` só
recebe a etapa inteira.

| # | mudança | vermelho antes | prova |
|---|---|---|---|
| 1 | branch + commit deste plano | — | — |
| 2 | coletor novo (`bin/valida-etapa5`, `tools/valida_etapa5/`) e **caracterização da cadeia `Twist` de hoje** | — (caracterização: registra o comportamento atual, inclusive a perda do controle com último frame não-zero) | pasta de evidência |
| 3 | `cmd_vel_to_wheels` dual (`use_stamped`, default `false`) | unitário com `true` (não existe) | unitários com controle positivo e barreira de descoberta; default ainda `Twist` com os mesmos `WheelSpeeds` |
| 4 | vermelho local e **corte atômico**: `dpad_reto` + teleop + mux + launch **num commit só** — metade trocada é robô parado em silêncio (o `dpad_reto` sozinho publicaria `TwistStamped` para um mux que espera `Twist`) | estáticos de coerência de tipo por tópico | suíte; launch com contexto |
| 5 | bancada na cadeia `TwistStamped` e **comparação com a caracterização** | — | as provas do §3.3, iguais à caracterização onde o comportamento não deveria mudar |
| 6 | decisão 054 (a fronteira), DIARIO, ESTADO; gate do §3.4 | — | — |

## 5. O que isto NÃO prova

Nada de hardware: nem o zero do firmware da MEGA no timeout, nem a placa, nem
o `frente:=-1.0` no chão. A etapa 5 prova que o comando atravessa a cadeia de
software com o contrato novo e **os mesmos números**; não prova que o robô
anda certo (etapa 8) nem que a `pilha` sobe o robô 3 (etapa 6).

## 6. Correções da revisão do dono (22-09), já incorporadas

1. o `dpad_reto` entra no corte atômico (sozinho, deixaria um commit quebrado);
2. o timeout descrito certo: o mux não repete nem zera, o `cmd_vel_to_wheels`
   não guarda estado — perda do controle = silêncio imediato com último frame
   não-zero; o timeout do mux é a volta da faixa de prioridade menor;
3. par `frente:=1.0` / `-1.0` idêntico no resto, para isolar o `linear_sign`;
4. `bin/valida-etapa5` novo; o da etapa 4 é evidência congelada;
5. "tipo errado não recebe" com controle positivo, barreira de descoberta e o
   tipo conferido no grafo;
6. o default do `cmd_vel_to_wheels` garantido por comportamento, não por
   "mesmo nó"; o diff contra `0c70691` é gate de fechamento.
