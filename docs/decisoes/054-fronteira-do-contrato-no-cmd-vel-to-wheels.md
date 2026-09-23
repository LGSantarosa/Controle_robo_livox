# 054 — A fronteira do contrato é o `cmd_vel_to_wheels`, por parâmetro

**Data**: 2026-09-22 (PC de dev, robô desligado; nada foi ao robô)
**Status**: aplicada na branch `etapa5-contrato` (`e12b1f4` o nó dual,
`ba32571` o corte atômico da cadeia); **não testada em hardware** — a etapa 5
prova a cadeia de software, não o chão.
**Toca**: `robot_nav/robot_nav/cmd_vel_to_wheels.py` (`use_stamped`),
`robot_nav/robot_nav/dpad_reto.py` (publica `TwistStamped`),
`robot_nav/config/teleop_xbox_robo3.yaml`, `robot_nav/config/twist_mux_robo3.yaml`,
`robot_nav/launch/controle_robo3.launch.py`, testes
`test_cmd_vel_to_wheels_stamped.py` e `test_contrato_robo3.py`.
**Vem de**: `PLANO_ETAPA5_ROBO3.md` §§0–3 (D1–D4), aprovado pelo dono com seis
correções; decisão 049 (o `cmd_vel_to_wheels` é compartilhado com o robô 2).

---

## 1. O que foi decidido

**A cadeia do robô 3 fala `TwistStamped` de ponta a ponta, e a conversão para
a cinemática acontece numa fronteira só: dentro do próprio
`cmd_vel_to_wheels`, escolhida por parâmetro (`use_stamped`), com default
`false`.**

- teleop (`publish_stamped_twist: true`), `dpad_reto` (publica `TwistStamped`,
  `frame_id` do teleop) e mux do robô 3 (`use_stamped: true`);
- o `controle_robo3.launch.py` passa `use_stamped: true` ao nó;
- o `robot.launch.py` (caminho do robô 2, via `launch.sh`) **não é tocado** e
  segue no default cru.

## 2. Alternativas descartadas

| alternativa | por que não |
|---|---|
| **nó adaptador** `TwistStamped → Twist` antes do `cmd_vel_to_wheels` | mais um nó na cadeia de comando, com salto próprio a medir; e a cadeia do robô 3 existe justamente para ser curta e auditável |
| **trocar o tipo padrão** do `cmd_vel_to_wheels` para `TwistStamped` | o nó é COMPARTILHADO com o `robot.launch.py` (decisão 049). Tipo diferente não dá erro em ROS 2: dá nó que não recebe nada. Quebraria o caminho do robô 2 **em silêncio** |
| **manter `Twist` no robô 3** e converter na etapa 6 | a etapa 6 quer trocar o mux do robô 3 pelo da `pilha`, que é `TwistStamped`. A dívida só ficaria maior e mais perto do hardware |
| **incluir o `bin/robot-key` no mux agora** (D3) | faixa humana nova = segundo comandante, risco próprio. A unificação torna isso POSSÍVEL; entrar é escopo de outra etapa. O `robot-key` hoje já tem contrato compatível e segue fora **de propósito** |

## 3. O que ficou provado, e o que não

Evidência: `docs/dados/2026-09-22-etapa5-contrato/` (13 pastas, README e
`SHA256SUMS`), bancada `bin/valida-etapa5` — **sem Gazebo**, por decisão do
plano geral (§4/§8: comando atravessando sem simulador para confundir).

- cadeia `Twist` (caracterização, linha de base) **26/26**;
- cadeia `TwistStamped` (depois do corte) **26/26**, com os **mesmos frames
  byte a byte** nas três condições — a troca de contrato não mudou número
  nenhum que chega na MEGA;
- prioridade do direcional, timeout do mux (0,35 s), homem-morto e TF com
  yaw 0 provados nos dois lados.

🔴 **Não prova hardware nenhum.** Nem o zero do firmware da MEGA, nem a placa,
nem o `frente:=-1.0` no chão.

🔴 **D4 segue em aberto** e está registrado como caracterização, não como
conserto: com o `/joy` sumindo, os frames **cessam na hora e o último fica
não-zero** — quem para o robô é o watchdog do firmware. Watchdog no PC seria
mudança de comportamento com decisão própria, e não entra escondido aqui.

## 4. O gate do §3.4 e a baseline que andou

O §3.4 exige que a etapa 5 **não toque no robô 2**: `git diff <baseline>`
sem nenhum arquivo de `robot_motion/`, `robot_base/`, `robot.launch.py` nem
`robot_nav/config/twist_mux.yaml`. O gate passa **ao pé da letra**, sem
ressalva.

A baseline era `0c70691` (fim da etapa 4) e andou **dois commits**, os dois
em 22-09 e os dois de **infraestrutura de teste** — código que não roda no
robô, não é de etapa nenhuma, e por isso foi para a `main` sozinho em vez de
pegar carona na branch:

| commit | o que é | por que não podia ficar na etapa 5 |
|---|---|---|
| `5e12f0e` | o carregador "sem ROS" do `test_placa_simulada.py` para de escrever `object` por cima de `rclpy.node.Node`, `QoSProfile`, `TwistStamped` e `Odometry` no processo inteiro | mora em `robot_base/test/`, e um arquivo de `robot_base/` no diff reprova o gate. Mas é **dependência** da suíte da etapa 5: sem ele, 31 reprovações e 18 erros — o `test_cmd_vel_to_wheels_stamped.py` é o primeiro teste do repo que sobe no rclpy de verdade depois dele |
| `f23ac4f` | a trava dos argumentos dos launches para de descer em include de pacote de terceiro e passa a rodar com um esqueleto fixo deles | a trava reprovava ou não conforme o PC tivesse `fast_lio` instalado. É defeito do passo 0 da etapa 4, anterior à etapa 5 e já na `main` |

**A baseline andou para NÃO relaxar o gate.** As duas alternativas eram deixar
um arquivo de `robot_base/` dentro do diff da etapa 5, ou abrir ressalva no
gate — e as duas foram recusadas pelo dono, nesta ordem, em 22-09. Mover a
baseline foi o único caminho que mantém as três coisas ao mesmo tempo: a
correção fora da etapa 5, a suíte verde, e o gate valendo literalmente.

O que a baseline **não** fez: nenhum dos dois commits muda comportamento de
robô. Se algum dia um arquivo que o robô 2 executa precisar mudar, o §3.4
manda parar e voltar ao dono — isso continua de pé.

## 5. Suíte

`1237 passed`, código de saída 0, com o overlay do repo carregado e sem o
`ros2_packages/twist_mux`, que é vendorizado (está no `.gitignore`), é teste de
`launch` de terceiro e **pendura** sob o `pytest` da raiz — registrado no
DIARIO desde 21-09.

## 6. Referências

- `docs/PLANO_ETAPA5_ROBO3.md` (§§0–3, §6 as seis correções do dono)
- decisão 049 (frente e ré trocadas no robô 3 — de onde vem o compartilhamento
  do `cmd_vel_to_wheels`)
- `docs/dados/2026-09-22-etapa5-contrato/README.md`
- DIARIO de 2026-09-22
