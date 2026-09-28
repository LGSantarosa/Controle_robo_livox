# 057 — Saídas não limpas no teardown são dívida separada, não reprovação da etapa 6

**Data**: 2026-09-24 (PC de dev, robô DESLIGADO)
**Status**: 🟡 **ANOTADA** — a evidência registra o fato; o conserto fica para
depois do passo 8, e o critério de pronto está no §5.
**Toca**: `bin/valida-etapa6`, `tools/valida_etapa6/test_valida_etapa6.py`
(só o registro da evidência).
**Não toca**: `robot_base/`, `robot_nav/`, os nós de `robot_motion/` que saem
com código 1, nem a versão do Nav2 instalada — é exatamente o que esta decisão
recusa a mexer agora.
**Vem de**: a auditoria da quarta corrida da etapa 6 (`5cee221`,
`~/etapa6/20260924_114704`), ampliada pela quinta (`8515a25`,
`~/etapa6/20260924_132510`).

---

## 1. O que apareceu

O `resultado.csv:17` da quarta corrida declarava **0** linhas
`ERROR/FATAL/died`, enquanto o `launch.log:624` **assinado pelo mesmo
`SHA256SUMS`** trazia cinco processos encerrados com código 1. A contagem era
feita **antes** do `limpa`; essas linhas nascem **durante** o teardown. Nada
estava corrompido — mas a pasta afirmava algo falso sobre um arquivo que ela
própria assina, e num PIBIT a pasta é a prova.

A quinta corrida, já com dois exames e três números (`0 / 7 / 7`), mostrou que
o conjunto é maior do que cinco.

## 2. Os seis nós, e o que exatamente se viu

| nó | pacote | saída | constância |
|---|---|---|---|
| `heading_controller` | `robot_motion` | código 1 | 4ª e 5ª corridas |
| `compensador_rumo` | `robot_motion` | código 1 | 4ª e 5ª corridas |
| `path_follower` | `robot_motion` | código 1 | 4ª e 5ª corridas |
| `placa_simulada` | `robot_base` | código 1 | 4ª e 5ª corridas |
| `freeze_capture` | `robot_nav` | código 1 | 4ª e 5ª corridas |
| `collision_monitor` | Nav2 (binário de prateleira) | **código −11 (SIGSEGV)** | **intermitente** — 5ª corrida; na 4ª ele saiu `finished cleanly [pid 230655]` |

Mais **uma linha de erro de TF**, na quinta corrida, `launch.log:463`:

> `[collision_monitor-15] [ERROR] [getTransform]: Failed to get
> "livox_frame"->"base_link" frame transform: Lookup would require
> extrapolation into the future. Requested time 18.501000 but the latest data
> is at time 18.500000`

Um milissegundo. Ela **não existia** na quarta corrida (`grep -c extrapolation`
= 0 lá).

⚠️ **Não se está afirmando que esse erro de TF causou o segfault.** São dois
fatos observados na mesma janela, e é tudo o que se tem. Há discussão upstream
sobre corridas durante o shutdown capazes de derrubar nós do Nav2, mas nada que
prove ser este o defeito, e nenhum backtrace foi colhido (ver §6).

## 3. Por que isso é TEARDOWN, e não erro de execução

A sequência do `launch.log` da quinta corrida (linhas exatas):

| linha | evento |
|---|---|
| 382 | `[INFO] [ros2-20]: process has finished cleanly` — o **gravador** fecha limpo |
| 383–408 | `signal_handler(SIGINT/SIGTERM)` — o sinal geral chega aos nós |
| 459 | `[collision_monitor]: Cleaning up` |
| 463 | o erro de TF acima |
| 631 | `[collision_monitor-15]: process has died ... exit code −11` |

Os dois achados novos caem **depois** de o sinal geral ter sido recebido e
**dentro** do `Cleaning up`. Portanto `0 / 7 / 7` descreve a corrida
corretamente: **execução com a pilha de pé, integração e limpeza sem órfãos
seguem APROVADAS**, e o que está anotado é o encerramento.

A distinção é medida, não argumentada: o validador tira um snapshot do
`launch.log` antes de qualquer sinal e examina o log final depois do `limpa`,
com o trecho posterior recortado pela fronteira de posição (§7).

## 4. O que esta decisão NÃO declara

⚠️ **Não** se está declarando saída não limpa aceitável **para hardware real**.
O que se mediu foi Gazebo headless. **Isto é dívida real, potencialmente
relevante no robô físico** — não é acabamento nem detalhe de log. Um nó que
segfalta ao encerrar pode deixar de rodar o que ia rodar no caminho de saída, e
no robô o caminho de saída é o que zera atuador; a zona morta deste projeto já
ensinou que falha sem sintoma custa hora de competição. E sendo o SIGSEGV
**intermitente**, ele não dá para ser descartado por uma corrida que passou: a
quarta passou. A etapa que subir a pilha no robô ligado avalia isso do zero, e
com o robô fisicamente contido.

## 5. Critério futuro (o que fecha esta dívida)

1. os **seis** nós registram `finished cleanly [pid N]` no `launch.log`;
2. **nenhuma** linha `ERROR/FATAL/died` depois do início do encerramento —
   inclusive a de TF;
3. os três números do resultado em **`0 / 0 / 0`**.

Enquanto isso não acontece, o validador **anota** e não esconde: reprovar aqui
seria reprovar a integração por um defeito que não é dela.

## 6. O Nav2 não se atualiza no meio da etapa

| | versão |
|---|---|
| instalado | `ros-jazzy-nav2-collision-monitor` **1.3.12**-1noble.20260615.160103 |
| candidato | **1.3.13**-1noble.20260903.031125 |

A comparação 1.3.12 → 1.3.13 **não traz correção nominal** para este caso, e
trocar a versão do Nav2 no meio de uma etapa que se fecha por evidência
invalidaria a comparação entre as cinco corridas. Fica para depois do passo 8,
como experimento próprio.

**Não há backtrace nem coredump** deste segfault: `systemd-coredump` não está
instalado nesta máquina (`coredumpctl: command not found`), então o −11 é tudo o
que se sabe da morte. Colher backtrace é parte do conserto futuro, não desta
etapa.

## 7. Por que o conserto fica fora do gate desta etapa

Corrigir os cinco de código 1 alcança `robot_base/`, `robot_nav/` e nós já
existentes de `robot_motion/` — os mesmos pacotes que a decisão
[056](056-a-pilha-escolhe-o-robo-e-a-reescrita-vira-arquivo.md) declara
**intocados**. O sexto alcança versão de pacote do sistema (§6). O gate da etapa
6 é: a pilha sobe o robô 3 no Gazebo sem mexer no que já funciona. Consertar
aqui trocaria a garantia "nada do robô 2 foi tocado" por um conserto no meio da
prova.

**A decisão 056 continua 🟡 PROPOSTA até o passo 8.** Esta decisão não a
promove, não a antecipa e não depende dela.

## 8. Alternativas descartadas

- **Reprovar a corrida (RC 1).** Descartada: a integração em execução está
  provada (grafo, TF, mux, bag legível, limpeza sem órfão), e o encerramento é
  outro assunto. Reprovar aqui ensinaria a ignorar o RC — e é justamente por não
  reprovar que o registro precisa ser explícito.
- **Filtrar as linhas do relatório de erros.** Descartada — é o defeito original
  com outro nome: a pasta voltaria a contradizer o arquivo que assina.
- **Consertar os nós agora.** Descartada pelo §7.
- **Atualizar o Nav2 para 1.3.13.** Descartada pelo §6.
- **Contar só depois do `limpa`, um número só.** Descartada: perderia a
  afirmação que mais importa — "com a pilha de pé, zero erros".
- **Mover a fronteira do snapshot para depois do gravador fechar.** Considerada
  na quinta corrida e descartada: a sequência do §3 mostra que os dois achados
  novos vêm **depois** do sinal geral, então a fronteira atual já os classifica
  certo. Mover não mudaria nenhum dos três números.

## 9. Referências

- `~/etapa6/20260924_114704/` — a quarta corrida, que expôs a contradição.
- `~/etapa6/20260924_132510/` — a quinta, RC 0 e `0 / 7 / 7`, com a sequência
  do §3. Ambas preservadas.
- Decisão [056](056-a-pilha-escolhe-o-robo-e-a-reescrita-vira-arquivo.md) — a
  fronteira de pacotes que esta decisão respeita.
- `tools/valida_etapa6/test_valida_etapa6.py` — as travas de ordem
  (snapshot → `limpa` → exame final → manifesto) e dos três números.


## Adendo (2026-09-28) — BLOQUEADORA PARA HARDWARE

O SIGSEGV intermitente do `collision_monitor` no teardown deixou de ser
cosmético: na bateria de subidas da decisão 061 (`~/subidas-robo3/20260928_141047`)
ele deixou **34 segmentos Fast DDS órfãos** em `/dev/shm`, 29 deles fora do
alcance do `fastdds shm clean` (sem a trava `_el`), o que impede a próxima
subida limpa. Por decisão do dono, esta dívida passa a **bloqueadora para o
hardware** até o SIGSEGV ser entendido. Em simulação, a recuperação é a
remoção manual controlada da decisão 061 §2.3.2, registrada como teardown
anômalo.

## Adendo (2026-09-28, tarde) — O MECANISMO IMEDIATO DO SIGSEGV

**Reapareceu** na bateria `20260928_145005` (061 §6), subida 1, depois de um
ensaio limpo às 14h48 — com `/dev/shm` zerado no início das duas. Não é
resíduo de corrida anterior.

**Backtrace** (`docs/dados/2026-09-28-subidas-robo3/apport_collision_monitor_141047/`):
o Apport guardou o crash das **14h11** — o da bateria `141047`, **não** o da
`145005` (com o relatório anterior ainda em `/var/crash`, o Apport não gera
outro para o mesmo executável). Cópia fiel e sha256 em `~/subidas-robo3/apport/`;
no repositório, os campos sem o CoreDump e o backtrace tirado com `gdb` do
CoreDump:

```
#0 rclcpp::PublisherBase::get_subscription_count() const      rdi (this) = 0x10
#1 nav2_collision_monitor::CollisionMonitor::process(...)
#2 nav2_collision_monitor::CollisionMonitor::cmdVelInCallbackStamped(...)
#6 rclcpp::Executor::execute_subscription(...)
#8 rclcpp::executors::SingleThreadedExecutor::spin()
```

`this = 0x10` é o ponteiro de um publisher **já zerado** acrescido do
deslocamento da base — `process()` usou um publisher desmontado. Leitura do
dono sobre o fonte oficial do Nav2 **1.3.12** (versão instalada:
`ros-jazzy-nav2-collision-monitor 1.3.12-1noble.20260615`):
`on_cleanup()` zera `collision_points_marker_pub_`, e `process()` o acessa
sem verificar. Fonte:
<https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_collision_monitor/src/collision_monitor_node.cpp>.
**Causa imediata: publisher desmontado no teardown, não o erro de TF.**

**Contagem diagnóstica** (`sigsegv_nos_launch_logs.txt`, não é taxa): dos 14
`launch.log` preservados em que o `collision_monitor` aparece, 4 têm `-11`
(o dono contou 12 com 4; a diferença de 2 não foi reconciliada). Pelo dono,
as quatro ocorrências têm também o erro de TF e as saídas limpas não — o que
é correlação; o backtrace não passa pelo TF.

**Os cinco `exit code 1`** (`rclpy` no shutdown) aparecem também nas subidas
**sem** SIGSEGV, inclusive no ensaio `144822` — são a falha já conhecida
deste registro, não consequência do crash.

**Ponto em aberto para a correção:** o executor é **single-threaded** (frame
#8). Se `on_cleanup()` e a callback correm no mesmo thread, não há duas
execuções simultâneas; o que precisa ser explicado é como uma callback de
`cmd_vel_in` executa **depois** do `on_cleanup()` (assinatura ainda viva, ou
mensagem já retirada pelo executor). Isso decide se a correção é exclusão
mútua ou "não usar recurso zerado / desfazer a assinatura antes do
publisher". Confirmar no fonte **antes** de escrever a correção.

**Alvo da correção (dono):** sincronizar `process()` com `on_cleanup()` ou
impedir que recursos sejam zerados com callback em voo. **Não** instalar a
1.3.13 como primeiro teste. **Hardware continua bloqueado** até a correção
passar por ensaio e bateria limpa (061).

## Adendo (2026-09-28, noite) — A CORRIDA CONFIRMADA NO FONTE

Só leitura, fontes das versões **instaladas** (clonadas por tag): Nav2 1.3.12
(`6be3614`), rclcpp/rclcpp_lifecycle 28.1.21 (`53cf81e`), rcl 9.2.11
(`22c0b95`), tf2_ros 0.36.21 (`99b1334`), bondcpp 4.2.0 (`8f024dd`).

**Há concorrência real apesar do `SingleThreadedExecutor`**: o preshutdown
roda na thread do manipulador diferido de sinais, não na do executor.

| thread | passo | onde |
|---|---|---|
| executor | `cmdVelInCallbackStamped → process()`; passa `if (!process_active_)` | `collision_monitor_node.cpp:419` |
| executor | `source->getData → Source::getTransform → Buffer::canTransform`: laço `while (now < prazo && … && rclcpp::ok()) sleep(10 ms)`, `transform_tolerance` 0,5 s | `source.cpp:135`, `buffer.cpp:151-159` |
| sinais | SIGINT → `Context::shutdown()` → callbacks de preshutdown **antes** do `rcl_shutdown()` | `signal_handler.cpp:157,267`, `context.cpp:358-374` |
| sinais | `on_rcl_preshutdown → runCleanups → deactivate()`: `process_active_ = false` (`bool` simples) | `lifecycle_node.cpp:105-131` |
| sinais | `destroyBond → ~Bond`: espera até **100 ms** (relógio estável) a confirmação do par, que chega por assinatura servida pelo executor — ocupado | `bond.cpp` (`waitUntilBroken(100ms)`) |
| sinais | `cleanup() → on_cleanup()`: `reset()` de assinatura e publishers, `sources_.clear()`, `polygons_.clear()`, `tf_buffer_.reset()` | `collision_monitor_node.cpp:169-185` |
| sinais | `rcl_shutdown()` → `rclcpp::ok()` falso | `context.cpp:374` |
| executor | o laço do TF sai, "extrapolation into the future"; `process()` segue e usa `collision_points_marker_pub_` zerado → `this = 0x10` → SIGSEGV | `collision_monitor_node.cpp:451/478` |

O `reset()` da assinatura não cancela a callback em curso: o executor recebe
o `SubscriptionBase::SharedPtr` por valor (`executor.cpp:543`) e o bind usa
o `this` cru do nó.

**Assinatura nos logs** (relógio de parede): nas 4 ocorrências,
`Deactivating → Cleaning up` = **104–108 ms** (o timeout do bond, executor
preso) e o erro de TF 6–36 ms **depois** do `Cleaning up`; nas 3 saídas
limpas conferidas (`144822`, `etapa7/114902`, `etapa6/114704`), 10–15 ms e
nenhum erro de TF. O erro de TF é o que **abre a janela**, não a causa. Não
confirmado se o relógio simulado estava parado — não é necessário: uma espera
de até `transform_tolerance` basta, então o mecanismo vale também no
hardware.

**1.3.13 e `main` (`7b9bcb4`, 21-09-2026) têm o mesmo código** — não há
correção upstream para portar.

**Rejeitado: mutex segurado durante o `process()`/`getData()`.** A espera do
TF só termina com `rclcpp::ok()` falso, que só vem depois do preshutdown; se
o preshutdown esperar o lock → deadlock até o launch escalar para SIGKILL
(que deixa exatamente os órfãos de SHM).

**Direção (não fechada):** `process()` usa `sources_`, `polygons_`,
`cmd_vel_out_pub_`, `state_pub_`, `collision_points_marker_pub_` e o TF, e o
`on_cleanup()` destrói tudo. Provável forma: sob **lock curto**, tirar um
snapshot com posse compartilhada de **todos** os recursos que a callback usa,
e o `on_cleanup()` trocar os membros sob o mesmo lock curto. `atomic<bool>` e
a segunda verificação ajudam, mas sozinhos não eliminam as corridas. Um
publisher vivo depois do shutdown é inofensivo no fonte
(`get_subscription_count()` devolve 0 com contexto inválido; o
`LifecyclePublisher` desativado descarta o `publish`). Exige compilar o
`nav2_collision_monitor` como overlay — decisão própria, **depois** do
reprodutor vermelho.

**Próximo:** reprodutor mínimo e determinístico (só o `collision_monitor`,
domínio isolado, 1.3.12 inalterada, ≤ 3 tentativas), critério vermelho =
callback provadamente na espera de TF antes do SIGINT + o mesmo SIGSEGV e
backtrace.
