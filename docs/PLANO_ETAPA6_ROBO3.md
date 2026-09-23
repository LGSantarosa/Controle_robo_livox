# Plano da etapa 6 — a `pilha` sobe o robô 3 no Gazebo

> Etapa 6 do `docs/PLANO_NAV2_ROBO3.md` (§8): "ensinar a `pilha` a escolher
> `sim_robo3` (`robo:=3`, `use_sim_time`, qual atuador encerra)", com o
> herdado da etapa 4 (§3 do `PLANO_ETAPA4_ROBO3.md`): `ros2 param get` do
> `footprint` e do `footprint_padding` **vivos** nos dois costmaps — na etapa 4
> só o YAML reescrito foi provado.
>
> Estado: **APROVADO pelo dono em 23-09**, com seis correções obrigatórias —
> incorporadas abaixo e listadas no §9. Autorizado seguir pelos testes
> vermelhos e pela implementação estática; as primeiras corridas de Gazebo
> (passos 6 e 7) seguem dependendo de aviso prévio e do "pode".
> Partida: `1f49981`, branch
> `etapa6-pilha-robo3`. Nada de hardware: esta etapa inteira roda no PC de dev,
> **robô desligado**. A auditoria do notebook
> (`docs/ROTEIRO_ETAPA6_PRECONDICAO.md`) é trilha paralela e continua sendo a
> porta obrigatória antes de qualquer ação física — ela não bloqueia isto, e
> isto não a substitui.

---

## 0. O que a leitura do código mostrou

Seis achados, e três deles mudam o desenho.

| # | achado | onde |
|---|---|---|
| **A** | **A trava não é a recusa, é a reescrita.** `_recusa_robo` é uma linha de porta; o que realmente impede o robô 3 é que a pilha monta `perfil.parametros(2, pkg)` **fixo** e **aborta de propósito** se o perfil pedir reescrita. O perfil do robô 3 pede **7**: `footprint` e `footprint_padding` nos dois costmaps, e três polígonos/tempo do reflexo | `pilha.launch.py:172–179`; `perfil.py:_robo3` |
| **B** | **Momento errado.** `generate_launch_description()` roda **sem contexto**; `robo` só existe dentro de um `LaunchContext`. Escolher o perfil pelo argumento exige `OpaqueFunction` ou ler o `argv` — e ler `argv` já tem precedente na casa (`_passou`) | `pilha.launch.py:150–162`, `:108` |
| **C** | **O include do simulador não troca só de arquivo.** A pilha inclui `robot_base/launch/sim.launch.py` passando `planta`, e o `sim_robo3.launch.py` **não declara `planta`** (ele declara `mundo`, `x`, `y`, `yaw`, `gui`, `placa`). Include com argumento não declarado morre na subida. O mundo padrão também difere: a pilha usa `worlds/pista_obstaculos.sdf` da raiz; o `sim_robo3`, `robot_base/worlds/pista_livre.sdf` | `pilha.launch.py:596–621`; `sim_robo3.launch.py:56–70` |
| **D** | **No Gazebo a cadeia NÃO termina no `cmd_vel_to_wheels`.** Ela termina no `hoverboard_base_controller`: `compensador_rumo` → `/cmd_vel_bruto` → `placa_simulada` → `ros2_control`. O `sim_robo3` **já sobe** a placa fingida e o controlador. A fronteira da decisão 054 (`WheelSpeeds` → MEGA) é do **hardware**, e `WheelSpeeds` não tem consumidor no Gazebo — foi exatamente isto que derrubou a "etapa 3" da v1 | `placa_simulada.py:186`; `sim_robo3.launch.py:94–150`; `PLANO_NAV2_ROBO3.md:91` |
| **E** | **Nenhum dos dois muxes serve inteiro ao robô 3.** O da pilha tem cinco faixas — `joy_vel` 100, `key_vel` 90, `web_vel` 50, **`unstuck_vel` 30** e `auto_vel` 10 — e **não tem `dpad_vel`**. O do robô 3 tem `dpad_vel` (110) e `joy_vel` (100), e **não tem nem `auto_vel` nem `unstuck_vel`** — com ele, a autonomia **e o desencalhe** publicam para o vazio, e o sintoma seria robô parado com o Nav2 dizendo que está navegando | `robot_motion/config/twist_mux.yaml:60–104`; `robot_nav/config/twist_mux_robo3.yaml` |
| **E′** | 🔧 **Correção da revisão de 23-09** (a primeira redação deste achado errou): o mux da pilha **já tem** `unstuck_vel`, e quem publica nele é o **`path_follower`** (`path_follower.py:741`), não um supervisor à parte. O comentário do topo do `twist_mux.yaml` ("lá existe uma entrada `unstuck_vel` … aqui ela não existe AINDA") está **vencido** desde que o desencalhe entrou. Não corrigir aqui: é arquivo do robô 2, e o §6 proíbe | `twist_mux.yaml:28` × `:98`; `path_follower.py:741` |
| **F** | O `sim_robo3` fixa `use_sim_time: True` **literal** nos nós dele; a pilha passa a **substituição** `sim` nos dela. A prova viva tem de cobrir os dois lados, porque são dois mecanismos diferentes de chegar ao mesmo valor | `sim_robo3.launch.py`; `pilha.launch.py` |

O achado **E** é o que tem cara de zona morta: faixa que não existe não dá
erro, dá silêncio.

## 1. Escopo

**Entra:**

- a `pilha` aprende a **materializar** o perfil **do robô 3**: aplicar as
  reescritas (`perfil.aplica_reescritas`, que já existe e é testada) e
  **escrever os DOIS YAMLs resultantes em disco** (Nav2 e `collision_monitor`),
  porque servidor do Nav2 lê arquivo, e dicionário passado ao `Node` do
  servidor não chega aos costmaps que moram dentro dele.
  🔴 **O robô 2 não materializa arquivo nenhum** — ele segue recebendo os
  caminhos originais do `share/`. Materializar para ele, ainda que com
  conteúdo idêntico, mudaria o valor do parâmetro (o caminho) e tornaria o
  gate "byte a byte" do §6 impossível de cumprir por construção;
- `robo:=3 sim:=true` escolhendo `sim_robo3.launch.py`, com os argumentos que
  ele de fato declara (achado C);
- **um mux só**, com faixa para a autonomia **e para o desencalhe** no robô 3
  (achados E e E′);
- a cadeia de atuação do simulador terminando no `hoverboard_base_controller`,
  e **nenhum** nó da fronteira de hardware no grafo (achado D);
- as provas vivas: `use_sim_time`, `footprint`/`footprint_padding` nos dois
  costmaps, `/Odometry`, `/scan`, TF, e um objetivo curto aceito pelo Nav2;
- o gate de não regressão do robô 2 (§6);
- coletor próprio `bin/valida-etapa6` + `tools/valida_etapa6/`, no molde da
  etapa 5 (o `bin/smoke-gazebo-robo3` é evidência congelada da trilha §8-B e
  **não** é reaproveitado por dentro).

**Não entra (fronteira explícita):**

- **`robo:=3 sim:=false`** — continua recusado, e é assim de propósito (§2);
- **o Livox de verdade, o FAST-LIO e a rede do Mid-360** — etapa 7. Aqui a
  nuvem vem do `gpu_lidar` do Gazebo e a odometria vem da ponte, não de LIO;
- **medir qualquer coisa do chassi 3** — massa, centro de massa e altura do
  Livox no URDF são provisórios/chute declarados no cabeçalho do `sim_robo3`;
- **sintonia**: nenhum ganho, tolerância ou velocidade do robô 3 muda aqui.
  Todos os `herdados_provisorios` do `perfil_robo3.yaml` seguem herdados, com
  a etapa que os fecha (7, 8, 9, 10);
- **o `compensador_rumo` com curvatura do robô 3**: não há curvatura medida
  (etapa 8). No robô 3 ele sobe com os defaults e `curv_medido_em` dizendo
  que não foi medido nesta máquina;
- **comparar robô 3 com robô 2**: o Gazebo não decide chassi (§8).

## 2. A matriz das quatro combinações

É o contrato central da etapa, e cada linha tem teste próprio.

| `robo` | `sim` | o que acontece | por quê |
|---|---|---|---|
| `2` | `true` | **intacto.** Mesma árvore, mesmos parâmetros de hoje, byte a byte | é a linha de base contra a qual todo CSV histórico foi medido |
| `2` | `false` | **intacto.** Idem | é o robô que hoje funciona; regressão aqui é o pior resultado possível desta etapa |
| `3` | `true` | **caminho novo**: perfil 3 materializado, `sim_robo3`, cadeia terminando no `hoverboard_base_controller` | é o que a etapa 6 existe para provar |
| `3` | `false` | **recusado**, com mensagem que diz o que falta | o único jeito de subir isto seria no robô: a pilha mandaria comando por uma cadeia cuja fronteira de hardware não está no grafo, com localização que não existe (o LIO é a etapa 7). Deixar passar é convite a subida acidental no robô |

A recusa do `3 sim:=false` **não** é "ainda não implementei": é uma trava
deliberada desta etapa, e o texto dela tem de nomear as duas coisas que
faltam (a fronteira do atuador real e a localização), não dizer "não existe".

## 3. Decisões — **DECIDIDAS pelo dono em 23-09**

| # | decisão | o que ficou |
|---|---|---|
| **D1** | como o número do robô chega à árvore (achado B) | **(ii) aprovada**: perfil escolhido lendo o `argv` (precedente `_passou`) **+ trava de coerência** no `LaunchContext`. É uma **ponte deliberada, não uma API geral** — e por isso a trava tem **três bocas**, todas matando **antes de qualquer processo ou escrita em disco**: (1) `robo:=` repetido no `argv`; (2) `argv` e contexto divergindo; (3) include **programático** com `robo:=3` que não aparece no `argv` (que cai na boca 2, porque o `argv` entrega o default `2`). Descartadas: (i) `OpaqueFunction` montando as 835 linhas — reescreve a launch que hoje sobe o robô 2 funcionando; (iii) escolher o arquivo por substituição — não resolve o `path_follower` (dicionário, não arquivo) e tornaria o `robot_base` dependência obrigatória do robô 2 |
| **D2** | onde moram os YAMLs materializados | **pasta persistente da corrida, aprovada**, e ela é a **raiz de evidências**. Os **dois** arquivos do robô 3 (Nav2 e `collision_monitor`) são escritos **atomicamente** (arquivo temporário no mesmo diretório + `os.replace`), com os **caminhos absolutos impressos** na subida e incluídos no `SHA256SUMS`. ⚠️ O rosbag vai num **subdiretório que ainda não existe**, porque `ros2 bag record -o` exige diretório inexistente e abortaria contra a raiz já criada |
| **D3** | o mux do robô 3 na pilha (achados E e E′) | **por perfil, aprovado, com correção de nome e de faixas**: nasce `robot_motion/config/twist_mux_pilha_robo3.yaml`, com **quatro** faixas — `dpad_vel` 110, `joy_vel` 100, **`unstuck_vel` 30**, `auto_vel` 10. Sem o `unstuck_vel`, o desencalhe que o **próprio perfil do robô 3 configura** (`perfil.py:108–115`: `re_largura`, `re_recuo_para_choque`, `desencalhe_pivo_folga`) publicaria para o vazio. O nome longo é de propósito: `robot_nav/config/twist_mux.yaml` (controle físico) fica **intocado**, e não se criam duas fontes aparentemente canônicas com o mesmo nome |
| **D4** | canais humanos novos no robô 3 | **`key_vel` fica fora**, como a etapa 5 já decidira, **e `web_vel` também não entra** — nem implicitamente, por estar no mux do robô 2. Canal humano novo exige decisão própria |

## 4. O que a etapa fixa por contrato

Cada item vira teste, e cada teste falha **antes** de existir a implementação.

1. **Um mux só, e com as quatro faixas.** Com `robo:=3 sim:=true`, exatamente
   **um** nó `twist_mux` no grafo, lendo
   `robot_motion/config/twist_mux_pilha_robo3.yaml`, com `dpad_vel` 110,
   `joy_vel` 100, `unstuck_vel` 30 e `auto_vel` 10 — conferidas **vivas**, por
   `ros2 param get`, não só no arquivo. O `controle_robo3.launch.py` **não** é
   incluído pela pilha, e `robot_nav/config/twist_mux_robo3.yaml` — o mux do
   controle físico do robô 3 — não é tocado.
2. **Quem encerra a cadeia no simulador**: `placa_simulada` consumindo
   `/cmd_vel_bruto` e publicando ao `hoverboard_base_controller`. E a prova
   pela negativa: **nenhum** `cmd_vel_to_wheels`, `mega_bridge` ou tópico
   `WheelSpeeds` no grafo com `sim:=true`.
3. **`use_sim_time` vivo**, por `ros2 param get`, em **lista nominal
   versionada** de nós — todos `true` com `sim:=true`. A lista ser versionada é
   o ponto: nó que aparecer no grafo e não estiver na lista **reprova**, senão
   a prova envelhece em silêncio a cada nó novo.
4. **`footprint` e `footprint_padding` vivos nos dois costmaps**, lidos por
   `ros2 param get` de `/global_costmap/global_costmap` e
   `/local_costmap/local_costmap`, e comparados com o que
   `perfil.parametros(3, …)` devolve — **nunca redigitados no teste**. O
   contraste é gritante e serve de controle positivo: robô 3 é
   `[[0.0825, 0.19], …, [-0.2913, 0.19]]`; o valor do robô 2 que está hoje no
   `nav2.yaml` é `[[0.35, 0.2775], …]`. Ler o segundo com `robo:=3` significa
   que a reescrita não pegou.
5. **`/Odometry`, `/scan` e TF**: os dois tópicos com publicador e ao menos
   uma mensagem recebida; `tf2_echo map base_link` e `tf2_echo base_link
   livox_frame` respondendo; e **um único** publicador de `map → odom` (o
   `tf_map_odom` e o AMCL se excluem, e os dois juntos fazem a pose piscar).
6. **O objetivo curto**, e o critério é exato: pista livre, robô na pose de
   spawn, goal a **1,0 m à frente na mesma orientação**. Aprovado quando as
   **três** valem: o BT devolve `SUCCEEDED`; a pose final está dentro do
   `xy_goal_tolerance` vigente (0,25 m, `nav2.yaml:140`); e o
   `hoverboard_base_controller` recebeu comando **não nulo** — sem este
   terceiro, "chegou" pode ser "já estava lá", que é o modo de falha mais
   provável com um alvo curto. Teto: 60 s de **tempo simulado**.
7. **Combinação recusada não deixa rastro.** A escrita dos YAMLs acontece
   **depois** da validação de `robo` e `sim`, nunca antes: uma subida recusada
   não pode deixar **falsa evidência** de uma corrida que não houve. Isto tem
   consequência de implementação, e é a forma da mudança: o **caminho** dos
   dois arquivos é calculado na descrição (o `Node` precisa dele), mas os
   **bytes** são escritos por uma ação que roda **depois** do `_recusa_robo`.
   Teste: subida recusada → a pasta da corrida não existe, ou existe vazia.
8. **Limpeza sem órfão.** Varredura de processos no molde do
   `bin/smoke-gazebo-robo3` (que **não** usa `pkill -f`, e o motivo está lá:
   já matou uma sessão ssh), estendida aos nós da pilha. Sobrou processo →
   código de saída ≠ 0 e a lista no relatório.

## 5. Testes — vermelho antes, e onde cada um roda

**5.1 Estáticos (sem ROS de pé), em `test_pilha_robo.py` e vizinhos.** Percorrem
a launch num `LaunchContext` real, como o arquivo já faz hoje:

- a matriz do §2, as quatro linhas, incluindo as duas mensagens de recusa;
- `robo:=3` monta pelo `perfil.parametros(3, …)` — trocado o perfil, troca o
  que o nó recebe (o teste de hoje faz isso para o robô 2 com `monkeypatch`);
- a trava de coerência da D1: `argv` dizendo um robô e contexto dizendo outro
  **morre**, e morre **antes** de qualquer ação operacional;
- o include do simulador recebe só argumento **declarado** pelo alvo (achado C)
  — teste que lê os `DeclareLaunchArgument` do `sim_robo3` em vez de confiar
  numa lista escrita à mão;
- nenhuma reescrita pedida fica sem aplicar: cada um dos **dois** YAMLs
  materializados, relido do disco, é **igual** a
  `aplica_reescritas(base, reescritas)`;
- **o robô 2 não materializa nada**: com `robo:=2`, os caminhos que os nós
  recebem são os do `share/`, e a pasta da corrida não ganha YAML nenhum;
- **recusa não escreve**: nas duas combinações recusadas, nenhum arquivo.

**5.2 Vivos (Gazebo), em `bin/valida-etapa6`.** Os oito itens do §4, um por
linha do CSV, com veredito por item e no fim. Sobe headless, roda, derruba,
confere que não sobrou nada, e deixa tudo em `~/etapa6/<carimbo>/`, que é a
**raiz de evidências**: console, `launch.log`, `resultado.csv`, os **dois YAMLs
materializados** (caminhos absolutos impressos na subida) e o `SHA256SUMS`, que
inclui os dois. ⚠️ O rosbag, quando entrar, vai em `<carimbo>/bag/` — um
subdiretório **que ainda não existe**, porque `ros2 bag record -o` exige
diretório inexistente e abortaria contra a raiz já criada.

O dono só roda `bash bin/valida-etapa6` e me manda a pasta. Nada de relatar
console.

## 6. Gate de não regressão do robô 2

O robô 2 é o que funciona hoje. Esta etapa fecha só se **as quatro** valerem:

- **(a) diff contra `1f49981`** tocando apenas `bin/`, `docs/`, `tools/`,
  `ros2_packages/robot_motion/launch/pilha.launch.py`,
  `ros2_packages/robot_motion/robot_motion/perfil.py`,
  `ros2_packages/robot_motion/config/twist_mux_pilha_robo3.yaml` (**novo**),
  `ros2_packages/robot_motion/config/perfil_robo3.yaml`,
  `ros2_packages/robot_motion/test/` e `ESTADO_PROJETO.md`.
  **Nenhum** arquivo de `robot_base/`, de `robot_nav/` (inclusive o
  `robot_nav/config/twist_mux_robo3.yaml`, o mux do controle físico do
  robô 3, e `robot_nav/config/twist_mux.yaml`), nem
  `robot_motion/config/{nav2,collision_monitor,twist_mux,movimentacao*}.yaml`;
- **(b) a suíte inteira verde**, com o número de hoje como piso: **1262**
  (coletados em `1f49981`, conferido — não o 1239 de 22-09, que envelheceu);
- **(c) as duas combinações do robô 2** percorridas e a lista de parâmetros de
  **cada nó** comparada com a de `1f49981`, byte a byte — a maquinaria
  (`evaluate_parameters`) já está no `test_pilha_robo.py`;
- **(d) uma corrida do robô 2 no Gazebo**, comparada com a linha de base v2
  (`bin/linha-de-base-robo2`). O (c) prova que a árvore não mudou; o (d) prova
  que ela ainda **anda**.

Se um arquivo do robô 2 precisar mudar, o passo **para e volta ao dono**.

## 7. Ordem — um commit por mudança, vermelho antes

| # | mudança | vermelho antes | prova |
|---|---|---|---|
| 1 | este plano + a decisão 056 como **proposta** | — | — |
| 2 | testes vermelhos da matriz do §2 (as quatro linhas) e da trava de coerência | eles mesmos | falham contra a pilha de hoje, pelo motivo certo |
| 3 | **infraestrutura de materialização instalada, e no-op para o robô 2** (D1 + D2): a função que aplica as reescritas e escreve os dois arquivos entra e é testada em isolamento; o robô 2, sem reescrita, **não escreve nada** e segue com os caminhos do `share/` | o materializado relido é igual a `aplica_reescritas(...)`; e o caminho que o nó recebe com `robo:=2` **não** muda | o robô 2 com a **mesma** lista de parâmetros (§6c) |
| 4 | `robo:=3` escolhe perfil 3 e `sim_robo3`, com os argumentos certos (achado C), a escrita depois da validação (§4.7), e `3 sim:=false` recusa | os do passo 2 viram verde | estáticos |
| 5 | o mux do robô 3 na pilha (D3): nasce `robot_motion/config/twist_mux_pilha_robo3.yaml` com as quatro faixas (`dpad_vel` 110, `joy_vel` 100, `unstuck_vel` 30, `auto_vel` 10) e o mux passa a vir do perfil | faixa de autonomia **e** de desencalhe ausentes reprovam | estáticos + grafo |
| 6 | `bin/valida-etapa6` + `tools/valida_etapa6/`, e a **primeira corrida** | — | pasta de evidência com os oito itens |
| 7 | o objetivo curto (§4.6) | — | a mesma pasta, com o critério de três partes |
| 8 | decisão 056 → **aplicada**, DIARIO, ESTADO, gate do §6 | — | — |

Os passos 6 e 7 sobem Gazebo neste PC — **anuncio e espero o "pode"** antes de
cada um; robô **desligado** o tempo todo, e nenhum deles toca no notebook.

## 8. O que isto NÃO prova

O Gazebo aprova **integração de software**, e só. Não prova:

- 🔴 **nada do Livox**: SDK, rede, IP do host, `/livox/lidar` real, FAST-LIO,
  `/Odometry` de LIO. Aqui a odometria vem da ponte do Gazebo, que é a
  verdade do simulador — o oposto de estimar pose;
- 🔴 **escala, massa e dinâmica do chassi 3**: são provisórias por declaração
  do próprio `sim_robo3` (o Livox e o NUC nem subiram no robô), e a altura do
  Livox é chute;
- 🔴 **derrapada, zona morta e a placa**: o modelo de atuador do robô 3 é
  **herdado do robô 2**, e o `rendimento_giro` 0,80 foi medido naquele
  contato, com outra bitola e três apoios em vez de quatro;
- 🔴 **que o robô 3 é melhor que o robô 2**: comparar chassi num simulador
  cujos números nós mesmos demos não compara nada;
- 🔴 **o `frente:=-1.0`, a MEGA e o watchdog dela**: a fronteira de hardware
  está fora do grafo aqui, por construção (achado D).

Corrida limpa no Gazebo é permissão para ir ao robô com uma hipótese, não
certificado de que funciona.

## 9. Correções da revisão do dono (23-09), já incorporadas

1. **O robô 2 não materializa arquivo nenhum** — segue com os caminhos do
   `share/`. A redação anterior ("materializar, ainda só para o robô 2") era
   **incompatível com o próprio gate do §6(c)**: materializar muda o valor do
   parâmetro (o caminho), e a lista deixaria de ser idêntica byte a byte à de
   `1f49981`. §1 e §7 (passo 3) corrigidos.
2. **A escrita acontece depois da validação de `robo` e `sim`** — combinação
   recusada não deixa falsa evidência. Virou o contrato §4.7, com a forma da
   implementação explicitada: caminho na descrição, bytes depois do
   `_recusa_robo`.
3. **Passo 3 redigido como "infraestrutura instalada, no-op para o robô 2"**.
4. **O mux do robô 3 leva `unstuck_vel` (30)**, não só `auto_vel`: o perfil do
   robô 3 **configura** a ré e o desencalhe (`perfil.py:108–115`) e o
   `path_follower` publica em `/unstuck_vel` (`path_follower.py:741`). Sem a
   faixa, o desencalhe publicaria para o vazio. 🔧 Isto também **corrige um
   erro da primeira redação** deste plano, que listou o mux da pilha sem o
   `unstuck_vel` — achado **E′** do §0.
5. **O piso da suíte é 1262**, não 1239 (coletados em `1f49981`, conferido).
6. **Os muxes aparecem qualificados pelo pacote**, e o novo arquivo chama-se
   `robot_motion/config/twist_mux_pilha_robo3.yaml` — nome inequívoco, para
   não existirem duas fontes aparentemente canônicas com o mesmo nome. O
   `robot_nav/config/twist_mux_robo3.yaml` (o mux do controle físico do robô 3)
   fica **intocado**, e o
   §6(a) passou a dizer isso explicitamente.

Também da revisão, nas decisões: a trava do D1 ganhou as **três bocas**
(duplicidade de `robo:=`, divergência `argv`×contexto, include programático);
o D2 ganhou **escrita atômica dos dois arquivos**, caminhos absolutos
impressos, entrada no `SHA256SUMS` e o **rosbag em subdiretório inexistente**;
o D4 passou a barrar também o `web_vel`.
