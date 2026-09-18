# Plano da etapa 4 — perfis `robo2`/`robo3` e bringup único do robô 3

> **Status: PLANO APROVADO para começar pelo passo 0** (revisão do dono em
> 2026-09-18, com as correções da §13 aplicadas). Nada implementado ainda. O
> Gazebo só sobe com o dono acompanhando.
>
> Etapa 4 do `docs/PLANO_NAV2_ROBO3.md` (§8), com o que herdou da etapa 3 pela
> fronteira mudada em 18-09 (decisão 052): consumo do footprint canônico,
> `footprint_padding` explícito, os outros consumidores de geometria e o
> `robot_state_publisher`.
>
> 🔴 **Validação só em desenvolvimento e Gazebo. Nenhum deploy em robô físico**
> — nem no NUC do robô 2, nem no notebook do robô 3 — até o dono decidir, depois
> da etapa inteira fechada.

---

## 0. O que a leitura do código mostrou (e muda o desenho)

Conferido no código em 18-09, não de memória:

1. **O seguidor assume robô simétrico em x.** `path_follower.vao_frente()`
   (`path_follower.py:1011`) chama `vao_no_corredor_frontal(..., re_largura,
   re_recuo_para_choque)`: o **recuo traseiro** é usado como **avanço do
   para-choque dianteiro**. No robô 2 isso passa por coincidência; no robô 3 a
   frente fica a **0,0825 m** do `base_link` e a traseira a **0,2913 m**. Com o
   recuo traseiro na frente, a mira adaptativa acharia 21 cm a menos de espaço
   à frente do que existe. ➡️ Não dá para resolver só com config: precisa de um
   parâmetro novo, `avanco_para_choque`.
2. **O robô 2 não declara `footprint_padding`** em nenhum YAML
   (`grep` vazio em `robot_motion/config/`). Ele roda hoje com o **padrão do
   Nav2**, que ninguém escreveu. Torná-lo explícito exige **medir o valor vivo**
   antes, não supor.
3. **O robô 2 amarra footprint = polígono de parada do reflexo** (decisão 032;
   `test_o_planejador_usa_o_CONTORNO_orientado_do_reflexo`,
   `test_configs_coerentes.py:1475`). Para o robô 3 essa igualdade é **proibida**
   (052: footprint sem folga, reflexo = footprint + margem). ➡️ A coerência
   passa a ser **por perfil**, e o teste do robô 2 continua valendo só para ele.
4. **Nomes de nó iguais, contratos diferentes.** `twist_mux`, `joy_node` e
   `teleop_twist_joy_node` existem na cadeia do robô 2 (`TwistStamped`,
   `use_stamped: true`, saída `/compensador_rumo/cmd_vel`) e na do robô 3
   (`Twist`, `use_stamped: false`, saída `/cmd_vel`). Juntos no mesmo domínio
   ROS, o ROS 2 **aceita os dois sem erro** — nó duplicado é falha silenciosa.
5. **A pilha só aprende a escolher o `sim_robo3` na etapa 6**, e o contrato de
   mensagem só se unifica na **etapa 5**. Então, na etapa 4, **nada** que sobe o
   robô 3 pode subir junto com a `pilha`.
6. O `desencalhe_pivo_folga` do robô 2 (0,334) é o raio varrido pela quina da
   caixa (0,314, comentário em `path_follower.py`) + margem. No robô 3 o ponto
   mais distante que gira em torno do `base_link` é a **boba varrida**:
   √(0,2485² + 0,105²) + 0,04272 ≈ **0,3125 m**. A quina do retângulo do
   footprint (0,348 m) é área **vazia** e não entra.

---

## 1. O comportamento do robô 2 que NÃO pode mudar

Critério objetivo: **o conjunto de parâmetros efetivos do robô 2 sai idêntico**,
com **duas** diferenças permitidas e declaradas, e nenhuma outra:

1. `footprint_padding` passa a estar **escrito** nos dois costmaps, com **o mesmo
   valor** que já vigora (medido no passo 0);
2. `avanco_para_choque` aparece no `path_follower`, **novo**, com valor **0,28**
   — o mesmo número que o `vao_frente()` já usa hoje, via `re_recuo_para_choque`.

A comparação é **semântica** (§10.2), não texto de `ros2 param dump`.

| o quê | onde | hoje |
|---|---|---|
| footprint global e local | `nav2.yaml:243`, `:384` | `[[0.35, ±0.2775], [-0.2665, ±0.2775]]` |
| `footprint_padding` | — (ausente) | padrão do Nav2 → **medir** (passo 0) |
| reflexo `PolygonStop` / `PolygonApproach` | `collision_monitor.yaml` | como está |
| `passagem_meia_largura` / `passagem_margem` | default do `path_follower` | 0,2275 / 0,03 |
| `re_largura` / `re_recuo_para_choque` | default do `path_follower` | 0,555 / 0,28 |
| avanço do para-choque (implícito) | = `re_recuo_para_choque` | 0,28 |
| `desencalhe_pivo_folga` | default do `path_follower` | 0,334 |
| cadeia de comando | `pilha` | `TwistStamped` ponta a ponta, mux `use_stamped: true` |
| nós e nomes | `pilha`, `base`, `joystick` | os mesmos, 1 instância cada |
| comandos | §8 | todos funcionando sem argumento novo |

O mesmo vale para a cadeia do robô 3 que existe hoje: `frente:=-1.0` padrão,
`Twist` cru, mux próprio, números do Xbox (decisões 046, 048, 049).

---

## 2. Formato e seleção dos perfis

### 2.1 Formato

- **Um arquivo de sobreposição por robô**, com **só as chaves que diferem** da
  base: `robot_motion/config/perfil_robo3.yaml`. O robô 2 **não tem** arquivo de
  sobreposição: o perfil dele **é** a base de hoje (`nav2.yaml`,
  `collision_monitor.yaml`, os defaults do `path_follower`). Assim o caminho do
  robô 2 não ganha camada nova para quebrar.
- Cada chave do perfil do robô 3 carrega a **classe** do §5 do plano geral
  ((b) geometria, (c) política…) e a origem. As de (b) **não levam número**:
  apontam para `robot_base/config/geometria_robo3.yaml` (§3).
- O que o robô 3 herda do robô 2 sem medida (inflação, `v_max`, ganhos) fica
  **listado no perfil como herdado provisório**, com a etapa que o fecha — não
  some por omissão.

### 2.2 Seleção

- Argumento `robo:=2|3` na `pilha.launch.py`, **padrão `2`**. Sem argumento,
  nada muda.
- Um único ponto monta os parâmetros: uma função pura
  `robot_motion/perfil.py → parametros(robo)`, que lê base + sobreposição +
  geometria e devolve o que cada nó recebe. A `pilha` passa a usá-la **para o
  robô 2 também** — é isso que a torna consumidor real, e não código paralelo.
- ✅ **D1 (aprovada):** `robo:=3` **existe e recusa com erro explícito** até a
  etapa 6 ("pilha do robô 3 é a etapa 6"). O perfil é montado e testado sem
  fingir que a pilha já funciona. Mesma técnica do
  `_recusa_combinacao_sem_sentido` que já existe na pilha.

⚠️ Consequência declarada: na etapa 4, o consumidor do perfil do robô 3 é a
função de montagem + os testes dela, e não um nó rodando. A função é a mesma
que monta o robô 2 em produção.

---

## 3. Consumo do footprint canônico sem copiar vértices

- `perfil.parametros(3)` **lê** `geometria_robo3.yaml` e gera a string
  `footprint` que o Nav2 exige, para os **dois** costmaps (global e local).
- Os costmaps são nós próprios (`global_costmap/global_costmap`,
  `local_costmap/local_costmap`) dentro do `planner_server` e do
  `controller_server`. Parâmetro passado como dicionário ao `Node` do servidor
  **não chega** neles. O caminho é reescrever o YAML na subida com
  `nav2_common.launch.RewrittenYaml` (chave `footprint`) ou equivalente.
- ⚠️ **O que dá para provar nesta etapa, e o que não dá.** Com D1 = recusar,
  **nenhum costmap do robô 3 roda na etapa 4**. Então:
  - **nesta etapa:** um teste avalia o YAML **reescrito**, o arquivo que o nó
    carregaria, e confere a chave aninhada
    `global_costmap.global_costmap.ros__parameters.footprint` (e a do local)
    igual ao artefato. Isso prova o que chega ao arquivo, **não** o que o nó
    aceita;
  - **na etapa 6**, quando a pilha do robô 3 subir no Gazebo: `ros2 param get`
    do `footprint` e do `footprint_padding` **vivos** nos dois costmaps. Fica
    escrito lá como critério herdado, para não ser esquecido.
- Teste de "não redigitar": `perfil_robo3.yaml` **não pode conter** lista
  numérica de vértices, e o footprint montado é **igual** ao artefato da 052.

---

## 4. `footprint_padding` explícito nos dois

| robô | valor | classe | de onde |
|---|---|---|---|
| 2 | **o valor vivo medido no passo 0** (esperado: padrão do Nav2) | (c) | medição, para não mudar nada |
| 3 | ✅ **D2 (aprovada):** o valor vivo medido no robô 2, nos **dois** costmaps | (c) | **política herdada**, provisória, declarada como tal no perfil |

Teste: os **dois** costmaps de **cada** perfil declaram `footprint_padding`;
ausente reprova.

---

## 5. Os outros consumidores de geometria, com valores próprios

Os de (b) saem da geometria e os de (c) são política, e ficam em **chaves
separadas** (o §5 do plano geral proíbe somar e guardar um número só).

✅ **D3 aprovada pelo dono com valores próprios**, provisórios (herdados do robô 2
até a etapa 8 medir a frenagem). O reflexo tem **duas funções de segurança
diferentes**, e cada uma tem a sua política, em **chaves separadas no perfil**:

- **`PolygonApproach`** — desacelera antes: footprint **+ 0,03 m em cada face**,
  `time_before_collision` **0,75** herdado.
- **`PolygonStop`** — para: margem **0,133 m na frente**, **0,05 m nas laterais**
  e **0,05 m atrás**.

Conferidas no robô 2 (corpo 0,433 × 0,455, `base_link` no centro da caixa):
Stop 0,35 − 0,2165 = 0,1335 na frente, 0,2665 − 0,2165 = 0,05 atrás,
0,2775 − 0,2275 = 0,05 no lado; Approach 0,03 em todas.

| parâmetro | robô 2 (inalterado) | robô 3 (b) geometria | robô 3 (c) política | robô 3 montado |
|---|---|---|---|---|
| footprint | literal de hoje | artefato 052 | `footprint_padding` = D2 | artefato 052 |
| `PolygonApproach` | literal de hoje | footprint 052 | `approach_margem`: 0,03 em cada face; `time_before_collision` 0,75 | x +0,1125 / −0,3213, y ±0,220 |
| `PolygonStop` | literal de hoje | footprint 052 | `stop_margem`: frente 0,133, lados 0,05, trás 0,05 | x +0,2155 / −0,3413, y ±0,240 |
| `passagem_meia_largura` | 0,2275 | **0,190** (pneu) | `passagem_margem` 0,03 (herdada, chave própria) | 0,190 |
| `re_largura` | 0,555 | 2 × 0,190 | + 2 × margem lateral do **Stop** (regra do robô 2: 0,555 = largura do Stop) | **0,480** |
| `re_recuo_para_choque` | 0,28 | **0,2913** (traseira varrida) | a folga fica no `re_folga` (0,30, inalterado) — **geometria separada da folga** | 0,2913 |
| `avanco_para_choque` (novo) | **0,28** (= o recuo de hoje; nada muda) | **0,0825** (frente do pneu) | — | 0,0825 |
| `desencalhe_pivo_folga` | 0,334 | **0,3125** (boba varrida) | + 0,020 (a do robô 2) | **0,3325** |

A coluna "montado" é o **resultado esperado** que o teste confere; o número que
mora no perfil é só o de (c). O de (b) vem do artefato.

Todos os valores de (b) do robô 3 são **calculados** a partir do URDF e do
artefato por teste, e não digitados.

---

## 6. Bringup único do robô 3 — quem sobe o quê

✅ **D4 (aprovada):** o bringup é o próprio `robot_nav/launch/controle_robo3.launch.py`,
que ganha o `robot_state_publisher`. Sem arquivo novo: um arquivo a menos em que
um nó pode aparecer duas vezes.

🔴 **Dependência explícita:** o URDF mora no `robot_base`, e hoje o
`robot_nav/package.xml` **não declara** `robot_base` (conferido). Entra
`<exec_depend>robot_base</exec_depend>` no mesmo passo — sem ela o launch
funciona no dev por acaso (os dois pacotes estão no mesmo `install/`) e quebra
em qualquer máquina que compile só o `robot_nav`.

| nó | sobe em | observação |
|---|---|---|
| `robot_state_publisher` (URDF `robo3`, `sim:=false`) | **controle_robo3** (novo) | publica `base_link → livox_frame` (yaw 0, 051). Sem `/joint_states` no robô 3 (a MEGA não é ros2_control), as juntas contínuas das rodas e bobas **ficam sem TF** — esperado, e declarado |
| `mega_bridge` | controle_robo3 | inalterado |
| `cmd_vel_to_wheels` (`linear_sign` = `frente`) | controle_robo3 | inalterado |
| `joy_node`, `teleop_twist_joy_node` | controle_robo3 | inalterado (`Twist` cru) |
| `dpad_reto` | controle_robo3 | inalterado |
| `twist_mux` (`twist_mux_robo3.yaml`, `Twist`) | controle_robo3 | inalterado — **um mux só é a etapa 5** |

⚠️ O bloco `HoverboardSystem` do URDF (`sim:=false`) fala de `hoverboard_driver`
e `/dev/ttyUSB0`, que o robô 3 **não usa**. O RSP ignora esse bloco. Fica
anotado, **fora** desta etapa.

---

## 7. Uma instância de cada nó

1. **Prova principal: o grafo em execução.** Em cada combinação do §10, com
   tudo ativo: `ros2 node list | sort | uniq -d` **vazio**, e a lista de nós
   **igual** a uma lista esperada escrita no roteiro (nem nó a mais, nem a
   menos). Nome duplicado ou nó inesperado reprova.
2. **Complemento: teste de launch com contexto real.** Percorrer
   `generate_launch_description()` **não** resolve condições, `IncludeLaunchDescription`
   nem `OpaqueFunction` — a `pilha` usa as três. O teste roda a descrição num
   `LaunchService`/`LaunchContext` de verdade, com os argumentos de cada
   combinação, e conta os nós que **seriam executados**. Serve para pegar cedo,
   no `pytest`, o que o grafo vivo confirma depois. **Não é a prova.**
3. **Entre launches, nesta etapa:** o robô 3 **não** sobe junto com a pilha (D1)
   nem com o `sim_robo3` (os dois têm RSP).
   ✅ **D5 (aprovada com correção):** o `controle_robo3` só **avisa**; quem
   **recusa** é o `sobe-robo3`. ⚠️ **E hoje o `sobe-robo3` não recusa — ele mata**,
   e o filtro do `vivos()` é por nome genérico (`joy_node`, `teleop_twist_joy`,
   `twist_mux`…): mataria nós do robô 2 ou do simulador que estivessem no mesmo
   PC. Exigência do plano:
   - o `sobe-robo3` guarda o **PID do grupo de processo** (`setsid`) do que ele
     próprio subiu, num arquivo de estado, e o `--mata` só derruba **esse grupo**;
   - **nunca** matar por nome genérico `robot_state_publisher`, `twist_mux`,
     `joy_node` ou `teleop_twist_joy`;
   - ao subir, se houver no domínio `/robot_state_publisher`, `/twist_mux`,
     `/joy_node` ou `/teleop_twist_joy_node` que **não** sejam do grupo dele:
     **recusar**, listar quem são e sair — sem matar nada.

---

## 8. Comandos antigos que continuam funcionando, sem argumento novo

| comando | o que tem de continuar fazendo |
|---|---|
| `ros2 launch robot_motion pilha.launch.py sim:=true` | pilha do robô 2 no Gazebo, idêntica |
| `ros2 launch robot_motion pilha.launch.py mapa:=nenhum` e `localizacao:=amcl …` | idem, robô 2 real |
| `bash bin/sobe-robo`, `sobe-robo slam`, `--mata` | idem |
| `ros2 launch robot_base base.launch.py` (`congela_parado:=…`) | idem |
| `ros2 launch robot_motion joystick.launch.py` | idem |
| `ros2 launch robot_base sim_robo3.launch.py` | simulador do robô 3, idêntico |
| `bash bin/smoke-gazebo-robo3` | idem |
| `ros2 launch robot_nav controle_robo3.launch.py` (`frente:=`, `sinal:=`, `porta:=`…) | mesmo comportamento **+ RSP** |
| `bash bin/sobe-robo3`, `--mata` | mesmo uso; muda por dentro: recusa conflito em vez de matar, e o `--mata` só derruba o próprio grupo de processo (D5) |

Teste: os argumentos declarados de cada launch **antes** da etapa viram lista
travada; nenhum some e nenhum default muda (exceto `robo`, novo, padrão 2).

---

## 9. Ordem de implementação — uma mudança por commit, com o vermelho antes

Trabalho numa **branch própria**, `etapa4-perfis`, criada a partir da `main`.
A `main` só recebe a etapa inteira, depois dos critérios do §10.

| # | mudança | fica vermelho ANTES | prova depois |
|---|---|---|---|
| 0 | **Linha de base, sem mudar código de produção.** Gazebo, `pilha sim:=true` do robô 2, **com o dono acompanhando**: esperar todos os nós da lista explícita (§10.2) ativos; dump de parâmetros de cada um, **normalizado** (§10.2) → `docs/dados/…-baseline-robo2/`. Medir o `footprint_padding` vivo nos dois costmaps. Travar a lista de nós esperados e a de argumentos dos launches (§8). A ferramenta de dump/normalização é o único código deste passo | — | arquivos versionados |
| 1 | `footprint_padding` explícito no `nav2.yaml` do robô 2 = valor medido | `test_footprint_padding_declarado` (robô 2) | dump do Gazebo **igual** à linha de base |
| 2 | `avanco_para_choque` no `path_follower`, default 0,28; `vao_frente()` passa a usá-lo | teste unitário: com avanço ≠ recuo, o vão frontal usa o **avanço** | suíte verde; dump do robô 2 igual + 1 parâmetro novo de valor 0,28 |
| 3 | `perfil.py` + a pilha montando o robô 2 por ele (`robo:=2` padrão) | `test_perfil_robo2_identico_a_linha_de_base` (a função ainda não existe) | dump igual à linha de base |
| 4 | `perfil_robo3.yaml` + ramo `robo:=3` da montagem; `robo:=3` na pilha **recusa** (D1) | testes do §3, §4 e §5 para o robô 3 (footprint = artefato, sem vértice redigitado, padding declarado, (b) calculado, (c) em chave separada) e `test_pilha_recusa_robo3_ate_a_etapa6` | suíte verde |
| 5 | Coerência footprint ↔ reflexo **por perfil**: a regra da 032 continua só no robô 2; o robô 3 exige reflexo ⊇ footprint **e** ≠ footprint | teste novo do robô 3 | o teste do robô 2 inalterado e verde |
| 6a | `sobe-robo3` por grupo de processo próprio: recusa conflito, `--mata` só o próprio grupo (D5) | teste do script contra processos fingidos com os mesmos nomes: hoje ele os **mata** (vermelho); depois, recusa e não mata | nós fingidos do "robô 2" sobrevivem ao `--mata` |
| 6b | RSP no `controle_robo3` + `exec_depend robot_base` no `robot_nav` (D4) | teste de launch com contexto: RSP presente, 1 instância de cada nó, argumentos antigos intactos; teste do `package.xml` | MEGA fingida: TF `base_link → livox_frame` presente; frames para a MEGA **iguais** aos de 14-09 (frente −120, esquerda steer 97, LB solto 0) |
| 7 | Validação final (§10) e registro | — | critérios do §10 |

Cada passo: teste vermelho → mudança → suíte da raiz verde → commit. Nenhum
passo sobe sem o anterior verde.

⚠️ **Fronteira com a etapa 5, explícita:** nenhum passo acima converte `Twist` ↔
`TwistStamped`, cria adaptador, junta os dois mux nem muda `use_stamped` ou
`publish_stamped_twist`. Teste existente (`test_o_mux_fala_stamped_…`) e um novo
para o robô 3 (mux e teleop em `Twist`) travam os dois contratos **como estão**.

---

## 10. Critérios objetivos de conclusão

1. Suíte da raiz verde (`pytest --ignore=ESTAGIO-2026`), com todos os testes
   novos do §9.
2. **Robô 2 no Gazebo, parâmetros:** comparação **semântica**, nunca de texto
   bruto do `ros2 param dump` (ordem de chave, formato de float e parâmetros
   automáticos como `qos_overrides` e `use_sim_time` variam sem significado):
   - por uma **lista explícita de nós** (a do passo 0), só depois de **todos
     ativos** — lifecycle em `active` e nós comuns respondendo;
   - cada dump carregado como YAML e **normalizado**: chaves ordenadas, números
     comparados como número (1 e 1.0 iguais), parâmetros automáticos excluídos
     por uma lista também escrita;
   - resultado **idêntico** à linha de base, exceto as duas diferenças do §1
     (`footprint_padding` explícito, mesmo valor; `avanco_para_choque` 0,28).
     Diff normalizado versionado.
3. **Robô 2 no Gazebo:** uma corrida da pista de obstáculos com o objetivo de
   sempre chega. É smoke, não medida — não prova desempenho.
4. **Robô 3 no Gazebo:** `bin/smoke-gazebo-robo3` 6/6 (com o item 6 já pelo
   serviço).
5. **Robô 3 com MEGA fingida (pty):** bringup sobe; TF `base_link → livox_frame`
   com yaw 0; frames para a MEGA iguais aos de 14-09; `sobe-robo3 --mata` limpo
   **e** nós com os mesmos nomes fora do grupo dele continuam vivos.
6. Em toda combinação acima: `ros2 node list | sort | uniq -d` vazio, e a lista
   de nós igual à esperada.
7. Perfil do robô 3: footprint montado = artefato 052; os dois costmaps
   declaram `footprint_padding`; (b) calculado; (c) em chave própria.
8. Decisão registrada (053), diário e estado atualizados.

🔴 **O que a etapa 4 NÃO prova:** que o robô 3 navega (etapa 6), que o contrato
está unificado (etapa 5), nem nada de hardware. O robô 2 físico **não** é
testado: a igualdade de parâmetros no Gazebo é a prova de que ele não muda.

---

## 11. Reversão

- Tudo na branch `etapa4-perfis`; a `main` fica intocada até o fim.
- Um commit por passo → `git revert <sha>` desfaz um passo sem arrastar os
  outros. Para abandonar a etapa inteira: `git revert` dos commits da etapa
  (ou simplesmente não fundir a branch). **Nada de `reset --hard`** — reverter
  deixa rastro no histórico, e o histórico é parte do registro do PIBIT.
- **Gatilho de parada:** o dump do robô 2 diferir da linha de base em qualquer
  chave fora das duas permitidas → **parar**, preservar o dump e reverter aquele
  passo **antes** de investigar.
- **Nenhum deploy durante a etapa.** O NUC do robô 2 recebe código por push
  manual (`ESTADO_PROJETO`, 15-09), então nada chega lá sozinho — mas o
  precedente da decisão 050 (código não validado entrando por `reset --hard` +
  `sobe-robo`) vale de aviso: até o dono liberar, **não empurrar** a `main` para
  o NUC nem para o notebook do robô 3.

---

## 12. Decisões que dependem do dono

| # | pergunta | decisão do dono (18-09) |
|---|---|---|
| D1 | `robo:=3` na pilha, na etapa 4 | ✅ existe e **recusa** explicitamente até a etapa 6 |
| D2 | `footprint_padding` do robô 3 | ✅ valor vivo do robô 2, nos dois costmaps, política herdada |
| D3 | margens do reflexo, `re_largura`, recuo, avanço, pivô | ✅ **com valores próprios** (§5): Approach +0,03 por face e 0,75 s; Stop 0,133/0,05/0,05; `re_largura` 0,480; recuo 0,2913 (folga no `re_folga`); avanço 0,0825; pivô 0,3325 |
| D4 | bringup | ✅ RSP no `controle_robo3`, **com `exec_depend robot_base`** |
| D5 | duas instâncias entre launches | ✅ launch avisa, `sobe-robo3` **recusa**; **grupo de processo próprio, nunca matar por nome genérico** |

---

## 13. Revisão do dono em 18-09 — o que mudou neste plano

1. §1: eram "uma única diferença"; são **duas** (`footprint_padding` e
   `avanco_para_choque` 0,28).
2. §7: o teste estático não resolve condições, includes nem `OpaqueFunction`.
   A prova principal passou a ser o **grafo em execução**; teste de launch com
   contexto real é complemento.
3. §10.2: comparação de parâmetros **semântica e normalizada**, por lista
   explícita de nós, com todos ativos — não texto bruto de `ros2 param dump`.
4. §11: saiu o `git reset --hard`; branch de trabalho + `git revert`.
5. §5: `PolygonApproach` e `PolygonStop` separados — são duas funções de
   segurança com políticas diferentes, e o "idem" misturava as duas.
6. D5: achado do dono — o `sobe-robo3` de hoje **mata** por nome genérico e
   derrubaria nós do robô 2 ou do simulador. Virou o passo 6a.
7. D4: faltava a dependência `robot_nav → robot_base`.
