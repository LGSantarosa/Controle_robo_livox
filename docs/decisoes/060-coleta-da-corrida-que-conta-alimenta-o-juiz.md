# 060 — A coleta da corrida que conta alimenta o juiz

**Data**: 2026-09-25 (PC de dev; robô, lidar e Gazebo desligados)
**Status**: aplicada — a corrida que conta rodou e fechou o passo 7 em 2026-09-28 (§6.6); escrita antes dos testes vermelhos e do código
**Toca**: um wrapper novo `bin/valida-etapa7`, um montador puro e um extrator
de bag em `tools/valida_etapa7/`, e os testes deles
**Não toca**: `tools/valida_etapa7/julga.py` (contrato fechado, 124/0),
`bin/valida-etapa6`, `bin/explora-objetivo-robo3`, pilha, launch, navegação ou
atuador

---

## 1. Contexto

O juiz do passo 7 (`julga.py`, `avalia(evidencia)`) está verde nos 124
contratos offline, mas é uma função pura: ninguém ainda monta a evidência que
ele recebe a partir de uma corrida de verdade. O passo 7 continua **aberto**
até uma corrida ser julgada pelos três critérios do §4.6/§4.6.1 do
`docs/PLANO_ETAPA6_ROBO3.md`.

O que já se sabe, conferido na corrida exploratória `20260924_160148`
(`~/etapa7-explora/`, neste PC):

- o bag foi gravado em **tempo simulado** (`starting_time` 3 000 000 ns) e já
  contém `/hoverboard_base_controller/cmd_vel` (`TwistStamped`, 146
  mensagens), `/cmd_vel_bruto` e `/clock`;
- o bag **não** contém `/navigate_to_pose/_action/status`: é tópico oculto, e o
  `ros2 bag record` só o grava com `--include-hidden-topics`. Sem ele não há
  janela no relógio simulado;
- o `objetivo.log` do `send_goal` traz o UUID (`Goal accepted with ID:
  9306d6c3…`) e a linha final (`Goal finished with status: SUCCEEDED`);
- não houve `ros2 param dump /placa_simulada`, nem leitura viva da tolerância,
  nem `ros2 topic info -v` do tópico final.

Ou seja: das sete entradas do juiz, só as amostras e as poses já têm fonte.
Janela, resultado estruturado, placa, tolerância viva e grafo não têm.

## 2. Decisão

### 2.1 Wrapper novo, e os dois antigos congelados

Nasce `bin/valida-etapa7`: **headless**, derivado do explora sem janela
gráfica, reutilizando `tools/valida_etapa6/lib.sh` (pré-condições, subida em
grupo próprio, encerramento dirigido do gravador por SIGTERM, limpeza sem
`pkill`). Termina chamando montador → `avalia` → `resultado.csv`.

`bin/valida-etapa6` e `bin/explora-objetivo-robo3` ficam **congelados**. O
primeiro é a prova do passo 6 e diz no próprio cabeçalho "aqui NÃO se manda
objetivo nenhum"; o segundo é exploratório e mede pelo olho do dono.

⚠️ **Divergência do plano.** A tabela do §7 do `PLANO_ETAPA6_ROBO3.md`, passo 7,
diz que a prova do objetivo curto é "a mesma pasta" da etapa 6. Esta decisão
troca isso por uma pasta própria da etapa 7, pelo motivo acima: estender o
`valida-etapa6` mudaria o que a evidência do passo 6, já fechado, significa.

**Quem grava o bag é o `bin/valida-etapa7`, não a launch.** Para a
`pilha.launch.py` e os wrappers antigos ficarem intocados:

1. a pilha sobe com `bag:=false` (o gravador da launch não roda);
2. o wrapper inicia o **próprio** `ros2 bag record --all-topics
   --include-hidden-topics --use-sim-time`;
3. antes do goal, espera esse gravador aparecer como **assinante** de
   `/navigate_to_pose/_action/status` e de
   `/hoverboard_base_controller/cmd_vel` — gravador que ainda não assinou
   perderia o começo da janela;
4. no fim, encerra esse gravador pelo mecanismo dirigido já provado (SIGTERM,
   espera do `metadata.yaml`).

### 2.2 A ação e a janela saem do status da ação, correlacionado pelo UUID

O gravador do §2.1 grava `/navigate_to_pose/_action/status`, tópico oculto,
por causa do `--include-hidden-topics`. A correlação é pelo **UUID do goal**, que o
`objetivo.log` identifica:

| campo | fonte canônica |
|---|---|
| UUID | a linha `Goal accepted with ID:` do `objetivo.log`, exatamente uma |
| `janela.objetivo_aceito` | `goal_info.stamp` desse UUID no status gravado |
| `janela.resultado` | **instante de gravação** da primeira ocorrência **terminal** desse UUID (SUCCEEDED, CANCELED ou ABORTED) |
| `resultado_acao` | esse mesmo status terminal estruturado |

O `objetivo.log` serve para **identificar** o UUID e para **conferência
cruzada** (`Goal finished with status:` tem de dizer o mesmo terminal); não é
fonte canônica do resultado.

Reprovam a coleta, sem escolha silenciosa:

- UUID ausente no `objetivo.log`, ou mais de uma linha de aceite;
- UUID do log que não aparece no status gravado;
- outro UUID no status durante a corrida (goals concorrentes);
- `goal_info.stamp` que muda para o mesmo UUID;
- cronologia inválida: `goal_info.stamp` e o instante do primeiro terminal têm
  de ser tempos ROS válidos e satisfazer aceite ≤ terminal; aceite posterior ao
  primeiro terminal reprova;
- transição inconsistente: status terminal seguido de não terminal, ou dois
  terminais diferentes para o mesmo UUID;
- nenhum terminal gravado;
- terminal do status divergente da linha final do `objetivo.log`.

### 2.3 Tempo: instante de gravação, em nanossegundos até o fim

O `t` de cada amostra é o **instante de gravação no bag**, em tempo simulado,
o mesmo relógio do fim da janela. O `header.stamp` da `TwistStamped` fica
**apenas como diagnóstico**, por um motivo conferido no código: a placa faz
`fora.header = msg.header` (`placa_simulada.py:431`), então o carimbo é
**herdado do comando de entrada** e não marca a saída depois da latência. Ele
diria quando o comando foi pedido, não quando chegou ao consumidor final.

Todos os tempos (gravação, `goal_info.stamp`) são carregados como **inteiros
em nanossegundos**; a conversão para segundos acontece uma única vez, na
montagem final da evidência que vai ao juiz.

### 2.4 Tolerância viva, pelo id que o próprio controller declara

1. ler `goal_checker_plugins` do `/controller_server` vivo;
2. exigir **exatamente um** id; com zero ou mais de um, reprova — não se sabe
   qual tolerância valeu;
3. ler `<id>.xy_goal_tolerance` do mesmo nó vivo.

O nome `goal_checker` **não** é redigitado nem suposto. O `perfil_nav2.yaml`
materializado da corrida é **conferência cruzada**: se o valor dele no mesmo
caminho divergir do vivo, **a coleta reprova**; o juiz continua usando o valor
vivo.

### 2.5 Placa e grafo, lidos com a pilha de pé

- `ros2 param dump /placa_simulada`: `modelo`, `deadband_speed`,
  `escala_real`, `raio`, `bitola`. Campo ausente fica ausente — o juiz reprova
  com o nome do que faltou.
- `ros2 topic info -v /hoverboard_base_controller/cmd_vel`: publicadores e
  assinantes. Capturado **antes do goal e depois do resultado**, antes do
  teardown; as duas capturas têm de ser iguais, senão a coleta reprova.
  As capturas cercam o objetivo e precisam coincidir; isso prova a topologia
  nas duas bordas, **não** exclui mudança transitória entre elas. O domínio
  isolado e o conjunto fixo de processos reduzem essa lacuna.

### 2.6 Onde mora cada coisa, e como a falha de coleta aparece

- `tools/valida_etapa7/monta.py` — **puro**: recebe os brutos (texto do param
  dump, do topic info, do `objetivo.log`, poses, tolerância viva e
  materializada, e as linhas já extraídas do bag) e devolve a evidência e a
  lista de falhas de coleta. Fonte que falhou deixa o campo **ausente**, nunca
  preenchido por outra fonte.
- `tools/valida_etapa7/le_bag.py` — extrator com `rosbag2_py` (ROS carregado):
  amostras do tópico final e do bruto (tempo de gravação e `header.stamp`, em
  ns) e o status da ação.
- No `resultado.csv`, cada falha de coleta é um item próprio, ao lado dos sete
  do juiz. O passo 7 só fecha com **todos** aprovados.

## 3. Alternativas descartadas

| alternativa | por que não |
|---|---|
| estender o `bin/valida-etapa6` | muda o significado da prova de um passo já fechado |
| gravar pelo `bag:=true` da launch | exigiria mexer na `pilha.launch.py` para incluir tópicos ocultos, e ela é compartilhada com os wrappers congelados |
| estender o `bin/explora-objetivo-robo3` | é exploratório por contrato, com janela gráfica que muda carga e grafo |
| janela pelo `/behavior_tree_log` | é log do BT, não o contrato da ação; não carrega o UUID |
| resultado pela linha do `objetivo.log` | texto de console; vale só como conferência cruzada |
| janela por `/clock` lido pelo wrapper no aceite e no fim | amostragem de fora, com latência de parede; o mesmo erro que o `d52b5da` removeu da coleta do trajeto |
| `t` pelo `header.stamp` | herdado do comando de entrada (`fora.header = msg.header`), não marca a saída; fica como diagnóstico |
| tempos em `float` de segundos desde a leitura | perde a igualdade exata nas bordas da janela fechada |
| nome `goal_checker` fixo, ou a tolerância do YAML materializado | redigitar; e o contrato pede o valor **vivo** |
| YAML ou `objetivo.log` antigos como fallback quando a fonte nova falta | provaria uma fonte diferente da escolhida |

## 4. Sequência, uma mudança por vez

1. **Esta decisão**, revisada antes de qualquer teste.
2. **Testes vermelhos do montador**, com brutos sintéticos: as regras do
   UUID/terminal (§2.2), os tempos em ns (§2.3), o id único do goal checker e a
   conferência com o materializado (§2.4), as duas capturas do grafo (§2.5), e
   "fonte que falha deixa campo ausente".
3. **O montador verde.**
4. **O extrator do bag**, testado contra um bag pequeno que o próprio teste
   escreve, incluindo o status oculto e os instantes de gravação.
5. **Prova offline contra o bag real de `20260924_160148`.** Ela prova
   **apenas** a extração: as **146** amostras do tópico final, conferidas com o
   `metadata.yaml`. Ela **não** pode aprovar SUCESSO, TOLERÂNCIA nem
   NASCEU_FORA pelo caminho novo: faltam o status oculto, os parâmetros vivos e
   o grafo, e o YAML e o `objetivo.log` antigos **não** entram como fallback.
6. **`bin/valida-etapa7`**, headless, só `bash -n`; nada executado.
7. Só então o pedido de "pode" para a corrida no Gazebo.

## 5. O que esta decisão não prova

Nada aqui foi executado. A corrida que conta ainda não existe, e o critério do
patamar continua valendo **só para a placa simulada herdada do robô 2** — não
mede a zona morta real do robô 3. O `d52b5da` (trajeto pelo CSV do seguidor)
segue não executado e não entra em nenhum dos três critérios.

## 6. Adendo de 2026-09-28 — a revisão do `b11f966`

O `b11f966` (wrapper + `corrida.py`) foi enviado em 25-09 **sem revisão**. A
revisão de 28-09 achou quatro defeitos, dois capazes de **falsa aprovação**,
e a correção de dois deles abriu uma lacuna cada, também fechada. Tudo
offline: nenhum ROS, Gazebo ou robô. Cada correção teve teste vermelho antes
do código e mutação conferida depois.

Esta seção **amplia o "Não toca"** do cabeçalho: o `julga.py` ganhou um item
(ponto A), e o `resultado.csv` passa a ter **13** itens do julgamento
(5 de coleta + **8** do juiz), não 12 como no §2.6.

### 6.1 Ponto A — o teto de 60 s simulados é do juiz, em ns inteiros

**Achado.** O juiz só conferia que a janela não estava invertida: uma ação
`SUCCEEDED` aos 61 s aprovava. O wrapper cortava por um `/clock` lido em
segundos inteiros **antes** da pose e do envio, e só anotava (`ANOTADO`).

**Correção** (`417dbff`, `5018cb9`, `9c5d64b`, `389e075`, `b6069d5`):

- o montador calcula `duracao_objetivo_ns = terminal_ns − aceite_ns`, dos
  **mesmos dois carimbos** da janela (o `goal_info.stamp` do UUID e o primeiro
  terminal); a janela em segundos segue servindo só ao recorte das amostras;
- o juiz ganhou o item **`7.1 a ação fechou em até 60 s simulados`**, com
  `TETO_SIMULADO = 60` e comparação **exata** em inteiros: 60 s aprova,
  **60 s + 1 ns reprova**. Float, bool, texto ou negativo reprovam pelo tipo;
- o wrapper **não corta mais** por relógio simulado: `relogio()`, `T0_SIM` e o
  item do `/clock` saíram. Fica só o **watchdog de parede** (6 × 60 = 360 s),
  que reprova pelo próprio motivo, sem atribuir causa ao `/clock`. Uma trava
  prende o `TETO_SIMULADO=60` do wrapper ao do juiz.

**Descartadas:**

| alternativa | por que não |
|---|---|
| comparar em segundos com folga de 1e-9 | em segundos, 60 s exatos pelo montador dão `60.00000000000001`; a folga para isso aprovaria 60 s + 1 ns, e a fonte já é inteira |
| o juiz receber só ns (sem janela em s) | a janela em s segue útil ao recorte das amostras; as duas vêm dos mesmos carimbos |
| corte simulado no wrapper, reprovando | o relógio lido antes do goal cortaria em 60 s do **marco** uma ação de 59 s aceita tarde; e mesmo alinhado, um terminal em 60 s exatos deixa o cliente vivo por um instante e o corte venceria |
| ler o aceite ao vivo para alinhar o corte | o `send_goal` não imprime o `goal_info.stamp`; assinar o status oculto ao vivo é mais uma peça, com a mesma corrida no fim |

**Custo aceito:** uma ação acima de 60 s agora roda até fechar ou até o
watchdog (até ~6 min a mais); o juiz reprova do mesmo jeito.

### 6.2 Ponto B — o alvo enviado é o alvo julgado

**Achado.** O `poses.yaml` guardava o alvo completo, mas o wrapper enviava o
texto `%.4f`: o juiz media a pose final contra um alvo que não foi o enviado
(até ~7e-5 m).

**Correção** (`cb49406`, `9a4f370`): `corrida.escalar_yaml()` imprime x e y do
alvo como o **escalar YAML** do float — o mesmo texto que o `yaml.safe_dump`
escreve no `poses.yaml`, byte a byte, e que o `ros2 action send_goal` relê
(`yaml.safe_load`, `send_goal.py:119`) como o **mesmo double**.

**Descartada:** `repr`. No YAML 1.1 do PyYAML, `1e-05` (sem ponto) é lido como
**texto**, e alvo perto de zero é realista (yaw π/2 a partir da origem dá
x ≈ 1e-16). O quatérnio segue em 6 casas: só vai para o goal, não entra no
julgamento (que é em xy).

### 6.3 Ponto C — o julgamento falha fechado

**Achado.** O wrapper ignorava o RC do `corrida.py julga` e só reprovava TSV
vazio; saída parcial, item trocado ou veredito com outro nome escapavam do
`grep` final.

**Correção** (`58dc1b2`, `5d67c20`): `corrida.confere_julgamento(texto, rc)`,
pura, e o subcomando `confere`. Aceita só RC `0`/`1`; TSV terminado em quebra,
três campos por linha; vereditos `APROVADO`/`REPROVADO`; o **conjunto exato**
dos 13 nomes, sem repetição; e RC 0 se e só se todos aprovam. Passou: o wrapper
anota `julgamento conferido APROVADO` e as linhas **conferidas**. Não passou
(ou o próprio `confere` quebrou): **uma** linha conhecida
`julgamento conferido REPROVADO <motivo>`, e nada do TSV chega ao `anota`.

**Descartada:** contar 13 linhas — deixaria trocar um item por outro.

### 6.4 Ponto D — o manifesto no padrão do `valida-etapa6`, e o escopo conferido

**Achado.** O `SHA256SUMS` assinava o `console.txt`, que o `tee` continuava
escrevendo depois (o veredito final): o manifesto nascia inválido. Não havia
`sha256sum -c`, e falha do manifesto não mudava o RC. É o defeito já
consertado em 24-09 no `bin/valida-etapa6`.

**Correção** (`0f4b6a1`, `cd5c200`, `616fc09`, `80df9f6`):

- escopo declarado em `escopo_do_manifesto.txt`, **sem** `console.txt`, e
  assinando a si mesmo; construído com `pipefail` (um `find`/`sort` que morre no
  meio deixaria escopo parcial e um manifesto que fecha contra ele);
- o escopo é **conferido contra os artefatos de prova** antes de assinar —
  resultado (txt e csv), `poses.yaml`, `objetivo.log`, julgamentos bruto e
  conferido, os dois dumps de parâmetros, os dois grafos, **exatamente um**
  `corrida_*/perfil_nav2.yaml`, `bag/metadata.yaml` e ao menos um
  `bag/*.mcap` —, registrado no último `anota`;
- o manifesto é gerado e conferido com `sha256sum -c`; falha em qualquer um
  força RC 1. Depois dele, só `echo` para o console. Pasta reprovada também é
  assinada e conferida: a evidência que existe fica preservada.

**Fora do escopo, registrado:** as saídas antecipadas (`exit 1` antes do goal)
continuam **sem manifesto**, como no `valida-etapa6`.

### 6.5 Estado depois do adendo

`tools/valida_etapa7` **281/0**; suíte da raiz **1670/0**. O
`bin/valida-etapa7` **continua sem ter rodado**; nada disto prova a corrida. A
próxima etapa é uma revisão offline única dos commits deste adendo e, sem
bloqueador capaz de falsa aprovação, o pedido de "pode" para o Gazebo.

### 6.6 A corrida que conta (2026-09-28, tarde) — passo 7 FECHADO

Código em **`81bf0dc`**, árvore limpa (`git.txt` da pasta). Gazebo headless
neste PC, domínio 49; robô e lidar **desligados**. Antes de cada corrida e
depois da última: nenhum processo ROS/Gazebo, nenhuma marca `VALIDA_ETAPA*`,
nenhum segmento DDS em `/dev/shm`, domínio 49 sem nós. As pastas ficam em
`~/etapa7/` (ESTA MÁQUINA; não vêm pelo git).

| corrida | resultado | papel |
|---|---|---|
| `20260928_114420` | REPROVADO no **build**, nada subiu | falha **pré-execução**: `build/robot_base/config/MID360_config.json` era um symlink de `--symlink-install` para o arquivo que a 058 renomeou para `MID360_config.template.json`; o `glob('config/*')` do `setup.py` pegou o link pendurado. Resto de build desta máquina, não defeito do código |
| `20260928_114620` | **APROVADO nos 13 itens, mas NÃO canônica** | na subida, `Switch controller timed out after 5 seconds!` e o spawner do `joint_state_broadcaster` morreu com código 1: `/joint_states` com **0** mensagens. Não toca os três critérios (odometria e `odom→base_link` vêm do `diff_drive`), mas a pilha não subiu nominal. Primeira vez em 8 corridas |
| **`20260928_114902`** | **APROVADO nos 13 itens — CANÔNICA** | subida nominal, `/joint_states` com **257** mensagens (`bag/metadata.yaml:559`, arquivo assinado) |

Entre a primeira e a segunda, só o symlink pendurado foi apagado (era o único:
`find build -xtype l`). Nenhum código mudou entre as três.

**A canônica, `20260928_114902`:**

- `SUCCEEDED` em **3 342 000 000 ns** simulados (teto 60 000 000 000 ns);
- pose final a **0,0906 m** do alvo, tolerância viva **0,2500 m**; partida a
  1,0000 m (o objetivo não nasceu dentro);
- **65** amostras do tópico final na janela; maior comando efetivo
  **0,4268 m/s**, acima do patamar vivo **0,3069 m/s**, em
  `/hoverboard_base_controller/cmd_vel`, publicado por `/placa_simulada` no
  modelo `medido`;
- alvo enviado e gravado idênticos (`3.000000000000001`, `5.0`);
- escopo com os artefatos de prova, **41** arquivos; `SHA256SUMS` conferido
  pelo wrapper e de novo **de fora**, depois do fim;
- os três pontos que só a corrida mostrava passaram: status oculto visto pelo
  `topic info -v`, gravador `/rosbag2_recorder`, `param dump` com
  `/placa_simulada` no topo;
- `launch.log`: os **cinco `exit code 1` conhecidos da 057**
  (`compensador_rumo`, `heading_controller`, `placa_simulada`,
  `path_follower`, `freeze_capture`), **só no teardown**; nenhum erro antes
  dele. O `collision_monitor` não caiu com SIGSEGV nesta nem na `114620`.

**Dívida nova, separada — ao lado da 057, não dentro dela:** a ativação do
`joint_state_broadcaster` pode estourar o timeout de 5 s do
`controller_manager` na subida (1 em 8). Decisão do dono: **não** vira
verificação no wrapper agora — `/joint_states` não pertence aos três critérios,
e a corrida seguinte, nominal, tirou a ambiguidade do fechamento. Fica
registrada para quando a subida for gate.

**Alerta para outras máquinas:** notebook e NUC com `build/` anterior à 058
batem no mesmo symlink pendurado no primeiro build. Conferir com
`find build -xtype l` e apagar o link (é artefato de build, fora do git).

🔴 **O que isto NÃO prova:** o patamar é o da **placa simulada herdada do robô
2**; nada aqui mede a zona morta real do robô 3, e é simulador (sem derrapada,
sem atuador físico).
