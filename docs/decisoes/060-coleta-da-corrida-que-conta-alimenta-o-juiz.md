# 060 — A coleta da corrida que conta alimenta o juiz

**Data**: 2026-09-25 (PC de dev; robô, lidar e Gazebo desligados)
**Status**: proposta; escrita antes dos testes vermelhos e do código
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
