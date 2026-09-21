# Baseline v2 do robô 2 no Gazebo — a régua dos passos seguintes da etapa 4

Capturada em 2026-09-21, commit **`52bbffc`**
(`02-baseline-v2-aprovada/commit.txt`), robô físico desligado, pelo roteiro de
sempre (`bash bin/linha-de-base-robo2`, `pilha.launch.py sim:=true` sem outro
argumento). Estado do código: passos 1 e 2 já aplicados.

## Por que existe

A baseline original (`../2026-09-21-baseline-robo2/`) tinha **dois dumps
vazios**, `/collision_monitor` e `/controller_manager`: o `get_parameters` do
rclcpp é tudo-ou-nada, e cada um deles lista parâmetros que não consegue ler. A
régua foi consertada (leitura nome a nome; ilegível e dump vazio reprovam;
exceções exatas e versionadas, e exceção sem uso também reprova).

## As duas capturas

- `01-reprovada-controller-manager/` (commit `146216a`): **REPROVADA**, só por
  quatro ilegíveis inesperados do `/controller_manager` —
  `fallback_controllers` e `node_options_args` do `hoverboard_base_controller`
  e do `joint_state_broadcaster`. Confirmados um a um na pilha viva, só leitura
  (`../2026-09-21-controller-manager-ilegiveis/saida.txt`: 48 nomes, 0 valores
  no lote), e cadastrados em `52bbffc`. Os dois `Polygon*.max_points` do
  `collision_monitor` já eram exceção (`../2026-09-21-collision-monitor-ilegiveis/`).
- `02-baseline-v2-aprovada/` (commit `52bbffc`, mesma pilha de pé): **APROVADA**
  — 30/30 nós, 25 dumps, **nenhum vazio**; seis ilegíveis, todos cadastrados;
  zero inesperados; zero exceções sem uso; padding
  `0.009999999776482582` nos dois costmaps.

## Continuidade (`continuidade.sh` → `continuidade.txt`)

1. **v2 × passo 2**, nos 23 nós antigos, sem permissão: **zero diferença** —
   inclusive no dump bruto.
2. **v2 × baseline original**, nos 23 nós antigos: só a permissão já aprovada
   do passo 2 (`/path_follower:avanco_para_choque`, entrou, 0,28).
3. **`/collision_monitor` (44) e `/controller_manager` (34)**, conteúdo novo
   (parâmetros normalizados): os polígonos Stop e Approach vivos são iguais aos
   do `collision_monitor.yaml`, e o `git diff feb064a..52bbffc` das fontes
   desses nós (config, URDF, `sim.launch.py`, `pilha.launch.py`) é **vazio**.

## Logs e limites

Único erro até a captura: o shader GLSL do RViz de sempre
(`logs/erros_launch.txt`). No desligamento o `ros2 launch` e o `ros2 bag` não
saíram com SIGINT e levaram `kill -9` do wrapper; as cinco linhas `process has
died` do `logs/launch.txt` vêm todas depois do SIGINT. Nenhum processo sobrou.

Prova o conjunto de nós e parâmetros efetivos do robô 2 com os passos 1 e 2.
Não prova desempenho, geometria nem hardware.

**A partir daqui, o passo 3 compara contra ESTA baseline, sem nenhuma
permissão** — o `avanco_para_choque` já está nela.
