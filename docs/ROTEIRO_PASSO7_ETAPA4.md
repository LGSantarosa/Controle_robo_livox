# Roteiro — etapa 4, passo 7: validação final (plano §10)

Tudo no **PC de dev**. **Robô físico desligado**, sem LiDAR, sem MEGA de
verdade. Três cenários **separados**, um por chamada: uma falha num não apaga
a evidência dos outros. O dono só roda; eu leio as pastas.

```
bash bin/valida-etapa4 robo2-gazebo 6.24 3.51
bash bin/valida-etapa4 robo3-gazebo
bash bin/valida-etapa4 mega-fingida
```

Antes de cada um: nada de ROS/Gazebo de pé no PC (o script recusa e não mata
nada se houver). Para o `mega-fingida`: **nenhum controle/joystick
conectado** (`/dev/input/js*`) — o `joy_node` real o leria e misturaria com o
`/joy` fingido. O Gazebo abre janela nos dois primeiros: pode olhar, não
precisa fazer nada.

## O que todo cenário faz, igual

| etapa | o quê | onde fica |
|---|---|---|
| pasta | `~/validacao_etapa4/<data>_<cenário>/` **nova** (recusa se existir). Nada é escrito em `docs/dados/` — as baselines só são lidas | a pasta |
| proveniência | commit, branch e `git status --porcelain`; **árvore suja recusa** | `git.txt` |
| pré-condições | nenhum ROS/Gazebo de pé; nenhum processo marcado de rodada anterior (e, no mega, sem joystick e sem registro do sobe-robo3) | console |
| isolamento | `ROS_DOMAIN_ID` próprio (41 robô 2, 42 robô 3, 43 MEGA, 44 externos), só localhost, `GZ_PARTITION` própria; todo processo lançado depois das pré-condições herda `VALIDA_ETAPA4_MARCA=<pasta>`. As baselines do robô 2 foram capturadas no domínio 0; o domínio não entra nos parâmetros comparados e **não é permissão de diferença** | `ambiente.txt` |
| build | `robot_base robot_motion robot_nav` e `install/` = fonte em todos os `data_files` | `build.log` |
| subida | cada coisa em grupo próprio (`setsid`), PID conferido como PGID, registrado com STARTTIME | `grupos_wrapper.txt`, `comandos.txt`, `*.log` |
| prontidão | `tools/linha_de_base/captura.py`: todos os esperados, nenhum a mais, nenhum duplicado, lifecycle `active`, comuns respondendo, **estável** em 3 consultas — só então lê os parâmetros | `captura*/` (brutos + normalizados, **também se reprovar**) |
| lista de nós | `ros2 node list --no-daemon` cru, `sort \| uniq -d` e comparação com a lista esperada (`lista_nos.py`, mesma regra da captura) | `*_nos.txt`, `*_uniq_d.txt`, `*_lista_nos.yaml` |
| limpeza | instalada em `EXIT` logo depois da marca: roda em TODA saída (build falhou, Ctrl+C, fim). Sinal só pelo `processos.py`: cada grupo registrado é classificado antes (líder com o STARTTIME registrado, ou órfão com TODOS os membros marcados; PGID reusado ou órfão com membro sem marca não recebe nada) e cada PID é reconferido (pgid, starttime, marca) logo antes do kill. INT, 20 s, KILL (anotado); depois quem ainda tiver a marca desta pasta (ex.: o `ros2-daemon`), também reconferido. **Prova**: zero processo marcado e `ros2 node list` com **código 0** e vazio em cada domínio usado — consulta com erro reprova | `limpeza.txt`, `nos_depois_da_limpeza_*` |
| resultado | um item por linha, APROVADO / REPROVADO / ANOTADO; sai 1 se algum reprovar | `resultado.txt`, `console.txt` |

Nunca mata por nome (teste estático em `tools/valida_etapa4/`).

## Cenário 1 — `robo2-gazebo 6.24 3.51` (§10.2, §10.3, §10.6)

1. `ros2 launch robot_motion pilha.launch.py sim:=true` — o comando do §8,
   sem outro argumento.
2. Captura contra `tools/linha_de_base/esperados_robo2_sim.yaml` (a mesma dos
   passos 0–3), prazo 240 s.
3. Lista de nós + `uniq -d`.
4. `compara_robo2.py`, reusando o normalizador sem mudança:
   - **A**: contra a baseline **v2**, sem permissão nenhuma (a v2 já tem as
     duas diferenças do §1);
   - **B**: contra a baseline **original**, nos 23 nós que ela tinha com dump,
     com a permissão cadastrada `permitidas/passo2_robo2.yaml` — que tem de ser
     **usada**;
   - **C**: `footprint_padding` dos dois costmaps = `0.009999999776482582`
     (§1 item 1; entrou escrito no passo 1 sem mudar o dump, por isso não tem
     arquivo de permissão).
5. `tools/banco/corrida_nav.py --alvo 6.24 3.51 --teto-s 120`, sem `--rumo`
   (ação `navigate_to_pose`, cancelável): 🟢 se chegou. O alvo é "o que o
   dono mandou" da decisão 032 e o das corridas aprovadas da 033 — não o
   (6,14 · 3,78), que era o ensaio da porta. **Smoke, não medida.** Roda
   depois da captura para o nó dela não entrar na lista.

## Cenário 2 — `robo3-gazebo` (§10.4, §10.6)

1. `sim_robo3.launch.py` com os argumentos do `smoke-gazebo-robo3` (pista,
   pose 2,0/5,0/0, gui).
2. Captura contra `tools/valida_etapa4/esperados_robo3_sim.yaml` — **previsão**
   pela leitura do launch; os 4 ilegíveis do `controller_manager` são
   **herdados** do robô 2 (mesmo plugin e mesmos nomes de controlador) e, se o
   robô 3 não os mostrar, a captura reprova e a previsão cai.
3. Lista de nós + `uniq -d`.
4. `tools/smoke_gazebo_robo3.py` (o verificador de 18-09): 6/6. Roda depois da
   captura (o nó dele não é oculto).

## Cenário 3 — `mega-fingida` (§10.5, §10.6)

1. **Externos de mesmo nome** no domínio 44: `joy_node`,
   `teleop_twist_joy_node`, `twist_mux`, `robot_state_publisher` — nós ROS de
   verdade (`externo.py`) com o argv que o `sobe-robo3` antigo mataria por
   nome. PID, PGID e STARTTIME em `externos.txt`, `ps` em
   `externos_ps_antes.txt`.
2. **Recusa real**: `sobe-robo3` no domínio 44 tem de sair ≠ 0 listando os 4,
   sem matar e sem registrar (`recusa.txt`).
3. Duas rodadas no domínio 43, cada uma na sua subpasta:

   | rodada | argumentos | por quê |
   |---|---|---|
   | `reproducao_14_09` | `sinal:=-1.0 frente:=1.0 bitola:=0.3225`, eixo de giro 0,375 | as condições efetivas de 14-09 (commit `6dfcdd0`): tem de dar **exatamente** frente −120, esquerda steer 97, solto 0. Prova que a cadeia **atual** reproduz os frames históricos sob condições equivalentes — **não** que ficou inalterada |
   | `padrao_hoje` | nenhum | o `sobe-robo3` puro de hoje: frente −120, esquerda steer **−256**, solto 0 |

   ⚠️ **Por que dois:** com a configuração de hoje a esquerda **não** dá 97 —
   o giro do teleop foi a 4,0 (`b9dcbeb`), o sinal a +1 (`b4d2c39`), a frente
   virou (`3f64638`) e a bitola a 0,320 (`cbb0e86`), tudo por decisão
   registrada. `test_valida_etapa4.py` recalcula as duas tabelas das configs.

   Em cada rodada:
   1. `mega_fingida.py` abre o pty, manda STATE a 50 Hz (placa respondendo,
      36,50 V), publica `/joy` parado e grava **todo byte** recebido
      (`bytes.bin`, `bytes.csv`) e os frames decodificados (`frames.csv`);
   2. `bash bin/sobe-robo3 porta:=<pty> <argumentos>` tem de sair 0;
   3. o registro do sobe-robo3 é copiado (`grupos_sobe_robo3.txt`);
   4. captura (`esperados_robo3_mega.yaml`, previsão) + lista de nós + `uniq -d`;
   5. fases do `/joy`: repouso, LB+frente, solto, LB+esquerda, solto; em cada
      fase com expectativa, **todos** os frames depois de 0,6 s têm de ser o
      frame esperado **byte a byte**; solto = último frame zero e nenhum
      não-zero. Junto, a **TF viva** `base_link → livox_frame`: xyz e
      roll/pitch/yaw contra a junta do URDF instalado, **yaw 0** (`mega.yaml`);
   6. foto de todos os processos (`antes_mata.csv`), `sobe-robo3 --mata`,
      foto de novo (`depois_mata.csv`) e `processos.py compara`, **grupo a
      grupo**: cada grupo registrado tinha identidade na foto de antes (líder
      com o STARTTIME registrado, ou órfão com todos os membros marcados),
      tinha membros, todos marcados, e todos morreram; **nenhum outro processo
      morreu; os externos continuam vivos com o MESMO starttime**
      (`quem_morreu.yaml`);
   7. os 4 externos ainda no grafo do domínio 44; domínio 43 vazio; registro
      removido; a MEGA fingida (fora do grupo do sobe-robo3) sobreviveu.
4. `externos_ps_depois.txt`, e a limpeza geral.

O bag de cada rodada fica em `~/bancada_robo3/controle_<data>/`, como sempre
com o sobe-robo3.

## Depois das três rodadas

A infraestrutura é commitada ANTES da primeira execução (o script recusa
árvore suja) — esse commit prepara o passo 7, não o fecha. Depois das três,
a evidência relevante vai para `docs/dados/2026-09-22-passo7-*/`, inclusive o
**diff normalizado** do robô 2 exigido pelo §10.2 (`comparacao.yaml`).

## O que isto NÃO prova

Nada de hardware: nem a placa, nem o `frente:=-1.0` no chão, nem a MEGA de
verdade. O Gazebo usa os números que nós demos. A corrida do robô 2 é smoke.
A TF viva prova a árvore fixa do RSP; rodas e bobas seguem sem TF dinâmica
(sem `/joint_states`, limite registrado no 6b).
