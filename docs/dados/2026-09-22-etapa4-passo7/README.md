# Etapa 4, passo 7 — validação final (plano §10), 22-09, PC de dev

Sete pastas, copiadas **inteiras e sem edição** de `~/validacao_etapa4/`
(`diff -rq` vazio contra a origem na cópia; `SHA256SUMS` cobre os 241
arquivos: `sha256sum -c SHA256SUMS`). Os `*.log` entram com `git add -f`
mantendo o nome, porque o `comandos.txt` de cada pasta os cita assim. Robô
físico desligado; nada de hardware. Os bags do `sobe-robo3` em
`~/bancada_robo3/` **não** foram copiados.

Roteiro e ferramentas: `docs/ROTEIRO_PASSO7_ETAPA4.md`, `bin/valida-etapa4`,
`tools/valida_etapa4/`.

| pasta | commit | papel | veredito |
|---|---|---|---|
| `20260922_125907_robo2-gazebo` | `cba7639` | robô 2 no Gazebo (§10.2, §10.3, §10.6) | ✅ APROVADO |
| `20260922_130525_robo3-gazebo` | `cba7639` | robô 3 no Gazebo — a previsão da lista de nós falhou | 🔴 REPROVADO (preservado) |
| `20260922_131343_scan2d-isolado` | `cba7639` | atribuição isolada do nó a mais | ✅ confirmado |
| `20260922_131814_robo3-gazebo` | `e3c69f4` | robô 3 no Gazebo, expectativa corrigida (§10.4, §10.6) | ✅ APROVADO |
| `20260922_132801_mega-fingida` | `b0121b7` | MEGA fingida — o defeito do daemon; interrompida | 🔴 REPROVADO (preservado) |
| `20260922_133840_registro-obsoleto` | `4d640f4`¹ | descarte do registro que a 132801 deixou | ✅ sem sinal |
| `20260922_133847_mega-fingida` | `4d640f4` | MEGA fingida corrigida (§10.5, §10.6) | ✅ APROVADO, 24/24 |

¹ Esta minicaptura não tem `git.txt`: o `4d640f4` vem da sequência registrada
(commit do conserto às 13:38:32, a captura às 13:38:40, a rodada seguinte
`133847` gravou `4d640f4` no `git.txt` dela). Não está gravado dentro desta
pasta.

Em todas: `ROS_DOMAIN_ID` próprio (41 robô 2, 42 robô 3, 43 MEGA, 44
externos, 45 isolado), só localhost, `GZ_PARTITION` própria. As baselines do
robô 2 foram capturadas no domínio 0; o domínio não entra nos parâmetros
comparados e **não é permissão de diferença**.

## Robô 2 — `20260922_125907_robo2-gazebo`

- captura: 30/30 nós, 25 dumps, nenhum vazio, só os 6 ilegíveis conhecidos;
  lista de nós = esperada, `uniq -d` vazio;
- **`comparacao.yaml` é o diff normalizado do §10.2**: (A) contra a baseline
  v2 **sem permissão** — nenhuma diferença; (B) contra a original, nos 23 nós
  com dump, com `permitidas/passo2_robo2.yaml` — só
  `/path_follower:avanco_para_choque` entrou 0,28, permissão usada; (C)
  `footprint_padding` = `0.009999999776482582` nos dois costmaps;
- corrida até (6,24 · 3,51), sem `--rumo` (o alvo das decisões 032/033):
  chegou a 0,133 m; **primeira amostra no raio em 29,55 s** (o "120,0 s" do
  `corrida.txt` é o teto — o coletor não encerra na chegada, dívida conhecida).
  **Smoke, não medida.** Dois `Costmap timed out waiting for update` durante a
  corrida (~14,2 s e ~16,6 s), com recuperação; o Nav2 declarou sucesso em
  29,51 s — transientes observados;
- `erros_launch.txt` tem as 3 linhas **anteriores à limpeza** (shader do RViz
  e os dois timeouts). Depois do INT, o `launch.log` ganhou 5 `process has
  died` (heading_controller, placa_simulada, path_follower, compensador_rumo,
  freeze_capture, código 1): efeitos da desmontagem;
- limpeza: só o grupo registrado (`pilha`); KILL em 2 membros que não saíram
  com INT em 20 s — o líder `ros2 launch` e o `[ros2-21]` (PID 838243), o
  gravador do rosbag2. 0 marcados, domínio 41 vazio.

## Robô 3 no Gazebo — a previsão falsificada e a correção

- **`130525` (reprovado):** a lista esperada era previsão escrita antes;
  sobrou `/transform_listener_impl_5a6bfed37c90`, sem serviços de parâmetro.
  O resto passou: os 4 ilegíveis do `controller_manager` **herdados do robô 2
  se confirmaram**, smoke 6/6, 0 ERROR/FATAL, limpeza aprovada;
- **`131343` (atribuição):** só `scan_2d.launch.py`, domínio 45, sem Gazebo:
  3/3 consultas com código 0 → exatamente `/scan_2d` + **uma** ocorrência de
  `^/transform_listener_impl_[0-9a-f]+$`; limpeza com domínio vazio. Base de
  código: `pointcloud_to_laserscan_node.hpp:82` (`tf2_listener_`) e o formato
  `transform_listener_impl_%zx` no `libtf2_ros.so`;
- **`131814` (aprovado):** família com cardinalidade 1 no
  `esperados_robo3_sim.yaml` (`e3c69f4`), testada (1 aprova; 0 e 2
  reprovam). Captura, lista, smoke 6/6 e limpeza aprovados;
- nas duas rodadas, `erros_launch.txt` tinha **zero linhas antes da
  limpeza**; o `launch.log` final ganhou um `process has died` da
  `placa_simulada`, código 1, **durante a desmontagem** — não invalida os
  cenários.

Consequência documental no robô 2 (`5e630a3`, só comentário, contagem 4
inalterada): dos 4 listeners dele, um é do `scan_2d`; os outros três, do
Nav2, **não atribuídos individualmente**.

## MEGA fingida — o defeito do daemon e a rodada corrigida

- **`132801` (defeito, interrompida):** o wrapper viu os 4 externos com
  `ros2 node list --no-daemon --spin-time 5`; logo depois o `sobe-robo3`
  consultou **pelo daemon recém-nascido**, recebeu vazio com código 0, disse
  "conflitos: nenhum" e subiu a pilha real (`porta:=/dev/null`) por cima —
  nomes duplicados no domínio 44. A limpeza recolheu tudo pela marca (pilha,
  bag, `ros2-daemon` do domínio 44): 0 marcados, domínios vazios. Os testes
  anteriores cobriam erro e travamento da consulta, **não a resposta vazia
  com código 0 durante descoberta incompleta**. Conserto `4d640f4`: `timeout
  10 ros2 node list --no-daemon --spin-time 5`, teste vermelho antes, e o
  wrapper passa a encerrar o cenário se a recusa ou a subida falham;
- **`133840`:** o registro que ela deixou (grupos 970569/971265, 0 membros
  vivos) descartado pelo `sobe-robo3 --mata` **sem sinal nenhum**;
- **`133847` (aprovado, 24/24):**
  - recusa real no domínio 44: listou os 4, não matou, não registrou;
  - `reproducao_14_09` (`sinal:=-1.0 frente:=1.0 bitola:=0.3225`, eixo de giro
    0,375): frente 49/49 = `speed −120`, esquerda 48/48 = `steer 97`, solto =
    zero — a cadeia **atual** reproduz os frames de 14-09 sob condições
    efetivas equivalentes, **não** "a cadeia não mudou";
  - `padrao_hoje`: frente 48/48 = −120, esquerda 48/48 = `steer −256`, solto
    = zero;
  - TF viva `base_link → livox_frame` nas duas: xyz (−0,093; 0; 0,24), yaw
    0,0°, diferença 0 para a junta do URDF;
  - em cada rodada: `--mata` limpo; `quem_morreu.yaml` grupo a grupo
    (identidade, membros marcados, todos mortos; nenhum outro processo
    morreu; externos vivos com o mesmo starttime); externos ainda no grafo;
    domínio 43 vazio; registro removido; a MEGA fingida (fora do grupo)
    sobreviveu;
  - **daemon recolhido na limpeza final:** um `ros2-daemon` do domínio 43, com
    a marca da rodada, veio de algum comando do ROS CLI posterior à consulta
    de conflitos (a origem exata não é determinável pela evidência); a
    segunda camada o sinalizou; depois, 0 marcados e domínios 43/44 vazios.
    O `--mata` não mata daemon, que pode ser compartilhado — é o contrato;
  - anotados sem explicação: o `/joy` fingido medido a 9,8 Hz na primeira
    rodada e 20,0 Hz na segunda; e a mensagem do launch "caindo no js0
    (mouce-library-fake-mouse)" engana (o `js0` é mouse virtual, sem
    `ID_INPUT_JOYSTICK`; registrado no `ambiente.txt`).

## O que isto NÃO prova

Nada de hardware: nem a placa, nem o `frente:=-1.0` no chão, nem a MEGA de
verdade. O Gazebo usa os números que nós demos. A corrida do robô 2 é smoke.
A TF viva prova a árvore FIXA do RSP; rodas e bobas seguem sem TF dinâmica
(sem `/joint_states`, limite do 6b).
