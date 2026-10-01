# Revisão de código do robô 2 — 2026-10-01 (fase 1, só leitura)

> Pedido do dono: revisar o projeto do **robô 2**, achar erros de código, de
> lógica e melhorias, e apontar o lixo velho. **Tudo do robô 3 sai deste
> repo**, porque ele agora tem repositório próprio. Esta é a **fase 1**: nada foi
> alterado no código. A fase 2 (limpeza e consertos) será feita item por item,
> com o ok do dono em cada um. Este documento também vai para revisão cruzada
> do Codex: a seção 8 diz o que mais vale contestar.

## 0. Escopo e método

**Lido linha a linha (a pilha que roda hoje no robô 2):**
`robot_motion/launch/pilha.launch.py`, `path_follower.py`,
`lei_de_seguimento.py`, `heading_controller.py`, `lei_de_rumo.py`,
`lei_de_pivo.py`, `lei_de_freio.py`, `compensador_rumo.py`, `perfil.py`,
`joystick.launch.py`, as configs `nav2.yaml`, `nav2_sem_mapa.yaml`,
`collision_monitor.yaml`, `localizacao_amcl.yaml`, `movimentacao*.yaml`,
`twist_mux.yaml`, `teleop_xbox.yaml` e a árvore
`replanejamento_com_suavizacao.xml`. No `robot_base`: `base.launch.py`,
`tracao.launch.py`, `localizacao.launch.py`, `scan_2d.launch.py` e
`scan_2d.yaml`, além de `tf_odom.py`, `nuvem_pontos.py`, `nuvem.py`,
`congela_parado.py` e `transformadas.py`. Também lidos o
`hoverboard_driver` (C++ e `hoverboard_controllers.yaml`), o
`robot_nav/freeze_capture.py` (único nó do `robot_nav` que a pilha sobe) e o
`bin/sobe-robo`. No `controle_web/`: `app.py`, `map_service.py`,
`controllers/robot_controller.py` e `nav_metrics.py`, mais trechos de
`power_monitor.py`, `camera_service.py` e do JS.

**Só inventariado (origem e uso, sem leitura linha a linha):** `robot_nav`
(exceto `freeze_capture`), `robot_planning`, `firmware/`, `tools/`, `bin/`,
`scripts/`, os arquivos da raiz e `docs/`.

**Não revisado:** `lei_de_reta.py` e `lei_de_freio_linear.py` por dentro (só a
interface), `placa_simulada.py`, `sim.launch.py`, `mapeia.launch.py` por
dentro, os testes, o `twist_mux` vendorizado e o JS além do E-STOP e do
clique-para-ir.

**Estado da suíte:** `pytest` na raiz deu **1262 passed** em 114 s
(2026-10-01, PC de dev). Nenhum achado abaixo é pego por teste existente.

**Legenda de confiança:**
- **[confirmado]**: provado lendo o código; o caminho de execução está
  descrito.
- **[verificar]**: o código indica o defeito, mas o efeito depende de
  geometria, rede ou hardware. Precisa de medida antes de virar conserto.

---

## 1. Resumo — o que mais importa

| # | Severidade | Achado | Confiança |
|---|---|---|---|
| A1 | 🔴 alta | O botão **STOP da web não para um objetivo único** (clique-para-ir), só rotas | confirmado |
| A2 | 🔴 alta | Objetivo **cancelado/abortado: o seguidor continua dirigindo** o plano retido por até 7 s | confirmado |
| A3 | 🔴 alta | **Teleop pela web está morto**: publica `Twist`, o mux só aceita `TwistStamped` | confirmado |
| A4 | 🔴 alta | A **ré e o pivô de escape furam o reflexo** decidindo só pelo `/scan`, que **não enxerga nada a menos de 0,35 m** | verificar |
| A5 | 🟠 média | `freeze_capture` **sobrescreve** o CSV da cadeia a cada subida da pilha (perda de dado do PIBIT) | confirmado |
| A6 | 🟠 média | Reenviar o **mesmo ponto** depois de mexer o robô à mão segue o **plano velho** | confirmado |
| A7 | 🟠 média | Corrida entre STOP e aceite do goal: o cancelamento de rota pode se perder | confirmado |
| A8 | 🟠 média | `heading_controller`: `dt` do freio de giro vale **0** em todo ciclo de pivô (e o pivô está ligado no robô) | confirmado |
| A9 | 🟠 média | O `controller_manager` roda a **10 Hz**: até 100 ms de atraso de comando, candidato a reduzir o tempo morto | verificar |
| A10 | 🟠 média | A TF `odom→camera_init`, sem a qual o robô não anda, **só existe no `bin/sobe-robo`** e vira órfã a cada subida | confirmado |
| A11 | 🟠 média | O monitor de tensão e as métricas Nav2 da web escutam **tópicos do robô 1**: no robô 2 gravam zero | confirmado |

O resto (seções 2 a 7) vai de bug latente a limpeza.

---

## 2. Achados de segurança e de lógica (detalhe)

### A1 🔴 STOP da web não cancela o objetivo único — [confirmado]

- `controle_web/static/js/client.js:516`: o E-STOP só emite
  `stop_waypoints`.
- `controle_web/map_service.py:591` (`stop_waypoints`): cancela apenas
  `self._wp_goal_handle`, o handle do executor de **rotas**.
- O clique-para-ir (`map_service.py:860`, `send_goal`) publica em
  `/goal_pose`. O `bt_navigator` cria esse objetivo por dentro e a web **não
  guarda handle nenhum**. Por isso não existe caminho, na web, que cancele um
  objetivo único.
- **Cenário:** o dono clica num ponto, vê o robô indo mal e aperta STOP. A tela
  mostra "STOP acionado (navegação cancelada)", e o robô continua.
- **Agrava:** junto com A2, mesmo um cancelamento que funcione deixa o robô
  andando por até 7 s.
- **Direção de conserto (não aplicada):** o STOP cancelar **todos** os
  objetivos do `navigate_to_pose`. A própria action aceita cancelar com
  `goal_id` zerado e stamp zero ("cancel all"). Além disso, publicar zero num
  canal de prioridade alta durante ~1 s.

### A2 🔴 Objetivo morto, seguidor ainda dirige — [confirmado]

- `path_follower.py:1199` (`passo`) nunca consulta `tem_objetivo()`. A única
  consulta está em `entra_na_re` (`:1379`), que só decide a ré.
- Quando o objetivo é cancelado ou abortado, `self.plano` continua cheio e
  `t_plano` só vence depois de `timeout_plano = 7,0 s` (`:657`). Até lá o bloco
  "seguindo" publica rumo e velocidade normalmente.
- **Cenário:** cancelamento pelo RViz, pela ação ou (depois de A1 consertado)
  pela web. A 0,5 m/s, são até 3,5 m de deslocamento sem objetivo vivo. O
  reflexo continua ativo, mas é cego abaixo de 0,10 m de altura (comentário do
  próprio `collision_monitor.yaml`).
- **Direção:** quando `tem_objetivo()` passar de True para False, chamar
  `para('objetivo encerrado')` e limpar o plano. Cuidado com o caso de bancada
  (`plano.py` dirige por `/plan` cru, sem ação), que hoje depende de não haver
  objetivo. Isso pede um knob, como o `re_exige_objetivo` já faz.

### A3 🔴 Teleop web nunca chega ao robô — [confirmado]

- `controle_web/controllers/robot_controller.py:235-236`: publisher
  `geometry_msgs/Twist` em `/web_vel`.
- `robot_motion/config/twist_mux.yaml`: `use_stamped: true`. O mux vendorizado
  (`twist_mux/src/twist_mux.cpp:89`) cria `VelocityStampedTopicHandle`, que
  assina `TwistStamped`.
- Tipos diferentes no mesmo tópico não se conectam no DDS. O próprio
  `bin/robot-key:45-48` documenta esse defeito para o teclado ("o DDS
  rejeitaria por type hash e o mux publicaria no vazio").
- Por isso a WASD da web, o gamepad da web e o "Space = stop" do E-STOP não
  movem nem param nada. O `bin/sobe-robo` sobe a web com `WEB_TELEOP=on`, e o
  painel parece funcionar.
- **Relacionado:** `BASE_ANGULAR_SPEED = 6.0` rad/s
  (`robot_controller.py:182`) é o knob anti-skid do robô 1 que o CLAUDE.md
  manda **não herdar**. Com o multiplicador de até 4×, a web pediria até
  24 rad/s, contra o teto de 1,25. Ao consertar o tipo, recalibrar também esse
  valor.

### A4 🔴 Ré e pivô de escape decidem por um `/scan` cego a 0,35 m — [verificar]

- `robot_base/config/scan_2d.yaml:74`: `range_min: 0.35`, medido a partir de
  `base_link` (`target_frame: base_link`). O comentário logo acima já admite que
  o valor é "**HERDADO e NÃO VALIDADO**" e que subir o corte "apaga junto o
  obstáculo real colado". Com `use_inf: true`, o que fica abaixo de 0,35 vira
  `inf`, ou seja, **livre**.
- Quem confia nesse `/scan` para mover o robô **por fora do reflexo** (canal
  `unstuck_vel`, prioridade 30, depois do `collision_monitor`):
  - **ré** (`vao_traseiro`): o para-choque traseiro fica a 0,28 m do centro
    (`re_recuo_para_choque`). Um obstáculo entre 0,28 e 0,35 m do centro (0 a
    7 cm atrás do para-choque) não aparece. O vão dá `inf`, o orçamento sai em
    `min(inf − 0,30; 0,30) = 0,30 m`, e o robô recua 30 cm para dentro dele.
  - **escape para frente** (`vao_frente`): mesmo raciocínio, com o para-choque
    dianteiro a 0,28 m (`avanco_para_choque`).
  - **pivô de escape** (`vao_giro` → `folga_radial`): exige
    `desencalhe_pivo_folga = 0,334` (`path_follower.py:595`). Como nenhum
    retorno abaixo de 0,35 existe, a folga medida é **sempre ≥ 0,35 > 0,334**,
    e a checagem do pivô **nunca reprova por obstáculo próximo**. A quina física
    varre 0,314 m.
- **Por que "verificar":** é preciso saber se, nessa faixa, outra coisa já
  segura o robô. O `nuvem_pontos` corta `raio_cego = 0,15` em torno do **eixo
  do sensor**, e o `collision_monitor` não vê esse canal. Medir com um objeto
  colado atrás do robô parado e ler `/scan`.
- **Direção:** baixar o `range_min` só depois de medir o autorretorno (a etapa
  9 já citada no yaml). Ou fazer a ré, o escape e o pivô consultarem a nuvem
  (`/livox/pontos`), que não tem esse corte. No mínimo, um teste que trave
  `desencalhe_pivo_folga > range_min` como erro de configuração.

### A5 🟠 `freeze_capture` apaga a corrida anterior — [confirmado]

- `robot_nav/robot_nav/freeze_capture.py:173`:
  `open(os.path.join(base, 'freeze_capture.csv'), 'w')`, e o mesmo vale para
  `freeze_diag.csv`. O nome é fixo e o modo `'w'` trunca.
- O `out_dir` é o `log_dir` da pilha (`~/logs_robo2`). Cada nova subida da
  pilha **apaga** o CSV da cadeia da subida anterior. O `path_follower` usa
  carimbo de tempo, este nó não. Os arquivos `freeze_capture_2026-09-30_*.csv`
  em `docs/dados/` foram renomeados à mão.
- **Lacunas no mesmo nó:**
  - `/unstuck_vel`, o único canal que fura o reflexo, **não está na lista de
    tópicos**: a ré e o pivô de escape não aparecem no CSV da cadeia;
  - `cmd_vel_nav`, `follow_state` e `motion_guard/state` são do robô 1 e nunca
    chegam, e `cmd_nav_vx/wz` do `freeze_diag.csv` sai sempre 0;
  - ele assina `/plan` (cru) e não o plano que de fato dirige
    (`/path_follower/plano_aceito`).

### A6 🟠 Mesmo ponto reenviado segue o plano velho — [confirmado]

- `path_follower.py:1141-1147`: um plano novo cujo fim fica a ≤ 0,30 m do fim
  do anterior é tratado como "mesmo objetivo". Com `aceita_replano = False` e
  objetivo vivo, ele é **descartado**, e `t_plano` é renovado como se o plano
  velho fosse fresco.
- **Cenário:** o robô chega (ou o objetivo aborta). O dono o move no Xbox e
  manda o **mesmo** ponto de novo. O plano novo, que parte da posição atual, é
  recusado. O seguidor dirige pelo plano antigo a partir do índice mais próximo
  (`indice_mais_proximo`, busca global), cortando por onde o planner não
  traçou. `estado` continua `'ocioso'` (`:1184` só muda no caminho de aceite),
  então o CSV registra `ocioso` com o robô andando.
- **Direção:** a trava de replanejamento deve valer só **dentro** do mesmo
  objetivo da ação (goal_id), e não por proximidade de ponto final.

### A7 🟠 Corrida no STOP de rota — [confirmado]

- `map_service.py:596`: `stop_waypoints` lê `self._wp_goal_handle`. Esse handle
  só é preenchido em `_on_goal_response` (`:721`), **depois** que o Nav2 aceita.
- Se o STOP chegar entre o `send_goal_async` e o aceite (cerca de 0,5 s depois
  de cada `_send`, inclusive na troca de waypoint), o handle é `None`. Nada é
  cancelado, o runner sai, e o goal é aceito em seguida e executado.

### A8 🟠 `dt` zero no freio durante o pivô — [confirmado]

- `heading_controller.py:451`: dentro do bloco do pivô, `self.t_passo = agora`.
  Logo abaixo, em `:485`, `dt_ciclo = agora − self.t_passo` dá **0**.
- O `FreioDeGiro` recebe `dt = 0` em todo ciclo com pivô, e o `teto_s` do freio
  nunca avança nesses ciclos. O pivô está **ligado no robô**
  (`movimentacao.yaml: limiar_pivo: 1.40`).
- **Também no heading_controller:**
  - `avisa_de_saida` (`:250`) grita "parâmetros NÃO MEDIDOS" em toda subida,
    mesmo com `zona_morta` e `a_dec` medidos no yaml. É log que ensina a ignorar
    aviso;
  - o ramo `v_alvo < 0` (`:400`, `comando_de_re`) está **morto**: desde a 025 a
    ré sai pelo `/unstuck_vel`, e o seguidor manda `velocidade_alvo = 0`. A
    docstring do `path_follower` ("NEGATIVA aciona a ré") está velha.

### A9 🟠 `controller_manager` a 10 Hz — [verificar, é melhoria]

- `hoverboard_driver/bringup/config/hoverboard_controllers.yaml:3`:
  `update_rate: 10`. O comando vai à placa a cada 100 ms, e o
  `diff_drive_controller` integra limites e odometria a 10 Hz.
- O projeto mede ~0,5 s de "retenção" e ~0,94 s de tempo morto na malha. Até
  100 ms disso podem vir só do relógio do driver. É barato testar 50 Hz, mas
  muda a planta medida: **é mudança grande para o robô** e pede ensaio de
  bancada antes.

### A10 🟠 TF `odom→camera_init` fora do código versionado da pilha — [confirmado]

- O FAST-LIO publica `/Odometry` em `camera_init`, e o `path_follower` faz
  lookup `map→camera_init`. A ligação é um `static_transform_publisher`
  identidade lançado por `ros2 run` em `bin/sobe-robo:230`. Nenhuma launch tem
  essa TF. Subir `pilha.launch.py` sozinha no robô (como o cabeçalho dela
  ensina) deixa o seguidor parado com "sem TF". É o defeito de 14-08 de novo.
- `bin/sobe-robo:67` (`vivos`) **não casa** `static_transform_publisher`, então
  cada `--mata` deixa esse processo vivo e cada subida acrescenta outro (já
  registrado na memória do assistente, nunca consertado no script).
- **Direção:** pôr a TF na `localizacao.launch.py` (a que sobe o FAST-LIO) e
  tirar do script.

### A11 🟠 Web: telemetria do robô 1 — [confirmado]

- `controle_web/power_monitor.py:239-250` assina `/battery/front`,
  `/battery/rear` (`BatteryState`), `/wheel_vel_setpoints` (`wheel_msgs`) e
  `/hoverboard/wheel_velocities`. Todos são da MEGA do robô 1. O robô 2 publica
  `/hoverboard/battery_voltage` (`Float64`). O chip de tensão da web e o CSV
  `logs/power` ficam zerados.
- `controle_web/nav_metrics.py:147-148` assina `/odom` e `/cmd_vel` (`Twist`).
  No robô 2 a pose vem em `/Odometry` e nada publica `/cmd_vel`, porque o Nav2
  está remapeado para o tópico ignorado. As colunas de distância, velocidade,
  tempo parado e reversões saem 0, o checkpoint nunca é escrito e
  `rec_backup/spin/wait` não existem neste robô (a árvore não tem
  recuperação). **Não usar esse CSV como dado do artigo** do jeito que está.
- `map_service.py:249` desenha `/plan` (cru), e não o plano que o robô dirige
  (`/path_follower/plano_aceito`). A tela mostra uma rota diferente da
  executada.

---

## 3. Bugs menores e latentes

| Onde | O quê | Efeito hoje |
|---|---|---|
| `path_follower.py:1253-1274` | No ramo "plano velho", `entra_na_re` pode entrar em `pivo_escape`. O código cai no `self.para('plano velho')` de `:1274`, que põe `estado = 'ocioso'` no mesmo ciclo | O pivô de escape nunca roda com o plano velho |
| `path_follower.py:1663` (`grava`) | Reescreve o CSV **inteiro** a cada 5 s, e `self.linhas` cresce sem limite (20 Hz) | Memória e IO crescentes no NUC em sessão longa; O(n²) |
| `path_follower.py:1177` | `rumo_objetivo` está no frame do plano (`map`) e é comparado em `aponta()` com rumo de `/Odometry` | Latente (`aponta_no_fim=False`) |
| `lei_de_seguimento.py` (`indice_mais_proximo`) | Busca global, sem janela em torno do índice anterior | Plano que passa perto de si mesmo faz o índice saltar de trecho |
| `path_follower.py` (`chegou`, `velocidade_de_seguimento`) | Usa distância **euclidiana** ao fim, não ao longo do caminho | Plano em U freia cedo; em caso extremo, chegada antecipada |
| `hoverboard_driver.cpp:295` | `max_velocity /= wheel_radius` a cada `on_activate` | Reativar o hardware divide de novo |
| `hoverboard_driver.cpp:319` | `exit(-1)` se a serial não abre | Derruba o `ros2_control_node` inteiro; deveria retornar `ERROR` |
| `hoverboard_driver.hpp:196`, `.cpp:342` | `port_fd` não inicializado; `on_deactivate` não manda zero nem zera o fd | Latente |
| `hoverboard_driver.cpp:144` | O parâmetro `"f"` grava em `pid_config.p` (copiar e colar) | PID fora de uso, mas errado |
| `hoverboard_driver.cpp` | O nó `hoverboard_driver_node` nunca recebe `spin` | O callback de parâmetros nunca roda |
| `robo2.urdf.xacro:280` | `/dev/ttyUSB0` fixo, sem regra udev | Outro adaptador USB-serial pode roubar o nome |
| `hoverboard_controllers.yaml:38` | `use_stamped_vel: false` | Parâmetro obsoleto no Jazzy (o controlador já é stamped) |
| `pilha.launch.py:828` | `ros2 bag record --all-topics` ligado por padrão. O comentário em `:791-793` diz que o `--all-topics` (560 MB/min) **travou o LIO** em 20-08 | Risco consciente; o MCAP de 30-09 deu 8,2 GB |
| `pilha.launch.py:826-827` | `log_dir:=''` com `bag:=true` gera `ros2 bag -o /corrida_…` na raiz do disco | Falha do gravador |
| `pilha.launch.py:150` (`_passou`) | Lê `sys.argv` | Não funciona se a pilha for incluída por outra launch |
| `robot_motion/package.xml` | A pilha sobe `robot_nav/freeze_capture`, mas não há `exec_depend` em `robot_nav` | `rosdep`/instalação limpa não sabe |
| `map_service.py` | `json`/`_json` importados duas vezes; `csv` não usado | Cosmético |
| `controle_web/app.py` (disconnect) | A desconexão de **qualquer** cliente faz `force_stop` global | Com 2 clientes, um fecha a aba e corta o outro (hoje é inócuo por A3) |
| `measure_web_lag.py` / `map.js:292` | Esperam `_sts` no `scan_update`, que o servidor nunca manda | A ferramenta de lag não mede nada |
| `joystick.launch.py` | Passa ao `joy_node` (SDL2 no Jazzy) o **número do `/dev/input/jsN`** como `device_id`. No SDL o índice é outro | Verificar: casa hoje por coincidência de haver um só controle |

---

## 4. Código morto e configuração que engana

- **Árvore de comportamento:** o `SmoothPath` usa `smoother_id="simples"`
  (escolha medida, comentada no XML). O `suave` (ConstrainedSmoother, com
  `minimum_turning_radius: 0.37`) e o `savgol` seguem configurados em
  `nav2.yaml:550` sem uso. Os comentários do `path_follower` que dizem que o
  plano suavizado "cabe" no raio da máquina descrevem um smoother que não está
  no caminho.
- **Queda para o plano cru:** o bloco em `path_follower.py:702-724` descreve a
  queda para `/plan` cru quando o suavizador cala. Mas `aceita_plano_cru=False`
  (`:668`) desliga essa queda por padrão. O comportamento real é "robô parado
  aguardando o suavizador", e o comentário diz o contrário.
- **`local_costmap`:** só alimenta o `controller_server`, cujo comando é
  descartado. São VoxelLayer a 5 Hz, com `publish_voxel_map: true`, gastando
  CPU do NUC. O servidor precisa existir (decisão registrada), mas as taxas
  podem cair.
- **`lei_de_seguimento.rumo_com_desvio`:** duplica `correcao_de_desvio` mais
  `CorrecaoDeDesvio`. Só `test_lei_de_seguimento.py` a chama (conferido); a
  produção não.
- **`hoverboard_driver`:** o PID é calculado e descartado em todo ciclo
  (`(void)pid_outputs`). `on_encoder_update`, as macros `ENCODER_*`/`TICKS`, o
  `config.hpp` e a dependência `control_toolbox` só servem a isso. Também são
  exemplo upstream sem uso: `description/` (diffbot), `bringup/launch/`,
  `doc/` e `hoverboard_driver.ros2_control.xacro` (o comentário no
  `robo2.urdf.xacro` avisa que ele **não** é incluído). O
  `hoverboard_controllers.yaml` traz comentários herdados ("0,35 m/s ficava
  abaixo do deadband… 0,7 = devagar") que contradizem a zona morta medida
  (0,0178).
- **Launches antigas do `robot_motion`:** `navegacao.launch.py` mais
  `goal_navigator.py` e `navegacao_ponto.py` (pré-Nav2, decisão 006), e
  `movimentacao.launch.py`. O `goal_navigator` **ainda é usado** por
  `tools/banco/corrida_gazebo.py:406`, então não é morto. Decidir se essa
  bancada continua; as launches em si não são chamadas por nenhum script.
- **Docstrings com planner errado:** `path_follower` e `lei_de_seguimento`
  falam em "Smac Hybrid-A* em Dubins" e "Theta*"; o `nav2.yaml` usa
  `SmacPlanner2D`.
- **Cabeçalhos herdados em `bin/robot-key`:** falam de PS4, `nav_vel` e
  `setup_pi.sh` (robô 1). O comando em si está certo.

---

## 5. Inventário do ROBÔ 3 para excluir (decisão do dono, 01-10)

Tudo abaixo é do robô 3 e sai deste repo. **Antes de apagar, conferir que o
repositório do robô 3 já tem cada peça**, sobretudo o firmware e as decisões.

**Código e config ROS**
- `robot_base/config/geometria_robo3.yaml`
- `robot_base/config/hoverboard_controllers_sim_robo3.yaml`
- `robot_base/description/robo3.urdf.xacro`, `robo3.gazebo.xacro`
- `robot_base/launch/sim_robo3.launch.py`, `robot_base/test/test_urdf_robo3.py`
- `robot_motion/config/perfil_robo3.yaml`, `robot_motion/test/test_perfil_robo3.py`
- `robot_nav/launch/controle_robo3.launch.py`
- `robot_nav/config/teleop_xbox_robo3.yaml`, `twist_mux_robo3.yaml`
- `robot_nav/test/test_contrato_robo3.py`, `test_controle_robo3_launch.py`
- `robot_nav/robot_nav/mega_bridge.py`, `cmd_vel_to_wheels.py`, `dpad_reto.py`
  e os testes deles. Esses nós são do robô 1 e foram adaptados para o robô 3;
  nenhum roda no robô 2.

**Entrelaçamentos que precisam de edição, não só de `rm`** (cada um é uma
mudança pequena):
- `robot_motion/robot_motion/perfil.py`: o ramo `robo == 3`, `_robo3`,
  `_retangulo` e o mecanismo de `*_rewrites` existem só para o robô 3. Sem ele
  o perfil vira constantes do robô 2, e os testes `test_perfil.py`,
  `test_reescritas.py` e `test_reflexo_por_perfil.py` encolhem junto.
- `pilha.launch.py`: o argumento `robo`, `_recusa_robo` (`:108`) e a checagem
  de reescritas (`:175-179`).
- `path_follower.py:516-520`: o comentário do `avanco_para_choque` cita o robô
  3; o parâmetro em si fica.
- `test_configs_coerentes.py`, `test_pilha_robo.py` e
  `test_avanco_para_choque.py`: partes que comparam com o robô 3.
- `pytest.ini`: remover `tools/sobe_robo3`, `tools/valida_etapa4`,
  `tools/valida_etapa5` e `firmware/mega_bridge/tools`.
- `.gitignore`, `robot_motion/package.xml` e `robot_nav/package.xml`: citações
  do robô 3 (conferir uma por uma).
- `ESTADO_PROJETO.md`: as seções de 10-09 a 24-09 são do robô 3. Arquivar ou
  mover para o repo dele.

**Ferramentas e scripts**
- `bin/sobe-robo3`, `bin/smoke-gazebo-robo3`, `bin/valida-etapa4`,
  `bin/valida-etapa5`
- `tools/sobe_robo3/`, `tools/valida_etapa4/`, `tools/valida_etapa5/`,
  `tools/smoke_gazebo_robo3.py`
- `tools/teclado_placa.py`, `tools/escuta_placa.py`, `tools/grava_pivo.py`
- `bin/linha-de-base-robo2` e `tools/linha_de_base/`: são do robô 2, mas
  existem como régua da migração para o robô 3 (etapa 4). **Pergunta ao
  dono:** fica como ferramenta do robô 2 ou sai junto?

**Firmware:** `firmware/mega_bridge/` (MEGA, robô 1 e depois robô 3),
`firmware/hover_ponte/`, `hover_escuta/` e `hover_sniff/`. O robô 2 não tem
microcontrolador.

**Documentos:** `docs/ETAPA1_MEDIDAS_ROBO3.md`, `PLANO_CONTROLE_ROBO3.md`,
`PLANO_ETAPA4_ROBO3.md`, `PLANO_ETAPA5_ROBO3.md`, `PLANO_NAV2_ROBO3.md`,
`ROBO3_REVISAO_CRUZADA.md`, `ROTEIRO_PASSO7_ETAPA4.md` e
`PROTOCOLO_ETAPA2_FRENTE_RE.md` (é do robô 3, conferido); as decisões **046–049 e 051–054**.
A **050** (trava do AMCL) é do robô 2 e fica.

**Dados:** `docs/dados/2026-09-14-robo3-*`, `2026-09-15-robo3-*`,
`2026-09-17-robo3-*`, `2026-09-18-robo3-*`, `2026-09-22-etapa4-passo7`,
`2026-09-22-etapa5-contrato` e `2026-09-24-robo3-lio`.

> ⚠️ **Decisão para o dono:** apagar decisões, diário e dados do robô 3 **muda
> o histórico do PIBIT**. Alternativa: mover para o repo do robô 3 e deixar aqui
> uma linha "movido para …". O `git log` guarda tudo de qualquer jeito.

---

## 6. Herança do ROBÔ 1 que não roda no robô 2 (lixo velho)

A decisão 000 já previa demolir estes itens; eles ficaram como "fósseis
conscientes".

- **Launcher antigo:** `launch.sh` e `start.sh` (LD06, MEGA, Pi, `--pi`,
  `maps/hotmilk_portas.yaml` e `worlds/sala.sdf`, **que não existem**),
  `bin/robot-up`, `bin/robot-connect` (trekking, PS4),
  `scripts/cpu_logger.sh` (Pi), `bin/analyze_zigzag.py` (lê `follow_debug.csv`
  do robô 1), `setup.sh` (cita `Controle_robo_web`), `setup_udev.sh` (só
  `/dev/mega`) e `CONEXOES.txt` (o próprio arquivo diz "não use").
- **`robot_nav`:** do pacote inteiro, o robô 2 só usa `freeze_capture`. Sobra
  `pose_estimator`, `unstuck_supervisor` (1433 linhas mais 1514 de teste),
  `motion_guard`, `scan_sanitizer`, `path_follower` (o antigo), `fused_odom`,
  `cone_pose_fix`, `sim_actuator_model`, `utils`, as launches `robot`, `nav2`,
  `sim` e `slam`, os `urdf/` (husky, sim_robot), `nav2_params_pi.yaml`,
  `nav2_params_legacy.yaml`, `twist_mux*.yaml`, `teleop_ps4.yaml`, `scripts/`
  (`arc_calib`, `spin_calib`), `tools/` (`flow_check`, `imu_check`) e
  `behavior_trees/navigate_w_backup_first_recovery.xml`.
  **Sugestão:** mover `freeze_capture` para o `robot_motion` (ou para um pacote
  `robot_logs`) e apagar o `robot_nav`. Depois disso, o `wheel_msgs` também fica
  sem uso (conferir o `power_monitor`).
- **Web:** o modo `slam` com `trekking/yaw_fix` (fala com o `pose_estimator`,
  que não roda aqui) e a limpeza `/local_costmap/clear_entirely_local_costmap`
  entre waypoints (inofensiva, mas herdada). `power_monitor` e `nav_metrics`
  estão na A11. `PLANO_HEADLESS_2026-05-22` e `docs/superpowers/specs` são
  citados em comentários e não existem neste repo; o `teleop_ps4.yaml` citado
  em `app.py` é o do `robot_nav` (robô 1).
- **`MIGRACAO_LIVOX.md`:** o README já diz que é histórico superado; pode ir
  para `docs/` como arquivo morto, ou sair.

---

## 7. Lixo local (fora do git, só nesta máquina)

- `tools/ab_freio/`, `tools/analise_corrida/`, `tools/audita_livox/`,
  `tools/subidas_robo3/`, `tools/valida_etapa6/` e `tools/valida_etapa7/` só
  têm `__pycache__`. O código foi apagado e o cache ficou.
- `docs/dados/` tem **35 GB** no disco (bags ignorados pelo git;
  `2026-08-20-constancia-gargalos-gazebo` sozinho tem 23 GB). Decisão do dono:
  manter, mover para disco externo ou apagar os bags já analisados.
- `controle_web/logs/`, `.pytest_cache/` e `__pycache__/` espalhados.
- Ao lado: `docs/decisoes/` tem **dois `010-*`** e **não tem `045`**; o
  `DIARIO.md` mistura ordens (o topo vai até 09-24, e 10-01 está no fim); o
  `ESTADO_PROJETO.md` tem ~5000 linhas e já não funciona como handoff rápido.
  Sugestão: arquivar as seções antigas em `docs/estado_arquivo/`.

---

## 8. Qualidade geral — e o que pedir ao Codex

**Observação estrutural.** Os nós carregam a história do projeto dentro do
código: há parâmetros com 40–60 linhas de comentário de diário (datas,
corridas, falas do dono, "o codex fez…"). Só o `path_follower.py` tem ~680
linhas de `declare_parameters`, quase todas comentário. A rastreabilidade
importa para o PIBIT, mas o lugar dela são `docs/decisoes/` e o diário. No
código bastaria **o valor, a unidade, a decisão (NNN) e, quando for o caso, a
invariante de segurança**. Do jeito que está, já aparecem os primeiros
comentários desatualizados (seção 4), e eles vão se multiplicar.

**Pontos que valem a pena o Codex contestar:**
1. **A4:** a conta do `range_min` contra o para-choque e a folga do pivô. A
   geometria está certa? Existe outra camada que segura a ré nessa faixa?
2. **A2/A6:** o efeito de parar no fim do objetivo e de destravar o replano por
   `goal_id` sobre o caso de bancada (`plano.py`, sem ação) e sobre a
   recuperação infinita.
3. **A3:** confirmar no grafo vivo (`ros2 topic info -v /web_vel`) que existem
   dois tipos no mesmo tópico.
4. **A8:** confirmar que o `dt=0` não muda o comportamento medido do pivô (o
   freio só atua com a lei calada).
5. Achados que eu **não** vi nos arquivos que só inventariei (seção 0).

**Ordem sugerida para a fase 2** (uma mudança por commit, cada uma com ok):
1. A1, A2 e A7 (STOP de verdade). É o maior risco operacional.
2. A3 (tipo do `/web_vel`) e a calibração da web.
3. A5 (`freeze_capture` com carimbo de tempo e com `/unstuck_vel`).
4. A10 (TF `camera_init` numa launch).
5. A4, só depois de medir.
6. Remoção do robô 3 (seção 5), por blocos: ROS, tools, firmware, docs.
7. Remoção da herança do robô 1 (seção 6) e mudança do `freeze_capture` de
   pacote.
8. Bugs menores (seção 3) e enxugamento de comentários, por arquivo.
