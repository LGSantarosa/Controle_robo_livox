# Estado do Projeto — Controle_robo_livox (PIBIT)

> Documento vivo. Resumo do que está acontecendo, BOs abertos, avanços e o que falta.
> Versionado na `main`. Atualizado em **2026-07-24**.
>
> **Este projeto é um PIBIT** — vai virar artigo. Toda decisão técnica tem um
> registro em `docs/decisoes/`, todo dia de trabalho entra no `docs/DIARIO.md`,
> e escolhas de abordagem são embasadas em literatura (`docs/REFERENCIAS.md`).
> Ritmo deliberadamente devagar: 1 mudança pequena por vez.

---

## O robô

- **Tração**: 2 rodas de hoverboard (diferencial) **na frente** + roda boba
  **atrás** (corrigido 07-24: o ESTADO dizia boba na frente). 1 placa
  hoverboard hackeada ligada **DIRETO no PC** por serial — sem Arduino MEGA.
  Protocolo `0xABCD` @115200, realimentação de **18 bytes**.
  ⚠️ Essa geometria (motriz dianteira, boba traseira) é **instável em rumo**:
  a traseira só acompanha por arrasto e amplifica oscilação. É dado de projeto
  do controlador, não detalhe.
- **Sensor**: Livox Mid-360 (LiDAR 3D 360°, IMU embutida, conexão Ethernet).
  Único sensor externo — sem câmera, sem IMU externa, sem optical flow.
  Também sem eletrônica auxiliar: sem relé de luz, LED de marco ou botão.
- **Computador**: Intel NUC (x86), Ubuntu 24.04 + ROS 2 Jazzy. Interface do
  lidar: `enp2s0` (IP `192.168.1.2`).
- **Ambiente**: novo (não é o do robô 1). **Não precisa de mapa** — a
  localização é LIO, sem AMCL (decisão 003).
- Estado físico: montado e pronto. **Base de software (tração + localização)
  verificada em hardware**; falta a movimentação.

### Medidas ainda NÃO conferidas (afetam tudo acima)

`wheel_separation: 0.32` e `wheel_radius: 0.0825` no controlador diferencial
são valores **herdados, não medidos neste robô**. Erram a odometria de roda e
a conversão do comando em rad/s. **Medir com trena** antes de calibrar
movimentação.

## 2026-07-14 — Nascimento do repo: clone do robô 1 + demolição

- Repo criado como **clone com histórico completo** de `Controle_robo_web`
  (`df76a47`) — decisão em `docs/decisoes/000-heranca-do-robo1.md`.
- **`MIGRACAO_LIVOX.md`** (raiz) = plano de demolição/reforma, aprovado pelo dono.
- **Demolição executada** (A `f406bed`, B `fe48a86`, C `2945725`): fora
  teb/costmap_converter, firmwares de diagnóstico, trekking/cone, porta
  (door_crossing e toda a cadeia), mapas/worlds/laudos do robô 1, PS4.
  274 testes verdes após cada fatia.
- Fósseis conscientes (remover na fase ADAPTA): standdown de porta no
  unstuck_supervisor (inerte sem `/door_zone`); `cone_pose_fix.py` (o
  pose_estimator importa `apply_pose_fix`).

## 🧭 2026-07-14 (2ª leva) — Arquitetura-alvo definida + varredura LD06

- **Decisão 001 (`docs/decisoes/001-gui-2d-localizacao-3d.md`)**: humano
  opera em mapa 2D (GUI herdada); o robô se localiza/navega em 3D (LIO no
  Mid-360). Nav2/localização/movimentação repensadas DO ZERO com literatura;
  "fase 1 clone barato" (stack 2D+AMCL como alvo) DESCARTADA — stack herdada
  vira referência/candidata, não caminho assumido.
- **Varredura LD06**: fora test_lidar.sh, lidar.launch.py, retry+watchdog
  serial do launch.sh (agora placeholder explícito do Livox em [3]), passo
  LiDAR do setup_udev.sh (Mid-360 é Ethernet; udev segue só pra MEGA),
  bin/teleop-pernas; README reescrito pro robô 2 (o antigo tinha 1406 linhas
  do robô 1).

## 2026-07-15 — Investigação ADAPTA 1-2: robô 2 NÃO tem MEGA

- Inventário real com o dono: **1 placa hover direto no PC** + Livox + NUC +
  baterias. Nada de MEGA/relé/LED/botão/IMU externa/flow — seção "O robô"
  corrigida acima.
- `cmd_vel_to_wheels.py` já é diferencial puro (knobs anti-skid já não
  existiam) → ADAPTA 2 = só calibração de params com o robô.
- Protocolo da placa (família EFeru/NiklasFauth, `0xABCD` @115200, feedback
  18 B) mapeado de `firmware/mega_bridge/*/hoverboard.{h,cpp}` — base pronta
  pra uma ponte direta em Python, se a bancada confirmar.
- Detalhes + checklist de inspeção: entrada 07-15 do `docs/DIARIO.md`.

## 2026-07-15 (2ª leva) — Meta final + time de 3 + rota 2D→LIO (decisão 002)

- **Meta final explicitada pelo dono**: robô 2 indo de um ponto a outro sem
  bater, igual ao robô 1. É o critério de sucesso.
- **Time vira 3 pessoas**: dono (~6 meses de ROS) + 2 iniciantes totais.
  Divisão em frentes: **A** base/motores · **B** percepção/Livox ·
  **C** infra/GUI. Organização do trabalho a 3 (branches, revisão cruzada,
  DIARIO com autor) a combinar com o time.
- **Decisão 002** (`docs/decisoes/002-rota-2d-primeiro-lio-depois.md`):
  clone 2D primeiro (receita do robô 1: /scan derivado + AMCL + Nav2),
  LIO 3D depois como evolução comparada — o baseline 2D vs LIO vira
  resultado do artigo. Revisa a ordem da 001; o conceito (GUI 2D pro
  humano) permanece.

## 🚀 2026-07-24 — Base de pé: tração + localização (decisão 003)

O robô deixou de ser projeto e virou máquina com base funcionando. **Decisão
003** (`docs/decisoes/003-base-ros2control-e-lio.md`) fecha o BO-1 e revisa a
001 e a 002.

- **Tração**: `ros2_packages/hoverboard_driver/` — interface `ros2_control`
  falando serial direta com a placa + `diff_drive_controller`. Compila limpo
  no Jazzy. **Substitui** `mega_bridge.py` e `cmd_vel_to_wheels.py`.
- **Localização**: Livox Mid-360 + **FAST-LIO** → `/Odometry`. Sem `/scan` 2D,
  sem mapa, sem AMCL — a rota "2D primeiro" da decisão 002 caiu.
- **`ros2_packages/robot_base/`** (novo) amarra as duas:
  `ros2 launch robot_base base.launch.py`.
- **`setup_livox.sh`** (novo) traz os drivers de terceiros em commits fixados,
  compila/instala o SDK nativo da Livox e aplica a config de rede do lidar.
- **Movimentação NÃO foi herdada** — decisão explícita do dono. A camada
  existente andava sempre a fundo (o teto do controlador engolia a
  desaceleração por desalinhamento) com o giro saturado → **anda em S** e não
  fecha curva. Vamos escrever a nossa, em SI real, sem escada de ganhos.

## 🧪 2026-07-24 (2ª leva) — Simulador (decisão 004)

Robô 2 montado no Gazebo Harmonic para ajustar movimentação **sem o robô**.
Premissa da decisão 004: o simulador só serve se **errar como o robô erra**.

- Caixa 0,50 × 0,50 × 0,30, fundo a 0,10 do chão; motrizes de hoverboard na
  frente (separação 0,20); boba no centro da traseira; **10 kg**.
- **Boba com trail de 4 cm e atrito no pivô** — é o que faz a traseira ser
  jogada pra fora no giro. Esfera lisa (o jeito fácil) não reproduz nada.
- **`/Odometry` = pose verdadeira do Gazebo**, mesmo papel do LIO. Odometria de
  roda esconderia a derrapada.
- **Mesmo `diff_drive_controller` e mesmos tetos** do robô real; só a camada de
  hardware muda. Sem isso o ajuste não transfere.
- Arquivos: `ros2_packages/robot_base/description/robo2.urdf.xacro`,
  `worlds/pista_livre.sdf`, `launch/sim.launch.py`,
  `config/hoverboard_controllers_sim.yaml`.
- **Verificado**: física estável no Gazebo (assenta nas 3 rodas em z≈0);
  11 testes travam geometria/massa. Suíte: **285 verdes**.
- ⚠️ Falta `ros-jazzy-gz-ros2-control` (apt, precisa de sudo) para dirigir.

## ⏳ Próximos passos

**Sem o robô (dá pra fazer agora):**

1. **Instalar `ros-jazzy-gz-ros2-control`** — `sudo apt install -y
   ros-jazzy-gz-ros2-control`. Destrava dirigir no simulador.
2. **Confirmar que o S aparece no simulador.** Se não aparecer, o simulador está
   otimista: primeiros suspeitos são `boba_pivo_atrito` (baixo demais) e
   `boba_trail` (curto demais), ambos no topo da URDF.
3. **Escrever a movimentação** — a meta declarada. Regra da decisão 003: SI real
   direto no `diff_drive_controller`, sem escada de ganhos, limite num lugar só.
4. *(opcional)* Instalar o SDK da Livox — `./setup_livox.sh` com `sudo`;
   compilação já feita em `third_party/Livox-SDK2/build`. Só serve pra compilar
   a localização aqui, que sem lidar não dá pra testar.

**Quando tiver o robô:**

5. **Medir `wheel_separation` e `wheel_radius` com trena** — e reconciliar com o
   simulador (0,20 lá × 0,32 no controlador real; nenhum dos dois medido).
6. **Confirmar o IP do lidar** — varredura procurando OUI `e4:7a:2c`. Já foram
   vistos `.169` e `.158`. Errado = `bind failed` = sem `/Odometry`, falha
   silenciosa. Ver `ros2_packages/robot_base/config/README.md`.
7. **Subir a base no robô**: `base.launch.py`, confirmar `/Odometry` e que as
   rodas obedecem a `cmd_vel`.
8. **Calibrar o simulador contra o robô** — critério: fazer um S *parecido* na
   mesma manobra.

## Fósseis conscientes (remover em fatia própria)

`robot_nav/mega_bridge.py` e `robot_nav/cmd_vel_to_wheels.py` deixaram de ser
caminho (decisão 003) mas **não foram apagados**: a GUI e a suíte de testes
ainda os referenciam, e remover tudo junto quebraria as duas. Também seguem
inertes o standdown de porta no `unstuck_supervisor` e o `cone_pose_fix.py`.

## BOs abertos

- ~~**BO-1 — Arquitetura da ponte PC↔placa hover**~~ ✅ **FECHADO 07-24**:
  serial direta, sem microcontrolador. Registro na decisão 003.
- **BO-2 — O que o artigo compara** (aberto 07-24): a decisão 003 tirou do
  caminho o baseline 2D vs LIO que a 002 previa como resultado. Candidatos:
  comparar métodos de LIO entre si, ou comparar estratégias de controle de
  rumo para esta geometria (motriz dianteira + boba traseira) — que é o
  problema real em mãos.
