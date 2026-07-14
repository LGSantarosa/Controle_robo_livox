# Estado do Projeto — Controle_robo_livox (PIBIT)

> Documento vivo. Resumo do que está acontecendo, BOs abertos, avanços e o que falta.
> Versionado na `main`. Atualizado em **2026-07-14**.
>
> **Este projeto é um PIBIT** — vai virar artigo. Toda decisão técnica tem um
> registro em `docs/decisoes/`, todo dia de trabalho entra no `docs/DIARIO.md`,
> e escolhas de abordagem são embasadas em literatura (`docs/REFERENCIAS.md`).
> Ritmo deliberadamente devagar: 1 mudança pequena por vez.

---

## O robô

- **Tração**: 2 rodas de hoverboard (diferencial) + roda boba na frente.
  Mesma eletrônica do robô 1: placa hoverboard hackeada + ponte Arduino MEGA.
- **Sensor**: Livox Mid-360 (LiDAR 3D 360°, IMU embutida, conexão Ethernet).
  Único sensor externo — sem câmera, sem IMU externa, sem optical flow.
- **Computador**: Intel NUC (x86). SO/versão: ⏳ a confirmar (Jazzy pede 24.04).
- **Ambiente**: novo (não é o do robô 1) — mapa e rede a definir.
- Estado físico: montado e pronto; **só falta o software**.

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

## ⏳ Próximos passos

1. **NUC**: confirmar Ubuntu/ROS Jazzy + ssh — pré-requisito de tudo.
2. **ADAPTA 1-2**: mega_bridge 2 motores (sem IMU/flow no frame serial) +
   `cmd_vel_to_wheels` diferencial (SEM knobs anti-skid do robô 1). É o
   "reuso garantido" da decisão 001 — pode andar antes das decisões de nav.
3. **Leitura da fase de localização 3D**: FAST-LIO2 vs Point-LIO vs LIO-SAM
   (fila em docs/REFERENCIAS.md) → decisão 002 com literatura.
4. **Driver Livox** (`livox_ros_driver2`, Ethernet/IP estático) no
   placeholder do launch.sh — instalar e ver a nuvem chegar já valida o HW
   sem comprometer arquitetura.

## BOs abertos

- (nenhum — projeto recém-nascido)
