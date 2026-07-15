# Estado do Projeto — Controle_robo_livox (PIBIT)

> Documento vivo. Resumo do que está acontecendo, BOs abertos, avanços e o que falta.
> Versionado na `main`. Atualizado em **2026-07-15**.
>
> **Este projeto é um PIBIT** — vai virar artigo. Toda decisão técnica tem um
> registro em `docs/decisoes/`, todo dia de trabalho entra no `docs/DIARIO.md`,
> e escolhas de abordagem são embasadas em literatura (`docs/REFERENCIAS.md`).
> Ritmo deliberadamente devagar: 1 mudança pequena por vez.

---

## O robô

- **Tração**: 2 rodas de hoverboard (diferencial) + roda boba na frente.
  1 placa hoverboard hackeada ligada **DIRETO no PC** — ⚠️ NÃO tem Arduino
  MEGA (o ESTADO anterior afirmava "mesma eletrônica do robô 1"; estava
  errado — corrigido 07-15 após inventário com o dono). Forma exata da
  conexão (adaptador? firmware da placa?) a confirmar na bancada.
- **Sensor**: Livox Mid-360 (LiDAR 3D 360°, IMU embutida, conexão Ethernet).
  Único sensor externo — sem câmera, sem IMU externa, sem optical flow.
  Também sem eletrônica auxiliar: sem relé de luz, LED de marco ou botão.
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

## ⏳ Próximos passos

1. **Inspeção de bancada** (com o dono, checklist na entrada 07-15 do
   DIARIO): como as rodas chegam no PC + firmware da placa → destrava a
   **decisão de arquitetura da ponte** (BO-1). De quebra: confirmar
   Ubuntu/ROS Jazzy + ssh da NUC — pré-requisito de tudo.
2. **Ponte PC↔placa hover** (ADAPTA 1 reformulado): implementar conforme a
   arquitetura decidida em BO-1. Candidata do assistente: nó
   `hoverboard_bridge.py` falando 0xABCD direto, substituindo o par
   firmware/mega_bridge + `mega_bridge.py`.
3. **ADAPTA 2 rebaixado a calibração**: `wheel_base`, `linear_scale`,
   `left/right_wheel_sign` — precisa do robô andando, sem código novo.
4. **Leitura da fase de localização 3D**: FAST-LIO2 vs Point-LIO vs LIO-SAM
   (fila em docs/REFERENCIAS.md) → decisão com literatura.
5. **Driver Livox** (`livox_ros_driver2`, Ethernet/IP estático) no
   placeholder do launch.sh — instalar e ver a nuvem chegar já valida o HW
   sem comprometer arquitetura.

## BOs abertos

- **BO-1 — Arquitetura da ponte PC↔placa hover** (aberto 07-15): direto via
  USB-serial (candidata A) vs reintroduzir MEGA (B). Dono decidiu escolher
  só com a bancada na frente. Vira `docs/decisoes/002` quando fechar.
  Numeração de decisões: a leitura LIO (passo 4) usa o próximo número livre
  na hora, sem reserva.
