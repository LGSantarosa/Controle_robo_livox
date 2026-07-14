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

## ⏳ Próximos passos (ordem do MIGRACAO_LIVOX.md)

1. **ADAPTA 1-3**: mega_bridge 2 motores (sem IMU/flow no frame serial),
   `cmd_vel_to_wheels` diferencial (SEM os knobs anti-skid do robô 1 —
   zona-morta 1.7, autoridade 6.0, spin_calib NÃO se aplicam), pose_estimator
   simplificado (só roda na fase 1).
2. **Launch/Livox**: driver `livox_ros_driver2` (Ethernet, IP estático) +
   `pointcloud_to_laserscan` → `/scan` 2D → stack herdada (fase 1).
3. **NUC**: Ubuntu/ROS Jazzy + ssh — confirmar com o dono.
4. **Fase 2 (depois de andar)**: odometria/localização 3D (FAST-LIO vs
   Point-LIO vs LIO-SAM — decisão COM literatura, registro em docs/decisoes/).

## BOs abertos

- (nenhum — projeto recém-nascido)
