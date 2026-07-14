# 000 — Herança do robô 1: clone com histórico + o que ficou e o que saiu

**Data**: 2026-07-14 · **Status**: decidido e executado

## Contexto

O projeto (PIBIT) precisa de uma base de software para um robô diferencial
(2 rodas de hoverboard + roda boba) com Livox Mid-360 e Intel NUC. Existe um
projeto anterior maduro e validado em campo — `Controle_robo_web` (robô 1,
skid-steer 4 rodas, LiDAR 2D LD06, Raspberry Pi) — com GUI web completa,
stack Nav2 tunada, nós de comportamento (guarda de movimento com prioridade
a humanos, supervisor de desencalhe, seguidor de caminho) e uma metodologia
de diagnóstico por CSV.

## Alternativas consideradas

1. **Começar do zero** — descartada: joga fora ~2 meses de conhecimento
   validado em campo (GUI, comportamentos, metodologia de medição) que é
   agnóstico a cinemática/sensor.
2. **Mesmo repo com perfis robo1/robo2** — descartada: as tecnologias
   divergem demais (3D vs 2D, diferencial vs skid, NUC vs Pi); perfis
   virariam `if robo2` espalhado e acoplariam os cronogramas dos 2 projetos.
3. **Fork no GitHub** — descartada: fork mantém vínculo de sincronia com o
   upstream, que é o oposto do desejado (divergência total); e o GitHub nem
   permite fork do próprio repo na mesma conta.
4. **✅ Clone com histórico completo + demolição seletiva** — escolhida:
   repo independente, `git log`/`blame` preservam a arqueologia das decisões
   (por que cada constante vale o que vale), e correções boas podem cruzar
   entre repos com `git cherry-pick` manual.

## O que foi demolido (e por quê) — commits `f406bed`, `fe48a86`, `2945725`

- **teb_local_planner + costmap_converter**: vendorados para experimento no
  robô 1, nunca entraram em produção (o controlador real é o path_follower
  próprio). ~156k linhas fora.
- **Trekking/cone (teach-and-repeat)**: muleta da era de odometria fraca
  pré-LiDAR-bom; sem papel com Mid-360 + Nav2.
- **Travessia de porta (door_crossing)**: máquina two-phase amarrada às
  anotações do mapa do robô 1; com localização 3D (fase 2) a hipótese é que
  passagem apertada se resolva sem nó dedicado. Cherry-pick se refutada.
- **Firmwares de diagnóstico, mapas, worlds, laudos**: específicos do
  hardware/ambiente do robô 1.

Fósseis conscientes (adiados para não inflar a demolição): standdown de
porta no unstuck_supervisor (inerte sem publisher de `/door_zone`) e
`cone_pose_fix.py` (importado pelo pose_estimator; morre na simplificação).

## O que ficou (hipótese de reuso a validar em campo)

GUI web inteira, motion_guard, unstuck_supervisor, path_follower,
scan_sanitizer, nav_metrics/power_monitor/freeze_capture (instrumentação),
configs Nav2 como ponto de partida. Premissa: todos consomem `/scan` 2D +
TF + `/plan`, cegos a cinemática e sensor. A fase 1 da migração
(Livox → pointcloud_to_laserscan → `/scan`) testa exatamente essa premissa.

## Risco assumido

Constantes calibradas para o skid-steer embutidas em defaults (banda morta
de giro, caps de velocidade angular) NÃO valem para o diferencial e precisam
de recalibração explícita — rastreado no MIGRACAO_LIVOX.md (ADAPTA 2/5).
