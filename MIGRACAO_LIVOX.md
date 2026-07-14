# Plano de migração — Controle_robo_web → Controle_robo_livox

> Robô 2: diferencial (2 rodas hover + boba), Livox Mid-360 (único sensor,
> IMU embutida), Intel NUC, ambiente novo. Base = clone do repo do robô 1
> em `df76a47`. **Revisar este plano ANTES de eu deletar qualquer coisa.**
>
> Fase 1 = "clone barato": Livox → pointcloud_to_laserscan → `/scan` 2D →
> stack atual roda quase inalterada. Fase 2 = FAST-LIO/Point-LIO 3D.

---

## 🗑️ RASGA (deletar de vez — o git guarda a história)

| Item | Por quê |
|---|---|
| `ros2_packages/teb_local_planner/` + `ros2_packages/costmap_converter/` | Vendorados p/ experimento, NUNCA entraram em produção (controlador real = path_follower custom). Maior limpeza do repo. |
| `firmware/flow_test/`, `firmware/imu_diag/`, `firmware/led_mega_test/`, `firmware/led_nano_test/` | Diagnóstico de sensores que o robô 2 não tem (PMW3901, MPU, anel de LED). |
| `robot_nav/cone_detector.py` + `cone_pose_fix.py` | Correção de pose por cone-âncora do robô 1, nunca validada em bancada. |
| `robot_nav/trekking_runner.py` + `controle_web/trekking_service.py` | Teach-and-repeat do robô 1 (era muleta da odometria fraca). Com Livox não faz sentido. ⚠️ tirar imports/rotas no `app.py` e na GUI junto. |
| `maps/*` (sala, hotmilk, golden) | Mapas do ambiente do robô 1. Ambiente novo = mapa novo. |
| PDFs e laudos da raiz: `Laudo_MPU6500.pdf`, `PS-MPU-9250A-01-v1.1.pdf`, `PROVA_MPU6500_NAO_9250.md`, `dados_testes_flow_pmw3901.txt`, `PLANO_HEADLESS_2026-05-22.md`, `AUDITORIA_*.md` (8 arquivos) | História do robô 1 — continua viva no repo original e no git log daqui. |
| `ESTADO_PROJETO.md` (conteúdo) | Zera: nasce um ESTADO novo do robô 2, mesmo método. |
| `docs/NAV2_GIRO_2026-06-03.md`, `docs/superpowers/specs/*` | Especificidades skid-steer/robô 1. |
| `worlds/` robô-1-específicos: `sala*.sdf`, `bolsao*.sdf`, `hotmilk_portas.*` | Cômodos do robô 1 + modelo sim 4-rodas skid. Fica só `empty.sdf`; sim do robô 2 (se rolar) nasce com modelo diferencial novo. |
| `setup_pi.sh` | NUC x86 usa o `setup.sh` (que já cobre desktop). |
| `pair-ps4.sh`, `bin/robot-pair-ps4`, `scripts/_bluez_fixes.sh` | Controle PS4 do robô 1. Se o robô 2 quiser PS4 um dia, cherry-pick de volta. |
| `controle_web/door_*.py` + testes + `robot_nav/door_crossing.py` | Máquina two-phase de porta amarrada às anotações do mapa `sala`. Se o ambiente novo tiver porta apertada, a fase 2 (FAST-LIO, pose melhor) provavelmente dispensa; cherry-pick se precisar. |

## ✅ FICA (intocado ou quase)

- **`controle_web/` inteira** (menos trekking/door): GUI, mapa, teleop, rotas,
  E-STOP, `nav_metrics` (+ checkpoint anti-apagão), `power_monitor` (robô 2
  também é hoverboard = mesmas leituras de tensão), `camera_service` (sem
  câmera fica quieto por design; se plugar uma USB, já funciona).
- **`robot_nav/`**: `motion_guard` (humano-prioridade + fantasma-de-parede),
  `unstuck_supervisor`, `path_follower`, `scan_sanitizer`, `freeze_capture`,
  `utils`. Todos consomem `/scan`+TF+`/plan` — cegos pra quantas rodas/qual lidar.
- **`wheel_msgs/`**, **`bin/`** (analyze_zigzag, pause_budget, robot-up,
  robot-connect, robot-key, map2world/world2map), `scripts/cpu_logger.sh`.
- **Configs Nav2** como ponto de partida (retune vem com o robô andando).

## 🔧 ADAPTA (o trabalho de verdade)

1. **`firmware/mega_bridge/`**: 2 motores (1 placa hover), remover IMU
   (MPU6050) e flow (PMW3901) do frame serial. Consultar pinagem antes
   (memória `project_mega_pinout`).
2. **`robot_nav/cmd_vel_to_wheels.py`**: cinemática diferencial 2 rodas.
   **⚠️ NÃO herdar os knobs anti-skid**: zona-morta 1.7, autoridade 6.0,
   `rot_min 2.4`, `slow_wz_cap 2.4`, spin_calib (0,6·(cmd−1,7)), proibição
   de arco — TUDO isso era atrito do skid 4 rodas. Diferencial gira fácil;
   começar com passthrough limpo e calibrar do zero.
3. **`pose_estimator.py`/`fused_odom.py`**: fase 1 = só odom de roda (flow e
   IMU externos não existem; a IMU do Mid-360 entra na fase 2 via FAST-LIO,
   fora deste nó). Simplificar, não deletar — a estrutura de publicar
   odom→base_link fica.
4. **`launch.sh`**: fora LD06 (serial+retry), IMU, flow; entra
   `livox_ros_driver2` (Ethernet, IP estático na NUC) +
   `pointcloud_to_laserscan` → `/scan`. Caminhos/systemd da NUC.
5. **Params do path_follower/collision**: os defaults atuais assumem skid
   (ex.: turn_enter/exit 16/7 calibrados pro robô 1). Manter como chute
   inicial MAS marcar tudo que é "constante de skid" pra recalibrar.
6. **`README.md`** + **`ESTADO_PROJETO.md`** novos, enxutos.

## ❓ DECIDIR com o dono (não bloqueia o resto)

- **Sim (Gazebo) pro robô 2?** Modelo diferencial é mais simples que o skid;
  vale montar quando houver mapa do ambiente novo (world2map/map2world ficam).
- **Rede do robô 2**: hotspot próprio? IP fixo? (repete-se a receita do robô 1).
- **Hostname/ssh da NUC** (workflow dev→git→pull continua igual ao da Pi).

## Ordem de execução

1. ✅ Plano revisado e aprovado pelo dono (07-14).
2. ✅ Demolição executada em 3 fatias (07-14): A `f406bed` (deletes puros),
   B `fe48a86` (trekking/cone), C `2945725` (porta). 274 testes verdes.
   Fósseis conscientes anotados no ESTADO (unstuck door-standdown,
   cone_pose_fix).
3. ⏳ Adaptações 1-3 (bridge 2 motores + cinemática + pose) com testes.
4. ⏳ Launch/Livox quando a NUC estiver acessível.
