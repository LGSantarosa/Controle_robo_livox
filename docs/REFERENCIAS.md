# Referências — Controle_robo_livox (PIBIT)

> Bibliografia anotada: o que é, por que importa pra NÓS, status de leitura.
> Entrada mínima: citação + link + 1 parágrafo de relevância.

## Odometria/Localização LiDAR-inercial 3D (decisão da fase 2 — ⏳ ler antes de decidir)

- **FAST-LIO2** — Xu, W.; Cai, Y.; He, D.; Lin, J.; Zhang, F. "FAST-LIO2:
  Fast Direct LiDAR-inertial Odometry". IEEE T-RO, 2022.
  https://github.com/hku-mars/FAST_LIO — Odometria LIO direta (sem extração
  de features), ikd-Tree; suporte nativo a Livox e usa a IMU embutida do
  Mid-360. Candidata principal pra substituir AMCL+odometria de roda.
  Status: ⏳ a ler.
- **Point-LIO** — He, D.; Xu, W.; et al. "Point-LIO: Robust High-Bandwidth
  LiDAR-Inertial Odometry". Advanced Intelligent Systems, 2023.
  https://github.com/hku-mars/Point-LIO — atualização ponto-a-ponto (sem
  acumular scan), robusta a movimento agressivo/saturação de IMU.
  Status: ⏳ a ler.
- **LIO-SAM** — Shan, T.; et al. "LIO-SAM: Tightly-coupled Lidar Inertial
  Odometry via Smoothing and Mapping". IROS 2020.
  https://github.com/TixiaoShan/LIO-SAM — LIO com pose-graph e loop closure;
  exige IMU de 9 eixos própria e ajustes pra Livox (padrão é Velodyne).
  Status: ⏳ a ler.

## Placa de hoverboard (atuador dos robôs 2 e 3)

- **hoverboard-firmware-hack-FOC** — EFeru. https://github.com/EFeru/hoverboard-firmware-hack-FOC
  — firmware das placas (protocolo `0xABCD`, 115200). Lido em 2026-09-15:
  `Inc/config.h` padrão `CTRL_TYP_SEL FOC_CTRL`, `CTRL_MOD_REQ VLT_MODE`,
  `FIELD_WEAK_ENA 0`, `I_MOT_MAX 15`, `N_MOT_MAX 1000`; `VARIANT_USART`
  `DEFAULT_STEER_COEFFICIENT 8192` (0,5) e `DEFAULT_SPEED_COEFFICIENT 16384`
  (1,0); `Src/main.c` `mixerFcn(speed << 4, steer << 4, &cmdR, &cmdL)` e
  `pwmr = -cmdR` sem `INVERT_R_DIRECTION`. Base da hipótese do puxão crônico
  (diário 15-09) e do giro fraco (decisão 048). Status: lido; **a configuração
  gravada nas placas dos robôs 2 e 3 não é conhecida** — pode diferir do padrão.

## Stack de navegação (herdada do robô 1)

- **Nav2** — Macenski, S.; et al. "The Marathon 2: A Navigation System".
  IROS 2020. https://docs.nav2.org — base da navegação herdada (planner,
  costmaps, collision monitor). Status: usada em produção no robô 1.
- **pointcloud_to_laserscan** (pacote ROS) — projeção da nuvem 3D do Mid-360
  em `/scan` 2D pra fase 1 (reuso da stack 2D). Decidir faixa de altura de
  corte quando o robô estiver de pé. Status: ⏳ avaliar parâmetros.
- **livox_ros_driver2** — https://github.com/Livox-SDK/livox_ros_driver2 —
  driver oficial do Mid-360 (Ethernet, config por JSON de IPs).
  Status: ⏳ instalar na NUC.
