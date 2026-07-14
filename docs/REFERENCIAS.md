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
