# Controle_robo_livox

Robô móvel **diferencial** (2 rodas de hoverboard + roda boba) com LiDAR 3D
**Livox Mid-360** e computador **Intel NUC**. Projeto **PIBIT** — pesquisa
documentada com rigor: decisões em `docs/decisoes/`, diário em
`docs/DIARIO.md`, bibliografia em `docs/REFERENCIAS.md`.

> Estado vivo do projeto: **`ESTADO_PROJETO.md`**.
> Plano de migração da base de código: **`MIGRACAO_LIVOX.md`**.
> Método de trabalho: **`CLAUDE.md`**.

## Arquitetura (conceito norteador — decisão 001)

- **Interface humana 2D**: GUI web (`controle_web/`) com mapa 2D — humano
  clica onde o robô deve ir. Herdada do robô 1, validada em campo.
- **Percepção/localização 3D**: o robô se localiza e navega usando a nuvem
  3D do Mid-360 (LIO — escolha do método na fase 2, com literatura).
- Nav2/localização/movimentação serão **repensadas do zero**; reuso apenas
  do que é comprovadamente igual (ponte de rodas, GUI, instrumentação CSV).

## Origem

Nasceu como clone (histórico git completo) do
[`Controle_robo_web`](https://github.com/LGSantarosa/Controle_robo_web) —
robô 1, skid-steer 4 rodas com LiDAR 2D. O `git log` anterior a 2026-07-14
documenta as decisões herdadas; a demolição do que não se aplica está
registrada em `MIGRACAO_LIVOX.md` e `docs/decisoes/000-heranca-do-robo1.md`.

## Estrutura

- `controle_web/` — servidor Flask + GUI web (mapa, teleop, rotas, métricas)
- `ros2_packages/robot_nav/` — nós ROS 2 (ponte MEGA, cinemática, comportamentos)
- `firmware/mega_bridge/` — firmware do Arduino MEGA (ponte placa hoverboard)
- `bin/` — ferramentas de análise (CSV) e operação
- `docs/` — decisões, diário e referências (material do artigo)

## Status

⏳ Migração em andamento — o robô ainda não roda com esta base. Ver
`ESTADO_PROJETO.md` § Próximos passos.
