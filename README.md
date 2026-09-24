# Controle_robo_livox

Robô móvel **diferencial** — 2 rodas de hoverboard **na frente**, roda boba
**atrás** — com LiDAR 3D **Livox Mid-360** e computador **Intel NUC**. Projeto
**PIBIT** — pesquisa documentada com rigor: decisões em `docs/decisoes/`,
diário em `docs/DIARIO.md`, bibliografia em `docs/REFERENCIAS.md`.

> Estado vivo do projeto: **`ESTADO_PROJETO.md`**.
> Método de trabalho: **`CLAUDE.md`**.
> `MIGRACAO_LIVOX.md` é histórico — o plano de reforma que ele descreve foi
> superado pela decisão 003; vale pela demolição já executada.

## Arquitetura

Definida na **decisão 003** (`docs/decisoes/003-base-ros2control-e-lio.md`),
que revisa as decisões 001 e 002.

**Base — verificada em hardware, `ros2_packages/robot_base/`:**

- **Tração**: `ros2_control` falando serial direta com a placa de hoverboard
  (sem microcontrolador no meio) + `diff_drive_controller`.
- **Localização**: Mid-360 + **FAST-LIO** → `/Odometry`. Sem `/scan` 2D, sem
  mapa, sem AMCL — o ambiente não precisa ser mapeado antes de o robô andar.

**Movimentação — a escrever, do zero.** O objetivo atual do projeto. Regra de
projeto que vem da 003: publicar velocidade em **unidades SI reais** direto no
`diff_drive_controller`, **sem escada de ganhos** entre nós. E levar a sério a
geometria: motriz dianteira + boba traseira é **instável em rumo**, porque a
traseira só acompanha por arrasto.

**Interface humana**: GUI web (`controle_web/`), herdada do robô 1 e ainda
apoiada na stack antiga — sua integração com a base nova está pendente.

## Subir a base

```bash
./setup_livox.sh --perfil nuc         # ou notebook; traz/compila drivers (pede sudo)
source install/setup.bash
ros2 launch robot_base base.launch.py # robô LIGADO
```

O perfil identifica somente a máquina; o sensor é descoberto na rede ou
informado com `--lidar-ip`. Antes da primeira vez, ler
`ros2_packages/robot_base/config/README.md`. IP errado = driver sem nuvem = sem
`/Odometry`, e a falha pode parecer uma inicialização normal.

## Origem

Nasceu como clone (histórico git completo) do
[`Controle_robo_web`](https://github.com/LGSantarosa/Controle_robo_web) —
robô 1, skid-steer 4 rodas com LiDAR 2D. O `git log` anterior a 2026-07-14
documenta as decisões herdadas; a demolição do que não se aplica está
registrada em `MIGRACAO_LIVOX.md` e `docs/decisoes/000-heranca-do-robo1.md`.

## Estrutura

- `ros2_packages/robot_base/` — **base do robô**: tração + localização, launches
- `ros2_packages/hoverboard_driver/` — interface `ros2_control` da placa hover
- `controle_web/` — servidor Flask + GUI web (mapa, teleop, rotas, métricas)
- `ros2_packages/robot_nav/` — nós herdados do robô 1. **Legado**: `mega_bridge`
  e `cmd_vel_to_wheels` foram substituídos pela base nova, mas seguem no repo
  porque a GUI e os testes ainda os referenciam (ver `ESTADO_PROJETO.md`
  § Fósseis conscientes)
- `firmware/mega_bridge/` — legado do robô 1; este robô não tem microcontrolador
- `bin/` — ferramentas de análise (CSV) e operação
- `docs/` — decisões, diário e referências (material do artigo)
- `setup_livox.sh` — traz os drivers de terceiros (não versionados) e o SDK Livox

## Status

🚧 **Base de pé** (tração + localização); **movimentação em aberto** — o robô
anda, mas ainda não anda *direito*. Ver `ESTADO_PROJETO.md` § Próximos passos.
