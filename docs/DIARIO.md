# Diário de pesquisa — Controle_robo_livox (PIBIT)

> Uma entrada por sessão de trabalho: o que foi tentado, o que funcionou,
> o que falhou E POR QUÊ. Fracasso documentado é resultado — vai pro artigo.
> Decisões formais têm registro próprio em `docs/decisoes/`.

## 2026-07-14 — Nascimento do repo

- Definido o caráter do projeto: PIBIT, artigo ao final, ritmo devagar,
  decisões embasadas em literatura.
- Repo criado como clone (histórico completo) do `Controle_robo_web` do
  robô 1; remote independente. Racional em `decisoes/000-heranca-do-robo1.md`.
- `MIGRACAO_LIVOX.md` escrito e aprovado pelo dono; demolição executada em
  3 fatias (commits `f406bed`, `fe48a86`, `2945725`), 274 testes verdes.
- Pendências pra próxima sessão: confirmar Ubuntu/ssh da NUC; começar
  ADAPTA 1 (mega_bridge 2 motores) — consultar pinagem da MEGA antes.

## 2026-07-14 (2ª leva) — Arquitetura-alvo + varredura LD06

- Dono definiu o conceito norteador: **GUI 2D pro humano, localização 3D pro
  robô**; nav/localização/movimentação do zero, embasadas — registrado em
  `decisoes/001-gui-2d-localizacao-3d.md`. A "fase 1 clone barato" (stack 2D
  herdada como alvo) foi descartada — teria mascarado a pesquisa.
- Varredura de sobras do robô 1 que a demolição não pegou: encanamento do
  LD06 (test_lidar.sh, lidar.launch.py, retry/watchdog serial do launch.sh,
  passo LiDAR do setup_udev.sh), teleop-pernas, README de 1406 linhas.
  launch.sh ganhou placeholder explícito do Livox (Ethernet) no estágio [3].
- Nenhum código novo escrito (regra: adaptação estrutural apenas).
