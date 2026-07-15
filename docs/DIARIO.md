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

## 2026-07-15 — ADAPTA 1-2: descoberta que muda o plano (sem MEGA!)

Sessão de investigação pro ADAPTA 1-2 (bridge + cinemática). Nenhum código
escrito — a sessão terminou em correção de premissa, não em implementação.

**O que encontramos no código herdado:**

- `cmd_vel_to_wheels.py` **já é diferencial puro**: L/R por `wheel_base` +
  `linear_scale` + saturação, sem nenhum knob anti-skid (foram removidos
  ainda no robô 1). ADAPTA 2 deixa de ser "reescrever cinemática" e vira
  "calibrar 4 params com o robô": `wheel_base` (bitola real), `linear_scale`,
  `left/right_wheel_sign`.
- Protocolo da placa hoverboard hackeada mapeado a partir de
  `firmware/mega_bridge/{include,src}/hoverboard.{h,cpp}`: família
  NiklasFauth/EFeru FOC — comando `0xABCD | steer i16 | speed i16 | checksum
  XOR` @115200; feedback 18 bytes (`cmd1, cmd2, speedR_meas, speedL_meas,
  batVoltage, boardTemp, cmdLed, checksum`). O firmware da placa tem timeout
  próprio de comandos (motores param se o stream some) — a segurança não
  dependia só do watchdog da MEGA.

**A descoberta (corrige o ESTADO):** o robô 2 **não tem Arduino MEGA** e não
tem nenhuma eletrônica auxiliar (sem relé, LED de marco, botão, IMU externa,
flow). Inventário real: 1 placa hover + rodas, Livox Mid-360, NUC, baterias
(rodas e computador). As rodas chegam "direto no PC" — a forma exata (que
adaptador, que firmware na placa) o dono ainda vai confirmar fisicamente.

**Nota histórica que pesa na decisão:** o docstring do `mega_bridge.py`
registra que ele substituiu um `ros2-hoverboard-driver` que falava direto
com uma única placa — ou seja, o robô 1 COMEÇOU sem MEGA; ela entrou pra
agregar a 2ª placa + IMU + flow + relé, nada do que existe no robô 2.

**Arquiteturas na mesa (decisão 002, pendente de bancada):**

- (A) ponte direta PC↔placa via USB-serial, nó `hoverboard_bridge.py` novo
  falando 0xABCD — sem firmware pra manter; recomendação do assistente;
- (B) reintroduzir uma MEGA como no robô 1 — exige comprar/instalar HW cuja
  razão de ser (agregação de sensores) não existe aqui.

Dono decidiu: **só decidir com a bancada na frente** — inspecionar como as
rodas chegam no PC antes de fechar. Decisão 002 só será redigida depois.

**Checklist da inspeção de bancada (próxima sessão hands-on, robô DESLIGADO
pra inspeção física; LIGADO só se formos ler a serial):**

1. Seguir o cabo das rodas: a placa hover conecta no PC como? USB direto
   (placa já tem conversor?) ou adaptador USB-TTL no meio? Qual chip/modelo
   do adaptador (CP2102, FT232, CH340…)?
2. Placa hover: foto, modelo/marcações da placa, qual conector serial está
   em uso (a UART "sideboard" do firmware hackeado).
3. Firmware gravado na placa: há etiqueta/anotação de quem gravou? Se nada
   indicar, teste vivo: ligar e escutar a serial a 115200 — o firmware
   EFeru/NiklasFauth cospe feedback `0xABCD` continuamente (dá pra confirmar
   com `hexdump` antes de qualquer nó ROS).
4. Nível TTL da UART da placa (3.3 V vs 5 V) — define que adaptador é seguro.
5. NUC: de quebra, confirmar SO (`lsb_release -a`) e habilitar ssh — destrava
   o resto do ESTADO.
