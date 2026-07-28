# Estado do Projeto — Controle_robo_livox (PIBIT)

> Documento vivo. Resumo do que está acontecendo, BOs abertos, avanços e o que falta.
> Versionado na `main`. Atualizado em **2026-07-28**.
>
> **Este projeto é um PIBIT** — vai virar artigo. Toda decisão técnica tem um
> registro em `docs/decisoes/`, todo dia de trabalho entra no `docs/DIARIO.md`,
> e escolhas de abordagem são embasadas em literatura (`docs/REFERENCIAS.md`).
> Ritmo deliberadamente devagar: 1 mudança pequena por vez.

---

## O robô

- **Tração**: 2 rodas de hoverboard (diferencial) **na frente** + roda boba
  **atrás** (corrigido 07-24: o ESTADO dizia boba na frente). 1 placa
  hoverboard hackeada ligada **DIRETO no PC** por serial — sem Arduino MEGA.
  Protocolo `0xABCD` @115200, realimentação de **18 bytes**.
  ⚠️ Essa geometria (motriz dianteira, boba traseira) é **instável em rumo**:
  a traseira só acompanha por arrasto e amplifica oscilação. É dado de projeto
  do controlador, não detalhe.
- **Sensor**: Livox Mid-360 (LiDAR 3D 360°, IMU embutida, conexão Ethernet).
  Único sensor externo — sem câmera, sem IMU externa, sem optical flow.
  Também sem eletrônica auxiliar: sem relé de luz, LED de marco ou botão.
- **Computador**: Intel NUC (x86), Ubuntu 24.04 + ROS 2 Jazzy. Interface do
  lidar: `enp2s0` (IP `192.168.1.2`).
- **Ambiente**: novo (não é o do robô 1). **Não precisa de mapa** — a
  localização é LIO, sem AMCL (decisão 003).
- Estado físico: montado, mas **a elétrica está ruim** (07-27: tentativa de
  medição no laboratório abortada por isso). **Base de software (tração +
  localização) verificada em hardware**; movimentação e navegação ponto a
  ponto escritas e verificadas em simulador, à espera dos números do robô.

### Medidas ainda NÃO conferidas (afetam tudo acima)

`wheel_separation: 0.32` e `wheel_radius: 0.0825` no controlador diferencial
são valores **herdados, não medidos neste robô**. Erram a odometria de roda e
a conversão do comando em rad/s. **Medir com trena** antes de calibrar
movimentação.

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

## 2026-07-15 (2ª leva) — Meta final + time de 3 + rota 2D→LIO (decisão 002)

- **Meta final explicitada pelo dono**: robô 2 indo de um ponto a outro sem
  bater, igual ao robô 1. É o critério de sucesso.
- **Time vira 3 pessoas**: dono (~6 meses de ROS) + 2 iniciantes totais.
  Divisão em frentes: **A** base/motores · **B** percepção/Livox ·
  **C** infra/GUI. Organização do trabalho a 3 (branches, revisão cruzada,
  DIARIO com autor) a combinar com o time.
- **Decisão 002** (`docs/decisoes/002-rota-2d-primeiro-lio-depois.md`):
  clone 2D primeiro (receita do robô 1: /scan derivado + AMCL + Nav2),
  LIO 3D depois como evolução comparada — o baseline 2D vs LIO vira
  resultado do artigo. Revisa a ordem da 001; o conceito (GUI 2D pro
  humano) permanece.

## 🚀 2026-07-24 — Base de pé: tração + localização (decisão 003)

O robô deixou de ser projeto e virou máquina com base funcionando. **Decisão
003** (`docs/decisoes/003-base-ros2control-e-lio.md`) fecha o BO-1 e revisa a
001 e a 002.

- **Tração**: `ros2_packages/hoverboard_driver/` — interface `ros2_control`
  falando serial direta com a placa + `diff_drive_controller`. Compila limpo
  no Jazzy. **Substitui** `mega_bridge.py` e `cmd_vel_to_wheels.py`.
- **Localização**: Livox Mid-360 + **FAST-LIO** → `/Odometry`. Sem `/scan` 2D,
  sem mapa, sem AMCL — a rota "2D primeiro" da decisão 002 caiu.
- **`ros2_packages/robot_base/`** (novo) amarra as duas:
  `ros2 launch robot_base base.launch.py`.
- **`setup_livox.sh`** (novo) traz os drivers de terceiros em commits fixados,
  compila/instala o SDK nativo da Livox e aplica a config de rede do lidar.
- **Movimentação NÃO foi herdada** — decisão explícita do dono. A camada
  existente andava sempre a fundo (o teto do controlador engolia a
  desaceleração por desalinhamento) com o giro saturado → **anda em S** e não
  fecha curva. Vamos escrever a nossa, em SI real, sem escada de ganhos.

## 🧪 2026-07-24 (2ª leva) — Simulador (decisão 004)

Robô 2 montado no Gazebo Harmonic para ajustar movimentação **sem o robô**.
Premissa da decisão 004: o simulador só serve se **errar como o robô erra**.

- Caixa 0,50 × 0,50 × 0,30, fundo a 0,10 do chão; motrizes de hoverboard na
  frente (separação 0,20); boba no centro da traseira; **10 kg**.
- **Boba com trail de 4 cm e atrito no pivô** — é o que faz a traseira ser
  jogada pra fora no giro. Esfera lisa (o jeito fácil) não reproduz nada.
- **`/Odometry` = pose verdadeira do Gazebo**, mesmo papel do LIO. Odometria de
  roda esconderia a derrapada.
- **Mesmo `diff_drive_controller` e mesmos tetos** do robô real; só a camada de
  hardware muda. Sem isso o ajuste não transfere.
- Arquivos: `ros2_packages/robot_base/description/robo2.urdf.xacro`,
  `worlds/pista_livre.sdf`, `launch/sim.launch.py`,
  `config/hoverboard_controllers_sim.yaml`.
- **Verificado**: física estável no Gazebo (assenta nas 3 rodas em z≈0);
  11 testes travam geometria/massa. Suíte: **285 verdes**.
- ⚠️ Falta `ros-jazzy-gz-ros2-control` (apt, precisa de sudo) para dirigir.

## 🔬 2026-07-27 — O S explicado, e a lei que o elimina

O simulador virou instrumento de medida. `gz-ros2-control` instalado, robô
dirigível, e o defeito **reproduzido e explicado com número**.

- **O S apareceu**: com o controlador velho (linear no teto, giro saturado no
  sinal do erro), o rumo vira **ciclo-limite** — ±18°, período 2,8 s, 21 cm de
  serpenteado a cada 1,9 m percorridos, **sem decair**.
- **A causa é a rampa de desaceleração, não a boba.** Entre "o erro cruzou
  zero" e "o giro parou" existe uma distância de frenagem de rumo de
  `wz²/(2·a_dec)`. Medido em malha aberta: comando cortado a wz=1,0 rad/s e o
  robô **girou mais 25°**. Previsto pela fórmula: 0,276 rad; medido: 0,34 rad.
  A derrapada da boba responde por ~20% do S; a rampa, por ~80%.
- **A lei que resolve** (validada, ainda não implementada):
  `wz = sinal(e)·min(wz_max, √(2·a_dec·|e|))`, linear cedendo com `cos(e)` —
  nunca pede mais giro do que consegue frear no erro que ainda falta.
- **O dono julgou o S do simulador FRACO** perto do robô real (barriga de ~50 cm
  contra os ±10 cm daqui). A barriga escala com `a_dec^-1,43`, o que estima o
  **`a_dec` real em ~0,5 rad/s²** — um terço do que está no YAML. A estimativa
  não vale como medida: entra no banco de ensaios.

### Estresse da lei — 10 corridas na planta degradada (a_dec = 0,3)

Meia-volta de 180°, alvo trocando de sinal, velocidade baixa, malha a 10 Hz
(taxa do robô real) e zona morta injetada: **sobrepasso entre 0,1° e 0,8° em
todos**, contra os ±47° do controlador velho. O S não voltou em nenhuma.

Três achados que não estavam no pedido:

1. **Errar o `a_dec` pra baixo é de graça.** Chutando 3x menos que a planta
   entrega: sobrepasso zero e 4,5 s de assentamento, contra 4,8 s com o valor
   exato. Chutar pra cima é que traz o S de volta (com 5x otimista: oscila,
   mas **decai** — degrada, não quebra).
2. **A zona morta é um precipício, não uma ladeira.** Com zona morta de roda em
   0,10 m/s a meia-volta trava 0,1 s e completa; em **0,15 m/s o robô fica 22 s
   parado**, 100% das amostras, com o controlador pedindo 1,0 rad/s. Ver BO-3.
3. **Defesa dimensionada**: `v_piso = zona_morta + wz_max·bitola/2 + margem`,
   confirmada nos dois valores. O piso não é número solto — depende da zona
   morta MEDIDA.

### Banco de ensaios (`tools/banco/`)

Roda igual no robô e no simulador (mesmos tópicos, mesmo CSV), o que torna os
dois diretamente comparáveis. `README.md` traz o protocolo de caracterização:
zona morta linear e de giro, degrau de giro (`a_dec`), curva sustentada em
várias velocidades e aceleração linear.

## 🧭 2026-07-27 (2ª leva) — Navegação ponto a ponto (decisão 006)

A pilha de movimento própria do robô 2 fechou a fatia A: **ir a um ponto**, sem
obstáculo. Vive em `ros2_packages/robot_motion/` — separada do `robot_nav`, que
ainda guarda os fósseis do robô 1.

- **Rumo alvo = direção até o ponto, recalculada todo ciclo.** Dissolve o erro
  lateral sem controlador extra: se o robô sai da linha, a direção muda e ele
  curva de volta. Resolve os 66 cm de desvio paralelo que o controlador de
  rumo sozinho deixava.
- **Aproximação = a lei de frenagem em distância**: `v = √(2·a_lin·dist)`.
  Mesmo princípio da decisão 005, mesmo tipo de parâmetro físico.
- **Chegada com piso e corte firme.** Não existe "chegar devagarinho": abaixo
  do mínimo viável a placa engole o comando e o robô para longe do ponto
  achando que chegou — o BO-3 disfarçado de sucesso. O nó recusa raio de
  chegada menor que a distância de parada a partir do piso (senão orbita o
  ponto) e avisa no log.
- **Camadas conversam pelo tópico público** (`rumo_alvo`, `velocidade_alvo`):
  a navegação decide *para onde*, a movimentação decide *o que o atuador
  aguenta*.
- **Verificado no simulador**, os dois nós reais empilhados: alvo em (2, 2) a
  2,83 m — chegou e **parou a 7 mm do ponto**, 45 s sem orbitar; alvo em
  (−1, 1), 135° atrás — chegou a **8 mm**.
- **Quatro defeitos achados clicando no RViz** (o dono, em 5 minutos, achou o
  que 10 corridas roteirizadas não acharam): um ponto a 0,65 m era **orbitado
  para sempre**. Causas, todas medidas: (1) a banda morta tem duas saídas e só
  uma estava programada — o pivô era proibido por aritmética; (2) a velocidade
  não tinha teto pela curva; (3) **a linear cedia pelo erro do BICO num robô
  que escorrega** — em órbita o bico ficava a 50° do alvo e o movimento a 87°,
  **37,5° de deriva**, aproximação zero; (4) chegando, ele continuava girando e
  se arrastava para fora (0,06 m viravam 0,27 m).
- **Depois dos quatro**: o ponto de 0,65 m fecha a 0,059 m — ele **para, pivota
  no próprio eixo** e só então arranca. Alvo à frente (2, 2): 9 mm. Alvo atrás
  (−1, 1): 8 mm em 9,8 s, com afastamento máximo de 1,67 m (era 2,15 m num
  laço andando). Parado no ponto: 9 mm de deriva em 30 s.
- **O simulador ganhou uma PLACA FINGIDA** (`robot_base/placa_simulada`), a
  pedido do dono: ela engole comando de roda pequeno demais, como a de verdade.
  Antes eu rodava com zona morta zero — fiel ao Gazebo e infiel ao robô. Agora
  o controle é desenvolvido contra uma zona morta plausível, e quando a bancada
  medir a real troca-se só o número. `sim.launch.py zona_morta:=0.10`.
- **A planta lenta virou perfil versionado** (`planta:=lenta`, padrão), fechando
  uma dívida de reprodutibilidade: ela vivia num arquivo solto e quem subisse o
  simulador pelo caminho oficial pegava a planta ágil e veria um robô melhor do
  que o real.
- **⚠️ Com a zona morta ligada, o pivô some — e a BITOLA é quem decide.**
  O pivô exige `wz_max·bitola/2 ≥ zona_morta + margem`. Com a bitola do modelo
  simulado (0,20) e zona morta 0,10, pivotar exigiria **1,3 rad/s** contra um
  teto de 1,0: impossível, o robô volta a fazer só arcos e **orbita pontos
  próximos** (chegou a 0,168 m de um alvo com raio de chegada de 0,15). Com a
  bitola real presumida (0,32), o mesmo caso dá 0,62 rad/s e o pivô **existe**.
  O nó diz qual dos dois é o caso, em voz alta, na subida.
  **Isso torna medir a bitola com trena mais urgente que medir a zona morta.**
- **Falta**: desviar de obstáculo (fatia B) — depende do Livox e de percepção
  que o repo ainda não tem.

## 🔙 2026-07-28 — A ré como manobra (decisão 007), e a boba do simulador cai

O ponto perto e de lado deixou de ser inalcançável, e uma premissa da decisão
004 caiu no mesmo dia.

- **A ré entrou** (`docs/decisoes/007-re-como-manobra.md`): quando o alvo exige
  um raio menor do que o robô consegue fazer (`d/(2·sen e) < raio_min_curva`),
  ele **recua reto** até a geometria abrir, e então entra normal. Gatilho
  geométrico — decidido antes de orbitar, não depois.
- **Ré reta por decisão do dono.** Curvar de ré é a manobra sem medida nenhuma:
  andando para trás a boba vira roda dianteira. Reta ainda mantém a zona morta
  simétrica.
- **A lei de rumo NÃO mudou**: `linear_de_avanco` segue com `max(0, cos e)` e o
  `test_nunca_anda_de_re` segue verde. A ré é modo à parte, acionado por
  **velocidade negativa** no tópico que já existia — sem tópico novo.
- **Com orçamento e com voz**: histerese de 1,3× para sair, teto de 1,0 m e 8 s
  para a manobra inteira. Estourou, para e grita com os números.
- **Verificado**: o alvo a 0,65 m de lado, que orbitava a 0,168 m para sempre,
  recua 9 cm em duas mordidas e **chega** (0,150 m, parado 30 s). Alvo (2, 2)
  continua em 8 mm sem acionar ré. **338 testes verdes** (eram 325).
- ⚠️ **Com os parâmetros de hoje o pivô não existe em NENHUM dos dois perfis** —
  no real, pivotar exigiria 1,25 rad/s contra teto de 1,0 (`zona_morta` 0,15,
  `bitola` 0,32). Medida a bitola, `raio_min_curva: 0` desliga a ré sozinho.
- **A boba do simulador é decorativa** — ver BO-4. Consequência imediata: a ré
  foi validada só no simulador, e a única coisa que preocupa nela (a boba
  virando roda dianteira) é justamente o que aquele modelo não pode mostrar.

## ⏳ Próximos passos

**Primeiro, com o robô (virou prioridade — a movimentação depende destes
números e hoje eles são chute):**

1. **Medir `wheel_separation` e `wheel_radius` com trena.** O
   `diff_drive_controller` usa os dois pra converter comando em rad/s de roda:
   errar aqui erra todo ensaio abaixo. (0,20 no simulador × 0,32 no controlador
   real; nenhum dos dois medido.)
2. **Rodar o protocolo de `tools/banco/README.md`** — zona morta, `a_dec`,
   curva por velocidade, aceleração, **e o ensaio 6 (reta com cutucão, ida ×
   ré)**, que é o que valida a manobra da decisão 007 e ataca o BO-4. O dono só
   roda; os CSV vêm por ssh.
3. **Confirmar o IP do lidar** — varredura procurando OUI `e4:7a:2c`. Já foram
   vistos `.169` e `.158`. Errado = `bind failed` = sem `/Odometry`, falha
   silenciosa. Ver `ros2_packages/robot_base/config/README.md`.

**Sem o robô:**

4. **Camada de movimentação** com a lei de frenagem, o piso de linear e o
   detector de plantão (BO-3). Parâmetros em SI num lugar só, conservadores
   até os ensaios chegarem.
5. **Calibrar o simulador contra o robô** com os números dos ensaios —
   critério: mesma manobra, S de tamanho parecido.
6. **Navegação** (ponto a ponto) por cima da movimentação. É dela o problema do
   erro lateral: o controlador de rumo trava o rumo mas segue paralelo à rota,
   deslocado — medido em 66 cm depois de uma meia-volta. E é dela também o giro
   parado, que a zona morta proíbe abaixo de ~1 rad/s.

## Fósseis conscientes (remover em fatia própria)

`robot_nav/mega_bridge.py` e `robot_nav/cmd_vel_to_wheels.py` deixaram de ser
caminho (decisão 003) mas **não foram apagados**: a GUI e a suíte de testes
ainda os referenciam, e remover tudo junto quebraria as duas. Também seguem
inertes o standdown de porta no `unstuck_supervisor` e o `cone_pose_fix.py`.

## BOs abertos

- ~~**BO-1 — Arquitetura da ponte PC↔placa hover**~~ ✅ **FECHADO 07-24**:
  serial direta, sem microcontrolador. Registro na decisão 003.
- **BO-2 — O que o artigo compara** (aberto 07-24): a decisão 003 tirou do
  caminho o baseline 2D vs LIO que a 002 previa como resultado. Candidatos:
  comparar métodos de LIO entre si, ou comparar estratégias de controle de
  rumo para esta geometria (motriz dianteira + boba traseira) — que é o
  problema real em mãos. **07-27: o segundo candidato ganhou corpo** — o S
  está explicado por `wz²/(2·a_dec)` e há uma lei que o elimina, com 10
  corridas medidas. Falta a comparação valer no robô.

- **BO-3 — Zona morta do atuador** (aberto 07-27): comando abaixo da zona morta
  deixa o robô **parado sem erro nenhum** — nó vivo, tópico publicando, log
  limpo, máquina imóvel. Já custou horas de depuração na competição de 2025.
  No simulador é precipício: zona morta de roda em 0,10 m/s passa raspando,
  em 0,15 m/s o robô fica 22 s plantado com o controlador pedindo 1,0 rad/s.
  **O valor real é desconhecido, e a faixa provável de uma placa de hoverboard
  cai bem em cima do precipício.**

  Defesa desenhada, em duas partes — porque prevenção pode falhar (bateria
  fraca, carga, piso diferente) e o custo real do defeito é o tempo de
  diagnóstico:
  1. *prevenir* — `v_piso = zona_morta + wz_max·bitola/2 + margem`;
  2. *delatar* — se há comando de movimento e a pose do LIO não muda por ~0,5 s,
     gritar no log com pedido e efetivo. Nunca parar em silêncio.

  **Fecha quando:** (a) zona morta medida na bancada (`tools/banco`, ensaios 1
  e 2); (b) `a_dec` medido (ensaio 3); (c) `v_piso` calculado pela fórmula com
  esses números; (d) meia-volta no robô real completando sem travar.

  Some junto o caso não resolvido: **girar parado devagar é impossível** —
  abaixo de `2·zona_morta/bitola` as duas rodas ficam na banda proibida. Isso
  não é ajuste de ganho, é limite físico, e cai no colo da navegação.
  **07-28: o alcance disso foi resolvido pela ré** (decisão 007) — o robô
  contorna a falta de pivô recuando. O limite físico continua de pé.

- **BO-4 — A boba do simulador não é uma boba** (aberto 07-28): o garfo do
  pivô **não se alinha com a direção de movimento**. Medido numa curva pra
  frente (v=0,25, wz=0,6, raio 0,42 m): ele deveria assentar a ~157° do corpo
  (`atan(0,18/0,42)` fora do eixo) e ficar lá; em vez disso saiu de 180° e
  girou continuamente até 38°, mantendo o rumo do **mundo**. É um patim, não
  uma boba.

  Multiplicar o atrito da boba por 16 (`mu 0,05 → 0,8`) mudou o rumo da mesma
  curva de 136,161° para 136,675° — **0,4%**. O contato dela não participa da
  dinâmica, e a hipótese do `mu2` baixo como causa foi testada e **descartada**.
  A causa real é desconhecida.

  **Custo:** a decisão 004 apoia-se em "trail de 4 cm + atrito no pivô
  reproduzem a traseira jogada pra fora", e o S de 27-07 foi atribuído ~20% à
  boba. Essa atribuição não se sustenta: a derrapada que o simulador mostra vem
  do `mu` baixo do contato, não da geometria de boba. Some junto a validação da
  ré (decisão 007), que no simulador não testa nada — lá ré e ida deram
  idênticas porque não há boba para virar.

  **Fecha quando:** (a) ensaio 6 do banco rodado no robô real, ida × ré, com a
  boba filmada; (b) a causa do garfo não alinhar identificada no modelo; (c)
  simulador reproduzindo o ângulo de boba medido no robô, ou a decisão 004
  corrigida para dizer o que ele de fato reproduz.
