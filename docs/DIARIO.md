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

## 2026-07-15 (2ª leva) — Meta final, time de 3, rota 2D→LIO

- Dono explicitou a **meta final do projeto**: robô 2 indo de ponto a ponto
  sem bater, igual ao robô 1 — nada além disso. E o contexto novo: o
  trabalho passa a ser de **3 pessoas** (dono ~6 meses de ROS; os outros
  dois nunca mexeram).
- Pergunta que abriu a sessão: "se eu der deploy hoje, o que temos?" —
  resposta mapeada nó a nó: sobe GUI + cadeia de comando inteira, mas o
  `mega_bridge` morre (sem `/dev/mega`) e o Livox é placeholder → painel
  funcionando, robô surdo e paralítico. As duas pontas de HW (BO-1 e driver
  Livox) são exatamente o que falta.
- **Decisão 002** (registro próprio): clone 2D primeiro (receita comprovada
  do robô 1), LIO 3D depois como evolução comparada. Revisa a ORDEM da 001
  mantendo o conceito; racional: com time iniciante e robô inerte, o risco
  dominante virou "nunca chegar ao LIO por falta de base", e o clone 2D é o
  baseline experimental que o artigo precisa de qualquer jeito.
- Escada de marcos M0–M5 registrada no ESTADO; frentes A (base/motores),
  B (percepção/Livox), C (infra/GUI) esboçadas — divisão real e método de
  trabalho a 3 (branches, revisão cruzada) a combinar com o time presente.
- Recomendação pros 2 novatos antes de pegar frente: tutoriais oficiais de
  ROS 2 Jazzy (CLI tools + client libraries, ~2 dias).

## 2026-07-24 — A base sai do papel: tração e localização verificadas

**O que mudou de verdade hoje:** o robô deixou de ser um projeto e passou a ser
uma máquina com base funcionando. Tração e localização foram colocadas de pé e
verificadas em hardware. Isso derrubou as duas incógnitas que travavam o
projeto — registrado na **decisão 003**.

### BO-1 fechado

A placa hover fala **serial direta com o PC**, sem microcontrolador no meio
(`0xABCD` @115200). A candidata B (reintroduzir um MEGA) morre: adicionaria
firmware, latência e ponto de falha para resolver algo já resolvido.

### O erro que mais custou tempo (anotar para não repetir)

O quadro de realimentação da placa tem **18 bytes**, não 26. Com a estrutura
errada o checksum nunca fecha → posição das juntas fica `NaN` → o controlador
rejeita **todo** comando ("non-finite error value") → **as rodas não giram**.
O sintoma é "não funciona" sem nenhum erro que aponte para a serial. O que
isolou isso foi ler a serial crua, fora do ROS, e conferir o checksum no fio.

**Lição de método:** quando a cadeia ROS inteira sobe sem erro e o hardware não
reage, descer abaixo do ROS e olhar o byte. Vale instrumentar isso em CSV.

### Zona-morta do motor

Abaixo de ~`speed 100`/1000 a roda não vence o atrito. A compensação escala as
**duas** rodas juntas até a de maior magnitude cruzar o piso. Escalar cada roda
por si (o jeito óbvio) sobe também a roda interna da curva, destrói a razão
entre elas e o robô **abre** a curva em vez de fechar — o "balão".

### O que NÃO trouxemos, e por quê

A camada de movimentação foi recusada. Não por ajuste ruim: ela empilhava
ganhos multiplicativos em três nós antes do controlador, que tinha teto de
velocidade. O mínimo produzido pela regra "desacelere quando desalinhado" já
saía **acima do teto** — ou seja, o robô andava sempre a fundo, alinhado ou
não, com o giro saturado. Reto a fundo + giro saturado = **anda em S** e não
fecha curva. É exatamente o sintoma que o dono relatou.

A geometria agrava: **motrizes na frente, roda boba atrás**. A traseira só
acompanha por arrasto e amplifica oscilação de rumo (o inverso do carrinho de
supermercado, estável porque a boba vai à frente). Isso passa a ser um dado de
projeto do controlador, não um detalhe.

**Regra que fica:** o controlador de movimentação publica em **SI real** direto
no `diff_drive_controller`, sem escada de ganhos. Limite mora em um lugar só.

### Feito nesta sessão (só arquivo, robô desligado)

- `ros2_packages/hoverboard_driver/` — interface `ros2_control` + diferencial.
  **Compila limpo no Jazzy** nesta máquina (só warnings de API depreciada).
- `ros2_packages/robot_base/` — pacote novo que amarra a base:
  `tracao.launch.py`, `localizacao.launch.py`, `base.launch.py`, e a config de
  rede do Mid-360 versionada (`config/`, com README explicando o porquê).
- `setup_livox.sh` — traz `livox_ros_driver2` e `FAST_LIO` em commits fixados,
  faz o preparo pró-ROS2 do driver, instala a config de rede e compila.

### Percalços do setup (ambos resolvidos e documentados no script)

1. O `livox_ros_driver2` **não embute** o SDK nativo da Livox — exige
   `liblivox_lidar_sdk_shared.so` em `/usr/local/lib`. Sem isso o colcon falha
   com "Could not find LIVOX_LIDAR_SDK_LIBRARY", que não diz o que fazer.
   O script passou a clonar e compilar o SDK.
2. O SDK **não compila no Ubuntu 24.04** (GCC 13): vários headers usam
   `std::uint8_t`/`uint64_t` sem incluir `<cstdint>`, que o GCC 13 deixou de
   puxar transitivamente. Contornado com `-include cstdint` no CMAKE_CXX_FLAGS
   — resolve todos os arquivos de uma vez, sem editar fonte de terceiro.

### Pendências imediatas

- **Instalar o SDK** (`sudo` em `/usr/local`) — único passo que exige o dono;
  a compilação já está feita em `third_party/Livox-SDK2/build`.
- **IP do lidar a confirmar**: já foram vistos dois valores neste robô
  (`.169` e `.158`). Varredura da sub-rede procurando OUI `e4:7a:2c` é a fonte
  da verdade. IP errado = `bind failed` = FAST-LIO sem nuvem = sem `/Odometry`,
  falha silenciosa.
- **Medir `wheel_separation` e `wheel_radius` com trena** — os valores no
  controlador são herdados, não medidos. Erram odometria e conversão de comando.
- **Escrever a movimentação**, que é o objetivo declarado do dono: fazer o robô
  andar direito. Precisa de robô ligado para iterar.

## 2026-07-24 (2ª leva) — Simulador do robô 2

Sem o robô em mãos, montamos o robô 2 no Gazebo para poder ajustar movimentação.
Decisão de projeto em **004**: o simulador só serve se **errar do mesmo jeito
que o robô**. Simulador que anda perfeito faria a gente ajustar contra um robô
que não existe.

### Geometria (informada pelo dono, robô ainda não medido)

Caixa de madeira 0,50 × 0,50 × 0,30, fundo a 0,10 do chão; motrizes de
hoverboard **na frente** separadas por 0,20; boba **no centro da traseira**.
Massa total 10 kg — "ele é leve".

### As três coisas que fazem o simulador não mentir

1. **Boba com trail e atrito no pivô.** O jeito fácil (esfera lisa) não tem
   orientação, então nunca precisa dar meia volta pra acompanhar e nunca empurra
   a traseira. O deslocamento de 4 cm entre eixo do pivô e contato é a **causa
   física** de a traseira sair no giro. Tem teste que falha se isso for zerado.
2. **Pose do chão, não das rodas.** `/Odometry` publica a pose verdadeira do
   Gazebo — mesmo papel do LIO no robô. Odometria de roda reportaria o movimento
   *comandado* e esconderia justamente a derrapada que queremos ver.
3. **Mesma cadeia de controle.** Mesmo `diff_drive_controller`, mesmos tetos de
   velocidade — foi um teto engolindo a modulação que causou o defeito original.
   Só a camada de hardware muda (`gz_ros2_control` no lugar da placa serial).

### Verificado

- URDF válida; 11 testes travam geometria, massa e a concordância entre URDF e
  YAML do controlador. Suíte total: **285 verdes** (274 + 11).
- **Física conferida no Gazebo**: robô inserido no mundo assenta sobre as três
  rodas em z≈0, estável, sem pular nem afundar.

### Onde parou

Não deu pra dirigir o robô ainda: falta `ros-jazzy-gz-ros2-control` (está no
apt, precisa de sudo). Tentei validar a boba aplicando torque de guinada pelo
serviço de wrench do Gazebo, mas o mundo não carrega o plugin que oferece esse
serviço — e adicionar um plugin só pra teste seria muleta, já que o
`gz_ros2_control` é necessário de qualquer forma para o trabalho real.

**Próximo passo:** instalar o pacote, dirigir o robô e conferir se o S aparece.
Se **não** aparecer, o simulador está otimista e os primeiros suspeitos são o
atrito do pivô da boba (baixo demais) e o trail (curto demais) — ambos no bloco
de propriedades no topo da URDF.
