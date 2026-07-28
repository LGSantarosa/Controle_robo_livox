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

## 2026-07-27 — O S deixa de ser sintoma e vira número

Sessão inteira no simulador, usado como instrumento de medida e não como
brinquedo. Entrou o `ros-jazzy-gz-ros2-control` (apt, com sudo), o que
destravou dirigir o robô simulado.

### O defeito reproduzido

Manobra roteirizada: cruzeiro a 0,7 m/s, alvo de rumo 45° a partir de t=2 s,
controlador velho (linear no teto, giro saturado no sinal do erro).

Resultado: **ciclo-limite**, não sobrepasso que decai. Rumo oscilando entre
0,48 e 1,11 rad em torno do alvo — ±18° — com período de 2,8 s, e amplitude
igual do começo ao fim dos 12 s. Em unidades que se comparam de olho com o
robô: serpenteia ±10 cm em torno da reta, e completa um S a cada 1,9 m.

### A causa, com número

O degrau em malha aberta deu a medida: comando de giro cortado com wz=1,0
rad/s, e o robô **girou mais 0,44 rad (25°) depois do corte**. Desses, 0,33
rad são a rampa do limitador (`angular.z.max_acceleration: 1.5` → wz²/2a) e
~0,11 rad são atraso da planta.

Ou seja, existe uma **distância de frenagem de rumo** de ~0,28 rad. Qualquer
controlador que mande giro saturado até o erro trocar de sinal atravessa o
alvo por essa margem, todo ciclo, para sempre. Sobrepasso previsto na
inversão (wz=0,91): 0,276 rad. Medido: 0,34 rad.

**A boba não é a culpada principal.** Comparando odometria de roda com a pose
verdadeira: a roda acha que girou até 0,16 rad a mais durante a curva, e sobra
0,082 rad (4,7°) por curva de 90°. Derrapagem real — o simulador não é
cinemático perfeito — mas responde por ~20% do S. A rampa responde por ~80%.

### A lei candidata

`wz = sinal(e)·min(wz_max, √(2·a_dec·|e|))`, com a linear cedendo em `cos(e)`.
Em uma frase: *nunca peça mais giro do que você consegue frear dentro do erro
que ainda falta*. É a mesma conta do sobrepasso, usada como limite em vez de
sofrida como defeito.

Mesma planta, mesmos tetos, mesma boba, mesma manobra:

| | bang-bang | frenagem |
|---|---|---|
| sobrepasso | 20,5° | 2,4° |
| assenta em | nunca | 1,28 s |
| erro em regime | ±17° eterno | 0,002 rad |

### O dono julgou o simulador fraco — e isso virou estimativa

Mostrado o S animado e em unidades físicas, o dono disse que o robô real faz
"barrigas maiores", da ordem de **50 cm** contra os ±10 cm daqui. Rodando o
mesmo ensaio com a planta degradada (a_dec 0,3 em vez de 1,5) a barriga foi a
~100 cm; a lei de potência entre os dois pontos (barriga ∝ a_dec^-1,43) coloca
o **a_dec real em ~0,5 rad/s²**. Estimativa, não medida — foi o que motivou o
banco de ensaios.

### Estresse: 10 corridas, e o que elas acharam

Tudo na planta degradada (a_dec 0,3, mais pessimista que a estimativa do robô).

| # | estresse | resultado |
|---|---|---|
| E1 | meia-volta 180° | sobrepasso 0,1° |
| E2 | alvo trocando de sinal | 0,1° nos dois trechos |
| E3 | velocidade baixa (0,2 m/s) | 0,1° |
| E4 | malha a 10 Hz (taxa real) | 0,6°, erro final 0,12° |
| E5 | zona morta andando | não mordeu nenhuma amostra |
| E6 | zona morta girando parado | **travado** — é navegação |
| E7 | meia-volta + zona morta 0,10 | soluço de 0,1 s, completou |
| E8 | E7 + piso de linear 0,25 | soluço eliminado |
| E9 | meia-volta + zona morta **0,15** | **22 s parado, erro 179,9°** |
| E10 | E9 + piso 0,30 (fórmula) | sobrepasso 0,8° |

Três achados:

1. **Errar o `a_dec` pra baixo é de graça** (chute 3x menor: sobrepasso zero,
   4,5 s contra 4,8 s do valor exato); pra cima é que traz o S de volta — mas
   mesmo 5x otimista ela **degrada, não quebra**: oscila e decai. É isso que
   autoriza escrever a movimentação antes de medir o robô.
2. **A zona morta é precipício.** Entre 0,10 e 0,15 m/s está a diferença entre
   "funciona" e "nunca sai do lugar". Virou o BO-3.
3. **A regra `cos(e)` zera a linear acima de 90° de erro** — e linear zerada é
   a condição do travamento. Daí o piso de linear, e a fórmula que o dimensiona
   a partir da zona morta.

### Fracassos e correções da sessão (ficam registrados)

- **Dois Gazebos vivos ao mesmo tempo** corromperam uma bateria inteira: o
  tempo andava pra trás no CSV e o rumo dava voltas de 180°. A limpeza matava o
  `gz sim` mas deixava a ponte ROS viva, e cada rodada somava um publicador de
  `/clock` — chegou a **9**. Corrigido com limpeza completa, trava que aborta
  se sobrar processo, e o banco passou a **morrer de propósito** se o relógio
  andar pra trás, em vez de gravar dado sujo.
- **Métrica de sobrepasso errada**: contava a troca de alvo como se fosse
  sobrepasso do controlador (E2 "+90°"). Corrigida para medir por trecho de
  alvo constante.
- **Previsão errada, pega pelo teste**: afirmei que a meia-volta travaria com
  zona morta. Travou 0,1 s e completou com erro 0,0°. Acertei *onde* morde
  (t=5,5 s, o instante previsto pelo log da E1), errei o tamanho. Foi o teste
  de sensibilidade (E9, zona morta 0,15) que mostrou o risco verdadeiro.
- **Superestimei a zona morta no começo**: disse que ela agiria sobre `wz`.
  Ela age sobre a velocidade de cada **roda** — andando a 0,7 m/s as rodas
  estão longe do limiar e ela não morde (E5 confirmou: zero amostras).

### Onde parou

Banco de ensaios versionado em `tools/banco/` (roda igual no robô e no
simulador). O dono foi ao laboratório medir o robô por completo — trena,
zona morta, `a_dec`, curva por velocidade. Com esses números a movimentação
deixa de ter parâmetro chutado dentro.

## 2026-07-27 (2ª leva) — Navegação ponto a ponto

Fatia A da navegação: ir a um ponto, sem obstáculo. Decisão 006.

O dono foi ao laboratório medir o robô e voltou sem medida — a elétrica está
ruim. Não é perda: os parâmetros da movimentação já eram conservadores de
propósito, o nó avisa que não foram medidos, e a lei degrada em vez de
quebrar quando o número está errado. Foi exatamente para este caso que a
propriedade foi verificada de manhã.

### O que a fatia A resolve, e como

Duas ideias, as duas herdadas de coisa medida hoje:

1. **Rumo alvo recalculado todo ciclo** como direção até o ponto. Isso dissolve
   o erro lateral sem controlador extra — o desvio de 66 cm que o controlador
   de rumo sozinho deixava depois de uma meia-volta some, porque não existe
   mais "linha a seguir", existe ponto para o qual apontar.
2. **Aproximação pela mesma lei de frenagem**, agora em distância:
   `v = √(2·a_lin·dist)`. Um parâmetro físico, medível.

E uma forma imposta pela zona morta: **não existe chegar devagarinho**. A
desaceleração ideal manda velocidades cada vez menores, a placa engole as
pequenas, e o robô para longe do ponto achando que chegou — o BO-3 disfarçado
de sucesso. Então a aproximação tem piso e o corte é firme. Consequência que o
nó verifica sozinho: raio de chegada menor que a distância de parada a partir
do piso faria o robô **orbitar o ponto**; ele recusa e avisa.

### Verificado (dois nós reais empilhados no simulador)

| alvo | distância | resultado |
|---|---|---|
| (2, 2) | 2,83 m à frente | parou a **7 mm**, 45 s sem orbitar |
| (−1, 1) | 1,41 m, 135° atrás | parou a **8 mm** |

### O custo que apareceu no segundo caso

Para o alvo atrás, o robô **se afastou até 2,15 m** de um ponto a 1,41 m antes
de voltar: fez um laço. É consequência direta de não pivotar — política que
existe porque girar parado devagar cai na zona morta. Fica registrado como
comportamento conhecido, não como defeito. O laço encolhe quando a zona morta
real for medida, porque é ela que dimensiona o piso, e o piso é o que abre o
arco.

### Fracasso da sessão

A primeira validação mandou o segundo objetivo perto do fim da janela do
observador e não deu tempo de verificar nada — reportei "enviado" sem ter
dado. Refeito com o alvo de trás como primeiro objetivo, aí sim medido.

### Onde parou

Falta a fatia B: desviar de obstáculo. Depende do Livox e de percepção que o
repo não tem. E continuam pendentes as medições do robô (BO-3), agora atrás de
um problema elétrico.

## 2026-07-27 (3ª leva) — Cliques do dono derrubam a navegação

Ligado o RViz à pilha (`/goal_pose`, o nome padrão do ROS — o mesmo fio serve
para a GUI web depois). O dono clicou pontos e **em cinco minutos achou o que
dez corridas roteirizadas não acharam**: um ponto a 0,65 m, de lado, era
orbitado para sempre.

Isso vale como método, não como anedota: os ensaios eram roteirizados por quem
escreveu a lei, e por isso testavam o que ela já sabia fazer — alvos longe,
manobras amplas. Nenhum testava ponto perto e de lado.

### A fronteira estava no log dele

    (3.99, -2.89)  a 4,37 m  -> chegou
    (3.72, -2.20)  a 0,73 m  -> chegou
    (3.15, -1.76)  a 0,65 m  -> NUNCA CHEGOU

### Quatro defeitos, um sintoma

1. **Banda morta com duas saídas.** A roda interna sai dela andando para frente
   OU para trás; só a primeira estava programada. O pivô ficou proibido por
   aritmética, não por física.
2. **Velocidade sem teto pela curva.** Perseguir um ponto a `d` com erro `e`
   exige girar a `v·sen(e)/d`; sem teto, o ponto escapa pelo lado.
3. **A linear cedia pelo erro do BICO.** Este era o principal, e foi o dono que
   apontou: *"não tem como não vencer a velocidade mínima se você estiver
   girando as duas rodas em lados opostos"*. Medido em órbita: bico a **50°**
   do alvo (cos = 0,64 → segue a 64% da velocidade), movimento a **87°** —
   perpendicular. **37,5° de deriva lateral**, constantes. A boba traseira
   sendo jogada para fora, exatamente como a decisão 004 previu; primeira vez
   que ela domina um comportamento em vez de ser detalhe de 20%.
4. **Chegando, não parava.** O rumo até o ponto gira sozinho quando se está em
   cima dele; o robô girava no lugar e o giro arrastava a traseira para fora.

### Depois

| caso | antes | agora |
|---|---|---|
| ponto a 0,65 m de lado | orbitava a 0,233 m para sempre | **chega a 0,059 m** |
| alvo à frente (2, 2) | 7 mm | 9 mm |
| alvo atrás (−1, 1) | 8 mm em 13,5 s, laço de 2,15 m | **8 mm em 9,8 s, laço de 1,67 m** |
| parado no ponto | derivava 0,06 → 0,27 m | **9 mm em 30 s** |

O "laço para alvo atrás", que eu tinha registrado como *custo aceito* na
decisão 006, era sintoma do mesmo defeito. Custo documentado não vira verdade
por estar escrito.

### Meus erros da sessão (o que mais importa aqui)

- **Não olhei o que já estava acontecendo.** O robô orbitava na tela do dono e
  eu fui montar experimento novo — e resetei a pose, apagando o caso dele.
  Reação: *"pq que vc só não viu oq já estava acontecendo"*. O log com os
  cliques já tinha a resposta.
- **Consertei antes de reproduzir.** Diagnostiquei "o piso proíbe o pivô",
  implementei, e só então rodei: a órbita continuou igual. O piso nem estava
  ativo — no simulador `zona_morta = 0`. A hipótese descrevia um bug real (a
  conta estava errada mesmo) mas **não era o bug que o dono viu**.
- **Alimentei o simulador com o chute do robô real.** Lá a zona morta é zero;
  o robô se defendia de um perigo inexistente naquele ambiente. Agora há
  config próprio de simulador, e o banco injeta zona morta quando quer
  testá-la.

### Onde parou

325 testes verdes. Falta a fatia B (desviar de obstáculo, depende do Livox) e
as medições do robô, agora atrás de um problema elétrico — o dono foi ao
laboratório e voltou sem medir.

## 2026-07-27 (4ª leva) — A placa fingida, e a bitola virando o item nº 1

Pedido do dono, e ele estava certo: em vez de rodar o simulador com zona morta
zero (fiel ao Gazebo, infiel ao robô), **colocar a zona morta dentro do
simulador**. Assim o controle é desenvolvido contra ela, e quando a bancada
medir a de verdade troca-se só o número.

- `robot_base/placa_simulada` fica entre o controlador e o simulador e engole
  comando de roda pequeno demais, na roda, que é onde o defeito mora.
- A planta lenta (`a_dec = 0,3`) virou perfil versionado — antes vivia num
  arquivo solto na máquina de quem trabalhava, e quem subisse o simulador pelo
  caminho oficial pegava a planta ágil e veria um robô melhor do que o real.
  Dívida de reprodutibilidade fechada.

### O que a zona morta ligada revelou

O ponto de 0,65 m voltou a ser inalcançável: o robô chega a 0,168 m e não fecha
os últimos 2 cm. **E desta vez não é defeito de código — é física.**

    pivô exige:  wz_max · bitola/2  >=  zona_morta + margem
    bitola 0,20 + zona morta 0,10  ->  precisa de 1,3 rad/s, teto é 1,0  X
    bitola 0,32 + zona morta 0,10  ->  precisa de 0,62 rad/s            OK

**A bitola decide.** O modelo simulado tem 0,20 m, mais estreito que os 0,32
presumidos do robô real — o simulador é mais pessimista que o robô neste ponto
específico. E isso muda a prioridade da bancada: **medir a bitola com trena é
mais urgente do que medir a zona morta**, porque é ela que define se o robô
consegue virar no lugar.

O nó agora anuncia na subida qual dos dois casos é o dele, com o número:

    pivô INDISPONÍVEL: girar parado exigiria mais de 1.30 rad/s, e o teto é
    1.00. O robô só faz arcos, e pontos perto dele ficam INALCANÇÁVEIS.

### Tropeço

`robot_base` não tinha `setup.cfg`, então o executável do nó novo não era
instalado e o launch morria com "libexec directory does not exist". Três
corridas abortadas até eu ler o log do launch em vez do log da corrida.

## 2026-07-28 — A ré entra, e a boba do simulador cai

Sessão que começou numa pergunta do dono sobre a decisão 005 ("ele tá fazendo
ré? pq dando ré ele pode chegar sem problema, não?") e terminou derrubando uma
premissa da decisão 004.

### O que eu prometi medir antes de mexer na lei

A ré resolve o ponto inalcançável por geometria — recuar aumenta `d` e o raio
necessário `d/(2·sen e)` abre. O contra que eu não sabia responder: de ré a
boba vira roda dianteira, e boba na frente é instável. Ficou combinado medir
antes de implementar.

### Erro de ensaio nº 1: reta pura não mede nada

Primeiro ensaio: reta, ida e ré, medindo desvio de rumo. Resultado nos quatro
casos: **0,0° e 0,0 cm em 4 m**, com `y` exatamente zero. O robô simulado é
perfeitamente simétrico num plano liso — ele anda numa reta matemática.
Instabilidade é bifurcação: só aparece se você perturbar. O ensaio `reta` do
banco ganhou um **cutucão** (pulso de giro de 0,5 s e solta) por causa disso.

### Com cutucão: ré ≡ ida, e é aí que a coisa fica suspeita

| | solto com | girou mais | assentou | \|wz\| nos últimos 2 s |
|---|---|---|---|---|
| frente 0,20 | 0,086 rad/s | 1,9° | 0,56 s | 0,000 |
| ré 0,20 | 0,084 | 1,9° | 0,56 s | 0,000 |
| frente 0,35 | 0,086 | 1,9° | 0,56 s | 0,000 |
| ré 0,35 | 0,084 | 1,9° | 0,54 s | 0,000 |

Idênticos demais. Fui olhar o pivô da boba direto no Gazebo (o `/joint_states`
só publica as duas motrizes; o ângulo veio da posição do contato em torno do
pivô, no `dynamic_pose/info`).

### A boba do simulador não é uma boba

Numa curva **pra frente** — v=0,25, wz=0,6, raio 0,42 m — o garfo deveria
assentar a ~157° do corpo, que é `atan(0,18/0,42)` fora do eixo, e ficar lá.
Em vez disso saiu de 180° e girou continuamente até 38°: ele mantém o rumo do
**mundo**, não acompanha o corpo. É um patim, não uma boba.

Hipótese: o `mu2=0.05` da boba (posto de propósito para a traseira derrapar)
matava o torque que alinha o garfo. **Errada.** Multipliquei o atrito por 16
(`mu 0,05 → 0,8`) e o rumo na mesma curva foi de 136,161° para 136,675° —
0,4%. O contato da boba não participa da dinâmica, e a causa real de o garfo
não alinhar continua desconhecida. Arquivo revertido; virou o BO-4.

Isso é maior que a ré: a decisão 004 diz que o trail de 4 cm + atrito no pivô
reproduzem a traseira jogada pra fora, e o S de 27-07 foi atribuído ~20% à
boba. Essa atribuição não se sustenta — a derrapada que o simulador mostra vem
do `mu` baixo do contato, não da geometria de boba.

### Decisão do dono: caminho B

Seguir com a ré conservadora e mandar a boba para a bancada, em vez de caçar a
causa no modelo agora. O simulador só se valida contra os números do robô, que
ainda não temos. Custo assumido e registrado: **a ré foi validada só no
simulador, e justamente a parte que preocupa nela é a que o simulador não pode
mostrar.**

### O que entrou (decisão 007)

Gatilho geométrico (`d/(2·sen e) < raio_min_curva`), ré **reta** por escolha do
dono, sinal negativo em `velocidade_alvo` como modo (sem tópico novo), lei de
rumo intacta (`test_nunca_anda_de_re` continua verde), histerese de 1,3× e
orçamento de 1,0 m / 8 s com grito no log.

Verificado: alvo a 0,65 m de lado, que orbitava a 0,168 m para sempre, agora
recua 9 cm em duas mordidas e **chega** — 0,150 m do ponto, parado 30 s.
Regressão do alvo (2, 2): 8 mm, sem acionar ré. 338 testes verdes (eram 325).

### Erro de ensaio nº 2: dez `/clock` órfãos

Duas corridas de aceitação saíram com o tempo embaralhado e o robô recuando
5 m. Causa: o `trap` do meu script expandia `$NAV` antes da variável existir,
então cada corrida deixava viva a pilha de navegação — e, pior, **dez
`parameter_bridge` de `/clock`** acumulados. Quando um Gazebo novo sobe, todos
voltam a republicar o mesmo `/clock` fora de ordem. O banco tem guarda contra
isso desde 07-27 ("relógio andou pra trás"); minha ferramenta de aceitação,
não. Depois de limpar, refiz a medição de ré e os números bateram com os de
antes — a conclusão não tinha sido contaminada.

Também gastei três chamadas descobrindo que `pkill -f 'gz sim'` casava com a
própria linha de comando do shell que o chamava, e eu me matava.
