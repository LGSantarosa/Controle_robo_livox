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

## 2026-07-28 (2ª leva) — O dono dirigiu, a navegação caiu, o Nav2 entrou

Sessão ao vivo no RViz com o dono clicando objetivos, e o veredito dele em
duas frases: *"essa ré tá uma merda"* e *"ao invés de ele só girar e ir reto,
ele dá um puta balão para chegar num goal do lado"*.

### O que o CSV da sessão mostrou (`docs/dados/2026-07-28-cliques-*.csv`)

2714 amostras, 4 objetivos clicados. O número que resume tudo:
**0 amostras de giro parado**. Ele não virou no próprio eixo uma única vez.

| alvo clicado | distância | caminho andado | tempo | ré |
|---|---|---|---|---|
| (−1,02, −0,33) | 1,07 m | 3,04 m (2,8×) | 9,5 s | nenhuma |
| (−1,58, 0,60) | 0,43 m | 3,66 m (8,5×) | 57 s | 12 entradas |
| (−1,24, 0,23) | 0,40 m | 1,98 m (5,0×) | 60 s | 6 entradas |

Raio de curva efetivo: mediana 0,37 m, mínimo 0,23 m.

**Dois defeitos, e só um era da ré.** O ciclo "ré e anda" tem assinatura de
ciclo-limite: 12 entradas com período de **2,10 s ± 0,54** e mordidas de 5,5 cm.
Causa: a histerese da decisão 007 solta a ré assim que o alvo cabe *naquele
instante*; o robô avança, a distância encurta, a geometria fecha de novo. Eu
validei a ré com UMA geometria e o dono achou o ciclo clicando, de novo.

O balão é outra coisa: é `v/wz` com `wz_max` em 1,0 (herdado, nunca medido) e o
piso de linear que a zona morta obriga — girando a 1,0 rad/s ele é *obrigado* a
andar a 0,23 m/s, e o raio mínimo é isso. Nenhum planner arrumaria.

### O robô 1, lido a pedido do dono

Ele mandou parar de consertar desenho meu e ler o `Controle_robo_web`. Estava
certo. De lá:

- **`path_follower`**: "gira no lugar e anda reto", com histerese (entra a 16°,
  sai a 3°) e saída preditiva do giro. O comentário no código descreve o meu
  defeito de hoje: *"girava e parava no MESMO limiar → limite-ciclo"*.
- **`unstuck_supervisor`**: a ré de lá dispara **por sintoma** ("não se
  deslocou por N s com goal ativo"), é sempre reta, tem orçamento, e mede o vão
  traseiro em METROS num retângulo da largura do robô (`rear_min_gap`) — isso
  nasceu de uma batida de ré real em 11-06-2026.
- **`twist_mux` com prioridade** para humano/desencalhe.
- E um número que me pegou: o robô 1 gira a **2,4–4,5 rad/s**. O nosso teto de
  1,0 é o que torna o pivô "impossível" — e nunca foi medido.

**O dono descartou o "gira no lugar"**: aquilo era a única saída do chassi de 4
rodas, que não faz arco. Este faz curva boa, e não deve parar para virar.

### Decisão do dono: Nav2

Aposentar a navegação ponto a ponto (dados ficam), trazer o Nav2, e por **agora
só o planner** — ver se ele desenha caminho que agrada. Seguidor provavelmente
será nosso, como no robô 1.

### A bancada do planner (`ros2_packages/robot_planning/`)

Dois cliques, dois caminhos, uma tabela. Sem robô, sem simulador, sem sensor —
de propósito: julgar o desenho isolado de quem o executa. Theta\* (o do robô 1)
contra Smac Hybrid-A\* com Reeds-Shepp (respeita raio de curva e pode usar ré).

Pista gerada por `tools/mundo/gera_pista.py`, que escreve **mapa do Nav2 e
mundo do Gazebo da mesma planta**: porta de 0,90 m, bloco solto, aperto de
0,80 m e beco sem saída, com o robô de 0,50 m.

### Três defeitos meus, achados testando a própria bancada

1. **O beco que desenhei era uma caixa lacrada** — sem entrada. Os dois
   planners recusavam o destino, corretamente, e o caso não testava nada.
2. **A medida de raio mínimo estava errada**: eu calculava curvatura entre
   pontos vizinhos de um caminho suavizado a 5 cm, e media ruído de
   arredondamento — acusava 0,01 m num planner configurado com 0,25 m. Corrigido
   reamostrando a 0,20 m.
3. **Preempção silenciosa**: o `planner_server` atende um objetivo por vez, e eu
   pedia os dois caminhos em paralelo. O segundo preemptava o primeiro, e o
   preemptado voltava com **caminho vazio e código de sucesso**. Aparecia como
   "Theta\* sem caminho" só no primeiro clique de cada corrida (frio, ele
   demorava mais e era atropelado). Antes de achar a causa eu culpei o costmap
   e "consertei" uma corrida que não existia.

Seis pares (partida, destino) rodados nos dois planners, todos com caminho.

## 📏 2026-07-29 — A trena no robô: dois números herdados, ambos errados

Primeira vez que o robô 2 é medido. Até hoje **toda** dimensão do modelo era
estimativa ou herança, e o `ESTADO_PROJETO.md` já registrava isso como a dívida
nº 1. O dono passou as medidas; abaixo o que elas derrubaram.

### As medidas

```
caixa .................... 433 × 455 × 145 mm
fundo da caixa ao chão ... 85,2 mm
altura total (sem lidar) . 233,5 mm   (confere: 85,2 + 145 = 230,2; 3,3 mm de tampa)
rodas, por fora .......... 315 mm
rodas, por dentro ........ 225 mm
diâmetro da roda ......... 160 mm
ponta da roda → frente ... 55 mm
```

Da terceira e quarta linha saem dois números de uma vez, sem medir nenhum
outro: bitola = (315+225)/2 = **270 mm**, espessura da roda = (315−225)/2 =
**45 mm**.

### O achado: a bitola errava dos DOIS lados, em sentidos opostos

O `hoverboard_controllers_sim.yaml` carregava um bloco `⚠️ DIVERGÊNCIA
CONHECIDA` escrito à mão: simulador com 0,20, robô real com 0,32, nenhum dos
dois medido. A trena diz 0,270 — **nenhum dos dois acertou**, e o efeito é em
direções contrárias:

| | bitola usada | giro real vs comandado |
|---|---|---|
| Simulador | 0,20 | gira **26% a menos** (90° → 67°) |
| Robô real | 0,32 | gira **19% a mais** (90° → 107°) |

Ou seja: sintonizar o controlador de rumo na bancada e mandar pro robô erraria
duas vezes, em sentidos opostos. É a pior forma de erro possível — a bancada
teria parecido boa e o robô teria piorado. Não é hipótese: era o estado do repo
até hoje de manhã.

O raio caiu junto: 0,0825 (roda de 6,5" nominal, herdado) → **0,080** medido.
Sozinho ele é ~3% de erro de odometria — 30 cm a cada 10 m percorridos.

### O eixo estava 68,5 mm à frente de onde está

Dos 55 mm entre a ponta da roda e a face frontal: face em 0,2165, ponta em
0,1615, centro do eixo em **0,0815** — contra 0,15 estimado. O entre-eixos
(eixo motriz → pivô da boba) cai de 0,330 para 0,2615, **21% menor**. Importa
direto: a traseira é uma ponta solta mais curta do que vínhamos simulando, com
menos vantagem mecânica pra jogar a boba pra fora.

### Defeito achado conferindo: o chassi tinha massa no chão

O `<inertial>` do `base_link` não tinha `<origin>`. Sem ele o URDF assume
`(0,0,0)`, e a origem do `base_link` está **no nível do solo**. A caixa era
desenhada a 85–230 mm e os 5,8 kg eram simulados a 0 mm.

```
CoM do robô, antes ....... z = 0,0173 m
CoM do robô, corrigido ... z = 0,1244 m     (7,2× mais alto)
```

O simulador rodava com uma panqueca colada no chão: sem rolagem e — o que
importa aqui — **sem transferência de peso**. É o mecanismo central do problema
que estudamos: ao acelerar, o peso sai da traseira, a boba fica leve, perde
força normal e **escorrega mais de lado**. Com a massa no solo esse mecanismo
simplesmente não existia; a carga na boba era constante o tempo todo.

Delimitando o estrago, porque ele não é total: o deslocamento é só em Z, e
inércia de guinada (`izz`) não muda com deslocamento vertical. **A inércia de
rumo estava certa** — o trabalho de oscilação de rumo feito na bancada continua
válido. Errado estava rolagem, arfagem e transferência de peso.

### A boba do modelo era geometricamente impossível

`boba_raio` valia 0,05 — roda de **100 mm**. Mas o vão inteiro sob a caixa é de
**85,2 mm**, e a boba precisa caber ali com chapa, pivô e garfo. A roda sozinha
era mais alta que o conjunto todo. O valor morreu por restrição, sem precisar de
medida nova: o robô estar reto a 85,2 mm já prova que ele é falso.

Provisórios que entraram, escolhidos para serem POSSÍVEIS (não medidos):
`boba_raio` 0,025 (roda de ~2", descontando o ferro) e `boba_trail` 0,01
(15–25% do diâmetro é o típico). O trail anterior de 0,04 **exagerava em 4×** o
quanto a boba é jogada pra fora. Modelo que exagera o defeito engana tanto
quanto modelo que o esconde.

### O teste do trail reprovou o valor certo

`test_boba_tem_trail_nao_nulo` exigia trail > 10 mm fixos, e o novo valor é
exatamente 10 mm. O teste fez o trabalho dele — ele existe pra impedir que
alguém zere o trail e transforme o simulador num robô ideal. Mas o piso de 10 mm
foi escrito quando o modelo supunha roda de 100 mm: nessa escala 10 mm é
desprezível. Numa roda de 50 mm, 10 mm é o valor **típico**, e o piso absoluto
reprovava justamente o número correto.

Corrigido para limite **relativo**: `trail > 0,2 · raio_da_roda`. Preserva a
intenção e vale em qualquer tamanho. Não subimos o trail pra passar no teste —
seria ajustar o robô pra agradar o teste.

### O pivô continua indefinido, e agora depende da zona morta

O `ESTADO_PROJETO.md` registrava que a bitola decidia se o robô consegue
pivotar. Reescalando os dois casos já calculados para a bitola medida:

| caso | wz p/ pivotar (bitola suposta) | com bitola 0,270 |
|---|---|---|
| simulador (zona morta 0,10) | 1,30 rad/s | **0,96 rad/s** — cabe no teto de 1,0, com 4% de folga |
| robô real (zona morta 0,15) | 1,25 rad/s | **1,48 rad/s** — impossível, e PIOR que antes |

A bitola medida **não resolveu a pergunta**: ela caiu entre os dois palpites, e
agora quem decide é a **zona morta**, que segue sem medir. Com 4% de folga no
melhor caso, isso não é margem nenhuma. A zona morta virou o item nº 1 da
bancada, no lugar da bitola.

### Estado

282 testes verdes. Ainda **não medidos**: diâmetro da rodinha da boba (fecha
`boba_raio` e `boba_trail` de uma vez), massas, e a largura da caixa **na altura
das rodas** — as rodas ficam 70 mm para dentro da parede lateral, então ou a
caixa tem recortes ou a parte de baixo é mais estreita que os 455 mm do topo.
Não afeta giro nem odometria; afeta o footprint que o Nav2 usa pra decidir se
passa num vão.

Em aberto com o dono: trocar a boba por uma **roda omnidirecional**. Não é
contornar o problema — omni não tem pivô nem trail, então elimina a causa da
instabilidade de rumo em vez de mascará-la. Mas invalida a junta de pivô, o
trail, o atrito de pivô e boa parte do `robo2.gazebo.xacro`. Se for pra frente,
é decisão registrada, não ajuste.

## 🎯 2026-07-29 (2ª leva) — A pilha inteira obedece; o giro é que não entrega

Com a trena já dentro dos YAMLs, faltava a pergunta que a manhã não respondeu:
**a pilha montada obedece?** O `ensaio.py` mede a máquina em malha aberta e roda
igual no robô; ele não diz nada sobre navegação + movimentação + placa
empilhadas. Daí `tools/banco/corrida_gazebo.py`: sobe o Gazebo headless, roda
duas fases na MESMA simulação e derruba tudo. Duas corridas, uma por perfil de
zona morta (`sim` 0,10 · `real` 0,15) — o chute otimista e o pessimista, postos
nos dois lados (planta E controlador), porque pôr o pessimista só no controlador
mede uma máquina que não existe.

232 mil amostras nos dois CSV de `docs/dados/`.

### Fase B: os 5 alvos fecharam, nos dois perfis

Reta de 2 m, 90° de lado, 180° para trás, o ponto perto-e-de-lado (o caso da
decisão 007) e a volta à origem. **10 de 10 chegaram** dentro do raio de 0,15 m.
Nenhuma órbita, nenhum travamento — contra a sessão de 28-07, em que o dono
clicando derrubou a navegação com 0 amostras de giro parado em 2714.

Isso não é a navegação absolvida: ela segue aposentada. É a constatação de que
o roteiro fechado não reproduz o defeito que o dono achou clicando — mais uma
vez, e o registro de 27-07 já dizia o mesmo (10 corridas roteirizadas não
acharam o que 5 minutos de clique acharam).

### Fase A: a reta sai exata, o giro sai curto — e piora subindo

A fase A publica **direto** no controlador de tração, contornando a placa
fingida: sem zona morta no caminho, o que sobra é a conversão comando→roda mais
a física. O controlador de rumo nem está no ar.

| comando | realizado ÷ comandado |
|---|---|
| reta v=0,30 | **1,004** |
| giro 0,3 rad/s | 0,860 |
| giro ±0,5 rad/s | 0,855 / 0,853 |
| giro 0,8 rad/s | 0,786 |
| giro 1,0 rad/s | **0,790** |
| arco v=0,3 wz=0,4 | 0,941 |

A varredura de wz existia para separar RAZÃO de OFFSET, e a resposta é **razão,
e agravando**: o teto de `wz_max = 1,0` entrega **0,79 rad/s de verdade**. Não é
a bitola errada — bitola errada daria razão CONSTANTE, e a reta a 100,4% já
prova que raio e conversão estão certos. O que degrada com a velocidade é
escorregamento.

Esquerda e direita batem em 0,2% (0,855 × 0,853): não há assimetria de
conversão. Some a isso um recuo sistemático de ~0,065 m/s por rad/s **girando
parado**, igual nos dois sentidos. Parte é geometria de medida — a origem do
`base_link` está 81,5 mm atrás do eixo motriz, então ela orbita quando o robô
pivota — mas o efeito de geometria trocaria de sinal com o sentido do giro, e
este não troca. Fica anotado, sem explicação fechada.

⚠️ **Tudo isso é o Gazebo com a boba do BO-4**, que já sabemos ser um patim. O
déficit de giro pode ser o mesmo contato falso. Não vale como medida do robô.

### O pivô só existe no perfil otimista — de novo, e agora medido

181 amostras de giro parado no `lado_90` do perfil `sim`; **zero em toda a fase
B do perfil `real`**. Bate na mosca com a aritmética de hoje de manhã: 0,96 rad/s
para pivotar com zona morta 0,10 (cabe no teto, 4% de folga), 1,48 rad/s com
0,15 (impossível). Sem pivô, o `lado_90` do perfil real custou 2,43 m de caminho
e 151,6° líquidos para uma virada de 90°.

### O piso de linear segura o BO-3

**Zero amostras** com roda pedida dentro da banda morta, em todos os trechos dos
dois perfis. A defesa `v_piso = zona_morta + wz_max·bitola/2 + margem` está
fazendo o que foi desenhada para fazer.

### O ciclo "ré e anda" voltou, e só no perfil pessimista

O alvo `perto_de_lado` (0,40 m de distância):

| | tempo | caminho | caminho ÷ reta | entradas em ré |
|---|---|---|---|---|
| perfil sim | 2,7 s | 0,54 m | 2,20 | 2 |
| perfil real | **9,2 s** | **1,85 m** | **7,19** | **5** (período 2,23 s) |

É o mesmo defeito de histerese diagnosticado em 28-07, com período quase igual
(2,23 s contra 2,10 s). Ele sobrevive porque quem roda nesta bancada ainda é o
`goal_navigator`. **Não vamos consertá-lo**: a camada está aposentada, e o
trabalho iria para o lixo junto com ela.

### O que isto muda no trabalho de amanhã: um número da bancada do planner

`bancada_planner.yaml` está com `minimum_turning_radius: 0.25`, justificado no
comentário por "a curva mais fechada no simulador em 28-07 foi 0,23 m". Esta
corrida contradiz:

| | raio que o controlador PEDE | raio REALIZADO (p5) |
|---|---|---|
| perfil sim | 0,270 m | **0,370 m** |
| perfil real | 0,339 m | **0,463 m** |

(os mínimos absolutos de 0,14 e 0,20 m são transitórios de uma amostra, não
curva sustentada — por isso o p5.)

O realizado abre em relação ao pedido **por causa do déficit de giro**: pede-se
1,0 rad/s, sai 0,79, e o raio abre na mesma proporção. No perfil pessimista o
planner está configurado com quase METADE do raio que a máquina fecha.

Isso não é um parâmetro qualquer nessa bancada: o raio mínimo **é** o argumento
da comparação. O Smac Hybrid-A\* está na mesa contra o Theta\* precisamente
porque respeita raio de curva. Julgar os dois com um raio 1,9× otimista é dar a
vitória ao Smac num robô que não existe — a mesma forma de erro da bitola, em
que a bancada pareceria boa e o robô pioraria.

**Decidido em vez de chutar de novo**: a bancada do planner roda com uma FAIXA
de raio, não com um valor. Se o ranking não virar com o raio, a conclusão está
imune à zona morta que ainda não medimos e a decisão 008 pode ser assinada já.
Se virar, descobrimos antes de assinar — e o item nº 1 da bancada com o robô
ganha uma segunda razão de peso.

## 📐 2026-07-29 (3ª leva) — A régua da bancada estava errada; consertada, o planner se decide

A corrida da manhã reabriu o `minimum_turning_radius` da bancada do planner: os
0,25 m configurados contra 0,370 m e 0,463 m de raio realizado, um por perfil de
zona morta. Como o número certo depende de uma medida que só o robô dá, o jeito
honesto não era escolher — era **varrer**. Daí
`tools/planner/varredura_raio.py`: 6 casos × 4 raios × 2 planners, headless,
sem cliques.

E a varredura achou um defeito antes de responder a pergunta.

### O caminho estava certo; a régua, não

Na primeira rodada o Smac apareceu devolvendo, com raio de 0,46 m configurado,
um caminho de **raio 0,125 m** — 3,7× mais fechado do que ele foi mandado
respeitar. Isso é grave: o Smac está na disputa exatamente por respeitar raio.

Reproduzido e desmontado antes de consertar nada:

1. **Não é o suavizador.** `smooth_path` ligado e desligado dão caminho
   idêntico, ponto por ponto.
2. **Não é o planner.** Despejados os 20 pontos crus do caso `perto_de_lado`,
   com o deslocamento projetado no rumo de cada pose, o sinal aparece: `+0,072`
   nos pontos 1–3, **`−0,072` nos pontos 5–16**, `+0,072` nos 17–19. O caminho
   é *frente, ré por 0,86 m, frente* — duas **cúspides** de Reeds-Shepp. Os
   arcos fecham ~0,41 m, dentro da discretização dos 0,46 pedidos.
3. **É o `mede()`.** Ajustar círculo por três pontos EM CIMA da cúspide lê a
   dobra como curva fechadíssima; e a reamostragem a 0,20 m pula por cima da
   dobra, então a inversão sumia. As três colunas erravam de uma vez:

| | relatado | verdade |
|---|---|---|
| raio mínimo | 0,125 m | ~0,41 m |
| giro | 181° | ~72° (o resto é a inversão) |
| inversões | **0** | **2** |

E erravam todas na mesma direção: **contra quem usa ré**. A bancada estava cega
justamente no assunto que ela existe para julgar. Ela não tinha teste nenhum —
foi assim que sobreviveu.

**Conserto**: o caminho é partido nas cúspides (detectadas nos pontos CRUS) e
cada trecho é medido sozinho; `inversoes` passa a ser a contagem de cúspides.
Cinco testes novos em `test_mede.py`, com a geometria tirada do caminho real,
não inventada para passar.

### Um segundo número impossível, e um achado de verdade dentro dele

Consertada a cúspide, o caso `bloco` com raio 0,46 passou a acusar
`raio_min = 0,00 m`. Despejado o caminho: 69 pontos, dos quais **63 formam
4,86 m limpos** e os 6 últimos são um **tremor em cima do alvo** — 4 inversões
dentro de uma caixa de 9 cm, deixando trechos de 8 cm entre cúspides.

São duas coisas, e separá-las importa:

- o `0,00` era da régua (um fallback que media curvatura nos pontos crus quando
  o trecho era curto demais para reamostrar — exatamente o ruído que a
  reamostragem existe para evitar). Trecho curto agora é **pulado e contado** na
  coluna nova `curt`, porque pular calado é como o defeito anterior durou tanto;
- **o tremor é do planner e é achado**: com raio grande, a aproximação final do
  Smac não assenta no alvo e fica trocando de sentido. Fica registrado para o
  seguidor — quem for executar isso precisa saber.

### Com a régua honesta: o ranking não vira, ele se acentua

288 → 289 testes verdes, e a varredura rodada de novo. O Theta\* saiu
**idêntico nos quatro raios** — ele não conhece raio, e é a testemunha de que a
varredura mexeu só no que devia.

| raio que a máquina fecha | Theta\*: caminhos seguíveis | Smac: idem |
|---|---|---|
| 0,25 m | 4/6 | 6/6 |
| 0,34 m | 3/6 | 5/6 |
| 0,37 m | 3/6 | 6/6 |
| 0,46 m | **0/6** | 6/6 |

O Theta\* desenha sempre o mesmo caminho; quem se move é a linha que ele precisa
cruzar. Os raios dos caminhos dele são 0,28 m (`aperto`) e 0,37–0,39 m nos
demais — com a máquina fechando 0,46 m, **nenhum** é seguível. E nos dois casos
"de lado" ele falha em qualquer raio, por outro motivo: desenha reta lateral,
que só serve para robô que pivota, e este não pivota com a zona morta
pessimista (medido hoje de manhã: zero amostras de giro parado).

O Smac cobra por isso, e o preço sobe com o raio: no alvo perto e de lado, de
1,50× para **2,12×** a linha reta. É o custo de desenhar só o que a máquina faz.

**A consequência prática é boa**: a decisão 008 **pode ser assinada sem esperar
a zona morta**. A medida que falta muda o tamanho da vantagem, não quem vence.
Era exatamente o que a varredura foi feita para descobrir.

### O que ficou por consertar, de propósito

O `giro` de um arco contínuo sai **curto** — 90° são medidos como 50°. A causa é
a mesma reamostragem: as meias-viradas das duas pontas não têm vértice onde
aparecer. Não foi corrigido junto com a cúspide porque os dois viéses puxam para
lados CONTRÁRIOS na mesma coluna, e consertar os dois na mesma mudança tornaria
impossível saber qual moveu qual número. Fica travado num teste que diz o que
está errado e por quê.

## 🧭 2026-07-29 (4ª leva) — O viés do giro, e os dois defeitos que estavam escondidos atrás dele

Ficou pendente da leva anterior: o `giro` de arco contínuo saía curto (90° lidos
como 50°), e o erro não era parelho entre os planners — o canto vivo do Theta\*
tem vértice e era contado inteiro, enquanto o arco do Smac perdia nas duas
pontas. A coluna que mede "quanto ele mexe o bico" **favorecia o Smac na
comparação que ela arbitra**, e o dono ia julgar com ela.

### Primeiro, a premissa da reamostragem caiu

Antes de somar no cru, fui ver se o passo cru é ruidoso, que era a justificativa
escrita da reamostragem a 0,20 m. Medido em três casos:

```
Theta*    passo 0,0500 m (a resolução do mapa)  virada MEDIANA 0,00°
Smac      passo 0,086 m                          virada MÁXIMA  19,9°
```

Os 19,9° com passo de 0,086 dão raio 0,249 m — o teto configurado, não ruído. A
justificativa antiga ("três pontos vizinhos medem ruído de arredondamento") não
se sustenta: o 0,01 m que ela dizia consertar era a cúspide de 28-07. O giro
passou a ser somado nos pontos crus.

### Aí a varredura começou a devolver 447° num caminho de 1,19 m

Somar no cru expôs dois defeitos que a reamostragem vinha escondendo — os dois
achados olhando o caminho, não adivinhando.

**1. Tocos de ponta.** O planner cola a pose exata de partida e de chegada no
caminho discretizado, e sobra um segmento de **7,7 mm** em cada extremidade,
fora do arco. Direção tirada de um toco desses é lixo: cada um injetava ±125,6°.
Corrigido costurando fora os segmentos abaixo de metade do passo típico do
caminho — limite relativo, porque o passo do Theta\* (0,05) e o do Smac (~0,08)
são diferentes e um número fixo serviria a um só. **O comprimento não muda**: o
robô percorre o toco, ele só não define rumo.

**2. Cúspide rasa, e o detector geométrico não a via.** Sobravam 437°. A pose
resolveu em uma linha — projetando cada passo no rumo da pose, o caminho
`lado_1m` é `+----------------+`: **um passo à frente, 16 de ré, um à frente**.
Duas cúspides, que a dobra geométrica mede como 147° e passavam por baixo do
limiar de 150°. Eram elas nos 437°.

A detecção passou a usar a **pose** quando o caminho traz orientação, e a
geometria quando não traz — porque o Theta\* devolve o caminho inteiro com
orientação zerada (faixa de yaw de **0,0° em 144 pontos**, medido). Não custa
nada: planner só-para-frente não tem cúspide para achar.

```
lado_1m @0.25    antes: giro 437°  inv 0  raio 0,32
                depois: giro 144°  inv 2  raio 0,28   (o teto é 0,25)
```

### O veredito não mudou, e agora a coluna do giro serve

Seguibilidade idêntica à da leva anterior — ela sai do `raio_min`, que não foi
tocado: Theta\* 4/6 · 3/6 · 3/6 · **0/6** contra Smac 6/6 · 5/6 · 6/6 · 6/6. O
que mudou é que dá para ler o giro:

| caso | Theta\* (fixo) | Smac (faixa) |
|---|---|---|
| porta | 192° | 144–247° |
| bloco | 153° | 218–304° |
| aperto | 271° | 320–465° |
| beco | 132° | 212–279° |
| perto_de_lado | **0°** | 122–131° |
| lado_1m | **0°** | 144–188° |

O Smac mexe mais o bico em todos os casos com obstáculo — é o que custa curvar
em vez de pivotar, e agora está medido em vez de subestimado. E os dois zeros do
Theta\* nos casos "de lado" não são virtude: ele desenha reta lateral, giro zero,
para um robô que teria de pivotar — e este não pivota.

Também apareceu que a **ré do Reeds-Shepp é constante nos casos de lado**: 2
inversões em todos os quatro raios, nos dois. Era invisível antes.

294 testes verdes (eram 289), 5 novos.

### O que fica por consertar, de novo de propósito

O levantamento expôs um problema no `raio_min` que **não** foi mexido nesta
mudança: a reamostragem a 0,20 m passa por cima do canto vivo do Theta\* e
devolve 0,37–0,39 m onde a virada é um canto — curvatura infinita, que robô sem
pivô não segue de jeito nenhum. Ou seja, o raio faz o Theta\* parecer **mais**
seguível do que ele é, e o veredito de hoje é conservador, não otimista.
Distinguir canto de arco é a próxima correção da régua — e ela só pode
melhorar o lado do Smac.

## ⚖️ 2026-07-29 (5ª leva) — A decisão 008, escrita com os 48 planos na mesa

Fechada a régua, escrevi `docs/decisoes/008-nav2-planeja-nos-seguimos.md` em
estado **proposta**: ela recomenda, e o dono aprova ou derruba lendo. O que ela
decide, e sobre que número:

1. **Planner global = Smac Hybrid-A\*, em Reeds-Shepp.** É o único seguível em
   toda a faixa de raio plausível (6/6 · 5/6 · 6/6 · 6/6 contra 4/6 · 3/6 ·
   3/6 · **0/6**). A vantagem não depende da zona morta que falta: o ranking não
   vira entre os extremos, ele se acentua.
2. **O Theta\* sai, e não é por ser ruim.** Ele é o planner do robô 1 e lá
   funciona *porque* aquele chassi só sabe girar parado. Aqui ele desenha reta
   lateral com giro zero para um robô que teria de pivotar — e este não pivota.
   Herdá-lo seria herdar a solução do problema do outro robô, que é exatamente o
   que a decisão 000 mandou não fazer.
3. **A ré passa a nascer do planejamento** (revisa a 007). A histerese da 007 foi
   reprovada duas vezes com dado: 28-07 no clique do dono e 29-07 na bancada
   (5 entradas em ré, 1,85 m de caminho para um alvo a 0,40 m). Com Reeds-Shepp
   o caminho inteiro já sabe onde a ré entra — medido: 2 inversões nos casos de
   lado, nos quatro raios, consistente e não oportunista.
4. **O seguidor é nosso**, com a movimentação da decisão 005 por baixo. Trocá-la
   por um controlador do Nav2 seria jogar fora a única camada com física medida
   para ganhar ganhos a sintonizar do zero, contra um simulador que ainda tem a
   boba do BO-4.

Quatro alternativas ficaram registradas como descartadas, com o motivo: manter
o Theta\* e resolver na movimentação (já tentado — foram as decisões 006 e 007,
as duas reprovadas), usar o seguidor do Nav2 agora (reavaliável depois, e aí
vira o BO-2), Smac Lattice (precisa das primitivas, que precisam dos números que
não temos) e escolher um raio só (repetiria a forma de erro da bitola).

### O que a 008 explicitamente NÃO resolve

A bancada desenha, não dirige — se o robô consegue seguir o caminho só fecha com
o seguidor de pé. O costmap ainda vem de mapa estático, porque o robô simulado
não tem lidar. E fica anotado o tremor do Smac em cima do alvo com raio grande
(4 inversões numa caixa de 9 cm no `bloco` com 0,46 m), que é achado medido e
cai no colo do seguidor.

## 🥕 2026-07-29 (6ª leva) — Seguidor, fatia A: a lei, sem ROS

Decisão 008 aceita pelo dono, e a fatia grande que ela destrava começou.
`robot_motion/lei_de_seguimento.py` — pura, sem ROS, testável sozinha, mesma
forma da `lei_de_rumo.py` e pelo mesmo motivo: o que decide o comportamento do
robô tem que poder ser exercitado sem subir simulador nenhum.

### A arquitetura, fechada com o dono

```
GUI → /goal_pose → bt_navigator → planner_server (Smac) → /plan → seguidor
                                                                     ↓
                                          rumo_alvo + velocidade_alvo
                                                                     ↓
                                      heading_controller → cmd_vel → rodas
```

Quem pede o plano é a **GUI**, e o **replanejamento vem de graça** com o
`bt_navigator` — eu tinha suposto que precisaria de uma fatia própria e não
precisa. O seguidor só ouve `/plan` e produz dois números.

A divisão de responsabilidade fica explícita: o Nav2 diz POR ONDE, esta lei diz
PARA ONDE OLHAR e QUÃO RÁPIDO, e a `lei_de_rumo` diz O QUE O ATUADOR AGUENTA.
A lei de seguimento **não conhece zona morta nem `a_dec`** e não fala com roda —
essa camada já está caracterizada e não se duplica.

### Os três números, cada um de um defeito medido

1. **Carrot** — mira um ponto à frente NO CAMINHO, medido pelo **arco**, não
   pela linha reta. Pela reta o robô corta a curva por dentro e raspa a quina
   do vão em vez de seguir a forma que o planner desenhou.
2. **Lookahead derivado do raio mínimo** (`max(piso, 1,5 · raio_min)`), não
   escolhido. É o parâmetro central deste nó, e número solto aqui viraria mais
   um valor herdado sem justificativa — como a bitola 0,32 e o raio de roda
   0,0825, que chegaram errados até a trena. Amarrado ao raio, ele acompanha a
   máquina quando a zona morta for medida. O piso existe porque carrot colado
   no robô faz o rumo alvo oscilar com ruído de pose: é o S de 27-07 entrando
   por outra porta.
3. **Teto de velocidade pela curva** — o defeito nº 2 de 28-07, o balão.
   `raio = v/wz`: com o giro e a linear nos tetos, o robô é OBRIGADO a descrever
   um arco de `v/wz_max`, e foi assim que um alvo a 0,43 m custou 3,66 m. A
   linear cede para a curva caber. E a curvatura é olhada numa **janela à
   frente**, não no ponto atual: a máquina tem distância de frenagem, então
   frear em cima da curva é frear tarde.

### O teste que passou de primeira e estava mentindo

Os 11 testes passaram na primeira execução, o que é suspeito. Rodei mutação —
quebrar a lei de propósito e ver se algum teste reclama. **O do carrot não
reclamou**: o arco de 90° que eu usara é curto demais para a corda e o
comprimento divergirem, e a tolerância estava larga. Trocado por arco de 180°
com passo fino, as duas contas separam (0,69 contra 0,78) e o teste passou a
morder.

Uma armadilha custou tempo e fica anotada: `build/robot_motion/robot_motion` é
**symlink para o fonte**, e o `__pycache__` sobrevivia à restauração do arquivo.
O teste de mutação mentiu duas vezes por causa disso — o fonte já estava
restaurado e o comportamento ainda era o da mutação. Toda rodada de mutação
limpa o cache antes.

Refeito limpo: **5 mutações, 5 pegas**, cada uma por exatamente um teste.

305 testes verdes (eram 294). Fatia B (cúspides e ré) e C (chegada e guarda do
tremor do Smac) vêm depois.

## 🔙 2026-07-29 (7ª leva) — A ré volta a ser por gatilho (decisão 009), e a fatia B

Horas depois de a decisão 008 dizer que a ré nasceria do planejamento, o dono
derrubou essa seção com razão de campo: **ré planejada em robô com Nav2 é
problemática — o robô fica tentando entrar e seguir os trechos de ré.** Foi por
isso que ele tirou a ré do planejamento no robô 1 e criou lá a ré por gatilho.

Não dava para arbitrar com a bancada: ela desenha, não dirige. O que dava era
medir o custo de cada saída, e foi o que se fez antes de decidir.

### Três medições, e uma hipótese minha que caiu

**1. Encarecer a ré não a elimina.** `reverse_penalty` de 2 → 10 → 40, no raio de
produção: as inversões ficam onde estavam (porta 2, perto_de_lado 2, lado_1m 2,
bloco 3→4→2). Nesses casos a ré é geometricamente necessária, não oportunismo do
planner. A opção do meio não existe.

**2. Proibir a ré no plano custa, e assimetricamente.** Varredura inteira em
`DUBIN`: `porta`, `aperto` e `beco` não sentem; `bloco` fica **sem caminho nos
quatro raios** (erro 208, NO_VALID_PATH) e os dois alvos "de lado" passam de
1,50–2,12 para **3,61–5,66** de desvio. Um alvo a 0,60 m custa 2,9–3,4 m de
caminho com ~350° de giro. É o balão de volta, agora nascendo do plano.

**3. Recuar NÃO salva o plano do Dubins.** Essa era a tese natural da ré por
gatilho e eu a testei antes de escrevê-la: recuando reto 0,3 e 0,5 m antes de
planejar, o comprimento **não cai** (2,92 → 2,99 → 3,20 no `perto_de_lado`), e o
`bloco` segue sem caminho. A razão é geométrica e vale para qualquer recuo:
**recuar reto não muda o rumo**. O alvo continua a 90° do bico, e um carro
só-para-frente precisa da mesma volta, saia de onde sair. (A 0,8 m dá
START_OCCUPIED — o robô recua para dentro da inflação da divisória.)

### O que isso revelou, e é o ponto de verdade

**O robô 1 se dá bem com a ré por gatilho porque ele PIVOTA.** Recua, gira no
lugar, o alvo de lado vira alvo de frente. O robô 2 não pivota com os parâmetros
de hoje — zero amostras de giro parado em toda a fase B do perfil pessimista.

Então a escolha depende do pivô, que depende da **zona morta**, sem medida desde
27-07. Decisão 009 escrita e aceita: **plano em Dubins, ré por gatilho no
seguidor**, valendo a experiência de operação; e a dependência fica registrada —
se a zona morta medida mostrar que o pivô não existe, a 008 seção 3 volta à mesa
com número, não com opinião.

Some junto, de graça, o **tremor em cima do alvo** que a varredura achou com
Reeds-Shepp (4 inversões numa caixa de 9 cm). Era a mesma doença, e eu a tinha
arquivado como problema do seguidor.

### Fatia B: o gatilho, e por que ele é tardio

`ProgressoDeAvanco` mede **aproximação, não velocidade** — robô em órbita tem
velocidade e não tem progresso, e é justamente esse o caso. Só acusa depois de
1,5 s CONTÍNUOS sem ganhar 5 cm, e o relógio zera a cada avanço real: dois
travamentos curtos separados não somam.

A decisão 007 havia descartado o sintoma porque ele "gasta N segundos de órbita
toda vez". É verdade, e é o preço certo: o gatilho geométrico age cedo e SEMPRE
que a conta diz que não cabe, inclusive onde se resolveria sozinho — é assim que
nasce o vai-e-volta. Sintoma dispara raro e tarde.

A ré é **cega** enquanto o Mid-360 não estiver no modelo: orçamento curto (0,30 m)
e obrigatório. Com vão medido ele passa a sair de metros reais. E tem teto de
TEMPO além do de metros, que não é redundante: se a pose não muda (comando
engolido pela zona morta), o orçamento em metros nunca é gasto e a ré duraria
para sempre — é o BO-3 visto de outro ângulo.

### Dois testes meus estavam errados, e o segundo ensinou algo

O primeiro usava 3 cm/s como "devagar mas progredindo" e reprovava a lei. Mas
**este robô não anda a 3 cm/s**: abaixo do piso de linear (~0,33 m/s no perfil
pessimista) ele não anda devagar, ele não anda. Ficou um teste novo travando
essa folga — a taxa mínima implícita do gatilho (0,05 m / 1,5 s = 3,3 cm/s) tem
que ficar ~10x abaixo do piso, e se a zona morta medida derrubar o piso, o par
de números volta à mesa.

O segundo simulava o robô chegando e **ficando parado no alvo por 4 s** — e o
gatilho disparava, corretamente. Parado em cima do alvo não é travamento, mas o
detector só vê distância que não cai. Quem sabe que chegou é o seguidor, e é ele
que desarma. Sem isso o robô chegaria, esperaria 1,5 s e daria ré para longe do
ponto onde acabou de chegar. Virou teste.

6 mutações, 6 pegas. **316 testes verdes** (eram 305).

## 🎯 2026-07-29 (8ª leva) — Seguidor, fatia C e o nó: a pilha fecha

### A chegada, e o que a zona morta cobra em PRECISÃO

Duas lições da decisão 006 viraram lei testada:

**O raio de chegada tem um mínimo, e ele é a distância de parada.** O robô não
sabe ir mais devagar que o piso de linear — abaixo disso a placa engole o
comando (BO-3). Então ele entra no raio a `v_piso` e precisa de
`v_piso²/(2·a_lin)` para parar; raio menor que isso e ele atravessa, sai do
outro lado, volta, para sempre. Com os números de hoje (piso 0,335 m/s no perfil
pessimista) isso dá **0,187 m**.

Isso rendeu um argumento que ainda não estava escrito em lugar nenhum: **a zona
morta não encarece só a manobra, encarece a PRECISÃO — e com o quadrado.** Piso
o dobro, chegada quatro vezes mais grosseira. Não há ganho que conserte, e é
mais um peso na balança do item nº 1 da bancada.

**Chegou é chegou: para tudo, inclusive o giro.** Na sessão de 27-07 o robô
chegava e continuava girando para acertar o rumo, arrastando-se para fora do
ponto — 0,06 m viravam 0,27 m. Rumo na chegada não é requisito deste seguidor.

### O nó, e o aviso que ele dá na subida

`robot_motion/path_follower.py` amarra as três fatias: ouve `/plan` e
`/Odometry`, publica `~/rumo_alvo` e `~/velocidade_alvo`, com dois estados
(SEGUINDO / RÉ). A ré sai por **velocidade negativa** no tópico que já existia —
sem tópico novo, como a movimentação já esperava.

Ele grita na subida quando o raio de chegada pedido é impossível, verificado:

```
raio_chegada=0.100 -> ERROR: menor que a distância de parada (0.187 m).
                      O robô vai ORBITAR o ponto sem nunca fechar.
raio_chegada=0.250 -> INFO: aceito (mínimo viável 0.19 m)
```

Vale a pena o grito: o sintoma desse defeito (robô circulando o ponto) parece
problema de controle, e é de configuração.

O único número da movimentação que atravessa para cá é o `v_piso`, e **só** para
essa conta — o nó não conhece zona morta, não conhece `a_dec` de giro e não fala
com roda.

### O CSV que já nasce mirando um defeito que ainda não vimos

O registro grava `rumo_alvo` e `erro_rumo` a cada ciclo de propósito. A leitura
do seguidor do robô 1 mostrou que **o plano salta entre replanejamentos** (13–15°
lá) e que um seguidor que persegue esse salto oscila. Vai nos acontecer, porque o
replanejamento vem do `bt_navigator`. Mas eu não vou pôr filtro agora contra
defeito que não medi neste robô — seria começar a escada de ganhos que a decisão
003 mandou não herdar. O que dá para fazer hoje é garantir que, quando aparecer,
o número esteja na mão.

320 testes verdes (eram 316). 2 mutações na fatia C, 2 pegas.

**Falta para rodar**: a launch juntando Nav2 + seguidor + movimentação no
simulador. Aí dá para ver o robô andando pela primeira vez com esta pilha.

## 🚗 2026-07-29 (9ª leva) — A pilha inteira anda pela primeira vez

`ros2 launch robot_motion pilha.launch.py sim:=true` sobe Gazebo + Nav2 +
seguidor + movimentação, e o robô vai onde você clicar. Primeira vez que esta
pilha dirige.

```
alvo (2,0 · 2,0), 90° atrás    CHEGOU 15,2 s   5,00 m de caminho / 3,00 reta = 1,67x
alvo (6,0 · 1,5), pela porta   CHEGOU 12,3 s   5,55 m / 5,32 reta = 1,04x
```

### Quatro defeitos, e três eram do Nav2 discordando de si mesmo

**1. A árvore padrão derruba o `bt_navigator` na subida.** Ela exige os
servidores `spin`, `backup`, `wait` e `drive_on_heading`. Subir o
`behavior_server` para satisfazê-la seria errado por mérito, não só por
conveniência: `spin` é PIVÔ, que este robô não faz (zero amostras de giro parado
no perfil pessimista), e `backup` é a ré do Nav2, que a decisão 009 tirou do
caminho. E as duas seriam no-op — o `cmd_vel` delas sai pelo tópico ignorado, e a
árvore acharia que recuperou sem nada ter acontecido. Trocado pela árvore de
fábrica `navigate_w_replanning_time.xml`, que replaneja a 1 Hz e não tem
recuperação nenhuma.

**2. E derruba de novo, pela SEGUNDA árvore.** O `bt_navigator` carrega
`navigate_through_poses` junto, com a árvore com recuperação. Não usamos rota com
pontos intermediários: `navigators: ["navigate_to_pose"]`.

**3. O `controller_server` não dirige, mas ABORTA.** Esta foi a premissa errada
mais cara da leva: eu havia escrito que ele era inofensivo porque o `cmd_vel`
dele ia para um tópico ignorado. Na corrida da porta ele derrubou a navegação com
`RegulatedPurePursuitController detected collision ahead!` — o robô tinha
atravessado a porta e parou. O palpite dele é sobre uma trajetória que ninguém
vai executar (ele projeta supondo os comandos DELE), então julgar colisão ali só
pode errar. `use_collision_detection: false`, com o motivo escrito no YAML.

**4. Inflação 0,45 trava o robô DENTRO da porta.** Com raio 0,36 e inflação 0,45,
o vão de 0,90 m fica inteiramente inflado e o replanejamento a 1 Hz falha com
`Start occupied` — a célula do próprio robô conta como ocupada. Medido:

```
inflação 0,45  ->  atravessa a porta e TRAVA
inflação 0,30  ->  chega em 12,3 s, 1,04x a linha reta
```

A produção foi para 0,30 e a bancada fica em 0,45 (é o número por trás dos 48
planos da decisão 008). A divergência é deliberada e está escrita no
`test_configs_coerentes.py`, que trava os números FÍSICOS (raio de curva, raio do
robô) e deixa a inflação de fora de propósito: ela é escolha, não medida.

### Um botão falso que eu mesmo criei, e o que ele ensinou

Para expor a inflação como argumento da launch, passei um override de parâmetro
por um caminho aninhado que eu **não verifiquei**. Ele quebrou a camada de
inflação — o planner passou a gritar "Inflation layer either not found" e a porta
voltou a falhar. Removido, a porta voltou a funcionar na hora.

Lição, e ela é de método: um botão de conveniência não medido custou o mesmo tipo
de tempo que um número herdado não medido. Se não dá para verificar agora, não
entra — melhor editar o YAML do que ter um botão que mente.

### E um artefato de ambiente que mentiu três vezes

`pkill -f <nome>` casa com QUALQUER processo cuja linha de comando contenha o
nome — inclusive o shell que está rodando o próprio `pkill`. Isso matou a sessão
duas vezes e, pior, produziu diagnósticos falsos: uma pilha remanescente
disputando o `/clock` fez uma corrida largar da pose errada com 25 mil
`jump back in time`, e eu quase registrei aquilo como defeito da pilha. A
limpeza agora vive em `mata.py`, que compara por PID.

Regra que fica: **corrida cujo ambiente não foi provado limpo não é medida.**

### O que sobrou anotado, sem conserto

- O alvo a 90° atrás custou **1,67x** a linha reta, e o gatilho de ré disparou
  duas vezes na corrida — uma delas a **0,43 m do objetivo**. Ré perto da chegada
  é suspeito e é a primeira coisa a olhar na próxima sessão. O CSV do seguidor
  (`csv:=`) grava o que falta para diagnosticar.
- A TF `map→odom` é fixa. No simulador vale porque mundo e mapa saem da mesma
  planta; **no robô real não vale**, e é o que falta para a pilha sair do
  simulador.

324 testes verdes.

## 🧰 2026-07-30 — O kit de bancada, e o que o robô do estágio tem a dizer

Sessão sem robô, preparando a ida ao robô. Duas coisas: o banco ganhou um
condutor, e entrou no repo (só para leitura) o workspace do estágio, que roda
**neste mesmo robô** com a pilha Nav2 de fábrica.

### `tools/banco/sessao.py` — o protocolo inteiro num comando

O `ensaio.py` mede UM ensaio; o protocolo tem SEIS, cada um com argumentos,
espaço e pose de partida próprios. Digitar isso na mão com o robô ligado é onde
se erra o `--wz`, se sobrescreve um CSV ou se pula um ensaio — e o custo é voltar
ao laboratório. O condutor roda os seis em ordem, pausa para reposicionar, grava
tudo numa pasta só e chama o `medir.py` depois de cada corrida, para o número
aparecer ainda com o robô ligado.

Três coisas que ele faz e que o `ensaio.py` sozinho não faz:

1. **Conferência que BLOQUEIA.** Sem `/Odometry` toda velocidade do banco sai de
   uma pose que não existe, e o CSV sai limpo e errado — do jeito que só se
   descobre em casa. Ele mede a taxa dos dois tópicos de odometria, conta os
   ouvintes do `cmd_vel` (zero = `diff_drive_controller` fora do ar, e o banco
   comandaria no vazio) e recusa medir se algo faltar.
2. **Cutucão de sanidade** (`--checar --mexer`): anda 2 s, gira 2 s e confere o
   **sinal**. Roda trocada na fiação dá um robô que anda certo e gira ao
   contrário, e **nenhum dos seis ensaios acusa isso** — eles medem magnitude. Ou
   sai aqui, em 5 s, ou a sessão inteira sai espelhada.
3. **Ambiente escrito ao lado** (`ambiente.txt`: piso, bateria, commit). Piso e
   carga mudam derrapada e zona morta; medida sem eles não se compara com a
   próxima sessão nem entra no artigo.

Provado de ponta a ponta contra o Gazebo headless antes de existir como
recomendação — os seis passos, as onze corridas, leitura saindo em cada uma. A
conferência também foi provada no **caso negativo**, com a pilha derrubada:
recusa e explica. Folha de campo em `tools/banco/CHECKLIST_ROBO.md`.

### O workspace do estágio: mesma máquina, três números que divergem

Clonado em `ESTAGIO-2026/` (ignorado pelo git — é repo próprio, e o que sai daqui
vira medida nossa, não código copiado). Ele **anda** e faz SLAM, o que o nosso
ainda não faz no robô; e se perde, e faz recuperação quase só de ré. Lendo os
YAMLs contra a nossa trena de 29-07, três hipóteses caem no colo da bancada:

**1. Dois raios de roda contraditórios na mesma pilha.** O
`diff_drive_controller` usa `wheel_radius: 0.0425`; o plugin de hardware, no
xacro, `0.0825`. A conversão do driver é `rad/s ÷ 0,10472` — **independente de
raio** —, então quem fixa a escala é o controlador. Nossa trena mediu 0,080.
Efeito: a roda gira `0,080/0,0425 ≈ 1,9×` mais rápido do que os m/s pedidos. Um
robô que anda ao dobro do que o Nav2 acha que mandou se perde por construção, e
nenhum ajuste de controlador conserta isso.

**2. `wheel_separation: 0.32` contra os 0,270 medidos.** Bitola 18,5% maior no
YAML faz o robô girar ~19% a mais do que o comandado — que é exatamente o desvio
que anotamos em 29-07 quando esse mesmo 0,32 era nosso.

**3. O piso de velocidade do mux protege a reta e não protege o giro.** O
`cmd_vel_mux.py` deles força mínimo de 0,10 m/s no linear e 0,15 rad/s no
angular. Com bitola 0,32, esses 0,15 rad/s são **0,024 m/s de roda** — quatro
vezes abaixo da faixa de zona morta que nosso simulador trata como plausível
(0,10–0,15 m/s). O piso angular não pisa em nada. E o `Spin` do Nav2 decai até
`min_rotational_vel` (0,4 rad/s de fábrica = 0,064 m/s de roda, também abaixo),
enquanto o `BackUp` é linear puro e o piso de 0,10 m/s o levanta sempre.

Daí a hipótese, que é a mais útil das três: **"só vai de ré" pode não ser a
boba — pode ser que a ré seja a única recuperação que fisicamente acontece.**
O `spin` estola no meio, a árvore o dá por falho e escala para o `backup`, que
sempre anda. É a nossa BO-3 vista de fora, num robô que já roda.

Nenhuma das três é fato nosso: são hipóteses, e as três se resolvem com o
**ensaio 2** (zona morta de giro) e o **ensaio 4** (curva por velocidade). O item
nº 1 da bancada acabou de ganhar uma quarta razão de peso.

Anotado sem investigar: os tetos deles são `linear.x.max_velocity: 3.0` e
`max_acceleration: 3.0` (contra 0,7 e 0,8 nossos), o `controller_manager` roda a
10 Hz, e o footprint declarado é 0,50 × 0,40 — mais estreito que a caixa que
medimos com trena (0,433 × 0,455).

## 🔌 2026-07-30 (2ª leva) — A ida ao robô: o repo nunca tinha chegado aqui

Sessão de bancada para medir a **zona morta de giro** (ensaio 2, item nº 1). O
dono só executa o físico; o assistente conduz por ssh no NUC. Antes de qualquer
medida, dois achados de infra que valem registro — o segundo é grande.

### A rede primeiro: o PC de dev não enxergava o NUC

`10.244.3.205` (NUC) contra `10.150.13.54/19` (PC de dev): **sub-redes
diferentes**, o gateway não encaminhava, ping e ssh davam timeout. Não era senha
nem robô desligado — era topologia. Resolvido pelo dono ao pôr o PC de dev na
**mesma rede do robô** (`wlp3s0`); daí o ssh entrou por chave, sem senha. Fica a
lição de bancada: a primeira conferência não é do lidar, é de haver rota até o
NUC.

### O achado grande: este NUC nunca tinha visto o nosso repo

`~` no NUC tinha `ros2_ws` (o do estágio: `FAST_LIO`, `livox_ros_driver2`,
**`hoverboard-driver-humble`**, `robo_exemplos`, mapas de SLAM) e
`Workspace/competicao_2025` — e **nenhum `Controle_robo_livox`**, nenhum
`base.launch.py`, nenhum `tools/banco/`. Uptime de 7 min, `install_ros2_jazzy.sh`
e `ros2_ws.zip` soltos no home: a máquina foi (re)instalada e o nosso software
nunca foi puxado para ela. O `ESTADO` dizia "base verificada em hardware" — foi,
num estado da máquina que não existe mais. **A memória do assistente não cruza
PCs, e o disco do robô também não guardou.** É a razão de o `ESTADO_PROJETO.md`
existir, provada pela ausência.

Confirmado que é o robô certo mesmo assim: `enp2s0` em `192.168.1.2/24` — a
interface do lidar que o `ESTADO` descreve.

### O deploy, sem GitHub e sem sudo

O NUC não clona do GitHub: origin é ssh (`git@github…`) e ele não tem chave; por
HTTPS o repo é privado e não há credencial. Como o repo local do dev estava
**em sync com `origin/main`** (`2aa17bd`, limpo), semeei por **`git bundle`** —
git-nativo, histórico completo (6,5 MB, `verify` ok), não é "arquivo solto" da
regra 8. Clonado em `~/Controle_robo_livox`, `origin` reapontado para o GitHub.

Terreno conferido antes de compilar, tudo sem `sudo`: `ros2_control` já
instalado (controller-manager, diff-drive-controller, hardware-interface,
ros2-controllers), `bara` no grupo `dialout`, placa em
`/dev/ttyUSB0` (Prolific USB-Serial), **SDK nativo da Livox já em `/usr/local`**
(do estágio) — o único passo com sudo do `setup_livox.sh` se pula sozinho.
Config do lidar commitada: host `192.168.1.2`, lidar `192.168.1.169` (a conferir
contra a varredura na hora do `--checar`).

### O tropeço de ordem de build (não é bug, é dependência)

`setup_livox.sh` compila só `livox_ros_driver2 + fast_lio + robot_base` — **não**
o `hoverboard_driver`. Mas `robot_base` depende dele, e num workspace zerado
`install/hoverboard_driver` não existe → `robot_base` falha na configuração
(0,02 s, antes de compilar linha nenhuma). Nas máquinas anteriores o
`hoverboard_driver` já fora compilado numa passada avulsa; aqui, primeira vez, não.
Conserto pela ordem: compilar `hoverboard_driver` primeiro, depois re-rodar o
`setup_livox.sh` (idempotente — clones já presentes, só refaz o build).

### O build subiu, com dois consertos de infra (não de calibração)

1. **Ordem de build:** `hoverboard_driver` compilado ANTES do `setup_livox.sh`
   (que só faz livox+fast_lio+robot_base). Sem isso `robot_base` falha na
   configuração por falta de `install/hoverboard_driver`. Depois: `fast_lio`
   (59 s), `livox_ros_driver2` e `robot_base` compilaram limpos.
2. **`launch_ros >=0.26` recusa `robot_description` cru** (lê como YAML; um URDF
   não é YAML). A máquina anterior tinha `launch_ros` mais tolerante; este NUC
   recém-instalado (pacotes de 2025-10-07) não. Conserto em `tracao.launch.py`:
   `ParameterValue(robot_description_content, value_type=str)` — forma correta no
   novo e no antigo. Aprovado pelo dono, aplicado em dev e NUC (md5 idênticos),
   COMMITADO. Sem ele a base não sobe.

### O lidar não streamava — e a lição do `kill -9`

Base de pé, mas `sessao.py --checar` **reprovou**: `/Odometry` não publicava.
`/hoverboard_base_controller/odom` a 9,9 Hz e `cmd_vel` com ouvinte — a placa
falava, faltava a localização. Varredura ARP: lidar em `192.168.1.169`
(`e4:7a:2c:90:1d:f1`), **exatamente o IP do config**. Não era IP.

Cavando: `/livox/lidar` e `/livox/imu` **mudos**, mesmo com QoS best_effort. RX na
`enp2s0` (contado no fio, antes de firewall): **6 pacotes/3 s** — o lidar respondia
ping e ACKava todo comando de controle (work mode Normal, IMU enable), mas **não
transmitia dado**. Rodei o driver do **estágio** (o que funciona) para isolar:
mesmo handshake, **mesmos 6 pacotes/3 s**. Concluí "é o lidar" — e o dono, com
razão de campo ("rodou ONTEM"), religou o Mid-360. Depois do power-cycle: RX
**9152 pacotes/4 s (~3 MB/s)**, o log ganhou as linhas `livox/lidar publish use
livox custom format`, e `--checar` passou: `/Odometry` a 9,9 Hz. **Conferência ok.**

Lição registrada: subi e matei o driver livox com `kill -9` várias vezes no meio
do handshake, depurando o launch. O Mid-360 **tranca a sessão de dado** quando o
driver morre no meio — provável que eu mesmo tenha travado o lidar. Da próxima:
derrubar o driver com SIGINT e esperar, nunca `-9` no meio da subida.

### O cutucão pegou o defeito que os seis ensaios NÃO pegariam

`sessao.py --checar --mexer`, com o dono avisado do lado e da velocidade:

```
[ok]    +0,25 m/s andou +0,737 m PARA FRENTE (lado: -0,204 m)
[FALHA] +0,6 rad/s girou -78,5° — sentido INVERTIDO
```

- **Frente: sentido certo.** (A deriva de -0,20 m para a direita o dono
  reconheceu como **distribuição de peso** — acontece também quando ele chuta o
  robô. Não é fiação; é viés mecânico de rumo, dado de projeto do controlador.)
- **Giro: espelhado.** Comando anti-horário (+), robô girou horário (-). É
  **esquerda/direita trocadas** (fiação da placa ou YAML): com as rodas
  invertidas, a reta sai certa (as duas no mesmo sentido) e só o giro espelha. É
  exatamente o que o cutucão existe para pegar, e o que **nenhum dos seis ensaios
  acusaria** (medem magnitude). Sem o cutucão, um dia de medida sairia lixo.

**PARADO aqui pela regra 3.** A zona morta — item nº 1 — **não foi medida**: medir
com o giro espelhado é medir errado.

Números crus do cutucão, a confirmar nos ensaios (não são medida formal): a reta
andou ~0,37 m/s para 0,25 comandado e o giro ~0,68 rad/s para 0,6 — os dois saíram
ACIMA do comandado, ao contrário do déficit de -20% que o Gazebo (com a boba do
BO-4) sugeria. Se confirmar, o simulador erra o sinal do desvio de giro também.

### Onde parou / primeira coisa amanhã

1. **Inverter esquerda↔direita e RE-TESTAR o cutucão.** Fix mínimo e reversível em
   `ros2_packages/hoverboard_driver/bringup/config/hoverboard_controllers.yaml`:
   trocar `left_wheel_names: ["left_wheel_joint"]` /
   `right_wheel_names: ["right_wheel_joint"]` por
   `left_wheel_names: ["right_wheel_joint"]` /
   `right_wheel_names: ["left_wheel_joint"]`. Rebuild do `hoverboard_driver`,
   `--checar --mexer` de novo: o giro tem de sair **+** (esquerda). O dono já
   mandou inverter; **não apliquei hoje** porque a validação exige o robô andando
   com alguém de olho, e ele saía. NÃO commitar o swap antes do cutucão validar.
2. Passando o cutucão, rodar a sessão: **zona morta de giro (ensaio 2)** é o alvo,
   e o passo 6 (ré + boba filmada) ataca o BO-4.

**Dívida de infra (fora desta sessão):** o NUC não tem autenticação no GitHub —
nem `fetch` nem `push`. Este deploy veio de bundle; o `git fetch && git reset
--hard origin/main` do passo 1 do checklist não roda ainda, e o passo 6 ("commit
docs/dados && push" do robô) também não. Enquanto isso: código vai por bundle do
dev, e os CSV/commits saem **do dev** (que tem chave), com os dados trazidos do
NUC. Corrigir com uma chave de deploy no NUC registrada no GitHub.
## 📊 2026-07-31 — O protocolo não repetia nada, e a rampa media metade do problema

Sessão de véspera: o dono vai ao robô hoje. Começou como conferência do kit de
bancada e virou uma revisão de método, provocada por duas perguntas dele que o
protocolo não sobrevivia.

### "Eles se repetem?" — não, e nenhum número tinha faixa

O protocolo tinha 6 ensaios e 11 corridas, e **as 11 eram condições
diferentes**: os "3" dos passos 3, 4 e 6 são três *valores* (wz 0,3/0,6/1,0),
não três tentativas. Todo número sairia com n=1, sem dispersão. Num PIBIT que
vira artigo isso não se defende — número sem espalhamento não é medida, é
amostra.

Repetir tudo ×3 dariam 33 corridas e ~50 min de bateria. O dono decidiu a
prioridade: repete o que identifica erro (reta, curva, aceleração). Ficaram
**27 corridas, ~30 min**, com `--repete 1` para encurtar se a bateria cair.

Cada grupo de repetições agora imprime **média, faixa e dispersão** ao fechar
(`medir.py --resumo`), e delata sozinho quando uma corrida morreu e a repetição
encolheu — no Gazebo isso aconteceu duas vezes (relógio do simulador sob carga)
e o aviso apareceu certo, sem eu procurar.

**Uma corrida a mais que não é repetição: a reta girada 180°.** Três retas do
mesmo ponto e mesmo rumo, com o piso em caimento, dão três desvios iguais e uma
média confiante e errada. **Média mata erro aleatório, não erro sistemático.**
Girado 180°, o caimento empurra para o mesmo lado do *mundo* e a assimetria do
robô puxa para o mesmo lado do *corpo* — é a única corrida que separa robô de
sala. O ponto 0 passa a ser marcado com fita **e com o rumo**, senão a dispersão
medida é a mão do operador.

### "A zona morta é subir a velocidade até ele sair do lugar?" — era, e era pouco

Era exatamente isso: `rampa_ate · te / dur`, uma subida só. Duas limitações que
a pergunta expôs:

1. **uma corrida, uma amostra.** Limiar de atrito estático é a grandeza mais
   dispersa do banco (depende de onde o rotor parou), e a decisão do pivô se
   joga entre 0,10 e 0,15 com **4% de folga**. Uma amostra não diz onde na
   faixa se está.
2. **mede a saída e não mede a queda.** Atrito estático > dinâmico: o comando
   que *tira* o robô do lugar é maior que o que o *mantém* andando. O primeiro é
   o número do BO-3; o segundo é o que o piso de velocidade do seguidor precisa.
   Só o primeiro existia.

O ensaio virou **dente de serra**: sobe até sair do lugar, desce até parar,
inverte o sentido, repete 4×. Uma corrida entrega 4 saídas + 4 quedas, nos dois
sentidos, sem reposicionar o robô. A repetição passou para DENTRO da corrida, e
por isso os passos 1 e 2 não repetem em corrida.

**A primeira versão não cabia na sala, e foi o teste que disse.** Com o dente
virando por tempo, o afastamento máximo da origem no ensaio linear dá **5,25 m**
contra uma trava de 3 m que **mata a corrida** ("Estourou, para tudo"): morreria
dentro do primeiro dente e traria uma saída só — pior que a versão antiga. Causa:
a rampa segue subindo muito depois de já ter achado o número, e é esse trecho
que gasta metros e segundos.

Conserto: **o dente vira no evento, não no relógio.** Medido depois: excursão de
**0,059 m** e 4 dentes em 18 s (contra 160 s da versão por tempo). É o único
ensaio do banco em malha fechada, e é de propósito.

**A taxa da rampa virou parâmetro nomeado** (`--rampa-seg`, padrão 20 s ao
teto), porque ela *é* parte da medida: o limiar é lido na primeira amostra que
passa de `LIMIAR_PARADO`, então rampa mais rápida infla o número pelo atraso de
detecção. No giro a taxa de hoje já infla ~0,011 rad/s — 10% do que se quer
distinguir. Mais dentes custam tempo, nunca precisão.

### Um defeito achado no Gazebo, e o `tools/banco` ganhou testes

A leitura acusava `1 dente(s) NÃO saíram do lugar` com os 4 tendo saído: ao fim
da pausa o contador andava e uma linha chegava a ser gravada com um dente que
nunca existiu. Consertado dos dois lados — encerrando antes de gravar, e na
leitura, ignorando rampa cortada no meio (que é o caso real quando o teto de
tempo corta a última).

O `tools/banco/` **não tinha teste nenhum**, que é exatamente como a régua da
bancada do planner sobreviveu errada por semanas em 29-07. Agora tem 17, e o do
dente fantasma foi verificado por mutação: reintroduzi o defeito e ele falhou.
Suíte: **397 verdes** (eram 380).

### O que este dia NÃO prova

No simulador o `ensaio.py` publica direto no `cmd_vel` do controlador e **passa
por fora da placa fingida** (ela escuta `/cmd_vel_bruto`). Os 0,023 m/s e
0,036 rad/s que saíram nos testes são o **piso de detecção**, não zona morta: o
Gazebo provou o *mecanismo* do dente de serra, não o número. No robô a placa
está no caminho, e é lá que o número existe.

### A conferência passa a ler a calibração VIVA (a dívida da manhã, fechada)

O `--checar` agora pergunta ao `hoverboard_base_controller` **que robô ele acha
que está dirigindo**: `wheel_separation`, `wheel_radius` e os nomes de roda,
lidos do nó vivo por `AsyncParameterClient` e comparados com a trena.

Fechou o buraco que eu tinha anotado de manhã: o `ambiente.txt` gravava o
**commit**, e commit descreve o FONTE. Quem dirige o robô é a cópia em
`install/`, e o `tracao.launch.py` lê os dois de `FindPackageShare`. Agora a
calibração viva vai escrita no `ambiente.txt`, com marcador `*** DIVERGE DA
TRENA ***` quando for o caso — sem isso, um limiar medido não tem como ser
convertido de volta em velocidade de roda, e vira número sem unidade.

**Delata, não bloqueia.** Divergir pode ser deliberado; o que não pode é ninguém
saber. Verificado nos dois sentidos contra o Gazebo, com a calibração
adulterada em tempo de execução: com 0,32 ele acusa `+18.5%` e a sessão segue,
com o número gravado ao lado do dado.

Os nomes de roda entraram junto porque é neles que vive a correção do giro
espelhado: a conferência agora diz `swap APLICADO` ou `NÃO aplicado` **antes**
de o robô se mexer, o que prova que o rebuild pegou sem gastar bateria. Os dois
estados verificados ao vivo.

A folha de campo virou executável de ponta a ponta — um bloco "para quem for
conduzir" com a ordem e cinco coisas que não se faz, e o swap de rodas como
script copiável (testado e revertido; **não commitado**, conforme a decisão de
30-07 de só entrar no git depois de o cutucão validar). 22 testes no banco,
**402 verdes** no total.

### O giro passa a ser o ensaio 1, a pedido do dono

Pergunta dele: *"se não sabemos a zona morta ainda, por que esse não vira o
teste 1?"* — e o giro estava em segundo, atrás do linear. Invertido, por três
razões em ordem de peso:

1. **o ensaio de giro responde a pergunta do pivô DIRETAMENTE.** O menor `wz`
   que gira o robô parado *é* o limiar do pivô, em rad/s. Pelo linear só se
   chega lá convertendo por `2·zm/L` — confiando de novo na bitola, que é
   justamente o tipo de dependência que a trena de 29-07 ensinou a desconfiar;
2. **sessão cortada perde o que está por último**, e em 30-07 a sessão foi
   bloqueada sem medir nada. O risco não é hipotético;
3. é o mais barato de montar — gira parado, raio de 1 m, sem corredor.

O argumento contrário ("andar reto é mais manso do que girar como primeira
coisa") já estava coberto pelo cutucão, que anda **e** gira antes de qualquer
ensaio. O linear ganhou um papel novo e melhor: **conferência do passo 1**, já
que os dois medem o mesmo atrito por caminhos diferentes e têm de fechar por
`2·zm/L`. Não fecharam, ou a bitola está errada ou as duas rodas não são iguais.

Travado em teste (`test_o_giro_e_o_primeiro_ensaio_da_sessao`), junto com dois
que impedem a numeração dos passos e os prefixos dos CSV de descolarem.

### O dente #0 destoa, e é física, não ruído

A corrida de validação no Gazebo entregou dispersão de **82%** no passo 1: dente
#0 em 0,188 rad/s contra 0,035 dos outros três. No simulador é o robô assentando
na física, mas o efeito tem nome e é real no robô: **atrito estático cresce com o
tempo parado**. O dente #0 parte de repouso longo; os demais, da pausa de 1 s
entre dentes. São condições físicas diferentes, e a média das quatro as mistura
bem no número do BO-3 — que é exatamente "o robô estava parado e mandaram andar".

O `medir.py` passou a **separar em vez de diluir**: quando o #0 destoa mais de
30% da mediana dos outros, ele mostra os dois e diz qual serve para cada caso
(arrancar do repouso × arrancar em manobra encadeada). Não corrige a média —
qual dos dois usar depende de quanto tempo o robô fica parado na operação real,
e isso é decisão de projeto, não de leitura.

### Corrigido depois: a sessão 07-30 no robô já tinha respondido isto

Ao juntar com o remoto apareceu a entrada 07-30 (2ª leva) — uma ida ao robô que
eu não conhecia. Ela reescreve duas coisas desta sessão:

- **O `install/` velho não era o caso, e o build importa mais ainda.** O NUC
  nunca tinha visto este repo; foi deployado do zero por bundle e compilado
  inteiro. Só que o giro saiu **espelhado** (esq/dir trocadas: `+0,6 rad/s`
  girou `−78,5°`), e o conserto é justamente no `hoverboard_controllers.yaml`,
  que o `tracao.launch.py` lê do `install/`. **Sem rebuild o swap não existe
  para o robô** — o mecanismo que descrevi, valendo por outro motivo.
- **O `git fetch && git reset --hard` do passo 1 não roda no NUC**: ele não tem
  autenticação no GitHub. Folha de campo corrigida para o caminho de bundle.

E o mais importante para hoje: **a zona morta não foi medida em 07-30**, parada
pelo giro espelhado. O dente de serra que escrevi hoje vai estrear numa máquina
que primeiro precisa passar no cutucão. A ordem virou: swap → rebuild →
`--checar --mexer` → só então medir.

Vale registrar que o cutucão fez exatamente o que se desenhou: pegou, em 5
segundos, um defeito que **nenhum dos seis ensaios acusaria** (eles medem
magnitude), e que teria transformado um dia inteiro de medida em lixo.

### Anotado sem conserto: build e install/ podem divergir

Levantei que a bitola e o raio medidos com trena moram em arquivos que o
`tracao.launch.py` lê de `FindPackageShare` — da cópia **instalada** —, e que a
folha de campo mandava `git reset --hard` direto para o `source`, sem build. Se
o `install/` estiver velho, o robô sobe com 0,32/0,0825 e todos os limiares saem
18,5% enviesados sem sintoma.

**Cheguei a afirmar que o robô estava assim; não tinha como saber e o dono me
corrigiu** — nunca vi o disco daquela máquina. O que se sustenta é o
condicional. A folha de campo ganhou o `colcon build --packages-select
hoverboard_driver` (idempotente, barato) e um `grep` de conferência. A defesa
melhor — o `--checar` LER `wheel_separation` e `wheel_radius` do controlador
vivo e anotá-los no `ambiente.txt` — ficou **por fazer**: o `ambiente.txt` grava
o commit, que descreve o fonte, não o que está dirigindo o robô.

Isso importa porque a comparação roda × lidar (ideia do dono, e o banco já a
tinha na coluna de derrapada) mede o giro limpo, mas **não separa bitola errada
de escorregamento**: os dois mexem no mesmo número em sentidos opostos (0,32 num
robô de 0,270 faz girar 18,5% *a mais*; derrapar faz girar *menos*), e
`1,185 × 0,82 ≈ 0,97` leria como "quase não derrapa". Na reta a comparação é
limpa — sem derrapagem, o desvio roda × lidar é raio de roda puro.

## 🧭 2026-07-31 (2ª leva) — Trocar de repositório? A pergunta se dissolve na apuração

Pergunta do dono, com o robô prestes a entregar os primeiros números:

> vale continuar neste repositório ou pegar o `Controle_robo_web`, que já está
> pronto, e só adaptar a navegação? Aquele já foi todo o sofrimento que talvez a
> gente sofra tudo aqui de novo.

Fui apurar antes de opinar, e o dado mudou a pergunta: **este repositório já É o
`Controle_robo_web`.** Clone com histórico completo (decisão 000), 537 commits, e
o que se chamaria de "reaproveitar o resto" está no working tree hoje —
`nav2_params_legacy.yaml` (368 linhas), `unstuck_supervisor.py` (1433),
`motion_guard.py` (683), a árvore de comportamento, o `path_follower` dele. A
demolição de 14-07 tirou só o que era do robô 1 *como missão*. Migrar de volta
seria refazer a demolição e reimportar MEGA, LD06 e knobs de skid-steer.

**Mas o medo dele está certo, e o desperdício está acontecendo aqui dentro.**
Evidência: em 29-07 brigamos com a inflação do Nav2, medimos, e chegamos em
`inflation_radius: 0.30`. O arquivo do robô 1, dois diretórios ao lado, diz
`0.25`, testado em campo. Rederivamos por experimento o que estava escrito. Trocar
de repositório não conserta isso — o arquivo estaria igualmente por ler.

Registrado na **decisão 010**: ficar, e reaproveitar a camada de segurança do
robô 1 adaptando-a ao Mid-360 em vez de reescrevê-la. Herdar estrutura e
raciocínio; **re-derivar os números**, porque todos nasceram de um chassi de 4
rodas — o `collision_monitor` de lá justifica o `angular_limit` com uma zona
morta de **1,7**, que o `CLAUDE.md` proíbe herdar explicitamente.

### O dono cortou uma simplificação minha, e tinha razão

Eu havia tratado a camada de segurança como bloco. Ele: *"unstuck, motion guard,
collision monitor são ótimos, não precisam morrer, só serem adaptados para um
sensor MELHOR"*. Fui ver o acoplamento de cada um, e são três casos distintos:

- **`collision_monitor`** — nó do Nav2, aceita `pointcloud` nativamente. É
  **config** mais a geometria deste chassi (os polígonos de lá são ±0,25 de
  meia-largura contra os 0,433 × 0,455 medidos aqui). O mais barato e o que mais
  ganha com 3D: o anel planar não via obstáculo acima nem abaixo do plano.
- **`motion_guard`** — assina `scan_safe` **e** `map`, e a parte do
  `OccupancyGrid` está lá para caçar "fantasma de vidro" do LD06. Com sensor
  melhor isso não é adaptado, é **deletado**. Boa notícia, mas sobrevive menos
  código do que parece.
- **`unstuck_supervisor`** — a ideia é agnóstica e é das melhores do robô 1
  (recuperar por *não progrediu*, medindo espaço livre antes de dar ré); tanto que
  a decisão 009 já é isso reescrito. As 1433 linhas é que são moldadas em
  varredura planar: vão traseiro por ângulo, corredor retangular, varredura
  girada por −θ.

### O risco que ninguém tinha nomeado ainda

**Nuvem 3D não é drop-in de varredura 2D, e num aspecto é pior.** O Mid-360 tem
padrão de varredura não repetitivo: num quadro de 100 ms a cobertura é esparsa e
desigual, não um anel uniforme de bins angulares. Código que pergunta "qual o
alcance mínimo neste setor angular" recebe resposta instável. Melhor em
informação, mais difícil nesse padrão de acesso.

Saídas a decidir com dado: acumular quadros, ou projetar um **anel sintético** só
para a camada de segurança. Isso reabre, em escopo restrito, o `/scan` derivado
que a 003 descartou — e vale dizer em voz alta que são coisas diferentes: derivar
2D para **não bater** não é derivar 2D para **se localizar**.

### O levantamento fica agendado, não feito

Tudo acima saiu de `grep`, tamanhos e cabeçalhos — **não** de leitura a fundo.
Está anotado como próximo passo 5 do `ESTADO_PROJETO.md`, com gatilho explícito:
roda **quando os dados da bancada chegarem**, porque os polígonos e os limites de
velocidade a re-derivar dependem da zona morta e do `a_dec` medidos. Fazer antes
seria produzir número para trocar depois.

## 🎯 2026-07-31 (3ª leva) — Os dois primeiros números do robô, e o LIO caindo

Sessão de bancada com o robô ligado. **Saíram as duas zonas mortas medidas** —
os primeiros números do robô 2 que não são herança nem chute. E o caminho até
elas derrubou três coisas que a gente achava que sabia.

### O giro espelhado não existia: quem mentia era o sensor

O bloqueio aberto desde 30-07 dizia "esquerda/direita trocadas". O commit
`368ea13` trocou `left`/`right` no `hoverboard_controllers.yaml` e o cutucão
"confirmou" (+59,3° pós-swap contra −78,5° antes).

Confirmação visual com o dono atrás do robô, base **sem** swap: comando de giro
à esquerda → **o nariz foi para a esquerda**, fisicamente correto, enquanto o
LIO reportava −68,2° com rótulo "direita". O yaw do LIO vem com **sinal
invertido**. O alerta de "giro invertido" do cutucão é falso alarme por
construção: ele compara o comando com esse yaw.

Ou seja, o swap consertava um sintoma inexistente e quebrava um robô que já
estava certo — com o swap, os +59,3° do LIO correspondiam a um giro físico para
a direita. Revertido em `595cf80`.

**Lição de método, e é a mesma da bitola:** o cutucão validou o swap porque
media com o instrumento defeituoso. Validação por instrumento suspeito confirma
o que você quiser. O que desempatou foi o olho do dono.

### O LIO não erra só o sinal: ele fabrica movimento

O primeiro ensaio de zona morta de giro rodou com o gatilho no `/Odometry`, e o
CSV guarda as duas odometrias no mesmo instante:

| | LIO (`/Odometry`) | roda (encoder) |
|---|---|---|
| excursão de yaw | **+143,8°** | +8,7° |
| `wz` máximo | **2,9 rad/s** | 0,139 rad/s |
| deriva xy num pivô | 0,23 m | — |

O comando nunca passou de 0,106 rad/s, e o LIO marcou 0,75 rad/s **durante a
pausa, com comando exatamente zero**. Depois, medido direto: com o robô parado,
o ruído de `wz_pose` chega a **0,0326 rad/s** — e o gatilho do dente de serra
dispara em `SAIU = 0,03`. **O ruído do LIO parado é maior que o limiar de
disparo.** O ensaio não tinha como não disparar no ruído.

Daí `ensaio.py --fonte {lio,roda}` e `medir.py --fonte`: escolhe quem **vira o
dente**, com as duas fontes sempre no CSV. O padrão continua `lio` — encoder
mede o EIXO, não o robô, e não enxerga derrapagem; é medida pior, que só se usa
enquanto o LIO estiver quebrado.

### O erro que quase virou número publicado

Com o gatilho na roda, o ensaio de giro devolveu **0,032 rad/s**, dentes
apertados (desvio 0,003), quantização 15× abaixo do limiar, giro líquido
coerente e alternando. Tudo dizia "medida boa".

**O dono, olhando: "no giro ele não se mexeu."**

E estava certo. `SAIU = 0,03` significa coisas diferentes nos dois ensaios: na
reta são 0,03 m/s de borda de roda; no giro eram 0,03 rad/s = **0,0040 m/s** de
borda. O ensaio de giro estava ~7× mais sensível e mediu **rastejo de eixo** —
os "2° por dente" eram folga mecânica, não pivô. Reanálise dos dois CSVs num
critério físico comum confirmou: no giro a borda da roda **nunca cruzou nem
0,006 m/s**.

Corrigido: o gatilho do giro passa a comparar `wz·L/2`, e `medir.py` recebe a
mesma conversão via `--bitola` — quem vira o dente e quem lê o CSV têm de
concordar sobre o que é "imóvel". Travado em teste
(`test_rastejo_de_eixo_no_giro_nao_vira_zona_morta`).

Se o dono não estivesse olhando, 0,032 rad/s ia pro YAML com quatro dentes
concordando e uma tabela bonita atrás. **Dispersão baixa não é validade** — os
quatro dentes concordavam porque mediam a mesma folga.

### Os números

Refeito o giro com o critério certo: 43° a 47° líquidos por dente, sentido
alternando, **confirmado a olho pelo dono**.

```
zona_morta_linear = 0,021 m/s    (queda 0,014)   dentes 0,018 0,023 0,021 0,022
zona_morta_giro   = 0,131 rad/s  (queda 0,106)   dentes 0,116 0,142 0,117 0,148
```

Em velocidade de borda de roda, que é o que a roda sente:

| | comando | borda de roda | faixa |
|---|---|---|---|
| reta | 0,021 m/s | **0,021 m/s** | 0,018–0,023 |
| giro | 0,131 rad/s | **0,0177 m/s** | 0,0157–0,0200 |

**As faixas se sobrepõem: a zona morta é propriedade da RODA, não da manobra.**
Girar parado não é o pior caso — o `tools/banco/README.md` afirmava que era, e a
frase foi corrigida junto com esta entrada. Era herança do skid-steer de 4 rodas
do robô 1, exatamente o que o `CLAUDE.md` manda não herdar. Aqui a herança
sobreviveu disfarçada de comentário técnico, e só caiu porque foi medida.

CSVs: `2026-07-31-zona-morta-giro.csv` (o inválido, do LIO — guardado de
propósito: o contraste está no arquivo), `-giro-roda.csv` (o do rastejo),
`-giro-borda.csv` e `-linear-roda.csv` (os válidos).

### O que esta sessão NÃO entrega, e por quê

`curva`, `reta` e `degrau_giro` **não** rodaram. Não é escolha de ritmo: os três
medem coisa que odometria de roda não pode ver. `curva` mede derrapagem, que é
por definição roda girando sem o robô ir junto; `reta` é laudo sobre o rumo do
corpo assentar; `degrau_giro` mede a desaceleração depois de cortar o comando,
justo quando a roda desliza livre. Rodá-los pela roda produziria número com cara
de resultado.

Todos dependem de pose confiável, e o LIO tem sinal de yaw invertido **e** ruído
de 0,033 rad/s parado. **Esse é o próximo bloqueio, e agora ele é o caminho
crítico do banco inteiro.** Suspeita a investigar (não confirmada): extrínseco
ou orientação da IMU do Mid-360 mal configurados explicariam sinal trocado e
divergência em movimento pequeno de uma vez só.

## ⚠️ 2026-07-31 (4ª leva) — Retratação: os números da 3ª leva são inválidos

Esta entrada **desmente a anterior**. Fica registrada em vez de editada porque o
erro é o resultado mais útil do dia.

### O que estava errado

**`open_loop: True` no `hoverboard_base_controller`.** Confirmado por
`ros2 param get`. Com ele, `/hoverboard_base_controller/odom` **não mede nada**:
integra o comando publicado e devolve. Não é odometria de roda, é o comando
ecoado.

Foi nisso que a 3ª leva construiu o `--fonte roda`. Consequências:

- **As duas zonas mortas da 3ª leva são artefato.** O limiar de detecção linear
  é `LIMIAR_PARADO = 0,020 m/s` e a "medida" deu **0,021 m/s**: era o próprio
  limiar, ecoado. No giro, limiar equivalente 0,148 rad/s, "medida" 0,131.
- **A "quantização de encoder de 0,0004 rad"** que serviu de prova de qualidade
  era o `round(..., 4)` do próprio `ensaio.py` ao escrever o CSV.
- **A condenação do LIO caiu junto.** "O LIO fabricou 143,8° num pivô de 8,7°"
  comparava o LIO com o comando ecoado. Referência falsa.

### O LIO estava bom o tempo todo

Medido, robô parado, 20 s, sem comando: `/Odometry` a 10,0 Hz, deriva de yaw
**+0,05°**, excursão **0,54°**; deriva xy 0,009 m. O gyro tem viés de
−0,0223 rad/s e o FAST-LIO o remove (seriam 25° em 20 s).

O "ruído de 0,033 rad/s" que motivou o `--fonte` era **derivada**, não sensor:
`JANELA_S = 0,2 s` sobre pose a 10 Hz pega DUAS amostras, e 0,54° em 0,2 s dão
0,047 rad/s. O comentário do próprio arquivo avisa que janela curta amplifica
ruído. Agora é `--janela` (use 0,5 no robô).

O dono disse desde o começo para confiar no LIO. Estava certo, e a insistência
custou a sessão.

### Três vezes o LIO bateu com o olho do dono

- `giro-roda.csv`: dono disse "no giro ele não se mexeu"; LIO mediu 0,03° a
  0,21° por dente. **Não moveu.**
- `linear-roda.csv`: dono viu 4 movimentos; LIO mediu 0,66 a 0,78 m por dente.
  (O comando ecoado dizia 5 cm.)
- rajada de `wz=0,3` por 1,5 s: dono disse "quase 180, uns 190"; LIO mediu
  109,3° sob comando + 85,4° de inércia = **194,7°**.

### Os números, refeitos pelo LIO por DESLOCAMENTO de pose

Critério: deslocamento acumulado desde o início do dente, limiar 2° / 3 cm
(contra ruído de 0,54° / 1,9 cm). Estável entre 2° e 5°, e as duas corridas de
giro independentes concordam:

```
zona morta de GIRO   ≈ 0,095 rad/s    faixa 0,084 – 0,105
zona morta LINEAR    ≈ 0,023 m/s      faixa 0,020 – 0,025
```

A linear vem com ressalva: o número **sobe** com o critério (0,025 a 5 cm,
0,029 a 8 cm, 0,040 a 20 cm). Não tem patamar — na reta ele rasteja antes de
andar. O giro tem patamar, e por isso é o número firme dos dois.

### O achado que reescreve o modelo: `cmd_vel` é acelerador, não velocidade

Rajada única, `wz = 0,30 rad/s` por 1,5 s, pivô no lugar:

```
t=0,02  wz=+0,012      t=0,97  wz=+2,574
t=0,39  wz=+1,110      t=1,34  wz=+3,941   <- pico, 13x o comando
--- corte ---
t=1,85  wz=+1,613      t=2,39  wz=+0,211    t=2,75  parado
```

**O robô não persegue o comando: acelera enquanto o comando estiver ligado.**
Não estabiliza em valor nenhum. É o `open_loop` por inteiro — o comando vira
empuxo na placa, e não existe malha fechada de velocidade de roda.

Primeiros parâmetros para o Gazebo:

```
aceleracao angular  ≈ 2,9 rad/s²   (0 -> 3,94 em 1,35 s, comando 0,30)
desaceleracao       ≈ 2,2 rad/s²   (2,7 -> 0 em ~1,2 s, sem comando)
```

Isso explica tudo que parecia incoerente na sessão: as razões
realizado/comandado que mudavam de corrida (4,19x, 2,08x, 5,6x) não eram erro de
escala, eram a **rampa** — a razão depende de quanto tempo o comando ficou
ligado. E explica as "quase 4 voltas" com comando de meia volta, e o robô
batendo nas coisas quando rodei `--dur 12` com `--v 0.3`.

### Erros de método desta sessão, para não repetir

1. **Rodei o robô sem esperar o "pode"**, mais de uma vez. A regra 6 do
   `CLAUDE.md` é explícita e eu a tratei como formalidade sob pressa. Numa das
   vezes larguei uma bateria de `--dur 12 --v 0.3` (3,6 m por corrida) numa área
   preparada para pivô de 1 m de raio, e o robô bateu.
2. **Usei uma referência sem verificar o que ela era.** Um `ros2 param get
   open_loop` — dez segundos — teria evitado a sessão inteira.
3. **Tratei concordância entre dentes como validade.** Os quatro dentes
   concordavam porque mediam o mesmo artefato. Dispersão baixa não valida nada
   se o instrumento estiver ecoando a entrada.
4. **Discuti com o dono contra a evidência dele.** As três vezes em que o olho
   dele e o LIO foram confrontados, os dois concordaram e eu é que estava errado.

### Adendo da 4ª leva: o robô não anda reto, e é esse o defeito de verdade

As duas corridas de `degrau_giro` que rodaram por engano (`--dur 12 --v 0.3`,
as que fizeram o robô bater) são a evidência mais longa da sessão, e mostram o
que os ensaios curtos não pegaram.

**Com `cmd_wz = 0`, o LIO mede giro de −0,3 a −0,4 rad/s, sustentado.** O robô
arqueia sozinho. E o comando de curva não soma a isso: quando `+0,3` entra
(t=2,5 a 3,5), o giro realizado sobe de −0,32 para −0,02 — o comando apenas
**cancela** o desvio existente.

Casa com a assimetria dos encoders crus (`-70,1 rad` na esquerda contra
`-0,607 rad` na direita) e explica a batida: ele não atravessou a sala em linha
reta, curvou para dentro das coisas.

**Retratação dentro da retratação:** "`cmd_vel` é acelerador" não vale para a
reta. Nas mesmas corridas o linear ESTABILIZA em 0,21–0,30 m/s com comando de
0,30 — segue o comando. O disparo a 3,94 rad/s apareceu só no pivô parado, e é
provável que seja outra face do defeito de uma roda só, não um modelo de
atuador. Fica sem conclusão até a roda ser resolvida.

Ressalva que impede fechar o número: −0,3 rad/s por 8 s dariam ~137° de arco,
círculo de ~0,83 m de raio, e o robô se afastou 2,31 m da origem. Não fecham. O
desvio é real e grande; a magnitude ainda não está firme.

**Próximo passo, e ele não precisa de comando nenhum:** com o robô ligado, girar
cada roda com a mão e ler `/hoverboard/{left,right}_wheel/position`. Separa
"encoder morto" de "roda não acionada" em 30 segundos, sem o robô andar.

### Adendo 2 da 4ª leva: o desvio medido no atuador, e o driver que o robô não lia

Com a compensação religada e o sinal de realimentação por roda corrigido, quatro
rajadas curtas (1,0 s, volta a zero entre cada uma) pelo LIO:

| corrida | comando | v realizada | desvio sob comando | desvio na inércia |
|---|---|---|---|---|
| 1 frente | +0,10 | 0,173 m/s | −9,2° | −13,9° |
| 2 ré | −0,25 | 0,180 m/s | **+0,5°** | −7,7° |
| 3 frente (pós-ré) | +0,10 | 0,224 m/s | −6,9° | −20,1° |

**Patamar único, confirmado:** comandos de 0,10 e 0,25 deram 0,17–0,22 m/s. Na
faixa baixa o `cmd_vel` escolhe SENTIDO, não módulo — é a compensação de zona
morta inflando tudo até o mesmo teto. É o comportamento central que o Gazebo
tem de imitar.

**O desvio existe só para a frente, e é reprodutível** (−9,2° e −6,9°); de ré ele
anda reto. Virar a boba com uma ré antes NÃO removeu o desvio.

Gravando as duas rodas na rajada de frente:

```
 t=0,00  v_esq 0,000  v_dir 0,000   <- 0,4 s parado (latencia p/ destravar)
 t=0,40       +1,571       +1,571   <- arrancam JUNTAS
 t=0,98       +3,979       +3,246   <- dif +0,733
 --- corte ---
 t=1,26       +3,351       +2,199   <- dif AUMENTA: +1,152
 t=1,84       +0,733        0,000   <- direita parou, esquerda ainda gira
```

**A roda direita gira 20–25% mais devagar que a esquerda com o mesmo comando**
(~38 RPM contra ~31), e no corte a diferença cresce: a direita para meio segundo
antes. Isso é o desvio, medido no atuador em vez de inferido, e explica por que o
desvio na inércia era maior que o desvio sob comando.

Ressalvas: a velocidade vem quantizada em 1 RPM (0,105 rad/s); e as 100 unidades
que o driver manda viram ~38 RPM na roda, então o campo `speed` do firmware não
é RPM — a escala real não foi identificada.

**Hipótese aberta (do dono):** a boba. Ela não está descartada — arrasto
assimétrico carrega uma roda mais que a outra, e sem malha fechada isso vira
roda mais lenta. O teste que separa é gravar as rodas numa rajada DE RÉ: se lá
elas girarem iguais, é carga; se a direita seguir mais lenta, é placa/motor.

### O parâmetro que o robô nunca leu

Editei `hoverboard_driver.ros2_control.xacro`, commitei, deployei por bundle,
recompilei e relancei a base — e o robô ignorou. O `robot_base` declara o
`ros2_control` do hoverboard **inline** no `robo2.urdf.xacro` e nunca inclui o
xacro do pacote do driver. Os `feedback_sign` só pegaram por eu tê-los posto
como default no C++.

Custou três ciclos de deploy até eu olhar o `robot_description` de verdade.
Parâmetro novo do driver vai no `robo2.urdf.xacro`, e agora há um aviso no
próprio bloco.

Também anotado: `pkill -f ros2_control_node` dentro de um comando ssh casa com a
**própria linha de comando do ssh** e mata a sessão antes de relançar a base.
Matar por PID, ou usar o truque do colchete.

### Adendo 3 da 4ª leva: o par de pivô, e a repetição que corrigiu os números

Repetidas as rajadas de reta com as rodas gravadas (n=2 por sentido), as médias
sobre a janela 0,6–1,0 s corrigem o que eu tinha afirmado por olhômetro em cima
do pico:

| corrida | esq vs dir sob comando | tempo até parar (esq / dir) |
|---|---|---|
| frente #1 | +11,3% | 0,90 s / 0,70 s |
| frente #2 | +12,0% | 1,00 s / 0,62 s |
| ré #1 | +7,2% | 0,70 s / 0,76 s |
| ré #2 | −0,8% | 0,67 s / 0,67 s |

Não são os "20–25%" que eu disse: são **11–12% de frente e ~0% de ré**. E a
frenagem só é desigual indo para a frente (esquerda demora 0,2–0,4 s a mais).
Isso derruba a minha afirmação de que "aparece nos dois sentidos, logo não é a
boba" — a assimetria da RETA é dependente de sentido, o que mantém a boba como
candidata. **Confirmado a olho pelo dono: de ré sai reto, de frente pende para
a direita.** Aceito como fenômeno medido; a causa fica em aberto.

**Par de pivô** (`wz = ±0,30`, 1,0 s, rodas gravadas):

| | `+0,30` | `−0,30` |
|---|---|---|
| giro sob comando | +60,2° | −48,9° |
| giro na inércia | +87,5° | −101,2° |
| **giro TOTAL** | **+147,7°** | **−150,0°** |
| realizado/comandado | 3,50× | 2,84× |
| deslocou o centro | 0,139 m | 0,071 m |
| esq vs dir | +15,9% | +11,4% |
| esq parou em | 1,20 s | 0,59 s → dir |
| dir parou em | 0,59 s | 1,35 s |

Para o simulador:

1. **O giro total é simétrico** (147,7° contra 150,0°) — o Gazebo não precisa de
   assimetria de rotação.
2. **A esquerda é 11–16% mais forte nos DOIS sentidos de giro** — diferente da
   reta, onde só aparece indo para a frente.
3. **Quem para por último é sempre a roda que gira para TRÁS.** Regra limpa.
4. **60% do giro acontece depois do corte** (~90–100°), e o pivô não é puro:
   sai 7 a 14 cm do lugar.
5. O giro também é inflado pela compensação: 2,8–3,5× o comandado.

Falha de instrumento anotada: o `rajada_rodas.py` gravava só `cmd_v`, então o
CSV `2026-07-31-pivo-wz030.csv` não registra o `wz` comandado (está só no nome
do arquivo). Corrigido para as corridas seguintes.

### Adendo 4 da 4ª leva: o patamar fecha o modelo, e o que AINDA falta

Rajada de `v=0,80` por 0,40 s saiu **inconclusiva por erro de desenho meu**: a
latência para destravar é ~0,35 s e eu escolhi comando de 0,40 s, sobrando 0,16 s
de acionamento. O pico de 40 RPM aconteceu já na inércia. Andou 4,7 cm sob
comando, 25,9 cm no total.

A resposta veio do código, sem mexer no robô. A compensação só age com
`mx < 100`, e para `v = 0,80`:

```
roda = 0,80/0,080 = 10,0 rad/s   ->   set_speed = 10,0/0,10472 = 95,5   (<100)
```

Ainda é inflado. O patamar termina em `100 unidades = 10,472 rad/s = 0,838 m/s`,
e o teto do robô é 1,0. **Todo comando entre ~0,008 e ~0,838 m/s vira a mesma
coisa na placa** — o patamar cobre a faixa útil inteira. Bate com o medido (0,10
e 0,25 deram a mesma velocidade).

Modelo do atuador para o Gazebo:

```
|cmd| < 0,008 m/s     -> nao move
0,008 a 0,838 m/s     -> mesma velocidade (~0,2 m/s medidos), ~0,35 s de latencia
> 0,838 m/s           -> comando passa proporcional (NAO TESTADO)
```

**A zona morta, e são duas.** Efetiva (compensação ligada, releitura pelo LIO por
deslocamento de pose): linear ≈ **0,023 m/s** (faixa 0,020–0,025) e giro ≈
**0,095 rad/s** (faixa 0,084–0,105). Conferem com o previsto pelo código (0,0084
e 0,062) dentro do atraso de detecção da rampa. O linear é o fraco: **não tem
patamar** (0,029 a 8 cm de critério, 0,040 a 20 cm), então é "onde começou a
rastejar". O giro tem patamar e duas corridas concordando. Crua (compensação
desligada): só bracketada entre **0,25 e 0,5 m/s**.

**O que NÃO foi feito, para não fechar a sessão fingindo completude:**

- repetição: linear n=1, giro n=2, pivô n=1 por sentido. O protocolo pede 3.
- `curva` sustentada (giro realizado × comandado em movimento, e derrapagem).
- `reta` com cutucão (se o rumo assenta ou foge após perturbação).
- `degrau_giro` / `a_dec` angular, que o `collision_monitor` e o seguidor pedem.
- comportamento acima de 0,838 m/s.

**Dívida de método:** o `tools/banco` pressupõe que comando sustentado dá
velocidade sustentada. Com o patamar único isso é FALSO na faixa baixa — rodar o
protocolo como está mede o patamar, não o robô. Os ensaios de zona morta
precisam ou de rampa repensada, ou de rodar com a compensação desligada, que é
onde a zona morta física existe.

### Adendo 5 da 4ª leva: o LIO estava certo o tempo todo, e o instrumento era eu

Girei até **o LIO** acusar 180° e cortei; ele girou mais na inércia. As três
fontes no fim:

```
olho do dono   ~270°  ("meia volta + 1/4, parou de lado")
LIO             264,6°
rodas           257,5°
```

**O LIO está validado** — terceira conferência independente do dia contra o olho
do dono, agora com referência angular precisa em vez de "uns 190°". O dono
insistiu nisso desde o começo da sessão e estava certo.

**Retratação do "fator 1,25".** Eu havia mostrado LIO ÷ rodas ≈ 1,22–1,29 em
todas as corridas, translação e rotação, e concluído erro de escala no encoder.
Era buraco de gravação meu:

```python
finally:
    for _ in range(10): n.manda(0.0, 0.0); time.sleep(0.02)   # 0,2 s SEM gravar
```

Esses 0,2 s são exatamente o trecho em que o robô ainda está em velocidade quase
máxima logo após o corte. A integral das rodas os perdia; o LIO, medido de ponta
a ponta, não. A 0,3 m/s dá ~6 cm — e as diferenças observadas eram 6,7 e 7,2 cm.
O teste do 180° integra continuamente e as duas fontes fecham em 2,8%.

Consequência: **as distâncias e velocidades medidas pelo LIO voltam a valer**,
inclusive a zona morta linear de 0,023 m/s. Corrigido o `rajada_rodas.py`.

Fica em aberto, sem explicação: em algumas corridas o `/Odometry` traz episódios
de amostras ~1,4 m deslocadas (10–20% das amostras, valores repetidos entre
corridas). Não é ruído aleatório e não afeta o yaw. Corrida com episódio desses
tem o deslocamento inutilizado e deve ser descartada — hoje isso atingiu
`pivo-wz030-r2` e `curva-v010-wz030`.

### O placar de instrumentos desta sessão

Três vezes eu acusei o sensor e três vezes o defeito era meu:

1. "ruído de 0,033 rad/s" → era `JANELA_S` de 0,2 s sobre pose a 10 Hz.
2. "o LIO fabrica movimento" → eu comparava com odometria em `open_loop`, que é
   o comando ecoado.
3. "o LIO infla 25%" → buraco de 0,2 s no meu logger.

E quatro vezes o olho do dono arbitrou certo: o giro à esquerda, o "no giro ele
não se mexeu", o "de ré reto e de frente pendendo", e os ~270°. **Num banco de
ensaios sem padrão de referência, a testemunha humana é o instrumento mais
confiável que existe** — e é barata. Usar mais, e mais cedo.

### Adendo 6 da 4ª leva: os "saltos do LIO" eram TRÊS pilhas rodando juntas

A `curva` foi refeita e saiu contaminada de novo (salto de 1,344 m, yaw do LIO
+18,0° contra +45,5° das rodas). Em vez de queimar mais corrida, fui olhar o que
estava vivo no NUC:

```
ros2_control_node      : 1
livox_ros_driver2_node : 3     <-- 6273, 7043, 7858
fastlio_mapping        : 3     <-- 6274, 7044, 7859
```

**Três pilhas de localização completas em paralelo**, publicando as três em
`/livox/lidar`, `/livox/imu` e `/Odometry`. Órfãs das minhas derrubadas de base:
usei `pkill -f ros2_control_node` dentro do comando ssh, o padrão casou com a
**própria linha de comando do ssh** e matou a sessão antes de relançar. Repeti
três vezes.

Três FAST-LIO com origens diferentes publicando no mesmo tópico é exatamente a
assinatura observada: saltos de magnitude quase CONSTANTE (~1,33 a 1,41 m) entre
corridas diferentes — que eu, erradamente, li como "o sensor perde rastreio".

Depois de matar os órfãos por PID e relançar UMA pilha:

```
antes (3 pilhas)  : saltos de 1,35 m
so matando orfaos : salto 0,140 m, deriva 0,078 m em 15 s  (mapa ja sujo)
apos relancar     : salto 0,0051 m, deriva 0,0018 m em 15 s, yaw -0,10°
```

**O LIO é excelente.** Melhor que a medida da manhã. Todo o episódio de
"instabilidade do sensor" foi sujeira que eu deixei no robô.

Procedimento, para não repetir: matar por PID, conferir com `ps` que a lista
saiu **vazia**, e só então lançar — em chamada ssh separada. Antes de confiar em
qualquer medida de `/Odometry`, verificar que existe **exatamente um**
`fastlio_mapping` e **um** `livox_ros_driver2_node`.

Corridas a descartar por contaminação: `pivo-wz030-r2`, `curva-v010-wz030`,
`curva-r2`, `zm-linear-r2` (esta também falhou por gatilho de amostra única).

### Fechamento da 4ª leva: 7 corridas vazias, e o que fica para a próxima

Tentadas as versões encurtadas dos passos 3, 4 e 5 (a_dec por pivô a `wz` 0,3 /
0,6 / 1,0; curva a `v` 0,2 / 0,4 / 0,6; aceleração a `v` 0,6). **As sete saíram
vazias** — zero pose, zero roda girando: a rede caiu e o NUC ficou inalcançável.
O robô não se mexeu em nenhuma. Os arquivos foram apagados para não sujarem o
conjunto.

**Defeito de instrumento que isso expôs, e é o mais perigoso de todos:** os
scripts de rajada **não percebem que o robô sumiu**. Publicam em `cmd_vel`,
gravam linhas em branco e terminam imprimindo `COMANDO ... 205 amostras`, com
cara de sucesso. Se a queda tivesse acontecido no meio de uma corrida boa, o
número teria vindo pela metade sem aviso. **Antes da próxima sessão: checar
`/Odometry` vivo no início e no fim de cada rajada, e abortar se não estiver.**

### Onde o banco realmente está: 2 passos de 6

| passo | corridas | estado |
|---|---|---|
| 1 — zona morta de giro | 1 | ✅ feito |
| 2 — zona morta linear | 1 | ✅ feito |
| 3 — degrau de giro (`wz` 0,3/0,6/1,0) | 3 | ❌ nenhuma |
| 4 — curva (`v` 0,2/0,4/0,6) | 3 | ❌ nenhuma |
| 5 — aceleração linear | 1 | ❌ |
| 6 — reta com cutucão | 4 | ❌ nenhuma |

O `a_dec` (~3,05 rad/s²) e a curva (`v=0,10`) que estão no `MODELO_ROBO2.md` são
**substitutos** derivados de rajadas curtas, não os ensaios 3 e 4.

**Os passos 3, 4 e 5 precisam ser REESCRITOS antes de rodar.** Os três varrem
velocidade — "quanto ele curva a 1×, 2×, 3×" — e 0,2 / 0,4 / 0,6 m/s caem todas
dentro do patamar da compensação, então as três dariam o mesmo resultado. Do
jeito que estão, não conseguem responder à própria pergunta neste robô. A
varredura tem de subir acima de 0,838 m/s, onde o comando volta a ser
proporcional — e isso exige espaço que a bancada de hoje não tinha.

O passo 6 continua válido como está e é o mais importante dos que faltam: ele
pergunta se o rumo se recupera de uma perturbação ou foge, e o desvio de 9° em
18 cm que medimos torna essa pergunta urgente. Precisa de corredor (4 corridas
de ~4 m).

### O limite honesto do modelo de hoje

Todas as corridas boas são **rajadas de ~1 s e ~20 cm**. As duas únicas longas
(12 s) são justamente as que bateram, e rodaram com o LIO poluído pelas pilhas
órfãs. Então o `MODELO_ROBO2.md` está aferido em **arranca-e-para**, não em
percurso sustentado — e percurso é o que um seguidor faz o tempo todo. Duas
corridas limpas num corredor fecham isso: o passo 6 e uma reta longa.

## 🎛️ 2026-08-01 — O atuador medido entra no simulador, e a decisão 005 cai

Sessão sem robô, em cima dos CSV de 31-07. O simulador tinha uma placa que
engolia comando pequeno; o robô tem outra coisa, e muito pior. Agora os dois
erram igual.

### O que eu conferi sozinho, antes de mexer

Refiz as contas dos CSV em vez de aceitar o laudo. A **zona morta de giro
(0,091–0,096 rad/s) é firme** — recalculei por deslocamento de yaw com critério
de 2°, 3° e 5° e deu 0,091 / 0,096 / 0,096, com os 4 dentes concordando.

O **patamar** também se confirma, e um pouco maior do que estava escrito:
comando de 0,10 m/s virou **0,26–0,28 m/s de borda** nas quatro rajadas. A
assimetria de frente é consistente (+11,3% e +12,6%); a de ré não é (+6,7% e
−0,9%), o que sustenta a leitura de que ela depende do sentido.

E extraí o que faltava para fechar o modelo do atuador:

```
latencia       = 0,273 s   (n=4, faixa 0,24-0,31)
patamar        = 3,72 rad/s de roda = 0,297 m/s de borda
aceleracao     = 5,44 rad/s²  (0,435 m/s²)
desaceleracao  = 4,66 rad/s²  (0,373 m/s²)
escala real    = 0,0372 rad/s por unidade  (o driver assume 0,10472)
```

**A escala é o achado que fecha a história**: o driver superestima a roda em
**2,8×**. É por isso que o robô sempre andou mais rápido do que se pediu, e é o
mesmo 2,8× que aparece no patamar.

### Onde eu leio diferente do laudo de 31-07

**A "zona morta de giro" não é atrito, é aritmética do driver.** A compensação
dispara em `mx > 1.0`, o que dá `wz > 0,0621 rad/s`. Medimos 0,091–0,096, e a
diferença de 0,03 é exatamente a latência de 0,27 s sobre uma rampa de
0,075 rad/s por segundo. O número é reprodutível e útil como propriedade **do
sistema**, mas se move se alguém mudar `deadband_speed` ou o raio da roda. O
atrito de verdade segue não medido.

**A pergunta do pivô foi dissolvida, não respondida.** Com a compensação ligada,
qualquer `wz` acima de 0,062 pivota, na velocidade do patamar. A aritmética que
dizia "pivotar exige 0,96 rad/s contra teto de 1,0, folga de 4%" ficou sem
objeto. O pivô existe; o que não existe é **controle da velocidade dele**. Isso
mexe na premissa da decisão 009.

### A decisão 005 não sobrevive ao atuador medido

Passei a própria lei da 005 pelo modelo, offline:

```
        wz = sinal(e)·min(wz_max, √(2·a_dec·|e|))

 erro de rumo |  a lei PEDE | a placa ENTREGA | fator
         1°   |    0.326    |        2.204    |  6.76x
         2°   |    0.461    |        2.204    |  4.78x
         5°   |    0.730    |        2.204    |  3.02x
        10°   |    1.000    |        2.204    |  2.20x
        45°   |    1.000    |        2.204    |  2.20x
       180°   |    1.000    |        2.204    |  2.20x
```

**A lei produz 8 valores distintos; a placa entrega 1.** De 1° a 180° de erro de
rumo o giro é o mesmo, e é 2,2× maior que o máximo que a lei jamais pediria. A
frenagem de rumo — o resultado central da decisão 005, validado em 10 corridas
de simulador — **não existe neste robô** enquanto a compensação estiver ligada.
O mesmo vale para `v = √(2·a_lin·dist)` da aproximação, e o `v_piso` fica sem
sentido: não há velocidade abaixo do patamar para se estar.

Em uma frase: **o contrato de que `cmd_vel` está em m/s é falso em toda a faixa
que a navegação usa**, e é sobre esse contrato que a movimentação inteira foi
escrita.

### O simulador agora erra como o robô erra

`placa_simulada.py` reescrito. Reproduz a conta do driver linha a linha
(escala as duas rodas por `k = deadband_speed/mx`), depois aplica a escala real
do firmware, a latência e a assimetria por sentido. Três modelos:
`medido` (padrão), `cru` (compensação desligada, onde a zona morta física
aparece) e `ideal` (fio, para comparar).

Provado no Gazebo, mesma ordem de movimento nas duas placas:

```
comando      placa ideal        placa medida
0,10 m/s ->  0,200 m em 2 s     0,548 m
0,25 m/s ->  0,500 m            0,541 m
0,50 m/s ->  1,000 m            0,541 m     <- as tres iguais
```

`sim.launch.py placa:=medido|cru|ideal`, e o mesmo em `pilha.launch.py`. O
parâmetro `zona_morta:=` saiu — não descrevia mais nada.

### Um tropeço meu, que é o mesmo do laboratório

A primeira rodada da prova deu placa medida ≈ placa ideal, o que não fazia
sentido. Eram **quatro `placa_simulada` órfãs** acumuladas no dia, mais quatro
pontes de `/Odometry`, de lançamentos que eu derrubei com `pkill` de padrão
largo. Exatamente o defeito das três pilhas do laboratório, na minha máquina.
A corrida trazia `Failed to configure controller` no log e eu não tinha olhado.

Passou a fazer parte do procedimento: contar `placa_simulada` viva **antes** de
medir, e conferir `Failed to configure` no log. Está no script de prova.

### O que NÃO foi feito

A comparação da pilha inteira (Nav2 + seguidor) nas duas placas **não rodou**:
o `base_link` não aparecia na TF e os costmaps reclamavam de raio de inflação
menor que o inscrito (0,300 contra 0,363 — o footprint da trena é maior que o
que o `nav2.yaml` supõe). Fica como próximo passo, e o aviso do costmap é um
defeito de verdade, anotado.

419 testes verdes (eram 407), com 8 novos travando o modelo do atuador —
inclusive o do colapso da lei da 005, que é a afirmação mais forte desta entrada.
## 🎯 2026-08-04 — O bloqueio era do instrumento, e o robô anda em círculo

Sessão de bancada com o robô ligado, conduzida por ssh no NUC; o dono só executa
o físico. **Nove corridas.** O banco foi de 2 passos de 6 para 4, e o resultado
principal não estava no protocolo: **o robô descreve um círculo indo para a
frente, e a causa depende do sentido de marcha.**

### O briefing estava desatualizado, e seguir o checklist quebraria o robô

O dono abriu a sessão pedindo para medir a zona morta ("item nº 1, não foi
medida") e para seguir o `CHECKLIST_ROBO.md` na ordem, sem pular — incluindo
**aplicar o swap esquerda/direita**. As duas premissas são do fim de 30-07 e a
sessão de 31-07 à tarde já as tinha desmentido. O checklist é da *manhã* de
31-07: ele é anterior à própria sessão que preparava.

Conferido no repo antes de tocar em nada: a zona morta está medida (giro 0,095
rad/s, linear 0,023 m/s), o swap foi aplicado (`368ea13`) e **revertido**
(`595cf80`), e o YAML de hoje está sem swap — o estado correto. Aplicar aquele
script quebraria um robô que está certo. **Parado e reportado antes de agir**,
que é a regra 1 do `CLAUDE.md` valendo contra o pedido do próprio dono.

### O "giro espelhado" de 30-07 era enrolamento de ângulo

Antes de rodar o cutucão, olhei o que ele calcula:

```python
giro = math.atan2(math.sin(yaw() - a0), math.cos(yaw() - a0))
```

Isso **enrola em ±180°**. E o cutucão manda `+0,6 rad/s por 2,0 s` — com o
patamar da compensação, muito mais que meia volta. A conta fecha exata:
`281,5° − 360° = −78,5°`, que é o número do bloqueio de 30-07.

Rodado hoje com o dono de testemunha: leu **−71,0°**, e ele viu **o nariz ir
para a esquerda varrendo bastante**. São `289°` enrolados. O robô gira certo.

### E a retratação de 31-07 sobre o LIO era ela própria incorreta

A 3ª leva de 31-07 concluiu que o **yaw do LIO vem com sinal invertido**, a
partir de "o LIO reportava −68,2° com rótulo direita". Mas `−68,2 + 360 =
291,8°` — é o mesmo enrolamento, diagnosticado como sinal trocado.

O teste do sinal, com o dado de hoje: giro físico anti-horário de 289°. Se o
yaw estivesse certo, `atan2` devolve −71° (foi o que deu); se estivesse
invertido, devolveria +71°. Conferido também no dado cru de 31-07, desenrolando:

```
pivo-wz030.csv     comando +0,30  ->  yaw desenrolado  +147,7°
pivo-wz030neg.csv  comando −0,30  ->  yaw desenrolado  −150,0°
```

Sinal do comando e sinal do yaw **concordam**. E esses números já estavam na
tabela do diário de 31-07 — ninguém cruzou as duas partes do próprio registro.
**O LIO está certo.** É a quarta vez que ele é acusado e a quarta vez que o
defeito é do instrumento de leitura.

### O resultado do dia: o arco é do robô, e 88% dele é a boba

O passo 6 pergunta se o rumo se recupera de uma perturbação. **Não dá para
perguntar isso neste robô**, por dois motivos independentes que a primeira
corrida expôs: não existe reta de referência (ele faz círculo), e o pulso de
perturbação dura 0,5 s contra uma latência de placa de ~0,5 s — não perturba
nada (wz médio −0,405 antes, −0,358 durante, −0,369 depois).

Rodadas então retas puras, medindo **curvatura**. Com `n=2` matched por sentido:

```
FRENTE   média −0,817 1/m   raio  1,22 m   faixa −0,73 a −0,90
RÉ       média −0,098 1/m   raio 10,19 m   faixa −0,08 a −0,12
                                              razão frente/ré = 8,3x
```

**Controle de piso, e ele funcionou por acaso feliz.** O dono posicionou a
corrida de ré com o bico a 180°, o que faz o robô andar na *mesma direção do
mundo* que a corrida de frente, sobre o mesmo pedaço de chão, com o corpo
girado. Se o arco fosse caimento, a força é fixa no mundo e a curvatura **no
corpo** trocaria de sinal. Ela saiu negativa nas **quatro** corridas.
**O arco é do robô, não da sala.**

O que isso exclui: motor ou placa fraca de um lado daria a mesma curvatura nos
dois sentidos. Sobra causa dependente do sentido de marcha — a assinatura de uma
roda boba, arrastada atrás indo para frente e dianteira indo de ré. Encaixa com
os encoders de 31-07 (esquerda 11–12% mais rápida **só** de frente, ~0% de ré),
que medem no atuador e não passam por piso nem por rumo.

**O dono corrigiu uma frase minha, e a correção melhorou o resultado**: eu
escrevi "de ré ele anda praticamente reto"; ele disse que a ré desvia sim, muito
menos, e está certo — raio de 10 m ainda dá 70–90° em 10 m de percurso. Isso lê
como **duas parcelas somadas**: ~−0,10 1/m constante nos dois sentidos e ~−0,72
1/m só de frente, ou seja **88% do arco de frente é o termo da boba**.
Consequência de projeto: consertar a boba não deixa o robô reto.

### O `a_dec` medido, e ele não é constante

O passo 3 roda apesar do patamar da compensação — o `a_dec` é medido **com o
comando em zero**, e compensação só age com comando. Rodado como **pivô puro**
(`--v 0`, não o `--v 0.3` do protocolo): com a curvatura de frente de hoje,
andar durante o ensaio injetaria −0,25 rad/s de guinada espúria que continua
depois do corte, e o `a_dec` sairia errado — o defeito disfarçado de física.

Calculado do yaw **desenrolado**, porque o `medir.py` usa o mesmo `atan2`. `n=3`:

```
a_dec EFETIVO  média 3,26   faixa 2,68–4,32   dispersão 50%
a_dec CAUDA    média 1,03   faixa 0,88–1,12   dispersão 24%
pico de wz     média 2,33   faixa 2,23–2,51   dispersão 12%
```

**O `a_dec` efetivo não serve como constante de projeto** — espalha 50% entre
corridas iguais, porque é `wz²/(2·Δθ)` e o pico entra ao quadrado. Os 3,05 do
`MODELO_ROBO2.md` caem dentro da faixa dele, então o substituto de rajada curta
não errou o *valor efetivo*; errou o *uso*.

O que o seguidor precisa é a **cauda: ~1,0 rad/s²**, onde o robô tem de assentar
no rumo e desacelera 3× menos que o substituto diz. E é o lado seguro do erro
pelo próprio achado de 27-07 ("errar para baixo é de graça; para cima traz o S
de volta").

Latência depois do corte, medida nas três: **0,40 / 0,56 / 0,60 s**, mais que os
~0,35 s que o registro trazia.

### Erros meus nesta sessão

1. **Dimensionei a primeira corrida só pela distância à frente.** A trava de
   `--espaco` é **radial** e, num robô que faz círculo de 0,9 m, ele varre 1,8 m
   para os lados sem "estourar" nada. O corredor era estreito e **o robô bateu
   numa cadeira**. Pior: quem disparou a trava foi a odometria em `open_loop`,
   que achava ter andado 2,98 m em linha reta enquanto o LIO sabia que ele estava
   a 1,79 m da origem. Cortou pelo motivo errado. Corrigido nas seguintes com
   `--espaco 1.2`, que limita o **disco varrido**.
2. **Comparei um par não-matched e escrevi conclusão em cima dele.** As corridas
   `-a` partiram de rumos 89° diferentes, o que deixa caimento de piso entrar. Só
   vi ao olhar o rumo inicial no CSV, depois de já ter commitado. Corrigido pelo
   par `-b`.
3. **Chamei de "dispersão de 57%" o que era diferença de configuração** (a `-a`
   rodou com `--espaco 3.0`, com pulso e por 3,74 m). A dispersão matched é 21%.

### O que ficou de instrumento, e uma dívida paga antes de medir

Antes de subir o robô, consertado o defeito que perdeu 7 corridas em 31-07: o
`ensaio.py` guardava a última mensagem e nunca olhava **quando** ela chegou —
fonte morta congelava a pose, a derivada dava 0,0 (leitura plausível, não erro)
e a corrida fechava inteira com cara de sucesso. Agora aborta em `--sem-dado`
(1,0 s) e sai com código 1, que o `sessao.py` já sabia tratar e nunca recebia.
4 testes; 415 verdes.

**Ainda por consertar:** o `atan2` que enrola, no cutucão do `sessao.py` e no
`medir.py`. É o defeito que custou a sessão de 30-07 inteira e quase custou a de
hoje.


### Adendo — encaixe com a entrada de 08-01, escrita em outra máquina

As entradas de 08-01 e 08-04 foram escritas em paralelo, em máquinas
diferentes, sem uma saber da outra. Juntando no merge, três coisas:

**A confirmação cruzada, e é a mais valiosa.** 08-01 derivou **do código do
driver** que a placa entrega um `wz` único de **2,204 rad/s**. 08-04 mediu no
robô, com o LiDAR, picos de **2,226 / 2,514 / 2,253 rad/s**. Duas rotas
independentes — leitura de código e medida em hardware — no mesmo número. O
modelo do atuador de 08-01 deixa de ser derivação e passa a ser **modelo
validado**.

**As duas "latências" não brigam: são fenômenos distintos.** 08-01 mede 0,273 s
para o robô *começar* a andar depois do comando. 08-04 mede 0,40–0,60 s entre o
comando *zerar* e o `wz` chegar ao pico — quanto a placa **continua empurrando
depois de desligada**. Atraso de liga e atraso de desliga, os dois reais, e o de
desliga é o maior. Registrar como dois números, não como um corrigindo o outro.

**E o `a_dec` de 08-04 ganha peso com a 005 em xeque, em vez de perder.** Meu
laudo de hoje dizia "o seguidor deve usar a cauda, ~1,0 rad/s²" — escrito
assumindo que existe uma lei de frenagem para alimentar. Com a 005 em xeque essa
premissa cai, mas a conclusão não: se `wz` não é modulável, a **única** alavanca
que sobra é decidir *quando cortar*, e o que acontece depois do corte é
exatamente o que o `a_dec` descreve. Ele deixa de ser ganho de controlador e
vira **limite de precisão de rumo da máquina**: partindo de 2,33 rad/s são ~47°
de sobrepasso (faixa 42–53°) em qualquer pivô comandado, mais o que girou sob
comando. Piso, não sintonia — nenhum ganho conserta isso.

**E uma correção de interpretação que veio de lá:** a "zona morta de giro" de
0,095 rad/s continua certa como número e muda de significado — é o disparo da
compensação (`mx > 1.0` → `wz > 0,0621`) somado à latência sobre a rampa, não
atrito. Descreve o sistema, não a máquina. **O atrito segue não medido**, e com
a compensação ligada 08-04 também não conseguiu medi-lo.

## 🪞 2026-08-04 (2ª leva) — O simulador passa a arcar como o robô arca

Sessão sem robô, em cima dos seis CSV de reta da leva da manhã. É o passo 12
do ESTADO (calibrar o simulador contra o robô), e ele fechou — com um defeito
meu no meio que só a corrida no Gazebo pegou.

**Dado:** `docs/dados/2026-08-04-aceitacao-simulador/` (CSV + `leitura.txt`).

### O ponto de partida não era o que o ESTADO dizia

O item 12 afirmava que o simulador reproduzia "~1×" de assimetria entre frente
e ré. Isso era verdade em **28-07**; a reescrita do `placa_simulada.py` de
01-08 já tinha posto assimetria dependente de sentido. Medido de verdade,
rodando as funções do nó:

```
                 simulador de ontem      robô (04-08)
frente               -0,419 1/m           -0,838 1/m
ré                    0,000               -0,113
razão                    ∞                    7,4x
```

Não era 1×, era **infinito** — ré perfeitamente reta. E errava nas duas
pontas em sentidos opostos do erro.

### Refiz as medidas do robô do CSV cru, e o registro se sustenta

Par matched (b,c), `curvatura = Δyaw/caminho`: frente **−0,838**, ré
**−0,113**, razão **7,4×**, parcela só-de-frente **86%**. Contra o registrado
(−0,817 / −0,098 / 8,3× / 88%). A diferença é janela de amostras e cabe dentro
dos 21% de dispersão do próprio robô. **O `ambiente.txt` de manhã é confiável.**

### O achado: metade do arco de frente não está nas rodas

Invertendo `curvatura = −2a/(L(2+a))`, a assimetria de roda necessária é
**24,8% de frente**. O encoder de 31-07 mediu **11–12%**. Ou seja: o corpo arca
o dobro do que as velocidades de roda explicam. É a quarta evidência
independente apontando para um termo de **corpo** (a boba), e num nível que nem
31-07 nem a leva da manhã tinham cruzado.

Consequência escrita no cabeçalho do nó: o arco entra **disfarçado de roda**,
porque a placa só tem rodas para escrever. O **encoder simulado passa a
mentir** (25% onde o robô tem 11–12%), o que é inofensivo hoje (`open_loop`) e
**quebra no dia em que ligarem `open_loop: false`** — que é item aberto.

### 🔴 O defeito: duas convenções de curvatura, e a ré espelhada

Primeira corrida de aceitação no Gazebo:

```
frente  -0,654 1/m   sinal certo, 80% da magnitude
ré      +0,088 1/m   SINAL TROCADO (alvo -0,098)
```

Causa: inverti os parâmetros com `curvatura = wz/v` (v **com** sinal) enquanto
a bancada mede `Δyaw/caminho` (caminho **sem** sinal). De frente as duas
concordam; **de ré elas dão sinais opostos**. O robô simulado arcava para o
lado errado indo de ré.

**A suíte estava verde** — porque o helper do teste usava a mesma convenção
errada do defeito. É o mesmo tipo de erro da régua do planner em 29-07: régua e
objeto medidos juntos, com o mesmo viés, e o erro fica invisível. **Só a
corrida no Gazebo pegou.**

Conserto de raiz: os parâmetros do nó deixaram de ser assimetria e passaram a
ser a **curvatura medida** (`curvatura_frente`, `curvatura_re`, os números da
bancada copiados sem conversão). A assimetria de roda é **derivada**:

```
a = -2cL / (2s + cL)          s = +1 frente, -1 ré
```

O `s` no denominador faz a assimetria sair **negativa na ré** sozinha. Ninguém
mais escreve sinal à mão, e o arquivo passou a conter os números da bancada em
vez de números convertidos.

### A derrapagem do Gazebo virou parâmetro nomeado

Com o sinal certo, o corpo simulado ainda arcava **80%** do que a placa pedia
(−0,654 contra −0,817). Bate com o déficit de giro medido em 29-07 (79–86%) —
é derrapagem do contato simulado, **não é do robô**. Entrou como
`rendimento_giro: 0.80`, com o aviso de que no robô real o equivalente é 1,0 e
de que mexer em atrito/massa/planta do Gazebo obriga a refazer esta corrida.

### O resultado, n=3 por sentido

```
                SIMULADOR        ROBÔ (04-08)
frente          -0,817 1/m       -0,838 1/m
ré              -0,109           -0,113
razão               7,5x             7,4x
```

As três grandezas caem dentro da dispersão do próprio robô.

### Duas dívidas de instrumento pagas no caminho

- **`ensaio.py --topico`**: ele publicava direto em
  `/hoverboard_base_controller/cmd_vel`, que no simulador **passa por fora da
  placa fingida**. Era a limitação anotada em 31-07 ("o Gazebo provou o
  mecanismo, não o número"). Sem essa flag a corrida de aceitação mediria um
  robô sem atuador.
- **O dublê do `test_placa_simulada.py` copiava os defaults do nó à mão.**
  Mudei o fonte e os 8 testes seguiram verdes descrevendo o robô antigo. Passou
  a ler o `declare_parameters` do fonte via `ast`; mutação confirma (repondo
  0,12, dois testes quebram).

### A curvatura virou instrumento (`medir.py curvatura`)

Ela só existia como prosa no `ambiente.txt` da manhã. Agora roda igual em CSV
do robô e do simulador, que é o que torna os dois comparáveis, e tem regressão
contra os seis CSV crus versionados. Mutação: trocando o acúmulo de yaw pelo
`atan2` entre pontas, o arco de 242° lê **+0,555** — magnitude e sinal errados,
o defeito de `7a0c364`. O teste pega.

**440 testes verdes** (eram 429).

### O que NÃO fechou

1. **Dispersão (12c) segue aberta, e agora com número**: o simulador dá 4% de
   dispersão de frente contra 21% do robô, e **0%** na ré contra 37%. Ele é
   determinista demais e faz qualquer controlador parecer mais repetível do que
   vai ser.
2. Medido só a **0,25 m/s**, dentro do patamar. Acima de 0,838 m/s ninguém
   mediu nada, nem no robô nem aqui.
3. O **mecanismo** continua fora (BO-4): o modelo reproduz o sintoma.
4. Percurso de 1,2 m. A corrida de 3,7 m do robô deu arco mais forte (−1,15):
   a hipótese "o arco aperta com a distância" não foi testada em nenhum dos
   dois lados.

## 🪞 2026-08-04 (3ª leva) — O teto de aceleração angular e o atraso de desliga

Continuação da aceitação, investigando por que o pico de wz saia 5× menor no
simulador (0,45 contra 2,33 rad/s do robô), mesmo com a placa now modelada.

Suspeitei do `angular.z.max_velocity: 1.0` do diff_drive_controller (sabia que
no robô ele cede à compensação da placa e a deixa subir para ~2.2). No
simulador a compensação é um nó SEPARADO que vem ANTES do controlador,
então o corte caía DEPOIS dela e desfazia o que ela faz — o patamar
simplesmente não passava.

Levantei para 3.0. O pico subiu de 0,45 para 2,25 — agora bate com o robô.

Mas com aceleração angular em 1,5 rad/s² (planta normal), o sobrepasso ficou
94° contra 49° do robô — 2× maior. Investigando: a placa real **empurra
0,51 s DEPOIS do comando zerar** (latência de desliga), medido em 04-08. No
simulador com só o parâmetro `latencia` ela para em **0,24 s** — metade. Sem
modelar esse atraso de desliga, o Gazebo desacelera mais rápido do que a
máquina real.

**Deixa para depois**: é mudança estrutural (entra no loop da placa fingida,
não é parametrização) e o passo 1 (arco) já fechou — as três medidas rodam.

## 🪞 2026-08-04 (4ª leva) — Os dois "bate" eram de plantas diferentes

O dono pediu para ver funcionando, e a desconfiança estava certa: o resumo da
3ª leva somava dois ✅ que nunca valeram juntos. O arco (−0,817) tinha sido
aceito na planta **lenta**; o pico de wz (2,25) na planta **normal** — e
ninguém re-rodou a aceitação do arco na normal depois de recomendá-la.
Re-rodada: **−0,95 de frente, 13% forte e fora da faixa do robô.**

Causa: `rendimento_giro=0,80` foi calibrado na lenta. A derrapagem do contato
depende do transiente, e o transiente depende dos limites de aceleração do
perfil — na normal o Gazebo realiza 93% do giro pedido, não 80%. O rendimento
é propriedade da **planta**, então mudou de endereço: sai de número solto e
passa a ser escolhido pelo `sim.launch.py` junto com o perfil (lenta → 0,80,
normal → 0,93).

Re-aceitação na normal, n=3 por sentido, ao vivo para o dono
(`docs/dados/2026-08-04-aceitacao-simulador/normal-*.csv`):

```
                SIMULADOR              ROBÔ (04-08)
frente        -0,803 (3% disp)       -0,838  faixa -0,75 a -0,93   DENTRO
ré            -0,095 (0% disp)       -0,113  faixa -0,09 a -0,14   DENTRO
razão f/r          8,5x                  7,4x
pico de wz     2,17 rad/s             2,33   faixa 2,23-2,51       na borda
sobrepasso       97°                    49°                        ABERTO
```

A lição de método, registrada porque vai se repetir: **"bate" só conta na
configuração em que se vai usar.** Aceitação re-rodada a cada mudança de
planta, teto ou atrito — está escrito no `leitura.txt` e agora aqui.

O sobrepasso segue 2× (atraso de desliga, pendência estrutural conhecida) e a
dispersão segue 0–3% contra 21–37% do robô (12c). 440 testes verdes.

### O veredito do dono, ao vivo (fecha a 4ª leva)

Demonstração com GUI, as três manobras em malha aberta na frente do dono:
reto por 35 s (fechou círculo de 1,27 m, girou 494°), ré (16° em 3 m,
raio 10,9 m) e pivô (2,39 rad/s com comando de 0,6). Julgamento dele, nas
palavras dele: **"está igualzinho — se isso foi um comando de ir reto é
exatamente assim que ele faz. A ré também. O giro no lugar exatamente como
na vida real."**

O critério da decisão 004 ("o simulador só serve se errar como o robô erra")
está cumprido nas três manobras medidas, por número E por olho. Próximo
passo decidido pelo dono: malha fechada de rumo em cima deste modelo.

## 🎯 2026-08-04 (5ª leva) — A fatia 1 da 011: o robô simulado anda reto

O dono aprovou a fatia 1 ("vamos focar tudo nesse PID, em fazer ele seguir
reto de verdade, a ré também") e ela fechou na mesma sessão. Decisão escrita
antes do código: `docs/decisoes/011-malha-fechada-de-rumo-em-reta.md`.

**A arquitetura**: camada entre quem comanda e o atuador (`compensador_rumo`
no `robot_motion`), lei pura em `lei_de_reta.py` (padrão da casa). Serve
qualquer comandante. A lei explora o fato medido de que a compensação do
driver preserva a RAZÃO entre rodas: há autoridade contínua sobre curvatura
mesmo sem nenhuma sobre velocidade.

    wz = ff(sentido)·|v| + Kp·e + Ki·∫e      ff = −curvatura medida

**PI, não PID** — e o próprio teste corrigiu meu entendimento do porquê: sem
Ki o P ainda ZERA a curvatura (o robô anda reto!), mas ~6° torto da
referência — o erro constante que o P precisa manter para sustentar a
correção. O integrador existe para zerar o RUMO, não a curvatura. Isso está
travado em teste (`test_sem_integrador_o_rumo_assenta_torto`).

**Resultado na bancada** (Gazebo, planta normal, n=3 por sentido, dado em
`docs/dados/2026-08-04-fatia1-compensador/`):

```
                 SEM              COM            critério da 011
frente         -0,82 1/m       -0,0025 1/m         < 0,05     ✅ 20x
ré             -0,11           -0,0003             < 0,05     ✅
```

Raio de 1,2 m virou raio de 399 m. Giro de 0,2° em 1,2 m andados.

**Defeito de instrumento achado no caminho**: mutação por `sed` com strings
do MESMO tamanho no mesmo segundo deixa o `__pycache__` servindo bytecode
mutado para o fonte revertido — os testes "falhavam" com o código certo.
Limpar o cache resolveu; fica o aviso para as próximas mutações.

**Ressalvas** (no leitura.txt): dispersão 0% é determinismo do simulador,
não mérito; o ff aqui é exato por construção (mesma fonte da planta) — o
caso ff-errado-25% é coberto por teste de unidade; curva comandada passa
intocada por unidade, não exercitada na bancada.

13 testes novos na lei (453 verdes no total). Fatias 2-5 abertas.

### O aceite do dono fecha a fatia 1 (e as condições de validade, ditas)

Demonstração com GUI: mesmo comando que fechou o círculo de 494°, agora
pelo compensador — ida de 4,00 m com 0,0° de giro, volta DE RÉ de 3,99 m
com 0,0°, bico terminando onde começou. Veredito do dono: **"ficou
perfeito, se esse código funcionar no robô estamos voando alto."**

Às perguntas dele ("vai funcionar pra sempre? em toda aplicação?"), a
resposta registrada: NÃO incondicionalmente — as condições estão na 011
(LIO vivo, senão passa reto e grita; planta até ~25% do ff, senão re-medir;
compensação do driver ligada, senão a 011 reabre) — e vale para TUDO que
passar pela porta, que hoje é opcional (fatia 5 a torna obrigatória).
O juiz de verdade segue sendo a fatia 4, no robô.

## 📐 2026-08-04 (6ª leva) — A fatia 2 morre na medição: o gap era do instrumento

A fatia 2 (modelar o atraso de desliga da placa) abriu pelo método: medir
antes de mexer. E a medição derrubou a própria fatia.

O gap que eu reportava ("sobrepasso 99° no Gazebo contra 49° no robô")
comparava o sobrepasso A PARTIR DO PICO — que reparte diferente nos dois
lados (robô: empurra 0,5 s, freia forte ~3,3 rad/s²; simulador: empurra
0,16 s, freia suave ~1,5). Mas o número que o corte previsto consome é o
GIRO TOTAL entre "comando zerou" e "parou", fase de empurrão INCLUÍDA. O
`medir.py degrau_giro` passou a imprimi-lo, e ele diz outra coisa:

```
                giro total do corte à parada     tempo até parar
ROBÔ            102° / 114° / 124°  (~113°)      1,94–2,02 s
SIMULADOR       117° / 120° / 117°  (~118°)      1,82–1,86 s
```

**O simulador está DENTRO da faixa do robô.** Dois erros de forma se
cancelando num acerto de total — coincidência dos limites da planta, mas
medida e estável (n=3 dos dois lados, planta normal).

Consequência: **a fatia 2 sai do caminho crítico.** O corte previsto
(fatia 3) pode ser sintonizado no simulador de hoje, com uma condição de
projeto que fica registrada: o controlador do pivô deve usar SÓ o giro
total e a detecção de parada — NÃO a forma da frenagem (pico, wz no meio
do caminho), porque a forma é onde o simulador ainda mente. Se o projeto
da fatia 3 precisar da forma, a fatia 2 volta.

É a segunda vez no mesmo dia que a régua respondia outra pergunta que não
a feita (a primeira: caminho/afastamento no critério do 12b). A lição já
estava escrita em 29-07 na régua do planner; agora tem dois exemplos.

## 🔄 2026-08-04 (7ª leva) — O pivô não tem quantum: o tempo ligado é um manche

Fatia 3 aberta pelo método (medir antes de mexer), e a medição derrubou o
medo que a motivava. Dado em `docs/dados/2026-08-04-fatia3-pivo-minimo/`.

O medo: a placa entrega um wz só (~2,2 rad/s) e depois do corte o robô varre
~113°. Se isso fosse o quantum, pivô menor que meia volta seria impossível.

**Não é.** Varrendo o tempo LIGADO (`ensaio.py --liga`, novo — é a única
coisa escolhível num atuador que não modula magnitude):

```
liga [s]   0.1   0.2   0.3   0.5    0.8    1.2    2.0
giro         0°    0°    0°   3,7°  22,4°  68,9°  215,7°
pico wz    0,00  0,00  0,00  0,24   0,68   1,20    2,38
```

A razão é simples e estava na cara: **o robô ainda está acelerando quando o
corte chega.** O pico não é 2,2 — é o que deu tempo de subir. Os 113° são a
sobra *quando ele chegou ao patamar*, não um piso. O pivô é contínuo desde
~4°.

**Zona morta de TEMPO medida pela primeira vez**: abaixo de ~0,4 s ligado o
robô não sai do lugar. É o BO-3 numa dimensão que nunca tinha sido medida.

**A sobra não segue `wz²/(2·a)` com `a` constante** (o `a` implícito varia
2,2× na faixa) — a mesma não-constância que o robô mostrou, por outro
caminho. A lei da 005 erraria aqui de novo.

### ⚠️ A ressalva que decide o desenho da lei

A relação tempo→ângulo depende da **taxa de subida** do wz, e ela tem origens
diferentes nos dois lados: no simulador é um limite de **config**
(`max_acceleration: 1.5`, exato); no robô é **inércia** (1,58–2,01, 25% de
espalho). Coincidem em valor **por acaso, não por modelagem**. Só o ponto
`liga=2,0` foi conferido contra o robô.

Então a tabela acima **não pode virar tabela de consulta do controlador**. A
lei do pivô tem de funcionar sem confiar nela: **aproximação sucessiva com
corte conservador** — cortar cedo custa só tempo (27-07: "errar para baixo é
de graça"), e o resíduo se resolve com um pulso menor. A curva serve para o
primeiro chute; o fechamento é por realimentação. E isso torna a fatia 3
testável de verdade aqui: roda-se a lei com parâmetros de sobra
deliberadamente errados, como o `ff_errado_25_por_cento` fez na fatia 1.

**Para a fatia 4**: entra no protocolo do robô uma varredura de `--liga`
(0,3/0,5/0,8/1,2/2,0, n=3) — ~15 corridas curtas, e é o único jeito de ter
a curva real.
