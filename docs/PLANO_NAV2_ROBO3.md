# Plano — adaptar o Nav2 para o robô 3 (v2.5)

> **Status: plano APROVADO em 17-09, depois de seis revisões cruzadas.**
> **Etapa 0 executada** no mesmo dia (§8) — é a única coisa implementada até
> aqui; da etapa 1 em diante nada foi feito, e nada foi ao robô.
>
> **Seis** revisões cruzadas em dois dias, e cada uma achou coisa real:
>
> - **v1** (2026-09-16) — a primeira revisão **derrubou duas etapas inteiras**
>   (§1): a pilha não subia sobre o simulador do robô 3 e a localização não
>   tinha TF.
> - **v2** (2026-09-17) — reescrita inteira depois da primeira revisão: ordem
>   nova, bringup do robô 3 como etapa própria, Livox adiado para a etapa 7.
> - **v2.1** (mesmo dia) — a segunda revisão pediu cinco ajustes, todos
>   aplicados: classificação de parâmetros; a fronteira `TwistStamped → Twist`
>   passa a ser **decidida** na etapa 5 e não adiada (§4); parada física virou
>   **pré-requisito das etapas 2, 8, 9 e 10**; a contradição do
>   `ESTADO_PROJETO` corrigida (trocar o lado do puxão **não** prova causa de
>   canal); validação do LIO ampliada para a pose 6D.
> - **v2.2** (mesmo dia) — a terceira achou **um erro meu na própria etapa 0**:
>   eu calculei a envolvente do robô 3 como 0,196 m usando a meia-diagonal em
>   torno do **centro da caixa**, mas o `scan_2d` produz no `base_link`, que no
>   robô 3 está no eixo das motoras. O valor certo é **0,276 m** (§8). Junto:
>   um teste geométrico **não** decide visibilidade (FOV, oclusão, ray casting)
>   — a validação real é com nuvem do robô parado; e três reclassificações:
>   `raio_min_curva` é planta medida, a **folga** somada ao raio varrido é
>   política, e `min/max_height` são **percepção**, não geometria pura (§5).
>
> - **v2.3** (ainda 17-09) — a quarta rodada pegou três incoerências minhas: o
>   texto dizia "quatro classes" com **cinco** enumeradas fora de ordem;
>   `range_min`/`laser_min_range` continuavam em (b) geometria embora o próprio
>   plano dissesse que só fecham na etapa 9 com autorretorno real (viraram
>   **(e) percepção**); e a etapa 0 prometia teste **"por robô"** quando existe
>   um `scan_2d.yaml` só, com os perfis nascendo na etapa 4 — além de eu ter
>   chamado o conserto de "uma linha", o que ele não é. Junto veio o risco
>   conceitual mais importante até agora: **`range_min` é corte radial num robô
>   assimétrico**, e dimensioná-lo pela cauda cega ~19 cm à frente do nariz (§8).
>
> - **v2.4** (ainda 17-09) — a quinta achou uma **contradição lógica** minha: a
>   etapa 0 exigia `range_min` **maior** que a envolvente (0,276 m), enquanto a
>   etapa 9 exigia enxergar obstáculo logo fora do contorno (nariz a 0,0825 m).
>   A faixa de 0,0825 a 0,276 m à frente está **fora do robô** e seria cortada:
>   o teste da etapa 0 **vetaria** a saída por filtro espacial antes de a etapa 9
>   poder avaliá-la. A relação certa é `autorretorno visível máximo ≤ envolvente`
>   — cota **superior** do autorretorno, não piso do corte (§8).
>
> - **v2.5** (ainda 17-09) — a sexta enxugou a etapa 0: a coerência
>   `range_min`↔`laser_min_range` **já é testada** (e com igualdade) em
>   `test_configs_coerentes.py`, então calcular a envolvente "só como
>   diagnóstico" seria **teste que não afirma nada** — saiu. A etapa 0 fica em:
>   remover o teste inválido, corrigir os textos, manter o de coerência. E a
>   etapa 9 passa a estar **incompleta** até os testes da estratégia escolhida
>   existirem, em vez de "depois da 9". Editorial: o histórico omitia a **v2.1**
>   e atribuía os ajustes dela à v2 — corrigido acima.
>
> O que sobreviveu de todas: consertar o teste da etapa 0 **não é** trocar
> `robot_radius` por footprint — isso reprovaria o robô 2 (0,447 m contra
> `range_min` 0,35). Essa parte estava certa; o número que derivei dela e a
> desigualdade que construí em cima dele, não.
>
> Irmão do `PLANO_CONTROLE_ROBO3.md`. Regra do projeto: cada etapa é uma mudança
> pequena, com o "pode" do dono antes de ir ao robô. Este documento não autoriza
> nada — ele ordena.

---

## 0. O que o dono decidiu (16-09)

1. **A frente do robô 3 são as MOTORAS; a traseira são as BOBAS.**
   Eletricamente é o contrário, e a correção fica **no código**.
2. **O Livox é emprestado do robô 2**, montado **no centro do robô, no topo**.
3. Atuador se baseia no **robô 1** (MEGA); seguidor e navegação, no **robô 2** —
   também diferencial de 2 motoras. O robô 1 é 4x4 skid-steer, não faz curva, só
   pivô: a lei de movimento dele não serve aqui.

⚠️ **Enquanto o robô 3 navega, o robô 2 não navega.** Só há um Mid-360.

---

## 1. O que a revisão de 17-09 derrubou da v1

Tudo abaixo foi **conferido no código**, não aceito de boca. A v1 está errada
nestes pontos e eles são a razão de esta versão existir.

| # | o que a v1 dizia | por que está errado |
|---|---|---|
| 1 | "etapa 3: subir a pilha do robô 2 sobre o `sim_robo3`" | `pilha.launch.py:556` inclui `robot_base/launch/sim.launch.py`, que é o **robô 2**. Com `sim:=false` para evitá-lo, a pilha inteira fica `use_sim_time=false`. E `cmd_vel_to_wheels` publica `WheelSpeeds`, que **não tem consumidor no Gazebo** — o sim do robô 3 termina em `ros2_control`. Fechar a cadeia até o `cmd_vel_to_wheels` não prova movimento nenhum. |
| 2 | "a costura é só a saída do `compensador_rumo`" | A cadeia do robô 2 é **inteira** `TwistStamped` (`compensador_rumo.py:237`, `collision_monitor`, `twist_mux.yaml: use_stamped: true`). A do robô 3 é **inteira** `Twist` (mux, teleop, `dpad_reto`, `cmd_vel_to_wheels`). São dois contratos, não um adaptador. |
| 3 | "etapa 5: localização de pé" | Nem o `controle_robo3.launch.py` nem o `localizacao.launch.py` sobem **`robot_state_publisher`**. Sem `base_link → livox_frame`, o `tf_odom.py:114` **deliberadamente não publica** `odom → base_link` — e a mensagem de erro dele já diz para conferir o RSP. Falta um bringup real do robô 3. |
| 4 | "girar 180° = trocar o sinal de 3 x" | Falso. O trail da boba é fixo em **−x** (garfo, visual e `caster_wheel_joint: xyz="${-boba_trail} 0 …"`). Trocar só `boba_x` deixa a boba **liderando** no modelo. Junto vêm: semântica esquerda/direita, orientação do `livox_frame`, envolvente varrida pelas bobas, e `test_urdf_robo3.py:119`, que trava `eixo_x < ox < boba_x` de propósito. |
| 5 | "8,35 cm de nariz" | **8,25 cm.** O comentário do próprio xacro erra 1 mm; o teste prova que a roda passa exatamente 2 cm da traseira da caixa (−0,0625 − 0,0825 = −0,0825). E a traseira tem de considerar a **varredura das bobas**, não a ponta da caixa. |
| 6 | "a margem de rejeição do piso cai quase pela metade" | Errado. `target_frame` é `base_link`, que está no chão: `min_height: 0.15` continua sendo 15 cm do chão com sensor a qualquer altura. O que muda com a altura é a **região observável**. |

**Duas coisas a mais que a revisão levantou e que a v1 nem mencionava:**

- **Dois launches querem ser donos do mux** (`pilha` e `controle_robo3.launch.py`).
  Subir os dois faz nó duplicado e duas arquiteturas concorrendo.
- **`test_scan_2d.py` está VERMELHO hoje**, e não tem nada a ver com o robô 3:
  procura um `robot_radius` que saiu do `nav2.yaml` na decisão 032.
  `test_urdf_robo3.py` 21/21 e `test_configs_coerentes.py` 83/83 passam.

---

## 2. Por que a frente nas motoras (mantido da v1)

A decisão 009 e a `lei_de_rumo`: andando para trás *"a boba deixa de ser
arrastada e passa a ser empurrada"* — e aí a direção fica instável. Hoje as
motoras estão em x=0 e as bobas em x ≈ +0,2485: na marcha que o URDF chama de
"frente" as bobas vão **empurradas**. Com a frente nas motoras, **arrastadas**.

Casa com o robô ter mostrado a ré andando reto e a frente puxando. **Hipótese,
não prova** — e a etapa 2 é o que a mede. Segue como pergunta aberta do artigo.

---

## 3. O giro de 180° é uma obra no URDF, não três sinais

O `base_link` continua no centro do eixo das motoras, no chão (convenção C8: o
centro de rotação é a origem, `wz` puro não desloca — isso é bom e não se mexe).
O que gira é o **corpo em relação a ele**. Itens obrigatórios:

1. `caixa_cx`, `boba_x`, `livox_x` — os três sinais (necessário, longe de suficiente).
2. **Direção do trail** da boba: hoje `-boba_trail`; girar o conjunto inverte o
   sentido em que a boba se alinha atrás do pivô.
3. **Semântica esquerda/direita**: uma rotação de 180° em z troca ±y. Qual roda
   física é `left_wheel_joint` muda junto.
4. **Orientação do `livox_frame`** (não só a posição): o yaw de montagem entra
   na nuvem inteira.
5. **Envolvente varrida pelas bobas** — elas giram no pivô; o contorno é o que
   elas varrem, não o ponto onde estão paradas.
6. **`test_urdf_robo3.py` e os comentários** que afirmam "motoras atrás": o teste
   está certo hoje e vai ficar errado depois. Ele se reescreve junto, de
   propósito e com registro — não se apaga.

**O `linear_sign: -1.0` fica.** Ele é o adaptador elétrico, e a decisão 049 já
provou que negar só a linear equivale à rotação de 180° **incluindo a troca de
qual roda é a esquerda** — então o caminho de comando já está certo para o frame
girado. ⚠️ Mas isso vale só para o **comando**: o `pose_estimator` tem
`left/right_wheel_sign` e **não** tem `linear_sign`. Se um dia a odometria de
roda entrar, ela não acompanha o giro sozinha.

---

## 4. Contrato de mensagens: unificar, não adaptar

Recomendação (adotada da revisão): **cadeia única em `TwistStamped` até o
compensador — Xbox e direcional inclusos — e conversão só na fronteira do
atuador.** O tópico se resolve por **remapeamento**; não precisa virar parâmetro.

Retirado da v1: o argumento de que um nó adaptador seria ruim "por causa dos
0,94 s". Aquele tempo morto é da **planta/malha do robô 2**, não de um salto de
nó — usá-lo como argumento era medida emprestada, exatamente o que este repo
proíbe. Se um adaptador for escolhido, o custo dele se **mede**.

Junto: **um mux só**. Hoje a `pilha` e o `controle_robo3.launch.py` disputam o
mesmo nó, e o `dpad_reto` e o teleop do robô 3 precisam passar a falar o mesmo
contrato do resto.

🔴 **A fronteira ainda NÃO está decidida, e a etapa 5 tem de fechá-la.** As duas
saídas vivas são: **(i)** um nó adaptador `TwistStamped → Twist`, ou **(ii)**
fazer o `cmd_vel_to_wheels` aceitar `TwistStamped`. Dizer "conversão na
fronteira" sem escolher é adiar, não decidir. A etapa 5 escolhe e **testa**:

- prioridade do mux respeitada (humano fura autonomia);
- `timeout` de cada faixa (o homem-morto continua soltando o robô);
- **sinal de `v` e de `wz`** ponta a ponta, com o `linear_sign` no meio;
- **nenhum nó duplicado** — nem dois mux, nem dois `cmd_vel_to_wheels`.

---

## 5. Perfis `robo2` / `robo3`: o que precisa de perfil próprio

A v1 listava 7 itens. São muitos mais, e **herdar número medido de outra máquina
é o defeito que este repo já pagou** (a bitola de 29-07).

São **cinco** classes, e misturá-las é o que fez a v1 mandar medir geometria com
CSV. Cada classe tem origem, dono e momento diferentes:

**(a) Planta medida — sai de ensaio com CSV, no robô.** `curv_frente`/`curv_re`,
`zona_morta`, `retencao_giro_s`, `linear_scale` e o ganho linear realizado,
ganho de giro, aceleração e frenagem reais, atraso liga/desliga, e o
**`raio_min_curva`**.
→ São propriedades **da máquina e da placa**. Etapa 8.

⚠️ O `raio_min_curva` parece geometria e não é: ele sai do que as rodas
**conseguem** entregar (zona morta, teto de velocidade, razão entre os lados),
não do contorno do corpo. A v2.1 o tinha posto em (b), errado.

**(b) Geometria — sai da trena e do URDF, NUNCA de CSV.** Footprint global e
local do Nav2, os **dois** polígonos do `collision_monitor`, **meia largura,
corredor de ré e recuo do para-choque** do `path_follower`, e o **raio físico
varrido pelas bobas**.
→ Existem **antes** de qualquer simulação de navegação. Etapa 3.

⚠️ O raio varrido é geometria, mas **a folga que se soma a ele é (c)**, política
de segurança. Os dois moram em lugares diferentes e mudam por motivos
diferentes; somar e guardar um número só apaga essa distinção.

**(c) Limites e política de segurança — escolha nossa, não medida.** `v_max`,
`wz_max`, largura de passagem e suas margens, `desvio_taxa_deg_s`, quando a ré é
permitida, o que o reflexo faz ao disparar.
→ Decisão registrada, com o "porquê". Limitada por (a), nunca maior que ela.

**(d) Sintonia do controlador — ajuste, dentro do que (a) e (c) permitem.**
`a_dec`, `a_lin`, `ganho_wz`, ganhos kp/ki do rumo e da reta.
→ Último a mexer, e só com corrida de controle antes e depois.

**(e) Percepção — informada por geometria E por ruído, não é geometria pura.**
`min_height` / `max_height` do `scan_2d`, alturas da `VoxelLayer`, e **`range_min`
com o `laser_min_range` que anda em par com ele**.
→ A geometria dá uma **cota**; quem fecha o valor é o **comportamento**: piso que
não é plano, o balanço do chassi (no robô 3, 4 apoios rígidos — 3 mm de junta de
piso viram 1,2°, e o Livox vai em cima disso) e o autorretorno real. Etapa 9,
com nuvem.

⚠️ `range_min` estava em (b) até a v2.2 — **incoerência minha**: o próprio plano
dizia que o valor final só fecha na etapa 9, com os autorretornos reais. Se o
número depende do que o sensor devolve, ele não é geometria.

⚠️ A v1 punha meia largura, corredor de ré e recuo do para-choque em "calibração
por CSV". São **(b)**: saem da trena. E `desvio_taxa_deg_s` não é planta nem
geometria — é **(c)**, uma escolha de segurança.

🔴 **O footprint é geometria, não calibração** — ele tem de existir **antes** de
qualquer simulação de navegação, não no fim. A v1 o jogava para a etapa 6.

⚠️ **`range_min` e `laser_min_range` andam em par** — mudar um sem o outro deixa
o AMCL com um limite e a fatia com outro, e isso continua valendo.

🔴 Mas **o `range_min` é (e), não (b)** — corrigido na v2.3, e este parágrafo era
o último resto do enquadramento antigo. A geometria própria visível do Livox
(rodas e varredura das bobas incluídas, não a largura de 24 cm da caixa) dá
apenas uma **cota inferior**; o valor que vai para o arquivo só fecha na **etapa
9**, contra autorretorno real — e sob a trava dos dois lados do §8, porque um
corte radial grande demais cega a frente de um robô assimétrico.

---

## 6. Escala SI: hoje o robô 3 não tem significado físico

`escala:=400` é, nas palavras do próprio launch, **"de partida, não calibrada"**
(`controle_robo3.launch.py:150`). Enquanto isso for verdade, `v_max: 0.5`, a
projeção do `collision_monitor` e qualquer distância de frenagem são números
sem lastro. Antes do Nav2, medir:

- `linear_scale` e o ganho linear realizado;
- ganho de giro e as velocidades máximas realmente executáveis;
- atraso liga/desliga e distância real de parada;
- soma dos watchdogs (mux 0,3–0,5 s + MEGA 0,5 s) e a retenção da placa;
- aceleração e frenagem;
- zona morta e curvaturas.

---

## 7. O ensaio de frente/ré precisa de protocolo (e não decide sozinho)

A v1 dizia "se o puxão trocar de lado, a causa é de canal". **Não prova.**
Assimetria de roda, de carga e o transiente das bobas produzem o mesmo sinal.
E o bag do `bin/sobe-robo3:67` **não grava pose** — sem Livox não há medida de
desvio lateral sem régua ou câmera externa.

O ensaio só vale com: distância e velocidade fixas, orientação inicial marcada,
**alinhamento prévio das bobas** (elas têm memória do movimento anterior), ordem
**aleatorizada** entre os dois sentidos, tensão da bateria anotada a cada
corrida, e uma medida objetiva de curvatura — não "andou tortinho".

---

## 8. Ordem adotada (da revisão, com o que cada etapa prova)

Uma etapa por sessão. Nenhuma começa sem a anterior fechada.

| # | etapa | prova / entrega | precisa do robô? |
|---|---|---|---|
| 0 | ✅ **FEITA (17-09)** — teste inválido removido, textos corrigidos, o de coerência intacto | `test_scan_2d.py` **7/7 verde**; o corte não ficou petrificado e não entrou teste que não afirma nada | não |
| 1 | Fechar **a placa** (§5.1 da revisão cruzada) e a geometria física autoritativa (trena) | sem isso a etapa 8 recomeça do zero | sim, desligado |
| 2 | Repetir o ensaio frente/ré com o protocolo do §7 | confirma ou derruba a premissa do §2 | sim, ligado |
| 3 | URDF completo girado (§3) + `robot_state_publisher` + footprints + testes reescritos | modelo e marcha concordam; o Nav2 passa a ter contorno | não |
| 4 | Perfis `robo2`/`robo3` e **um bringup único** do robô 3 | RSP + MEGA + `cmd_vel_to_wheels` + Xbox/direcional + mux único num lugar só | não |
| 5 | Unificar o contrato de mensagens (§4) e testar a cadeia **sem Gazebo** | comando atravessa de ponta a ponta, sem simulador para confundir | não |
| 6 | Ensinar a `pilha` a escolher `sim_robo3` (`robo:=3`, `use_sim_time`, qual atuador encerra) | a etapa 3 da v1, agora possível | não |
| 7 | Montar o Livox, medir a pose **6D** e validar o LIO por inteiro (abaixo) | a árvore de TF fecha com medida, não com chute | sim |
| 8 | Calibrar escala e dinâmica (§6); depois rumo e curvatura | os números do Nav2 passam a ter lastro físico | sim |
| 9 | Validar percepção e reflexo (`scan_2d`, `collision_monitor`) — **os dois lados do `range_min`** (§8), escolher a estratégia **e escrever os testes dela** | o robô enxerga e freia antes de planejar, **e não fica cego na frente**. 🔴 Não fecha só com a escolha | sim |
| 10 | Nav2 `mapa:=nenhum`, espaço livre, **parada física independente do Xbox** | objetivo curto | sim |

⚠️ A ordem mudou de verdade em relação à v1: **o Livox só sobe na etapa 7**, e
não na 4. Até lá o robô 2 continua navegando.

🔴 **A parada física independente do Xbox é PRÉ-REQUISITO de toda etapa com as
rodas no chão — 2, 8, 9 e 10**, não só da 10. O homem-morto do LB depende do
controle, do Bluetooth e da pilha ROS de pé; nenhum dos três é confiável durante
justamente os ensaios em que se está mexendo neles. A decisão 048 já registrou
a placa girando sozinha com a MEGA mandando zero.

**Etapa 0 — o que o teste passa a conferir, e por que não é substituição direta.**
`test_o_robo_nao_se_enxerga_como_parede` lia `robot_radius` do `nav2.yaml` e
exigia `range_min > robot_radius` (0,35 > 0,32). O `robot_radius` saiu na decisão
032. **Removido em 17-09** — ver o bloco de comentário que ficou no lugar dele. 🔴 **Mas trocar por "o footprint" reprova o robô 2**: o vértice do contorno
da 032 está a √(0,35² + 0,2775²) = **0,447 m**, contra `range_min` 0,35.

Não é defeito do `range_min` — é a grandeza errada. O que o robô vê de si mesmo
é o que está **dentro da fatia** (0,15–1,00 m), e não o contorno no chão (que
inclui para-choque e roda, abaixo da fatia). A meia-diagonal da caixa do robô 2
é √(0,2165² + 0,2275²) = **0,314 m** — exatamente o *"0,32 = 0,314 medido + 6 mm"*
que o `nav2.yaml` documenta. Era isso que o `robot_radius` representava ali.

🔴 **CONTRADIÇÃO DA v2.3, corrigida aqui (v2.4) — e era minha.** A v2.3 mandava o
teste exigir `range_min` **maior** que a envolvente. Isso briga de frente com a
trava de dois lados que a mesma versão criou para a etapa 9: para o robô 3,

    nariz 0,0825 m   ·   envolvente 0,276 m

um obstáculo frontal **entre 0,0825 e 0,276 m está FORA do robô** e seria
apagado pelo corte. Ou seja: o teste da etapa 0 **proibiria de antemão** a saída
por filtro espacial com `range_min` pequeno — justamente a saída que a etapa 9
pode concluir ser a certa. Teste que veta a solução antes de ela ser avaliada
não é trava, é chute petrificado.

➡️ **A relação correta é a inversa, e é uma cota SUPERIOR:**

    autorretorno visível máximo  ≤  envolvente geométrica

A envolvente limita **onde o autorretorno pode estar**; ela não é piso
obrigatório para o `range_min`. Com filtro espacial, o corte radial pode e deve
ficar pequeno.

🔴 **Mas a etapa 0 NÃO pode ser "por robô", e isso é correção da v2.3.** Hoje
existe **um** `scan_2d.yaml` só; os perfis nascem na **etapa 4**, e a geometria
autoritativa do robô 3 só fecha nas etapas **1** (trena) e **3** (URDF girado).
Prometer "por robô" na etapa 0 é prometer contra coisa que ainda não existe.
Então:

**Etapa 0 — ✅ EXECUTADA em 17-09** (`test_scan_2d.py`: 7 passaram, 0 falharam).
Escopo enxuto (v2.5), e foi só isto:

⚠️ **O critério é "testes DO PROJETO verdes", não "pytest inteiro verde"** — e a
diferença importa. `pytest` da raiz, sem argumento (ROS carregado) dá
**845 passed, 1 failed, 7 errors**, e as 8 restantes são do **`twist_mux`
vendorizado** (pacote de terceiro, testes de `launch` que penduram). Dizer
"suíte verde" sem essa ressalva é afirmação que não se sustenta — eu disse, e
está corrigido aqui.

⚠️ E **a invocação decide o resultado**: passar `ros2_packages/` como alvo dá
*"no tests collected"* e aborta com um `ImportError` de `rclpy.qos` que é
`sys.path`, não defeito. Quem usar a invocação errada vai "descobrir" um defeito
que não existe.

1. **remover** `test_o_robo_nao_se_enxerga_como_parede`, que é inválido: ele
   afirma uma desigualdade que a v2.4 derrubou e depende de um `robot_radius`
   que saiu na decisão 032;
2. **corrigir o texto** que sobra dele e o **comentário do `scan_2d.yaml:48`**
   (*"`range_min` 0,35 > raio do robô (0,32)"*), categórico e citando o mesmo
   parâmetro extinto;
3. **manter** o teste de coerência AMCL↔fatia que **já existe e já passa** —
   `test_o_alcance_do_amcl_bate_com_o_do_SCAN`, parametrizado em
   (`laser_min_range`,`range_min`) e (`laser_max_range`,`range_max`), exigindo
   **igualdade** entre `robot_motion` e `robot_base`.

🔴 **O que NÃO entra: calcular a envolvente "como diagnóstico".** Um teste que
não afirma nada não protege nada — é código a manter com aparência de rigor. A
coerência que a v2.4 queria proteger **já está coberta** pelo item 3. O cálculo
da envolvente espera haver uma **estratégia concreta de filtragem** para validar.

- **etapa 9**: mede o autorretorno **real** e **escolhe** entre corte radial e
  filtro espacial. ⚠️ Ela **não fecha** na escolha: só está completa quando os
  **testes da estratégia escolhida** estiverem escritos (§8, abaixo);
- **etapa 4**: quando os perfis existirem, tudo isso passa a valer **por robô**.

⚠️ **Sobre o custo disto, e a conta mudou duas vezes.** Calcular a envolvente a
partir do URDF exige interpretar caixas, rodas, alturas e as transformações até
o `base_link` — não é "uma linha", como eu havia escrito na v2.2. Mas na **v2.5
esse cálculo saiu da etapa 0** (acima): sem estratégia de filtragem para
validar, ele não protegeria nada. Então a etapa 0 **voltou a ser pequena**, e o
aviso de custo passa a valer para **quando a envolvente for de fato calculada**,
na etapa 9 ou depois — não para agora.

🔴 **Erro da v2.1, corrigido aqui (17-09):** eu escrevi que a envolvente do robô
3 era 0,196 m, que é a meia-diagonal em torno do **centro da caixa**. Errado: o
`scan_2d` produz no `target_frame: base_link`, e no robô 3 a C8 pôs o
`base_link` no **eixo das motoras**, não no centro da caixa — que fica deslocado
`caixa_cx` = 0,093 m. No frame certo:

    √((0,093 + 0,1555)² + 0,120²) ≈ 0,276 m

No robô 2 os 0,314 m valem porque lá o `base_link` **é** o centro da caixa. Esta
é exatamente a armadilha que o próprio xacro avisa que a C8 cria, e eu caí nela
no mesmo documento em que a citei.

⚠️ **E um teste puramente geométrico não fecha a questão.** Estar dentro da
faixa vertical do `pointcloud_to_laserscan` **não** significa ser visto pelo
Livox: entram FOV vertical, posição e orientação do sensor, oclusão, e a
distância tem de ser calculada no `base_link`. Fazer isso direito exigiria ray
casting. Então:

- **no teste**, uma cota conservadora derivada do URDF no `base_link` (é o que
  ele pode afirmar sem mentir);
- **a validação de verdade** é com **nuvem real do robô parado**, olhando o
  autorretorno — etapa 9, não agora.

⚠️ Segue aberta, e é do robô 2 de hoje: **o que mais sobe acima dos 0,15 m**
(roda com topo a 0,16 m, cabos, o suporte do Livox)?

🔴 Mas **cuidado com a conclusão** — até a v2.3 este parágrafo terminava dizendo
que, se algo passasse de 0,35 m em raio, "o `range_min` está apertado". Isso é o
raciocínio que a v2.4 derrubou: achar autorretorno longe **não** manda subir o
corte. A pergunta mede **onde o autorretorno pode aparecer**, e a resposta é
entrada da decisão da etapa 9 — corte radial *ou* filtro espacial. Se o
autorretorno for longe **e** houver obstáculo real a cobrir na mesma faixa, é
justamente o caso em que o corte radial não serve e o filtro espacial é a saída.

### 🔴 O risco de ver o `range_min` como "aumentar até parar de se enxergar"

`range_min` é um corte **radial**, e o robô 3 é **muito assimétrico no
`base_link`**: o nariz (ponta da motriz) está a 0,0825 m e a cauda a 0,2485 m.
Um corte dimensionado pela cauda cega a frente:

    0,276 (corte) − 0,0825 (nariz) ≈ **0,19 m de cegueira à frente do robô**

Ou seja: subir o `range_min` até o autorretorno sumir **também apaga obstáculo
real colado na frente** — justamente onde o robô anda. Trocar um defeito sem
sintoma (anel fixo que trava o AMCL) por outro sem sintoma (obstáculo baixo
invisível na direção de marcha) não é conserto.

➡️ **A etapa 9 testa os DOIS lados, e um só não aprova:**

1. **sem autorretorno** com o robô parado (nuvem real, todos os setores);
2. **preservando um obstáculo alto posto logo fora do contorno físico** — ele
   tem de aparecer no `/scan`.

➡️ **Se um `range_min` único não fizer os dois, a saída não é aumentar o corte
radial** — é **filtro espacial/angular de autorretorno** (descartar por *onde* o
ponto está no `base_link`, não por *quão perto* ele está). Fica registrado aqui
para não ser reinventado no susto, no meio da etapa 9.

➡️ **E o teste só ganha dente com a estratégia escolhida** — se o corte é radial,
que ele cubra o autorretorno medido; se é filtro espacial, que o filtro pegue o
autorretorno **e** que a visibilidade logo fora do contorno continue de pé.
Antes disso o teste não tem o que travar, e fingir que tem foi o erro da v2.3.

🔴 **Isso é parte da etapa 9, não "depois" dela (v2.5).** Chamar de "depois"
deixava a etapa fechar com uma decisão tomada e nada travando — que é
exatamente como uma escolha vira folclore no projeto. **A etapa 9 está
incompleta enquanto os testes da estratégia escolhida não existirem.**

**Validação do LIO na etapa 7 (a v1 pedia só x, y e yaw):**
z, roll e pitch; **deriva com o robô parado**; **sentido positivo do yaw**
(girando para a esquerda o yaw sobe); e **sobreposição da nuvem com o `/scan`**
— se os dois discordarem, a fatia 2D está mentindo para o AMCL sem sintoma.

---

## 9. Opiniões e dúvidas em aberto (para o dono)

1. **Reusar ou duplicar?** Continuo em **reusar** `robot_motion`/`robot_base`
   com perfil `robo:=3`. Duplicar faz toda correção ter de ser aplicada duas
   vezes — e já há `sim_robo3.launch.py` e
   `hoverboard_controllers_sim_robo3.yaml` separados como precedente.
2. **A placa do robô 3 está decidida?** É o item de maior alavancagem (§5.1 da
   revisão cruzada): se mudar, zona morta, patamar e latência caem junto e a
   etapa 8 recomeça. Por isso subiu para a etapa 1.
3. **Só LIO no começo**, deixando IMU e optical flow da MEGA de fora: um sensor
   a mais sem necessidade é um modo de falha a mais.
4. ~~**Quero fazer a etapa 0 agora?**~~ ✅ **Respondida e feita em 17-09.** Com o
   escopo enxuto da v2.5 ela voltou a ser pequena — remover um teste inválido e
   corrigir dois textos — e o cálculo da envolvente, que era a parte cara, saiu
   de cena até haver estratégia de filtragem para validar (§8).
5. **Etapa 1 primeiro — e a minha sugestão anterior estava errada.** Eu havia
   dito que, se a placa dependesse de prazo ou compra, valia fazer a **2** antes
   porque "não desperdiça nada". **Desperdiça:** se a placa mudar, a dinâmica e
   a assimetria observadas no ensaio mudam junto, e ele tem de ser repetido. A
   parte geométrica da etapa 1 (trena, robô desligado) **não se perde em nenhum
   cenário**, e é o que fecha a decisão de maior alavancagem.

   🔴 **A etapa 2 só começa depois de três coisas**, e nenhuma é opcional:
   - **decisão da placa registrada** (§5.1 da revisão cruzada);
   - **parada física independente do Xbox confirmada** — já era pré-requisito de
     toda etapa energizada, e aqui é a primeira vez que morde;
   - **protocolo de medição pronto** (régua ou câmera): o bag do
     `bin/sobe-robo3` **não grava pose**, então sem isso não há desvio lateral
     medido, só impressão.
