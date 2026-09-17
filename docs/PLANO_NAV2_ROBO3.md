# Plano — adaptar o Nav2 para o robô 3 (v2.1)

> **Status: nada implementado. Aguardando a aprovação do dono.**
> v1 em 2026-09-16; **v2 em 2026-09-17**, depois de uma revisão cruzada que
> derrubou duas etapas inteiras (§1); **v2.1 no mesmo dia**, com os cinco
> ajustes da segunda revisão já aplicados:
>
> 1. classificação de parâmetros separada em quatro classes (§5);
> 2. a fronteira `TwistStamped → Twist` passa a ser **decidida** na etapa 5, não
>    adiada (§4);
> 3. parada física virou **pré-requisito das etapas 2, 8, 9 e 10**, não só da 10;
> 4. a contradição do `ESTADO_PROJETO` corrigida — trocar o lado do puxão **não**
>    prova causa de canal;
> 5. validação do LIO na etapa 7 ampliada para a pose 6D inteira.
>
> Junto veio um achado próprio: consertar o teste da etapa 0 **não é** trocar
> `robot_radius` por footprint — isso reprovaria o robô 2. Ver §8.
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

São **quatro** classes, e misturá-las é o que fez a v1 mandar medir geometria com
CSV. Cada classe tem origem, dono e momento diferentes:

**(a) Planta medida — sai de ensaio com CSV, no robô.** `curv_frente`/`curv_re`,
`zona_morta`, `retencao_giro_s`, `linear_scale` e o ganho linear realizado,
ganho de giro, aceleração e frenagem reais, atraso liga/desliga.
→ São propriedades **da máquina e da placa**. Etapa 8.

**(b) Geometria — sai da trena e do URDF, NUNCA de CSV.** Footprint global e
local do Nav2, os **dois** polígonos do `collision_monitor`, alturas da
`VoxelLayer`, `scan_2d` (`min/max_height`, `range_min`), `laser_min_range` do
`localizacao_amcl.yaml`, **meia largura, corredor de ré, recuo do para-choque,
raio mínimo e folga de pivô** do `path_follower`.
→ Existem **antes** de qualquer simulação de navegação. Etapa 3.

**(c) Limites e política de segurança — escolha nossa, não medida.** `v_max`,
`wz_max`, largura de passagem e suas margens, `desvio_taxa_deg_s`, quando a ré é
permitida, o que o reflexo faz ao disparar.
→ Decisão registrada, com o "porquê". Limitada por (a), nunca maior que ela.

**(d) Sintonia do controlador — ajuste, dentro do que (a) e (c) permitem.**
`a_dec`, `a_lin`, `ganho_wz`, ganhos kp/ki do rumo e da reta.
→ Último a mexer, e só com corrida de controle antes e depois.

⚠️ A v1 punha meia largura, corredor de ré e recuo do para-choque em "calibração
por CSV". São **(b)**: saem da trena. E `desvio_taxa_deg_s` não é planta nem
geometria — é **(c)**, uma escolha de segurança.

🔴 **O footprint é geometria, não calibração** — ele tem de existir **antes** de
qualquer simulação de navegação, não no fim. A v1 o jogava para a etapa 6.

⚠️ **`range_min` e `laser_min_range` andam em par.** `range_min` vem da maior
geometria própria visível do Livox — rodas e varredura das bobas incluídas —,
não da largura de 24 cm da caixa; e mudá-lo sem mudar o `laser_min_range` deixa
o AMCL com um limite e a fatia com outro.

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
| 0 | Consertar `test_scan_2d.py` (vermelho por `robot_radius` da 032) — **contra a envolvente real, ver abaixo** | "suíte verde" volta a ser critério válido de etapa | não |
| 1 | Fechar **a placa** (§5.1 da revisão cruzada) e a geometria física autoritativa (trena) | sem isso a etapa 8 recomeça do zero | sim, desligado |
| 2 | Repetir o ensaio frente/ré com o protocolo do §7 | confirma ou derruba a premissa do §2 | sim, ligado |
| 3 | URDF completo girado (§3) + `robot_state_publisher` + footprints + testes reescritos | modelo e marcha concordam; o Nav2 passa a ter contorno | não |
| 4 | Perfis `robo2`/`robo3` e **um bringup único** do robô 3 | RSP + MEGA + `cmd_vel_to_wheels` + Xbox/direcional + mux único num lugar só | não |
| 5 | Unificar o contrato de mensagens (§4) e testar a cadeia **sem Gazebo** | comando atravessa de ponta a ponta, sem simulador para confundir | não |
| 6 | Ensinar a `pilha` a escolher `sim_robo3` (`robo:=3`, `use_sim_time`, qual atuador encerra) | a etapa 3 da v1, agora possível | não |
| 7 | Montar o Livox, medir a pose **6D** e validar o LIO por inteiro (abaixo) | a árvore de TF fecha com medida, não com chute | sim |
| 8 | Calibrar escala e dinâmica (§6); depois rumo e curvatura | os números do Nav2 passam a ter lastro físico | sim |
| 9 | Validar percepção e reflexo (`scan_2d`, `collision_monitor`) | o robô enxerga e freia antes de planejar | sim |
| 10 | Nav2 `mapa:=nenhum`, espaço livre, **parada física independente do Xbox** | objetivo curto | sim |

⚠️ A ordem mudou de verdade em relação à v1: **o Livox só sobe na etapa 7**, e
não na 4. Até lá o robô 2 continua navegando.

🔴 **A parada física independente do Xbox é PRÉ-REQUISITO de toda etapa com as
rodas no chão — 2, 8, 9 e 10**, não só da 10. O homem-morto do LB depende do
controle, do Bluetooth e da pilha ROS de pé; nenhum dos três é confiável durante
justamente os ensaios em que se está mexendo neles. A decisão 048 já registrou
a placa girando sozinha com a MEGA mandando zero.

**Etapa 0 — o que o teste passa a conferir, e por que não é substituição direta.**
`test_o_robo_nao_se_enxerga_como_parede` hoje lê `robot_radius` do `nav2.yaml` e
exige `range_min > robot_radius` (0,35 > 0,32). O `robot_radius` saiu na decisão
032. 🔴 **Mas trocar por "o footprint" reprova o robô 2**: o vértice do contorno
da 032 está a √(0,35² + 0,2775²) = **0,447 m**, contra `range_min` 0,35.

Não é defeito do `range_min` — é a grandeza errada. O que o robô vê de si mesmo
é o que está **dentro da fatia** (0,15–1,00 m), e não o contorno no chão (que
inclui para-choque e roda, abaixo da fatia). A meia-diagonal da caixa do robô 2
é √(0,2165² + 0,2275²) = **0,314 m** — exatamente o *"0,32 = 0,314 medido + 6 mm"*
que o `nav2.yaml` documenta. Era isso que o `robot_radius` representava ali.

➡️ O teste passa a conferir `range_min` contra a **envolvente própria dentro da
faixa de altura da fatia, derivada do URDF** (caixa + o que mais suba acima de
`min_height`), por robô. Sem número mágico solto e sem ressuscitar o
`robot_radius`. Para o robô 3 a caixa é 0,311 × 0,240 → meia-diagonal 0,196 m,
e é por isso que o `range_min` dele **pode e deve** baixar (§5b).

⚠️ E fica a pergunta que este conserto levanta e não responde: a envolvente da
fatia é a caixa, mas **o que mais do robô 2 sobe dos 0,15 m** (rodas a 0,16 m de
topo, cabos, o próprio suporte do Livox)? Se algo passar de 0,35 m em raio, o
`range_min` de hoje está apertado. Medir na etapa 9, não chutar agora.

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
4. **Quero fazer a etapa 0 agora?** É uma linha de teste, não toca o robô, e sem
   ela não dá para usar "suíte verde" como critério. Pendente de "pode".
