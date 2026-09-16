# Plano — adaptar o Nav2 para o robô 3

> **Status: PLANO, nada implementado.** Escrito em 2026-09-16 no PC de dev, robô
> desligado. Irmão do `PLANO_CONTROLE_ROBO3.md` (que fechou o teleop).
>
> Regra do projeto: cada etapa abaixo é uma mudança pequena, com o "pode" do
> dono antes de ir ao robô. Este documento não autoriza nada — ele ordena.

---

## 0. O que o dono já decidiu (16-09)

1. **A frente do robô 3 são as MOTORAS; a traseira são as BOBAS.**
   Eletricamente é o contrário, e a correção fica **no código**.
2. **O Livox é emprestado do robô 2**, montado **no centro do robô, no topo**.
3. O atuador se baseia no **robô 1** (MEGA); o seguidor e a navegação, no
   **robô 2** — que também é diferencial de 2 motoras. O robô 1 é 4x4 skid-steer,
   não faz curva, só pivô: a lei de movimento dele não serve aqui.

⚠️ Consequência de (2): **enquanto o robô 3 navega, o robô 2 não navega.** Só
há um Mid-360. Vale registrar como custo, não como detalhe.

---

## 1. Por que a frente nas motoras é a escolha certa (e não só gosto)

A decisão 009 e a `lei_de_rumo` dizem que, andando para trás, *"a boba deixa de
ser arrastada e passa a ser empurrada"* — e é aí que a direção fica instável.

No `robo3.urdf.xacro` de hoje as **motoras estão em x=0 e as duas bobas em
x ≈ +0,2485**: na marcha que o URDF chama de "frente", as bobas vão **na
frente, empurradas** — a configuração instável. Com a frente nas motoras elas
passam a ser **arrastadas**.

Isso casa com o que o robô vinha mostrando (`ESTADO_PROJETO`): **a ré andava
reto e a frente puxava para a direita**. A hipótese que este plano adota é que
a "ré" era o sentido mecanicamente estável. **Não está provado** — segue como
pergunta aberta do artigo, e a etapa 1 abaixo é o que a mede.

---

## 2. A fundação: girar o `base_link` do robô 3 em 180°

Hoje o URDF e o comando discordam: o comando já anda com a frente nas motoras
(`linear_sign: -1.0`, decisão 049), mas o **modelo** ainda acha que a frente é
o lado das bobas. Para o Nav2 isso é fatal: footprint, costmap, fatia 2D e
plano saem todos espelhados em relação à marcha real.

**Proposta:** girar o corpo 180° em torno de z no `robo3.urdf.xacro` — trocar o
sinal de x de `caixa_cx`, `boba_x` e `livox_x`. O `base_link` continua no centro
do eixo das motoras, no chão (convenção C8, que é boa e não se mexe: o centro de
rotação é a origem, e `wz` puro não desloca nada).

**E o `linear_sign: -1.0` FICA.** Ele é o adaptador elétrico — a placa entende
`speed>0` como o outro lado, e é exatamente isso que o dono mandou corrigir no
código. Bônus já provado na decisão 049: negar só a linear equivale a uma
rotação de 180°, **incluindo a troca de qual roda é a esquerda**. Ou seja: o
caminho de comando já está certo para o frame girado, sem renomear canal nenhum.

O que muda de concreto no contorno (o xacro já avisa):

| | hoje | depois do giro |
|---|---|---|
| da origem até a ponta da frente | 24,85 cm | **8,35 cm** |
| da origem até a ponta de trás | 8,35 cm | **24,85 cm** |

Nariz curto e corpo comprido atrás do centro de giro — **bom** para navegar em
lugar apertado, e muda o footprint inteiro.

---

## 3. A cadeia proposta

```
                    ┌── do ROBÔ 2 (navegação) ──────────────────┐
  /goal_pose → bt_navigator → planner_server → /plan
                                                 ↓
                                         path_follower
                                                 ↓
                                      heading_controller
                                                 ↓
                                        collision_monitor
                                                 ↓
                                           twist_mux  ← Xbox (humano fura)
                                                 ↓
                                        compensador_rumo
                    └───────────────────────────┬───────────────┘
                                                 ↓   ⬅ A COSTURA (§4)
                    ┌── do ROBÔ 1 (atuador) ─────┴───────────────┐
                              cmd_vel_to_wheels
                                                 ↓
                                  mega_bridge → MEGA → placa
                    └────────────────────────────────────────────┘

  Localização: Livox → FAST-LIO → /Odometry → tf_odom → odom→base_link
               Livox → scan_2d → /scan → AMCL (quando houver mapa)
```

Do robô 2 vem tudo em `robot_motion` (`path_follower`, `heading_controller`,
`compensador_rumo`, `lei_de_*`, `collision_monitor`) e a localização de
`robot_base` (`localizacao.launch.py` + `tf_odom` + `scan_2d`). Do robô 1 vem
o `cmd_vel_to_wheels` + `mega_bridge`, que já estão de pé e testados no teleop.

---

## 4. A costura, que é o único pedaço realmente novo

O robô 2 termina em `ros2_control`: o `compensador_rumo` publica
**`TwistStamped`** em **`/hoverboard_base_controller/cmd_vel`**.

O robô 3 não tem `ros2_control` no hardware: ele termina em **`Twist`** no
tópico **`cmd_vel`**, que o `cmd_vel_to_wheels` consome.

São duas diferenças: **tipo** (Stamped ou não) e **tópico**. Opções:

| opção | avaliação |
|---|---|
| parametrizar o tópico/tipo de saída do `compensador_rumo` | **preferida**: um nó, um parâmetro, sem nó novo no caminho crítico; o robô 2 não sente |
| nó adaptador `TwistStamped → Twist` no meio | mais um salto de latência numa malha que já tem 0,94 s de tempo morto no robô 2 — ruim |
| fazer o `cmd_vel_to_wheels` aceitar `TwistStamped` | espalha a decisão por dois pacotes |

⚠️ Conferir antes de escolher: `twist_mux_robo3.yaml` está em
`use_stamped: false`, e o YAML avisa que **se só um lado mudar o DDS recusa a
ligação por type hash e o robô não anda, sem erro nenhum**. Essa armadilha é a
primeira coisa a checar na etapa 3.

---

## 5. O que NÃO se herda do robô 2 (cada um é uma medida)

Herdar número medido de outra máquina é o defeito que este repo já pagou caro
(a bitola de 29-07). Todos abaixo são do robô 2 e **não valem** no robô 3:

| parâmetro | valor do robô 2 | por que não vale |
|---|---|---|
| `curv_frente` / `curv_re` | −0,8365 / −0,098 | é a curvatura parasita **daquela** máquina, e no robô 3 a frente acabou de trocar de lado |
| `zona_morta` | 0,0178 m/s | é da **placa**; a do robô 3 pode ser outra (§5.1 da revisão cruzada) |
| `retencao_giro_s` | 0,52 s | idem, é retenção de firmware |
| `a_dec` | 0,1 rad/s² | escolha conservadora amarrada ao S do robô 2 |
| `footprint` | `[[0.35,0.2775],…]` | outro corpo, e agora assimétrico ao contrário (§2) |
| `scan_2d` min/max height | 0,15 / 1,00 (sensor a 0,42 m) | **o sensor cai para 0,24 m** — ver §6 |
| bitola / raio | 0,270 / 0,080 | robô 3: **0,3225 / 0,0825** (já sabidos) |

---

## 6. A fatia 2D muda, e não é detalhe

O `scan_2d.yaml` é derivado da altura do sensor. Com o Livox no topo da caixa:

    topo da caixa = 0,070 (fundo) + 0,135 (caixa) = 0,205 m
    centro óptico ≈ 0,205 + 0,0325 = 0,2375 m  ≈ 0,24 m

🟢 O `livox_z_solo = 0,24` que já está no URDF como **chute** casa com a
montagem decidida. Sorte, mas conferir com trena antes de confiar.

Com 0,24 m em vez de 0,42 m, e o campo de −7° a +52°:

| | robô 2 (0,42 m) | robô 3 (0,24 m) |
|---|---|---|
| o chão entra no campo a | 3,4 m | **1,95 m** |
| o raio de +52° chega a 1,00 m a | 0,45 m | 0,59 m |

O `min_height: 0.15` continua cortando o chão, mas a **margem encolheu quase
pela metade**. E o `range_min: 0.35` era o raio do robô 2; no robô 3 o sensor
fica no centro da caixa (0,240 m de largura), então ele pode e deve baixar.
Os dois se recalculam, não se copiam.

---

## 7. Etapas propostas, na ordem, e o que cada uma prova

Cada etapa é uma sessão. Nenhuma começa sem a anterior fechada.

**Etapa 1 — provar a hipótese da frente (robô 3 como está hoje, sem Livox).**
Reta de ida e volta pelo `dpad_reto`, com o bag de sempre: `frente:=-1.0`
(padrão de hoje) contra `frente:=1.0`, 3x cada, mesma bateria. Mede-se o desvio
lateral nos dois sentidos.
→ Se o sentido das motoras à frente anda reto e o outro puxa, §1 está provado e
o resto do plano fica de pé. **Se o puxão só trocar de lado, a causa é de canal
e não de sentido**, e aí o plano muda: o conserto é elétrico, antes do Nav2.
*(Não precisa do Livox. Dá para fazer já, é a etapa mais barata.)*

**Etapa 2 — o giro de 180° no URDF (§2), sem robô.**
Xacro + `test_urdf_robo3.py` (o teste que trava URDF e YAML em par). Conferir no
RViz que o eixo x aponta para as motoras e que as bobas ficam atrás.
→ Prova que o modelo e a marcha concordam. Nada vai ao robô.

**Etapa 3 — a costura (§4) no simulador.**
`sim_robo3.launch.py` já tem Gazebo, lidar simulado, `/Odometry` de verdade de
chão e `scan_2d`. Subir a pilha do robô 2 por cima dele e fechar o caminho até o
`cmd_vel_to_wheels`. Aqui se pega a armadilha do `use_stamped`.
→ Prova a cadeia inteira sem gastar bateria nem o Livox do robô 2.
⚠️ Gazebo só com o dono olhando.

**Etapa 4 — montar o Livox no robô 3 e medir.**
Trena: altura do centro óptico, x/y, e se está torto em yaw. Atualiza o URDF com
a **medida**, não com o chute. Recalcula o `scan_2d` (§6).
→ A partir daqui o robô 2 está sem sensor.

**Etapa 5 — localização de pé no robô 3.**
`localizacao.launch.py` + `tf_odom` + `scan_2d`. Pré-voo: `/Odometry` a 10 Hz,
`/scan` vivo, e `odom → base_link` respondendo no `tf2_echo`.
→ Prova que o robô sabe onde está. Sem isso o Nav2 nem ativa.

**Etapa 6 — calibrar o que não se herda (§5), na ordem de alavancagem.**
Zona morta e retenção primeiro (são da placa), depois `curv_frente`/`curv_re` na
frente nova, depois o footprint.
→ Cada um é um ensaio com CSV, como os de agosto.

**Etapa 7 — Nav2 no robô, `mapa:=nenhum`, espaço livre.**
Só então objetivo curto, com a mão no Xbox (o humano fura o reflexo).

---

## 8. Opiniões e dúvidas em aberto (para o dono)

1. **Eu faria a etapa 1 antes de qualquer outra coisa** — ela é barata, não
   precisa do Livox, e é a única que pode derrubar a premissa do plano inteiro.
   Se o defeito for de canal, gastamos o Livox do robô 2 à toa.
2. **Pacote novo ou reuso?** Minha opinião: **reusar** `robot_motion` e
   `robot_base` com um argumento `robo:=3`, em vez de duplicar. Duplicar é o que
   criou `sim_robo3.launch.py` e `hoverboard_controllers_sim_robo3.yaml`
   separados — defensável lá (o robô 2 é a linha de base histórica), mas caro se
   virar regra: toda correção passa a ter de ser aplicada duas vezes.
3. **A placa do robô 3 está decidida?** A §5.1 da revisão cruzada diz que é o
   item de maior alavancagem: se a placa mudar, zona morta, patamar e latência
   caem junto, e a etapa 6 recomeça. Vale resolver antes da etapa 6.
4. **O robô 3 tem IMU (MPU6050) e optical flow pela MEGA.** O robô 2 não usa —
   ele localiza só por LIO. Proponho começar igual ao robô 2 (só LIO) e deixar a
   fusão para depois: um sensor a mais sem necessidade é um modo de falha a mais.
5. **`pose_estimator` fica de fora?** É o que digo em (4): com LIO, a odometria
   de roda do robô 1 não entra. Se entrar um dia, atenção — ela tem
   `left/right_wheel_sign` mas **não** tem `linear_sign`, então o feedback não
   segue o giro de 180° sozinho.
