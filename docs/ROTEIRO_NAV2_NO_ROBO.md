# Roteiro — Nav2 no robô, sessão de 12-08 (tarde/noite)

> Ordem de risco CRESCENTE. Cada passo tem comando exato, o que esperar, e o
> que fazer se der errado. **O dono só roda; os números voltam por CSV.**
>
> Objetivo da sessão, nas palavras do dono: *"quero testar o nav2 hoje, mesmo
> que com esse problema de ir reto na parede, pois se tudo der certo ele não
> deve bater e conseguir dar a ré para desviar"*.

---

## ⛔ PASSO 0 — ANTES DE QUALQUER DEPLOY (não pule)

**Os 8 CSV de 2,5 m de 11-08 ainda estão no NUC.** O deploy é
`git reset --hard`, e um `git clean -fd` os apaga para sempre.

```bash
# do PC de dev
scp -r <usuario>@<nuc>:~/Workspace/Controle_robo_livox/docs/dados/2026-08-11* \
    ~/Workspace/Controle_robo_livox/docs/dados/
```

Se a pasta não existir lá, procure por `*.csv` na raiz do repo do NUC:
```bash
ssh <usuario>@<nuc> 'ls -la ~/Workspace/Controle_robo_livox/*.csv \
                        ~/Workspace/Controle_robo_livox/docs/dados/ 2>/dev/null'
```

---

## PASSO 1 — Deploy 🔌 robô pode estar DESLIGADO

O trabalho de hoje está na branch **`slam-meu-mapa`** (17 commits à frente).

```bash
# no PC de dev
git push origin slam-meu-mapa

# no NUC
cd ~/Workspace/Controle_robo_livox
git fetch --all
git reset --hard origin/slam-meu-mapa
colcon build --packages-select robot_motion robot_base --symlink-install
source install/setup.bash
```

⚠️ **O `colcon build` NÃO é opcional hoje.** Mudaram YAML (`nav2.yaml`,
`twist_mux.yaml`, `movimentacao_sim.yaml`), uma árvore de comportamento nova e
a launch. Config vive em `install/share/`, e sem build o robô sobe com o de
ontem — e o log não avisa.

**Confirmação barata**: `grep limiar_pivo install/robot_motion/share/robot_motion/config/*.yaml`
tem de mostrar `3.20`.

---

## PASSO 2 — Pré-voo 🔌 robô LIGADO, mas NÃO anda (30 s)

```bash
ros2 launch robot_motion pilha.launch.py sim:=false mapa:=nenhum &
sleep 25
python3 tools/banco/checa_pilha.py --csv ~/dados/12-08-prevoo.csv \
        2>&1 | tee ~/dados/12-08-prevoo.txt
```

**Espera-se 20/20.** Se algum item falhar, o conserto está escrito na própria
linha — não improvise.

### 🆕 As três linhas NOVAS que este pré-voo tem de mostrar hoje

```
heading_controller:  "pivô por corte FORA DO CAMINHO (limiar_pivo=3.20 rad...)"
smoother_server:     "Smoother Server has suave simples savgol smoothers available."
lifecycle:           SEIS servidores active (entrou o smoother_server)
```

🔴 **Se o `smoother_server` não subir, o bringup inteiro aborta** — ele entrou
na lista do `lifecycle_manager`, e servidor da lista que não responde derruba
tudo (foi assim que o `collision_monitor` morreu em 06-08). Sintoma: os outros
cinco servidores também ficam `unconfigured`. Saída de emergência: reverter a
launch com `git revert f8820f0` no NUC e rebuildar.

---

## PASSO 3 — O freio de mão 🔌 robô LIGADO, rodas no ar ou espaço livre

**Nada de autonomia antes disto.** Em 06-08 o teleop não publicou NADA em
`/key_vel` — 44 s de CSV com zero amostras dessa fonte.

```bash
# terminal 1
ros2 launch robot_motion pilha.launch.py sim:=false mapa:=nenhum
# terminal 2 (ssh separado)
bin/robot-key
# terminal 3 — o prova_mux não recebe argumento; ele sobe o próprio mux
python3 tools/banco/prova_mux.py 2>&1 | tee ~/dados/12-08-mux.txt
```

Aperte as teclas de frente/ré/giro. O `robot-key` ganhou três contadores no
`rosout` que separam **"não lê a tecla"** de **"lê e não publica"** de
**"publica e ninguém escuta"**:

```bash
ros2 topic echo /rosout --field msg | grep -i "tecla\|key"
```

✅ **Critério**: com o teclado apertado, `/cmd_vel` tem de vir do teclado, não
da autonomia. Se falhar, **a sessão para aqui** — sem freio de mão não se testa
navegação.

---

## 🆕 PASSO 4 — O CANAL QUE FURA O REFLEXO 🔌 robô LIGADO, parado

**Este passo é novo e é o mais importante da sessão.** Hoje entrou um canal
(`unstuck_vel`, prioridade 30) que **passa por fora do `collision_monitor`**.
É a primeira vez que algo dirige este robô sem o freio de mão automático. Ele
só é seguro porque mede o vão de trás antes — e essa medida **nunca foi feita
com o `/scan` real**.

### 4a. A taxa do `/scan` real

O limiar `re_scan_velho_s = 0,8 s` foi calibrado contra o simulador (7,7 Hz,
p99 0,317 s, máx 0,513 s). O Livox real passa pelo `nuvem_pontos` e pelo
`scan_2d`, que é outra história.

```bash
ros2 topic hz /scan --window 100
```

Anote **média, mín e máx**. Se o pior intervalo passar de 0,8 s, a ré vai se
recusar a recuar dizendo *"perdi a medida do /scan"* — que é o lado **seguro**
do erro, mas inutiliza a recuperação. Nesse caso, subir `re_scan_velho_s` **só
depois de refazer a conta**: `re_scan_velho_s × v_piso` tem de continuar menor
que `re_folga` (0,30 m). Há teste travando esse par.

### 4b. O vão traseiro contra a TRENA

Ponha o robô com a traseira a uma distância **medida com trena** de uma parede
(sugestão: 1,00 m do para-choque). Depois repita a 0,40 m.

```bash
python3 - <<'EOF'
import rclpy, sys
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
sys.path.insert(0, 'ros2_packages/robot_motion/robot_motion')
from lei_de_seguimento import vao_no_corredor_traseiro
rclpy.init(); n = Node('vao')
def cb(m):
    v = vao_no_corredor_traseiro(m.ranges, m.angle_min, m.angle_increment,
                                 0.50, 0.28, alcance_max=m.range_max)
    print(f'vao traseiro medido: {v:.3f} m')
n.create_subscription(LaserScan, '/scan', cb,
    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
for _ in range(40): rclpy.spin_once(n, timeout_sec=0.2)
EOF
```

✅ **Critério**: o número tem de bater com a trena dentro de ~5 cm, **e tem de
ser 0,00 quando você põe uma caixa encostada atrás**. Se ele disser "livre" com
obstáculo atrás, **a sessão para aqui** — é exatamente a batida de 2026-06-11,
e agora o comando que se apoia nessa medida fura o reflexo.

⚠️ Teste também a **quina**: caixa no canto traseiro, fora do centro. O medidor
é retangular justamente para pegar isso.

---

## PASSO 5 — Navegação SEM MAPA 🔌 robô LIGADO, área livre, ~3 m

Objetivo curto, sem parede por perto. É a autonomia mais barata.

```bash
ros2 launch robot_motion pilha.launch.py sim:=false mapa:=nenhum
python3 tools/banco/corrida_nav.py --alvo 2.0 0.0 \
        --csv ~/dados/12-08-nav-livre.csv --teto-s 60
```

✅ **Critério (o mesmo que passou no Gazebo)**: `raw_v` não-nulo em **mais de
80%** das amostras e distância caindo de forma monótona.

🔴 **Se o robô girar no lugar sem sair**: o pivô voltou. Confira o `rosout` —
tem de dizer `pivô por corte FORA DO CAMINHO`. Se disser `pivô DISPONÍVEL`, o
build não pegou o `limiar_pivo` novo (volte ao passo 1).

---

## PASSO 6 — Navegação COM MAPA 🔌 robô LIGADO, na sala/corredor

### 🔴 A decisão que precisa ser tomada AQUI, e ela tem número

```
                          folga p10   corpo não cabe
scan_andar3_ajustado        0,400 m        7,5%
robot_radius configurado    0,32 m

e o que 12-08 mediu no meu_mapa, com a inflação da produção:
inflation_radius 0,50  ->  0 de 8 pontos de partida PLANEJAM
inflation_radius 0,20  ->  4 de 8
```

⚠️ O andar 3 é mais folgado que o `meu_mapa`, então talvez a inflação de 0,50
passe. **Mas ninguém mediu isso**, e 7,5% das células livres seguem apertadas
demais para o corpo. Trate como incerto e meça.

A `inflation_radius: 0,50` foi escolhida em 05-08 **para a porta de 0,90 m da
pista simulada** — é ela que centra o robô no vão. **No corredor real ela
proíbe planejar.**

Comece com a produção. Se o planner recusar (`Either of the start or goal pose
are an obstacle!` ou `failed to create plan`), baixe **editando o YAML e
rebuildando** — `ros2 param set` NÃO chega nos costmaps do jeito que se espera,
e não chega mesmo nos nossos nós:

```bash
# nos DOIS costmaps do config/nav2.yaml
sed -i 's/inflation_radius: 0.50/inflation_radius: 0.20/' \
    ros2_packages/robot_motion/config/nav2.yaml
colcon build --packages-select robot_motion --symlink-install
```

⚠️ **Anote qual valor você usou em cada corrida.** Sem isso os CSV não
comparam, e essa é a decisão que a próxima leva de dev tem de fechar.

```bash
ros2 launch robot_motion pilha.launch.py sim:=false \
        mapa:=$PWD/maps/andar3/scan_andar3_ajustado.yaml localizacao:=amcl \
        pose_x:=<x> pose_y:=<y> pose_yaw:=<yaw>
```

**O mapa é o do andar 3 do estágio**, copiado em 12-08 e escolhido por medida
(`maps/andar3/README.md`): `scan_andar3_ajustado` tem folga p10 de 0,400 m
contra 0,212 m do `scan_andar3` cru, e 7,5% de célula livre onde o corpo não
cabe contra 13,3%.

Se a sala do teste não estiver dentro dele (ele cobre 73,4 × 20,0 m com origem
em `[-28,672 · -15,476]`), o outro recorte está ao lado e é só trocar o
argumento:
```bash
        mapa:=$PWD/maps/andar3/mapa_3_andar.yaml
```

⚠️ **`pose_x/pose_y/pose_yaw` é onde o robô ESTÁ de verdade** no frame do mapa.
Errar aqui dá pose convergindo errado com cara de fatia 2D ruim. Se não souber,
use o RViz: **2D Pose Estimate** (a seta verde) — este é o botão certo *aqui*,
porque agora existe AMCL escutando.

Confira antes de mandar objetivo:
```bash
python3 tools/banco/casa_scan.py \
        --mapa $PWD/maps/andar3/scan_andar3_ajustado.yaml \
        --pose amcl --csv ~/dados/12-08-scan-mapa.csv
```

⚠️ `--pose amcl` e não o default `odom`: com AMCL de pé é a pose dele que vale,
e medir contra `odom` compararia a fatia 2D com uma pose que ninguém usa.
Espera-se algo perto do que 12-08 deu no simulador: **100% dentro de 0,15 m**.

---

## PASSO 7 — O TESTE QUE O DONO QUER 🔌 robô LIGADO, perto de parede

Objetivo que obrigue o robô a passar perto de uma parede ou por um vão.

```bash
python3 tools/banco/corrida_nav.py --alvo <x> <y> \
        --csv ~/dados/12-08-nav-parede.csv --teto-s 120
```

**O comportamento esperado, e ele é o de hoje no Gazebo** (não é o ideal, é o
medido):

1. ele vai em direção ao vão, **um pouco torto**;
2. o **reflexo para o robô antes de bater** (`Robot to stop due to PolygonStop
   polygon` no `rosout`);
3. o planner pode recusar (`Start occupied`) e o `bt_navigator` abortar;
4. o seguidor detecta emperramento, mede o vão de trás e **dá UMA ré de 0,30 m
   pelo canal que fura o reflexo**;
5. com o plano novo, ele passa.

✅ **Critério mínimo da sessão**: **ele não bate.** É o que o dono pediu.

🔵 **O critério que ele quer de verdade, e que NÃO vai passar hoje**: o reflexo
só deveria disparar por surpresa. Disparar contra parede mapeada é falha de
navegação, e está medido que ainda acontece (erro de trajeto p50 0,128 m contra
0,118 m de margem num vão de 0,90 m). **Isso é dívida conhecida, não surpresa.**

🔴 **PARE A SESSÃO se**: o robô encostar em qualquer coisa; a ré andar mais de
uma vez seguida sem melhorar; ou o robô andar de ré por mais de ~0,4 m. As três
têm defesa em código, e as três defesas nunca rodaram no robô real.

---

## O que trazer de volta

```bash
scp -r <usuario>@<nuc>:~/dados/12-08-* ./docs/dados/2026-08-12-robo-nav2/
```

E o `ambiente.txt` da praxe: piso, **bateria no início e no fim**, commit
(`git rev-parse --short HEAD`), e uma linha por corrida dizendo qual
`inflation_radius` estava valendo.

---

## Perguntas que esta sessão responde

1. O pivô saiu do caminho no robô real, e ele anda? (passo 5)
2. O `/scan` real é rápido o bastante para a ré não-cega? (passo 4a)
3. O medidor de vão traseiro bate com a trena, inclusive na quina? (passo 4b)
4. Dá para planejar no `scan_andar3_ajustado`, e com qual inflação? (passo 6)
5. O robô não bate? (passo 7)
