# Próxima sessão no robô — roteiro de operação

> **Para quem é este documento: o assistente que chegar frio.** A memória dele
> não cruza sessões nem máquinas — é por isso que este arquivo existe, e é por
> isso que ele começa pelo método e não pelos comandos.
>
> **Reescrito em 2026-08-06**, depois da sessão de bancada daquele dia (18
> corridas). A versão anterior mandava rodar experimentos que **já foram
> feitos** — se você está lendo uma cópia que pede varredura de ganhos ou pivô
> `liga 0,10`, ela está velha: esses fecharam. Tudo aqui tem origem citada.

---

## 0. O MÉTODO — leia antes dos comandos

O dono elogiou explicitamente o modo de conduzir de 05-08 e pediu que se repita:

> *"direto aos testes, sem se enrolar, só fazendo o que foi pedido, avisando
> pra desligar o robô, pedindo 'pode' para soltar o comando"*

1. **O DONO SÓ RODA.** Nunca peça para ele relatar console. Tudo grava CSV ou
   vai para o `rosout`, que se lê por ssh. Ele executa o físico: posicionar,
   ligar, olhar, apertar tecla.
2. **UMA CORRIDA POR VEZ.** Antes de cada comando que move: dizer o que o robô
   vai fazer, quanto espaço precisa, e **esperar o "pode"**.
3. **NÚMERO NA HORA**, com o robô ligado. Não acumular CSV para ler depois.
4. **RESPOSTA CURTA.** Em 06-08 o dono cortou com *"responda simples"* depois de
   blocos longos entre uma corrida e outra. Durante a bancada: o número e o
   veredito, 2–4 linhas. Tabela e ressalva vão para o commit e o `DIARIO`.
5. **AVISAR LIGADO vs DESLIGADO**, e avisar quando já pode desligar.
6. **NÃO EXPANDIR ESCOPO** com a bateria correndo. Precisa escrever ferramenta?
   Escreva com o robô desligado — foi assim que o `homem_morto.py` nasceu.
7. **RETIRAR HIPÓTESE ASSIM QUE O DADO A DERRUBAR**, em voz alta.

⚠️ **Bateria é o recurso escasso.** Em 06-08 foram 41,16 → 40,92 V em 18
corridas — dá folga, mas conversa é o que come tempo, não corrida.

---

## 0.5 ✅ O ENSAIO DO ESTIMADOR JÁ RODOU (11-08 tarde) — não repita

`docs/ROTEIRO_ESTIMADOR_DO_FF.md` **está cumprido**. Veredito: o estimador
**não está provado** (o A-B-A mostrou que a melhora era ordem temporal), e a
configuração adotada é compensador com ff do dia e **`adapta` desligado**. Ver
o veredito no fim da decisão 016.

🔴 **AS DUAS PRIMEIRAS COISAS DA PRÓXIMA SESSÃO, nesta ordem:**

1. **Puxar os 8 CSV que ficaram no NUC** — o robô desligou antes do `scp`. Eles
   estão em `~/Controle_robo_livox/docs/dados/2026-08-11-estimador/`, não são
   rastreados pelo git (o `reset --hard` não os toca, mas **`git clean -fd`
   apaga**). Puxar ANTES de qualquer deploy:

   ```bash
   scp 'bara@<ip>:~/Controle_robo_livox/docs/dados/2026-08-11-estimador/*.csv' \
       docs/dados/2026-08-11-estimador/
   ```

2. **Três corridas de 2,5 m com o robô FRIO**, mesma configuração de 11-08
   (`curv_frente:=-0.8275`, sem `adapta`, `--topico /compensador_rumo/cmd_vel`).
   É a previsão barata que fecha o dia de ontem: **~6° confirma que a melhora é
   do controlador; 18–21° significa que ela era térmica** e o rumo não está
   resolvido. Julgar com `mede_o_s.py`, nunca com curvatura média.

Volte para este documento aqui para o teste C, o teste D ou a percepção.

## 1. Contexto mínimo

| ler | por quê |
|---|---|
| `ESTADO_PROJETO.md`, o topo | o que 06-08 provou e o que derrubou |
| `docs/decisoes/013-o-ff-fixo-nao-fecha-o-arco.md` | o experimento nº 2 desta sessão |
| `docs/DIARIO.md`, 06-08 (3ª leva) | por que os testes C e D falharam |

**O que mudou desde a última vez que o robô rodou, e nunca foi visto por ele:**

- existe um nó novo, **`tf_odom`**, que publica a TF `odom → base_link` — a que
  faltava e que derrubou o teste D. Sobe junto na `localizacao.launch.py`;
- o **`bin/robot-key` foi consertado** (`set -u` brigava com os `setup.bash`);
- o **teleop ganhou diagnóstico** por `rosout` a cada 2 s;
- 🆕 **os dois costmaps do Nav2 passaram a consumir a nuvem do Livox**
  (decisão 014, 07-08). Antes só o `collision_monitor` a consumia: o robô não
  desviava de obstáculo novo, **parava** na frente dele. Duas consequências para
  quem estiver na bancada:
  - o **teste D muda de figura** — com o obstáculo agora no costmap, o Nav2 pode
    replanejar em vez de deixar o reflexo agir sozinho. O teste continua sendo
    do reflexo; só não se assuste se o plano mudar;
  - ⚠️ **no global costmap a marcação é PERMANENTE** (não há janela rolante), e
    a TF `map→odom` é fixa e provisória. Deriva do LIO vira **obstáculo
    fantasma acumulado**. Se o planejador começar a recusar caminho que estava
    livre, limpe antes de investigar:
    `ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap`;
- 🆕 **o ff do dia entra por argumento de launch** (decisão 013, caminho 3,
  implementado na 4ª leva de 07-08): `pilha.launch.py` aceita `curv_frente:=`,
  `curv_re:=` e `curv_medido_em:=`, e o compensador **anuncia no `rosout`** se o
  feedforward foi medido hoje ou é herdado. O default é `HERDADO` — pilha que
  sobe sem a medida do dia se denuncia sozinha. Ver §5;
- 🆕 **08-10 RODOU, e mudou três coisas** (leia a entrada 08-10 do diário antes
  de qualquer corrida):
  - **a régua de aceitação mudou**: corrida de 1,2 m **não serve** para julgar
    rumo. A mesma corrida mede −0,0162 (passa a 011) cortada em 1,2 m e +0,0882
    (reprova) medida em 2,5 m — o corte curto cai no cruzamento de zero do S.
    Aceitação agora é `--espaco 2.5` (~10 s), pelo menos um período;
  - **a percepção está morta no robô e não é config**: `/livox/lidar` sai em
    `livox_ros_driver2/msg/CustomMsg` e os costmaps assinam `PointCloud2`. Não
    adianta mexer em faixa de altura antes de existir conversor — e, quando
    existir, há um segundo defeito em série (o `odom` fica na altura do sensor,
    `z = −0,477 m`, e a faixa 0,10–0,50 rejeita tudo);
  - **o ff do dia (caminho 3 da 013) é piso, não solução**: melhora a excursão
    2,7× e o rumo continua sem assentar. Ver o adendo da decisão 013.
- **o resto não rodou no robô.** É a primeira coisa a conferir.

---

## 2. Acesso e deploy

```bash
# 1) achar o robô. Em 06-08 foi 10.244.3.205, e o IP MUDA.
ip -4 addr show | grep "inet "         # veja em que /24 você está
ping -c2 <ip-do-nuc>
ssh -o ServerAliveInterval=15 bara@<ip-do-nuc>
```

```bash
# 2) deploy — o NUC NÃO tem auth no GitHub, então vai por bundle
git bundle create /tmp/repo.bundle --all          # no dev
scp /tmp/repo.bundle bara@<ip>:/tmp/
#    no NUC:
cd ~/Controle_robo_livox
git fetch /tmp/repo.bundle main && git reset --hard FETCH_HEAD
source /opt/ros/jazzy/setup.bash
colcon build --base-paths ros2_packages --symlink-install \
    --packages-select hoverboard_driver robot_base robot_motion
source install/setup.bash
```

⚠️ **`colcon build` SEMPRE.** O `reset --hard` troca o fonte e **não** o
`install/`, e é de lá que a descrição é lida.

✅ **O `twist_mux` já está compilado no NUC** desde 06-08 (`./setup_twist_mux.sh`
não precisa de novo, a menos que o `install/` tenha sido apagado).

⚠️ **O NUC CAI JUNTO COM O ROBÔ.** Parece ter alimentação separada; não tem. Em
06-08 um deploy morreu no meio (`No route to host`) e ele voltou com `up 0 min`.
Salve em levas para o `origin`. E **`/tmp/logs` não sobrevive ao reboot** —
`mkdir -p /tmp/logs` antes de qualquer `nohup`, senão o launch falha calado.

---

## 3. 🔴 PASSO BLOQUEANTE

**Robô LIGADO.** Terminal 1, vivo a sessão inteira:

```bash
mkdir -p /tmp/logs
ros2 launch robot_base base.launch.py
```

Terminal 2:

```bash
python3 tools/banco/sessao.py --checar
```

**TEM de imprimir** `wheel_separation = 0.2700`. Passou em 06-08, mas
**reconferir**: o NUC reinicia junto com o robô. Bitola errada enviesa tudo sem
sintoma.

E a conferência de sempre — exatamente UMA pilha:

```bash
pgrep -a -f "[f]astlio_mapping"          # UM processo
pgrep -a -f "[l]ivox_ros_driver2_node"   # idem
```

⚠️ **`pgrep -a`, nunca `-c`**, e **leia quem casou**: o colchete protege contra
o `pgrep`, não contra a própria linha de comando do ssh. Isso mordeu duas vezes,
a última em 06-08 com `teleop_teclado`. Quando houver dúvida, `ros2 node list`.

---

## 4. EXPERIMENTO 1 — o PRÉ-VOO (30 s, não move o robô)

**Um comando responde por tudo o que foi escrito sem robô**, incluindo o
`tf_odom`, a camada de obstáculo (014) e o perfil sem mapa (015):

```bash
D=docs/dados/$(date +%Y-%m-%d)-sessao && mkdir -p $D
python3 tools/banco/checa_pilha.py --csv $D/preflight.csv
```

Ele confere, item a item, com o número que produziu o veredito e o conserto
escrito na própria linha quando falha: uma pilha só (lendo `ps`, não `pgrep
-c`), nuvem e taxa, TF `odom → base_link`, TF `base_link → livox_frame` contra
os 0,42 m da trena, fração de nuvem transformável, `lifecycle` dos quatro
servidores, perfil sem mapa, os dois costmaps marcando, e a cadeia de comando
inteira.

**Leia de cima para baixo e conserte o PRIMEIRO ❌** — ele costuma explicar os
de baixo.

⚠️ **Uma coisa ele não faz sozinho: empurrar o robô.** A TF `odom → base_link`
parada só prova que existe. Rode, **empurre o robô com a mão**, rode de novo: a
translação tem de mudar. Se não mudar, o LIO não está seguindo o corpo.

🔴 **O `tf_odom` VAI falhar na subida, e o conserto está medido (08-10).** Ele
reclama `NÃO EXISTE TF body -> base_link` — e a resposta **não** é passar o
frame que ele citou: `body` é o frame da IMU do FAST-LIO, que não está no URDF.
O certo é dizer a que link do URDF aquela pose corresponde:

```bash
ros2 run robot_base tf_odom --ros-args -p frame_da_pose:=livox_frame
```

Sem isso a árvore TF fica partida e **os quatro servidores do Nav2 não ativam**
(foi o que matou 06-08). Com isso, em 08-10, os quatro subiram `active` pela
primeira vez no robô real e o pré-voo foi de 10/19 para 16/19.
⚠️ Dívida conhecida: `body` fica 5 cm do lidar (`extrinsic_T` do `mid360.yaml`).

⚠️ `/auto_vel` calado com o robô parado **é o certo**: o `collision_monitor` não
republica comando nulo (medido 07-08), e parado o `heading_controller` só
publica zero. O pré-voo já sabe disso e marca ⚠️, não ❌.

⚠️ Repare em **qual** mensagem o nó dá ao subir: ele diz se compôs com o URDF ou
se a pose já era do corpo. Se disser "já era o corpo" mas o `child_frame_id` for
do sensor, a TF sai 42 cm errada **em silêncio** — é o modo de falha que o nó
foi escrito para evitar.

---

## 5. EXPERIMENTO 2 — as três corridas SEM compensador (a medida que falta)

Isto é o que fecha a **decisão 013**, e é barato. Só existe curvatura crua de
04-08 (−0,8031) e 05-08 (−0,9116): dois pontos, 13,5% de diferença, faixas que
não se tocam. **O terceiro ponto caracteriza a tendência em vez de inferi-la.**

🛑 **SEM compensador rodando.** Se ele estiver de pé, mate. O resíduo com a
malha fechada **não mede** o `ff` — o integrador come parte dele (medido: em
05-08 o esperado era −0,095 e o medido foi +0,0417, com o sinal invertido).

```bash
D=docs/dados/$(date +%Y-%m-%d)-curva-crua && mkdir -p $D
python3 tools/banco/ensaio.py --ensaio reta --v 0.25 --wz 0 \
    --dur 12 --espaco 1.2 --janela 0.5 --csv $D/cru-frente-a.csv
```

Repetir `-b` e `-c`, do mesmo ponto 0 e mesmo rumo. Leitura na hora:

```bash
python3 tools/banco/medir.py --resumo curvatura $D/cru-frente-*.csv
```

⚠️ **`--janela 0.5` sempre no robô.** O padrão de 0,2 s sobre pose a 10 Hz
inventa ruído.

**O robô arca com raio de ~1,2 m indo para a frente** — sem compensador ele
descreve um círculo. Espaço em TODAS as direções, não só à frente; em 04-08 ele
bateu numa cadeira num corredor "de 3 m livres".

**Como julgar:** o número em si é o produto. Se cair perto de −0,80, a planta
está como em 04-08; perto de −0,91, como em 05-08; no meio, a tendência é
contínua e o caminho 3 da decisão 013 (medir no início de cada sessão) fica bem
justificado.

### 🆕 E o número agora TEM PARA ONDE IR (07-08, 4ª leva)

O `--resumo` termina imprimindo a linha de launch pronta, com a média e a data
de hoje já preenchidas — é só colar:

```
  ➡️ o ff do dia (decisão 013) — subir a pilha com:
     ros2 launch robot_motion pilha.launch.py mapa:=nenhum \
         curv_frente:=-0.9000 curv_medido_em:=2026-08-07
```

⚠️ **Se a linha NÃO sair, o número não serve como ff** e ela diz por quê:
menos de três corridas válidas, dispersão acima de 5% (dentro do dia este robô
repete em 2–3%) ou frente misturada com ré no glob. Nesses casos, repetir a
condição antes de usar — não colar a média assim mesmo.

✅ **A conferência de que o valor chegou** é uma linha do `rosout`, e ela vale
por todo o resto:

```bash
ros2 topic echo /rosout --field msg | grep -i "^ff "
```

```
ff MEDIDO em 2026-08-07: curv_frente -0.9116 1/m   -> o número do dia entrou
⚠️ ff HERDADO: curv_frente -0.8170 NÃO foi medido  -> subiu com o valor velho
```

`HERDADO` é o default do nó de propósito: pilha que sobe sem o ff do dia tem de
se denunciar, senão a bancada mede um robô que não existe (o defeito da bitola,
29-07). **Todo experimento com compensador daqui para a frente roda depois
desta conferência.**

---

## 6. EXPERIMENTO 3 — teste D, o reflexo de colisão

**Só se o experimento 1 tiver passado.** Sem TF o `collision_monitor` recebe e
não publica nem zero — foi assim que ele morreu em 06-08.

**Caixa de 50 cm**, não 40: o Mid-360 está a 42 cm e a 0,5 m só vê acima de
36 cm. Cadeira serve mas é pior alvo (vazada, pernas finas).

```bash
# o curv_frente é o que o experimento 2 acabou de medir (§5) — a linha sai
# pronta do `medir.py --resumo curvatura`
ros2 launch robot_motion pilha.launch.py mapa:=nenhum \
    curv_frente:=<a média de hoje> curv_medido_em:=$(date +%F)
ros2 lifecycle get /collision_monitor        # TEM de dizer active
```

🔴 **`mapa:=nenhum` é obrigatório no robô** (decisão 015, 07-08). Sem isso a
pilha sobe o mapa da pista SIMULADA — parede onde não há nada, livre onde há
parede — e desde a 014 mistura isso com marcação real do Livox. Com `nenhum`
não há `map_server`: o costmap global é uma janela de 20 × 20 m feita só do que
o sensor vê.

🛑 **Se disser `inactive`, o `lifecycle_manager` abortou o bringup** (o Nav2 não
ativa sem `map → base_link`). Ativar na mão:

```bash
ros2 lifecycle set /collision_monitor activate
```

⚠️ **Mate o `heading_controller` antes de comandar**: ele publica zero em
`/auto_vel_raw` e disputa o tópico com o comando de teste. Matar **por PID**.

```bash
ros2 topic pub -r 10 /auto_vel_raw geometry_msgs/msg/TwistStamped \
    "{twist: {linear: {x: 0.25}}}"
```

**Dimensione o tempo pela distância**, e com margem: a 0,25 m/s são 0,75 m em
3 s. Com a caixa a ~1,1 m, 3 s **não bate** mesmo se o reflexo falhar — foi assim
que se testou com segurança em 06-08. A zona de parada é `x ∈ [−0,28, +0,49]`,
então ele deve andar ~60 cm e parar.

**Esperado:** para com ~0,27 m de folga entre o para-choque e a caixa.

---

## 7. EXPERIMENTO 4 — teste C, o homem-morto

⚠️ **É o único teste que o DONO opera** — precisa de teclado interativo, e o
assistente não consegue digitar num ssh dele.

```bash
# terminal do dono:
cd ~/Controle_robo_livox && bin/robot-key
```

Em 06-08 o nó subiu, imprimiu, e **nada saiu em `/key_vel`** (44 s de gravação,
zero amostras dessa fonte). Agora ele se explica sozinho, a cada 2 s no
`rosout` — **leia por ssh, não peça para o dono relatar**:

```bash
ros2 topic echo /rosout --field msg | grep -i "publicadas"
```

```
publicadas=0                     -> o timer não roda; o nó subiu quebrado
publicadas>0  teclas_lidas=0     -> publica, mas não lê o teclado (le_tecla)
teclas_lidas>0  ouvintes=0       -> lê e publica, mas o twist_mux não assina
tudo >0 e o robô não anda        -> o problema é da cadeia para baixo
```

Com isso funcionando, o gravador mede os três intervalos:

```bash
python3 tools/banco/homem_morto.py --csv $D/homem-morto-a.csv
```

Protocolo: aperta `w`, segura ~2 s, **solta**, não toca em nada por ~3 s. Uma
solta por corrida. `[A]` deve dar **0,40–0,45 s** (a régua tem viés de +0,05 s
porque o teleop publica a 20 Hz).

⚠️ **Se ele não parar sozinho, PARE TUDO.** Teleop que não para é pior que
teleop nenhum.

---

## 8. Se sobrar

- **vídeo da traseira** durante uma corrida de frente — o item mais barato que
  falta no projeto. Deixou de ser urgente (o S era do laço, não da boba), mas
  segue valendo;
- **desempate do preditor de Smith** — precisa de **corredor longo**, que a sala
  não dá. Sustentada × assenta só aparece em corrida longa.

---

## 9. Como o dado volta

```bash
scp 'bara@<ip>:~/Controle_robo_livox/docs/dados/AAAA-MM-DD-*/*.csv' docs/dados/...
git add docs/dados/ && git commit -m "..." && git push
```

**Salve durante a sessão, não no fim** — em 06-08 o NUC caiu no meio e nada se
perdeu por causa disso.

**Escreva o `ambiente.txt`**: piso, bateria (início E fim), commit, e a
calibração viva que o `--checar` imprimiu.

---

## 10. Ao terminar

- avisar que **o robô já pode ser desligado**;
- `docs/DIARIO.md`: entrada da sessão, **incluindo o que falhou**;
- `ESTADO_PROJETO.md`: atualizar o topo e a lista da próxima ida;
- **atualizar ESTE arquivo** — a versão anterior mandou refazer trabalho pronto.

---

## Apêndice — armadilhas que já custaram tempo neste projeto

| armadilha | como evitar |
|---|---|
| `pgrep -c` contando a si mesmo | `pgrep -a` e LER quem casou; na dúvida, `ros2 node list` |
| `pkill` matando a própria sessão ssh | matar **por PID** |
| matar os nós e deixar o `ros2 launch` vivo | ele **não** morre com os filhos; em 07-08 isso empilhou 3 pilhas e o sintoma foi bringup abortando com cara de bug. Matar o launch também, e conferir com `ps` |
| `ros2 node list` mostrando nó que já morreu | é cache do **daemon**; `ros2 daemon stop && ros2 daemon start`. Quem está vivo de verdade, `ps` responde |
| `ros2 param set` que não chega no nó | matar e subir; conferir a primeira linha do log |
| `git reset --hard` sem `colcon build` | build sempre; a descrição vem do `install/` |
| `/tmp/logs` sumindo no reboot | `mkdir -p` antes do `nohup`, senão o launch morre calado |
| trava `--espaco` é RADIAL | dimensionar pelo **disco varrido**, 1,0–1,2 |
| `--janela 0.2` no robô | sempre `0.5` |
| comparar n=1 com n=1 num processo bimodal | n=3, e **faixa** contra faixa |
| órfãos republicando `/Odometry` | matar por PID e conferir **1 publicador** |
| dispersão em % sobre número quase nulo | ler o espalho **absoluto** também |
| calibrar `ff` com o compensador ligado | o integrador come o resíduo — medir **sem** ele |
| achar que um nó lifecycle está de pé porque aparece no `node list` | `ros2 lifecycle get` |
