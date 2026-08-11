# Estado do Projeto — Controle_robo_livox (PIBIT)

> Documento vivo. Resumo do que está acontecendo, BOs abertos, avanços e o que falta.
> Versionado na `main`. Atualizado em **2026-08-11**.
>
> **Este projeto é um PIBIT** — vai virar artigo. Toda decisão técnica tem um
> registro em `docs/decisoes/`, todo dia de trabalho entra no `docs/DIARIO.md`,
> e escolhas de abordagem são embasadas em literatura (`docs/REFERENCIAS.md`).
> Ritmo deliberadamente devagar: 1 mudança pequena por vez.

---

## 👁️ 11-08 (2ª leva) — A PERCEPÇÃO SAI DO PAPEL: dois defeitos em série (dev)

Decisões **017** e **018**, as duas do pré-voo de 10-08. **327 testes verdes**
(eram 311).

🔴 **017 — o contrato da nuvem era um NOME, não um tipo.** `/livox/lidar` sai do
driver como `CustomMsg`; costmaps, reflexo e pré-voo assinavam `PointCloud2` e
**nunca receberam nada**. No simulador o `gpu_lidar` publicava PointCloud2 no
mesmo nome — a 014 nasceu inerte e ninguém viu.

```
/livox/lidar    CRU (robô: CustomMsg · simulador: PointCloud2)
/livox/pontos   PointCloud2 SEMPRE — o que a percepção consome
```

No robô converte o nó novo `nuvem_pontos` (sobe na `localizacao.launch.py`); no
simulador a ponte publica direto. ⚠️ O ponto `(0,0,0)` do Mid-360 (raio que não
volta) é **descartado**: no frame do sensor ele é o próprio robô, e marcaria
célula letal em cima dele a cada quadro.

⚠️ **Nenhum dos muitos testes de config pegou isto** — todos conferiam que os
consumidores concordavam ENTRE SI. Faltava perguntar *quem publica, e em que
tipo*. Três testes novos fecham o buraco.

🔴 **018 — o `odom` estava na altura do sensor** (o `z = −0,477` de 10-08). Com
o chão em z ≈ −0,42, a percepção do Nav2 rejeita a nuvem em dois lugares
independentes (faixa de altura e `origin_z` da `VoxelLayer`, no frame global).
Passa a pré-compor com a inversa: **`odom` é a pose do `base_link` na largada**.

⏳ **Previsão que separa as duas**: a 017 sozinha **não** faz os costmaps
marcarem. Se marcarem só com ela, a hipótese da altura está errada e a suspeita
seguinte é a `VoxelLayer`, não a altura.

---

## 🧠 11-08 — O ff DEIXA DE SER UM NÚMERO E VIRA UMA ESTIMATIVA (dev, sem robô)

Decisão **016**, que implementa o caminho 2 da 013 — o que o dono pediu na
bancada de 10-08 (*"o compensador deve conseguir identificar o erro atual"*).

🟢 **O integrador já identificava o erro; o que estava errado era a unidade e a
memória.** Ele é rumo acumulado [rad·s], vive num laço com 0,94 s de tempo morto
e é zerado a cada parada. A mudança é de **escala de tempo**: o que ele segura
em regime é drenado devagar para a `curv_*` [1/m], que sobrevive à parada.

```
transf = integral · dt / adapta_t      Δcurv = −(ki·transf)/v_real
integral −= transf                     (adapta_t = 8 s, valor de partida)
```

⚠️ **Sem solavanco por construção**: o ff cresce exatamente o que o termo
integral encolhe. Degrau de comando num laço com 0,94 s de tempo morto é como se
fabrica a oscilação que o estimador veio matar.

**Opt-in** (`-p adapta:=true`), grampeado a ±0,5 1/m da semente, não estima
abaixo de 0,05 m/s, e o nó publica `curv_hat` no `rosout` a cada 2 s — a
diferença entre "aprendeu" e "encostou no grampo" não aparece no comportamento.

⏳ **NADA DISTO FOI AO ROBÔ.** Previsão falsificável: três corridas de 2,5 m
seguidas, com ff velho de propósito — a corrida 2 tem de ser MENOR que a 1, e a
3 menor que a 2. Se a 2 empatar com a 1, o mecanismo não está agindo.

⚠️ **A planta de brinquedo não arbitra isto**: ela dá 4,1° de pico onde o robô
fez 13–22°, e erra o sobrepasso por 3,7×. Os testes provam o MECANISMO
(converge, é lento, não dá solavanco, grampeia, sobrevive à parada).

**311 testes verdes** (eram 300), cinco novos verificados por mutação.

🔧 **Duas armadilhas de ferramenta, pagas caro nesta sessão** (detalhe no
diário): desfazer mutação com `git checkout --` **apaga trabalho não commitado**
— use cópia de segurança; e mutação do mesmo tamanho em bytes restaurada no
mesmo segundo deixa o `.pyc` velho valendo (Python valida por mtime + tamanho),
fazendo a suíte reprovar um arquivo correto. **Limpar `__pycache__` entre
rodadas de mutação.**

---

## 📏 10-08 — O S ESTAVA LÁ O TEMPO TODO; A RÉGUA É QUE ERA CURTA (robô)

Doze corridas na sala. Dados em `docs/dados/2026-08-10-*`; entrada 08-10 do
diário. **Bateria não foi lida** — pedida duas vezes, a sessão andou sem ela.

🔴 **O ACHADO: a mesma corrida passa ou reprova conforme o tamanho da régua.**

```
comp-longa-a.csv   medida em 1,2 m  ->  −0,0162   PASSA no critério da 011
                   medida em 2,5 m  ->  +0,0882   REPROVA
```

Mesmo robô, mesma corrida, mesmo `medir.py`. O corte de 1,2 m cai no cruzamento
de zero do S: o rumo vai a −13,5°, volta, e de ponta a ponta dá quase reto. Em
2,5 m aparece o ciclo inteiro — **envoltória crescendo 1,65×**, meio-período
~5,3 s.

➡️ **MUDANÇA DE PROTOCOLO: corrida de aceitação tem de durar ao menos um
período (~10 s / 2,5 m).** Isso reclassifica o `+0,0417` que deu "aceito" em
05-08: ele foi medido com a régua curta. A releitura de 06-08 suspeitava; agora
está provado com a mesma corrida medida das duas formas.

🔴 **A PLANTA DERIVA 19% EM 5,4 MINUTOS** (seis retas cruas idênticas, mesmo
ponto e mesmo rumo):

```
+0,0 min 0,8395   +1,5 min 0,9358   +4,8 min 0,9313
+0,9 min 0,8154   +2,5 min 0,9175   +5,4 min 0,9687
                  ajuste +0,022 1/m por minuto (r = +0,80)
```

Velocidade linear igual nas seis; o que muda é o giro. Não é o mundo (mesmo
rumo, mesmo chão). **Os 13,5% entre 04-08 e 05-08 que motivaram a decisão 013
acontecem aqui dentro de uma bancada** — o caminho 3 (medir no começo da sessão)
envelhece dentro da própria sessão. Causa não investigada **por decisão do
dono**, e ela estava certa: térmico ou bateria não muda o que fazer.

🔴 **E FECHAR A MALHA NÃO SALVA UM ff ERRADO** — a conta dizia que caberia
(`ki·int_max` = 0,072 rad/s de autoridade contra 0,038 rad/s necessários), a
máquina disse que não:

```
ff VELHO −0,8275   −13,5° → +22,3°   excursão 35,8°   envoltória cresce
ff HOJE  −0,9383     0,0° → +13,0°   excursão 13,0°   sobrecorrige, não assenta
```

➡️ **O conserto não é medir o ff mais vezes — é o compensador ESTIMAR a
curvatura enquanto anda** (caminho 2 da 013). É o que o dono pediu com todas as
letras: *"o compensador deve conseguir identificar o erro atual para ajeitar"*.
Próxima sessão de dev.

🟢 **O Nav2 SUBIU INTEIRO NO ROBÔ REAL PELA PRIMEIRA VEZ** — os quatro
servidores `active`. O que faltava era a TF `odom → base_link`: o FAST-LIO manda
a pose no frame `body`, que não está no URDF, e o `tf_odom` **se recusava a
publicar, corretamente** (publicar embutiria os 42 cm do Mid-360 sem sintoma).
Conserto: `-p frame_da_pose:=livox_frame`. Pré-voo 10/19 → **16/19**.
⚠️ Dívida: `body` é o frame da IMU, 5 cm do lidar.

🔴 **A DECISÃO 014 ESTÁ INERTE NO ROBÔ**: `/livox/lidar` sai em
**`livox_ros_driver2/msg/CustomMsg`** e costmaps, `collision_monitor` e o
pré-voo assinam `PointCloud2`. O FAST-LIO funciona porque lê CustomMsg — a
localização vai bem e a percepção é zero. No simulador o lidar é `gpu_lidar` e
publica PointCloud2: a 014 foi aceita num ambiente onde o defeito não existe.

⚠️ **Previsão falsificável, não testada**: converter a nuvem **sozinho não faz
os costmaps marcarem**. Com o `tf_odom` compondo a pose do sensor, o `odom` fica
na altura do sensor (`z = −0,477 m` medido), o chão vai para z ≈ −0,42 e a faixa
de altura do costmap (0,10–0,50) rejeita tudo. Dois defeitos em série.

---

## 🎚️ 07-08 (4ª leva) — O ff DO DIA TEM POR ONDE ENTRAR (dev, sem robô)

A decisão **013** foi escolhida de manhã (caminho 3: medir a curvatura crua no
começo da sessão e passar por parâmetro) e, olhando o código à tarde, **não
existia parâmetro para passar**:

- `compensador_rumo.py` declarava `curv_frente` com default **−0,817** — o valor
  de 04-08, o número que a própria decisão diz não valer como verdade;
- `pilha.launch.py` subia os dois compensadores com **só** `use_sim_time` e
  `segura_rumo`; nenhum YAML carrega a curvatura;
- sobrava `ros2 param set`, que o apêndice do roteiro lista como armadilha.

➡️ **O experimento nº 2 da próxima ida produziria um número sem destino.**

🟢 **O protocolo agora fecha ponta a ponta:**

```
três corridas SEM compensador -> medir.py --resumo curvatura
  -> a linha pronta para colar -> pilha.launch.py curv_frente:= curv_medido_em:=
  -> rosout: "ff MEDIDO em <data>"
```

- **`pilha.launch.py`**: `curv_frente`, `curv_re` e `curv_medido_em`, nos DOIS
  compensadores (sim e robô), com os defaults do nó. ⚠️ Numéricos vão como
  `ParameterValue(..., value_type=float)` — argumento de launch chega como texto
  e o parâmetro é double; cru, o compensador **cai na subida** com *parameter
  type mismatch*, e o robô arca 0,82 1/m com a pilha inteira de pé;
- **o nó anuncia a procedência do ff no `rosout`**, em WARN, com default
  `HERDADO`: pilha que sobe sem a medida do dia se denuncia. Ler com
  `ros2 topic echo /rosout --field msg | grep -i "^ff "`;
- **`medir.py --resumo curvatura`** imprime a linha de launch com a média e a
  data de hoje — e **se recusa** com n<3, dispersão acima de 5% (dentro do dia o
  robô repete em 2–3%) ou frente misturada com ré. Linha colável a partir de
  medida ruim é pior que nenhuma: ela seria colada.

⚠️ **O que isto NÃO resolve**: a curvatura segue medida **uma vez por sessão**.
Planta que mude no meio da bancada envelhece o valor dentro da própria sessão —
isso é o caminho (2) da 013 (estimador online), que segue fora.

⏳ **Nada foi ao robô**; o que a próxima sessão confirma é operacional: a linha
sai, sobe a pilha, e o `rosout` diz `ff MEDIDO em <hoje>`. Se disser `HERDADO`,
o número não chegou.

**300 testes verdes** (eram 281), por pacote: `robot_motion` 171, `tools` 70,
`robot_base` 47, `robot_planning` 12. Três novos verificados por mutação.

🔧 **Dívida vista e não paga**: rodar `robot_motion/test` e `tools/` no MESMO
processo pytest derruba `test_o_raio_de_chegada_do_nav2_bate_com_o_do_seguidor`
(o `sys.path.insert` do `test_lei_de_reta.py` quebra o import relativo do
`path_follower`). É anterior a esta leva — confirmado com `git stash` — e não
aparece rodando por pacote.

---

## 👁️ 07-08 — O COSTMAP PASSA A ENXERGAR O LIVOX (dev, sem robô)

Sessão de máquina de dev, robô desligado. Decisão **014**; entrada 07-08 do
diário. **274 testes verdes** (eram 264).

🔴 **O buraco que ninguém tinha na lista: o único consumidor da nuvem era o
reflexo.** O Mid-360 publica em `/livox/lidar` desde 24-07 (robô) e 04-08
(simulador), e **os dois costmaps do Nav2 rodavam só com o mapa estático** — o
comentário no YAML ainda dizia *"o robô simulado ainda não tem lidar"*. O robô
não desviava de obstáculo novo: **parava** na frente dele. A fatia B estava
bloqueada por configuração, não por sensor.

🟢 **Entrou uma `VoxelLayer` com a nuvem NOS DOIS costmaps**, com a mesma faixa
de altura do reflexo (0,10–0,50 m), travada em teste para os dois não
divergirem em silêncio.

⚠️ **`VoxelLayer` e não `ObstacleLayer`, e é o OPOSTO do robô 1** (que trocou
voxel por obstacle no `nav2_params_pi.yaml`). Lá o sensor é planar; aqui ele
olha para cima (−7° a +52°) e enxerga a altura `0,42 − 0,123·d`: uma caixa de
0,30 m **some quando o robô chega a 0,5 m dela**. Com raytrace 2D, o raio que
passa por cima apagaria a marca no exato momento de manobrar.

🔴 **A camada só no LOCAL costmap não resolve — medido.** Mundo
`pista_surpresa.sdf` (obstáculo que o mapa não tem), caminho de (2,0 · 5,0) a
(2,0 · 7,2):

```
                      caminho / reta   folga do centro
só no local             2,20 / 2,20      0,035 m   ATRAVESSA
nos dois                2,74 / 2,20      0,530 m   CONTORNA
```

Quem dirige via a caixa; quem planeja não. Obstáculo só no local **não produz
desvio, produz travamento educado**: o robô vai reto até lá, o reflexo para, e o
replanejamento a 1 Hz devolve o mesmo plano ruim para sempre.

⚠️ **`expected_update_rate` 0,30 → 0,50 s, corrigido por medida.** Com 0,30 o
log enchia de `has not been updated for 0.43 seconds` — e a nuvem estava
perfeita (9,7 Hz, carimbo de 100,0 ms sem cauda em 281 quadros). O atraso é do
consumidor: 20 000 pontos por quadro. **Buffer vencido deixa a camada
não-current, e costmap não-current PARA de atualizar** — errar para baixo aqui
desliga a percepção tendo um aviso amarelo por sintoma.

🧰 **Três instrumentos novos, porque a pista antiga não provava percepção**
(mundo e mapa saíam da mesma planta, então "viu" e "lembrava" eram
indistinguíveis):

- `gera_pista.py` agora escreve **`worlds/pista_surpresa.sdf`** — obstáculos que
  nunca entram no mapa;
- **`tools/banco/percepcao.py`** — células letais do costmap onde o mapa diz
  livre;
- **`tools/banco/plano.py`** — pede caminho por `compute_path_to_pose`, que
  **planeja sem mover o robô** (serve na bancada com a bateria parada).

⚠️ **O robô só conhece a FACE que viu**: 0,075 m² de uma caixa de 0,25 m². O
planejador contorna uma lasca do obstáculo, não o obstáculo.

⏳ **NADA DISTO RODOU NO ROBÔ**, e há uma dívida específica: no global costmap a
marcação é **permanente** (não há janela rolante). Com a TF `map→odom` fixa e
provisória, toda deriva do LIO vira **obstáculo fantasma acumulado**. Seguro no
simulador, dívida no robô — a saída é a mesma que já obriga o mapa a ser
argumento: localização que case `map` com `odom`.

✅ **Decisão 013 RESOLVIDA pelo dono: caminho 3** — medir a curvatura crua no
começo de cada sessão e passar `curv_frente` por parâmetro. Vira passo do
protocolo de bancada; as três corridas sem compensador já são o experimento nº 2
do roteiro.

### 🗺️ E o Nav2 do robô real não tinha mapa — decisão **015**

🔴 **A `pilha.launch.py` subia o mapa da pista SIMULADA no robô real** (uma sala
de 12 × 8 m que não existe), com o `global_costmap` em `StaticLayer` sobre isso.
O cabeçalho da launch já avisava, mas era aviso **sem saída**: não existia o
"sem mapa" para escolher. A decisão 014 piorou — o global passou a misturar
paredes fantasma com marcação real e permanente do Livox.

🟢 **Entra `mapa:=nenhum`** — sem `map_server` (e fora da lista do
`lifecycle_manager`, que aborta o bringup se um servidor não responder), sem
`static_layer`, e o `global_costmap` vira **janela rolante de 20 × 20 m**
alimentada só pelo sensor. É um **overlay** (`config/nav2_sem_mapa.yaml`), não
um segundo `nav2.yaml`: há teste que falha se ele redefinir geometria.

```
                    caminho / reta   folga    manchas no global
com mapa              2,74 / 2,20     0,530 m   2 (as duas surpresas)
sem mapa nenhum       2,78 / 2,25     0,530 m   6 (surpresas + as paredes)
```

Sem mapa, o costmap global é **só o que o sensor viu**, e as seis manchas batem
com a planta (parede oeste 0,19 contra 0,20 real; divisória 3,91 contra 3,90;
parede norte 7,80/7,81 contra 7,80). O robô planeja sem mapa e chega ao mesmo
desvio.

⚠️ **O custo é MEMÓRIA CURTA**: fora dos 20 m ele não sabe de nada e o que nunca
viu conta como livre. Não é regressão — parede fantasma no lugar errado não é
conservadora, é aleatória.

⚠️ **Armadilha nova para o apêndice**: `ros2 launch` **não morre com os nós**.
Matar os filhos por PID e deixar o launch vivo empilhou **3 pilhas simultâneas**
(3 `planner_server`, 5 `tf_map_odom`), e o sintoma foi bringup abortando com
cara de bug no código. Some-se: **`ros2 node list` mostra fantasma** do daemon
mesmo com `ps` provando zero processos (`ros2 daemon stop && start` limpa). Para
saber o que está vivo, `ps`.

### 🛫 O PRÉ-VOO — um comando responde por tudo (`tools/banco/checa_pilha.py`)

30 s, **não move o robô**: uma pilha só, nuvem e taxa, as duas TFs (incluindo
`base_link → livox_frame` contra os 0,42 m da trena), fração de nuvem
transformável, `lifecycle` dos quatro servidores, perfil sem mapa, os dois
costmaps marcando e a cadeia de comando inteira — cada falha com o conserto na
própria linha. **20/20 contra o simulador** no perfil da sessão. É o
experimento 1 do roteiro.

🔵 **HIPÓTESE RETIRADA — o registro de 06-08 estava errado num ponto.** Lá o
`collision_monitor` *"recebe e não publica nem zero"* foi lido como evidência de
que ele não conseguia transformar a nuvem. Medido em 07-08:

```
3 s de comando ZERO  em /auto_vel_raw  ->  /auto_vel recebeu    0
3 s de comando 0,05  em /auto_vel_raw  ->  /auto_vel recebeu  150
```

**Ele não republica comando nulo — é como o Nav2 funciona**, e parado o
`heading_controller` só publica zero. A falta da TF segue provada por outras
duas vias (o `tf2_echo` e o `lifecycle`), mas aquele silêncio não era prova.

**281 testes verdes.**

---

## 🤖 06-08 (tarde) — O ROBÔ CONFIRMOU: o S morre com os ganhos novos

**18 corridas na cerâmica da sala**, bateria 41,16 → 40,92 V. Tudo o que estava
"previsto e nunca visto no robô" foi visto. Dados em `docs/dados/2026-08-06-*`.

🟢 **A previsão principal PASSOU — a oscilação divergente acabou.**

```
                 invs   período medido   envoltória 1ª→2ª
VELHOS  (1,0/0,5)  1,2,2     2 de 3        2,13× 2,53× 1,87×  cresce
NOVOS  (0,25/0,12) 0,1,0     0 de 3        monotônico / 0,58×
```

A evidência limpa é o **período**, e vem do `mede_o_s`: medido em 2 das 3
corridas velhas (2,28 e 2,44 s, dentro dos 2,2–2,4 s de 05-08) e em **nenhuma**
das novas. Sem período não há ciclo. O controle do dia reproduziu 05-08 quase
exato (1,87–2,53× contra 1,86–2,18×) — a comparação é **interna**, não entre
dias. O dono, a olho: *"faz um S mas tá reto, tá bom."*

🔴 **Mas o arco NÃO é do laço, e sobrou inteiro.** `|curvatura|` deu 0,0764
(novos) contra 0,0707 (velhos) — a mesma coisa. Baixar o ganho 4,1× não mexeu
nela, porque ela vem do `ff` (−0,817 1/m, idêntico nas duas). O limiar
`< 0,05` do plano mede erro de **feedforward**, não estabilidade. ➡️ **Acertar
o `curv_frente` é hoje o conserto de maior valor do projeto.**

🟢 **O pivô é bimodal, como a quantização de 10 Hz previa.** `liga 0,10` (1
ciclo) variou **13×** entre corridas idênticas (2,2 · 3,3 · 29,7°); `liga 0,30`
(3 ciclos) ficou em ±8% (59,2 · 67,6 · 69,7°). ⚠️ O modo alto deu ~30°, não os
~16° previstos — o modelo acerta a **estrutura** e erra a **escala** por ~2×.

🟡 **O preditor de Smith EMPATOU** (invs 1,0,1; nenhum período), contrariando a
bancada onde perdia claro. **O desempate não coube na sala**: sustentada ×
assenta só aparece em corrida longa, e a trava de 1,2 m corta em 4,7 s. Fica
desligado — em empate ganha quem não depende de modelo, ainda mais com o pivô
mostrando o modelo errando escala por 2×.

🔵 **Hipótese retirada:** o S era do laço, não da boba (BO-4).

🔴 **OS TESTES C E D FALHARAM, e o D tem causa única: falta o TF `odom →
base_link`.** O FAST-LIO publica `/Odometry` como mensagem e não a
transformada. A árvore TF parte em dois, o `planner_server` não ativa, o
`lifecycle_manager` **aborta o bringup** e leva o `collision_monitor` junto —
ativado na mão, ele recebe e não publica nem zero, porque não consegue
transformar a nuvem. Sensor bom, nuvem a 8 Hz, cadeia de tópicos inteira.
**Falta uma transformada.** No teste C, o teleop não publicou **nada** em
`/key_vel` (44 s de CSV, zero amostras dessa fonte).

---

## 🎚️ 06-08 — O S tem mecanismo, número e conserto projetado

🔴 **A OSCILAÇÃO DO ROBÔ CRESCE — é instabilidade, não transiente.** Olhando a
envoltória das três corridas compensadas de 05-08 (o que eu não tinha feito ao
contar só inversões):

```
corrida    1ª excursão   2ª excursão   cresceu   meio-período
a             −2,97°        +6,46°      2,18×       2,38 s
b             −6,47°       +12,04°      1,86×       2,50 s
c             −5,43°       +11,84°      2,18×       2,50 s
```

✅ **O TEMPO MORTO DO LAÇO É 0,94 s**, e o número fecha por **duas rotas
independentes**: (a) oscilar a 1,277 rad/s com o PI que estava rodando exige
esse atraso; (b) a placa medida tem 0,27 s de liga (01-08) + 0,52 s de desliga
(04-08), mais pose a 10 Hz e a janela de 0,2 s ≈ 0,94 s.

✅ **GANHOS REDUZIDOS 4,1× — `kp` 1,00 → 0,25 e `ki` 0,50 → 0,12.** Crescer
2,07× por meio-período põe o ganho de laço em ~2,07 na travessia de fase, e ele
precisa ficar abaixo de 1. Os dois caem **juntos** (a razão `ki/kp` não muda —
é ganho a menos, não controlador diferente). Margem de ganho ~2×.

🔴 **A PLANTA DE BRINQUEDO DO TESTE TINHA O MESMO PONTO CEGO DO GAZEBO.** O
`_roda_planta` já modelava atraso, mas de **0,26 s** — só a latência de liga.
Com esse valor **ela não oscila com ganho nenhum**, e era por isso que o S não
aparecia em lugar nenhum. Corrigida para 0,94 s, ela reproduz o fenômeno e vira
**o único lugar do projeto onde a estabilidade do rumo se julga sem robô**:

```
                     atraso 0,26 s (antes)      atraso 0,94 s (medido)
ganhos ANTIGOS       4,6 · 0,2 · 0,0            15,2 · 15,3 · 12,2 · 10,0 · 8,2
ganhos NOVOS         8,2 · 0,3 · 0,1            16,5 · 0,4 · 0,2 · 0,1
```

⚠️ **`int_max` FICA EM 0,6, e isso foi decidido com medida.** Baixar `ki` 4,1×
encolhe junto a autoridade do integrador (`ki·int_max`: 0,30 → 0,072 rad/s).
Subi para 2,5 para preservar o produto e **o sino voltou** (9° de segunda
excursão): com tempo morto, esse teto não é só autoridade, é proteção contra
**windup**. O preço de mantê-lo baixo: com o ff 25% errado o rumo assenta ~6,8°
fora da referência. **O conserto disso é acertar o `curv_frente`**, não subir o
integrador.

🔮 **PREDITOR DE SMITH implementado e OPT-IN (`preditor:=true`) — e ele PERDE.**
Em vez de baixar o ganho, desconta o que está a caminho. Prevê **só a correção**,
nunca o feedforward (o efeito futuro do ff é cancelado pelo arco futuro).

```
configuração                   excursões (graus)      assenta   rumo
ANTIGOS 1,0/0,5 sem preditor   15,2 15,3 12,2 10,0     39,9 s   +0,94°
ANTIGOS 1,0/0,5 COM preditor   15,3  2,3  3,7  3,6     nunca    +3,65°
NOVOS 0,25/0,12 sem preditor   16,5  0,4  0,2  0,1      9,6 s   +0,06°
```

Mata a **divergência** (12° viram 3,6°) mas deixa ondulação **sustentada**, onde
o detune assenta abaixo de 0,1°. Fica desligado, com o veredito travado em teste.
⚠️ Ressalva: a planta de brinquedo aplica o arco **imediatamente** enquanto o wz
chega atrasado; no robô os dois chegam juntos. Só a máquina desempata — 3 corridas.

⏳ **NADA DISSO FOI VISTO NO ROBÔ.** A previsão que a próxima sessão testa é uma
só e é falsificável: **a amplitude tem de DECAIR em vez de crescer.** Se
continuar crescendo, o S não é do laço e o caminho passa a ser o **BO-4**.
Plano de campo: `docs/PLANO_SINTONIA_RUMO.md`.

**530 testes verdes.** Ganhos, atraso, comparação com o preditor e o caso do
**modelo errado** travados em teste; quatro verificados por mutação.

---

🟢 **O COMPENSADOR DE RUMO PASSOU NO ROBÔ REAL (05-08)** — 21 corridas, dados e
registro em `docs/dados/2026-08-05-bancada-robo/`, entrada 08-05 (8ª leva) do
diário:

```
                sem compensador          com compensador       redução
FRENTE  n=3    −0,9116 (raio 1,10 m)   +0,0417 (raio 24 m)      95,4%
RÉ      n=3    −0,0968 (raio 10,3 m)   −0,0466 (raio 21 m)      52%
                                    critério (011): |curv| < 0,05
```

As duas médias passam. Ressalvas que o número esconde: 3 das 6 corridas
compensadas estouram o critério **individualmente**; os dois sentidos falham por
motivos **opostos** (de frente **oscila** — o S que o dono viu; de ré fica
**aquém**); e o espalho absoluto **não mudou** (0,035 → 0,044 1/m), porque o
compensador tira viés e não toca variabilidade de planta.

  ⚠️ **RELEITURA DE 06-08, e ela muda o que aquele `+0,0417` significa**: não é
  um viés estável, é a **média de uma oscilação que CRESCE, cortada em 1,2 m**.
  Ou seja, o valor depende de onde a corrida terminou. Consequências:
  · o número **não** diz nada sobre o sinal do erro do feedforward — o que
    obriga a retirar a conclusão de 05-08 de que corrigir o `curv_frente`
    pioraria (ela tinha lido o resíduo como sobrecorreção em regime);
  · os ganhos que produziram essa tabela **não são mais os do repo** (caíram
    4,1× em 06-08), então a próxima sessão tem de rodar os antigos como
    **controle do dia** para a comparação valer.

🔴 **O PIVÔ EM MALHA ABERTA NÃO FUNCIONA NESTE ROBÔ (05-08).** `liga 0,15 s` dá
16,6–33,4°; `liga 0,20 s` dá 30,3–35,8° — **as faixas se sobrepõem**. Duas
corridas com o **mesmo tempo medido** deram 16,6° e 33,4°. A grandeza que o
controlador escolhe **não determina** a que ele quer; pivô tem de ser malha
fechada no yaw. E a "zona morta de tempo abaixo de 0,4 s" que o simulador
previu **não existe**: 0,3 s dão 48°.

  ✅ **A CAUSA FOI ACHADA na 9ª leva do mesmo dia, e é QUANTIZAÇÃO** — não a
  planta. O `controller_manager` roda a **10 Hz**, então o tempo de comando é
  contado em **ciclos de 100 ms**: `liga 0,15 s` são 1,5 ciclos e a fase decide
  se cabem 1 ou 2. Com 16,6° por ciclo, `0,20` = 2 ciclos (33,2° previsto,
  30,3–35,8 medidos) e `0,30` = 3 ciclos (49,8° previsto, 48,0 medidos). E
  **33,4 / 16,6 = 2,012**. Reproduzido dentro do simulador depois de igualar a
  taxa. ⚠️ **Isto corrige o que o `ambiente.txt` da bancada e a 8ª leva do
  diário dizem** (lá a causa foi atribuída a atrito de partida, bateria ou
  comutação do motor).

  ⏳ **Previsão falsificável para a próxima ida**: repetir `liga 0,10` com n=3.
  Se a quantização estiver certa, tem de sair **bimodal** — ora ~0°, ora ~16°.
  O único valor que temos (2,7°) é compatível com ter pego **zero** ciclos.

✅ **O ROBÔ REAL GANHOU CORPO (05-08, 11ª leva).** O `tracao.launch.py` carregava
o `diffbot.urdf.xacro` — o **exemplo de demonstração do `ros2_control`** (caixa
0,10×0,10×0,05, roda 0,015, bitola 0,10, duas bobas, sem Livox). Passou a
carregar o `robo2.urdf.xacro` com `sim:=false`: a arquitetura "uma descrição,
dois hardwares" **já existia** (os dois blocos `ros2_control` chaveados por
`<xacro:arg name="sim">`) e só nunca tinha sido ligada ao robô.

  Os blocos de hardware foram comparados renderizando os dois xacro **antes** da
  troca: plugin, juntas, `device`, `wheel_radius`, `feedback_sign_*` e
  `deadband_*` saem idênticos — **o atuador não muda**. O que muda é a geometria,
  e o **`livox_frame` passa a existir**, que era o bloqueio do teste D.

  ⚠️ **Falta confirmar no robô**: `sessao.py --checar` mostrando
  `wheel_separation = 0.2700` vivo depois da troca. Primeiro passo da próxima ida.

📏 **MID-360 MEDIDO COM TRENA: 42 cm do chão, centrado (05-08)** — não os 27 cm
supostos. A zona cega vai de 2,19 m para **3,40 m**, e a 0,5 m ele só vê acima de
**36 cm** (era 21). O simulador estava 55% otimista nela; corrigido nos dois
lados. **A caixa do teste D tem de ter 50 cm, não 40.** Travado em 4 testes
novos, verificados por mutação.

🔴 **O `collision_monitor.yaml` descreve o robô errado (aberto).** O comentário
que justifica `max_height: 0.50` diz *"o robô tem 0,30 m de alto"* — número do
modelo antigo da decisão 004. A caixa termina a **0,230 m** e o Mid-360 está a
**0,42 m**: é o sensor que define o gabarito. Não mexido de propósito —
`max_height` é parâmetro de segurança e merece decisão própria.

✅ **`twist_mux` RESOLVIDO (05-08, 10ª leva) — `./setup_twist_mux.sh`.** Ele
entra por **fonte em commit fixado** (tag 4.5.0), mesmo padrão do
`setup_livox.sh`; compilado no NUC liga contra os headers que a máquina tem.
Nenhuma lib do sistema é tocada.

  🛑 **E ficou PROVADO que o `apt upgrade` de um pacote só seria perigoso.**
  Lendo os símbolos do `.deb` da 4.2.7 **sem instalar**:

  ```
  diagnostic_updater 4.2.6   exporta  ...NodeTopicsInterfaceEEd     (só)
  diagnostic_updater 4.2.7   exporta  ...NodeTopicsInterfaceEEdh    (só)
  ```

  O símbolo antigo **desaparece** — é substituição, não adição. Subir a lib
  quebraria todo consumidor compilado contra a 4.2.6, inclusive o
  `controller_manager` que a base usa. Aquele caminho só existe como **upgrade
  coerente da pilha inteira**, em sessão própria, com a base conferida depois.

  ⚠️ **Deploy**: o NUC precisa rodar `./setup_twist_mux.sh` uma vez antes do
  teste C. O `git reset --hard` não traz o clone (ele é `.gitignore`, como o
  Livox e o FAST-LIO).

  ✅ **Itens 2 e 3 do teste C já estão provados sem robô**
  (`tools/banco/prova_mux.py`): o humano vence a autonomia, e soltar devolve o
  comando. **Falta o item 1**, que precisa de máquina: soltar o teclado e o robô
  parar sozinho em 0,4 s.

---

## 📋 A PRÓXIMA IDA AO ROBÔ — tudo o que está esperando máquina

> Atualizado depois da sessão de **06-08**, que rodou 18 corridas. Os itens 1, 2
> e 5 da lista antiga **foram fechados**; o 3 e o 4 falharam, e por motivos que
> agora têm causa. O roteiro de operação (`docs/PROXIMA_SESSAO_NO_ROBO.md`)
> segue válido no **método** — o que mudou foi a lista de experimentos.

🔴 **O conserto de maior valor NÃO precisa de robô ligado para ser escrito.**
Dois dos três bloqueios de hoje são de dev:

| # | o que | por quê | onde |
|---|---|---|---|
| 1 | **TF `odom → base_link`** | o FAST-LIO publica `/Odometry` como mensagem e **não** publica a transformada. A árvore TF fica partida, o `planner_server` não ativa, o `lifecycle_manager` **aborta o bringup inteiro** e leva o `collision_monitor` junto. Bloqueia Nav2 **e** o teste D | DIÁRIO 06-08 (3ª leva) |
| 2 | ~~**o `ff` fixo não fecha o arco**~~ ✅ **FECHADO NO DEV (07-08, 4ª leva)** | a planta muda de dia (−0,8031 em 04-08 contra −0,9116 em 05-08; 13,5%, faixas que não se tocam, contra 2–3% *dentro* do dia), então nenhum valor único serve. O dono escolheu o caminho 3, e ele agora existe na máquina: `pilha.launch.py curv_frente:=… curv_medido_em:=…`, a linha sai pronta do `medir.py --resumo curvatura`, e o `rosout` denuncia ff herdado. **Falta só rodar** | decisão **013** |
| 3 | **teleop não publica em `/key_vel`** | 44 s de gravação, **zero** amostras dessa fonte. Não é o `le_tecla()` — é antes disso | `homem_morto.py` |
| 4 | **`bin/robot-key` com `set -u`** | briga com `COLCON_TRACE` e `AMENT_TRACE_SETUP_FILES` dos `setup.bash`. Uma linha | — |

**Só depois disso vale voltar ao robô**, e aí a lista é curta:

```
0. deploy: bundle -> colcon build   (o twist_mux JÁ está compilado no NUC)
1. sessao.py --checar   🔴 wheel_separation TEM de dar 0,2700 (passou em 06-08,
                        reconferir: o NUC reinicia junto com o robô)
```

| # | o que | por quê | onde |
|---|---|---|---|
| 1 | **teste C item 1** (homem-morto) | único teste cuja falha é *pior que não ter a função*. Agora com gravador: `tools/banco/homem_morto.py` mede os três intervalos em CSV | `PLANO_TESTE_ROBO.md` §1 |
| 2 | **teste D** (reflexo) | **bloqueado até o TF existir.** O plano está errado ao listá-lo como executável sem `map` | `PLANO_TESTE_ROBO.md` §1 |
| 3 | **desempate do preditor** | empatou em 06-08; precisa de **corredor longo** — a sala não deu. Sustentada × assenta só aparece em corrida longa | `PLANO_SINTONIA_RUMO.md` §4 |
| 4 | **`heading_controller`: pivô indisponível** | ele recusa girar parado (pediria 1,48 rad/s, teto 1,00), mas a máquina **faz** 69,7° com `liga 0,30`. Quem recusa é o teto, não o robô | `movimentacao.yaml` |
| 5 | **vídeo da traseira** | segue barato; deixou de ser urgente (o S era do laço) | BO-4 |

⚠️ **O NUC CAI JUNTO COM O ROBÔ.** Parecia ter alimentação separada; não tem.
Em 06-08 um deploy morreu no meio (`No route to host`) e o NUC voltou com
`up 0 min`. Salve em levas para o `origin` — foi o que impediu perda hoje. E
`/tmp/logs` **não sobrevive**: recriar antes de qualquer `nohup`.

🟢 **PISO E BATERIA: resolvido em 06-08.** Cerâmica da sala, 41,16 → 40,92 V,
gravados no `ambiente.txt` de cada pasta de dados. A queda de 0,24 V ao longo de
18 corridas mostra que a suspeita de deriva de planta por bateria **não se
materializou** nesta sessão — as duas condições da sintonia são comparáveis.

---

🟢 **SIMULADOR AGORA BATE COM O ROBÔ EM ARCO E EM PICO DE WZ (04-08):**
- **Arco (1ª leva)**: frente −0,817 1/m (robô −0,838), ré −0,109 (robô −0,113)
- **Pico de wz (3ª leva, planta normal)**: 2,25 rad/s (robô 2,33)
- As duas maiores grandezas de fidelidade estão dentro da dispersão da máquina.
- ~~**Terceira pendência (sobrepasso):** o simulador desacelera 2× mais rápido
  por falta do atraso de desliga da placa (~0,5 s).~~ ✅ **ENTROU em 05-08 (9ª
  leva)**, junto com duas outras correções que o dado obrigou —
  `docs/dados/2026-08-05-aceitacao-atraso-desliga/`:
  1. **atraso de desliga** (0,52 s, decaindo — segurar o valor cheio deixava o
     simulador dando ~37° para 1, 1,5 **e** 2 ciclos de comando);
  2. **a latência de liga deixou de DESCARTAR o comando** — era um `return 0,0`
     que fazia **todo pulso menor que 0,27 s produzir exatamente nada**, e o
     robô gira 32° com pulso de 0,20 s. Virou fila de atraso;
  3. **`update_rate` 50 → 10 Hz, igual ao robô** — remove uma divergência que
     estava documentada como deliberada.

  **Arco (n=3): PASSOU e melhorou** — ré de −0,109 para −0,0938 (robô: −0,0968),
  razão 7,5× → **8,74×** (robô: 7,4× em 04-08, 9,4× em 05-08). O medo do BO-4
  ao baixar a taxa não se concretizou.
  **Pivô (n=3): NÃO passou** — o simulador é chato demais (19° com 1 ciclo, 30°
  com 3; o robô é linear, fator 3,0). Parado de propósito: os dois pontos em
  que ele mais discorda são os dois em que o **robô tem n=1**.

- ✅ **O item 12c (dispersão) veio de graça com a taxa.** O `ESTADO` registrava
  "o simulador é determinista demais". Com 10 Hz ele ficou **bimodal como o
  robô**: tempos múltiplos de 100 ms saem repetíveis, e `liga 0,15 s` (1,5
  ciclos) espalha exatamente entre os valores de 1 e de 2 ciclos.

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
  ⚠️ **07-30: o NUC foi reinstalado** — o nosso repo tinha sumido do disco (só
  restava o workspace do estágio) e foi **re-deployado do zero** nesta sessão de
  bancada, via `git bundle` (o NUC não tem auth no GitHub — ver DIARIO 07-30 2ª
  leva). A base foi reconstruída (com um conserto de `launch_ros` no
  `tracao.launch.py`, commitado) e **volta a subir**: `--checar` passou,
  `/Odometry` a 9,9 Hz, placa e lidar de pé (este só depois de um power-cycle).
  📄 **O robô medido está em `docs/MODELO_ROBO2.md`** — atuador, zona morta,
  desvio de rumo, rotação e curva, com o que o modelo NÃO cobre e como conferir
  que o robô está em estado de medir. É o documento a ler antes de tocar no
  simulador ou no controlador.

  ✅ **07-31: bancada no robô real — o modelo está medido.**
  Detalhes completos em `docs/MODELO_ROBO2.md`. Resumo:

  ```
  zona morta linear   0,0178 m/s (frente)   0,0148 m/s (ré)
  zona morta de giro  0,095 rad/s           faixa 0,084 – 0,105
  a_dec angular       ~3,05 rad/s²          faixa 2,08 – 3,67
  curva v=0,10 wz=0,30: obedece a 0,94x, raio 0,333 m
  ```

  🔴 **`cmd_vel` não é obedecido em MAGNITUDE na faixa útil.** O driver escala
  qualquer comando pequeno até a roda maior bater em 100 unidades de firmware,
  com fator `k = 100/mx` — então **todo comando entre ~0,008 e ~0,838 m/s vira a
  mesma coisa na placa**. O teto do robô é 1,0 m/s, ou seja, o patamar cobre a
  operação inteira: `cmd_vel` escolhe **sentido**, não módulo. Latência de
  ~0,35 s para destravar. É o primeiro fato a levar para o Gazebo.
  A compensação **fica ligada**: sem ela o robô não sai do lugar (`v=0,25` andou
  2 mm em 1,5 s).

  🔴 **O robô não anda reto indo para a FRENTE** — roda esquerda 11–12% mais
  rápida e demorando 0,2–0,4 s a mais para parar, dando −9,2°/−6,9° de desvio em
  ~18 cm. De ré as rodas saem simétricas (+0,5°). Confirmado a olho pelo dono.
  **↑ MEDIDO EM PERCURSO LONGO em 08-04, e é maior do que isso** — ver o bloco
  de 08-04 abaixo: de frente ele descreve um **círculo de 1,22 m de raio**; de
  ré desvia 8,3× menos, mas **não** é reto (raio 10,2 m). A causa deixou de ser
  "em aberto": o arco está preso ao corpo (controle de piso feito) e 88% dele é
  o termo que só existe indo para a frente.

  ✅ **O LIO é excelente** — parado, deriva **1,8 mm em 15 s**. Bateu com o olho
  do dono em todas as conferências do dia, inclusive numa de ~270°. As três
  acusações que fiz contra ele durante a sessão eram defeitos meus (janela de
  derivação curta, odometria em `open_loop` como referência, buraco de gravação)
  — ver DIARIO 07-31 4ª leva.

  ⚠️ **`/hoverboard_base_controller/odom` não mede nada**: `open_loop: True`, ele
  integra o comando e devolve. Odometria de roda real só pelos encoders crus
  (`/hoverboard/{left,right}_wheel/velocity`). Resolvida a "anomalia" de
  `-70,1 rad` contra `-0,607 rad`: os motores são espelhados e reportavam sinais
  opostos: corrigido com `feedback_sign_left/right` no `robo2.urdf.xacro`.

  ⚠️ **Antes de medir qualquer coisa**, conferir que há **exatamente um**
  `fastlio_mapping` e **um** `livox_ros_driver2_node` no NUC. Em 07-31 três
  pilhas órfãs publicando em `/Odometry` produziram saltos de ~1,35 m e custaram
  horas de diagnóstico errado.

  ~~❌ **O banco está em 2 passos de 6.**~~ **↑ 08-04: são 4 de 6** — ver abaixo.

  ⚠️ **Passos 4 e 5 precisam ser REESCRITOS antes de rodar.** Os dois varrem
  velocidade (0,2 / 0,4 / 0,6 m/s) e as três caem dentro do patamar da
  compensação — dariam o mesmo resultado. A varredura tem de subir acima de
  0,838 m/s, e isso exige espaço. (O passo 3 **não** tinha esse problema e
  rodou em 08-04: o `a_dec` é medido com o comando em ZERO, e a compensação só
  age enquanto há comando.)

  ~~⚠️ **O modelo só está aferido em rajadas de ~1 s e ~20 cm.**~~ **↑ FECHADO
  para o desvio de rumo em 08-04**: seis corridas de 1,3 a 3,7 m, limpas, com
  uma pilha só de localização. Segue aberto para velocidade sustentada acima do
  patamar.

  ~~🔧 **Dívida de instrumento:** os scripts de rajada não percebem que o robô
  sumiu.~~ **↑ PAGA em 08-04 para o `ensaio.py`** (`7d7fad8`): fonte que cala
  por mais de `--sem-dado` (1,0 s) **aborta** a corrida e sai com código 1, que
  o `sessao.py` já sabia tratar e nunca recebia. 4 testes. **Segue aberta no
  `rajada_rodas.py`.**

## 🎯 2026-08-04 — O bloqueio era do instrumento, e o robô anda em círculo

Nove corridas. **O banco foi de 2 passos de 6 para 4.** Detalhes na entrada
08-04 do `docs/DIARIO.md`; dados crus e leitura em
`docs/dados/2026-08-04-bancada-robo/` (com `ambiente.txt`).

  🔴 **O DESVIO DE RUMO, medido em percurso longo e com controle de piso**
  (`n=2` matched por sentido, `--espaco 1.2`):

  ```
  FRENTE   curvatura −0,817 1/m   raio  1,22 m   faixa −0,73 a −0,90
  RÉ       curvatura −0,098 1/m   raio 10,19 m   faixa −0,08 a −0,12
                                            razão frente/ré = 8,3x
  ```

  **O arco é do ROBÔ, não da sala.** Nas quatro corridas o corpo ficou a ~0°
  (frente) ou ~180° (ré) andando para a mesma faixa de direção do mundo — mesmo
  pedaço de chão, corpo girado. Caimento de piso é força fixa **no mundo** e
  faria a curvatura **no corpo** trocar de sinal. Ela saiu negativa nas quatro.

  Isso **exclui motor/placa fraca de um lado** (daria a mesma curvatura nos dois
  sentidos). Sobra causa dependente do sentido de marcha — a assinatura da roda
  boba, arrastada atrás indo pra frente e dianteira indo de ré. Encaixa com os
  encoders de 31-07 (esquerda 11–12% mais rápida **só** de frente).

  **Lê como duas parcelas somadas**, e a consequência é de projeto:

  ```
  constante nos dois sentidos  ~ −0,10 1/m  (raio 10,2 m) — sobrevive à ré
  só de frente (a boba)        ~ −0,72 1/m  = 88% do arco de frente
  ```

  ➡️ **Consertar a boba NÃO deixa o robô reto.** Sobra raio de ~10 m, que o
  seguidor tem de fechar em malha fechada de rumo.

  🔴 **`a_dec` MEDIDO (passo 3, `n=3`), e ele NÃO é constante:**

  ```
  a_dec EFETIVO  média 3,26   faixa 2,68–4,32   dispersão 50%
  a_dec CAUDA    média 1,03   faixa 0,88–1,12   dispersão 24%
  pico de wz     média 2,33   faixa 2,23–2,51   dispersão 12%
  ```

  **O número que o seguidor deve usar é a CAUDA, ~1,0 rad/s²** — 3× menor que os
  3,05 do `MODELO_ROBO2.md`. O `a_dec` efetivo espalha 50% entre corridas iguais
  e não serve como constante de projeto (é `wz²/(2·Δθ)`, o pico entra ao
  quadrado); os 3,05 caem dentro da faixa dele, então o substituto não errou o
  *valor efetivo* — errou o *uso*. A cauda é onde o robô assenta no rumo, e é o
  lado seguro do erro por 27-07 ("errar para baixo é de graça, para cima traz o
  S de volta").

  ⚠️ **Latência da placa depois do corte: 0,40 / 0,56 / 0,60 s** (média ~0,52 s),
  mais que os ~0,35 s que o registro trazia.

  ✅ **O "giro espelhado" que bloqueou 30-07 NÃO EXISTE** — era o `atan2` do
  cutucão enrolando em ±180°. Com o patamar, `+0,6 rad/s` por 2 s gira bem mais
  que meia volta: `281,5° − 360° = −78,5°`, o número exato do bloqueio. Hoje leu
  −71,0° e o dono viu o nariz ir **para a esquerda varrendo bastante** = 289°
  enrolados. **O swap de rodas NÃO deve ser aplicado. O estado atual é o certo.**

  ✅ **O yaw do LIO NÃO tem sinal invertido** — a retratação da 3ª leva de 31-07
  era ela própria incorreta, tirada do mesmo enrolamento (`−68,2 + 360 =
  291,8°`). Conferido no dado cru daquele dia: comando `+0,30` → yaw desenrolado
  `+147,7°`; comando `−0,30` → `−150,0°`. Os sinais concordam, e esses números já
  estavam na tabela do diário — ninguém cruzou as duas partes do registro.

  ❌ **O passo 6 não pode rodar como está escrito**, por dois motivos
  independentes: não existe reta de referência (o robô faz círculo, então "o rumo
  volta ou foge?" não tem sentido) e o pulso de perturbação dura 0,5 s contra uma
  latência de ~0,5 s. Medido: wz médio −0,405 antes, −0,358 durante, −0,369
  depois. O que rodou hoje foram **retas puras medindo curvatura**, que é a
  pergunta que este robô sabe responder.

  ⚠️ **A trava de `--espaco` é RADIAL e roda por cima da odometria em
  `open_loop`.** Na primeira corrida ela achou que o robô tinha andado 2,98 m em
  linha reta enquanto o LIO sabia que ele estava a 1,79 m da origem, fazendo
  círculo — **cortou pelo motivo errado, e o robô bateu numa cadeira**. Ela não
  protege contra excursão lateral. Num robô que arca, dimensionar por `--espaco`
  pequeno é o jeito de limitar o **disco varrido**.

  ✅ **CONSERTADO no mesmo dia** (`7a0c364`): o `atan2` que enrolava, no cutucão
  do `sessao.py` e no `a_dec` do `medir.py`. Os dois passam a **acumular** o yaw
  amostra a amostra. No `medir.py` veio junto um segundo defeito entrelaçado: a
  janela de frenagem começava no **corte** e não no **pico**, incluindo os 0,4–0,6 s
  em que a placa ainda empurra e o robô ainda ACELERA — consertar só o `atan2`
  deixaria a função devolvendo número errado com cara de consertada. Ele passa a
  imprimir também o `a_dec` da **cauda**. 6 testes, os 4 principais verificados
  por mutação. Suíte: **421 verdes**.

  ⚠️ **Piso e bateria não foram informados** nas nove corridas — o `ambiente.txt`
  registra `NÃO INFORMADO`. Sem eles a sessão não se compara com a próxima.

### 🔗 Como 08-01 e 08-04 se encaixam (escritas independentes, em máquinas diferentes)

  ✅ **CONFIRMAÇÃO CRUZADA, e é forte.** A entrada de 08-01 derivou **do código**
  que a placa entrega um único `wz` de **2,204 rad/s**. Em 08-04 o robô foi
  medido e os picos das três corridas de pivô deram **2,226 / 2,514 / 2,253
  rad/s**. Duas rotas independentes — leitura do driver e LiDAR no robô — no
  mesmo número. O modelo do atuador de 08-01 **está validado em hardware**.

  ⚠️ **As duas "latências" NÃO se contradizem: são coisas diferentes.**
  08-01 mede **0,273 s** para o robô *começar* a andar depois do comando (atraso
  de liga). 08-04 mede **0,40 / 0,56 / 0,60 s** entre o comando *zerar* e o `wz`
  atingir o pico — ou seja, quanto a placa **continua empurrando depois de
  desligada** (atraso de desliga). São dois fenômenos, os dois reais, e o de
  desliga é o maior. Não tratar um como correção do outro.

  🔴 **O `a_dec` de 08-04 fica MAIS importante com a 005 em xeque, não menos.**
  Se `wz` não é modulável — a placa entrega um valor só —, a lei de frenagem
  não roda, e **a única alavanca que sobra é DECIDIR QUANDO CORTAR**. O que
  acontece depois do corte é exatamente o que o `a_dec` descreve. Então ele
  deixa de ser um ganho de controlador e passa a ser **o limite de precisão de
  rumo da máquina**:

  ```
  pico de 2,33 rad/s  ->  47° de giro DEPOIS do corte (faixa 42–53°)
  ```

  ➡️ **Qualquer pivô comandado neste robô custa ~47° de sobrepasso**, mais o que
  ele girou sob comando. Isso é piso, não sintonia: nenhum ganho conserta, e é o
  número que a saída escolhida para o BO da compensação vai ter de derrubar.

  ⚠️ **A "zona morta de giro" de 07-31 (0,095 rad/s) segue valendo como número, e
  08-01 corrige o que ela SIGNIFICA**: é o disparo da compensação (`mx > 1.0` →
  `wz > 0,0621`) mais a latência sobre a rampa, não atrito. Descreve o
  **sistema**, e se move se alguém mexer no `deadband_speed`. **O atrito segue
  não medido** — e, com a compensação ligada, 08-04 não conseguiu medi-lo
  tampouco.

### Medidas ✅ CONFERIDAS COM TRENA (2026-07-29)

```
caixa 433 × 455 × 145 mm, fundo a 85,2 mm do chão
bitola 270 mm   roda Ø160 mm (raio 0,080)   espessura 45 mm
eixo motriz a +81,5 mm do centro (era 0,15 estimado)
```

A bitola era o item nº 1 e **os dois valores herdados estavam errados, em
sentidos opostos**: simulador com 0,20 girava 26% a MENOS que o comandado,
robô real com 0,32 girava 19% a MAIS. Sintonizar rumo na bancada e levar pro
robô erraria duas vezes, em direções contrárias. Raio: 0,0825 → 0,080 (~3% de
odometria, 30 cm a cada 10 m). Detalhes na entrada 07-29 do `docs/DIARIO.md`.

### Medidas que AINDA faltam

1. **Diâmetro da rodinha da boba** — fecha `boba_raio` e `boba_trail` de uma
   vez. Os valores no arquivo hoje são PROVISÓRIOS (0,025 / 0,01), escolhidos
   só para serem possíveis: o anterior (roda de 100 mm) não cabia nos 85,2 mm
   de vão. Governam o comportamento que o projeto inteiro quer reproduzir.
2. **Largura da caixa na altura das rodas** — as rodas ficam 70 mm para dentro
   da parede lateral, então ou há recortes ou a parte de baixo é mais estreita
   que os 455 mm do topo. Não afeta giro nem odometria; afeta o footprint que
   o Nav2 usa pra decidir se passa num vão.
3. **Massas** — os 10 kg (5,8 + 2 + 2 + 0,2) seguem estimados.

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
  **↑ SUPERADO em 2026-07-29** — a bitola foi medida: 0,270. Ela caiu ENTRE os
  dois palpites e **não resolveu a pergunta**. Reescalando: com zona morta 0,10
  o pivô exige 0,96 rad/s (cabe no teto de 1,0, mas com 4% de folga — o que não
  é margem nenhuma); com zona morta 0,15, exige 1,48 rad/s e é impossível, PIOR
  que os 1,25 que se supunha. Quem decide agora é a **zona morta**, ainda não
  medida — ela tomou o lugar da bitola como item nº 1 da bancada.
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
  **↑ A expectativa NÃO se cumpriu (07-29).** A bitola medida (0,270) é MENOR
  que os 0,32 supostos, e braço menor exige MAIS wz: o pivô passa a exigir
  1,48 rad/s nesse cenário, contra os 1,25 que se temia. Medir a bitola
  **piorou** o caso em vez de resolvê-lo, e a ré da decisão 007 segue
  necessária. Só a zona morta pode mudar isso agora.
- **A boba do simulador é decorativa** — ver BO-4. Consequência imediata: a ré
  foi validada só no simulador, e a única coisa que preocupa nela (a boba
  virando roda dianteira) é justamente o que aquele modelo não pode mostrar.

## 🔀 2026-07-28 (2ª leva) — Navegação própria APOSENTADA, Nav2 entra

O dono dirigiu o robô no simulador clicando no RViz e reprovou o resultado. O
CSV da sessão está em `docs/dados/2026-07-28-cliques-movimentacao.csv` e o
diagnóstico no diário. O resumo em um número: **0 amostras de giro parado em
2714** — ele nunca virou no próprio eixo; alvo a 0,43 m custou 3,66 m de
caminho e 57 s.

- **Dois defeitos distintos, e só um era da ré.** O ciclo "ré e anda" (12
  entradas, período 2,10 s) é da histerese da decisão 007, que solta a manobra
  assim que o alvo cabe *naquele instante*. O **balão** é da movimentação:
  `raio = v/wz`, com `wz_max` 1,0 (herdado, nunca medido) e o piso de linear
  que a zona morta obriga — girando a 1,0 rad/s ele é obrigado a andar a
  0,23 m/s.
- **Decisão do dono**: aposentar a navegação ponto a ponto (`goal_navigator` +
  a ré da decisão 007 — os dados ficam), trazer o **Nav2**, e por agora testar
  **só o planner**. O seguidor provavelmente será nosso, como no robô 1.
- **O "gira no lugar e anda reto" do robô 1 foi DESCARTADO** pelo dono: aquilo
  era a única saída do chassi de 4 rodas, que não faz arco. Este faz curva boa
  e não deve parar para virar.
- **O que vale trazer do robô 1** (lido a pedido dele): a ré como recuperação
  **por sintoma** com vão traseiro medido em metros (`rear_min_gap`), o
  `twist_mux` com prioridade, e o carrot no plano quando houver seguidor.
- ⚠️ **`wz_max = 1,0` nunca foi medido** e é 4× menor que o do robô 1
  (2,4–4,5 rad/s). É ele que torna o pivô "impossível" por aritmética. Virou
  item de bancada.
  **↑ 07-29: segue sem medida no robô, e piorou no simulador** — em malha aberta
  o comando de 1,0 rad/s entrega 0,79. O teto efetivo é ainda menor que o número
  escrito, e o pivô fica mais longe, não mais perto.

### Bancada do planner (`ros2_packages/robot_planning/`)

Dois cliques no RViz, dois caminhos desenhados, uma tabela de números —
**sem robô, sem simulador, sem sensor**, de propósito: o defeito que trouxe o
Nav2 nasceu na movimentação, e julgar planner junto com quem executa mistura as
culpas. Compara **Theta\*** (o do robô 1) com **Smac Hybrid-A\*** em
Reeds-Shepp, que respeita raio de curva e pode usar ré no próprio plano.

Pista em `tools/mundo/gera_pista.py`, que gera **mapa do Nav2 e mundo do
Gazebo da mesma planta**: porta 0,90 m, bloco solto, aperto 0,80 m, beco sem
saída. Como rodar e como ler: `ros2_packages/robot_planning/README.md`.

**Julgado com dado em 29-07**: `docs/decisoes/008-nav2-planeja-nos-seguimos.md`
foi **ACEITA pelo dono em 29-07**. Decide o **Smac Hybrid-A\*** (único seguível
em toda a faixa de raio plausível — o Theta\* vai a 0/6 no pior caso), a ré
nascendo do planejamento em vez do susto (revisa a 007) e o **seguidor próprio**
(a movimentação da 005 é a única camada com física medida).

## 🎯 2026-07-29 (2ª leva) — A pilha obedece; o giro é que não entrega

`tools/banco/corrida_gazebo.py` (novo) responde o que o `ensaio.py` não
responde: **a pilha montada obedece?** Sobe o Gazebo headless e roda duas fases
na mesma simulação, uma corrida por perfil de zona morta. CSVs em
`docs/dados/2026-07-29-bancada-gazebo-{sim,real}.csv`.

- **10 de 10 alvos alcançados** (5 por perfil), sem órbita e sem travamento.
  Não absolve a navegação aposentada: confirma pela terceira vez que roteiro
  fechado não reproduz o que o dono acha clicando.
- **O giro entrega 79–86% do comandado, e piora subindo** — medido em malha
  ABERTA, com a placa contornada e o controlador de rumo fora do ar. A reta sai
  a 100,4% e esquerda/direita batem em 0,2%, o que exclui bitola e conversão: é
  escorregamento. **`wz_max = 1,0` vale 0,79 rad/s de verdade.**
- **O pivô só existe no perfil otimista**: 181 amostras de giro parado no perfil
  `sim`, **zero** em toda a fase B do perfil `real`. Confirma a aritmética de
  hoje de manhã (0,96 rad/s com zona morta 0,10; 1,48 com 0,15).
- **O piso de linear segura o BO-3**: zero amostras com roda pedida dentro da
  banda morta, nos dois perfis.
- **O ciclo "ré e anda" voltou no perfil pessimista** (5 entradas, período
  2,23 s; 1,85 m de caminho para um alvo a 0,40 m). É o defeito de 28-07, e
  **não será consertado** — vive no `goal_navigator`, já aposentado.
- ⚠️ Tudo isto é Gazebo com a boba do BO-4, que é um patim. O déficit de giro
  pode ser o mesmo contato falso. Não vale como medida do robô.
- **Consequência para a bancada do planner**: `minimum_turning_radius: 0.25`
  está otimista. O raio realizado (p5) foi 0,370 m no perfil sim e **0,463 m no
  real** — ele abre em relação ao pedido justamente por causa do déficit de
  giro. Como o raio mínimo **é** o argumento da comparação Theta\* × Smac,
  julgar com um valor só repetiria a forma de erro da bitola: a bancada
  pareceria boa e o robô pioraria. A bancada passa a rodar uma **faixa** de
  raio.

## 📐 2026-07-29 (3ª leva) — A régua da bancada do planner estava errada

`tools/planner/varredura_raio.py` (novo): 6 casos × 4 raios mínimos × 2
planners, headless, sem cliques. Antes de responder a pergunta, achou um defeito
na régua da bancada.

- **`mede()` lia cúspide de Reeds-Shepp como curva fechadíssima.** No caso real
  `perto_de_lado` com 0,46 m configurado, ela acusava raio 0,125 m, 181° de giro
  e ZERO inversões — as três erradas ao mesmo tempo, e **todas contra quem usa
  ré**. O caminho estava certo (frente · ré por 0,86 m · frente, arcos de
  ~0,41 m); a régua, não. Suavizador e planner foram descartados como causa
  antes do conserto, cada um com seu teste.
- Consertado partindo o caminho nas cúspides. **5 testes novos**, com geometria
  do caminho real. A bancada não tinha teste nenhum — foi assim que sobreviveu.
- **Trecho curto entre cúspides agora é pulado e CONTADO** (coluna `curt`), em
  vez de virar `raio_min = 0,00`. Dentro desse defeito havia um achado de
  verdade: com raio grande, o Smac **treme em cima do alvo** — no `bloco` com
  0,46 m são 4 inversões dentro de uma caixa de 9 cm, depois de 4,86 m limpos.
  Fica para o seguidor.
- ~~**Por consertar**: o `giro` de arco contínuo sai curto.~~ **FEITO na 4ª
  leva** (abaixo), e atrás dele havia mais dois defeitos.

### O veredito da varredura: o ranking não vira, ele se acentua

| raio que a máquina fecha | Theta\*: caminhos seguíveis | Smac: idem |
|---|---|---|
| 0,25 m | 4/6 | 6/6 |
| 0,34 m | 3/6 | 5/6 |
| 0,37 m | 3/6 | 6/6 |
| 0,46 m | **0/6** | 6/6 |

O Theta\* sai **idêntico nos quatro raios** (não conhece raio) — é a testemunha
de que a varredura mexeu só no que devia. Quem se move é a linha que ele precisa
cruzar. Nos dois casos "de lado" ele falha em qualquer raio: desenha reta
lateral, que só serve para robô que pivota. O Smac cobra caminho mais longo, e o
preço sobe com o raio (1,50× → 2,12× no alvo perto e de lado).

**A decisão 008 pode ser assinada sem esperar a zona morta**: a medida que falta
muda o tamanho da vantagem, não quem vence. Falta o julgamento do dono.

## 🧭 2026-07-29 (4ª leva) — O viés do giro, e dois defeitos escondidos atrás dele

O `giro` encolhia arco contínuo (90° lidos como 50°) e **favorecia o Smac na
comparação que ele arbitra** — o canto vivo do Theta\* tem vértice e era contado
inteiro. Consertado somando nos pontos crus, o que exigiu derrubar antes a
premissa da reamostragem: medido, o passo cru é limpo (Theta\* anda 0,05 m com
virada mediana de 0,00°; o Smac 0,086 m com virada máxima de 19,9°, que é o arco
no raio configurado). A justificativa antiga descrevia a cúspide, não ruído.

Somar no cru expôs dois defeitos que a reamostragem escondia:

- **Tocos de ponta**: o planner cola a pose exata de partida/chegada no caminho
  discretizado e sobra um segmento de 7,7 mm em cada ponta, injetando ±125,6°.
  Costurados fora por limite relativo (metade do passo típico). O comprimento
  não muda — o robô percorre o toco, ele só não define rumo.
- **Cúspide rasa**: a dobra geométrica nas inversões do `lado_1m` mede 147° e
  passava por baixo do limiar de 150°. A pose resolve: projetando o passo no
  rumo, o caminho é `+----------------+` — 1 passo à frente, 16 de ré, 1 à
  frente. A detecção passou a usar **pose quando existe, geometria quando não**
  (o Theta\* devolve orientação zerada — faixa de yaw de 0,0° em 144 pontos).

```
lado_1m @0.25    antes: giro 437°  inv 0        depois: giro 144°  inv 2
```

**O veredito não mudou** (a seguibilidade sai do `raio_min`, não tocado). O que
mudou é que a coluna do giro passou a servir: o Smac mexe MAIS o bico em todos
os casos com obstáculo (é o custo de curvar em vez de pivotar), e os dois zeros
do Theta\* nos casos de lado não são virtude — é reta lateral para um robô que
teria de pivotar. A ré do Reeds-Shepp aparece constante nos casos de lado: 2
inversões nos quatro raios. 294 testes verdes.

⚠️ **Próxima correção da régua, não feita aqui**: a reamostragem passa por cima
do canto vivo do Theta\* e devolve 0,37–0,39 m onde a virada é um canto
(curvatura infinita). O `raio_min` faz o Theta\* parecer MAIS seguível do que
é — o veredito de hoje é conservador, e distinguir canto de arco só pode
melhorar o lado do Smac.

## 🔙 2026-07-29 (7ª leva) — A ré volta a ser por gatilho (decisão 009)

O dono derrubou a seção 3 da 008 com razão de campo: ré planejada em robô com
Nav2 faz o robô ficar tentando entrar e seguir os trechos de ré. A bancada não
sabe arbitrar isso (ela desenha, não dirige), então mediu-se o custo de cada
saída antes de decidir.

- **Encarecer a ré não a elimina**: `reverse_penalty` de 2 a 40 não muda as
  inversões — nesses casos ela é geometricamente necessária.
- **Proibir a ré (Dubins) custa**: `bloco` fica sem caminho nos 4 raios, e os
  alvos "de lado" passam de 1,50–2,12 para 3,61–5,66 de desvio (~3 m de caminho
  para um alvo a 0,60 m). `porta`, `aperto` e `beco` não sentem.
- **Recuar NÃO salva o plano Dubins** — hipótese minha, testada e derrubada:
  recuar reto não muda o RUMO, e o alvo segue a 90° do bico.
- **O que sustenta a ré por gatilho é o PIVÔ**, e o robô 1 pivota. O robô 2 não,
  com os parâmetros de hoje.

**Decisão 009 aceita**: planner em `DUBIN`, ré por gatilho no seguidor, disparada
por SINTOMA (não progrediu) e não por geometria. Fica registrado que **a zona
morta reabre esta decisão**: se o pivô não existir, a 008 seção 3 volta com
número. É a terceira razão de peso para o item nº 1 da bancada.

**Fatia B do seguidor feita**: `ProgressoDeAvanco` (mede aproximação, não
velocidade — órbita tem velocidade e não tem progresso), orçamento de ré cego de
0,30 m enquanto o Mid-360 não estiver no modelo, e teto de tempo além do de
metros (se a pose não muda, o orçamento em metros nunca é gasto). 316 testes.

## 🚗 2026-07-29 (9ª leva) — A pilha inteira anda

`ros2 launch robot_motion pilha.launch.py sim:=true` — Gazebo + Nav2 + seguidor
+ movimentação, e o robô vai onde se clicar. Dois alvos verificados: 90° atrás
(15,2 s, 1,67x a reta) e através da porta de 0,90 m (12,3 s, 1,04x).

Quatro defeitos achados e corrigidos, três deles do Nav2 discordando de si mesmo:
a árvore padrão exige `spin`/`backup` (pivô que este robô não faz e a ré que a
009 tirou) → trocada pela `navigate_w_replanning_time.xml`; a segunda árvore
(`navigate_through_poses`) derrubava a subida → `navigators` restrito; e o
`controller_server` **não dirige mas ABORTA** — o detector de colisão dele
derrubou a navegação com o robô já do outro lado da porta.

**Inflação medida como alavanca**: 0,45 trava o robô DENTRO da porta de 0,90 m
(`Start occupied` no replanejamento, porque o vão inteiro fica inflado); 0,30
passa em 1,04x. Produção em 0,30, bancada em 0,45 (é o número dos 48 planos da
008) — divergência deliberada e travada em teste.

⚠️ **Anotado sem conserto**: o alvo a 90° atrás custa 1,67x, e o gatilho de ré
disparou **a 0,43 m do objetivo**. Ré perto da chegada é suspeita e é a primeira
coisa da próxima sessão. E a TF `map→odom` é fixa — vale no simulador (mundo e
mapa saem da mesma planta), **não vale no robô real**.

## 🧰 2026-07-30 — Kit de bancada pronto, e o robô do estágio como testemunha

**`tools/banco/sessao.py`**: o protocolo inteiro do `tools/banco/README.md` num
comando — os 6 passos em ordem, 11 corridas, pausando para reposicionar, tudo
numa pasta só com `ambiente.txt` (piso, bateria, commit) e `leituras.txt`. Chama
o `medir.py` depois de cada corrida, então o número sai ainda com o robô ligado.
Folha de campo: **`tools/banco/CHECKLIST_ROBO.md`**.

Duas defesas que o `ensaio.py` sozinho não tem:

- **conferência que bloqueia** — sem `/Odometry`, sem ouvinte no `cmd_vel`, ele
  recusa medir e diz por quê. CSV gravado com a base incompleta sai limpo e
  errado, e isso só se descobre em casa;
- **cutucão de sanidade** (`--checar --mexer`) — anda 2 s, gira 2 s e confere o
  **sinal**. Roda trocada na fiação dá robô que anda certo e gira ao contrário, e
  nenhum dos 6 ensaios acusa (eles medem magnitude).

Provado ponta a ponta contra o Gazebo headless, e no caso negativo também (pilha
derrubada → recusa). 380 testes verdes.

### O workspace do estágio (`ESTAGIO-2026/`, fora do git) — 3 hipóteses

Mesma máquina, pilha Nav2 de fábrica, **anda e faz SLAM** (o que o nosso ainda
não faz no robô), se perde, e recupera quase só de ré. Lido contra a nossa trena:

1. **Dois raios de roda contraditórios na mesma pilha**: `0.0425` no
   `diff_drive_controller`, `0.0825` no plugin de hardware. A conversão do driver
   (`rad/s ÷ 0,10472`) não usa raio, então quem fixa escala é o controlador —
   com a nossa medida de 0,080, a roda gira **~1,9×** mais que os m/s pedidos.
2. **`wheel_separation: 0.32`** contra os 0,270 medidos → gira ~19% a mais que o
   comandado. É o mesmo desvio que anotamos em 29-07 quando o 0,32 era nosso.
3. **O piso de velocidade do mux deles protege a reta e não o giro**: 0,10 m/s
   no linear, 0,15 rad/s no angular — que com bitola 0,32 são **0,024 m/s de
   roda**, 4× abaixo da faixa de zona morta plausível. O `Spin` do Nav2 decai
   até 0,4 rad/s (0,064 m/s de roda, também abaixo); o `BackUp` é linear puro e
   o piso o levanta sempre. **Hipótese: "só vai de ré" pode não ser a boba — pode
   ser que a ré seja a única recuperação que fisicamente acontece.** É a nossa
   BO-3 vista de fora, num robô que já roda.

Nenhuma é fato nosso. As três se resolvem com os **ensaios 2 e 4** — quarta razão
de peso para o item nº 1 da bancada.

## 📊 2026-07-31 — O protocolo passa a repetir, e a zona morta vira dente de serra

Véspera da ida ao robô. Duas perguntas do dono derrubaram partes do método.

- **Nenhum ensaio repetia.** As 11 corridas eram 11 condições diferentes — os
  "3" dos passos 3, 4 e 6 são três *valores*, não três tentativas. Todo número
  sairia com n=1, sem faixa. Agora são **27 corridas**: reta, curva e aceleração
  ×3 (decisão do dono: repete o que identifica erro), com `--repete 1` para
  encurtar se a bateria cair. Cada grupo imprime **média, faixa e dispersão** ao
  fechar, e delata quando uma corrida morreu e a repetição encolheu.
- **Uma corrida de controle, girada 180°** (passo 6). Não é repetição: média
  mata erro aleatório e **não mata erro sistemático**, então três retas do mesmo
  ponto medem o caimento do piso e a média sai confiante e errada. É a única
  corrida que separa robô de sala. Ponto 0 marcado com fita **e com o rumo**.
- **Zona morta virou DENTE DE SERRA**: sobe até sair do lugar, desce até parar,
  inverte o sentido, 4×. Uma corrida dá **4 saídas e 4 quedas**, nos dois
  sentidos, sem reposicionar — a repetição vive dentro dela. E mede dois números
  onde havia um: **saída** (do repouso, atrito estático — o do BO-3) e **queda**
  (já andando, menor — é ela que o piso de velocidade do seguidor precisa).
- **A primeira versão não cabia na sala**, e quem disse foi o teste: virando por
  tempo, o ensaio linear se afastava **5,25 m** contra trava de 3 m que mata a
  corrida. O dente passou a virar **no evento, não no relógio** — excursão de
  0,059 m e 4 dentes em 18 s (eram 160 s). É o único ensaio em malha fechada.
- **A taxa da rampa virou parâmetro nomeado** (`--rampa-seg`): o limiar é lido na
  primeira amostra que passa de `LIMIAR_PARADO`, então rampa mais rápida infla o
  número. No giro a taxa atual já infla ~0,011 rad/s — 10% do que se distingue.
- **`tools/banco/` ganhou 17 testes** (não tinha nenhum, que é como a régua do
  planner sobreviveu errada em 29-07). O do dente fantasma verificado por
  mutação. Suíte: **397 verdes** (eram 380).
- ⚠️ **O Gazebo provou o mecanismo, não o número**: lá o `ensaio.py` publica
  direto no `cmd_vel` do controlador e passa **por fora da placa fingida**, então
  os 0,023 m/s e 0,036 rad/s medidos são piso de detecção, não zona morta.

### O giro vira o ensaio 1, e o dente #0 é um caso à parte

**Ordem invertida a pedido do dono**: a zona morta de **giro** passa a ser o
passo 1. Ela responde a pergunta do pivô **diretamente** — o menor `wz` que gira
o robô parado *é* o limiar do pivô, sem converter por bitola. Pelo linear só se
chega lá por `2·zm/L`, confiando de novo num número medido. Some o risco de
sessão cortada (em 30-07 não se mediu nada): o que fica por último se perde.
O linear ganhou papel melhor — **conferência do passo 1**, já que os dois medem
o mesmo atrito por caminhos diferentes e têm de fechar por `2·zm/L`.

**O dente #0 destoa, e é física.** Na validação no Gazebo, dispersão de 82% no
passo 1 (#0 em 0,188 rad/s contra 0,035 dos demais). Só o #0 parte de repouso
longo; os outros, da pausa de 1 s. Atrito estático cresce com o tempo parado, e
a média dos quatro misturaria as duas condições **bem no número do BO-3**. O
`medir.py` separa e diz qual usar para cada caso, em vez de corrigir a média —
qual serve depende de quanto tempo o robô fica parado em operação, o que é
decisão de projeto.

### ✅ Fechado no mesmo dia: a conferência lê a calibração VIVA

`sessao.py --checar` pergunta ao `hoverboard_base_controller` **que robô ele
acha que está dirigindo** — `wheel_separation`, `wheel_radius` e os nomes de
roda, do nó vivo — e compara com a trena. **Delata, não bloqueia**, e grava
tudo no `ambiente.txt` com marcador `*** DIVERGE DA TRENA ***`.

Fecha o buraco de que o `ambiente.txt` gravava só o **commit**, que descreve o
fonte: quem dirige o robô é o `install/`. Sem esse registro, um limiar medido
não tem como voltar a ser velocidade de roda — vira número sem unidade.

Os nomes de roda entram junto porque é neles que vive a correção do **giro
espelhado**: a conferência diz `swap APLICADO` / `NÃO aplicado` **antes** de o
robô se mexer, provando que o rebuild pegou sem gastar bateria. Os dois estados
e o caso divergente foram verificados ao vivo contra o Gazebo.

A folha de campo virou **executável de ponta a ponta**: bloco "para quem for
conduzir", ordem fixa, cinco coisas que não se faz, e o swap de rodas como
script copiável (testado e revertido; **não commitado** — só entra no git
depois de o cutucão validar, decisão de 30-07).

### Aberto: `install/` velho envenena todos os limiares

A bitola e o raio da trena moram em arquivos que o `tracao.launch.py` lê de
`FindPackageShare` — da cópia **instalada**. `git reset --hard` troca o fonte e
não troca o `install/`. Se o build faltar, o robô sobe com 0,32/0,0825 e todo
limiar sai 18,5% enviesado, sem sintoma. A folha de campo ganhou o `colcon build
--packages-select hoverboard_driver` e um `grep` de conferência.

**FEITO no mesmo dia** (seção acima). Importava porque a comparação roda × lidar
é limpa na reta (raio de roda puro) mas **não separa bitola errada de
escorregamento** no giro: 0,32 num robô de 0,270 faz girar 18,5% a mais,
derrapar faz girar menos, e `1,185 × 0,82 ≈ 0,97` leria como "quase não
derrapa". Com a calibração gravada ao lado do CSV, os dois voltam a ser
separáveis em casa.

## 🎛️ 2026-08-01 — O atuador medido entra no simulador, e a decisão 005 cai

Sessão sem robô, em cima dos CSV de 31-07.

- **Confirmei os números da bancada refazendo as contas.** A zona morta de giro
  (0,091–0,096 rad/s) é firme: critérios de 2°, 3° e 5° dão 0,091/0,096/0,096,
  com os 4 dentes concordando. O patamar também: comando de 0,10 m/s virou
  **0,26–0,28 m/s de borda** nas quatro rajadas.
- **Fechei o modelo do atuador** com o que faltava: latência **0,273 s** (n=4),
  patamar **0,297 m/s de borda**, aceleração 0,435 m/s², desaceleração
  0,373 m/s², e a escala real do firmware **0,0372 rad/s por unidade** contra os
  0,10472 que o driver assume — **o driver superestima a roda em 2,8×**.
- ⚠️ **A "zona morta de giro" NÃO é atrito, é aritmética do driver.** A
  compensação dispara em `mx > 1.0`, o que dá `wz > 0,0621`; os 0,03 de
  diferença são a latência sobre a rampa. O número descreve o **sistema**, não a
  máquina, e se move se alguém mudar `deadband_speed`. **O atrito segue não
  medido.**
- ⚠️ **A pergunta do pivô foi dissolvida, não respondida.** Com a compensação
  ligada qualquer `wz > 0,062` pivota, na velocidade do patamar. A aritmética da
  folga de 4% ficou sem objeto: o pivô existe, o que não existe é controle da
  velocidade dele. **Mexe na premissa da decisão 009.**

### ⛔ A decisão 005 não sobrevive ao atuador medido

Passando a própria lei da 005 pelo modelo: de **1° a 180°** de erro de rumo, a
lei pede 8 valores distintos de `wz` e a placa entrega **um só** (2,204 rad/s,
2,2× o máximo que a lei pediria). A frenagem de rumo — resultado central da 005,
validado em 10 corridas de simulador — **não existe neste robô** com a
compensação ligada. O mesmo vale para `v = √(2·a_lin·dist)`, e o `v_piso` fica
sem sentido: não há velocidade abaixo do patamar.

**O contrato de que `cmd_vel` está em m/s é falso em toda a faixa que a navegação
usa** — e a movimentação inteira foi escrita sobre esse contrato.

### O simulador agora erra como o robô erra

`placa_simulada.py` reescrito: reproduz a conta do driver linha a linha, depois
aplica escala real, latência e assimetria por sentido. Três modelos —
`medido` (padrão), `cru` (compensação desligada) e `ideal` (fio).
`sim.launch.py placa:=…` e `pilha.launch.py placa:=…`; o `zona_morta:=` saiu.

Provado no Gazebo, mesma ordem nas duas placas:

```
comando      placa ideal      placa medida
0,10 m/s ->  0,200 m em 2 s   0,548 m
0,25 m/s ->  0,500 m          0,541 m
0,50 m/s ->  1,000 m          0,541 m   <- as tres iguais
```

**419 testes verdes** (eram 407), 8 novos travando o modelo — inclusive o do
colapso da lei da 005.

⚠️ **Tropeço meu, igual ao do laboratório:** a primeira prova deu medida ≈ ideal
porque havia **quatro `placa_simulada` órfãs** acumuladas, de lançamentos que
derrubei com `pkill` de padrão largo. Contar processo vivo antes de medir entrou
no procedimento.

## 📋 PARA IR AO ROBÔ: `docs/PLANO_TESTE_ROBO.md`

Escrito em 05-08. Responde o que testar, com que mapa e como rodar. O resumo
que decide a próxima sessão:

- ✅ **Testável JÁ, sem mapa nenhum**: compensador de rumo (reta e ré), pivô,
  `twist_mux` + teleop (freio de mão) e reflexo de colisão. Os quatro usam só
  o `/Odometry` do LIO e a nuvem do Mid-360.
- 🔴 **Bloqueado**: tudo do Nav2 (planner, seguidor, chegada com ângulo).
  E **não é falta de mapa** — é falta de **localização contra o mapa**. A
  `pilha.launch.py` publica uma TF `map→odom` FIXA, que só vale no simulador
  porque mundo e mapa saem da mesma planta.
- ⚠️ **O mapa do repo é sintético** (`GERADO por gera_pista.py`) e não
  corresponde a lugar nenhum. **O mapa do estagiário NÃO está neste repo** —
  o workspace dele está fora do git.
- ➡️ Saída recomendada: **SLAM online** (o mapa nasce enquanto anda, `map ≡
  odom` por construção). É a menor mudança que destrava, e mantém a 003.

## 🎯 O TRABALHO ATUAL: decisão 011 — malha fechada de rumo (o "PID")

Aprovada pelo dono em 04-08 ("focar tudo nesse PID"). Plano em 5 fatias;
registro completo em `docs/decisoes/011-malha-fechada-de-rumo-em-reta.md`.

- ✅ **Fatia 1 — reta e ré no Gazebo: FEITA E ACEITA (04-08, 5ª leva).**
  `compensador_rumo` (nó novo no `robot_motion`, lei pura em
  `lei_de_reta.py`): entra `cmd_vel` desejado, sai `cmd_vel` corrigido,
  yaw do `/Odometry` fechando a malha. ff medido (−0,817/−0,098) + PI
  (kp=1,0, ki=0,5). Bancada n=3 por sentido:

  ```
                   SEM              COM           critério
  frente         -0,82 1/m       -0,0025 1/m       <0,05    passa 20x
  ré             -0,11           -0,0003           <0,05    passa
  ```

  Dados: `docs/dados/2026-08-04-fatia1-compensador/`. O Ki existe para
  zerar o RUMO, não a curvatura (sem ele o robô anda reto, mas ~6° torto)
  — descoberto pelo próprio teste, travado em teste.

- ❌ **Fatia 2 — atraso de desliga na placa fingida: DESCARTADA na medição
  (04-08, 6ª leva).** O gap que a motivava ("sobrepasso 99° no Gazebo contra
  49° no robô") era artefato de comparar sobrepassos **a partir do pico**,
  que repartem diferente nos dois lados. O número que o pivô consome é o
  **giro total entre o comando zerar e o robô parar**, fase de empurrão
  incluída — e nele os dois batem:

  ```
              giro total do corte à parada      tempo até parar
  ROBÔ        102° / 114° / 124°  (~113°)       1,94–2,02 s
  SIMULADOR   117° / 120° / 117°  (~118°)       1,82–1,86 s
  ```

  ⚠️ **Condição de projeto que isso impõe à fatia 3**: o controlador do pivô
  pode consumir o **giro total** e a **detecção de parada**, mas **não a
  forma da frenagem** (pico, wz no meio do caminho) — a forma é onde o
  simulador ainda mente. Se o projeto precisar da forma, a fatia 2 volta.

- ⏳ **Fatia 3 — pivô por corte previsto** (desbloqueada pelo achado acima).
  NÃO é PID: a placa entrega um wz só, então a única alavanca é **decidir
  quando cortar**. Dado de partida: depois do corte o robô ainda varre
  ~113°, o que levanta a pergunta de projeto que a fatia tem de responder
  primeiro — **existe pivô menor que isso?**
- ⏳ **Fatia 4 — robô real.** Mesmo protocolo da bancada de 04-08, o dono
  conduz, `ensaio.py --topico /compensador_rumo/cmd_vel`. É quem julga de
  verdade: no Gazebo o ff é exato por construção; no robô a planta do dia
  difere do ff em até 21% e o integrador é quem paga.
- ⏳ **Fatia 5 — integração permanente** no módulo de movimento (launch,
  prioridade humana no mux, delator do BO-3 nesta camada).

## 🪞 Pendências abertas por 04-08

**Item 12 (calibração do simulador):** o passo 12a (arco) fechou, 12b (aceitação)
está medido e dentro da dispersão do robô. Ficou aberto:

- **12c — Dispersão**: o simulador dá 4% de frente contra 21% do robô, e **0%**
  na ré contra 37%. Determinista demais faz controlador parecer mais repetível
  do que vai ser. É decisão de projeto (injetar ruído, de que tipo), não foi
  tomada. Deixa para quando o controlador já estiver operacional.

- **Atraso de desliga da placa** (novo achado em 04-08, 3ª leva): a placa
  empurra ~0,51 s DEPOIS do comando zerar. O parâmetro `latencia` da
  `placa_simulada` modela o atraso de liga (aquele que trava por 0,27 s no
  arranque), não o de desliga. Com só a latência de liga, o Gazebo desacelera
  2× mais rápido que o robô (sobrepasso 94° contra 49°). Precisaria entrar na
  lógica do `cb` do nó, não é parametrização.

- **Teto de aceleração angular** (corrigido em 04-08, 3ª leva): `angular.z.max_acceleration`
  com valores baixos (0,3 na planta lenta) limitava o pico indiretamente. Com a
  planta normal (1,5) ele sobe de 0,45 para 2,25 rad/s e agora bate. Mas o
  sobrepasso sai 2× maior por falta do atraso de desliga.

- **Velocidade sustentada acima do patamar**: nunca foi medida acima de 0,838 m/s
  em nenhum dos dois lados (é onde o comando volta a ser proporcional).
  Recomputado mas não validado em hardware.

- **Mecanismo (BO-4)**: o arco entra disfarçado de assimetria de roda,
  porque a placa só tem rodas para escrever. Prova de que entra: 24,8% de
  assimetria necessária contra 11–12% medidos pelo encoder. O encoder
  simulado mente e **quebra no dia em que ligarem `open_loop: false`**.
  Reproduz o sintoma, não o mecanismo.

## ⏳ Próximos passos

**Primeiro, com o robô (virou prioridade — a movimentação depende destes
números e hoje eles são chute):**

1. ~~Medir `wheel_separation` e `wheel_radius` com trena.~~ **FEITO 07-29**:
   bitola 0,270 e raio 0,080, os dois já no URDF e nos três YAMLs. Em seu lugar,
   o novo item nº 1 é a **zona morta** — é ela que agora decide se o robô
   consegue pivotar (ver a nota superada acima), e a folga no melhor caso é de
   4%. Medir a rodinha da boba junto, se der.
2. ~~**PRIMEIRO: corrigir o giro espelhado** (bloqueio 07-30).~~
   🛑 **NÃO FAZER. O giro espelhado NÃO EXISTE** (08-04) — era o `atan2` do
   cutucão enrolando em ±180°. O swap foi aplicado (`368ea13`) e **revertido**
   (`595cf80`); **o estado atual, SEM swap, é o correto**. Aplicar aquele script
   quebra um robô que está certo. O `atan2` foi **consertado** em `7a0c364`, então
   o veredito de giro do `--checar --mexer` voltou a valer — se ele acusar agora,
   é para levar a sério, mas confirme com o olho de alguém atrás do robô antes de
   mexer em qualquer coisa.
3. ~~**Rodar a sessão de bancada** (27 corridas).~~ **PARCIALMENTE FEITO**: o
   banco está em **4 de 6** (zona morta de giro e linear em 07-31; `a_dec` e
   desvio de rumo em 08-04). Falta:
   - **passos 4 e 5** (curva e aceleração), que **precisam ser reescritos** — a
     varredura de velocidade cai inteira dentro do patamar da compensação e tem
     de subir acima de 0,838 m/s, o que exige espaço;
   - **filmar a boba** — o vídeo da traseira em corrida de ré é a única medida
     possível dela, e não foi trazido para o repo.
   ⚠️ Rodar pelo `sessao.py` **não serve como está**: ele não deixa passar
   `--espaco` por fora (padrão 4,0 m, que num robô que arca é excursão lateral
   demais) e o passo 6 dele está morto. Conduzir corrida a corrida pelo
   `ensaio.py` com `--espaco` pequeno, como em 08-04.
   O dono só roda; os CSV vêm por bundle/ssh (o NUC não tem autenticação no
   GitHub — ver a dívida de infra na entrada 07-30 do diário).
4. **IP do lidar** — confirmado em `192.168.1.169` na sessão de 07-30, igual ao
   config. Mas o Mid-360 pode ficar **mudo mesmo com o IP certo**: ele tranca a
   sessão de dado se o driver morrer no meio do handshake (não usar `kill -9`;
   SIGINT e esperar). Sintoma: pinga e ACKa, RX de ~6 pacotes/3 s, `/livox/lidar`
   mudo. Cura: power-cycle do lidar. Ver a entrada 07-30 (2ª leva) do diário.

**O simulador está pronto para rodar controlador:**

  Rodá-lo com `planta:=normal` (padrão; a_dec 1,5 rad/s²). A `lenta` (0,3)
  era propositalmente pessimista para validar a lei de frenagem da decisão 005
  — não é a planta do robô real. Com a normal o pico de wz sai 2,25 rad/s
  (robô tem 2,33), que agora bate. O sobrepasso fica 2× maior por falta do
  atraso de desliga, mas isso não vai descer antes de modelar esse atraso.

**Sem o robô, e agora urgentes (01-08):**

5. **Decidir o que fazer com a compensação de zona morta do driver.** É ela que
   cria o patamar e derruba a decisão 005. Três saídas, e nenhuma é de graça:
   (a) desligar e viver com a zona morta física — que só está bracketada entre
   0,25 e 0,5 m/s de borda, e se for isso a velocidade mínima do robô é rápida
   demais para chegar num ponto; (b) mapa estático melhor — **impossível**:
   nenhum preserva razão entre rodas E magnitude abaixo do limiar físico, é o
   que "zona morta" significa; (c) **malha fechada** por roda, que é a saída de
   verdade e o driver já tem o que ela precisa.
6. **`open_loop: false`** — é de graça e teria evitado a sessão perdida de 31-07.
   O driver já exporta posição **e** velocidade por roda (é de lá que saem
   `v_esq`/`v_dir` nos CSV); a odometria de roda hoje é o comando ecoado por
   escolha, não por falta de encoder. Trocar dá a comparação roda × lidar.
7. **Raio de inflação menor que o inscrito** (achado ao rodar a pilha em 01-08):
   `0.300` configurado contra `0.363` de raio inscrito do footprint da trena. O
   Nav2 reclama nos dois costmaps. É defeito, não aviso cosmético.
8. **Comparar a pilha inteira nas duas placas** — não rodou em 01-08 (`base_link`
   ausente na TF na subida). É o que mede o estrago de ponta a ponta.

**Pontas soltas de 08-04 (baratas, e cada uma já custou algo):**

8a. **`rajada_rodas.py` ainda não percebe que o robô sumiu.** O `ensaio.py` foi
   consertado (`7d7fad8`: aborta em `--sem-dado` e sai com código 1), o outro
   não. Foi esse defeito que gravou 7 corridas em branco com cara de sucesso em
   31-07. Mesmo conserto, mesmo teste.

8b. **Trazer o vídeo da traseira** filmado em 08-04. É a única evidência direta
   do garfo da boba e o único caminho para o critério (b) do BO-4. Vai por fora
   do repo (grande demais) — mas o `ambiente.txt` tem de dizer onde ele está.

8c. **Registrar piso e bateria** das nove corridas de 08-04, se ainda der para
   lembrar. Estão como `NÃO INFORMADO`, e sem eles a sessão não se compara com a
   próxima.

8d. **Reescrever os passos 4 e 5 do banco** antes da próxima ida ao robô. Os dois
   varrem 0,2 / 0,4 / 0,6 m/s e as três caem dentro do patamar — medem o driver,
   não o robô. A varredura tem de subir acima de 0,838 m/s, e isso muda o espaço
   que a sessão precisa. **Reescrever em casa é barato; descobrir no laboratório
   custa a sessão.**

8e. **`sessao.py` não conduz mais uma sessão deste robô.** Não deixa passar
   `--espaco` por fora (padrão 4,0 m, que num robô que arca é excursão lateral
   demais), o passo 6 dele está morto e os passos 4 e 5 medem o patamar. Ou
   ganha `--espaco`/`--dur` por fora, ou o protocolo passa a ser conduzido pelo
   `ensaio.py` corrida a corrida — que foi o que funcionou em 08-04.

**Assim que os dados da bancada chegarem:**

9. **Levantamento da camada de segurança do robô 1** (decisão 010) — ler
   `collision_monitor`, `motion_guard` e `unstuck_supervisor` arquivo por
   arquivo e dizer, com número, o que sobrevive ao Mid-360. Expectativa
   preliminar: o `collision_monitor` é config + geometria deste chassi; o
   `motion_guard` **encolhe** (parte dele existe para caçar "fantasma de vidro"
   do LD06); o `unstuck_supervisor` tem a ideia agnóstica e 1433 linhas moldadas
   em varredura planar. Roda DEPOIS dos dados porque os polígonos e limites a
   re-derivar dependem da zona morta e do `a_dec` medidos.

**Sem o robô:**

10. ~~Varrer o raio mínimo e julgar o planner.~~ **FEITO 07-29**: 48 planos, o
   ranking não vira entre 0,25 m e 0,46 m, e a **decisão 008 está escrita**
   (`docs/decisoes/008-nav2-planeja-nos-seguimos.md`) e **ACEITA pelo dono**.
   O Smac Hybrid-A\* é a escolha oficial; o Theta\* sai.
11. **Modelo 3D real do robô** no simulador (o dono vai levantar), com o
   Mid-360 no topo. É ele que troca a fonte de obstáculos do mapa estático
   para o sensor, e corrige footprint e bitola do modelo.
12. ~~**Calibrar o simulador contra o robô**~~ ✅ **FEITO 08-04 (2ª leva)** —
   **12a e 12b fechados; 12c segue aberto.** O simulador arca como o robô arca,
   medido com o mesmo instrumento nos dois lados (n=3 por sentido):

   ```
                   SIMULADOR        ROBO (04-08)
   frente          -0,817 1/m       -0,838 1/m
   re              -0,109           -0,113
   razao               7,5x             7,4x
   ```

   As três caem dentro da dispersão do próprio robô. Dado e leitura em
   `docs/dados/2026-08-04-aceitacao-simulador/`; entrada 08-04 (2ª leva) do
   diário. Antes disso ele dava −0,419 de frente e **zero** de ré.

   🔴 **O ponto de partida escrito abaixo estava errado**: dizia que o
   simulador reproduzia "~1×". Isso era de 28-07 — a reescrita de 01-08 já
   tinha assimetria por sentido. O real era **∞×** (ré perfeitamente reta).

   🔴 **Um defeito meu, que só a corrida no Gazebo pegou:** inverti os
   parâmetros com `curvatura = wz/v` (v com sinal) enquanto a bancada mede
   `Δyaw/caminho` (sem sinal). As duas convenções concordam de frente e **se
   opõem de ré** — o simulador arcava para o lado errado indo de ré, com a
   **suíte verde**, porque o helper do teste usava a mesma convenção do erro.
   Régua e objeto medidos com o mesmo viés, igual à régua do planner em 29-07.
   Conserto de raiz: os parâmetros do nó passaram a ser a **curvatura medida** e
   a assimetria de roda é **derivada** (`a = −2cL/(2s + cL)`), então o sinal da
   ré sai sozinho.

   ⚠️ **`rendimento_giro: 0.80`** (novo): o Gazebo entrega 80% do giro pedido —
   é derrapagem do contato simulado (bate com os 79–86% de 29-07), **não é do
   robô**. Mexer em atrito, massa ou planta do Gazebo obriga a **refazer a
   corrida de aceitação**.

   ⚠️ **`ensaio.py` ganhou `--topico`**, e no simulador ele é obrigatório: sem
   `--topico /cmd_vel_bruto` o ensaio publica direto no controlador e **passa
   por fora da placa fingida** — era a limitação anotada em 31-07 ("o Gazebo
   provou o mecanismo, não o número").

   **Plano original, para registro:**

   **12a. Injetar o arco dependente de SENTIDO** no `placa_simulada.py` (ou logo
   acima dele): `−0,82 1/m` indo para a frente, `−0,10 1/m` de ré. O atuador
   (patamar, latência de liga e de desliga, escala do firmware) **já está** lá
   desde 01-08 e ficou validado em hardware em 08-04 — o que falta é o corpo.

   ⚠️ **Isto entra FENOMENOLÓGICO, e o comentário no código tem de dizer isso.**
   Não dá para derivar o arco da geometria enquanto o BO-4 estiver aberto: a boba
   do simulador é um patim (multiplicar o atrito dela por 16 mudou o rumo em
   0,4%). Consequência que precisa estar escrita ao lado do número: o modelo
   **reproduz o sintoma, não o mecanismo**, então serve para desenvolver
   controlador e **não** serve para responder "e se" — outra carga, outro piso,
   ou depois de consertar a boba.

   **12b. Escrever a corrida de ACEITAÇÃO** contra
   `docs/dados/2026-08-04-bancada-robo/`. Mesma manobra, alvo numérico:

   ```
   frente:  razão caminho/afastamento  2,09x em 3,7 m de percurso
   ré:      1,08x
   razão frente/ré da curvatura:        8,3x
   ```

   Vira teste, não impressão — e o primeiro valor dele é medir **o quanto o
   simulador de hoje já está longe**, antes de mexer em qualquer coisa.

   **12c. Só então decidir se entra DISPERSÃO.** Medida em 08-04: 21% na
   curvatura de frente e 50% no `a_dec` efetivo, entre corridas idênticas. Um
   simulador determinístico devolve sempre o mesmo número e faz qualquer
   controlador parecer mais repetível do que vai ser — que é a forma clássica de
   o simulador enganar. Fica por último porque é decisão de projeto, não de
   ajuste.

   ⏳ **SEGUE ABERTO, e agora com número dos dois lados** (08-04, 2ª leva):

   ```
                    dispersao SIM   dispersao ROBO
   frente                4%              21%
   re                    0%              37%   (0,0002 contra 0,04 absoluto)
   ```

   O simulador é determinista demais. Decidir se entra ruído, e de que tipo,
   é decisão de projeto — não foi tomada.

   **O que o simulador NÃO vai cobrir, e é para estar escrito no `README` dele:**
   acima de **0,838 m/s** nunca foi medido (é onde o comando volta a ser
   proporcional); o **atrito real** segue não medido (os 0,095 rad/s são
   aritmética do driver, não atrito); e as nove corridas de 08-04 saíram com
   **piso e bateria não registrados**, então a rigor não se sabe a que condição
   os números pertencem.
13. **Seguidor próprio** por cima do plano do Nav2 — destravado pela 008, é a
   fatia grande seguinte. Carrot no plano, como no robô 1, por cima da
   movimentação da decisão 005.


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

  🟢 **06-08: o segundo candidato virou o mais forte, e por mérito próprio.**
  Há agora DOIS "S" distintos, com mecanismos diferentes, os dois medidos:
  · o de **27-07** (simulador), da rampa de desaceleração — `wz²/(2·a_dec)`;
  · o de **05-08** (robô real), de **tempo morto** — 0,94 s de atraso no laço,
    com a oscilação CRESCENDO 2,07× por meio-período.
  O segundo tem o que um artigo precisa: fenômeno medido no hardware, mecanismo
  confirmado por **duas rotas independentes** (a frequência da oscilação e a
  soma dos atrasos da placa), um conserto **projetado** a partir do número
  (ganhos 4,1× menores), uma alternativa clássica **implementada e comparada**
  (preditor de Smith, que perdeu), e uma **previsão falsificável** esperando o
  robô. E um achado de método que vale por si: o defeito era invisível em TODOS
  os modelos do projeto — Gazebo e teste unitário — porque os dois subestimavam
  o atraso.

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

  **Fecha quando:** (a) ~~ensaio 6 do banco rodado no robô real, ida × ré, com a
  boba filmada~~ **PARCIALMENTE FEITO 08-04** — ver abaixo; (b) a causa do garfo
  não alinhar identificada no modelo; (c) simulador reproduzindo o ângulo de
  boba medido no robô, ou a decisão 004 corrigida para dizer o que ele de fato
  reproduz.

  ✅ **08-04: o alvo de comparação existe, e é um número.** Ida × ré rodados no
  robô real com controle de piso (`n=2` cada): de frente ele arca com curvatura
  **−0,817 1/m** (raio 1,22 m), de ré **−0,098 1/m** (raio 10,2 m) — 8,3× de
  diferença, e **88% do arco de frente é o termo que só existe indo para a
  frente**. O arco está preso ao **corpo**, não ao mundo (as quatro corridas
  saíram com curvatura de mesmo sinal, com o corpo girado 180° entre os
  sentidos), o que exclui caimento de piso; e motor/placa fraca de um lado
  também está excluído, porque daria a mesma curvatura nos dois sentidos.

  **Isso vira o critério (c) em teste com número:** o simulador tem de
  reproduzir **8,3× de assimetria entre frente e ré**. ~~Hoje ele reproduz ~1×
  (ré e ida deram idênticas)~~ — essa frase era de 28-07 e já estava vencida.

  ✅ **CRITÉRIO (c) ATENDIDO em 08-04 (2ª leva)**: o simulador entrega **7,5×**
  contra **7,4×** do robô, com as curvaturas dos dois sentidos dentro da
  dispersão da máquina (`docs/dados/2026-08-04-aceitacao-simulador/`).

  ⚠️ **Mas o BO-4 NÃO fecha com isso**, e é importante não confundir: o arco
  entra no simulador **disfarçado de assimetria de roda**, porque a placa só
  tem rodas para escrever. Metade do arco de frente, no robô, **não está nas
  rodas** — a assimetria necessária é 24,8% e o encoder de 31-07 mediu 11–12%.
  O modelo reproduz o **sintoma**; o critério (b) (a causa do garfo não
  alinhar) segue intocado, e o preço é que **o encoder simulado mente** — o que
  quebra no dia em que ligarem `open_loop: false`.

  🔴 **06-08: o BO-4 ganhou um segundo sintoma, e ele é grande.** O S do robô
  não aparece no simulador **em ganho nenhum** — varrer `ki` de 1,0 a 0,0 não
  mudou uma única inversão, e a corrida longa (27 s, 8 m) deu 1 inversão e
  deriva final 0,0°. Duas razões estruturais, e a segunda é este BO: no
  simulador o ff cancela o arco por construção (mesmo −0,817 dos dois lados),
  **e a boba é um patim**. Oscilação de rumo puxada por roda boba arrastada não
  pode aparecer num modelo cujo contato de boba não participa da dinâmica.
  ➡️ Se a sintonia de 06-08 falhar no robô (amplitude continuar crescendo com
  os ganhos novos), o S deixa de ser assunto de laço e passa a ser **este BO**.

  ⚠️ **Falta ainda o vídeo da traseira** — foi filmado em 08-04 mas não trazido
  para o repo. É a única evidência direta do garfo, e sem ela o critério (b)
  não anda. **Virou item da próxima ida** (nº 6 da lista lá em cima): filmar a
  traseira DURANTE o S é o que liga os dois sintomas. E o dono confirmou, empurrando o robô **com a mão** e com ele
  desligado, que o desvio **se repete** — ou seja, o fenômeno não depende de
  acionamento, o que é um dado forte a favor de causa geométrica.
