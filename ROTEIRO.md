# ROTEIRO — o que fazer, em ordem, com critério de parada

> Escrito em 2026-08-20 depois de uma sessão longa e de saldo pequeno. Existe
> para que a próxima sessão comece medindo e não conversando. Cada item tem
> **objetivo**, **como rodar** e **critério numérico de aprovação**. Sem número
> de aprovação, o item não entra aqui.

---

## O OBJETIVO FINAL

O robô recebe um objetivo do outro lado da porta 2 e chega lá, **três vezes
seguidas**, sem bater e sem travar. Hoje ele não faz isso nem uma vez de forma
repetível.

Dois defeitos separam o robô disso:

| # | defeito | estado |
|---|---|---|
| A | **a porta**: chega, não atravessa, dá ré, repete | causa NÃO isolada |
| B | **o S do corredor**: serpenteia em trecho reto | causa localizada, não atacada |

---

## O QUE ESTÁ PROVADO (com número, e nada além disto)

```
o robo CABE na porta         corpo 0,455 x 0,433 m; vao efetivo 0,600 m
                             folga 7,2 cm por lado, centrado
                             comando reto na mao atravessa: y 18,20 -> 19,52

a maquina nao entrega v_max  com v_alvo cravado em 0,50, a velocidade real da
                             p50 0,26 e p90 0,32 m/s  (n=2247, robo, 20-08)

a porta 2 tem 0,600 m        medido no tunel inteiro, nao numa linha so.
                             Numa linha (y=19,0) da 0,70 e ESTA ERRADO

o plano quebra               emendas p50 11,3 graus a cada 32 cm no /plan cru;
                             2,6 graus depois do suavizador (18-08)

a mira fica curta 87% do tempo  inclusive no corredor reto, onde deveria
                             esticar (medido no CSV do robo, 20-08)
```

## O QUE É FALSO — não perseguir

```
"ele acelerou a 0,55 m/s"    NAO. Estava em RE (v_alvo -0,20). O v_alvo nao
                             passa de 0,50 por construcao
"a pose saltou 60 cm"        NAO. Era o dono carregando o robo de volta:
                             19 m em 15 s, trajetoria continua
"a porta nao cabe"           NAO. Essa conta usou a CAIXA do reflexo (0,555)
                             como se fosse o corpo (0,455)
"o reflexo e o gargalo"      NAO PROVADO. Saiu de 4 corridas num mundo de
                             Gazebo defeituoso (ver Armadilha 1)
```

---

## SESSÃO 1 — a porta, no Gazebo (não precisa do robô)

**Objetivo**: descobrir, com número, **quem** impede a travessia navegando.

```bash
ros2 launch robot_motion pilha.launch.py sim:=true gui:=false rviz:=false \
  bag:=false log_dir:=~/logs_robo2 \
  mundo:=$PWD/worlds/andar3porta2.sdf \
  mapa:=$PWD/maps/andar3todoalterado/andar3todoalterado.yaml \
  pose_x:=6.30 pose_y:=16.0 pose_yaw:=1.5708

# objetivo do outro lado da porta 2:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 7.0, y: 20.5}, \
   orientation: {z: 0.7071, w: 0.7071}}}}"
```

**Aprovação**: `y > 19,4` em **3 de 3** corridas.

**Se reprovar**, o veredito vem de um comando só:

```bash
bin/pause_budget.py ~/logs_robo2/freeze_capture.csv
```

| linha do orçamento | quem é o culpado | o que atacar |
|---|---|---|
| `collision` | o reflexo cortou comando vivo | a caixa (ver Sessão 1b) |
| `movimentacao_muda` | ninguém cortou; a lei não pediu | a lei de rumo |
| `vx_zona_morta` | comando abaixo do piso | a movimentação |
| `compensador_gap` / `placa_engoliu` | o comando morreu depois do mux | driver |

⚠️ **Rodar as duas condições**, `gargalo:=false` (default) e `gargalo:=true`
(liga o alinhamento de passagem). Elas mudam o resultado e a comparação é o
dado.

### Sessão 1b — SÓ se o orçamento disser `collision`

A caixa `PolygonStop` tem 0,555 m de largura num vão de 0,600: **2,2 cm de
folga por lado**. O corpo tem 7,2 cm.

⚠️ **A decisão 041 já encolheu essa caixa e o robô bateu a traseira**, com o
dono olhando, e o placar não melhorou (3/5 antes, 3/5 depois). Não repetir.
A caixa atual veta **0,8 s antes** da encolhida, e a placa segura a saída
0,52 s — essa antecedência é o que para o robô a tempo.

Se for atacar mesmo assim, a única forma que não repete a 041: encolher **só a
largura**, **só enquanto a passagem está ativa**, **só com o rumo já alinhado**,
mantendo o comprimento (que é a distância de parada) intacto. E medir a folga
da quina de trás em cada corrida.

---

## SESSÃO 2 — o S do corredor, no Gazebo

**Objetivo**: medir o S num trecho reto e ver se ele existe no simulador.
Nunca foi medido com o mundo do corredor real.

Mesmo comando da Sessão 1, com objetivo `(6,3 · 16,0)` partindo de `(6,3 · 6,0)`
— 10 m de corredor reto, sem porta.

**Aprovação**: `|desvio_lateral|` **p90 < 0,15 m** no trecho.

**A cadeia do S, já levantada** (não re-derivar):
1. o plano não é reto: emendas de 11,3° a cada 32 cm;
2. a mira fica curta 87% do tempo e amplifica cada emenda;
3. tempo morto de 0,94 s fecha o laço.

**O elo 2 é uma gangorra conhecida**: a régua que prende a mira curta
(`mudanca_de_rumo <= 3°`, passa só 14% das vezes) é a mesma que protege a porta.
Mexer nela sem rodar a prova da porta junto é o que faz uma sessão desfazer a
anterior. **Toda mudança na mira roda as DUAS provas.**

---

## SESSÃO 3 — no ROBÔ, e só depois de 1 e 2 aprovadas

**Objetivo**: confirmar no robô o que o Gazebo aprovou.

```bash
bin/sobe-robo            # a pilha, com o freeze_capture junto
# uma corrida: corredor -> porta 2 -> objetivo do outro lado
scp bara@<ip>:~/logs_robo2/*.csv docs/dados/AAAA-MM-DD-.../
bin/pause_budget.py <o freeze_capture.csv baixado>
```

**Aprovação**: atravessa a porta 2 em **2 de 3** corridas, sem encostar.

⚠️ Bag `--all-topics` **desligado**. Ele grava 560 MB/min, travou o LIO em
20-08 e fez a pose saltar 60 cm com o robô parado. A cadeia em CSV custa alguns
kB/s e cobre o diagnóstico.

---

## SESSÃO 4 — a pergunta grande, ainda sem dono

**Por que a máquina entrega 0,26 m/s quando se pede 0,50?** É metade do teto,
está medido com n=2247, e nunca foi investigado. Afeta tudo: tempo de missão,
margem de parada, e a conta de qualquer lei que suponha o `v_max`.

Suspeitos, em ordem: o `ganho_wz`/compensador comendo linear em curva; o
patamar da placa; a bateria. **Nenhum medido.**

---

## ARMADILHAS — cada uma custou horas

1. **Mundo de Gazebo gerado por `map2world.py` sem `--caixas`**: a colisão é um
   mesh côncavo e **não segura o robô** — ele escala a parede (medido:
   `z = 0,068` no meio da porta). O sintoma é idêntico ao defeito real: chega
   na porta, não passa, dá ré. Use `--caixas --recorte`.
2. **Medir vão numa linha só**: a porta 2 dá 0,70 m em `y=19,0` e **0,600 m** no
   túnel inteiro. Medir sempre o túnel.
3. **`movimentacao.yaml` é do ROBÔ; `movimentacao_sim.yaml` é do GAZEBO.**
4. **`ros2 node list` mente** com o daemon travado — conferir por processo.
5. **Comparar `map` e `odom` só com o robô parado**, ou pareado no tempo.
6. **Contar EPISÓDIOS, não amostras** — o CSV é a 20 Hz.
7. **Derivar velocidade da pose com janela ≥ 0,3 s e COM SINAL.** A pose
   distinta chega a ~4,5 Hz; janela curta explode, e módulo numa virada
   ré→frente inventou os "0,55 m/s".
8. **Matar a pilha exige o padrão completo** e conferir por processo depois.
9. **O robô não alcança o GitHub**: deploy é
   `git push bara@<ip>:/home/bara/Controle_robo_livox main:main`.

## REGRA DE MÉTODO QUE ESTA SESSÃO COMPROU CARO

**Antes de estudar um defeito numa bancada nova, prove a bancada.** Uma corrida
de controle num cenário que já funcionou (`pista_obstaculos`, ida em ~71 s)
custa 2 minutos e teria economizado quatro corridas e meia sessão gastas
estudando um robô que estava montado em cima de uma parede.
