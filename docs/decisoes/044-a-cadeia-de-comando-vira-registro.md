# 044 — A cadeia de comando vira registro

**Data**: 2026-08-20 (2ª leva, dev; prova de ponta a ponta no Gazebo)
**Status**: instrumentação **ligada e provada**; a pergunta que ela existe para
responder continua aberta até a próxima corrida no robô.
**Toca**: `robot_nav/freeze_capture.py` (tópicos e tipos vira parâmetro),
`robot_motion/launch/pilha.launch.py` (o nó entra na pilha),
`bin/pause_budget.py` (limiares do robô 2 e categorias novas),
`robot_nav/package.xml` (`nav2_msgs`).
**Vem de**: duas perguntas que a sessão de 20-08 no robô deixou sem resposta
**possível**, e não por falta de tentativa.

---

## 1. As duas perguntas, e por que nenhum CSV existente as responde

Na porta 2, medido no `seguidor_2026-08-20_183507.csv`:

```
travessia 1 (t=160-192)   32 s dentro da porta
                          v real MEDIANA  +0,01 m/s   com v_alvo = 0,50
                          |erro de rumo|  p50 23,6°  max 51,5°
                          |wz| real       p50  2,9 °/s
```

A lei pede giro com 50° de erro na mão e o robô não gira. Pede 0,50 m/s e o
robô anda 1 cm/s. **E o CSV do seguidor não pode dizer por quê**: ele grava o
que o seguidor PEDE. O pedido morre em algum ponto da cadeia, e a cadeia
inteira estava fora do registro:

```
path_follower ──(rumo_alvo, velocidade_alvo · Float64)──▶ heading_controller
   ──/auto_vel_raw──▶ collision_monitor ──/auto_vel──▶ twist_mux
   ──/compensador_rumo/cmd_vel──▶ compensador ──▶ atuador
```

Três explicações incompatíveis cabem no mesmo dado: (a) a lei de rumo não
converteu o erro em comando; (b) o comando saiu abaixo da zona morta;
(c) o reflexo de colisão cortou. **Sem os elos gravados, escolher entre elas é
chute** — e foi o que aconteceu: 20-08 registrou o reflexo como "INOCENTADO por
medida" com base em 4 disparos vistos num monitor ao vivo, que é uma amostra,
não um registro.

## 2. O que já existia, e por que estava mudo

Nada disto precisou ser escrito: `freeze_capture` (grava a cadeia) e
`bin/pause_budget.py` (atribui cada segundo parado a UMA camada) existem no
repo desde o robô 1. **Chegaram mudos aqui, e mudos em silêncio.**

1. **Tipo errado.** A cadeia do robô 2 é `TwistStamped` inteira
   (`twist_mux.yaml: use_stamped: true`, `collision_monitor.yaml:
   enable_stamped_cmd_vel: true`). O coletor assinava `Twist`. Não é erro de
   DDS: é tipo diferente, o casamento não acontece, e o resultado é um CSV com
   cabeçalho e nenhuma linha — que se lê exatamente como "não aconteceu nada".
2. **Tópicos que não existem aqui.** `follow_vel`, `auto_vel_pre`,
   `unstuck_vel`, `motion_guard/state` são do robô 1. Neste robô o seguidor
   fala rumo+velocidade em `Float64` e quem abre a cadeia de twist é o
   `heading_controller`.
3. **Não subia.** O nó está na `nav2.launch.py` do `robot_nav` — a pilha do
   robô 1. O robô 2 sobe `robot_motion/pilha.launch.py`, e lá ele não estava.
4. **Limiares herdados.** `bin/pause_budget.py` classificava com `CMD_WZ =
   0,50`, casado com a zona morta de giro 1,7 do skid-steer. Aqui `wz_max` é
   1,00 e a zona morta da roda é **0,0178 m/s** (medida 31-07, decisão 020):
   um pedido de 0,3 rad/s — comando de verdade — contava como "ninguém pediu",
   e a culpa caía na camada errada. É o `CLAUDE.md` avisando em letras
   grandes: *"o diferencial gira fácil: calibrar do zero, não herdar"*.

## 3. A alternativa descartada: mexer no seguidor

A tentação era ajustar a mira, o ganho ou a caixa do reflexo — havia hipótese
pronta (a mira trava em 0,37 m dentro da porta, pelo gate de espaço
`folga_min = 0,60`). **Ela foi testada primeiro, e caiu**: a taxa de variação
do `rumo_alvo` é p90 **40°/s no corredor livre** contra **37-40°/s na porta**.
Igual. A mira curta não é o diferencial.

E é o mesmo erro que a sessão anterior registrou como o que mais custou: três
correções seguidas na régua que LÊ o plano, enquanto o defeito estava em outra
camada. **Instrumentar antes de sintonizar** é a escolha desta decisão, e o
preço dela é uma sessão sem conserto novo.

## 4. O que muda, em uma linha por peça

- `freeze_capture`: tópicos, tipo (`stamped`), `odom_topic`, `plan_topic`,
  `scan_topic` e o estado do reflexo viram parâmetro, com o **default na cadeia
  do robô 2**. Passa a gravar `/collision_monitor_state` — só as TRANSIÇÕES,
  senão 20 linhas/s de `DO_NOTHING` afogam o arquivo.
- `pilha.launch.py`: o nó sobe junto, com `out_dir` no mesmo `log_dir` das
  corridas.
- `pause_budget.py`: limiares do robô 2; categorias novas `humano`,
  `compensador_gap`, `placa_engoliu` e — a que interessa —
  **`movimentacao_muda[estado do reflexo]`**, que é o nome de "ninguém cortou:
  a movimentação não pediu".

⚠️ **Custa alguns kB/s**, contra os 560 MB/min do bag `--all-topics` que travou
o LIO em 20-08 e fez a pose saltar 60 cm com o robô parado. Corrida com o bag
DESLIGADO passa a ter a cadeia mesmo assim.

## 5. A prova, e o que ela não prova

No Gazebo, pilha completa (`pilha.launch.py sim:=true`, bag desligado):

```
/auto_vel_raw · /auto_vel · /compensador_rumo/cmd_vel
/hoverboard_base_controller/cmd_vel · /cmd_vel_bruto · odom   TODOS gravando
collision_state   APPROACH:PolygonApproach ... STOP:PolygonStop   gravado
```

E o `pause_budget` lendo a corrida inteira acertou o diagnóstico sem ajuda:
296,7 s de `movimentacao_muda` — que é o que de fato houve, porque o planner
respondeu `no valid path found` ao alvo que pedi e o seguidor nunca comandou
nada.

⚠️ **O que a prova NÃO diz**: nada sobre a porta. O Gazebo não tem o batente
que prende o robô, e o dono já disse que a fidelidade dele não alcança este
defeito. Isto aqui é prova de **fiação** — de que o registro existe e casa. A
resposta vem da primeira corrida no robô.

⚠️ E a alternância `APPROACH ↔ DO_NOTHING` a cada 30 ms que aparece no CSV da
prova é **artefato do teste**: injetei comando por cima do `heading_controller`,
que publicava zero, e o reflexo respondeu a cada um. Não é trepidação do
reflexo — não leia como achado.

## 6. Como usar, na próxima sessão no robô

```
bin/pause_budget.py ~/logs_robo2/freeze_capture.csv
```

Na porta, a linha do orçamento decide entre as três explicações do §1:

```
collision[...]        o reflexo cortou      -> a caixa é o assunto
movimentacao_muda     ninguém cortou        -> a lei de rumo é o assunto
vx_zona_morta         comando fraco demais  -> o piso da movimentação é
```
