# Ida e volta do robô 3 no Gazebo — 29-09-2026

Sessão pedida pelo dono para dirigir o robô 3 pelo RViz num percurso de ida e
volta. PC de desenvolvimento (`luiz-santarosa-750XGK`), robô e lidar físicos
**desligados**, branch `etapa6-pilha-robo3` no commit `350f6dd`, domínio ROS 50.
Os dois objetivos foram clicados no RViz. **Não** é corrida dos validadores das
decisões 060 ou 061, e **não** é o A/B da 062 — é a linha de base dele.

Pilha: `robo:=3 sim:=true gui:=true rviz:=true localizacao:=fixa`, mundo
`worlds/pista_obstaculos.sdf`, pose inicial (2,000; 5,000).
⚠️ `freio_linear` estava **LIGADO** — esta sessão é o lado **A** do A/B.

## Resultado

| | corrida 1 (ida) | corrida 2 (volta) |
|---|---|---|
| duração com goal ativo | 51,04 s | 61,94 s |
| partiu de | (2,000; 5,000) | (11,051; 1,558) |
| **pose no `SUCCEEDED`** | (11,067; 1,583) | (2,029; 4,747) |
| **`vx` no `SUCCEEDED`** | **+0,298 m/s** | **+0,282 m/s** |
| distância até o `SUCCEEDED` | 13,259 m | 13,266 m |
| **pose de repouso** | (11,051; 1,558) | (2,013; 4,812) |
| repouso em | +1,492 s | +1,578 s |
| distância até o repouso | 13,484 m | 13,510 m |
| `vx` máximo | +0,366 m/s | +0,366 m/s |
| `vx` mínimo | **−0,245 m/s** | **−0,219 m/s** |

⚠️ **A pose do `SUCCEEDED` não é onde o robô parou** — ele ainda anda a ~0,29 e
~0,28 m/s nesse instante. As duas linhas estão separadas de propósito: a
distância de 13,484 / 13,510 m **inclui o assentamento** posterior, e é maior
que a percorrida até o `SUCCEEDED`. "Repouso" aqui tem definição: parado
(`|vx| <= 0,005 m/s`) por 0,30 s seguidos — ver `analise.json` e o script.

As duas fecharam. Avaliação do dono: *"se ele se mover assim na vida real é o
melhor q tivemos até hoje"*, com a ressalva de que dá para ficar mais "clean".

## 1. O rebote na parada — reprodutível nas duas

Tomando como zero o instante em que `goal_active` cai:

| | corrida 1 | corrida 2 |
|---|---|---|
| comando negativo em `/cmd_vel_bruto` | −0,500 m/s, 11 amostras | −0,500 m/s, 10 amostras |
| duração do contra-torque | 0,52 s | 0,51 s |
| robô cruza o zero | +0,642 s | +0,798 s |
| pico de ré | −0,245 m/s | −0,219 m/s |
| para de vez | +1,492 s | +1,578 s |
| **recuo** | **9,6 cm** | **8,7 cm** |

A entrada do compensador (`/compensador_rumo/cmd_vel`) estava em **zero** em
toda essa janela: o −0,500 é do `FreioLinear` da 038, não do Nav2 nem do
seguidor. É a medida que originou a **decisão 062**.

### O freio atuou CINCO vezes na sessão, não duas

| # | t (epoch) | duração | onde |
|---|---|---|---|
| 1 | 1790689350,182 | 0,52 s | chegada da ida |
| 2 | 1790689943,868 | 1 amostra | durante a volta, fora de parada |
| 3 | 1790689958,076 | 0,43 s | **início do `PolygonStop`** |
| 4 | 1790689964,701 | 1 amostra | **fim do escape, ainda no `PolygonStop`** |
| 5 | 1790689998,170 | 0,52 s | chegada da volta |

⚠️ **Contar as linhas de `FREIO LINEAR` no `launch.log` dá 7, e 7 está errado**:
a mensagem tem `throttle_duration_sec=0.5`, então um episódio de 0,52 s aparece
duas vezes. Os episódios acima vêm dos **dados** — amostras negativas
consecutivas em `/cmd_vel_bruto` com a entrada em zero —, não do log.

## 2. 🔴 O freio atua no `PolygonStop`, não só na chegada

Este é o achado que a linha de base acrescenta, e ele **confirma a ressalva da
revisão** de que desligar o freio mexe em mais coisa que a parada final.

Houve **um** `PolygonStop` na sessão, na **volta**, na segunda porta (o aperto
de 0,80 m em `x = 8,9…9,1`). Ele durou **6,98 s** — pior que os 5,282 s da
sessão de 28-09.

🔴 **Medir os 6,98 s inteiros não serve**: essa janela mistura quatro coisas
diferentes. Separadas, com as réguas do script:

| fase | quando | quanto |
|---|---|---|
| 1. **avanço residual** — a planta ainda indo para a frente depois do corte | do STOP até +0,728 s | **14,1 cm para a frente** |
| 2. **ré do freio** — do cruzamento do zero até assentar | +0,728 s → +1,587 s | **8,2 cm para trás** |
| 3. **escape frontal** do `path_follower`, deliberado, ainda dentro do STOP | começa ~+5,1 s, termina +6,63 s | 0,21 m em 1,3 s |
| 4. **segunda atuação do freio**, ao cortarem o escape | +6,628 s | robô a +0,27 m/s |
| — | STOP libera | **+6,982 s** (resultado secundário) |

O reflexo zerou `/auto_vel` com o robô a +0,30 m/s **dentro do vão da porta**.
O robô ainda avançou 14,1 cm por inércia, o freio então o jogou 8,2 cm para
trás, o `path_follower` o desencalhou com um escape reto — e quando o escape
foi cortado, **o freio atuou de novo**, agora contra o próprio escape.

⚠️ **Isto corta para os dois lados no A/B da 062**: sem o freio não haverá a ré
da fase 2 nem a fase 4, mas também não haverá nada cancelando a retenção da
placa — o avanço residual da fase 1 pode ficar **maior** que 14,1 cm, e ele é
justamente o que empurra o robô para dentro do polígono. É exatamente por isso
que o A/B tem de repetir esta passagem, e não só uma parada em área livre.

Houve **dois escapes retos** do `path_follower` na sessão (0,21 m em 0,8 s e
0,21 m em 1,3 s); o segundo é a fase 3 acima.

⚠️ **Limite deste dado para inferir causa**: a volta começou onde a ida
terminou, então **o próprio freio da ida escolheu a pose de entrada na porta**.
Para testar a porta causalmente, cada tentativa tem de começar da **mesma pose**
perto do destino da ida — não encadeada. Está no protocolo da 062 §7.

## 3. Qualidade do seguimento (2 009 amostras)

| métrica | mediana | p90 | máximo |
|---|---:|---:|---:|
| `|erro_rumo|` | 0,1902 rad (10,9°) | 0,5019 rad (28,8°) | 3,0632 rad |
| `|desvio_lateral|` | 0,0437 m | 0,0899 m | 0,1425 m |

Estados: `seguindo` 98,0 %, `re` 2,0 % (41 amostras).

**O desvio lateral é bom** — mediana de 4,4 cm, nunca passou de 14,3 cm. O que
sustenta a impressão de "não tão clean" é o **rumo**, não a trajetória.

### Zigue-zague

| | corrida 1 | corrida 2 |
|---|---:|---:|
| inversões de sinal de `wz` | 11 | 9 |
| por minuto | **12,9** | **8,7** |
| `\|wz\|` mediana | 0,553 rad/s | 0,552 rad/s |
| `\|wz\|` máximo | 1,712 rad/s | 1,897 rad/s |
| amostras com `\|wz\| > 1,5` | 7,9 % | 7,3 % |

Uma inversão de giro a cada ~5 s é o candidato mais provável para o "poderia
ficar mais clean". 🔴 **Isto é observação, não diagnóstico**: não foi
investigado se vem do ganho da malha de rumo, do lookahead do seguidor, ou de
o plano do Smac ser naturalmente ondulado. Fica como pergunta aberta, depois
do A/B.

## 4. Outras notas

- Em `t = 525,17 s`, com o robô **parado** entre as duas corridas, o monitor
  registrou `Robot to stop due to invalid source` e voltou ao normal 0,13 s
  depois. Não afetou corrida nenhuma; fica registrado.
- Na subida, a **primeira** consulta de `ros2 action list` respondeu ausente e
  a segunda, segundos depois, listou `/navigate_to_pose`. Foi cedo demais, não
  defeito — mas uma leitura só não é pré-voo.

## 5. Evidência

Nesta pasta, em `ros_logs/`:

- `freeze_capture.csv` — **63 816** amostras: `odom`, `/auto_vel`,
  `/auto_vel_raw`, `/compensador_rumo/cmd_vel`, `/cmd_vel_bruto`,
  `/hoverboard_base_controller/cmd_vel` e os quatro `goal_active`;
- `freeze_diag.csv` — **3 970** amostras de diagnóstico periódico;
- `seguidor_2026-09-29_103958.csv` — 2 009 amostras do seguidor;
- `launch.log` — bringup, objetivos, escapes, o `PolygonStop` e o teardown;
- `perfil_nav2.yaml` e `perfil_collision_monitor.yaml` — parâmetros
  materializados da corrida.

E fora de `ros_logs/`:

- `analise.json` — a saída do script, com as réguas usadas gravadas dentro;
- `ORIGEM_SHA256SUMS` — hashes dos arquivos na pasta original, MCAP e
  `metadata.yaml` inclusive.

**As fórmulas estão versionadas** em `tools/analise_corrida/freio_e_porta.py`,
que é o mesmo instrumento que vai julgar o A/B — ele fixa o limiar de "parado"
(`|vx| <= 0,005 m/s`), o tempo de repouso (0,30 s), o que separa dois episódios
de freio (0,30 s de buraco), a janela depois do objetivo (6,0 s) e de quais
tópicos sai cada número (`/cmd_vel_bruto` para a saída, `/compensador_rumo/cmd_vel`
para a entrada, `odom` para pose e `vx`). O `wz` da seção 3 sai de
`/cmd_vel_bruto`.

⚠️ `launch.log` está versionado com `git add -f`: a raiz ignora `*.log`, e sem
o `-f` ele não chegaria a um clone novo — e é nele que estão os objetivos, os
escapes, o `PolygonStop` e o teardown.

🔴 **O bag MCAP bruto fica FORA do Git**: `~/sessao-robo3/20260929_103954/`,
**10 132 979 782 bytes** (9,4 GiB), 794 s, ~2,76 milhões de mensagens, dois
`/goal_pose`. SHA-256
`59e849eade2cac77344e7a179500245de4747fd703ff5d810540a27305097c7d`. O arquivo
foi posto em **somente-leitura** depois do hash; nada foi derivado dele antes
disso. As análises acima saíram todas dos CSVs — o MCAP fica reservado para
plano, costmap e a segunda porta.

⚠️ `--all-topics` grava a nuvem do Livox simulado e custa **~600 MB/min**. Para
corrida longa, restringir os tópicos ou vigiar o disco.

## 6. O que esta sessão NÃO prova

- **hardware** — nada aqui tocou o robô físico;
- **a 062** — esta é a linha de base com o freio LIGADO; o lado B não rodou;
- **a placa do robô 3** — o simulador usa `placa:=medido`, o atuador do robô 2;
- **a causa do zigue-zague** — medido, não diagnosticado.
