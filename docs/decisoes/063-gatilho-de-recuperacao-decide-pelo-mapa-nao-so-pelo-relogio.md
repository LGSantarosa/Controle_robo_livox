# 063 — O gatilho da recuperação decide pelo mapa, não só pelo relógio

**Data**: 2026-09-29 (PC de dev; robô e lidar DESLIGADOS, Gazebo com RViz)
**Status**: aplicada na branch `etapa6-pilha-robo3`. A 1ª leva **foi corrida e
reprovou parcialmente** (§4); a 2ª leva (o portão `near_mapped`) está
**implementada e testada, mas ainda NÃO corrida**
**Toca**: `lei_de_seguimento.py` (`ProgressoDeAvanco.atualiza` ganha teto por
chamada; função nova `mapa_ocupado`), `path_follower.py` (dois métodos novos,
cinco parâmetros novos, o aborto do desencalhe), `perfil_robo3.yaml` e
`test_perfil_robo3.py` (classificação), `test_lei_de_seguimento.py` e
`test_gatilho_rapido.py` (18 testes novos)
**Não toca**: o `collision_monitor` e seus polígonos, as manobras em si (ré,
escape reto, pivô), os tetos `re_max_seguidas`/`re_teto_s`, Nav2, mux, o freio
da 062
🔴 **ERRATA (revisão da mesma tarde): "não toca o robô 2" era FALSO** e saiu
desta linha. O `re_parado_s_mapeado` nasceu como default do nó e o perfil do
robô 2 não sobrescreve `path_follower` — o robô 2 passou a decidir em 2,0 s
junto. Achado R1 em `docs/ROBO3_REVISAO_CRUZADA.md`, **aberto**, e bloqueador da
corrida da 2ª leva junto com o R2 (a mudança do `/scan`, sem teste e sem
autorização).

---

## 1. Contexto

Pedido do dono, depois da corrida de 29-09 de manhã: *"vc já pode observar como
podemos fazer ele não travar tanto tempo antes de decidir se ele pode ou não
andar novamente"*, com a referência explícita: *"O robô 1 faz exatamente o q
quero, toda a lógica de retomada, de freio, tudo, menos a parte de segurança a
humanos"*.

A queixa é de LATÊNCIA DE DECISÃO, não de manobra. Medido na corrida da manhã
(`docs/dados/2026-09-29-robo3-sem-freio-salinha/`): `PolygonStop` na porta e a
primeira recuperação **9,91 s** depois; a segunda esperou mais **7,73 s**.

## 2. O que o robô 3 já tinha, e o que faltava

As manobras foram herdadas inteiras e funcionam: ré medida pelo vão traseiro,
escape reto de 20 cm e pivô de último recurso. O que faltava era a ESTRATÉGIA
DE DECISÃO em volta delas. Levantamento contra o
`unstuck_supervisor` do robô 1:

| estratégia do robô 1 | robô 3 antes |
|---|---|
| `stuck_timeout_mapped` 2,0 s (bloqueio à frente no `/map`) | não existia |
| `near_mapped` (parede mapeada a <0,6 m, QUALQUER lado) | não existia |
| `escalate_after` 2 → giro no lugar | **continua não existindo** (§7) |

## 3. Por que `re_parado_s` não podia simplesmente cair

Os 4,0 s são medidos, e contra um defeito real: a ré dura 1,6–2,8 s e recua
0,30 m, então relógio curto **rearma a ré antes de o robô ter tempo físico de
aproveitar a anterior** — realimentação positiva, e o sintoma é um robô que dá
ré à toa estando apontado certo. Havia teste travando o par.

A saída não é baixar o número: é **separar os dois trabalhos que um número só
fazia**.

    primeira decisão   pode ser rápida, se o bloqueio já é CONHECIDO
    rearme entre rés   continua nos 4,0 s, sempre

A fronteira escolhida é `res_seguidas == 0`, e ela não é arbitrária: é a mesma
conta do teto estrutural contra a fuga. `res_seguidas` só volta a zero quando o
robô bate a distância que tinha ANTES da ré — isto é, quando aquela ré pagou o
que gastou. Enquanto não pagou, o teto cheio vale, e o que a medida de 12-08
protege fica intacto.

## 4. A 1ª leva foi corrida, e o número dela é o que produziu a 2ª

Corrida das 15:26 (`~/sessao-robo3/20260929_152605-gatilho-rapido/`), ida e
volta pelo RViz, as duas com `Goal succeeded`. **RTF medido em 0,99** antes de
subir o objetivo — ou seja, o "0,5x" da manhã era carga da máquina, e não
configuração; nada foi mexido no Livox simulado.

Contagem das paradas, pelo log:

| causa | ida (72,3 s) | volta (86,0 s) |
|---|---|---|
| `STOP:PolygonStop` | 0 | 2 |
| `STOP` por nuvem do Livox VELHA | 6 | 7 |

🔴 **Treze das quinze paradas não foram o reflexo: foram o Livox chegando
atrasado** (`Latest source timestamps differ on 1,0–1,4 s. Ignoring the
source.`, acima do `source_timeout: 1.0`), com 81 avisos de nuvem velha, 336 de
costmap sem nuvem e 33 limpezas inteiras do costmap global. Avaliação do dono:
*"meu pc n aguenta ficar simulando direito esse lidar"*. Isso é limite de
máquina, está registrado como tal, e **nenhuma lógica de retomada o conserta** —
o robô para porque o sensor parou.

As duas paradas que SÃO deste assunto, na porta 2:

    1ª  PolygonStop em +28,4 s  ->  liberou SOZINHA em 4,34 s
    2ª  PolygonStop em +33,2 s  ->  escape reto em +40,1 s  =  6,84 s

Melhor que os 9,91 s da manhã, e **longe dos 2,0 s**. O motivo está no próprio
log do seguidor: *"EMPERRADO com frente livre (**4,38 m**)"*.

## 5. O defeito da 1ª leva: sondar na direção errada

O corredor à frente estava limpo por quatro metros porque **quem segura o robô
na porta é a ombreira AO LADO**. A sonda frontal caía no vazio, respondia "não é
parede mapeada", e o teto cheio voltava a valer.

O robô 1 sempre teve DOIS portões e a 1ª leva portou só o fraco:

    obstacle_mapped   bloqueio à FRENTE coincide com parede do /map
    near_mapped       parede mapeada a menos de 0,6 m do robô, QUALQUER LADO
                      — o comentário do robô 1 diz, literalmente, "ex. batente"

E lá o raio do segundo foi de 0,35 para 0,6 m em 2026-06-28, pelo BO *"demorou
~15 s pra desencalhar do conhecido"*. É a mesma queixa, com quatro meses de
diferença, e a lição é a mesma: **em vão apertado o que decide não está na
frente do robô, está do lado dele.**

## 6. O que foi implementado

1. `ProgressoDeAvanco.atualiza(t, dist, parado_s=None)` — o teto entra pela
   CHAMADA, o relógio é o mesmo. Alternar os tetos não perde tempo contado.
   Teto ausente, zero ou negativo cai no do construtor: *"sem teto"* não pode
   virar *"dispara sempre"*.
2. `mapa_ocupado(...)` — porte do `map_occupied` do robô 1.
   🔴 **Desconhecido (−1) e fora do mapa contam como LIVRES**, ao contrário de
   `passagens_estreitas`, onde contam como ocupados. A inversão é deliberada e
   o critério é o que cada resposta AUTORIZA: lá ocupado significa *"não invento
   vão"*; aqui significa *"encurto o relógio"*. Consequência boa: o robô real,
   que roda `mapa:=nenhum`, **não** ganha o gatilho rápido em lugar nenhum.
3. `bloqueio_mapeado(x, y, rumo)` — os dois portões, e basta um. Transforma as
   sondas para o frame do mapa pelo TF; sem TF, devolve `False` e avisa (é o
   defeito de 14-08 de comparar `map` com `odom` sem transformada).
4. `teto_de_emperramento(x, y, rumo)` — a fronteira do §3.
5. O aborto do desencalhe deixou de zerar o relógio **quando o que falhou foi a
   medida**. 🔴 **ABERTO (achado R2):** esta mudança não foi pedida pelo dono,
   **nenhum dos 18 testes a cobre**, e o racional "retenta no primeiro quadro
   fresco" não vale sempre — com `res_seguidas > 0` o teto volta a exigir 4 s.
   Reverter ou separar. Os dois abortos tinham a mesma frase de log e causas opostas: vão
   que FECHOU é o mundo dizendo que a manobra não existe mais (zerar está
   certo); perder o `/scan` é o sensor falhando, e cobrar 4 s do robô por um
   soluço do sensor foi o que custou 7,73 s na corrida da manhã.

**Testes**: 18 novos (7 em `test_lei_de_seguimento.py`, 11 em
`test_gatilho_rapido.py`), suíte **1845/0**. Quatro mutações conferidas: ignorar
o teto da chamada, inverter a semântica do desconhecido, remover o portão
`near_mapped` e deixar o rearme encurtar — cada uma reprova testes específicos.

## 7. O que NÃO entrou, e é dívida declarada

- **O escalonamento para o giro.** É o pedido literal do dono (*"duas rés, giro
  no lugar para pegar outro angulo"*) e é o `escalate_after: 2` do robô 1.
  Hoje, esgotadas as duas rés, o robô 3 loga *"Parado até o plano mudar"* e
  fica. O pivô dele existe mas só é alcançado quando frente **e** traseira estão
  bloqueadas, que é outro caso. **É o próximo item desta frente.**
- **O 0,6 m do `near_mapped` é número herdado e ainda não medido aqui.** Lá vem
  de meia-diagonal 0,25 + ~0,2 de registro pose↔mapa do AMCL; aqui a
  meia-diagonal do corpo é 0,314 e, com `localizacao:=fixa`, não há registro a
  compensar. Está classificado como **herdado provisório** no
  `perfil_robo3.yaml`, com a etapa que o fecha. Se provar afobado, é este número
  que se mede — não o gatilho que se desliga.
- **A perda do Livox no simulador** (§4) é a maioria das paradas e não é
  assunto de retomada. Fica como limite de máquina reconhecido pelo dono.
- A 2ª leva **não foi corrida**. O critério para ela é o §4 repetido: a volta
  pela porta 2, com o tempo entre `STOP:PolygonStop` e a manobra medido no log.

## 8. Alternativas descartadas

- **Baixar `re_parado_s` para 2,0 s direto.** Reprovada pelo §3: apaga a medida
  de 12-08 e traz de volta o ciclo "ré e anda".
- **Usar `passagem_ativa` (o gargalo do mapa) como sinal.** Era o candidato mais
  barato, e o dado o matou: `passagem_estreita_habilitada` nasce `False` e o CSV
  da corrida da manhã não tem uma única amostra em modo `gargalo_*`. Sinal que
  não apareceu não vira gatilho.
- **Assinar `/collision_monitor_state` no seguidor e agir no `STOP`.** É o sinal
  mais direto que existe (positivo, não ausência de progresso) e continua sendo
  a melhor ideia de fundo. Ficou fora por ser **assinatura nova na cadeia que
  dirige**, e porque o pedido do dono era explicitamente *"como o robô 1"* — que
  decide pelo mapa. Anotada para quando o portão do mapa mostrar seu limite.

## 9. Referências

- Robô 1: `ros2_packages/robot_nav/robot_nav/unstuck_supervisor.py`
  (`map_occupied`, `block_point_mapped`, `near_mapped`, `mapped_near_radius`) e
  `ros2_packages/robot_nav/launch/nav2.launch.py` (`stuck_timeout_mapped: 2.0`).
  Repositório independente: portar de lá é leitura e reescrita, não merge.
- Medida da manhã: `docs/dados/2026-09-29-robo3-sem-freio-salinha/`.
- Medida da 1ª leva: `~/sessao-robo3/20260929_152605-gatilho-rapido/launch.log`
  e `~/logs_robo2/seguidor_2026-09-29_152607.csv` (fora do git).
- Decisão 009 (gatilho por sintoma, não por geometria), 025 (a ré não é cega),
  031 (sem objetivo vivo não se recua), 062 (o freio que saiu antes disto).
