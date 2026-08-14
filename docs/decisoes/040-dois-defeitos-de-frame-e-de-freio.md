# 040 — Dois defeitos: o seguidor sem TF e o pivô sem freio

**Data**: 2026-08-14 (dev + Gazebo, o dono na tela até o almoço; o protocolo
final rodado pelo assistente, com autorização explícita dele)
**Status**: os dois defeitos **corrigidos e no ar**. ⚠️ **A PORTA NÃO ESTÁ
RESOLVIDA**: protocolo de 5 corridas com condição inicial idêntica deu **1
passagem em 5**. Números na seção 6.
**Toca**: `path_follower.py`, `heading_controller.py`, `pilha.launch.py`,
`lei_de_seguimento.py`, `lei_de_rumo.py`, `movimentacao_sim.yaml`
**Vem de**: um dia inteiro de tentativas erradas. O valor deste registro está
tanto no que falhou quanto no que ficou.

---

## 1. O defeito nº 1 — o seguidor comparava frames diferentes

```
path_follower.py   subscription(Odometry, '/Odometry')   frame: odom
path_follower.py   subscription(Path,     '/plan')       frame: map
grep tf2|TransformListener|lookup_transform  ->  NADA no arquivo
```

O seguidor media sua posição contra o plano **sem nunca aplicar a transformada
entre os dois frames**. A diferença entre `map` e `odom` é exatamente a
correção do AMCL, e ela foi medida nas corridas do dia:

```
corrida A   salto entre correções p90  14,3 cm   deriva total   76 cm
corrida C                              9,6 cm                   56 cm
corrida E                             100,3 cm                  833 cm  (o AMCL fugiu)
```

➡️ O seguidor se achava fora do caminho por uma quantidade que era **puro erro
de frame**, e dirigia para corrigir um desvio que não existia — desvio que
pulava a cada atualização do AMCL. Isso explica o S, a entrada torta na porta,
e — o mais importante — **por que toda melhoria de responsividade PIORAVA o
resultado**: mais fidelidade a um sinal errado.

**Conserto**: o plano é trazido para `odom` a cada ciclo (`plano_em_odom`). Vai
o plano para a pose, e não a pose para o plano, porque o `heading_controller`
mede rumo no referencial do `/Odometry` — está na primeira linha da docstring
dele. Mexer no frame da pose obrigaria a girar o `rumo_alvo` junto, e seria a
mesma armadilha trocada de lugar.

⚠️ Sem transformada disponível o nó **não dirige** e grita. Antes ele caía em
silêncio no comportamento errado, que é como isto sobreviveu tanto tempo.

## 2. O defeito nº 2 — o pivô era o único caminho sem freio

```python
heading_controller.py     # dentro do pivô
    self.publica(0.0, wz)
    self.plantao(agora, 0.0, wz)
    return                # <- saía AQUI

heading_controller.py     # o freio de giro (037) mora abaixo
    wz = self.freio.passo(...)
```

O `return` pulava o bloco do freio. O pivô era **o único caminho da cadeia que
girava sem freio nenhum**, e a 037 mede o que isso custa: 63,8° de varredura
sem freio contra 14–28° com.

Medido na corrida G (pivô ligado, ainda sem freio): varredura de **150 a 310°
por pulso**, p50 170°. O resíduo virava erro grande do outro lado e ele
disparava de novo — nas palavras do dono, *"metendo um monte de 180"*.

**Conserto**: o `wz` do pivô cai no freio como o resto da cadeia. Durante o
giro o freio deixa passar (ele só age com a lei calada); no assentamento, onde
o pivô pede zero e a inércia entregaria a sobra, ele morde.

Com isso a varredura caiu para **91° e 131°** (corrida H) — a ordem certa para
o gatilho de **80°** que o dono tinha pedido na decisão 036.

## 3. O que eu tentei e falhou — quatro vezes, e todas medidas

Registrado porque o projeto é PIBIT e fracasso medido é resultado. E porque a
lição comum às quatro é a mesma: **eu mexi em número onde o defeito era de
mecanismo.**

| tentativa | o que eu fiz | o que mediu | veredito |
|---|---|---|---|
| 039, `k_lat=1,0` | termo de Stanley sem limite de taxa | amplitude p90 da referência 20,0° → **39,8°**, pico 88,1° | reprovado |
| 039, `k_lat=0,5` + limite de taxa | rampa em vez de degrau | p90 21,1°, mas inversões do corpo 26,9 → **30,5**/min | reprovado |
| mira adaptativa | 0,37 m em curva, 1,00 m em reta, com histerese | não avaliada isolada | pendente |
| pivô em 34°, `a_dec` 6,6 | derivei o `a_dec` da 037 | varredura de **150–310°** por pulso | reprovado |

🔴 **O `pivo_a_dec` real é 0,79** (medido na corrida H), contra os 0,6 que já
estavam lá e os 6,6 que eu inventei **duas vezes**. Os dois números que eu
derivei erraram por ~8×. A lição é dura e vale escrita: **derivação não é
medida**, e neste projeto a régua sempre existiu — bastava rodar.

## 4. O número que atravessou o dia

```
período da oscilação de rumo, sete corridas:  2,0 a 2,8 s   — CONSTANTE
enquanto mudavam: lei, ganho, mira, pivô, frame e histerese
```

Período constante enquanto tudo o mais muda é assinatura de **planta**. Relé
com tempo morto `L` oscila em ~4L, e o `atraso_desliga` medido é 0,52 s → 2,08 s.
A placa É um relé (decisão 020: *"a placa escolhe o módulo"*).

➡️ **Consequência de projeto**: se isto se confirmar, nenhuma reescrita do
seguidor mata o S — o alvo passa a ser a placa (o `deadband_enable`, o tempo
morto). É a hipótese mais importante em aberto no projeto.

## 5. Log de toda corrida, por padrão

Pedido do dono: *"quero que toda corrida, tanto Gazebo quanto robô real,
salvem log de tudo, assim como o robô 1 faz"*.

Dois furos encontrados, não um:

1. `path_follower` nascia com `csv: ''` — log desligado;
2. **`grava()` nunca era chamado**. Mesmo configurando o caminho, nada seria
   escrito. Era código morto desde que existe.

Resultado: **nenhuma corrida deste projeto foi gravada pelo nó**, e o dia
inteiro dependeu de alguém lembrar de subir um `ros2 bag` à mão — foi assim que
a primeira corrida BOA de 14-08 se perdeu sem registro.

Agora, nos DOIS perfis: `log_dir` (default `~/logs_robo2`) faz o seguidor abrir
um CSV com carimbo de tempo a cada subida e reescrevê-lo a cada 5 s (corrida que
morre em Ctrl-C, bateria ou `kill -9` não pode perder o registro), e o launch
sobe um `ros2 bag` da corrente inteira junto. Sem nuvem e sem costmaps de
propósito: são os dois que fariam o bag inviável no NUC.

O robô 1 grava `follow_debug.csv` sempre, sem opt-in — é de lá que a ideia vem.

## Referências

- decisão 020 — a placa escolhe o módulo (o relé)
- decisão 023 — o pivô sai do caminho (o argumento que sobreviveu a mim)
- decisão 036 — o gatilho de 80°, que era do dono e estava certo
- decisão 037 — o freio de giro, cujo alcance eu superestimei
- `Controle_robo_web` — `path_follower.py`: histerese, carrot adaptativo e
  alvo congelado, todos com o número que os justifica


## 6. O protocolo de 5 corridas — e o que ele diz

`tools/banco/protocolo_porta.sh`, pilha NOVA a cada corrida (o robô termina
onde parou, e a geometria de aproximação é o que está sob teste — duas corridas
seguidas sem reset medem coisas diferentes). Alvo (6,14 · 3,78), teto de 90 s.

```
corrida  veredito       parou em       folga   sobra p/ o corpo   yaw    reflexo
   1      PASSOU     (+6.15,+3.71)     0.949      +0.722         no alvo   NÃO AGIU
   2      TRAVOU     (+4.64,+1.41)     0.402      +0.175         -10.2°    14x, 51 s
   3      TRAVOU     (+4.65,+1.58)     0.347      +0.120         -16.7°     4x, 69 s
   4      TRAVOU     (+4.72,+1.52)     0.408      +0.180         -33.6°     3x, 70 s
   5      TRAVOU     (+4.92,+1.58)     0.353      +0.126         -33.2°     3x, 69 s
```

🔴 **Em NENHUMA falha o corpo invadiu o mapa.** Ele parou com 12 a 18 cm de
folga por lado e ficou congelado ~69 s. Não é colisão — é o reflexo travando
com margem sobrando, e a causa está na última coluna do meio: **as quatro
entraram tortas**, de −10° a −34°.

O `PolygonApproach` tem meia-largura de 0,2575 m (contra 0,2275 do corpo) e é
**projetado pela velocidade comandada**. Entrando a −33°, a projeção alcança a
ombreira muito antes de o corpo chegar perto dela. Some com a ré, que nessas
poses mede "atrás há 0,00 m" e se recusa — e o robô fica preso até o teto.

➡️ **A dívida nº 1 do projeto continua sendo a mesma, e agora com régua:** ele
entra torto na porta. Nada do que eu fiz hoje mexeu nisso — as duas correções
de mecanismo eram reais e necessárias, mas atacavam outra coisa.

**O próximo alvo, e ele é geométrico**: chegar à porta JÁ APONTADO. A decisão
026 nasceu desse mesmo miolo ("chegar já apontado") e não bastou. As duas
alavancas plausíveis, nesta ordem:

1. **o alinhamento antes de entrar** — o robô 1 resolveu com um estado
   dedicado (`door_crossing`: STAGING → ROTATING → CROSSING, com trava de
   `|lat|<8 cm` e `|yaw|<5°` antes de cruzar). Está desativado lá, mas o
   desenho é o que a medida daqui pede;
2. **o `PolygonApproach` projetado** — 3 cm de folga lateral sobre o corpo é
   pouco para um vão de 88 cm com entrada torta. Mexer nisso é band-aid e o
   dono já recusou band-aid antes (a ré da 025), então vem depois de (1).

## 7. O A/B da histerese, e o que ele revelou por acidente

10 corridas, condição inicial idêntica, `tolerancia_entra_rumo` como argumento
de launch (exposto para isto — config editada à mão no meio de um protocolo é
como se perde a condição idêntica).

```
                 passou   inversões de rumo/min (nas que passaram)
COM histerese     3/5              2,7 – 4,1
SEM histerese     3/5              8,7 – 8,8
```

➡️ **A histerese não muda a taxa de passagem, mas CORTA AS INVERSÕES PELA
METADE.** Ela faz o que foi feita para fazer (o S) e não tem relação com a
porta. **Fica**, e passa a ser a primeira coisa do dia que se sustenta em
medida controlada.

### 🔴 O que a coluna que eu não tinha olhado mostra

```
nas TRAVADAS:  yaw −23° a −27°   mas   erro_rumo  1,5°  5,7°  −7,5°  11,0°
```

**O erro de rumo é PEQUENO enquanto o corpo está a 25° atravessado.** O
seguidor está alinhado com o que a referência pede — quem está torto é a
**referência**. O plano curva atravessando o vão e o robô obedece a curva para
dentro da porta, em vez de se endireitar antes de entrar.

➡️ **Não é falha de rastreamento; é geometria de aproximação.** Isso reclassifica
o dia inteiro: todas as minhas tentativas melhoravam o rastreamento de uma
referência que já estava sendo seguida corretamente. A dívida nº 1 do projeto
("ele entra torto na porta") não é do seguidor — é de **quem decide o rumo na
aproximação**.

O desenho que a medida pede é o do robô 1 (`door_crossing`, mesmo desativado
lá): alinhar ao eixo do vão até `|lat| < 8 cm` e `|yaw| < 5°`, e só então
cruzar reto. É a decisão 026 ("chegar já apontado") levada a sério — ela nasceu
desse miolo e parou no meio do caminho.

### ⚠️ E uma contaminação que vale para TODAS as medidas de hoje

```
protocolo das 14h (load 15–19)   1 de 5
A/B      das 12h (load  2–5)     3 de 5  e  3 de 5
```

**A taxa de passagem triplica com a máquina descarregada.** As corridas da tarde
estavam contaminadas por CPU faminta — inclusive as que eu usei para reprovar as
minhas próprias mudanças. Nenhum veredito de hoje sobre `k_lat` ou mira
adaptativa deve ser considerado definitivo sem repetir com a máquina limpa.
