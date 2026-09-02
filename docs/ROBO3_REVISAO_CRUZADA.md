# Robô 3 — folha de revisão cruzada (Claude ⇄ Codex)

> **Aberto em 2026-09-02, antes de qualquer medida.** O dono anunciou a troca do
> robô 2 (duas hover + uma boba atrás) por um **robô 3 menor: duas rodas bambas
> na FRENTE, duas motoras ATRÁS**. As medidas serão tiradas hoje.
>
> Este arquivo existe para dois assistentes — **Claude** e **Codex** — lerem o
> que o outro afirmou e **verificarem se faz sentido antes de virar código**.
> Não é ata nem diário: é a bancada onde as afirmações ficam expostas com a
> evidência do lado.

---

## 0. Protocolo entre os dois assistentes

1. **Toda afirmação carrega evidência**: `arquivo:linha`, número medido, ou o
   nome de quem falou. Afirmação sem evidência é **hipótese**, e vai marcada
   como tal.
2. **Ninguém edita a seção do outro.** Cada um escreve na sua e responde na
   seção 7 (log de revisão), citando o item pelo número.
3. **Veredito por item**: `CONFIRMA` · `CONTESTA` · `FALTA DADO`. `CONTESTA`
   exige contra-evidência, não opinião.
4. **Nada vira código com item em aberto** que toque no mesmo arquivo. Empate
   entre os dois → **o dono decide**, e a decisão vira `docs/decisoes/NNN-*.md`.
5. Herança do `CLAUDE.md` que vale aqui: **1 mudança pequena por vez**, causa
   confirmada antes de solução, e o robô 2 não é referência automática — o que
   era do robô 1 já provou que não atravessa sozinho.

---

## 1. O que muda, em uma frase

O robô 2 tem **as motoras à frente do centro** (`roda_x = +0,0815`) e **uma
boba atrás** (`boba_x = −0,19`). O robô 3 **inverte isso**: bambas na frente,
motoras atrás — e passa de 3 para **4 pontos de apoio**.

---

## 2. Base factual — o que está no repo HOJE (robô 2)

Levantado por Claude em 2026-09-02, lendo os arquivos vivos (fora de
`build/`, `install/`, `docs/dados/`).

### 2.1 Geometria declarada

| Grandeza | Valor | Fonte |
|---|---|---|
| Caixa (compr × larg × alt) | 0,433 × 0,455 × 0,145 m | `robot_base/description/robo2.urdf.xacro:29-31` |
| Altura do solo ao chassi | 0,0852 m | `…robo2.urdf.xacro:32` |
| Roda motora: raio / largura | 0,080 / 0,045 m | `…robo2.urdf.xacro:45-46` |
| Separação das motoras | 0,270 m | `…robo2.urdf.xacro:47` |
| x do eixo motor | **+0,0815 m** (à frente) | `…robo2.urdf.xacro:55` |
| Boba: raio / x / trail | 0,025 / **−0,19** / 0,01 m | `…robo2.urdf.xacro:69-71` |
| Massas: chassi / roda / garfo / boba | 5,8 / 2,0 / 0,1 / 0,1 kg | `…robo2.urdf.xacro:76-79` |
| Livox: altura do solo / x | 0,42 / 0,0 m | `…robo2.urdf.xacro:109-110` |
| `wheel_separation` / `wheel_radius` | 0,270 / 0,080 m | `hoverboard_driver/bringup/config/hoverboard_controllers.yaml:19-20` |
| Footprint dos 2 costmaps | `[[0.35, 0.2775] … [-0.2665, -0.2775]]` | `robot_motion/config/nav2.yaml:243` e `:384` |
| PolygonApproach (reflexo) | `[[0.2465, 0.2575] … ]` | `robot_motion/config/collision_monitor.yaml:93` |
| PolygonStop (reflexo) | `[[0.35, 0.2775] … ]` | `robot_motion/config/collision_monitor.yaml:257` |

Os 0,270 e 0,080 estão marcados como **medidos com trena em 29-07-2026**
(`hoverboard_controllers.yaml:16-18`); antes eram 0,32 e 0,0825 herdados do
robô 1 e nunca conferidos aqui.

### 2.2 Os outros arquivos que carregam geometria

Mudar a caixa ou as rodas obriga a passar por **todos** estes — vários são
conferidos em par por teste:

```
ros2_packages/robot_base/description/robo2.urdf.xacro
ros2_packages/robot_base/config/hoverboard_controllers_sim.yaml
ros2_packages/robot_base/config/hoverboard_controllers_sim_lento.yaml
ros2_packages/robot_base/launch/sim.launch.py
ros2_packages/robot_base/launch/tracao.launch.py
ros2_packages/robot_base/robot_base/placa_simulada.py
ros2_packages/robot_base/test/test_urdf_robo2.py
ros2_packages/hoverboard_driver/bringup/config/hoverboard_controllers.yaml
ros2_packages/robot_motion/config/nav2.yaml
ros2_packages/robot_motion/config/collision_monitor.yaml
ros2_packages/robot_motion/config/movimentacao.yaml
ros2_packages/robot_motion/config/movimentacao_sim.yaml
ros2_packages/robot_motion/robot_motion/heading_controller.py
ros2_packages/robot_motion/test/test_configs_coerentes.py
ros2_packages/robot_motion/test/test_lei_de_rumo.py
ros2_packages/robot_motion/test/test_lei_de_seguimento.py
ros2_packages/robot_nav/urdf/sim_robot.sdf
tools/banco/{ensaio,medir,meia_lio,sessao,test_banco}.py
docs/MODELO_ROBO2.md          <- vira MODELO_ROBO3.md, não se edita por cima
```

---

## 3. Afirmações de Claude (2026-09-02) — a verificar

> Cada item abaixo espera `CONFIRMA` / `CONTESTA` / `FALTA DADO` do Codex na
> seção 7.

**C1 — A inversão não é cosmética.** Com as motoras atrás, o ponto de rotação
do diferencial sai de +0,0815 (à frente do centro) para trás do centro. A
consequência é geométrica, não de sintonia: **a frente do robô passa a varrer
mais no giro parado**, e é a frente que entra na porta primeiro.
*Status: dedução geométrica, ainda sem número do robô 3.*

**C2 — O latch do eixo e a lei de rumo precisam recalibrar do zero, não
ajustar.** Foram sintonizados na geometria atual (commits `87732c4` — latch do
eixo, descentragem 24 cm → 11 cm — e `5e804d9` — mira no meio das paredes,
REPROVADA). Mudando onde fica o centro de rotação, o ganho que corrigia rumo
sem descentrar não tem por que continuar valendo.
*Status: hipótese com base no histórico; não medida no robô 3.*

**C3 — Duas bambas em vez de uma trocam o apoio isostático por hiperestático.**
Com 3 pontos o robô sempre apoia nos 3. Com 4 pontos e chassi rígido, **uma
roda pode ficar no ar** e o robô cambaleia — e cambalear mexe no Livox, que
está a 0,42 m do solo (braço de alavanca grande para o LIO).
*Status: mecânica geral; precisa conferir se as bambas do robô 3 têm mola.*

**C4 — A ordem de trabalho é: registrar → geometria numa mudança só → testes →
só então sintonia.** Os YAMLs de geometria são acoplados e `test_configs_
coerentes.py` confere par a par; mexer em um de cada vez deixa a árvore
vermelha no meio do caminho sem informação nova.
*Status: procedimento, não fato físico.*

---

## 4. Discrepância encontrada no repo (independe do robô 3)

Não é sobre a troca, mas apareceu na leitura e **contamina qualquer conta de
porta que se faça depois**. Fica aqui porque os dois assistentes vão usar
esses números.

**D1 — O comentário do footprint do `nav2.yaml` não bate com o número que ele
introduz.**

O comentário em `nav2.yaml:240-242` diz: *"Corpo medido (0,433 x 0,455 m) mais
3 cm em toda volta, igual ao PolygonApproach"*. A aritmética:

```
corpo + 3 cm em toda volta   = 0,493 x 0,515   -> x ∈ [-0,2465, +0,2465], y = ±0,2575
PolygonApproach (linha 93)   = 0,493 x 0,515      IDÊNTICO  ✅
footprint (linha 243)        = 0,6165 x 0,555  -> x ∈ [-0,2665, +0,3500], y = ±0,2775
PolygonStop     (linha 257)  = 0,6165 x 0,555      IDÊNTICO  ✅
```

Ou seja: **o footprint do costmap é o PolygonStop, não o PolygonApproach**, e
a margem real é **5 cm por lado + 13,4 cm na frente**, não 3 cm em toda volta.
Os 0,35 da frente estão justificados no `collision_monitor.yaml:226-250` como
margem de reação (a placa segura a saída cheia por 0,52 s, decisão 020) — o
que faz sentido para o **reflexo**. O que falta é justificativa para o
**planejador** carregar a mesma margem de reação.

Isso é exatamente o número que o `ESTADO_PROJETO.md` corrigiu em 20-08
("usei o footprint inflado, não o corpo", commit `45b8d5c`).

*Pergunta aberta para o Codex: o footprint do costmap deve ser o corpo+3 cm
(PolygonApproach) e a margem de reação ficar só no reflexo, ou há razão para o
planejador enxergar o robô com 13 cm a mais na proa? Antes de responder, ver
decisão 041 — a tentativa de encolher a caixa terminou com o robô batendo a
traseira.*

---

## 5. Checklist de medição do robô 3 (para hoje)

Trena + balança. A coluna da direita diz quem consome o número.

| # | O que medir | Consumidor |
|---|---|---|
| 1 | Caixa: comprimento × largura × altura | URDF, footprint, polígonos do reflexo |
| 2 | Altura do ponto **mais baixo** do chassi ao solo | `altura_solo`, checagem de rampa |
| 3 | Roda motora: raio **com o robô no peso dele** (pneu comprime), largura | `wheel_radius`, URDF |
| 4 | Separação centro-a-centro das motoras | `wheel_separation` — entra direto na odometria |
| 5 | Distância do eixo motor ao centro do corpo, **com sinal** | `roda_x` (deve ficar **negativo** agora) |
| 6 | Bambas: raio, largura, x/y de **cada uma**, e o *trail* (pivô → contato) | URDF, estabilidade |
| 7 | As bambas têm mola/suspensão? | decide se C3 é problema real |
| 8 | Livox: altura do centro óptico ao solo, x/y do centro, e se está torto em yaw/pitch | TF, LIO |
| 9 | Massa total com baterias; se der, chassi separado das rodas | inércia do URDF |
| 10 | Onde cai o centro de massa (apoiar em dois pontos numa balança) | tombamento, C1/C3 |
| 11 | Quais baterias e onde estão montadas | CoM, `massa_chassi` |

⚠️ Lembrete que já custou tempo antes: **são duas baterias independentes**
(rodas e NUC+Livox) — anotar as duas.

### 5.1 🔶 A PLACA AINDA NÃO ESTÁ DECIDIDA — item de maior alavancagem

**Status em 2026-09-02, palavra do dono:** *"é uma diferente talvez, irei testar
se ela funciona hoje, se não funcionar será a mesma, mas aviso."*

É a pergunta que mais muda o tamanho do trabalho, e ela tem **duas saídas**:

| Se… | O que acontece com o `MODELO_ROBO2.md` |
|---|---|
| **a placa nova funcionar** (placa diferente) | 🔴 **o modelo inteiro cai.** A zona morta (§2), o patamar de saída (§1) e a latência de 0,35 s para destravar são propriedades **daquele** firmware, não do robô. Levantamento de atuador recomeça do zero, e o `placa_simulada.py` junto. |
| **a placa nova não funcionar** (fica a mesma) | 🟢 §1 e §2 do modelo **sobrevivem à troca**: a zona morta e o patamar são da placa, e a placa não mudou. Só a geometria remede. |

⚠️ **Nenhum dos dois assistentes deve tratar o `MODELO_ROBO2.md` como válido
enquanto esta linha estiver aberta.** Vale para a sintonia inteira: os ganhos de
rumo foram medidos **em cima** daquela zona morta.

Também não se confunda o que cada saída preserva: a placa decide o **atuador**;
a inversão das rodas mexe na **geometria**. Placa igual **não** salva a lei de
rumo — C2 continua de pé nos dois cenários.

**Fica aguardando o aviso do dono.** Enquanto não vier, o trabalho de hoje é só
medida física (itens 1-11) — nada de sintonia.

---

## 5.2 Primeira leva de medidas — robô 3, 2026-09-02

Números como o dono passou, em **cm**, sem arredondar nem interpretar:

| Peça | Medidas dadas |
|---|---|
| Caixa | comprimento **31,1** · largura **24** · altura **13,5** |
| Roda hover | diâmetro **15** · perímetro **53** · altura **16,5** |
| Roda boba (frontal) | altura **4** (6 com a estrutura) · diâmetro **5** · perímetro **~15** |
| Envelope total | altura **20** (com rodas) · comprimento **33,1** (2 cm de roda passando) · largura **39** |

### 5.2.1 🔴 D2 — a roda hover tem TRÊS diâmetros, e eles não são o mesmo

Este é o número mais caro da leva: o raio da roda entra **linearmente** na
odometria. Errar 12% no raio é o robô achar que andou 12% a mais do que andou,
em toda corrida, para sempre.

```
diâmetro declarado   15,00 cm  ->  perímetro seria 47,12   (medido: 53)
"altura" da roda     16,50 cm  ->  perímetro seria 51,84   (medido: 53)
perímetro medido     53,00 cm  ->  diâmetro é      16,87
```

Duas das três medidas apontam para **16,5-16,9 cm**; só o "diâmetro: 15" destoa,
e ele é o que erra o perímetro em 6 cm. Diferença de escala entre usar 15 e usar
16,87: **12,5%**.

**Recomendação de Claude:** ficar com o **perímetro**, não com o diâmetro. Fita
métrica em volta do pneu é a medida mais confiável das três *e é literalmente a
grandeza que a odometria integra* — distância por volta. Daria
`wheel_radius = 53/(2π) = 0,0844 m` (o robô 2 usava 0,080).

⚠️ Com uma ressalva que muda o número: **medir o perímetro com o robô no peso
dele**. Pneu comprimido roda num raio menor que o raio livre, e é o raio de
rolagem que a odometria vê.

➡️ **Pendência:** remedir diâmetro e perímetro da hover, com peso em cima.

### 5.2.2 🟡 D3 — a boba: altura 4 cm com diâmetro 5 cm não fecha

Uma roda de 5 cm de diâmetro apoiada no chão tem 5 cm de altura, não 4. O
perímetro (~15) confirma o diâmetro: `15/π = 4,77`. Suspeita de Claude: os 4 cm
são a roda **sem** o pneu, ou a medida saiu do eixo e não do chão. Pequeno em
valor absoluto, mas é a altura da boba que **nivela a frente do chassi** — 1 cm
aqui inclina o robô inteiro, e o Livox está no topo.

### 5.2.3 🟡 D4 — 0,5 cm sobrando na altura

```
altura total 20,0 − caixa 13,5 = 6,5 cm de vão livre implicado
altura da boba com a estrutura  = 6,0 cm
                                  ------
                                  0,5 cm sem dono
```

Ou o fundo da caixa não encosta na estrutura da boba, ou há algo em cima da
caixa entrando nos 20 cm, ou é arredondamento. **Medir direto o fundo da caixa
até o chão** resolve (item 2 do checklist) e entrega o `altura_solo`.

### 5.2.4 🟢 C5 — o eixo motor está DERIVADO em ~ −9 cm, e isso confirma o C1

Dos 33,1 cm totais contra 31,1 da caixa sobram 2 cm de roda passando. **Assumindo
que passam atrás** (é onde estão as motoras — confirmar), o eixo cai em:

```
raio 7,50  ->  eixo em x = −10,05 cm do centro da caixa
raio 8,25  ->  eixo em x =  −9,30 cm
raio 8,44  ->  eixo em x =  −9,12 cm     <- o do perímetro
```

Comparando com o robô 2, em fração do próprio comprimento:

```
robô 2:  eixo em +8,15 cm de 43,3  =  +19% -> à FRENTE do centro
robô 3:  eixo em  −9,1  cm de 31,1  =  −29% -> ATRÁS do centro
```

**O C1 deixa de ser dedução e vira número: o centro de rotação atravessa o corpo
de ponta a ponta**, de +19% para −29% do comprimento. Toda a dianteira passa a
ser balanço. É a confirmação de que a lei de rumo recalibra do zero (C2).

*Status: DERIVADO, não medido. O item 5 do checklist (distância do eixo ao centro
da caixa, com sinal) continua obrigatório — a derivação depende de o "2 cm de
roda passando" ser atrás, e do raio que a D2 ainda não fechou.*

### 5.2.5 🟢 C6 — a porta de 70 cm deixa de ser problema de CORPO

Refazendo a conta do `ESTADO_PROJETO.md` com o envelope novo (39,0 × 33,1):

```
                                       larg × compr   diagonal   folga a 0°   pior caso
robô 2  corpo real                      45,5 × 43,3      62,8      +12,2       +3,6 cm/lado
robô 2  footprint do costmap            55,5 × 61,6      83,0       +7,2       NÃO CABE (9,5°-74,4°)
robô 3  corpo real                      39,0 × 33,1      51,2      +15,5       +9,4 cm/lado
```

**O robô 3 atravessa a porta de 70 cm em QUALQUER ângulo com 9,4 cm de folga por
lado no pior caso** — contra 3,6 cm do robô 2. O aperto de 1 cm que fazia a
travessia virar sorte (20-08) desaparece do corpo.

🔴 **Mas só do corpo.** Se as margens forem copiadas do robô 2, o vício volta:

```
robô 3 + margem approach (+3 cm/lado)   45,0 × 39,1      59,6      +12,5      +5,2 cm/lado   ok
robô 3 + margem stop/footprint          49,0 × 51,5      71,1      +10,5      NÃO CABE (33,5°-53,6°)
```

A janela proibida sai de **9,5°-74,4° (robô 2) para 33,5°-53,6°** — muito melhor,
mas ainda existe, e nasce inteira dos 13,4 cm de proa que a **D1** apontou como
sem justificativa no planejador. **A D1 deixou de ser dívida técnica e virou
requisito**: com o robô novo dá para pagá-la sem gastar segurança.

### 5.2.6 ⚠️ Armadilha: a caixa não é o que passa na porta

No robô 2 as rodas ficavam **dentro** da largura da caixa (rodas em 27+4,5 ≈ 31,5
contra caixa de 45,5), então footprint = caixa + margem estava certo. **No robô 3
inverteu**: caixa 24 cm, envelope 39 cm — **as rodas são agora a parte mais
larga, com 7,5 cm saindo de cada lado**.

➡️ O footprint e os polígonos do reflexo têm de ser construídos sobre o
**envelope total (39 × 33,1)**, nunca sobre `caixa_y`. No URDF a caixa continua
24 (é o colisor do chassi), mas quem vai para o `nav2.yaml` e o
`collision_monitor.yaml` é o envelope.

### 5.2.7 O que ainda falta — sem isto não se escreve YAML nenhum

| # | Falta | Por que trava |
|---|---|---|
| 4 | **Separação das motoras (centro a centro)** | 🔴 **o mais grave.** É o parâmetro do `wz`: sem ele a odometria de giro não existe. Dá para cercar por `39,0 − largura da roda`, mas cercar não serve: **medir direto**. |
| 3 | Largura da roda hover | fecha a conta acima e o URDF |
| 6 | Bobas: x/y de cada uma, raio, largura, *trail* | posição do apoio, e o C3 |
| 7 | As bobas têm mola? | decide se o C3 (4 apoios, uma roda no ar) é problema real |
| 8 | Livox: altura ao solo, x/y, e se está torto | TF e LIO — no robô 2 estava a 0,42 m |
| 9-11 | Massa total, massa por parte, centro de massa, baterias | inércia do URDF e tombamento |

Referência do que a separação significa (**cercada, NÃO medida**):

```
largura da roda 4,5 -> separação 34,5 cm      largura 5,5 -> separação 33,5
largura da roda 5,0 -> separação 34,0 cm      largura 6,0 -> separação 33,0
(robô 2: separação 27,0 com roda de 4,5)
```

Ou seja: o robô encolheu no corpo mas a **bitola aumentou** (~27 → ~33-34 cm).
Robô mais estreito e ao mesmo tempo mais estável lateralmente.

---

## 5.3 Pedido de remedição enviado (2026-09-02)

⚠️ **Quem mediu não foi o dono** — as medidas da §5.2 vieram de outra pessoa. Ou
seja: as três divergências (D2, D3, D4) não podem ser resolvidas por
interpretação nossa, e nenhum dos dois assistentes deve "escolher" o diâmetro
que prefere. Foi enviado o pedido abaixo, escrito para leigo, com o *como* e o
*porquê* de cada item.

**Bloqueiam qualquer YAML:**

1. Separação das rodas de trás, **centro a centro** (ou faces internas +
   espessura da roda) — o parâmetro do `wz`.
2. Espessura (largura) da roda de trás.
3. Do chão até o **fundo da caixa** → `altura_solo`.
4. Do **eixo traseiro até a traseira da caixa** → fecha o `roda_x` que a §5.2.4
   só derivou.
5. Bobas: distância até a frente da caixa, distância entre elas, **e se têm
   mola** → resolve o C3.
6. Os 2 cm de roda passando: **atrás ou à frente?** → a derivação do C5 assume
   atrás.
7. Livox: altura do chão ao sensor, e se está centrado.
8. Peso total com baterias, e onde ficam as baterias.

**Remedição das divergências:**

9. **D2** — refazer diâmetro E perímetro da roda de trás, o perímetro com fita
   em volta do pneu e **o robô apoiado no próprio peso**.
10. **D3** — diâmetro da rodinha da frente, e separado a altura do chão até o
    ponto onde ela parafusa na estrutura.
11. **D4** — altura total do ponto mais alto até o chão.

Pedidas **fotos com a trena aparecendo** em cada medida. Enquanto não voltar,
a §5.2 vale como *primeira leva*, não como modelo.

---

## 5.4 Segunda leva — a remedição voltou (2026-09-02)

Veio isto:

```
distancia centro a centro           27 cm
espessura da roda traseira           5 cm + 1,5 cm de cubo ("a partezinha de metal")
diametro                          16,5 cm
perimetro                           53 cm
altura                        16,5~17 cm
do chao ate o fundo da caixa         7 cm
do eixo ate a caixa                1,5 cm
altura total                        20 cm   -- E NAO TEM CILINDRO EM CIMA
os 2 cm de roda passam PARA TRAS
```

### 5.4.1 🟢 D2 RESOLVIDA — o "diâmetro 15" era erro de leitura

```
perímetro 53      -> diâmetro 16,87  (raio 0,08435)
diâmetro medido   -> 16,5            (perímetro seria 51,8)
altura no chão    -> 16,5 a 17
```

A divergência caiu de **12,5% para 2,2%**, que é erro de trena. O "15" da 1ª
leva está descartado. **Fica em aberto só qual dos dois adotar**, e há um
argumento físico de cada lado:

- **0,0844 (perímetro)** — é a distância por volta, exatamente o que a odometria
  integra. Mas foi medido com a fita **por fora do pneu**, que anda um pouco mais
  que o raio de rolagem.
- **0,0825 (altura no chão)** — a altura da roda apoiada **já é o diâmetro
  comprimido pelo peso**, que é o que de fato rola.

**Recomendação de Claude:** entrar com **0,0835** (o meio) e **fechar por
corrida reta**: 5 m em linha, `odom` contra o LIO, e o fator sai medido. 2,2% em
5 m é 11 cm — o LIO enxerga isso sem esforço. É calibração que a gente ia fazer
de qualquer jeito.

### 5.4.2 🔴 D5 — os 27 cm NÃO CABEM nos 39 cm de largura total

O achado grave desta leva, e ele derruba o parâmetro mais importante de todos.

```
separação 27 + pneu 5                      = 32,0 cm de largura total
separação 27 + pneu 5 + cubo 1,5           = 33,5 cm
separação 27 + pneu + cubo dos dois lados  = 35,0 cm
                                    medido = 39,0 cm    <- faltam 5,5 cm
```

Nenhuma combinação fecha. Mas uma leitura alternativa fecha **exata**:

```
SE os 27 forem de FACE INTERNA a FACE INTERNA, com roda de 6,0 cm:
   centro a centro = 27 + 6,0 = 33,0 cm
   largura total   = 27 + 12,0 = 39,0 cm   <- o número medido, na mosca
```

⚠️ **E há uma coincidência que aumenta a suspeita: 27,0 cm é exatamente o
`wheel_separation` do robô 2** (`hoverboard_controllers.yaml:19`). Pode ser
reaproveitamento real do mesmo eixo hover — ou um número que veio de outro
lugar que não a trena. Não dá para distinguir daqui.

**Por que isto é o pior de todos:** `wheel_separation` é o divisor do `wz`.
Errar 27 contra 33 é **22% de erro em todo giro do robô, para sempre** — e giro
é justamente onde o robô 2 já apanhava na porta.

➡️ **Nada de giro se escreve até isto fechar.** E fecha com uma medida que não
depende de achar centro nenhum:

```
A = de FORA a FORA dos dois pneus        (é a própria largura total)
B = de DENTRO a DENTRO dos dois pneus
    centro a centro = (A + B) / 2        largura da roda = (A − B) / 2
```

Duas medidas de fita, sem ambiguidade, e ainda conferem uma à outra.

### 5.4.3 🟡 D6 — "do eixo até a caixa: 1,5" responde outra pergunta

Foi pedida a distância **longitudinal** do eixo traseiro até a traseira da
caixa. O 1,5 não pode ser essa:

```
leitura LONGITUDINAL: eixo a 1,5 da traseira -> a roda sobraria 6,75 cm atrás
                      mas ele confirmou que sobram 2,0 cm            ❌
leitura VERTICAL    : eixo a 8,25 do chão − fundo da caixa 7,0 = 1,25 ≈ 1,5  ✅
```

Ou seja: ele mediu do **eixo até o fundo da caixa**, na vertical. O número é
bom e é consistente — só não é o que trava o `roda_x`.

### 5.4.4 🟢 C5 confirmado — o eixo está em −9,3 cm

Com "os 2 cm passam **para trás**" confirmado pelo próprio medidor, a derivação
da §5.2.4 fica firme e apertada:

```
raio 8,25  -> eixo em x = −9,30 cm do centro da caixa  (−30% do comprimento)
raio 8,44  -> eixo em x = −9,12 cm                     (−29%)
robô 2     -> eixo em x = +8,15 cm                     (+19%)
```

**±0,2 cm de incerteza.** O centro de rotação sai de +19% para −30% do
comprimento: atravessa o corpo inteiro, e a dianteira vira balanço puro.

### 5.4.5 🟡 D4 mudou de sinal, e apareceu um desnível de 1 cm

```
1ª leva: total 20,0 − caixa 13,5 − boba 6,0 =  +0,5 cm sobrando
2ª leva: fundo 7,0 + caixa 13,5             =  20,5 contra total 20,0
                                               -> 0,5 cm FALTANDO agora
```

Mais interessante que os 0,5: **a boba com estrutura tem 6,0 cm e o fundo da
caixa está a 7,0 cm do chão**. Se a boba é o que sustenta a frente, a frente
está **1 cm mais baixa que a traseira** — o robô inclina ~**1,8°** para a
frente ao longo dos 31,1 cm.

Isso não é decoração: 1,8° de *pitch* fixo entra no LIO e nas normais do
`scan_sanitizer`. **Ou o chassi está mesmo inclinado — e aí vai para o URDF —
ou uma das três medidas está errada.**

### 5.4.6 🔴 D7 — "não tem cilindro em cima": o Livox não está montado

Perguntei a altura do chão até o sensor e a resposta foi que **não existe
cilindro em cima**. Conclusão de Claude: **o robô 3 ainda não recebeu o Livox**
(nem, provavelmente, o NUC).

Consequências, e são grandes:

1. **Não há TF do sensor.** `livox_z_solo`, `livox_x` e o *yaw* de montagem não
   existem ainda — e sem eles não há LIO, não há `/scan`, não há navegação.
2. **A massa medida vai estar errada.** Livox + NUC + bateria deles são
   carga que ainda vai subir no robô, e mexe no peso **e no centro de massa** —
   que é o que decide o C3 (uma boba no ar) e o tombamento.
3. **Onde ele vai ser montado é decisão de projeto, não medida.** No robô 2 ficou
   a 0,42 m do solo, mais que o dobro da altura total do robô 3 (0,20 m). Não dá
   para herdar: a altura do Livox muda o horizonte que ele enxerga.

➡️ **Item novo para o dono, não para quem mediu:** onde o Livox vai, e quando.
Remedir massa e CoM **depois** de montar — medir agora é medir outro robô.

### 5.4.7 Placar depois da 2ª leva

| Item | Estado |
|---|---|
| Caixa 31,1 × 24 × 13,5 | 🟢 firme |
| Envelope 39 × 33,1 × 20 | 🟡 a largura depende da D5 |
| Raio da roda | 🟢 0,0825-0,0844; fecha por corrida reta |
| `roda_x` = −9,3 cm | 🟢 derivado, ±0,2 cm |
| `altura_solo` = 7,0 cm | 🟡 briga com a boba de 6,0 (D4) |
| **`wheel_separation`** | 🔴 **27 ou 33 — bloqueia todo o giro (D5)** |
| Bobas: posição, trail, mola | 🔴 não respondido |
| Peso, centro de massa, baterias | 🔴 não respondido — e prematuro (D7) |
| Livox: altura, x/y, yaw | 🔴 **nem montado (D7)** |

---

## 5.5 Terceira leva — as bobas (2026-09-02)

Dono: *"as rodinhas são rodas bobas padrão, ela vai tentar pegar o diâmetro
delas, mas tem a estrutura delas em volta"*. Foi pedido então o que não depende
de brigar com o garfo — **do chão até o centro do eixinho** (= o raio) e **o
recuo horizontal entre o parafuso vertical e o eixo da roda** (= o *trail*).
Resposta: **2 cm os dois**.

### 5.5.1 🟡 D3 continua empatada, mas virou irrelevante

```
chão até o eixo 2,0  -> diâmetro 4,0  -> perímetro seria 12,6
1ª leva: altura 4,0     BATE com d = 4,0
1ª leva: diâmetro 5,0   \ apontam para d ≈ 4,8-5,0
1ª leva: perímetro ~15  /  (50 mm é tamanho de prateleira; 40 mm é menos comum)
```

**2 a 2, e o empate não se desfaz daqui.** Mas ele deixou de importar, e a razão
é geométrica: **a altura da frente do robô é o conjunto de 6,0 cm com a
estrutura, medido direto.** O diâmetro da roda não entra na altura de rodagem —
entra só na inércia da rodinha, que é desprezível perto dos 3 kg por hover.

➡️ **Adotar 5,0 cm** (tamanho padrão, duas medidas apontando) e seguir. Se
aparecer contradição depois, custa uma linha de URDF. **D3 sai do caminho
crítico.**

### 5.5.2 🔴 C7 (novo) — o trail dobrou E duplicou: a ré vai custar rumo

Este é o achado da leva, e é uma **previsão falseável**:

```
robô 2:  boba_trail 1,0 cm, UMA boba      (robo2.urdf.xacro:71)
robô 3:  trail       2,0 cm, DUAS bobas
```

Numa inversão de marcha a boba tem de girar 180° em torno do pivô, e o contato
varre um círculo de raio igual ao trail:

```
robô 3: contato desloca 2 × 2,0 = 4,0 cm lateralmente, em DOIS pontos
robô 2: contato desloca 2 × 1,0 = 2,0 cm, em UM ponto
```

Com o eixo motor em −9,3 cm e as bobas em algum ponto da dianteira, o
entre-eixos fica em ~19-23 cm. O escorregão, **como limite geométrico**:

```
bobas em x = +10 -> entre-eixos 19,3 cm -> até 11,7°   (robô 2: 5,9°)
bobas em x = +12 -> entre-eixos 21,3 cm -> até 10,6°   (robô 2: 5,4°)
bobas em x = +14 -> entre-eixos 23,3 cm -> até  9,7°   (robô 2: 4,9°)
```

⚠️ **Isso é o limite geométrico, não o erro realizado** — as motoras resistem ao
escorregão. Mas a razão entre os dois robôs (**~2×**) não depende dessa
resistência, e um trail de 2,0 cm num pneu de ~5,0 é **40% do diâmetro**: boba de
balanço longo.

**Por que isto importa exatamente aqui:** o robô 2 deu **7 rés** numa única
aproximação de porta (`ESTADO_PROJETO.md`, 20-08). Se cada inversão de marcha
passa a cobrar o dobro de perturbação de rumo, **a manobra de recuperação que
salvava o robô 2 vira a que estraga o robô 3.**

➡️ **Previsão para verificar em bancada, antes de sintonizar:** comandar
frente → parada → ré e medir o transitório de *yaw* nos primeiros 30 cm. Se o
robô 3 der ~2× o desvio do robô 2 na mesma manobra, o C7 está confirmado e a
política de ré precisa mudar junto com a lei de rumo (C2).

*Status: previsão geométrica, NÃO medida. Depende ainda da posição das bobas.*

### 5.5.3 O que a terceira leva não trouxe

Ainda pendente, e a dona das medidas disse que manda: **posição das bobas**
(distância até a frente da caixa e distância entre elas — o C7 acima precisa
dela para sair de faixa e virar número), **se têm mola** (C3), **peso** e **onde
ficam as baterias**. Lembrando a D7: peso e centro de massa medidos **antes** do
Livox e do NUC subirem são de outro robô.

---

## 5.6 Quarta leva — e a largura piorou (2026-09-02)

Veio: **do lado interno das rodas hover até o lado interno da outra = 37,5 cm**,
**as bobas não têm mola**, e **cada uma fica na ponta da frente da caixa, uma em
cada extremidade**.

### 5.6.1 🔴 D5 agora tem TRÊS medidas de largura, e uma é impossível

```
27,0  dito "centro a centro"    (2ª leva)
37,5  dito "interno a interno"  (4ª leva)
39,0  dito "largura total"      (1ª leva)
```

⚠️ **27 e 37,5 não podem coexistir como foram ditos**: centro a centro é sempre
MAIOR que interno a interno, nunca menor. Uma das duas etiquetas está trocada.

**História A — o "27" era o interno, e o "37,5" é o externo:**

```
centro a centro = (37,5 + 27)/2 = 32,25 cm
largura da roda = (37,5 − 27)/2 =  5,25 cm   <- a espessura medida foi 5,0  ✅
largura total   = 37,5 + cubos 1,5 = 39,0    <- a 1ª leva disse 39,0        ✅
```

**As três medidas caem dentro de 1,5 cm uma da outra, e a largura da roda cai
sozinha em cima do valor que foi medido em separado.** Não forcei nada: a
espessura de 5 cm nunca entrou nessa conta, ela saiu dela.

**História B — o "37,5" é interno ao pé da letra:**

```
roda 5,0 -> centro a centro 42,50 | largura total 47,5 cm
roda 6,5 -> centro a centro 44,00 | largura total 50,5 cm
```

Exige **jogar fora o 27 E o 39**, e ainda deixa o robô com 47-50 cm de largura.

**Claude fica com a história A** — mas fica **como hipótese**, não como fato,
porque duas coisas grandes dependem dela:

```
wz:     separação 32,25 (A) contra 42,50 (B) = 32% de erro em todo giro
porta:  envelope 39,0 x 33,1 -> diagonal 51,2 -> pior caso +9,4 cm/lado   (A)
        envelope 47,5 x 33,1 -> diagonal 57,9 -> pior caso +6,1 cm/lado   (B)
```

🔴 **Na história B o robô 3 tem 47,5 cm de largura contra os 45,5 do robô 2 — ou
seja, seria MAIS LARGO que o robô que não passava na porta.** O C6, que é a
melhor notícia do projeto até agora, está pendurado nesta etiqueta.

➡️ **Uma medida só encerra:** com o robô de frente, **a largura no ponto mais
largo dele — de fora do pneu de um lado até fora do pneu do outro.** Um número,
sem ambiguidade de "onde é o centro" nem de "interno de quê". Se der ~39 → A;
se der ~47 → B.

### 5.6.2 🔴 C3 CONFIRMADO — quatro apoios rígidos, e as bobas nas quinas

*"Não tem mola"* + *"cada uma na ponta da frente da caixa, uma em cada
extremidade"*. O C3 deixa de ser preocupação genérica e vira geometria:

```
bobas   em x = +15,55 (ponta da caixa), y = ±12,0 (meia-largura da caixa)
motoras em x =  −9,30              , y = ±16,1 (história A)
entre-eixos 24,9 cm | braço do balanço na diagonal 15,9 cm
```

**Quatro pontos rígidos num chassi rígido é hiperestático**: em piso irregular
uma boba sai do chão e o corpo balança entre as duas diagonais. O braço de
15,9 cm converte irregularidade de piso direto em inclinação do corpo:

```
degrau de  3 mm sob uma boba -> 1,1° de balanço
degrau de  5 mm               -> 1,8°
degrau de 10 mm               -> 3,6°
```

⚠️ **E o Livox vai em cima disso.** No robô 2 o sensor estava a 0,42 m, com
apoio isostático de 3 pontos que não balança por definição. Aqui, 3 mm de junta
de piso viram mais de 1° de *pitch/roll* no sensor — e o corredor do andar 3 é
piso de prédio, com junta.

➡️ **Isto é decisão de projeto do dono, não medida:** ou entra complacência (uma
boba com mola, ou um balancim que devolva o apoio a 3 pontos), ou o LIO vai
receber esse balanço como movimento real. **Vale registrar a pergunta antes de o
Livox subir, porque depois o conserto é mecânico.**

Nota lateral: a bitola das bobas (24,0, nas quinas da caixa) é **mais estreita
que a das motoras** (32,25 na história A). Para tombamento lateral quem manda é
a parte estreita, e ela está na frente.

### 5.6.3 🟢 C7 sai da faixa e vira número

Com as bobas na ponta da caixa (`x = +15,55`), o entre-eixos fecha em
**24,85 cm**, e o limite geométrico do escorregão de ré:

```
robô 3 (trail 2,0, duas bobas, escorregão 4,0 cm) -> até 9,1°
robô 2 (trail 1,0, uma boba,   escorregão 2,0 cm) -> até 4,6°
```

**Exatamente 2×**, e agora sem depender de chute de posição. O C7 fica pronto
para o teste de bancada da §5.5.2.

### 5.6.4 Placar depois da 4ª leva

| Item | Estado |
|---|---|
| Caixa, envelope em comprimento e altura | 🟢 firme |
| Raio da roda motora | 🟢 0,0835 provisório, fecha por corrida reta |
| `roda_x` = −9,3 cm | 🟢 ±0,2 cm |
| Bobas: posição (+15,55, ±12,0) e trail 2,0 | 🟢 firme |
| Bobas: sem mola | 🟢 respondido — e é o que **confirma o C3** |
| `altura_solo` 7,0 vs boba 6,0 (1,8° de caimento) | 🟡 D4 aberta |
| Diâmetro da boba | 🟡 adotado 5,0, fora do caminho crítico |
| **Largura: `wheel_separation` e envelope** | 🔴 **A ou B — trava o giro E a conta da porta (D5)** |
| Peso, centro de massa, baterias | 🔴 pendente, e prematuro até o Livox subir |
| **Livox: onde monta** | 🔴 **decisão do dono, agora com o C3 junto** |

---

## 6. Plano proposto por Claude (aguardando ok do dono)

1. `docs/decisoes/045-troca-para-o-robo-3.md` — medidas, motivo da troca, e o
   que se perde do que já estava sintonizado.
2. Geometria em **uma mudança só**: URDF + os YAMLs acoplados da §2.2.
3. `test_urdf_robo2.py` e `test_configs_coerentes.py` verdes (renomear o
   primeiro se o robô 2 sair de cena).
4. `MODELO_ROBO3.md` — novo arquivo; o do robô 2 fica como está, para o artigo.
5. **Só então** sintonia, e a lei de rumo do zero (C2).

---

## 7. Log de revisão cruzada

> Formato de uma linha: `item · quem revisou · veredito · evidência`.
> Item sem linha aqui = **não revisado**, não "aceito".

| Item | Revisor | Veredito | Evidência / observação |
|---|---|---|---|
| C1 | Codex | | |
| C2 | Codex | | |
| C3 | Codex | | ← CONFIRMADO na 4ª leva: sem mola, 4 apoios rígidos (§5.6.2) |
| C4 | Codex | | |
| C5 | Codex | | ← derivado em §5.2.4, não medido |
| C6 | Codex | | ← conta da porta em §5.2.5 |
| C7 | Codex | | ← 🔴 fechado em 9,1° contra 4,6° do robô 2 (§5.6.3) |
| D1 | Codex | | |
| D2 | Codex | | ← RESOLVIDA na 2ª leva (§5.4.1): "15" descartado, sobra 2,2% |
| D3 | Codex | | ← empate 2×2, mas saiu do caminho crítico (§5.5.1) |
| D4 | Codex | | ← virou desnível de 1 cm / 1,8° de pitch, §5.4.5 |
| D5 | Codex | | ← 🔴 piorou: 3 medidas de largura, uma impossível (§5.6.1) |
| D6 | Codex | | ← "1,5" era vertical, não longitudinal, §5.4.3 |
| D7 | Codex | | ← Livox não está montado, §5.4.6 |

### Afirmações do Codex (a preencher)

> Codex: escreva aqui numerando `X1`, `X2`, … no mesmo formato da §3
> (afirmação + evidência + status). Claude responde na tabela acima,
> acrescentando as linhas `X1`, `X2`, … com a coluna Revisor = Claude.

_(vazio)_
