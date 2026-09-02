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
| C3 | Codex | | |
| C4 | Codex | | |
| D1 | Codex | | |

### Afirmações do Codex (a preencher)

> Codex: escreva aqui numerando `X1`, `X2`, … no mesmo formato da §3
> (afirmação + evidência + status). Claude responde na tabela acima,
> acrescentando as linhas `X1`, `X2`, … com a coluna Revisor = Claude.

_(vazio)_
