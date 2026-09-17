# Etapa 1 — lista de medidas do robô 3 (trena, robô DESLIGADO)

> ## ✅ RESPONDIDA EM 17-09 — trena do dono + 8 fotos
>
> **Parte A, a placa:** *"é o mesmo **modelo** da placa do 1 e do 2, mas não a
> mesma, cada um tem a sua."* → saída 🟢: o modelo de atuador sobrevive, a
> etapa 8 vira **conferência** e não levantamento novo. Ressalva: mesma
> **classe**, peça diferente — os números entram como ponto de partida.
>
> **Parte C, o que a trena mudou no URDF:**
>
> | | era | virou | como saiu |
> |---|---|---|---|
> | `roda_separacao` | 0,3225 | **0,320** | (38,0 + 26,0)/2 |
> | `roda_largura` | 0,058 | **0,060** | (38,0 − 26,0)/2 |
> | `boba_raio` | 0,025 | **0,020** | chão→centro da rodinha = 2,0 |
> | `altura_solo` | 0,070 | **0,065** | topo 20,0 − corpo 13,5 |
>
> Confirmados sem mexer: caixa 0,311 × 0,240 × **0,135** (o "±14" fechou em
> 13,5), topo do corpo a 0,200, comprimento do contorno 0,331.
>
> **D3 e D4 encerradas.** A D4 nunca foi caimento — era o `altura_solo` errado:
> 20,0 − 13,5 = 6,5, que é exatamente o "6,5 da boba até o corpo". Palavra do
> dono: *"está sim alinhado, não está caído não."*
>
> **A bitola tem três caminhos apontando o mesmo lugar** — trena (32,0), desenho
> no Gazebo (32,25) e a restrição "o corpo encosta nas rodas". Os 42,5 da §5.7
> estão enterrados.
>
> ⚠️ **Mas isso é NOMINAL, não validação com 2,5 mm** (corrigido no mesmo dia).
> A largura da roda sai da **diferença** das duas leituras e é robusta a viés
> comum; a bitola sai da **média**, onde o mesmo viés **soma**. Concordância
> entre métodos descarta erro grosseiro — não é barra de erro. A bitola é o
> divisor do `wz`: quem a fecha de verdade é **ensaio de pivô** contra o LIO.
>
> **Continua aberto:** `roda_raio` (0,0825 provisório, fecha por corrida reta na
> etapa 8), **a bitola** (0,320 é nominal — fecha por ensaio de pivô, mesma
> etapa), Livox (só quando montar, etapa 7) e massas (prematuro até Livox e NUC
> subirem).

> Etapa 1 do `PLANO_NAV2_ROBO3.md`. **Você passa a trena e anota; eu comparo e
> atualizo o URDF.** Nada aqui liga o robô.
>
> 🔴 **Placa DESLIGADA e robô desligado.** Nenhuma medida desta lista precisa do
> robô energizado, e há registro dele girar as rodas sozinho com a MEGA mandando
> zero.
>
> **Por que esta etapa vem antes do ensaio (etapa 2):** se a placa mudar, a
> dinâmica e a assimetria do ensaio mudam junto e ele tem de ser repetido. Já a
> geometria **não se perde em nenhum cenário**.

---

## Parte A — a decisão que não é de trena

**A placa do robô 3 está decidida?** É o item de maior alavancagem (§5.1 da
revisão cruzada), e a sua última palavra sobre ela foi em 02-09:
*"é uma diferente talvez, irei testar se ela funciona hoje, se não funcionar
será a mesma, mas aviso."*

| se… | consequência |
|---|---|
| **placa nova funciona** | 🔴 o modelo de atuador cai inteiro: zona morta, patamar de saída e latência são daquele firmware. A etapa 8 recomeça do zero, e o `placa_simulada.py` junto |
| **fica a mesma** | 🟢 zona morta e patamar sobrevivem; só a geometria remede |

➡️ **Só preciso de "é a mesma" ou "é outra".** Enquanto estiver aberta, nada de
sintonia — e é por isso que ela abre a etapa.

---

## Parte B — onde medir

Vista **de cima** (o `base_link` fica no centro do eixo das motoras, no chão):

```
                    frente (as MOTORAS)  ←  é esta a frente, decidida em 16-09
                             ▲ x
        ┌────────────────────┼────────────────────┐
     ┌──┴──┐                 │                 ┌──┴──┐
     │ RODA│                 │                 │ RODA│   ← M4 (separação)
     │ esq │◄────────── M4 ──┼──────────►      │ dir │   ← M3 (largura da roda)
     └──┬──┘                 │                 └──┬──┘
        └───────┐    ▪ base_link (0,0)    ┌───────┘
                │            │            │
                │   ┌────────┼────────┐   │
          M1 →  │   │       CAIXA     │   │  ← M2 (altura do fundo ao chão)
                │   │  31,1 × 24,0    │   │
                │   └────────┼────────┘   │
                │        ┌───┴───┐        │
                │      ○ │  M6   │ ○      │   ← bobas (M6: x, y, trail)
                └────────┴───────┴────────┘
                             │
                        traseira (as BOBAS)
```

Vista **de lado**:

```
                    ┌──────── M7 (Livox, quando montar) ────────┐
                    │                                           ▼
                  ╔═══════════════════╗                    ┌─────────┐
                  ║      CAIXA        ║ M2 ↕               │ Livox   │
        ╭───╮     ╚═══════════════════╝                    └─────────┘
        │ R │  M5 ↕ (raio COM o peso em cima)
        ╰───╯  ────────────────────────────────  chão
```

---

## Parte C — a lista

🟢 **RESPONDIDA.** A coluna do meio é o que o URDF tinha **antes** da sessão; a
última diz no que deu.

| # | medir | o URDF tinha | ➡️ virou |
|---|---|---|---|
| **M1** | Caixa: comprimento × largura × altura | 0,311 × 0,240 × 0,135 | ✅ **confirmado**, sem mexer |
| **M2** | Altura do **ponto mais baixo** do chassi ao chão | 0,070 | ➡️ **0,065** — e **fecha a D4**: não era caimento, era este número |
| **M3** | Largura da roda motora (só o pneu, sem o cubo) | 0,058 | ➡️ **0,060** = (38 − 26)/2. Cai o 0,058 de catálogo e o 0,050 da §5.8 |
| **M4** | **Separação centro-a-centro das motoras** | 0,3225 | ➡️ **0,320** = (38 + 26)/2. ⚠️ nominal, não validado — ver o aviso no topo |
| **M5** | Raio da roda **com o peso em cima** | 0,0825 | 🟡 **continua aberto**: fecha por corrida reta (etapa 8) |
| **M6** | Bobas: x/y, raio, largura e o *trail* | x +0,2485 · y ±0,105 · raio 0,025 · larg 0,030 · trail 0,020 | ➡️ **raio 0,020** (fecha a D3); o resto confirmado |
| **M7** | As bobas têm **mola ou suspensão**? | modelo assume **não** | ✅ **confirmado: não** — o C3 fica de pé |
| **M8** | Envelope total | 0,3805 × 0,331 × 0,200 | ➡️ **0,380** × 0,331 × 0,200 |

### 🟢 Alerta da M2 — RESOLVIDO, e a resposta foi a mais chata das duas

A pergunta era: `altura_solo` 7,0 contra boba de 6,0 — o robô está caído ~2,3°
para a frente, ou **uma das medidas está errada**? O modelo assumia nivelado e
inventava um "suporte de 10 mm" sob a boba para fechar a conta.

**Era a medida.** Topo 20,0 − corpo 13,5 = fundo **6,5**, que é exatamente o
"6,5 da boba até o corpo". O `altura_solo` virou 0,065 e o suporte imaginário
sumiu do modelo. Dono: *"está sim alinhado, não está caído não."*

➡️ Fica a lição: uma hipótese mecânica inteira (inclinação permanente, com
consequência para onde o Livox aponta) existia só para acomodar **um número
errado**. Antes de modelar um fenômeno, conferir se a conta fecha.

### ⚠️ Nota da M4 — confirmar, não recalcular

A bitola já teve duas histórias: a §5.7 decidiu **0,425** por argumento e a
**§5.9 a revisou para 0,3225** desenhando no Gazebo (D5 fechada). O código
sempre teve 0,3225 — e em 17-09 achei a tabela-resumo §5.8 **ainda com o 0,425**,
já corrigida.

➡️ Por isso M4 é **medida direta, centro a centro do pneu**. Se der algo longe
de 0,3225, não recalcule por subtração de larguras (foi assim que a história
errada nasceu): me mande o número cru.

---

## Parte D — o que NÃO medir agora

- **Massa e centro de massa.** Prematuro: o Livox e o NUC não subiram no robô,
  então medir hoje é medir outro robô (D7).
- **Livox: altura, x/y, yaw.** Ele ainda não está montado. Quando for — centro
  do robô, no topo, como você decidiu — isso é a **etapa 7**, com pose 6D.
  (O chute atual do URDF, x +0,093 e z 0,240, já bate com essa montagem.)
- **Qualquer coisa de sintonia.** Bloqueado pela Parte A.

---

## Como me mandar

Só os números crus, na ordem M1…M8, e o "mesma"/"outra" da placa. **Não
arredonde e não interprete** — se duas leituras da mesma peça derem diferente,
me mande **as duas**: foi assim que a D2 e a D5 se resolveram, e foi descartar
leitura "estranha" que quase petrificou a bitola errada.

➡️ E vale a regra que a §5.9 deixou: **medida que sobrevive a duas leituras vai
para o Gazebo antes de virar decisão.** Depois que eu atualizar o URDF, eu
renderizo e te mando o desenho para você olhar antes de a gente fechar.
