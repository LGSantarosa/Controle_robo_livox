# 2026-09-17 — robô 3 na trena (etapa 1 do plano do Nav2)

Sessão que **fechou a etapa 1** do `docs/PLANO_NAV2_ROBO3.md`: geometria física
medida com o robô desligado, e a decisão da placa respondida.

---

## 🔴 De onde vêm os números — leia antes de usar estas fotos

**Os valores autoritativos são os que o dono DITOU**, listados abaixo. As fotos
são **contexto**: elas mostram onde a fita foi apoiada e confirmam a ordem de
grandeza. **Não são fonte de leitura.**

Motivo, e vale registrar em vez de descobrir de novo: nestas fotos a fita
aparece **angulada**, e em várias o zero está **fora de quadro ou atrás da
roda**. Tentar rederivar milímetro daqui produziria número novo com cara de
medida — exatamente o tipo de coisa que já custou levas inteiras neste projeto
(ver a D2 e a D5 na `ROBO3_REVISAO_CRUZADA.md`).

➡️ Se um número desta sessão for contestado, o caminho é **medir de novo**, não
ampliar a foto.

---

## Os números ditados

| grandeza | valor |
|---|---|
| Comprimento do corpo | 31 cm |
| Largura do corpo | 24 cm |
| Altura do corpo | 13,5 cm (o dono deu "±14" e fechou em 13,5) |
| Chão até o topo do corpo | 20 cm |
| Motoras: extremo esquerdo → extremo direito | ±38 cm |
| Motoras: face interna esq. → face interna dir. | ±26 cm |
| Bobas: chão até o centro da rodinha | 2 cm |
| Bobas: da rodinha até o corpo | 6,5 cm |

### O método que fez a bitola fechar de primeira

As duas medidas que mais importam **não foram lidas direto** — saíram da
combinação de duas leituras independentes:

```
centro a centro  = (38 + 26)/2 = 32,0 cm   -> wheel_separation
largura da roda  = (38 − 26)/2 =  6,0 cm
```

⚠️ **E as duas contas NÃO são igualmente robustas** — a primeira versão deste
README dizia que sim, e estava errada:

- **a largura** sai da **diferença**. Viés comum às duas leituras (fita apoiada
  torto, ponto de apoio escolhido no olho) erra as duas para o mesmo lado e
  **cancela** aqui. É a conta robusta;
- **a bitola** sai da **média**. O mesmo viés **soma**. Não há cancelamento.

A bitola tinha custado quatro levas de medida e uma etiqueta trocada, e agora
três caminhos apontam o mesmo lugar — trena 32,0, desenho no Gazebo 32,25 (§5.9)
e a restrição "o corpo encosta nas rodas". **Isso descarta erro grosseiro; não é
barra de erro.** 0,320 é **nominal**.

➡️ A bitola é o divisor do `wz`: 1% aqui é 1% em todo giro. Quem a fecha de
verdade é **ensaio de pivô** com o LIO medindo o yaw, não trena.

---

## O que cada foto mostra

Nomeadas pelo **ponto de vista**, não pelo número — pelo motivo do aviso acima.

| arquivo | o que aparece |
|---|---|
| `01-boba-chao-ao-centro.jpeg` | rodinha boba (laranja) no chão, fita na vertical até o centro dela |
| `02-motoras-largura-de-frente.jpeg` | robô de frente, fita atravessando as duas motoras |
| `03-lateral-chao-ao-topo-do-corpo.jpeg` | lateral, fita na vertical do chão ao topo do corpo |
| `04-motoras-largura-de-lado.jpeg` | mesma largura, tomada de lado |
| `05-motoras-largura-de-frente-2.jpeg` | segunda tomada frontal da largura |
| `06-quina-boba-e-fundo-do-corpo.jpeg` | quina dianteira: a boba montada e o fundo do corpo |
| `07-caixa-de-cima-largura.jpeg` | caixa vista de cima, fita na largura |
| `08-caixa-de-cima-comprimento.jpeg` | caixa vista de cima, fita no comprimento |

---

## O que isto mudou no `robo3.urdf.xacro`

| | era | virou | de onde |
|---|---|---|---|
| `roda_separacao` | 0,3225 | **0,320** | (38 + 26)/2 |
| `roda_largura` | 0,058 | **0,060** | (38 − 26)/2 |
| `boba_raio` | 0,025 | **0,020** | chão → centro da rodinha |
| `altura_solo` | 0,070 | **0,065** | topo 20,0 − corpo 13,5 |

Confirmados sem mexer: caixa 0,311 × 0,240 × 0,135, topo do corpo 0,200,
comprimento do contorno 0,331.

**Duas divergências antigas morreram aqui:**

- **D3** (raio da boba): o empate 40 × 50 mm acabou. Com a rodinha apoiada, a
  altura do centro **é** o raio.
- **D4** (o suposto caimento): **nunca houve caimento** — havia
  `altura_solo` errado. 20,0 − 13,5 = 6,5, que é exatamente a medida "da boba
  até o corpo". O modelo carregava um "suporte de 10 mm" **inventado** para
  acomodar 1 cm de diferença entre duas fitas. Palavra do dono: *"ele está sim
  alinhado, não está caído não."*

**E a placa foi respondida** (§5.1, o item de maior alavancagem): *"mesmo
modelo da placa do 1 e do 2, mas não a mesma, cada um tem a sua."* O modelo de
atuador **não cai**.

🔴 **Mas a etapa 8 continua sendo MEDIÇÃO do atuador, não "conferência"** — esta
linha dizia o contrário e foi corrigida no mesmo dia. "Mesmo modelo" **não prova
mesmo firmware**, e a configuração gravada nesta placa é desconhecida. É mesma
classe e **peça diferente**: os números do `MODELO_ROBO2.md` entram como ponto de
partida, não como medida deste robô.

---

## Continua aberto

- `roda_raio` — 0,0825 provisório, fecha por **corrida reta** (etapa 8).
- **Livox** — não montado. Altura, x/y e yaw só na etapa 7.
- **Massas e centro de massa** — prematuro até o Livox e o NUC subirem (D7).

## Onde mais isto está registrado

- `docs/DIARIO.md`, entrada de 17-09
- `docs/PLANO_NAV2_ROBO3.md`, etapa 1 da tabela
- `docs/ETAPA1_MEDIDAS_ROBO3.md` — a lista que originou a sessão
- `docs/ROBO3_REVISAO_CRUZADA.md` §5.1 (placa) e §5.8 (tabela-resumo corrigida)
