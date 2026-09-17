# Protocolo da etapa 2 — o ensaio de frente/ré do robô 3

> Etapa 2 do `PLANO_NAV2_ROBO3.md`. **Mede a curvatura da reta em cada sentido**
> (`κ_A` × `κ_B`) e usa o RPM por roda como **diagnóstico**.
>
> 🔴 **Ele NÃO decide sozinho "sentido ou lado"** — a abertura dizia isso e era
> promessa a mais. Separar canal de carga exige troca física de conectores com
> par controlado de RPM antes/depois, que não existe (§1).
>
> 🔴 **Não começa sem os dois pré-requisitos**, e um deles ainda falta:
> - ✅ placa decidida (17-09);
> - 🔴 **parada física independente do Xbox**;
> - 🔴 **modo de medir o desvio** — este documento resolve.

---

## 0. Por que este ensaio existe

O `ESTADO_PROJETO` registra: **a ré andava reto e a frente puxava para a
direita**. A decisão 049 contornou promovendo a ré a frente, e a hipótese do
plano (§2) é que isso é o **sentido mecanicamente estável**: com as motoras à
frente, as bobas são **arrastadas**; no outro sentido, **empurradas** — a
configuração instável da decisão 009.

**Hipótese não é resultado.** Este ensaio decide.

---

## 1. 🔴 O que a v1 do plano errava, e por que o teste mudou

A v1 dizia: *"se o puxão só trocar de lado, a causa é de canal."* **Não prova.**
Assimetria de roda, de carga e o transiente das bobas dão o mesmo sinal.

O discriminador honesto é outro, e é barato: **trocar os canais L/R e repetir.**

| o puxão segue… | causa | o que fazer |
|---|---|---|
| o **canal** (troca de lado quando L/R trocam) | roda, motor, realimentação ou cabo — é **elétrico/mecânico de um lado** | conserta na origem; o contorno de 049 não resolve |
| o **sentido** (fica no mesmo lado do corpo, apesar da troca) | **geometria**: bobas arrastadas × empurradas | a hipótese do plano se confirma, e 049 é o conserto certo |

🔴 **ERRO GRAVE DESTA SEÇÃO, corrigido em 17-09 — leia antes de executar.**

A versão anterior dizia que inverter `left_wheel_sign` e `right_wheel_sign`
**juntos** trocava os canais. **Não troca.** O próprio `cmd_vel_to_wheels.py`
diz o contrário, em texto escrito no mesmo dia: inverter os dois é um
**ESPELHO** — inverte frente **e** giro —, e `linear_sign` é **rotação**.
*Rotação ≠ reflexão* (decisão 049). Nenhum dos dois troca qual roda física
recebe qual setpoint, e **não existe parâmetro de swap L/R no nó**.

➡️ **Consequência: o discriminador do §1 NÃO é executável hoje**, e este
protocolo não deve ser rodado como se fosse. Trocar canal de verdade exige uma
das duas:

| caminho | avaliação |
|---|---|
| **trocar os conectores dos motores** na placa | é a troca de verdade. Física, com o robô **desligado**. ⚠️ **Já foi feita uma vez, em 14-09** — e por isso a próxima só vale com **par controlado de RPM antes/depois**: da outra vez o "antes" foi observação a olho, e é justamente o que impede de fechar canal × carga hoje |
| ~~parâmetro `swap_lr` no `cmd_vel_to_wheels`~~ | 🔴 **NÃO FUNCIONARIA, e não é falta de código — é aritmética.** O `mega_bridge` manda para a placa `speed = (L+R)/2` e `steer = (L−R)/2`. Numa **reta** `L == R`, então `steer = 0` e trocar L↔R produz **o frame idêntico**. Um swap de software é literalmente um no-op aqui |

➡️ **O que dá para fazer sem troca nenhuma**, e com dado que o bag já grava:
ler o **RPM por roda** (`/hoverboard/wheel_velocities`). Com os setpoints
iguais, ele separa "a tração está assimétrica" de "a tração está simétrica e o
robô puxa mesmo assim". Ver §6.

⚠️ O ensaio mede a **curvatura por sentido** (A × B) — o que já responde a
pergunta da decisão 049 — e o RPM estreita a causa. O que continua fora de
alcance é **isolar canal elétrico de carga**, e isso é honesto dizer.

---

## 2. Como medir sem pose

O bag do `bin/sobe-robo3` **não grava pose**: o robô 3 **terá** o Mid-360 — ele
só ainda **não está montado**, e a pose dele no URDF é provisória. Enquanto isso,
o desvio se mede **no chão**, e é isso que dá o número.

```
   partida                                            chegada
      │                                                  │
      ├──────────────── L = 3,00 m ─────────────────────►│
      │  linha de referência (fita no chão)              │
      │                                                  │
      │                                             d ↕  ← desvio lateral
      │                                                  ●  onde o robô parou
```

**Instrumentação:** uma fita de referência no chão, um ponto marcado no robô
(pode ser um prumo pendurado no centro do eixo), e a trena para `d`.
Alternativa melhor se houver tripé: celular filmando de cima da chegada.

**A conta**, para desvio pequeno (`d ≪ L`) — é curvatura constante equivalente:

```
raio       R  ≈  L² / (2·d)
curvatura  κ  ≈  2·d / L²          [1/m]
```

Exemplo: `L` = 3,00 m e `d` = 12 cm → `κ ≈ 0,027 1/m`, `R ≈ 37 m`.

➡️ **Reportar `κ`, não `d`.** `d` depende do comprimento da corrida; `κ` é a
propriedade da máquina, e é comparável com o `curv_frente` do robô 2 (−0,8365).

---

## 3. As quatro condições que fazem a corrida valer

1. **Comprimento e velocidade fixos.** `L` = 3,00 m, e o comando **só pelo
   direcional** (`dpad_reto`), nunca pelo analógico: o analógico escapa de lado
   e aí não se sabe se o desvio foi da mão ou do robô. Sem turbo.
2. **Orientação inicial marcada.** O robô parte alinhado com a fita, e isso se
   confere **antes de cada corrida** — não uma vez no começo da leva.
3. 🔴 **Alinhamento prévio das bobas.** Elas têm memória do movimento anterior:
   uma boba virada de lado no arranque dá um chute inicial que não é do sentido
   em teste. **Antes de cada corrida**, empurre o robô ~0,5 m **no sentido que
   vai ser testado**, para as bobas assentarem.
4. **Tensão da bateria anotada a cada corrida.** A placa entrega diferente com a
   bateria caindo, e uma leva inteira num sentido seguida de outra no outro
   sentido confunde queda de bateria com efeito de sentido.

---

## 4. A ordem, e por que ela é alternada

**6 corridas por sentido, em blocos CONTRABALANCEADOS** (ABBA / BAAB):

```
A  frente (motoras à frente)
B  ré

bloco 1:  A B B A
bloco 2:  B A A B
bloco 3:  A B B A
```

🔴 **Por que não `A B A B`** — era o que esta seção dizia, e está errado: nessa
sequência **A vem sempre antes de B**, então a queda de bateria empurra todos os
B para o fim e vira exatamente o "efeito de sentido" que se quer medir.
`ABBA/BAAB` equilibra a posição média de cada sentido na ordem, que é o ponto do
contrabalanceamento.

São 12 corridas de 3 m, e **anote a tensão a cada uma** — o contrabalanceamento
reduz o viés, não substitui o registro.

⚠️ As configurações C e D (canais trocados) **saíram por enquanto**: ver o §1,
não há como trocar canal hoje.

---

## 5. Planilha de anotação

```
corrida | sentido | tensão | d (cm) | lado do desvio | observação
   1    |    A    |  __,_  |   __   |  esq / dir     |
   2    |    B    |  __,_  |   __   |  esq / dir     |
   3    |    B    |  __,_  |   __   |  esq / dir     |
   4    |    A    |  __,_  |   __   |  esq / dir     |   <- fim do bloco ABBA
```

⚠️ O **RPM por roda não vai nesta planilha**: ele já está no bag
(`/hoverboard/wheel_velocities`), e eu leio de lá. Anote só o que a trena e o
olho veem — o resto o robô grava sozinho.

⚠️ **"Lado do desvio" é em relação ao SENTIDO DE MARCHA**, não à sala. Anote
como se você estivesse sentado no robô, olhando para onde ele vai.

---

## 6. Como eu leio

1. **Média e dispersão de `κ`** nos dois sentidos, A e B. (C e D não existem —
   ver §1: não há como trocar canal hoje.)
2. **`κ_A` contra `κ_B`** — é o que o ensaio decide de fato: o sentido com as
   motoras à frente anda mais reto que o outro? Isso confirma ou derruba a
   premissa do §2 e a decisão 049.
3. **O RPM por roda, do bag** (`/hoverboard/wheel_velocities`, já gravado).

   🔴 **E isto NÃO é ideia nova — já foi medido em 14-09**, e o ensaio novo tem
   de bater contra aquilo em vez de fingir que começa do zero
   (`docs/dados/2026-09-14-robo3-gnd-xbox-reta/`):

   | condição | FL × FR |
   |---|---|
   | **no ar**, +250 | L = R (**−0,1 %**) |
   | no ar, −250 | L **+2,7 %** |
   | **no chão, cabos JÁ trocados**, frente | **+9,8 %** (3/3 corridas) |
   | no chão, ré | **−1,8 %** |

   ➡️ A leitura: **no ar os canais são simétricos; no chão, não** — e isso com
   os conectores já trocados. Aponta para algo que **depende do contato com o
   solo**, não para um desequilíbrio puramente eletrônico.

   ⚠️ **Mas não fecha**, e é honesto dizer por quê: falta um **par controlado de
   RPM antes/depois** da troca de conectores. O "antes" de 14-09 foi observação
   a olho.

   ⚠️ **E RPM iguais NÃO provam "causa fora da tração"** — a versão anterior
   desta seção dizia isso e era exagero. Provam só que o puxão **não veio de
   diferença de velocidade angular medida nos eixos**. Continuam de pé: raio
   efetivo diferente, deformação do pneu, escorregamento e carga desigual.
4. Se a dispersão dentro de um sentido for da ordem da diferença entre os
   sentidos, **não decide nada** — e a resposta honesta é "precisa de mais
   corridas ou de instrumentação melhor", não escolher a hipótese preferida.

---

## 7. Segurança

- 🔴 **A parada física independente do Xbox é pré-requisito**, não recomendação.
  O homem-morto do LB depende do controle, do Bluetooth e da pilha ROS de pé —
  e um ensaio é justamente quando se mexe nos três.
- Espaço livre de pelo menos `L` + 2 m à frente, e ninguém no corredor de teste.
- Há registro da placa **girar as rodas sozinha** com a MEGA mandando zero
  (decisão 048): energizar só com todo mundo fora do caminho.
