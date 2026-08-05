# Levantamento da camada de segurança do robô 1 (o que a decisão 010 pediu)

**Data:** 2026-08-05 · **Gatilho cumprido:** a 010 mandou rodar isto *"quando
os dados da bancada chegarem"*. Chegaram (04-08 e 05-08), e o Mid-360 entrou no
simulador (decisão 012), então dá para dimensionar contra sensor de verdade.

**Status:** levantamento, **não é aplicação**. O dono foi explícito: o
`collision_monitor` e as travas entram **depois do Nav2 impecável**.

---

## O que o robô 1 tem, e como as peças se ligam

Não são quatro nós soltos: é um **pipeline de dois mux** com o reflexo no meio.

```
nav + seguidor + porta
      │
      ▼  twist_mux_auto            (arbitra só a AUTONOMIA)
   auto_vel_pre
      │
      ▼  motion_guard    683 linhas — objeto EM MOVIMENTO
   auto_vel_raw
      │
      ▼  collision_monitor (Nav2)  — reflexo reativo, 2 polígonos
   auto_vel
      │
      ▼  twist_mux FINAL
         ├── joy_vel      prio 100   humano (PS4)
         ├── key_vel      prio  90   humano (teclado)
         ├── web_vel      prio  50   humano (web)
         ├── unstuck_vel  prio  30   desencalhe — FURA o collision
         └── auto_vel     prio  10   autonomia, já filtrada
      │
      ▼  /cmd_vel
```

**A ideia central, e ela é boa:** o reflexo de colisão filtra **toda** a
autonomia num ponto só, e o humano e o desencalhe entram **a jusante** — ou
seja, atravessam o reflexo de propósito. Isso resolve o problema real de
"quem manda quando o robô está encostado numa parede e precisa sair dela".

---

## Peça por peça, e o que sobrevive ao robô 2

### 1. `collision_monitor` (Nav2) — **porta quase inteiro, e é o de maior valor**

Não é código nosso: é configuração. Dois polígonos:

```
PolygonSlow   [[0.65,0.32],[0.65,-0.32],[-0.45,-0.32],[-0.45,0.32]]
              action "limit": linear_limit 0,10 · angular_limit 4,0
PolygonStop   type "approach" (projeta o footprint pela velocidade comandada)
```

**O que porta:** a estrutura, o `action_type: limit` e o `approach`. E há uma
lição de campo embutida que vale ouro: o lateral já foi ±0,45 e prendia o robô
a 30% da velocidade *em todo corredor* (29 desacelerações num log). Encolheram
**só o lateral**, mantendo a frente. É o tipo de erro que a gente repetiria.

**O que NÃO porta, e é um número que tem de ser re-derivado:**

- **A geometria.** Robô 1: carcaça 50×50, LiDAR no centro. Robô 2: caixa
  0,433 × 0,455, raio inscrito 0,363, e o Mid-360 **não está no centro medido**
  (a posição no URDF é provisória, decisão 012).
- **`angular_limit: 4,0` é livre de propósito** — no robô 1 capar o giro
  matava o giro-no-lugar (zona morta do skid). No robô 2 **isso não se traduz**:
  a placa entrega um `wz` só (~2,2 rad/s) e não há o que capar parcialmente.
  Limitar giro aqui ou é no-op ou é liga-desliga.
- **`linear_limit: 0,10` é IMPOSSÍVEL neste robô.** O patamar da compensação
  entrega ~0,30 m/s e não há nada entre 0 e isso. Pedir 0,10 vira "anda a 0,30"
  ou "não anda". O `limit` linear no robô 2 é, na prática, **binário**.

🔴 **E o achado que mais pesa, e que só existe porque medimos o sensor:**
o Mid-360 tem **~2,04 m de cegueira de chão** (decisão 012). Os polígonos do
robô 1 vivem entre −0,45 e +0,65 m — **inteiramente dentro da zona cega**.
Um obstáculo baixo dentro do polígono **não existe para o sensor**:

```
a 0,5 m   só aparece acima de +0,21 m
a 1,0 m   só aparece acima de +0,15 m
```

Ou seja: o `collision_monitor` no robô 2 **protege contra parede e pessoa em
pé, e é cego para caixa, degrau e pé de mesa** — exatamente o que ele existiria
para pegar. Não é defeito de sintonia; é geometria do sensor. Ou o polígono se
alimenta de **memória** (o que foi visto de longe), ou de outro sensor.

### 2. `unstuck_supervisor` (1433 linhas) — **a ideia porta; o corpo, não**

**O que porta, e é a melhor ideia do arquivo:**

- **Gatilho por DESLOCAMENTO, não por velocidade.** *"O robô tentando girar sem
  sair do lugar mexe uns mm e enganava o gatilho por velocidade. Não se
  deslocou = travado, ponto."* É agnóstico de sensor e é exatamente o BO-3
  visto de outro ângulo. **Já temos meio caminho**: o `ProgressoDeAvanco` do
  nosso seguidor é esse gatilho.
- **`rear_min_gap` em METROS**, medido do para-choque num corredor da largura
  do robô — nasceu de uma batida (11-06: checagem por setor angular era cega
  para quina, o robô recuou em cima de um obstáculo). E de outra (15-06: lata
  de lixo, `rear_stop_margin` 0,10 → 0,20). **No robô 2 isso fica mais fácil,
  não mais difícil**: o Mid-360 é 360° nativo, sem o setor cego traseiro do
  LD06 — desde que o obstáculo esteja acima da linha de cegueira.
- **Gate por STATUS do action server** do `bt_navigator` (mata a "ré póstuma"
  pós-cancel).

🔴 **O que MORREU, e é a quarta decisão a cair pelo mesmo motivo:**

> *"SEM GIRO: o giro a baixa velocidade não vence o atrito do skid-steer
> (parecia 'não fez nada'). A manobra é SEMPRE ré."*

**O robô 2 pivota** (fatia 3 da 011: 7 manobras, 7 fechadas). A premissa "a
recuperação tem de ser ré porque girar não funciona" é do **chassi do robô 1**.
Aqui, a recuperação natural é **pivotar**, e a ré é o caso raro — foi
exatamente o que 05-08 mediu ao desligar a ré e ver o caminho cair de 1,60×
para 1,13×.

Junto morrem `spin_speed: 3,0`, `spin_angle`, `spin_left_boost: 1,4` — sintonia
de skid-steer.

### 3. `motion_guard` (683 linhas) — **encolhe muito, como a 010 previu**

Compara varreduras no frame `odom`: célula livre há 0,5 s e agora com retorno =
coisa se movendo. A ideia é sensor-agnóstica e boa (nada mais na pilha distingue
móvel de estático).

**O que não porta:** ele é 2D-scan-shaped de ponta a ponta, e **~metade dele é
o filtro anti-vidro** — nascido de um defeito do LD06 (*"29 de 52 paradas eram
gente vista através do vidro"*). O Mid-360 tem artefatos **próprios e
diferentes**: hoje medimos retornos 3,8 cm **abaixo do plano do chão** em
incidência rasante. Portar o anti-vidro seria portar a defesa contra o defeito
do sensor errado.

**Veredito:** é o último da fila, e deve ser **reescrito pequeno**, não portado.

### 4. `twist_mux` — **porta inteiro, e o robô 2 NÃO TEM**

A escada de prioridade é arquitetura pura, sem sensor e sem chassi:

```
humano (100/90/50)  >  desencalhe (30)  >  autonomia (10)
```

⚠️ **Este é o buraco mais gritante do robô 2 hoje.** O `CLAUDE.md` lista
"humano tem prioridade sobre goal" como um dos poucos princípios que **valem
dos dois lados**, e a pilha atual não tem árbitro nenhum: o
`heading_controller` publica direto. Não existe hoje como um humano tomar o
controle de um robô 2 que está indo para a parede.

---

## Ordem que este levantamento recomenda

A 010 propôs `collision_monitor` → `unstuck` → `motion_guard`. O levantamento
**muda a ordem**, e por uma razão medida:

1. **`twist_mux`** — primeiro, porque é o único que não depende de nada
   (nem de sensor, nem de Nav2 impecável) e porque sem ele **não há como
   parar o robô**. É também o que o método do projeto exige.
2. **`collision_monitor`** — segundo, com polígonos re-derivados da trena do
   robô 2 e com a **zona cega escrita ao lado**, para ninguém confiar nele
   contra obstáculo baixo.
3. **Detecção por sintoma** (a metade agnóstica do `unstuck`), reaproveitando
   o `ProgressoDeAvanco` que já existe — mas com **pivô** como recuperação
   primária, não ré.
4. **Medida de vão livre** contra a nuvem, para a ré quando ela for necessária.
5. **`motion_guard`**, reescrito pequeno, por último.

## O que este levantamento NÃO resolveu

- **Anel sintético × nuvem crua**: a 010 deixou em aberto se a camada de
  segurança consome a nuvem direto ou um `/scan` derivado. Com a zona cega
  medida, a pergunta muda de forma: um anel a que ALTURA? Um anel na altura do
  sensor não vê o chão; um anel projetado no chão não vê nada dentro de 2 m.
  **Fica em aberto, e agora com o número que faltava.**
- **A altura de montagem do Mid-360 no robô real** segue não medida com trena,
  e a zona cega escala com ela (~8,1× a altura). É item de bancada barato e de
  alto retorno — cada centímetro a mais são 8 cm de raio cego.
