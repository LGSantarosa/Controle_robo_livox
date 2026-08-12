# 024 — A recuperação que existia e era inalcançável por construção

**Data**: 2026-08-12 (4ª leva; dev, corridas no Gazebo)
**Status**: aceita e implementada — **e medida como INSUFICIENTE sozinha**
**Toca**: `path_follower.py`, `test/test_configs_coerentes.py`
**Vem de**: a corrida da porta logo depois da decisão 023
**Revisa**: a 009 (a ré por gatilho), religando-a com emprego novo

## O que aconteceu

Com o pivô fora do caminho (023), o robô voltou a percorrer o plano. Na corrida
para o alvo (6,0 · 1,5), atravessando a porta de 0,90 m, ele andou 3,31 m, o
reflexo o parou diante da ombreira — **e ele nunca mais saiu de lá**: 87 s de
CSV com a pose imóvel na mesma casa decimal, a 2,49 m do objetivo.

🟢 **De quebra, a primeira evidência do reflexo funcionando.** O
`ESTADO_PROJETO` listava o `collision_monitor` como *"configurado, NUNCA visto
parando o robô"*. Agora está medido:

```
[collision_monitor]: Robot to stop due to PolygonStop polygon
```

O canto dianteiro direito do polígono alcançava (4,07 · 1,97) e a ombreira sul
vai até y=2,05. **Ele não bateu.** Veredito do dono: *"parou antes de bater,
ótimo (...) ele não bateu, que é o principal"*.

## A corrente do travamento

```
1. reflexo para o robô a 0,34 m da parede, 0,16 m fora do centro do vão
2. planner: "failed to plan from (3.66, 2.34): Either of the start or goal
   pose are an obstacle!"                       <- o Start occupied de 29-07
3. bt_navigator: "Goal failed"        <- a árvore é a SEM recuperação (29-07)
4. sem plano novo -> path_follower: "parado (plano velho)", para sempre
```

## O defeito desta decisão: era de ORDEM

O seguidor **sempre teve** uma recuperação — a ré por gatilho da 009, disparada
por `ProgressoDeAvanco` ("não progrediu"). Ela era **inalcançável por
construção**:

```python
if plano_velho:
    self.para('plano velho'); return     # <- este return
...
if self.progresso.atualiza(t, dist):     # <- nunca alcançado
    self.entra_na_re(...)
```

E o plano vence **exatamente quando o robô trava**, porque ninguém replaneja
para um robô emperrado. A recuperação estava programada para nunca acontecer no
único caso em que faria falta.

⚠️ **Nenhum teste de valor pegaria isto** — todos os números estavam certos. É
defeito de ordem de linhas, e os testes desta decisão leem a ordem por AST.

## A decisão

```
path_follower  o despacho de `estado == 're'` vem ANTES da guarda de plano velho
path_follower  o ramo de plano velho consulta o progresso antes de desistir
path_follower  re_habilitada        False -> True
path_follower  re_max_sem_plano     novo, = 1
```

### Por que a ré volta, e por que uma das objeções de 05-08 continua de pé

Ela foi desligada em 05-08 com dois argumentos. Um morreu, o outro não se
aplica:

- *"ela atropelava o pivô"* — **não há mais pivô para atropelar** (023);
- *"recuar reto não muda RUMO"* (medido em 29-07) — **continua verdade**, e
  continua sendo motivo para não usar a ré como conserto de rumo. Aqui o
  emprego é outro: tirar o robô de uma **célula** que o planner recusa. Isso é
  posição, não rumo, e recuar reto muda posição.

### Por que a ré tem teto (`re_max_sem_plano = 1`)

Recuar tira o robô da célula recusada, mas **não ressuscita objetivo abortado**.
Sem teto o robô atravessaria a sala de ré em passos de 0,30 m — movimento que
parece recuperação e não é. Com teto, o nó grita o que de fato aconteceu:

```
recuei 1x e nenhum plano novo chegou em 21 s. Quem abortou foi o OBJETIVO
(bt_navigator), não o seguidor — sair do lugar não traz plano de volta.
```

## 🔴 E a medida que esta decisão NÃO resolve

A corrida de aceitação disparou a ré e ela **não moveu o robô**:

```
[path_follower]: EMPERRADO a 2.45 m do objetivo — ré de até 0.30 m
[path_follower]: fim da ré: recuou 0.00 m em 8.0 s        (3 vezes)
```

O CSV diz por quê, e o número é redondo:

```
amostras com ré pedida (raw_v < 0)                        831
...que passaram pelo reflexo (saida_v < 0)                  0
...vetadas                                                831   (100%)
```

**O `PolygonStop` é cego para direção.** Ele é um polígono estático de −0,28 a
+0,49 m em x com `action_type: stop`: se há ponto dentro dele, zera **qualquer**
comando. A ombreira que o parou está lá dentro — então o mesmo reflexo que o
salvou de bater é o que o impede de se afastar.

➡️ **Fica registrado como o próximo elo, e ele é do reflexo, não do seguidor.**
A saída provável é polígono que considera a velocidade comandada (o
`action_type: approach` do `collision_monitor` projeta a pegada ao longo do
twist pedido, então ré que se AFASTA do obstáculo não é vetada). Não entra hoje
porque mexer no freio de mão merece leva própria, com régua — e porque o robô
parar diante do obstáculo é o comportamento que se quer preservar inteiro.

## O que isto NÃO resolve

- **O robô entrou torto na porta** (0,16 m fora do centro de um vão de 0,90 m,
  com um corpo de 0,63 m — 13 cm por lado). Em 05-08 ele passou *"perfeitamente
  no meio"* e **tinha o pivô** para se esquadrejar antes do vão. Sem ele, arca
  para dentro. É um custo que a 023 introduziu, e merece leva própria com o
  `folga.py`, que já existe para medir exatamente isso.
- **O objetivo abortado não volta sozinho.** Nada no sistema re-manda um
  objetivo que o `bt_navigator` desistiu. Hoje quem manda é o operador.
