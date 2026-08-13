# 038 — O freio linear, e o ganho da cadeia de giro

**Data**: 2026-08-13 (dev + Gazebo, com o dono olhando a tela o tempo todo)
**Status**: as duas mudanças implementadas e medidas **no simulador**. No robô
real as duas nascem no valor neutro — ver "O que vai para o robô", que é a parte
que importa antes da próxima sessão com ele ligado.
**Toca**: `lei_de_freio_linear.py` (novo), `lei_de_reta.py`,
`compensador_rumo.py`, `pilha.launch.py`, `tools/banco/freio_linear.py` (novo),
`tools/banco/ganho_de_giro.py` (novo)
**Vem de**: o robô batendo na porta com o dono olhando, e a frase dele depois:
*"prioridade maxima em fazer ele NÃO BATER"*

---

## 1. O freio linear — a batida

O dia começou com o robô entrando na porta. A corrida gravada
(`docs/dados/2026-08-13-re-na-porta/volta-pra-sala.csv`) dá o instante:

```
t=21,29 s  o reflexo começa a frear: raw_v 0,32 -> saída 0,29 0,27 0,25 0,22 0,13
t=21,85 s  o reflexo ZERA a saída          folga 0,30 m
t=22,31 s  o robô para                     folga 0,20 m
```

**O reflexo não falhou**: cortou o comando a 0,30 m da parede. O que não existia
era o freio. A meia-largura do corpo é 0,2275 m (trena, 29-07), então parar a
0,20 é encostar. Os 0,10 m foram andados com o comando em ZERO — a retenção de
`atraso_desliga` (0,52 s, medida no robô em 04-08).

A 037 mediu e resolveu isso no eixo do GIRO. Este é o mesmo mecanismo no eixo
que ficou de fora, e era o eixo que estava batendo.

### A medida (`tools/banco/freio_linear.py`, campo aberto)

```
limiar  freou por   SOBRA        leitura
 0.35     0.33 s   -0.006 m
 0.25     0.44 s   -0.010 m
 0.25     0.04 s   +0.018 m
 0.20     0.00 s   +0.107 m   <- freio NÃO engatou: é a LINHA DE BASE
 0.20     0.00 s   +0.127 m   <- idem
 0.15     0.54 s   -0.020 m
 0.15     0.50 s   -0.084 m   <- passou do ponto e voltou 8 cm
```

As duas linhas que não engataram são a melhor testemunha do ensaio: **sem freio
a sobra é +0,107 e +0,127 m**, em cima dos +0,100 m medidos na batida. Com o
freio engatado, entre −0,084 e +0,018 m.

⚠️ **A varredura não resolve o limiar ótimo**, e isso está escrito na lei
também: o portão `pico_min` da bancada domina o instante de soltar, então o
medido é "engatou / não engatou". Para a segurança tanto faz — qualquer engate
corta a invasão em ~10x e o preço do exagero é recuar alguns centímetros, para
LONGE da parede.

### Onde ele mora, e por que não pode ser em outro lugar

No `compensador_rumo`, a **última camada antes do atuador**. Quem corta o
comando é o reflexo, que está ACIMA: um freio antes do `collision_monitor`
teria o próprio contra-torque zerado por ele — morreria no instante em que é
preciso. O argumento já estava escrito no `twist_mux.yaml` para o arco: *"o arco
do corpo é do robô, não da fonte"*. A inércia da placa também é.

**Freio que alguém pode vetar não é freio.** E por morar ali ele pega o corte de
QUALQUER fonte — reflexo, humano ou autonomia — sem abrir furo novo na cadeia de
segurança (nenhum canal novo no mux, nenhuma prioridade nova).

### Verificação

```
sobra depois do corte, mesma bancada, mesmo trecho
sem freio    +0,107   +0,127 m
com freio    +0,048   +0,031 m
```

---

## 2. O ganho da cadeia — a ré na porta

Com o freio de pé o robô parou de bater, mas continuou precisando de **ré** para
passar a porta. O dono, e ele estava certo: *"se ele fizesse a curva para ir de
frente para a porta mais fechada ele não precisaria"*.

### A hipótese que eu tinha e que estava ERRADA

Eu disse que o compensador "desliga na curva". Não desliga — a `lei_de_reta`
soma o feedforward também na curva pedida, e o comentário no código diz por quê
("o arco existe girando também: ele é do CORPO, não do comando"). Quem me
enganou foi a mensagem de subida do nó, desatualizada. **O que a curva dispensa
é o PI de rumo, não o cancelamento do arco.**

### O que a conta mostrou, refeita COM o `ff` somado

```
janela    pedido  +ff     = comando   arco real   medido   ganho
10-12 s    +0,15  +0,253    +0,403     −0,253     −0,03     0,55
15-17 s    +0,23  +0,229    +0,459     −0,229     +0,02     0,54
19-21 s    −0,47  +0,253    −0,217     −0,253     −0,34     0,40
```

E a bancada dedicada (`tools/banco/ganho_de_giro.py`, campo aberto, janela de
regime de 3 s) confirmou:

```
wz pedido   wz real    leitura
   0,00     −0,116     o arco sobrando, sem ninguém pedir curva
  +0,15     −0,042     pediu esquerda, foi para a direita
  +0,30     −0,001     pediu esquerda, ficou reto
  +0,50     +0,050     10% do pedido
  −0,30     −0,297     99%
  −0,50     −0,378     76%
```

`g = 0,433` pela regressão; refeito linha a linha com o `v` de cada corrida dá
0,30–0,57, **mediana 0,45**. O intercepto (−0,129 rad/s) bate com o previsto
pelo modelo `−(1−g)·|curv|·v` (−0,099) — que é o critério de aceitação do ensaio.

**A assimetria não é um segundo defeito**: o arco puxa para a direita, então
pedido para a direita soma e pedido para a esquerda apanha.

### O conserto

O `ff` é calculado em rad/s de giro DESEJADO, mas quem executa entrega `g` disso.
Sem dividir, o cancelamento do arco chega a 45% e sobra `(1−g)·|curv|·v ≈ −0,12`
rad/s — **do tamanho exato da curva suave que sumia**. Dividindo a saída por `g`,
o arco é cancelado inteiro E a curva pedida sai do tamanho pedido.

### Verificação, mesmo alvo e mesmo ponto de partida

```
                       bateu (13h)   só freio    freio + ganho
chegou                     não        SIM 39,5s    SIM 30,8s
folga mínima             0,20 m       0,35 m       0,35 m
reflexo cortou             12%          12%          0%
ré ativa                    —            6%          0%
caminho / reta            1,40x        1,40x        1,21x
erro de trajeto p50      0,034 m      0,033 m      0,024 m
```

Zero ré, zero corte do reflexo — ele atravessou sem nunca chegar perto o
bastante para o reflexo precisar agir, que é diferente de "o reflexo salvou".

---

## 3. O que isto ABRIU, e ficou aberto: o S na reta

O dono, na mesma corrida: *"nas curvas tá muito melhor, mas ele não anda mais
reto, saiu metendo vários S"*. Medido:

```
                   varreu   líquido   desperdício
sem ganho            323°      63°      260°  (26,9°/m)
com ganho            511°      63°      449°  (53,5°/m)
```

**A guinada desperdiçada dobrou.** E a causa não é laço rápido demais — o
comando trocou de sinal 3x em 30 s, MENOS que antes. É que a lei de rumo é
`wz = √(2·a_dec·|e|)`, íngreme perto de zero: com `a_dec 0,3`, 6° de erro pedem
0,245 rad/s. Com a placa segurando 0,52 s depois do corte, essa correção varre
~7° **depois** do comando zerar: passa do ponto sozinha, e a seguinte vem para o
outro lado. Antes isso ficava escondido porque a planta engolia o pedido.

### Duas tentativas de conserto, as duas FALHARAM

| tentativa | reta | curva | veredito |
|---|---|---|---|
| mira do seguidor 0,30 → **0,55 m** | `\|wz\|` 0,236 → 0,235 | 0,346 → 0,226 | não arrumou a reta e amoleceu a curva |
| tolerância de rumo 0,02 → **0,12 rad** | — | reflexo cortou **77%** | **não chegou ao alvo** |

A segunda ensina o que restringe o problema: **reta e porta puxam para lados
opostos com um knob global**. Longe e rápido, tolerar erro ajuda; perto e
devagar, tolerar erro cega — ele deixa de afinar o rumo na chegada, entra torto,
e o reflexo segura.

### A próxima tentativa, NÃO RODADA

`a_dec` de 0,3 para 0,10 — mexer em QUANTO corrigir, não em QUANDO. Erro de 6°
passaria a pedir 0,11 rad/s (em vez de 0,245) e erro de 57° ainda pediria 0,45,
então a porta continua sendo corrigida. O YAML ficou em **0,3**, que é o valor
da corrida boa: valor não medido não entra na frente de valor medido.

---

## O que vai para o robô

🔴 **O freio linear vai LIGADO, e é a primeira vez que ele roda na máquina de
verdade.** A física que o justifica foi medida NO ROBÔ (a retenção de 0,52 s,
04-08), mas os parâmetros (`solta_em 0,25 m/s`, `pico_min 0,12`) saíram do
Gazebo. Antes de navegar com ele lá: rodar `tools/banco/freio_linear.py` num
trecho livre e comparar a sobra medida com a daqui.

🟢 **O ganho da cadeia vai NEUTRO (1,0)** — identidade, o nó se comporta como
antes. O 0,45 é default só quando `sim:=true`. Medir no robô com
`tools/banco/ganho_de_giro.py` antes de ligar lá; se o ganho real for ~1, a ré
na porta tem outra causa no robô e este registro fica valendo só para o
simulador.

🟢 **`a_dec`, tolerância e mira**: os três estão nos valores de sempre. As
mudanças que tentei e falharam vivem só no `movimentacao_sim.yaml`, como
comentário.

## Referências

- 037 — o freio de giro, de onde sai o mecanismo inteiro (contra-torque, soltar
  cedo, esperar o pico, teto de tempo);
- 011 — o compensador de rumo e o feedforward de arco;
- 020 — a placa entrega um módulo só: o comando escolhe o raio, não o módulo;
- 013 — o protocolo de "knob de planta se mede na máquina, não se herda".
