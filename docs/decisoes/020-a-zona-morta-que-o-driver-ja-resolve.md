# 020 — A zona morta que o driver já resolve, e a curva que ela abria

**Data**: 2026-08-12 (dev, robô desligado)
**Status**: aceita e implementada
**Toca**: `config/movimentacao.yaml`, `path_follower.py`
**Vem de**: a bancada de 07-31 (`docs/MODELO_ROBO2.md`), que mediu e nunca foi
propagada para o perfil do robô real

## O problema

`config/movimentacao.yaml` — o arquivo que a `pilha.launch.py` carrega quando
`sim:=false`, ou seja, **o perfil que roda em toda navegação no robô** — abria
assim:

```
# ⚠️ NENHUM DESTES NÚMEROS FOI MEDIDO NESTE ROBÔ AINDA.
zona_morta: 0.15
```

O aviso era verdade quando foi escrito. Deixou de ser em **07-31**, quando a
bancada mediu a zona morta linear em **0,0178 m/s** (frente) e 0,0148 (ré). A
`bitola` daquela mesma leva foi propagada (por causa do defeito de 29-07); a
zona morta e o `a_dec` ficaram para trás, e o arquivo seguiu doze dias dizendo
que nada ali tinha sido medido.

**O erro é de 8×**, e ele não fica parado no arquivo: `zona_morta` entra em três
contas da movimentação e da navegação.

## O que o número errado causava

### 1. Proibia o pivô — por aritmética, contra a máquina

Girando parado cada roda anda a `wz·bitola/2`. A lei conclui que só há pivô se
esse valor superar `zona_morta + margem`:

```
                      zona morta 0,15        zona morta 0,0178
wz mínimo para pivotar   1,48 rad/s              0,50 rad/s
teto de wz               1,00 rad/s              1,00 rad/s
veredito                 IMPOSSÍVEL              possível
```

O nó anunciava `pivô INDISPONÍVEL … pontos perto dele ficam INALCANÇÁVEIS` em
toda subida, e a saída "pivô" da `ajusta_para_zona_morta` ficava inalcançável
(`teto_de_pivo` em `wz=1,0` dava **−0,065**, negativo; com a medida dá +0,067).

**A máquina sempre pivotou.** 07-31 mediu `wz=±0,30` por 1 s dando 147,7° e
−150,0° de giro total, e o dono confirma corridas de pivô desde então. A
proibição era uma linha de YAML.

⚠️ E ela tinha **escapado do código para a documentação**: o
`ESTADO_PROJETO.md` listava "pivô indisponível" como item 4 da próxima ida ao
robô — trabalho de bancada agendado para consertar um defeito que não existe.

### 2. Abria as curvas — e o raio é a única coisa que este atuador obedece

Este é o estrago que importa para a navegação, e ele exige entender o driver.

`hoverboard_driver.cpp` escala as duas rodas **juntas** até a maior vencer o
limiar do firmware:

```cpp
double mx = fmax(|set_speed[0]|, |set_speed[1]|);
if (deadband_enable && mx > 1.0 && mx < deadband_speed) {
    double k = deadband_speed / mx;   set_speed[0] *= k;  set_speed[1] *= k;
}
```

Escalar as duas juntas **preserva a razão entre elas** e destrói o módulo. Como
a razão entre as rodas é o que define o raio do arco:

> **`cmd_vel` escolhe o RAIO. A velocidade quem escolhe é a placa.**

Medido ontem (11-08), três corridas cruas de 2,5 m com `cmd_v = 0,250`:

```
v realizada (pose do LIO)   0,305   0,292   0,298 m/s
```

Nenhuma delas é 0,25. E em 07-31 comandos de 0,10 e 0,25 m/s produziram a mesma
velocidade realizada — é o patamar.

Agora o defeito: quando o par `(v, wz)` cai na banda proibida, a
`ajusta_para_zona_morta` **acelera** o robô para tirar a roda de dentro da zona
morta. Acelerar não muda a velocidade (o patamar manda), mas **muda a razão** —
e a razão é o raio:

```
pedido (v · wz)   R pedido    com zm=0,15         com zm=0,0178
(0,10 · 0,30)      0,333 m    (0,240·0,30) 0,802   (0,108·0,30) 0,361
(0,15 · 0,50)      0,300 m    (0,268·0,50) 0,535   (0,150·0,50) 0,300
(0,25 · 1,00)      0,250 m    (0,335·1,00) 0,335   (0,250·1,00) 0,250
```

A primeira linha é a corrida de 07-31: `v=0,10 wz=0,30` foi ao chão e entregou
**raio 0,333 m**, contra 0,333 comandado — a máquina obedece ao raio. A lei
intercepta esse mesmo pedido e manda **0,802 m** para ela. **A curva abre 2,4×.**

O seguidor pede curva ao planejar desvio; em porta e corredor, 2,4× é a
diferença entre passar e raspar. É por isso que esta correção entrou na leva de
navegação e não na de sintonia.

### 3. Inflava o raio de chegada mínimo

`v_piso` (0,335) é o piso que sai da mesma conta, e dele sai o raio de chegada
mínimo `v_piso²/(2·a_lin)`: **0,187 m** contra **0,069 m** com a medida. Não
chegou a morder — `raio_chegada` é 0,25 — mas apertava o projeto sem motivo.

## A decisão

```
movimentacao.yaml   zona_morta   0.15  ->  0.0178      (medido 07-31, frente)
path_follower.py    v_piso       0.335 ->  0.203       (= 0,0178 + 0,135 + 0,05)
```

O valor de **frente** e não o de ré (0,0148) porque é o maior dos dois: entre
duas medidas, o piso conservador é o que cobre as duas.

⚠️ **0,0178 já é conservador por 2×, e vale registrar por quê.** O limiar de
verdade nesta cadeia não é atrito, é o `mx > 1.0` do driver — 1 RPM de roda, que
com `roda_raio = 0,080` dá **0,0084 m/s**. Bate com o `|cmd| < 0,008 → não move`
do `MODELO_ROBO2`. A rampa de 07-31 mediu 0,0178 porque é onde o movimento foi
*detectado*, um pouco acima do degrau. Fica-se com o número medido, não com o
calculado: ele é o mesmo fenômeno com margem.

## Alternativas consideradas

**(a) Baixar `v_max` em vez da zona morta.** Não resolve: quem distorce o raio é
o piso, e o piso não olha para `v_max`. O robô continuaria abrindo curva.

**(b) Tirar a lei da zona morta inteira, já que a compensação do driver
resolve.** Descartada, e a razão é o acoplamento: a compensação é um
**parâmetro** (`deadband_enable` no `robo2.urdf.xacro`). Desligada — e o banco
precisa desligá-la para caracterizar o atuador — a zona morta de verdade
(0,25–0,50 m/s) volta e a lei é a única proteção. Ela fica; o que estava errado
era o número.

**(c) Modelar o limiar como RAZÃO em vez de valor absoluto.** É o modelo certo:
com a compensação ligada, a roda de dentro cai no limiar do firmware quando
`interna/externa` fica pequeno demais, o que é uma condição de **raio**, não de
velocidade. Medido em 07-31: no pedido `(0,10 · 0,30)` a razão comandada era
0,42 e o encoder leu 0,22 numa das corridas — a roda de dentro ficou aquém.
**Não entra agora**: é lei nova, não parâmetro, e a mudança de hoje é de uma
linha com efeito medido. Fica registrada como a próxima pergunta desta trilha.

**(d) Remedir no robô antes de mexer.** O dono cortou, e com razão: o dado existe
desde 07-31 em `docs/dados/2026-07-31-*`, e nada mudou no atuador desde então.

## Como isto se prova

**Quatro testes novos**, e três deles fecham buracos que já tinham deixado passar
este defeito:

1. **o piso do seguidor sai da movimentação** — lê `v_piso` do
   `path_follower.py` e `zona_morta`/`bitola`/`margem_piso` do
   `movimentacao.yaml` e confere a fórmula. O comentário no código já mandava
   "TEM QUE BATER", e era só comentário: foi por essa fresta que os dois
   arquivos andaram separados;
2. **o pivô existe com os números do robô** — `pivo_disponivel` verdadeiro com o
   perfil real, travando o item que virou trabalho de bancada fantasma;
3. **a curva pedida é a curva entregue** — o par de 07-31 `(0,10 · 0,30)` sai da
   lei com o raio que a máquina mediu, e não 2,4× ele;
4. **a zona morta supõe a compensação ligada** — lê `deadband_enable` do
   `robo2.urdf.xacro`. Se alguém desligar a compensação, este teste cai e obriga
   a revisar o piso junto. É o acoplamento da alternativa (b), travado.

⏳ **Não foi ao robô.** O que a próxima ida confirma, e é barato: a subida do
`heading_controller` tem de dizer `pivô DISPONÍVEL acima de 0,13 rad/s` em vez do
`INDISPONÍVEL` de hoje. Falsificável e sem corrida.

⚠️ **O que esta decisão NÃO resolve, e não deve ser lida como se resolvesse**: a
velocidade continua não sendo comandável. Toda a lei de velocidade do seguidor
(`velocidade_de_seguimento`: frear na curva, frear perto do objetivo) é **inerte
neste robô** — ele percorre qualquer arco a ~0,30 m/s e chega no ponto nessa
velocidade. Isso é dívida aberta, e é a mesma família da alternativa (c).

## Referências

- `docs/MODELO_ROBO2.md` §1 e §2 — o patamar e as zonas mortas medidas
- `docs/dados/2026-07-31-*` — a bancada; `rodas-frente`, `curva-v010-wz030`,
  `pivo-wz030*` são as corridas citadas
- `docs/dados/2026-08-11-estimador/cru-{a,b,c}.csv` — as três corridas cruas de
  ontem, de onde saiu a velocidade realizada de 0,29–0,31 m/s
- `ros2_packages/hoverboard_driver/hardware/hoverboard_driver.cpp` — a
  compensação, e o comentário de 31-07 que já avisava que ela destrói a
  caracterização
- `docs/decisoes/019-o-default-e-o-caso-seguro.md` — mesma família: configuração
  que envelhece sem dar sintoma onde é decidida
