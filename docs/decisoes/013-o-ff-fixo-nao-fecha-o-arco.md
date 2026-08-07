# 013 — O feedforward fixo não fecha o arco, porque a planta muda de dia

**Data:** 2026-08-06
**Estado:** análise fechada; **escolha feita pelo dono em 07-08 — caminho 3**
(medir a curvatura crua no começo de cada sessão e passar por parâmetro)
**Depende de:** [011 — malha fechada de rumo em reta](011-malha-fechada-de-rumo-em-reta.md)

## O que se sabia, e o que mudou

A decisão 011 pôs um feedforward de curvatura no compensador:

    ff = −curv_frente · |v|        curv_frente = −0,817 1/m  (medido em 04-08)

A ideia era simples: o robô comandado a ir reto arca; se a gente sabe **quanto**
ele arca, é só somar o oposto. E funcionou — o arco caiu de −0,82 para a casa
de 0,04–0,08 1/m.

O que a sessão de 06-08 mostrou é que **o resto não é resíduo de ajuste fino: é
a planta mudando de um dia para o outro.**

## A medida que fecha o argumento

Curvatura **crua**, sem compensador nenhum, três corridas por dia:

```
             média       faixa               dispersão no dia
04-08      −0,8031   −0,8282 a −0,7832            3%
05-08      −0,9116   −0,9310 a −0,8958            2%
```

Duas coisas ao mesmo tempo, e é o contraste entre elas que importa:

- **dentro do dia o robô é muito repetível** — 2 a 3% de dispersão;
- **entre dias ele muda 13,5%**, e as faixas **não se sobrepõem**: 04-08 não
  passa de −0,828, 05-08 não sobe acima de −0,896.

Não é ruído de medida. É a planta em dois estados diferentes.

## Por que isso condena o número fixo

Com `curv_frente = −0,817` (perto do valor de 04-08), num dia como 05-08 sobra:

    −0,9116 + 0,817 = −0,095 1/m

que é exatamente a ordem do resíduo medido em 06-08 (0,07 a 0,09 1/m nas três
condições). Calibrar para 05-08 apenas moveria o erro para os dias como 04-08.
**Nenhum valor único serve para os dois**, e o limiar de aceitação da 011
(`|curvatura| < 0,05`) fica dentro da faixa em que a planta se move sozinha.

## E por que não dá para calibrar pelo resíduo com o compensador ligado

Foi a primeira coisa que tentei, e não se sustenta. Se o modelo fosse linear e
o integrador não existisse, valeria `curv_novo = curv_atual + resíduo`. Mas em
05-08:

    esperado sem integrador : −0,9116 + 0,817 = −0,095
    medido com compensador  : +0,0417

O sinal inverte. Quem faz isso é o **integrador**, que trabalha durante a
corrida — então o resíduo medido com a malha fechada não é uma leitura do erro
de `ff`, é o que sobrou depois de o integrador ter comido parte dele. Some-se
que as corridas duram 4,7–5,0 s (trava de espaço) e nem chegam ao regime.

➡️ **Corolário prático: `curv_frente` só se mede com o compensador DESLIGADO.**

## O que isso sugere como conserto — e por que não escolhi sozinho

O problema deixou de ser "achar o número certo" e virou "seguir um número que
muda". Três caminhos, e eles têm custos diferentes:

1. **Dar autoridade ao integrador para absorver a diferença.** É o que um
   integrador existe para fazer. O impedimento está medido e é sério: baixar o
   `ki` 4,1× encolheu `ki·int_max` de 0,30 para 0,072 rad/s, e subir o
   `int_max` para 2,5 para compensar **trouxe o sino de volta** (9° de segunda
   excursão) — com tempo morto de 0,94 s aquele teto não é só autoridade, é
   proteção contra windup. Mexer aqui é mexer no que a sessão de 06-08 acabou
   de provar que funciona.
2. **Estimar o `ff` online** (identificação recursiva do arco durante a
   marcha). Resolve a deriva entre dias por construção, e é o caminho com mais
   trabalho e mais risco — um estimador que diverge é pior que um número fixo
   errado.
3. **Medir a curvatura crua no começo de cada sessão** e passar por parâmetro.
   Barato, honesto, e não é automação nenhuma — vira um passo do protocolo de
   bancada, três corridas antes de qualquer coisa.

**Não escolhi.** A (1) mexe no que acabou de ser validado no robô, a (2) é uma
mudança grande que o método deste projeto manda não levar blind para a máquina,
e a (3) muda o protocolo de trabalho do dono. É decisão dele.

## ✅ A escolha (2026-08-07): caminho 3

O dono escolheu **medir a curvatura crua no começo de cada sessão** e passar o
`curv_frente` por parâmetro. Consequências práticas:

- as **três corridas sem compensador** deixam de ser "o experimento que falta" e
  viram **passo fixo do protocolo de bancada**, antes de qualquer outra medida
  do dia — já estão como experimento nº 2 em `docs/PROXIMA_SESSAO_NO_ROBO.md`;
- o valor medido no dia entra por parâmetro; **nenhum número fixo de
  `curv_frente` no YAML deve ser tratado como verdade** — é só o último valor
  visto;
- os caminhos (1) e (2) ficam **descartados por ora**, não refutados: se a
  tendência entre dias virar previsível com três ou mais pontos, o (2)
  (estimador online) volta à mesa com dado para se justificar.

## O que fica pendente de medida

Falta a curvatura crua de **06-08** — a sessão foi direto para as condições com
compensador e não repetiu a linha de base sem ele. Com ela seriam três dias e a
tendência ficaria caracterizada em vez de inferida de dois pontos. **Três
corridas sem compensador no começo da próxima sessão** resolvem, e são o
primeiro item do ensaio de qualquer um dos três caminhos acima.

## Referências

- `docs/DIARIO.md`, 06-08 (3ª leva) — a sessão que levantou isto
- `docs/dados/2026-08-04-aceitacao-simulador/normal-frente-*.csv` — cru, 04-08
- `docs/dados/2026-08-05-bancada-robo/A1-sem-comp-frente-*.csv` — cru, 05-08
- `docs/dados/2026-08-06-sintonia-rumo/` — as nove corridas com compensador
