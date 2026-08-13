# 032 — O planejador precisa de GRADIENTE, não de raio maior

**Data**: 2026-08-14 (dev, robô carregando; tudo medido no Gazebo)
**Status**: aceita e implementada, verificada em 3 corridas + 1 regressão
**Toca**: `config/nav2.yaml`, `test_configs_coerentes.py`
**Revisa**: a decisão 029 (a inflação de 0,20) sem apagá-la
**Vem de**: o dono, olhando o RViz: *"ele tá querendo colar nos obstáculos e
paredes ao invés de ir pelo meio... até no RViz dá para ver que o desenho do
robô claramente não passa onde a linha diz que passa"*

## O que ele viu, e por que ele estava certo

O planejador **é o Theta\***, não o Smac (aposentado em 05-08). E o Theta* não
tem footprint nenhum: ele bloqueia célula com custo `> 252` e pronto
(`MAX_NON_OBSTACLE_COST` no `theta_star.hpp` do Jazzy). Então o tamanho do robô
entra no plano por **uma via só**:

```
raio bloqueado = min(inflation_radius, raio inscrito do footprint)
```

E daí saem as duas coisas que o dono viu, que parecem opostas e têm a mesma raiz:

**1. Ele planejava onde o corpo não cabe.** Com `inflation 0,20` e
`robot_radius 0,32`, o bloqueio era 0,20 — o planejador acreditava num robô 38%
menor do que o real (circunscrito medido: 0,314).

**2. Ele colava na parede.** Este é o achado que eu não tinha entendido: com
`inflation_radius <= raio inscrito` **não existe faixa graduada nenhuma**. A
`inflation_layer` marca 253 tudo dentro do inscrito e só gradua o que está
ENTRE o inscrito e a inflação. Se a inflação não passa do raio, essa faixa é
vazia:

```
célula perto  253 (proibida)
célula longe    0 (de graça)
nada no meio
```

A paisagem de custo fica **binária**, e o Theta*, que minimiza distância, raspa
a borda do proibido. *"Ir pelo meio"* não é uma preferência que ele tinha e
perdeu — ela nunca existiu.

## O que foi tentado e reprovado no caminho (as duas pontas erradas)

**`inflation 0,33` com `robot_radius 0,32`** — subir o bloqueio até o corpo.
Reprovou na hora: bloquear 0,324 proíbe **15,1% das células livres da sala**
(varredura sobre `sala_andar3.pgm`), incluindo aquela onde o robô está, e o
planejador devolve `Could not generate path between the given poses`. Numa porta
de 0,90 m sobra um canal de 0,25 m para o centro. É o travamento de 29-07 outra
vez, com outro número.

**`footprint` poligonal no lugar de `robot_radius`** — dar o retângulo real
(0,433 × 0,455). Errado para ESTE planejador: com contorno o raio inscrito cai
para 0,2165, o bloqueio DIMINUI, e o Theta* passaria ainda mais perto. Contorno
só ajudaria com um planejador que faz checagem de footprint (o Smac faz; o
Theta* não). Ficou registrado porque é a saída certa se um dia o Smac voltar.

## A decisão

```
robot_radius       0,32 -> 0,26     nos dois costmaps
inflation_radius   0,20 -> 0,90     nos dois costmaps
cost_scaling_factor      3,0        intocado
```

**`robot_radius: 0,26` fica na única faixa que serve**, e ela é estreita:

```
>= 0,26   a meia-largura que o polígono do reflexo exige. Abaixo disso o
          planejador manda por onde o reflexo veta e o robô fica preso entre
          os dois — o impasse que o dono previu de véspera
<= 0,314  o circunscrito medido. Acima disso o Theta* recusa planejar de 15%
          da sala, inclusive de onde o robô está
```

O que sustenta 0,26 geometricamente: para **atravessar** um vão o que aperta é a
LARGURA da caixa (0,455, meia 0,2275), não o circunscrito (0,314) — este só
manda quando o robô gira parado, e girar perto de parede é o caso que o reflexo
cobre.

**`inflation_radius: 0,90` é o que cria o gradiente.** Com bloqueio em 0,26, a
faixa de 0,26 a 0,90 tem custo decaindo (`253·e^{-3(d-0,26)}`: 139 a 0,46 m, 76
a 0,66 m, 37 a 0,90 m), e as inflações das duas paredes se encontram no meio do
vão. É o gradiente que empurra o caminho para o centro — e ele só passou a
existir agora.

⚠️ **Isto não é voltar aos 0,50 de 13-08** (decisão 029). Lá a inflação também
BLOQUEAVA até 0,324 e encarecia a sala inteira: o plano saía torto, caminho
1,45× a reta. Aqui o bloqueio é 0,26 fixo e a inflação só inclina o custo. É a
mesma palavra para duas coisas diferentes, e foi essa confusão que me fez errar
duas vezes no mesmo dia.

## A medida

Alvo (6,24 · 3,51), o que o dono mandou e que falhava. Folga = distância do
CENTRO do robô à parede mais próxima, ao longo do trajeto executado:

```
                                       chegou   cam/reta   folga min   folga p10
antes  (raio 0,32 · inflação 0,20)       não      1,60x      0,35 m      0,40 m
depois (raio 0,26 · inflação 0,90)       SIM      1,40x      0,35 m      0,35 m
```

⚠️ Honestidade sobre este número: **a folga mínima não melhorou** — o ganho da
032 sozinha foi o plano deixar de morrer (`Could not generate path`) e o
caminho ficar navegável. Quem fez o robô chegar foi a combinação com as decisões
033 (reflexo com direção) e 034 (árvore tolerante). Nenhuma das três sozinha
atravessa.

## Referências

- decisão 029 — a inflação que entortava o plano (a que esta revisa)
- decisão 033 — o reflexo ganha direção
- decisão 034 — a árvore sobrevive a um replanejamento que falha
- `theta_star.hpp` do Jazzy — `MAX_NON_OBSTACLE_COST = 252`
- dados: `docs/dados/2026-08-14-sim-porta/`
