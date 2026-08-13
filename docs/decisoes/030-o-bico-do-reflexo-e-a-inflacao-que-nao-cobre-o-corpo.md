# 030 — O bico do reflexo, e a inflação que não cobre o corpo do robô

**Data**: 2026-08-14 (dev, robô desligado/carregando)
**Status**: o bico está implementado; a inflação está **aberta**, aguardando o
sweep no simulador
**Toca**: `config/collision_monitor.yaml`, `test_configs_coerentes.py`
**Vem de**: 13-08 no robô — a 2ª tentativa de porta terminou em contato, e o
critério do dono passou a ser *"ele não pode bater de jeito nenhum"*

## O que o dono viu, e o que a conta diz

> *"ele para muito próximo das paredes e objetos"*

Não é impressão. O polígono de parada foi dimensionado para cobrir a distância
de parada **exatamente**:

```
gatilho à frente do para-choque   0,49 − 0,2165 (meia caixa)  = 0,2735 m
distância de parada MEDIDA        0,149 (coast) + 0,119 (freio) = 0,2680 m
                                                   margem         0,0055 m
```

**Fator de segurança 1,02.** O reflexo estava projetado para parar encostando —
5,5 mm é menos que a espessura da tinta da parede. E ele disparou de verdade em
13-08: dez eventos `Robot to stop due to PolygonStop polygon` só no log da
inflação 0,20. Qualquer coisa que alongue o coast (bateria fraca mexendo no
patamar da placa, piso, carga) vira contato.

## Por que um BICO e não uma caixa maior

Aumentar a caixa inteira resolveria a parede e **estragaria a porta**: a caixa
cresce em todas as direções, e num vão de 0,90 m o que decide se o reflexo
dispara dentro do vão é a **quina**, não o nariz. Os dois casos usam partes
diferentes do polígono:

```
parede de frente  ->  quem chega primeiro é o NARIZ (centro da frente)
batente da porta  ->  quem raspa é a QUINA (a lateral)
```

Entrando torto de um ângulo θ, a largura varrida por um vértice `(x, y)` é
`x·senθ + y·cosθ`. Com meia-porta = 0,450:

```
     θ      quina (0,49 / 0,26)     bico (0,57 / 0,20)     quem manda
    0°           0,260                   0,200               quina
   10°           0,341                   0,296               quina
   15°           0,378                   0,341               quina
   20°           0,412                   0,383               quina
```

O bico só passaria a ser o ponto mais largo se `y_bico > 0,26 − 0,08·tanθ`, o
que com `y_bico = 0,20` acontece **acima de 37°** de erro de rumo — ângulo em
que a passagem já falhou por outro motivo. **Custo na porta: zero.**

```
antes  [[0.49, 0.26], [0.49,-0.26], [-0.28,-0.26], [-0.28, 0.26]]
agora  [[0.57, 0.20], [0.57,-0.20], [0.49,-0.26], [-0.28,-0.26],
        [-0.28, 0.26], [0.49, 0.26]]

margem de frenagem   0,0055 m  ->  0,0855 m
```

Dois testes novos, os dois verificados por mutação (restaurando o polígono
antigo eles reprovam):

- `test_o_poligono_tem_MARGEM_de_frenagem_e_nao_so_a_conta` — exige 5 cm de
  folga sobre a conta medida, para ninguém voltar ao fator 1,02;
- `test_o_bico_nao_pode_estreitar_a_porta` — varre θ de 0° a 36° e exige que a
  quina continue sendo o ponto mais largo.

## 🔴 O BO que a pergunta do dono abriu, e ele é maior que o bico

> *"o nav2 não tem que acompanhar essa mudança? pra que ele não pare sozinho
> mesmo que não fosse bater? ai se o nav2 achar que passa e o colision falar que
> não ele vai ficar preso"*

A preocupação está certa e a ordem correta é **o planejador tem de ser MAIS
exigente que o reflexo**: tudo que o Nav2 propõe precisa caber no que o reflexo
aceita, senão o robô fica parado entre uma camada que insiste e outra que veta.
Hoje a ordem está **invertida**, e não por causa do bico:

```
reflexo exige de folga lateral (do centro)                    0,26 m
planejador bloqueia célula a até                              0,20 m   <- inflation_radius
corpo do robô (circunscrito da caixa 0,433 × 0,455)           0,314 m
```

A `inflation_layer` do Nav2 só percorre células até `inflation_radius`; célula
mais longe que isso **nunca recebe custo**, mesmo estando dentro do
`robot_radius`. E o Smac, com footprint circular, decide colisão pelo custo da
célula do centro — ou seja, **o `robot_radius: 0.32` só é obedecido se
`inflation_radius ≥ 0,32`**. Com 0,20 o planejador acha que o robô é 36% menor
do que ele é, e planeja passagens onde o corpo não cabe. Quem segura é o
reflexo, e aí o robô trava — que é exatamente a descrição da 2ª tentativa de
13-08 (encravou, ré rodou 8 s para 1 cm, planner devolveu *"start é
obstáculo"*).

⚠️ Isto **não é erro da decisão 029**: 029 mediu que 0,20 desentortou o plano
(referência de rumo 83,8° → 23,4°, caminho 1,45× → 1,02× a reta), e aquilo está
certo. O que 029 não viu é que, junto, o corpo saiu do modelo do planejador.

**Candidato**: `inflation_radius 0,20 → 0,33` nos dois costmaps (logo acima do
`robot_radius` 0,32), `cost_scaling_factor` intocado em 3,0. Aí a ordem fica
`planejador 0,32 > reflexo 0,26` e a porta continua plannable — sobra um canal
de 0,90 − 2×0,32 = **0,26 m** para o centro. Não vai para o robô sem número: o
sweep é no simulador, mesmo cenário de 029, comparando `chegou / referência /
yaw / caminho-por-reta` em 0,20, 0,33 e 0,40.

## Alternativas consideradas

**(a) Fazer o Nav2 usar o polígono do reflexo como footprint.** Descartada: o
polígono do reflexo é uma zona de **frenagem** (0,57 à frente), não um corpo.
Como footprint ele proibiria a porta inteira e o planejador pararia de planejar.

**(b) Encolher o reflexo até caber no que o planejador permite hoje (0,20).**
É o remédio ao contrário: tiraria a única camada que impediu contato ontem, para
acomodar um planejador que está errado sobre o tamanho do robô.

**(c) Não mexer em nada e confiar na ré para desencalhar.** É o que estava no ar
em 13-08, e a ré recuou **1 cm em 8 s**. A saída de emergência não está provada;
não dá para projetar contando com ela.

## Referências

- decisão 020 — `cmd_vel` escolhe o RAIO, a velocidade quem escolhe é a placa
- decisão 027 — o reflexo enxerga o próprio robô
- decisão 029 — a inflação que entortava o plano
- `docs/dados/2026-08-13-robo-mapa-web/eventos-13-08-inf020.txt` — os disparos
