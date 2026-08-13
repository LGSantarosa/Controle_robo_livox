# 033 — O reflexo ganha direção: a caixa reta proibia a curva do próprio robô

**Data**: 2026-08-14 (dev, robô carregando; medido no Gazebo)
**Status**: aceita e implementada, verificada em 3 corridas + 1 regressão
**Toca**: `config/collision_monitor.yaml`, `test_configs_coerentes.py`
**Revisa**: a decisão 030 (o bico), **13 horas depois de ela ser escrita**
**Vem de**: o dono, vendo o robô travar no mesmo lugar: *"travou EXATAMENTE no
mesmo lugar de antes, ficou dando ré e indo de novo no mesmo lugar"*

## O defeito, com número

O reflexo era **um polígono estático só**, uma caixa de 0,57 m à frente. Ele
pergunta *"tem coisa na caixa à minha frente?"* e **não olha para onde o robô
vai**. E como a decisão 023 tirou o pivô, **toda curva deste robô é arco**: o
nariz varre para o lado enquanto o corpo contorna.

Medido no Gazebo, alvo (6,24 · 3,51), separando as amostras em que o reflexo
cortou o seguidor das que ele deixou passar:

```
                       n     |wz| mediano   folga mediana
amostras CORTADAS     549      0,51 rad/s      0,45 m
amostras que passaram 853      0,25 rad/s      0,55 m
```

**Ele vetava justamente quando o robô girava** — o dobro de `wz`, com folga
parecida. E o seguidor não tinha culpa: na mesma corrida o erro de trajeto dele
foi **p50 de 8 mm e p90 de 6 cm**, ou seja, estava em cima do plano. Quem
proibia a manobra de contorno era o reflexo.

⚠️ **Isto revisa a decisão 030, escrita hoje de manhã.** Lá eu estiquei o nariz
de 0,49 para 0,57 para comprar margem de frenagem, e a conta daquela margem
continua certa. O que estava errado era o modelo: numa caixa reta, margem de
frenagem e proibição de curva são a **mesma dimensão**, então comprar uma
sempre custa a outra. Não dava para consertar dentro daquele desenho.

## A decisão: duas camadas, com papéis separados

```
PolygonApproach   projeta o CORPO pela velocidade comandada (v, wz) e portanto
                  ACOMPANHA O ARCO — parede da qual o robô está virando para
                  longe deixa de ser motivo de parada.  action_type: approach
PolygonStop       caixa estática pequena (frente 0,30), a trava de "vou
                  encostar agora", válida com comando nulo ou estranho
```

O bico de 0,57 saiu e a frente estática voltou para 0,30. **Isto não é
afrouxar** — é a frenagem mudando de dono: quem cobre coast + freio agora é a
projeção, que sabe para onde o robô vai.

### O `time_before_collision`, e por que ele é calibração e não conta

A objeção histórica ao `approach` (registrada no próprio arquivo desde 05-08)
continua verdadeira: ele projeta pela velocidade **comandada**, e neste robô o
comando não diz a velocidade — pede 0,50, anda 0,298, porque *o comando escolhe
o RAIO e a placa escolhe o módulo* (decisão 020).

Duas escolhas foram testadas hoje, nesta ordem:

```
1,80 s   escolhido pelo PIOR comando (v_piso 0,203 -> 0,353 m de projeção).
         REPROVOU: no comando típico (0,47 m/s) a projeção vale 0,85 m, 2,4x a
         distância de parada, e o robô ficou 30 s parado numa quina que ele
         atravessaria. Projeção longa demais tem o MESMO defeito da caixa de
         0,57 — proíbe a curva.
0,75 s   calibrado pelo comando TÍPICO: 0,47 x 0,75 = 0,35 m ~= parada medida
         mais margem (0,353 m).  APROVOU: 3 de 3 corridas chegaram.
```

🔴 **Dívida aberta, e ela é real**: abaixo de ~0,47 de comando a projeção fica
curta, porque a placa entrega 0,298 m/s independentemente do que se pede. O
buraco é coberto hoje só pelo `PolygonStop` estático, que dá 8,3 cm — pouco. O
conserto certo é **normalizar o comando para o patamar real antes do reflexo**,
preservando a razão `v/wz` (que é o que a placa obedece de fato): aí a projeção
passa a ser constante em metros e o pior caso desaparece. Não foi feito hoje por
ser nó novo na cadeia que dirige, e isso não se faz com o dono fora.

## A medida

Alvo (6,24 · 3,51), três corridas do zero, mais uma regressão no alvo curto:

```
                            chegou   tempo    cam/reta   reflexo   folga min
antes (caixa 0,57)            não      —        1,60x      18%       0,35 m
com approach 1,8 s            não      —        1,31x      35%       0,30 m
com approach 0,75 s  #1       SIM     40,4 s    1,41x      12%       0,35 m
                     #2       SIM     40,0 s    1,40x      12%       0,35 m
                     #3       SIM     39,7 s    1,38x      12%       0,35 m
regressão alvo curto          SIM      7,0 s    1,00x       0%       0,70 m
```

**Folga mínima 0,35 m em todas**, contra 0,2275 de meia-largura do corpo: ele
não encostou em nada, que é o critério que manda desde 13-08.

## Alternativas consideradas

**(a) Encolher só o nariz da caixa estática.** Não resolve: o que veta a curva
não é o nariz, é a caixa inteira ser reta enquanto o robô descreve arco.

**(b) `action_type: limit` (desacelerar perto).** Impossível nesta máquina, e
está no arquivo desde sempre: a placa não tem velocidade entre 0 e 0,298.

**(c) Publicar polígono dinâmico calculado por nó nosso** (o
`polygon_subscribe_topic` do Nav2). É a saída mais poderosa e resolveria junto a
dívida do comando desonesto. Descartada por hoje: nó novo na cadeia que dirige,
com o dono fora.

## Referências

- decisão 020 — o comando escolhe o RAIO, a placa escolhe o módulo
- decisão 023 — o pivô sai do caminho (é por isso que toda curva é arco)
- decisão 030 — o bico, que esta revisa
- dados: `docs/dados/2026-08-14-sim-porta/`
