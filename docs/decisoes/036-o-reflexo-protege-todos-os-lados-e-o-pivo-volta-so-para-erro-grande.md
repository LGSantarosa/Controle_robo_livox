# 036 — O reflexo protege todos os lados; o pivô volta só para erro grande

**Data**: 2026-08-14 (2ª leva; dev + Gazebo, com o dono olhando)
**Status**: implementadas; a do reflexo **verificada na tela** (ele parou em vez
de bater), a do pivô verificada em um giro forçado
**Toca**: `config/collision_monitor.yaml`, `config/nav2.yaml`,
`config/movimentacao.yaml`, `config/movimentacao_sim.yaml`
**Revisa**: a decisão 033 (o reflexo de duas camadas) e a 023 (o pivô fora)

## Parte 1 — A QUINA era o buraco, e por ela ele bateu

Na 033 eu encolhi a caixa estática para frente 0,30 · lateral 0,26 e passei a
frenagem para a projeção. **Com o dono olhando, o robô bateu.**

A conta que explica, medida na pose em que ele parou:

```
meia-largura do corpo      0,2275 m
lateral protegida          0,2600 m   <- ok andando reto
MEIA-DIAGONAL do corpo     0,3140 m   <- a quina, NÃO protegida
```

Andando reto isso nunca aparece. **Girando, a quina sai da caixa.** E foi
exatamente girar perto de parede que eu destravei na 033 — tirei o congelamento
sem cobrir o que o giro expõe. Troquei segurança por mobilidade sem dizer o
preço, e o preço apareceu.

⚠️ Vale registrar como método: **eu tinha escrito essa dívida na própria 033** e
segui assim mesmo. Dívida anotada não é dívida coberta.

### A decisão

Pedido do dono: *"essa segurança é pra TODOS os lados, uns mais que os outros,
como a frente e as quinas da frente, mas todos os lados devem parar o robô
quando chega a uma distância mínima"*. A caixa passa a ser o **corpo medido +
margem, lado a lado**:

```
              corpo     antes     agora
frente        0,2165    0,30      0,35     (mais, como ele pediu)
lateral       0,2275    0,26      0,2775
traseira      0,2165    0,28      0,2665
quina frente  0,3140    0,388     0,447    <- o furo, agora coberto
```

E o `PolygonApproach` deixou de ser o corpo cru: virou **corpo + 3 cm**, pelo
mesmo motivo — a projeção também precisa cobrir a quina enquanto o robô curva.

O `robot_radius` do planejador subiu junto, **0,26 → 0,28**, senão volta o
impasse: reflexo exigindo mais folga do que o planejador garante = robô parado
entre os dois, que é o defeito que o dono previu de véspera e viu acontecer.

🟢 **Verificado na tela**: com a quina coberta ele **voltou a parar em vez de
bater**, no mesmo lugar onde tinha batido.

## Parte 2 — O pivô volta, e só para erro grande

Pedido do dono, vendo o robô fazer balão para seguir a linha: *"caso o ângulo da
linha comparado com o robô esteja a 80 graus de erro ele para e faz o pivô e
acerta, pro balão poder arrumar"*.

A decisão 023 tirou o pivô, e **o argumento dela continua válido**: a placa
entrega um módulo de giro só e a varredura pós-corte não depende do `wz` do
corte. O que muda é ONDE a manobra é usada. Ela não ficou controlável — continua
sendo um **quantum grosso** — mas para erro de 80°+ o quantum É a manobra certa:

```
erro < 80°   lei contínua (arco), que assenta em 0,5–0,6° (medido na 023)
erro > 80°   UM pulso de pivô, e devolve para o arco
```

```
movimentacao.yaml / movimentacao_sim.yaml
    limiar_pivo       3,20 -> 1,40 rad (80°)
    pivo_max_pulsos      6 -> 1
```

`pivo_max_pulsos: 1` é o que impede a volta do ciclo-limite de 12-08 (±50–60°,
período 5,5 s): com 6 pulsos a lei caça o alvo com um martelo. Um pulso é "vira
grosso e sai da frente".

🟢 **Medido**: erro de −149° fechado com **+3,4° de resíduo**, num golpe só. E na
rota boa ele **não disparou nenhuma vez** — o caso bom não mudou.

⚠️ O default do NÓ continua desligado (`limiar_pivo` 3,20). Quem liga é o
perfil, pela regra da 019: quem sobe o `heading_controller` sem perfil não pode
ganhar de brinde uma manobra que só é segura numa faixa de ângulo.

## Referências

- decisão 019 — o default é o caso seguro
- decisão 023 — o pivô sai do caminho (o argumento que esta preserva)
- decisão 033 — o reflexo de duas camadas (cuja lateral esta corrige)
