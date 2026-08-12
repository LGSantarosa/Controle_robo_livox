# 027 — O reflexo enxerga o próprio robô, e a origem não era o único ponto

**Data**: 2026-08-12 (robô ligado para medir, desligado para consertar)
**Status**: aceita e implementada · ⏳ **previsão ainda não testada no robô**
**Toca**: `robot_base/nuvem.py`, `robot_base/nuvem_pontos.py`
**Vem de**: a primeira navegação autônoma no robô real (diário 12-08, 8ª leva)

## O problema, medido

Na primeira corrida do Nav2 no chão real o robô chegou ao alvo, mas fez um S de
rumo que o dono viu a olho (63° de amplitude para 0,17 m de desvio lateral). O
CSV mostrou quem estava cortando o comando:

```
durante a corrida   o collision_monitor vetou 26 dos 156 comandos (17%),
                    em 11 rajadas de ~0,1 s cada
com o robô PARADO   928 "Robot to stop due to PolygonStop polygon" no log,
                    o último 0,4 s antes de eu ir medir
```

Cada veto zera o comando dentro de uma malha de rumo que tem **0,94 s de tempo
morto** (medido em 06-08, por duas rotas independentes). Isso não é um freio
agindo: é perturbação periódica injetada no laço.

Contando os pontos da nuvem que caíam dentro do polígono de parada, com o robô
imóvel no meio da sala:

```
0 a 2 pontos por quadro, sempre no MESMO lugar (frame do sensor):
    x ≈ +0,04 m   y ≈ +0,09 m   z ≈ 0,00 a 0,05 m
    raio horizontal 0,10 m            min_points do reflexo: 2
```

10 cm ao lado do Mid-360 e na altura dele. **Era peça do próprio robô** — havia
uma estrutura montada em volta do sensor. Como a contagem oscilava entre 1 e 2,
o reflexo ligava e desligava sozinho o tempo todo.

🔧 **O dono arrancou a peça** ao ver a medida. Isso **não fecha o assunto**, e é
por isso que esta decisão existe: o defeito de software continua lá, esperando o
próximo suporte, cabo ou parafuso montado perto do lidar.

## Por que isto é reincidência, e não novidade

A **decisão 017** já tinha pego este mesmo mecanismo:

> O ponto `(0,0,0)` do Mid-360 (raio que não volta) é descartado: no frame do
> sensor ele é o próprio robô, e marcaria célula letal em cima dele a cada
> quadro.

Tratou-se **um ponto**. A generalização — "o sensor enxerga o corpo em que está
montado" — não foi feita, e o vizinho a 10 cm passou reto por doze dias.

## As alternativas, e por que as outras foram descartadas

| caminho | por que não |
|---|---|
| **subir `min_points` de 2 para 6** | esconde o sintoma e **cega o reflexo para obstáculo pequeno de verdade** — perna de cadeira, pé de pessoa. Troca segurança por silêncio, e o número que passaria a valer não teria origem em medida nenhuma |
| **encolher o polígono de parada** | o polígono descreve o CORPO do robô; encolher para caber o defeito faria o reflexo parar tarde demais na frente, que é onde ele tem de funcionar |
| **confiar na peça arrancada** | conserta este robô hoje e não conserta o próximo parafuso. E deixa o software com uma regra que a própria 017 já mostrou ser incompleta |
| **filtrar no `collision_monitor`** | resolveria só para o reflexo. Os dois costmaps consomem a MESMA nuvem e marcariam célula letal em cima do robô do mesmo jeito — o planejador recusaria caminho por estar "dentro" de obstáculo |

## O que entrou

`ponto_valido(x, y, z, raio_cego)` passa a descartar, além da origem exata, todo
ponto a menos de `raio_cego` do **eixo** do sensor. O nó ganha o parâmetro
`raio_cego`, default **0,15 m**, e anuncia o valor no `rosout` na subida.

```
0,15 m  ->  o corpo do robô sai da nuvem, para TODO consumidor
0,00 m  ->  comportamento de antes de 12-08 (existe para medir, ver abaixo)
```

⚠️ **CILINDRO, não esfera**, e isso está travado em teste. O corpo do robô fica
**abaixo** do lidar: o que cega é estrutura ao longo de todo o z, não uma
vizinhança da origem. Esfera deixaria passar exatamente a coluna que atrapalha.

⚠️ **O custo é real e tem tamanho**: obstáculo de verdade a menos de 0,15 m do
eixo do sensor fica invisível. Isso cai inteiramente dentro do chassi — o
polígono de parada vai de −0,28 a +0,49 m em x e ±0,26 m em y —, então para
haver algo ali ele já teria de estar encostado no robô. Há teste travando que a
0,20 m o ponto **passa**.

⚠️ **A regra da 017 não foi substituída**: `(0,0,0)` sai sempre, inclusive com
`raio_cego = 0,0`. Também travado em teste.

## Previsão falsificável — e a peça arrancada a deixou de graça

O dono removeu a estrutura **depois** da medida e **antes** do conserto. Isso
separa hardware de software, e a ordem do teste importa:

1. **Primeiro, com `raio_cego:=0.0`** (o comportamento velho): com o robô
   parado, os pontos dentro do polígono têm de dar **0** e o `PolygonStop` tem
   de **calar**. Se calar, a causa está confirmada **por intervenção** — que é
   evidência melhor que a minha contagem, e é o que vai para o artigo.
   ⚠️ Se ele **continuar disparando**, a peça não era a culpada e a hipótese
   está errada: nesse caso o suspeito seguinte é a faixa de altura
   (`min_height: 0.10` no frame `base_link`) pegando o chão, e o filtro por
   raio não teria resolvido nada.
2. **Depois, no default 0,15 m**: a contagem tem de continuar em 0, e a corrida
   do passo 5 tem de repetir a chegada **sem os 17% de vetos**.
3. **A previsão sobre o S**: se o S de rumo encolher junto com os vetos, o
   reflexo era o excitador. **Se o S continuar igual**, ele é do laço de rumo
   (a família de 06-08, com envoltória crescendo) e o reflexo era só um
   agravante — e aí o caminho é a sintonia, não a percepção.

⚠️ **Não misture os dois passos numa corrida só.** Foi exatamente esse o erro
de 11-08 com o estimador do ff: duas mudanças no mesmo deploy deixaram a
previsão que as separava sem teste possível.

**75 testes verdes no `robot_base`** (eram 70). Cinco novos, verificados por
mutação em duas direções: trocar o cilindro por esfera derruba um teste;
desligar o filtro derruba dois.
