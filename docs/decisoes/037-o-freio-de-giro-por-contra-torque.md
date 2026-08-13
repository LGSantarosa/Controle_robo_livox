# 037 — O freio de giro por contra-torque

**Data**: 2026-08-14 (3ª leva; dev + Gazebo, com o dono olhando)
**Status**: implementado e ligado (`freio_ligado: True`), com a lei **medida na
bancada**. 🔴 **Não atuou na única corrida de navegação em que foi exposto** —
ver "O que ele NÃO resolve", que é a parte importante deste registro.
**Toca**: `lei_de_freio.py` (novo), `heading_controller.py`,
`tools/banco/freio_de_giro.py` (novo)
**Vem de**: o dono, descrevendo a física antes de qualquer medida

## O problema, na descrição do dono

> *"ele chega no 90, mas chega rápido, aí solta o motor, mas a inércia joga ele
> mais 90 graus até parar de verdade... queria que isso fosse mais controlado"*

Está exatamente certo. **Esta máquina não tem freio**: zerar o comando não para
nada, porque a placa segura a saída cheia por `atraso_desliga` (0,52 s, medido
no robô em 04-08).

Medido em 14-08 no Gazebo, comando de 0,6 s e depois zero:

```
fase comandada     3,5°
SOBRA             60,3°     <- 94% do giro acontece com o comando em ZERO
total             63,8°
pico de wz        ~1,1 rad/s, e ele chega 0,65 s DEPOIS do corte
para em           ~1,5 s
```

🟢 **E isto reproduz a máquina real** (06-08, `liga 0,30`): fase comandada ~3°,
sobra 56–68°, total 59–70°, parada em 1,86–2,16 s. É essa fidelidade que
autoriza calibrar o freio no simulador.

## A ideia, e por que é a única disponível

A placa entrega **um módulo de giro só** (2,204 rad/s) e não sabe desacelerar —
pedir menos não dá menos (decisão 020: *o comando escolhe o RAIO, a placa
escolhe o módulo*). A única forma de tirar energia é **torque contrário**, e ela
sabe fazer isso porque o módulo vale nos dois sentidos.

## O número que manda: QUANDO SOLTAR

A retenção de 0,52 s vale para o contra-comando **também**. Soltar com o robô já
parado deixa meio segundo de torque reverso sobrando e ele gira para o outro
lado. Por isso o freio solta **cedo**, com o robô ainda girando no sentido de
origem:

```
solta em    giro total   veredito
 1,10 rad/s    27,3°     ok
 1,00          27,7°     ok
 0,90          13,8°     o melhor visto
 0,80          27,8°     ok
 0,60         −50,7°     INVERTEU
 0,40         −60,0°     INVERTEU
```

**Sem freio 63,8°; com freio soltando entre 0,9 e 1,1, de 14 a 28°.** Corta o
giro de 2,3 a 4,6 vezes e nunca inverte nessa faixa.

⚠️ **O espalho é real**: duas corridas no mesmo 0,90 deram 13,8° e 27,0°. Mesma
assinatura bimodal do robô real em 06-08 com pulso curto (2,2° contra 29,7°),
onde a causa foi a quantização do laço de 10 Hz. O número honesto é *"solta
entre 0,9 e 1,1 e o giro cai para um terço"*, não um ótimo fino.

## Onde a lei entra, e por quê

`lei_de_freio.FreioDeGiro`, chamado na **última linha antes de publicar** no
`heading_controller`. A posição é o desenho: ele não decide para onde virar nem
quando parar de girar — isso é da lei de rumo, que já rodou. Ele só responde
*"a lei parou de pedir giro e o robô ainda gira: o que mando agora?"*.

`v` **não é tocado**: o freio é de giro. Misturar os dois ali seria comandar
arco no lugar de frenagem.

Três travas, cada uma vinda de um erro que aconteceu na bancada:

- **espera o pico** antes de poder soltar — o `wz` sobe depois do corte, e um
  freio que solta na subida não freia nada (`freou por 0,00 s` em toda linha da
  1ª varredura);
- **testa o componente no sentido de origem**, nunca o módulo — depois de
  inverter, o módulo volta a subir e o contra-torque fica preso até o teto
  (−300° de giro numa varredura);
- **teto de tempo** — sensor travado não pode deixar o freio ligado para sempre,
  senão ele vira acelerador para o outro lado.

## 🔴 O que ele NÃO resolve, e isto é o mais importante daqui

**O freio é de GIRO. A colisão que o dono viu depois de ele entrar no ar foi
LINEAR** — o robô ganhou velocidade e entrou na parede ao lado do vão. Na
corrida em que ele foi exposto, o contador `FREIO:` marcou **zero atuações**.

E o dono tinha descrito o caso linear ANTES, com estas palavras: *"se ele ganha
velocidade indo reto e tenta fazer um balão pra passar pela porta, se ele não
diminuir essa velocidade ele vai de cara na porta"*. Eu medi e implementei o
outro eixo.

A física é a mesma nos dois: a placa segura a saída 0,52 s depois do corte, então
zerar não freia nem giro nem linha. **O freio linear não existe**, e é o mesmo
mecanismo — contra-torque, mesma bancada, mesma forma de medir.

⚠️ Registrar também o que isso ilumina: boa parte do que foi mexido em 14-08 no
polígono do reflexo era **tentativa de compensar com geometria a falta de
freio**. Geometria não para inércia.

## Referências

- decisão 020 — o comando escolhe o RAIO, a placa escolhe o módulo
- decisão 023 — a retenção de 0,52 s e a varredura pós-corte
- dados: `docs/dados/2026-08-14-freio-de-giro/` e `docs/dados/2026-08-06-pivo/`
