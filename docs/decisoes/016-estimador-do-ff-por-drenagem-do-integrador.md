# 016 — O feedforward de rumo é APRENDIDO, drenando o integrador

**Data**: 2026-08-11 · **Estado**: implementado, opt-in, NÃO visto no robô
**Vem de**: decisão 013 (caminho 2), adendo de 10-08 · **Toca**:
`lei_de_reta.py`, `compensador_rumo.py`

## Contexto — o que a máquina provou em 10-08

A curvatura crua deste robô **deriva dentro da sessão**. Seis retas idênticas,
mesmo ponto, mesmo rumo, mesmo pedaço de chão:

```
+0,0 min 0,8395   +1,5 min 0,9358   +4,8 min 0,9313
+0,9 min 0,8154   +2,5 min 0,9175   +5,4 min 0,9687
             +0,022 1/m por minuto (r = 0,80) — +19% em 5,4 minutos
```

Os **13,5% entre dias** que motivaram a 013 acontecem em **cinco minutos**. O
caminho 3 (medir no começo da sessão) é piso, não solução.

E a saída óbvia — "deixa o integrador cobrir" — foi medida e **falhou**, apesar
de a conta dizer que caberia (`ki·int_max` = 0,072 rad/s de autoridade contra
0,038 rad/s necessários):

```
corrida de 2,5 m     excursão de rumo
ff VELHO −0,8275     −13,5° → +22,3°    envoltória CRESCE 1,65x
ff HOJE  −0,9383       0,0° → +13,0°    sobrecorrige, não assenta
```

**A razão é de escala de tempo.** O integrador é o único que enxerga o erro,
mas vive dentro de um laço com **0,94 s de tempo morto** (foi por isso que os
ganhos caíram 4,1× em 06-08) e é **zerado a cada parada** pelo `_descarta` —
regra que existe por um bom motivo: referência de rumo velha é pior que
nenhuma, o robô pode ter sido girado no chão enquanto esperava. O efeito é que
cada corrida recomeça do zero e gasta os 10 s reaprendendo o que a anterior já
sabia.

O dono cortou a investigação de causa (térmico × bateria) na hora certa:

> *"o compensador deve conseguir identificar o erro atual para ajeitar, se tem
> essa diferenciação aí, por isso um PID"*

## Decisão

**Separar as duas escalas de tempo.** O integrador continua com o rápido
(rad·s de rumo, descartado na parada). O que ele segura **em regime** é drenado
devagar para a `curv_*`, que é uma curvatura [1/m], é propriedade do robô e
**sobrevive à parada**:

```
transf = integral · dt / adapta_t        [rad·s]
Δcurv  = −(ki · transf) / v_real         [1/m]
integral −= transf
```

Três propriedades, e cada uma tem teste:

1. **sem solavanco** — no instante da transferência o `wz` de saída não muda: o
   ff cresce `ki·transf` e o termo integral encolhe `ki·transf`. Degrau de
   comando num laço com 0,94 s de tempo morto é como se fabrica a oscilação que
   este estimador veio matar;
2. **grampeado perto da semente** (±0,5 1/m) — ele corrige deriva de planta,
   não inventa robô novo. `/Odometry` travado ou referência ruim empurram o
   integrador para um lado só, e sem grampo a curvatura iria atrás e ficaria lá;
3. **não estima parado nem devagar** (`v_real ≥ 0,05 m/s`) — `Δcurv` divide por
   `v_real`.

`adapta_t = 8 s` é valor de PARTIDA, e a margem é curta de propósito para ser
testada: o laço assenta em ~9,6 s. Dois integradores em série com escalas
parecidas oscilam juntos. **Quem arbitra é o robô.**

## Alternativas consideradas, e por que não

**(a) Observador de perturbação lendo o `wz` medido.** Estimar
`curv_planta = (wz_medido − wz_comandado_atrasado) / v_real` direto do LIO.
É mais rápido e não depende do integrador. **Descartado por ora**: exige
alinhar o comando com a medida através dos 0,94 s de atraso, e esse atraso é
composto (0,27 s de liga + 0,52 s de desliga + pose a 10 Hz) e não é constante
— o de desliga *decai*. Erro de alinhamento vira viés de estimativa, e viés de
estimativa é exatamente o defeito que estamos consertando. Fica como caminho se
a drenagem for lenta demais no robô.

**(b) Subir `int_max` para dar mais autoridade ao integrador.** Já foi medido em
06-08 e **perde**: com tempo morto, `int_max` não é só teto de autoridade, é
proteção contra windup. Subir para 2,5 trouxe o sino de volta (9° de segunda
excursão).

**(c) Medir o ff mais vezes por sessão (caminho 3 mais frequente).** É o que
10-08 falsificou: a deriva é de minutos, e cada medida custa três corridas SEM
compensador — o robô arcando com raio de 1,2 m, que é o que precisa de espaço e
de vigilância. Não escala para operação.

**(d) Um D no PI.** O viés é constante em escala de segundos; derivar pose de
10 Hz contra um atuador com 0,94 s de atraso amplifica ruído (é o que a 011 já
tinha registrado ao escolher PI e não PID).

## Como isto se prova, e onde ele pode falhar

**Opt-in** (`-p adapta:=true`), como o preditor entrou, para que a condição de
controle da próxima bancada seja o comportamento de hoje.

⚠️ **A planta de brinquedo NÃO arbitra este projeto.** Ela reproduz a primeira
excursão de 10-08 quase exata (−13,6° contra −13,5°) e erra o sobrepasso por
3,7× (+6,0° contra +22,3°) — é otimista justamente onde este estimador precisa
ser julgado. Os testes provam o MECANISMO (converge, é lento, não dá solavanco,
grampeia, sobrevive à parada), não o resultado.

**Previsão falsificável para o robô** — três corridas de 2,5 m seguidas, com o
ff do dia deliberadamente velho e `adapta:=true`:

```
                corrida 1     corrida 2     corrida 3
sem estimador   repete        repete        repete       (o erro não melhora)
com estimador   igual à 1     MENOR         MENOR ainda  (e curv_hat se move)
```

Se a corrida 2 não for melhor que a 1, o mecanismo não está agindo — ler
`curv_hat` no `rosout`. Se `curv_hat` encostar no grampo, a semente está errada
ou a referência de rumo é ruim; **não é deriva**.

⚠️ **Régua de aceitação: 2,5 m (~10 s), nunca 1,2 m.** Em 10-08 a mesma corrida
mediu −0,0162 (passa no critério da 011) cortada em 1,2 m e +0,0882 (reprova)
medida inteira, porque o corte curto cai no cruzamento de zero do S.

## Referências

- `docs/decisoes/013-o-ff-fixo-nao-fecha-o-arco.md`, adendo de 10-08
- `docs/decisoes/011-malha-fechada-de-rumo-em-reta.md` — o critério de aceitação
- `docs/DIARIO.md`, 10-08 e 11-08
- `docs/dados/2026-08-10-curva-crua/` e `docs/dados/2026-08-10-ff-velho/`

---

## ⚖️ VEREDITO NO ROBÔ — 11-08 (tarde): NÃO PROVADO, e fica opt-in

Ensaio completo em `docs/dados/2026-08-11-estimador/` (13 corridas) e na entrada
11-08 (3ª leva) do `DIARIO`. Semente deliberadamente velha (−0,8275), planta
crua do dia −0,9145, todas as corridas de 2,5 m.

```
na ordem do tempo →   controle (adapta OFF)    8,9°   21,1°   18,4°
                      adapta ON               10,3°   10,1°    8,2°
                      VOLTA ao controle OFF    8,1°    6,2°
```

**A previsão falsificável desta decisão PASSOU e mesmo assim o veredito é
negativo.** 10,3 > 10,1 > 8,2 é a monotonicidade pedida. O que a derrubou foi um
**A-B-A** decidido na bancada: voltando à condição de controle, o robô deu 8,1°
e 6,2° — as melhores corridas do dia, **sem** estimador. A melhora era da ordem
temporal (planta assentando ao longo da tarde), não do mecanismo.

⚠️ **A previsão estava mal desenhada, e este é o aprendizado transferível**:
comparar uma sequência de corridas contra uma sequência anterior não protege
contra confundimento de ordem quando a planta deriva — e esta planta deriva 19%
em 5 minutos (10-08). **Ensaio de estimador exige retorno à condição de
controle.** Custou 2 corridas; teria custado uma sessão inteira de conclusão
errada.

✅ **O que sobrevive**: o mecanismo age no robô, como a implementação promete. O
`curv_hat` andou `−0,8275 → −0,6935` em três corridas, sem encostar no grampo, e
o nó anunciou tudo no `rosout`. A implementação não está em dúvida; a **utilidade**
está.

🔵 **Achado que a bancada não previu — o estimador anda para o lado errado.** A
planta crua do dia mediu −0,9145 (mais curvatura) e o `curv_hat` foi para −0,69
(menos). Duas leituras, nenhuma testada:

- **(a)** o ff efetivo em malha fechada não é a curvatura crua medida em malha
  aberta, e o estimador está achando o valor certo para o laço;
- **(b)** o sinal da drenagem está invertido, e o rumo foi segurado pelo termo
  proporcional apesar do estimador.

**Isto se resolve sem robô**, na planta de brinquedo: semear com erro dos dois
lados (semente maior e menor que a planta) e conferir se `curv_hat` caminha na
direção da planta nos dois casos. Se (b) for verdade, é um defeito de sinal e a
decisão volta a valer com o conserto.

**Estado da decisão**: implementação mantida, **default segue `adapta:=false`**.
Não vira padrão até (1) o sinal estar esclarecido e (2) um ensaio com A-B-A
mostrar ganho. A configuração adotada para operação é o compensador com o ff do
dia e o estimador desligado.
