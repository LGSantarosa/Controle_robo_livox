# 041 — A quina de trás do reflexo come a porta

**Data**: 2026-08-14 (3ª leva, só análise offline — **o Gazebo não subiu e o
robô não foi ligado**)
**Status**: causa **medida e fechada**; conserto de **um parâmetro** no ar,
**ainda não verificado em corrida**.
**Toca**: `config/collision_monitor.yaml` (a caixa do `PolygonStop`),
`test/test_configs_coerentes.py` (invariante nova).
**Dados**: as 10 corridas do A/B da histerese,
`docs/dados/2026-08-14-porta-gazebo/ab_com_histerese` e `ab_sem_histerese` —
as mesmas de sempre, relidas com pergunta diferente.
**Corrige**: a atribuição de culpa da **decisão 040**, que estava errada.

---

## 1. O pedido que abriu esta decisão

> *"o door crossing nunca será implementado aqui. Pode fazer as medidas, mas
> quero essa porra seguindo o plan direito, só arrumar os parâmetros"* — o dono

Duas coisas, e a medida respondeu as duas de um jeito que nenhum dos dois
esperava.

## 2. O seguidor JÁ segue o plano — e a régua estava enviesada

A coluna `desvio_lateral` do CSV do seguidor, nas 10 corridas (todas **depois**
do conserto de frame da 040, máquina limpa, `k_lat = 0`):

```
                        |e_lat| p50      p90       máx
travaram (4)             0,015–0,026   0,064–0,081   0,109–0,132
passaram (6)             0,021–0,037   0,067–0,106   0,132–0,298
```

**Não há separação.** E o tamanho absoluto responde o pedido do dono: contra os
**0,110 m** que o seguidor comia em 13-08 (decisão 039), hoje ele come **2,5 cm
na mediana**. O conserto de frame da 040 já resolveu o seguimento do plano; não
sobrou defeito de seguimento para consertar por parâmetro.

⚠️ **E a primeira leitura desta mesma tabela foi um artefato meu**, que vale
registrar porque quase virou conclusão. Filtrando por `v_alvo != 0` (o pedido
do seguidor) a separação aparecia **perfeita e invertida**: travadas com p90
0,029–0,038 contra 0,066–0,105 das que passaram, sem nenhuma sobreposição. A
causa: numa travada o seguidor continua **pedindo** velocidade por ~70 s
enquanto o reflexo zera a saída depois dele, e o robô fica parado **em cima do
plano** — 1200 a 1400 amostras de `e_lat ≈ 0` que são o robô congelado, não o
robô seguindo bem. Trocado o filtro por deslocamento **medido** entre amostras,
a separação some.

➡️ **Filtro de "está andando" tem de olhar para o que o robô fez, não para o
que ele pediu.** É primo do erro da 023 (mediana de rumo "zero" que era o
trecho morto do CSV).

## 3. O plano entra torto — e o robô entra MENOS torto que ele

Vão medido no mapa `sala_andar3` (busca por células ocupadas, não por chute):

```
jamba de baixo  bloco isolado, topo em y = 1,031, x de 4,686 a 4,886
jamba de cima   parede horizontal, borda em y = 1,931
vão             0,900 m          centro (4,786 · 1,481)     eixo 0° (leste)
```

Onde cada um cruza o plano da porta (`x = 4,786`), desvio em relação ao centro:

```
                 PLANO              ROBÔ
             desvio   ângulo    desvio   rumo
             0,133    −18,4°    0,067   −20,5°
             0,120     −5,7°       —       —
             0,100    −11,3°    0,009    +4,8°
             0,050      0,0°    0,034    −6,8°
```

**O plano cruza mais fora do centro que o robô.** Puxar o seguidor para cima do
plano (`k_lat > 0`) o mandaria para os 13 cm e para os −18° do plano, e é um
mecanismo suficiente para o `k_lat = 1,0` ter medido pior na 040 — sem precisar
invocar a contaminação de CPU.

## 4. Quem trava é o `PolygonStop`, e a 040 acusou o polígono errado

`/collision_monitor_state`, integrando até o fim do bag (o intervalo final
ficava aberto e escondia isso na primeira leitura):

```
                       STOP      APPROACH
com_histerese 1  TRAVOU  63,0 s     0,1 s
com_histerese 3  TRAVOU  61,4 s     0,0 s
sem_histerese 1  TRAVOU  63,9 s     0,1 s
sem_histerese 2  TRAVOU  62,2 s     0,0 s
```

A 040 escreveu que *"o `PolygonApproach` é projetado pela velocidade, e a −33° a
projeção alcança a ombreira antes do corpo"*. **É falso**: o `PolygonApproach`
aparece 0,0 a 0,1 s. Quem segura o robô por um minuto é a **caixa estática**.

## 5. A causa, na pose exata da travada

Caixa e corpo desenhados na pose em que cada uma congelou:

```
pose da travada              folga do CORPO   folga da CAIXA
(4,792 · 1,561)  −26,9°         +0,069           +0,002
(4,547 · 1,565)  −27,1°         +0,065           −0,002
(4,872 · 1,585)  −24,6°         +0,049           −0,017
(4,878 · 1,571)  −23,1°         +0,066           +0,000
```

Quatro poses independentes, todas a menos de 2 cm de zero na caixa, todas com
**5 a 7 cm sobrando no corpo**. Nenhuma encostou em nada. E o canto que encosta
é sempre o mesmo: a **quina de TRÁS do lado de dentro da curva**, que sobra para
a jamba enquanto o nariz já entrou.

🔴 **A razão é aritmética, e é a armadilha de somar margem por FACE.** Somar
5 cm em cada face de um retângulo empurra a quina em 5·√2 ≈ **7,1 cm**:

```
caixa   traseira 0,2665 · lateral 0,2775  ->  quina 0,3847
corpo   traseira 0,2165 · lateral 0,2275  ->  quina 0,3141      +7,1 cm
```

A caixa anunciava margem de 5 cm e cobrava 7,1 cm justo onde o vão é escasso.
Num vão de 0,90 m com corpo de 0,455, o orçamento por lado é 22,3 cm, e a 25° o
corpo já gasta ~17 cm dele. Os 7,1 cm da quina não cabem no que sobra.

## 6. O conserto — margem medida na quina, 3 cm

```
por face 0,021 m  ->  na quina 0,021·√2 = 0,030 m
traseira  0,2165 + 0,021 = 0,2375        (era 0,2665)
lateral   0,2275 + 0,020 = 0,2475        (era 0,2775)
frente    0,35                            INALTERADA
```

Pior folga nas 4 poses de travada: **+0,022 m**, contra −0,017 de hoje.
Quina a 0,3466 = corpo + 3,3 cm. Os 3 cm são os mesmos do `PolygonApproach`,
o que deixa os dois polígonos com uma margem só, e não duas inventadas.

⚠️ **A frente não muda, e isso é medida e não deferência.** Varrida de 0,35 até
0,2165, a folga na jamba **não se move 1 mm** — o canto que encosta é o de trás.
O pedido do dono depois da batida (*"a frente e as quinas da frente"*) fica
inteiro de pé.

## 7. Alternativas descartadas

| alternativa | por que não |
|---|---|
| `door_crossing` (alinhar antes de cruzar) | **vetado pelo dono**; e o pivô desta máquina tem quantum de 91–131° por pulso (040), então "girar 25° parado" não existe aqui |
| subir `k_lat` para colar no plano | o plano cruza **mais torto** que o robô (§3): colaria no defeito |
| encolher a frente do `PolygonStop` | medido irrelevante (§6), e desfaz o pedido do dono |
| subir `time_before_collision` / mexer no `PolygonApproach` | ele nem participa das travadas (§4) |
| caixa = corpo nu (margem 0) | o reflexo deixaria de ver o que já está encostando no para-choque; o teste `test_a_caixa_estatica_cobre_ao_menos_o_CORPO` proíbe |

## 8. Invariante nova, verificada por mutação

`test_a_margem_da_caixa_estatica_e_medida_na_QUINA`: a quina de trás não pode
passar de 4 cm além da quina do corpo. Com os valores antigos o teste
**reprova** (conferido). 324 testes verdes em `robot_motion`.

## 9. O que isto NÃO resolve, e é para ler antes de comemorar

- **não foi rodado.** A régua é o protocolo de 5 corridas
  (`tools/banco/protocolo_porta.sh`), e a linha de base com a máquina limpa é
  **3/5**. Precisa do dono na tela;
- a previsão a bater, escrita **antes**: as travadas somem, porque o motivo
  delas era a caixa; o que sobrar de falha passa a ter outra causa;
- **o robô continua entrando torto** (−23° a −27°). Isto não conserta a
  geometria de entrada, só para de proibir uma passagem que cabe. Se o dono
  quiser entrada reta um dia, a medida da §3 diz que o alvo é o **plano**, não
  o seguidor;
- os vereditos de `k_lat` e mira adaptativa da 040 continuam medidos com CPU
  faminta — §3 dá um segundo motivo para o `k_lat` ter reprovado, mas não
  reabilita a medida.
