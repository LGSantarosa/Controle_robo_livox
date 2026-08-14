# 042 — O seguidor nunca viu o plano suavizado

**Data**: 2026-08-14 (4ª leva, dev + Gazebo com o dono acompanhando)
**Status**: defeito de **fiação** medido e corrigido; verificação pelo
protocolo de 5 corridas **em andamento** quando este registro foi escrito.
**Toca**: `path_follower.py` (assinatura e `cb_plano`),
`test/test_plano_suavizado.py` (novo).
**Vem de**: o pedido do dono — *"quero essa porra seguindo o plan direito, só
arrumar os parâmetros"* — e de **três hipóteses minhas mortas pela medida**
antes de chegar aqui. Ver §5, que é a parte com valor de método.

---

## 1. O defeito

```
path_follower.py   create_subscription(Path, '/plan', ...)      <- Theta* CRU
behavior_trees/…   SmoothPath  ->  {path}  ->  FollowPath        <- suavizado
smoother_server    publica  /plan_smoothed                       <- suavizado
```

A decisão **026** escreveu uma árvore de comportamento própria só para
suavizar, com a justificativa certa: *"o plano do Theta* ia cru para o
seguidor, com quinas de 23° a 46 cm da porta"*. A árvore foi escrita, roda, e
escreve o plano suavizado de volta em `{path}`.

**Só que quem dirige este robô não é o `FollowPath` do Nav2 — é o
`path_follower`, e ele lia o tópico do PLANEJADOR.** Desde a 026 o suavizador
trabalha e o resultado vai para o lixo, sem uma linha de log.

## 2. O tamanho do defeito, medido

Curvatura exigida pelo plano na aproximação da porta (x de 3,6 a 4,9), nas
5 corridas do protocolo de `docs/dados/2026-08-14-porta-041`:

```
                     pico [1/m]        raio mínimo exigido
/plan                3,64 a 4,65       0,215 a 0,275 m      NÃO CABE
/plan_smoothed       2,10 a 2,49       0,402 a 0,477 m      cabe

a máquina fecha 0,37 m  (`raio_min_curva`, medido em 29-07)
```

**5 corridas em 5, sem exceção**: o plano cru pede curva mais fechada do que o
robô sabe fazer, e o suavizado cabe. E o lugar onde ele pede é a boca de um vão
de 0,85 m.

➡️ Ele **chega ainda girando**. O resíduo de rumo custa caro: a 25° o corpo
gasta ~17 cm dos 22,3 cm de orçamento por lado, e o que sobra é o que separa
passar de travar.

## 3. A geometria do vão, que eu também tinha errado

Não é uma porta numa parede. É um **bloco que sobe pelo meio de um corredor**:

```
corredor antes e depois   1,30 m de largura, centro em y = 1,256
a garganta                0,85 m,            centro em y = 1,481
profundidade da garganta  0,20 m (x de 4,736 a 4,886)
```

O centro do vão está **22,5 cm acima** do centro do corredor por onde ele vem.
O plano tem de subir ~0,68 m em ~0,70 m de avanço, e é essa subida que exige a
curva impossível.

## 4. O critério, medido nas 15 corridas do dia

```
passou LIMPO (sem ré, 4 de 15)   folga do corpo na garganta  0,104 a 0,154 m
todo o resto        (11 de 15)                               0,037 a 0,113 m
```

E a folga já está decidida em **x = 4,55**, antes da garganta:

```
folga do corpo        x=4,40   x=4,55   x=4,70
limpo   (n=4)          0,048    0,162    0,167
com ré  (n=5)          0,007    0,058    0,139
travou  (n=6)         −0,051    0,064    0,103
```

🔴 **A ré é o que desencalha**: das 15, a ré disparou em 5 e as 5 passaram; nas
6 que travaram ela não disparou nenhuma vez. O log diz por quê —
*"emperrado e sem vão para recuar, atrás há 0.00 m"*: com o robô a 30°
atravessado, o corredor retangular da ré pega a ombreira. Ele fica preso entre
as duas regras de segurança dele. **Por decisão do dono a ré NÃO é mexida** —
o alvo é ele não precisar dela.

## 5. As três hipóteses minhas que a medida matou, e por quê importam

| hipótese | como morreu |
|---|---|
| **a caixa do reflexo** (041) | encolhida e rodada: **3/5, igual à base**. Nas travadas novas a caixa tinha +0,009 e +0,020 m de folga e o `PolygonStop` travou assim mesmo. Era condição, não causa |
| **a mira adaptativa esticada** | reconstruída offline com os parâmetros de produção: **42% do tempo esticada nos dois lados**. E o portão do vão frontal DISPARA — o corredor cai para 0,32–0,47 m, abaixo dos 0,60 |
| **a inflação do costmap** | reproduzida offline (inflação e custo do Theta*): de `0,90 · 3,0` até `1,50 · 0,7` o caminho **não se move** — a subida começa em x 4,45 ou 4,55 em todos. O que prende o caminho embaixo é a zona PROIBIDA (`robot_radius`), não o gradiente |

⚠️ **E duas vezes eu quase concluí de um artefato de amostragem.** Filtrando por
`v_alvo != 0` (o pedido do seguidor), as travadas apareciam seguindo o plano
MELHOR que as passagens, com separação perfeita e invertida — porque numa
travada o seguidor continua pedindo velocidade por ~70 s com o reflexo zerando
depois dele, e o robô fica parado **em cima do plano**. Com filtro por
deslocamento MEDIDO, a separação some.

➡️ **Filtro de "está andando" tem de olhar o que o robô fez, não o que ele
pediu.** É primo do erro da 023 (mediana de rumo "zero" que era o trecho morto
do CSV). Aconteceu duas vezes no mesmo dia, com dois indicadores diferentes.

## 6. O conserto

`topico_plano` (default `/plan_smoothed`) com **queda para `/plan`** quando o
suavizado fica mais velho que `timeout_plano` (2,0 s).

A queda não é enfeite: `SmoothPath` pode recusar (aconteceu 1 vez em 79 na
corrida 4), e a árvore trata isso seguindo com o cru. Sem a queda, um
suavizador que para deixaria o robô parado com objetivo aceito e plano
desenhado — o sintoma de 13-08 que não acusa ninguém no log. Quando ela entra,
sai `warn`, porque assumir o plano cru é degradação.

E quando **nunca** houve suavizado (bancada, pilha sem `smoother_server`), o
cru vale de primeira: não há o que preferir.

## 7. Alternativas descartadas

| alternativa | por que não |
|---|---|
| assinar `/received_global_plan` | é o `{path}` da árvore e teria a queda de graça, mas no bag ele **não tem amostra** na janela da porta — o `controller_server` não dirige nesta pilha |
| trocar `/plan` por `/plan_smoothed` sem queda | um `SmoothPath` que recusa vira robô parado sem culpado |
| mexer na inflação | medido inerte (§5) |
| mexer na ré | **vetado pelo dono**: o alvo é não precisar dela |
| planejador cinemático (Smac Hybrid) | resolveria na raiz, mas está aposentado desde 05-08 e é troca de planejador, não parâmetro |

## 8. Testes

`test_plano_suavizado.py`, 5 testes, os quatro de lógica verificados por
mutação (voltar o default para `/plan` reprova um; tirar a queda reprova
outro). **505 verdes** no repositório.

## 9. A verificação — 4/5 e **zero ré**

Protocolo de 5 corridas, máquina limpa, `docs/dados/2026-08-14-porta-042`.

```
             passou   folga na garganta   corridas com ré
cx041 (antes)  3/5      0,037 a 0,063          3 de 5
cx042 (agora)  4/5      0,109 a 0,158          0 de 5
```

Três vezes mais folga, e **nenhuma corrida precisou de ré** — que era o alvo
que o dono fixou (*"fazer ele sequer precisar dela pra passar"*). Em três das
cinco o reflexo **não agiu uma única vez**; em 20 corridas de porta, isso nunca
tinha acontecido.

A régua, juntando as 20 corridas do dia, separa sem sobreposição:

```
passou LIMPO (sem ré)  n=8    folga +0,122 a +0,158
passou com ré          n=5          +0,047 a +0,105
travou                 n=7          +0,037 a +0,109
```

⚠️ **A previsão acertou o número e ERROU o mecanismo, e isso vale registrar.**
Estava escrito que ele *chegaria mais reto*. Não chega: o yaw na garganta é
+27° a +39°, o mesmo de antes. O ganho é de **posição**, não de ângulo — com
uma curva que a máquina fecha, ele deixa de ser jogado contra a ombreira. A
previsão numérica bateu (0,15 previsto, 0,144–0,158 medido) por um caminho
diferente do que eu descrevi.

⚠️ **A que travou teve folga 0,109** — a maior de qualquer falha do dia,
encostada no limiar de 0,11. Não é mais o regime das travadas de antes
(0,037–0,066), e a causa dela ainda não foi investigada.
