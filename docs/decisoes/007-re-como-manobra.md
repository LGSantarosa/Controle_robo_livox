# 007 — A ré como manobra: recuar para abrir a curva

**Data:** 2026-07-28
**Estado:** aceita
**Depende de:** [005](005-lei-de-frenagem-de-rumo.md), [006](006-navegacao-ponto-a-ponto.md)

---

## Contexto

A decisão 006 deixou um caso em aberto, e ele apareceu assim que a placa
fingida entrou no simulador: **ponto perto e de lado é inalcançável**. Medido:
alvo a 0,65 m, o robô estabiliza a 0,168 m dele e orbita para sempre, com raio
de chegada de 0,15 m.

A causa está medida e não é ajuste de ganho:

```
pivô exige:  wz_max · bitola/2  >=  zona_morta + margem
```

Sem pivô, o robô tem um **raio mínimo de curva**, e um ponto que exige menos
que ele fica *dentro* do círculo que o robô descreve. Perseguir um ponto por
dentro do próprio círculo não converge — o robô o contorna indefinidamente.

E isso não é só o simulador: com os parâmetros do perfil real
(`zona_morta` 0,15, `bitola` 0,32, `wz_max` 1,0), pivotar exigiria 1,25 rad/s
contra um teto de 1,0. **O pivô não existe em nenhum dos dois perfis hoje.**

A lei da 005 proibia a ré explicitamente (`max(0, cos e)`, com o teste
`test_nunca_anda_de_re`): "prefiro me orientar a andar na direção errada". O
dono levantou que a proibição custa caro justamente aqui — dando ré, o problema
morre, porque recuar aumenta `d` e o raio necessário `d/(2·sen e)` abre junto.
É a manobra de baliza.

## Decisão

### 1. O gatilho é geométrico, não reativo

Perseguir um ponto a `d` com erro de rumo `e` exige o raio

```
raio_necessario = d / (2 · sen e)
```

Se ele for menor que o raio mínimo do robô, o alvo está dentro do círculo:
**manobra**. A decisão é tomada *antes* de orbitar, pela geometria, e não por
um detector de "não estou chegando" — que só age depois de o defeito já estar
acontecendo.

Descartado: detectar a órbita pelo sintoma (distância que não cai por N
segundos). Funcionaria, mas gasta N segundos de órbita toda vez, e o número N
seria mais um parâmetro sem medida por trás.

### 2. A ré é RETA

Recuar é ganhar espaço, não navegar. Curvar de ré é a manobra sobre a qual não
existe medida nenhuma neste projeto: andando para trás a boba deixa de ser
arrastada e passa a ser empurrada, que é a configuração instável do carrinho de
supermercado — e o simulador **não pode** responder isso (ver BO-4: a boba dele
não chega a virar).

Reta também mantém a zona morta simétrica: as duas rodas na mesma velocidade,
longe da banda proibida, sem a assimetria que a curva introduz.

### 3. Ré é modo explícito, e a lei de rumo não muda

`linear_de_avanco` continua com `max(0, cos e)` e o `test_nunca_anda_de_re`
continua valendo. A ré vive em `comando_de_re()`, à parte, e é acionada por
**velocidade negativa** no tópico que já existe (`velocidade_alvo`) — sem
tópico novo e sem modo escondido. Para a movimentação, ré é só uma velocidade
com sinal trocado; quem sabe que aquilo é manobra é a navegação, que conhece a
geometria do alvo.

### 4. A ré anda no mínimo viável, não em "devagar"

`|v| = v_min_viavel`, e a movimentação ainda aplica seu próprio piso
(`zona_morta + margem`). Ré mais lenta que a zona morta é o BO-3 de novo, agora
no meio de uma manobra: robô parado em silêncio com o log limpo. `v_re_max` é
o teto **desejado** — quando o mínimo viável passa dele, o nó avisa em vez de
obedecer a um número que travaria a máquina.

### 5. Histerese na saída, orçamento no total

Sair da manobra exige folga de 1,3× no raio; entrar, não. Sem isso o robô
alterna ré e avanço a cada ciclo na fronteira e não sai do lugar.

O orçamento (1,0 m e 8 s) fecha a manobra que não converge: para, **grita com
os números** e declara o objetivo não alcançado. Recuar sem fim é pior que não
alcançar — o robô sai do lugar onde alguém foi procurá-lo, e o log fica limpo.

## Verificação (simulador, planta lenta, zona morta 0,10, bitola 0,20)

O caso que o dono achou clicando, alvo a 0,65 m de lado:

```
alvo a 0.33 m e 33° exige raio de 0.30 m, e o meu mínimo é 0.30 m — RECUANDO
RÉ a 0.15 m/s (manobra)
alvo cabe na curva agora (precisa de 0.39 m, tenho 0.30) — recuei 0.04 m
... (segunda mordida, 0.05 m)
chegou — 0.15 m do ponto
```

- **Antes**: órbita a 0,168 m, indefinidamente.
- **Depois**: chega e para, 0,150 m do ponto, imóvel por 30 s.
- Ré total: **9 cm em duas mordidas**, contra orçamento de 1,0 m.
- Regressão: alvo (2, 2) continua fechando em **8 mm**, sem acionar a ré.

Ele chega na **borda** do raio de chegada, não em cima do ponto — a ré destrava
o alcance, não a precisão. Com pivô disponível o mesmo caso fechava a 0,059 m.

## O que esta decisão NÃO resolve

- **Não foi validada no robô.** O simulador não pôde testar a única coisa que
  preocupa na ré (a boba virando roda dianteira), porque a boba dele não vira.
  Ver BO-4 e o ensaio de ré em `tools/banco/README.md`.
- **A ré é cega a obstáculo** — como a navegação inteira, que ainda não tem
  percepção (fatia B). O Livox enxerga 360°, então a ré não é um ponto cego
  novo: é o mesmo buraco que já existe para a frente.
- **Não substitui medir a bitola.** Com pivô, `raio_min_curva = 0` e a manobra
  desaparece sozinha, sem trocar código. A ré é a saída enquanto a geometria
  não permite virar no lugar.
