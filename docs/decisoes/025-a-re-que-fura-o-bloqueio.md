# 025 — A ré que fura o bloqueio, com olho no vão

**Data**: 2026-08-12 (5ª/6ª levas; dev, corridas no Gazebo)
**Status**: aceita, implementada e **medida na máquina**
**Toca**: `path_follower.py`, `lei_de_seguimento.py`, `config/twist_mux.yaml`
**Vem de**: a 024, que tornou a ré alcançável e mediu que ela era vetada
**Estratégia apontada pelo dono**: *"o recuo deve ser um furo do bloqueio, ele
vê se não tem nada atrás e aí dá a ré furando o bloqueio todo"*

## O problema

A 024 religou a ré e ela **não moveu o robô**: 831 amostras de ré pedida, **0**
passaram. O `PolygonStop` do `collision_monitor` é **cego para direção** —
polígono estático de −0,28 a +0,49 m com `action_type: stop`, e ponto lá dentro
zera *qualquer* comando. O mesmo reflexo que salvou o robô de bater na ombreira
é o que o impedia de se afastar dela.

## A decisão, em duas peças

### 1. O canal que fura

```
teclado    90     <- o humano vence tudo, inclusive o desencalhe
web        50
desencalhe 30     <- NOVO: `unstuck_vel`, entra no mux DEPOIS do reflexo
autonomia  10
```

30 está abaixo do humano **de propósito**: um desencalhe automático que o
operador não consegue interromper é pior que um robô parado — e este canal, por
definição, dirige o robô sem o freio de mão.

### 2. O olho no vão, que é o que torna o furo aceitável

`vao_no_corredor_traseiro` mede em **metros**, no **corredor retangular da
largura do robô**, sobre o `/scan` da decisão 021.

🔴 **A FORMA IMPORTA, E É POR ISSO QUE NÃO É SETOR ANGULAR.** Um cone traseiro
é cego para a quina: obstáculo encostado no canto aparece num feixe cujo ângulo
cai fora do cone, e a checagem diz "livre" enquanto o corpo já vai raspar. Um
cone só cobriria o corpo se fosse mais largo que o robô a toda distância, e
nenhum ângulo fixo faz isso. **O retângulo é a forma certa porque é a forma do
robô.** Há teste com o caso exato: feixe a 145°, fora de um cone de ±30°, mas
com `y = 0,34 m` dentro da meia-largura de 0,35.

Regras, e cada uma é uma linha de log:

```
sem /scan ou /scan velho   -> NÃO recua ("não recuo às cegas")
vão menor que a folga      -> NÃO recua ("parado, e é a coisa certa")
vão some no MEIO da ré     -> PARA na hora (remedido a cada ciclo)
```

⚠️ **`re_scan_velho_s` foi corrigido por medida**: começou em 0,5 s e caía em
cima do pior caso. 245 quadros medidos deram p50 0,103 · p90 0,207 · p99 0,317
· **máx 0,513 s**. Ficou 0,8 s, e o teto vem de uma conta travada em teste:
recuando a `v_piso`, uma janela vencida gasta 0,8 × 0,203 = **0,16 m às cegas**,
contra 0,30 m de folga que o orçamento já desconta. A cegueira cabe na margem.

## 🔴 E a primeira versão fabricou uma FUGA

Na corrida de aceitação o robô atravessou a porta — e cobrou 16,41 m de caminho
para 5,32 m de reta (tortuosidade 3,09), com **9 rés**:

```
ré 1  t=14,2s  dist 2,50 -> 2,81 m      <- legítima
ré 2  t=17,6s  dist 2,87 -> 3,18 m      <- já andando, e se AFASTANDO
...
ré 9  t=47,5s  dist 5,10 m              <- vão traseiro de 3,17 -> 0,31 m
```

Ele andou 2,6 m de costas em linha reta até acabar o espaço. Palavra do dono:
*"ele ativou dnv a ré mesmo estando reto na porta"*.

**Causa**: `re_parado_s` era 1,5 s e a própria ré dura 1,6–2,8 s (medido). O
relógio rearmava **antes de o robô ter tempo físico de aproveitar a manobra
anterior** — a placa ainda derrapa 0,52 s para trás depois do zero, ele precisa
virar, e só então começa a ganhar. Realimentação positiva.

**Dois consertos, e o segundo não depende de sintonia:**

- `re_parado_s` 1,5 → **4,0 s**, acima da manobra mais longa medida;
- `re_max_seguidas` = **2**: recuo que não faz o robô bater a distância que ele
  já tinha antes da ré **não é recuperação**, e repetir é andar de costas com
  cara de quem se desencalha. Zera assim que houver progresso de verdade.

## A medida

Mesmo alvo (6,0 · 1,5), atravessando a porta de 0,90 m:

```
                                     chegou     dist mín   rés
recuperação inalcançável (pré-024)     não       2,49 m      0
ré alcançável, vetada pelo reflexo     não       2,46 m      0
ré furando, sem teto                  65,8 s     0,07 m      9
ré com teto e relógio medido          29,9 s     0,08 m      1
```

Tortuosidade 3,09 → **1,30**. Replanejamentos 59 → 28. A única ré é exatamente a
que o dono aprovou olhando a tela.

## O que isto NÃO resolve

- **O robô ainda entra torto na porta** e precisa da ré para corrigir. Custo da
  023 (sem pivô ele arca para dentro de um vão de 0,90 m com corpo de 0,63 m).
  Leva própria, com o `folga.py` de régua — já decidido.
- **Nada disto foi ao robô.** O `/scan` real é a fatia 2D do Mid-360, e a taxa
  medida aqui é a do simulador. `re_scan_velho_s` tem de ser reconferido lá.
- **O objetivo abortado continua não voltando sozinho** (024). Aqui o robô se
  desencalha antes de o `bt_navigator` desistir, mas isso é sorte de timing,
  não garantia.
