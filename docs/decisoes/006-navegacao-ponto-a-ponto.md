# 006 — Navegação ponto a ponto: mirar no alvo, frear pela distância

**Data:** 2026-07-27
**Estado:** aceita
**Depende de:** [005](005-lei-de-frenagem-de-rumo.md)

---

## Contexto

A meta declarada do projeto é o robô ir de um ponto a outro sem bater. A
decisão 005 entregou a metade "andar reto": um controlador que leva o bico ao
rumo pedido e o segura lá, com sobrepasso medido entre 0,1° e 0,8°.

Ele deixa dois buracos, os dois vistos em ensaio:

1. **Ele não volta para a rota.** Numa meia-volta, o robô fecha o rumo com
   erro de 0,01° e segue **66 cm paralelo** ao caminho de ida. Rumo é o que ele
   controla; distância lateral, não.
2. **Ele não sabe parar.** Não existe alvo, só direção — o robô anda para
   sempre.

Esta decisão fecha os dois, sem obstáculo: **fatia A**. Desviar de obstáculo é
a fatia B, depende do Livox e de percepção que o repo ainda não tem.

## Decisão

### 1. O rumo alvo é a direção até o ponto, recalculada todo ciclo

```
rumo_alvo = atan2(alvo_y - y, alvo_x - x)
```

É isso que dissolve o erro lateral **sem controlador extra**: se o robô sai da
linha, a direção até o alvo muda, e ele curva de volta sozinho. O erro lateral
deixa de existir como grandeza — não há linha a seguir, há um ponto para o qual
apontar.

Descartado: um controlador de *cross-track* separado (seguir a reta entre
origem e destino, com ganho sobre a distância lateral). Resolveria o mesmo
problema com mais um ganho para ajustar e mais um acoplamento entre malhas — e
a regra do projeto é não empilhar ganho sem necessidade física.

### 2. A aproximação é a lei de frenagem, aplicada a distância

```
v = √(2 · a_lin · dist)     limitada ao teto e ao piso
```

A mesma conta da decisão 005, com distância no lugar de erro angular: **nunca
chegar mais rápido do que se consegue frear no que falta**. Um só parâmetro
físico, `a_lin`, medível pelo ensaio `aceleracao_linear`. Mesma regra de ouro:
subestimar só faz o robô começar a frear antes.

### 3. A chegada tem piso, e o corte é firme

Aqui a zona morta (BO-3) impõe uma forma que não se escolheria por estética.

A desaceleração ideal manda velocidades cada vez menores conforme o ponto se
aproxima. Abaixo do mínimo viável, **a placa engole o comando**: o robô para
longe do ponto, sem erro nenhum no log, e o sistema acha que chegou. É a falha
silenciosa do BO-3 outra vez, agora disfarçada de sucesso.

Então: a velocidade de aproximação tem **piso** (`v_min_viavel`), e ao entrar
no raio de chegada o comando **vai a zero de uma vez**. Não existe "chegar
devagarinho" neste robô.

Consequência aritmética, que o nó verifica sozinho: parando a partir do piso,
o robô ainda percorre `v_min²/(2·a_lin)`. Um raio de chegada menor que isso
faria o robô **orbitar o ponto** — entrar, cortar, deslizar para fora, voltar,
para sempre. O nó recusa raio menor que esse mínimo e avisa no log.

### 4. Navegação e movimentação conversam pelo tópico público

O navegador publica em `rumo_alvo` e `velocidade_alvo` — os mesmos tópicos que
qualquer outro cliente usaria. Sem atalho por dentro.

A divisão de responsabilidade que isso força é a parte que interessa: **a
navegação não conhece atuador**. Ela pede "vá para lá a tal velocidade"; se
essa velocidade não sustenta a curva sem jogar a roda interna na zona morta, é
a movimentação que corta o giro (decisão 005). Navegação decide *para onde*;
movimentação decide *o que a máquina aguenta*.

Por isso a navegação também não trata o alvo atrás do robô como caso especial:
ela pede meia-volta e velocidade positiva, e o `cos(e)` da movimentação segura
o avanço enquanto o rumo estiver torto.

## Verificação

Os dois nós reais empilhados no simulador da planta degradada:

- **alvo à frente** (2, 2), a 2,83 m: chegou e parou a **7 mm** do ponto; 45 s
  parado depois, sem orbitar.
- **alvo atrás** (−1, 1), a 1,41 m e 135° do bico: chegou e parou a **8 mm**.

### Custo medido: alvo atrás sai por um laço

No segundo caso o robô **se afastou até 2,15 m** de um ponto que estava a
1,41 m, antes de voltar. É consequência direta de não pivotar: sem giro no
lugar, a única forma de virar é contornar.

Aceito por ora, porque a alternativa é justamente a manobra que a zona morta
proíbe (BO-3). O laço encolhe sozinho quando a zona morta real for medida — ela
dimensiona o piso de linear, e o piso é o que abre o arco. Se depois da
medição o laço continuar caro, aí sim vale discutir marcha à ré para alvos
atrás, com número na mão.

## O que esta decisão NÃO resolve

- **Obstáculos.** Fatia B. O robô vai em linha reta até o ponto e bate no que
  estiver no caminho.
- **Referencial.** Objetivo e pose têm que estar no mesmo `frame_id`; o nó
  recusa e reclama se forem diferentes, em vez de transformar. Quando houver
  mapa e TF completos, isso volta à mesa.
- **`a_lin` e `v_min_viavel` reais.** Não medidos — valores conservadores, e o
  nó avisa na subida. BO-3.
