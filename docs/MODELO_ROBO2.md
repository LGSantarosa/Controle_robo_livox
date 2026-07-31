# Modelo medido do robô 2 — para o Gazebo

Tudo aqui foi **medido no robô real em 2026-07-31**, com o LIO como referência de
pose e os encoders da placa como segunda testemunha. Nada é herdado do robô 1 nem
estimado. Os CSV estão em `docs/dados/2026-07-31-*`; a narrativa completa, com os
erros do caminho, está no `DIARIO.md` (07-31, 3ª e 4ª levas).

> ⚠️ **Leia antes de usar:** este modelo descreve o robô **como está
> configurado**, com a compensação de zona morta do driver LIGADA. Ela não é
> opcional — sem ela o robô não sai do lugar na faixa útil. É esse sistema que o
> simulador tem de imitar, não o atuador cru.

## 1. O atuador não obedece a `cmd_vel` em magnitude

O `hoverboard_driver` escala qualquer comando pequeno até a roda de maior
magnitude alcançar 100 unidades de firmware:

```cpp
if (mx > 1.0 && mx < 100.0) { k = 100.0/mx; set_speed[0] *= k; set_speed[1] *= k; }
```

Como `k` cresce quando o comando diminui, **toda a faixa útil vira o mesmo
comando na placa**:

```
|cmd| < 0,008 m/s      -> nao move
0,008 a 0,838 m/s      -> mesma velocidade de saida (~0,2 m/s medidos)
> 0,838 m/s            -> comando passa proporcional   [NAO TESTADO]
```

O teto do robô (`max_velocity`) é 1,0 m/s, então o patamar cobre praticamente
tudo. Confirmado por medida: comandos de 0,10 e 0,25 m/s produziram a mesma
velocidade realizada.

**Latência para destravar: ~0,35 s** com a compensação ligada. Sem ela, `v=0,50`
ficou 0,9 s plantado antes de sair.

## 2. Zona morta

| | valor | como foi medido |
|---|---|---|
| linear, frente | **0,0178 m/s** | rampa a 0,0175 m/s², corta em 3 cm confirmados |
| linear, ré | **0,0148 m/s** | idem |
| giro | **0,095 rad/s** | faixa 0,084–0,105, duas corridas independentes |

A zona morta **crua** do atuador (compensação desligada) fica entre **0,25 e
0,50 m/s** — não foi fechada, e é irrelevante para a operação porque a
compensação está ligada.

## 3. O robô não anda reto (indo para a frente)

| | sob comando | ao frear |
|---|---|---|
| frente | roda esquerda **11–12% mais rápida** | esquerda demora **0,2–0,4 s a mais** para parar |
| ré | ~0% (0,7% e −0,8%) | param juntas |

Resultado no rumo, medido em rajadas de ~18 cm: **−9,2° e −6,9° indo para a
frente**, **+0,5° de ré**. Confirmado a olho pelo dono: *"a ré fica reta e a
frente ele pende para a direita"*.

Mais da metade do desvio acontece **depois** do corte do comando, porque as rodas
não param juntas.

**Causa não fechada.** A dependência do sentido aponta para algo mecânico — a
roda boba é a candidata do dono e não foi descartada; um motor intrinsecamente
mais fraco apareceria nos dois sentidos, e não aparece.

## 4. Rotação

Pivô no lugar, `wz = ±0,30` por 1,0 s:

| | `+0,30` | `−0,30` |
|---|---|---|
| giro sob comando | +60,2° | −48,9° |
| giro na inércia | +87,5° | −101,2° |
| **giro total** | **+147,7°** | **−150,0°** |
| esq vs dir | +15,9% | +11,4% |

- **Simétrico**: girar para um lado ou para o outro dá o mesmo. O simulador não
  precisa de assimetria de rotação.
- **60% do giro acontece depois do corte.** Inércia rotacional grande.
- **Quem para por último é sempre a roda que gira para trás.**
- O pivô **não é puro**: sai 7 a 14 cm do lugar.
- Desaceleração angular: **a_dec ≈ 3,05 rad/s²** (faixa 2,08–3,67, n=4).

## 5. Curva sustentada

`v = 0,10` + `wz = +0,30` por 1,0 s:

```
v realizada   0,094 m/s   (0,94x do comandado)
wz realizado  0,282 rad/s (0,94x)
raio do arco  0,333 m
```

**Em curva o robô obedece ao comando** (94% nos dois eixos) — diferente do pivô,
que entregou 2,8–3,5× o comandado. *Pergunta aberta:* os dois caem na faixa da
compensação e deveriam sofrer o mesmo `k`. Não explicado.

**Derrapagem:** o diferencial das rodas implicaria +52,3° de giro; o robô girou
+16,3°. ~70% do diferencial vira esfrega em vez de rotação.

## 6. O que este modelo NÃO cobre

- comportamento acima de 0,838 m/s (onde o comando volta a ser proporcional)
- `reta` com cutucão — se o rumo se recupera de uma perturbação ou diverge
- repetição a n=3: o protocolo pede três corridas por condição; temos 1 ou 2
- a causa do desvio (boba × placa)
- a inconsistência do item 5

## 7. Como conferir que o robô está em estado de medir

```bash
# EXATAMENTE um de cada — duplicata publicando em /Odometry produz saltos de ~1,35 m
ps -eo pid,cmd | grep -E "fastlio|livox_ros|ros2_control_node" | grep -v grep

# a base delata a propria configuracao ao subir:
grep "sinal da realimentacao" ~/base.log
# esperado: esq +1  dir -1 | zona morta: LIGADA (100)
```

Com a pilha limpa, o LIO parado deriva **1,8 mm em 15 s**. Se estiver derivando
centímetros, há órfão rodando.
