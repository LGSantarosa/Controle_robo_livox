# Passo de rumo 0,40 — Gazebo `sala_andar3` — 2026-08-20

Única configuração aprovada no MAPA REAL. Veredito do dono:
**"agora foi bom, gostei dessa"**.

## O que mudou

```
mira_rumo_passo   0,20 -> 0,40      <- a mudança
mira_rumo_estica  3,0°  (intocado)
mira_rumo_encolhe 5,0°  (intocado)
mira_tol_estica   0,07  (intocado)
mira_tol_encolhe  0,08  (intocado)
```

Só a **régua** mudou, não os limiares. A porta e a curva de raio 2 m enxergam o
mesmo tanto nos dois passos; o que muda é o serrilhado da reta:

```
passo    PORTA (tem que encolher)   curva R=2 m   estica na reta real
0,20 m           71,6°                 21,4°           14,2%
0,40 m           71,6°                 18,5°           23,7%
```

## O caminho até aqui (três tentativas, duas reprovadas)

| tentativa | resultado |
|---|---|
| `tol_estica` 0,07 → 0,03 | **reprovou** — prendeu a mira no curto (80%), S igual |
| `rumo_estica` 3° → 6° | **reprovou na curva** — *"piorou demais a curva"* |
| `mira_rumo_passo` 0,20 → 0,40 | **aprovado** |

As duas primeiras trocavam curva por reta. A terceira não: ela filtra o
serrilhado antes de medir, sem tocar em nenhum limiar de proteção.

## O achado que sustenta isso

Medido sobre 219 amostras de `/plan_smoothed` reais:

```
desvio_da_corda <= 0,07 m      passa em 77,2%
mudanca_de_rumo <= 3,0 graus   passa em 14,2%   <- o gargalo
```

Uma curva de raio R muda o rumo ~0,8/R rad na janela de 1 m, então 3° só
aceita raio >= 15,3 m enquanto o `tol_estica` aceita raio >= ~2 m. O critério
de rumo anula o de desvio sempre. Alinhar os dois pediria ~23°, e isso é mexer
na proteção da porta — não foi feito.

## ⚠️ Não medido

A corrida foi aprovada a olho e **não tem repetição**: as duas tentativas de
repetir foram perdidas, uma por eu matar o `gz sim` da própria pilha, outra por
saturação de CPU. Não há número desta configuração comparado com o da anterior.

## ⚠️ CPU satura e isso ESTRAGA a corrida

Na última tentativa o robô ficou errático e a causa era carga, não código:

```
carga 15,75 em 12 nucleos
gz sim server 120% | mysqld (snap) 97% | path_follower 88%
ros2 bag record 68%  <- --all-topics, 560 MB/min gravando a nuvem
/scan caiu para 6,2 Hz (nas corridas boas: 7 a 10 Hz)
```

Rodar com `bag:=false` quando não precisar de `/plan` ou
`/collision_monitor_state`; o CSV do seguidor cobre o resto.

## Arquivos

- `corrida_01/seguidor_*.csv` e `corrida_01/parametros/*.yaml` (9 nós) — no git
- `corrida_01/corrida_*/` — bag, **local, fora do git**
