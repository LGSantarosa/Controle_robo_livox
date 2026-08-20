# Mira devolvida aos valores da 042 — Gazebo — 2026-08-20

Melhor corrida do dia. Veredito do dono: **"ele passou direto das duas portas
na ida. na volta ele tmb passou tmb, foi absurdamente bem, não parou em
momento nenhum"**.

## Condição

- commit `51b7023`, mundo/mapa `pista_obstaculos`, spawn `(2,00 · 5,00)`
- ida `(10,65 · 6,60)`, volta `(2,00 · 5,00)`
- **a mudança sob teste**: mira devolvida aos valores medidos no robô (042)

      mira_tol_estica     0,15 -> 0,07        mira_rumo_estica    15° -> 3°
      mira_tol_encolhe    0,20 -> 0,08        mira_rumo_encolhe   20° -> 5°

- resto igual à corrida anterior: gargalo desligado, `passagem_v_max` neutro,
  `a_dec: 0,3`, smoother `simples`, pivô e recuperação ligados

## Medido

Ida e volta em **111,5 s**, **zero paradas** (nenhuma pose congelada > 1,5 s),
**1 amostra de invasão da caixa** na corrida inteira.

| travessia | yaw na soleira | folga da CAIXA | folga do CORPO | invasões |
|---|---|---|---|---|
| ida, porta 1   | +0,8° / −10,6°  | +0,167 / +0,009 | +0,243 / +0,068 | 0 |
| ida, porta 2   | −4,0° / +12,8°  | +0,000 / +0,067 | +0,052 / +0,146 | 0 |
| volta, porta 2 | ≈8° do eixo     | +0,116 / +0,027 | +0,166 / +0,101 | 0 |
| volta, porta 1 | ≈5° do eixo     | +0,080 / −0,001 | +0,152 / +0,052 | 1 |

Desvio lateral: mediana **6 cm**, p90 13 cm, pior 27 cm.

## Contra a corrida anterior (mesma config, mira alargada do codex)

|  | mira 0,15/0,20 | **mira 0,07/0,08 (042)** |
|---|---|---|
| paradas (pose congelada > 1,5 s) | 3 | **0** |
| amostras invadindo a caixa | 3 | **1** |
| rés | 3 | 43 amostras, nenhuma parada |
| desvio lateral mediano | — | **6 cm** |
| duração | 107,1 s | 111,5 s |

4 s mais lenta e sem parar nenhuma vez. A mira da 042 ganhou.

## Estado da dívida

Isto **resolve** a pendência aberta em `51b7023`: a mira alargada do codex
tinha sido levada ao robô sem nunca ter sido isolada. Agora foi, e o valor
medido (042) venceu no Gazebo também. Vai para o robô validado dos dois lados.

⚠️ Ainda n=1 nesta configuração. O dono pediu repetição — a próxima corrida
sobe do zero, sem mexer em nada.

⚠️ `parametros/collision_monitor.CONFIG-NAO-DUMP.yaml` é a config do repo, não
o dump do nó vivo: o `param dump` desse nó não respondeu em quatro tentativas.
Os outros oito são dump de verdade.

## Arquivos

- `corrida_01/seguidor_*.csv` e `corrida_01/parametros/*.yaml` — versionados
- `corrida_01/corrida_*/` — bag `--all-topics`, 2,4 GB, **local, fora do git**
