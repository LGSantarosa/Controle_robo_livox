# Travessia com o gargalo DESLIGADO — Gazebo — 2026-08-20

Primeira corrida aprovada pelo dono depois da leva do codex.
Veredito dele: **"passou top a ida. A volta tmb foi."**

## Condição

- commit: `7fa1ac3` (o alvo no centro do vão é uma singularidade)
- mundo/mapa: `pista_obstaculos`, spawn `(2,00 · 5,00)`
- ida `(10,65 · 6,60)`, volta `(2,00 · 5,00)`
- `passagem_estreita_habilitada: False` — a mudança sob teste
- smoother do BT: `simples`; `passagem_v_max: 0,5`; `a_dec: 0,3`
- pivô de desencalhe e recuperação infinita LIGADOS (pedido do dono, 20-08)

## Medido (`corrida_01`)

Ida e volta completas em **107,1 s**, 3 rés curtas, nenhuma dentro de um vão.

| travessia | yaw na soleira | folga da CAIXA | folga do CORPO | invasões |
|---|---|---|---|---|
| ida, porta 1   | +5,1° / −3,1°   | +0,143 / +0,054 | +0,209 / +0,105 | 0 |
| ida, porta 2   | +1,6° / +24,4°  | −0,015 / +0,053 | +0,036 / +0,114 | 3 |
| volta, porta 2 | ≈18° do eixo    | +0,104 / +0,002 | +0,154 / +0,082 | 0 |
| volta, porta 1 | ≈4° do eixo     | +0,109 / +0,026 | +0,187 / +0,085 | 0 |

**3 amostras de invasão da caixa na corrida inteira** — contra 90 a 220 por
travessia nas corridas com o gargalo ligado. O corpo teve folga em todas.

Desvio lateral: pior 23 cm, p90 15 cm.

## Comparação com o mesmo percurso

|  | 13:56 gargalo ON | 14:37 gargalo ON | **14:43 gargalo OFF** |
|---|---|---|---|
| ida, porta 1 (yaw) | +1,6° | **+86°** | +5,1° |
| ida, porta 2 (yaw) | +4 a +7° | preso 55 s | +1,6° |
| rés | 7 | 5+ | **3** |
| duração | 209 s | não terminou | **107 s** |

As duas primeiras rodaram o MESMO código (só o `cost_check_points` diferia, e
ele pertence a um smoother fora da cadeia). É essa instabilidade — mesma
configuração, resultados opostos — que motivou desligar a travessia de gargalo.

## ⚠️ Uma métrica que NÃO vale como prova

O `raio_curva` do CSV caiu de 0,067 m para 0,365 m de mínimo, e a fração abaixo
dos 0,37 m da máquina foi de 7% para 1%. **Parte disso é artefato**: essa coluna
é medida na janela da mira, e sem o modo gargalo a mira deixou de encolher para
0,14 m. Janela maior mede curva mais suave. O que NÃO é artefato e sustenta o
resultado: yaw na soleira, folga, invasões e número de rés.

## O que ainda não está provado

n=1. A falha que foi removida depende de quão centrado o robô chega no vão, e
já produziu corrida boa e corrida péssima com o mesmo código. Repetir o percurso
mais duas vezes SEM MEXER EM NADA antes de tratar isto como resolvido.

## Arquivos

- `corrida_01/seguidor_*.csv` — CSV do seguidor (versionado)
- `corrida_01/parametros/*.yaml` — dump dos 9 nós vivos (versionado)
- `corrida_01/corrida_*/` — bag `--all-topics`, 2,4 GB (**local, fora do git**)
