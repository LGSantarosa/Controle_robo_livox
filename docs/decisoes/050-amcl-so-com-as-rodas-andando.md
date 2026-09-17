# 050 — Robô 2: o AMCL só corrige a pose quando as rodas andam

**Data**: 2026-09-17 (PC de dev, robô desligado)
**Status**: aplicada em código, **NÃO testada no robô**.
**Toca**: `robot_base/congela_parado.py` (novo), `robot_base/tf_odom.py`,
`launch/localizacao.launch.py`, `launch/base.launch.py`,
`robot_motion/config/localizacao_amcl.yaml`. **Só o robô 2.**

## 1. O pedido

O dono: a pose do robô 2 pula no mapa com o robô parado ("o bug clássico do
AMCL"). Quer que ela só atualize quando as rodas se mexem, como no robô 1.

## 2. Como o robô 1 faz (conferido no `Controle_robo_web`, não de memória)

Não há código explícito. O efeito vem de duas coisas:

- o `odom → base_link` do `pose_estimator.py` tem translação **só de encoder**
  e giro de IMU com âncora magnética: roda parada ≈ odom parado, e o AMCL nunca
  chega ao `update_min_d`/`update_min_a`;
- o AMCL dele não declara `recovery_alpha_*` → padrão 0, **sem** injeção de
  partícula aleatória.

⚠️ Na primeira resposta eu disse que o robô 1 "fazia assim" antes de ler o
código; o dono perguntou, e a leitura mostrou que não há um gate explícito.

## 3. O que muda no robô 2

1. **Cópia fiel**: `recovery_alpha_slow/fast` 0,001/0,1 → 0,0/0,0.
2. **Mecanismo novo que imita o efeito**: no robô 2 o odom é o LIO, que oscila
   e deriva parado. O `tf_odom` ganhou `congela_parado` (padrão `false`): com
   as DUAS rodas (`/hoverboard/*_wheel/velocity`, realimentação medida da
   placa, rad/s em passos de 0,105) abaixo de 0,05 rad/s por 0,5 s, a TF
   publicada fica parada e o que o LIO anda vai para uma correção `C`. Ao
   voltar a andar publica `C ∘ LIO` — sem degrau.
3. Só a `base.launch.py` (robô 2) liga. Robô 3 e simulador ficam no padrão.

## 4. Alternativas descartadas

- **Só aumentar `update_min_d/a`**: adia o pulo, não o impede — a deriva
  acumula do mesmo jeito.
- **Só zerar `recovery_alpha`**: tira uma fonte de pulo, mas o LIO parado ainda
  faz o AMCL reamostrar.
- **Leitura de roda velha congela** (como o robô 1 com a MEGA muda): descartada.
  Aqui o LIO é a verdade; travar a TF de um robô que pode estar andando é pior.

## 5. Riscos a olhar no ensaio

- Robô **empurrado com a mão** ou roda patinando com leitura zero: a TF não
  acompanha até a roda girar.
- A placa retém ~0,5 s depois do comando zero (DIARIO); a espera de 0,5 s é
  chute, não medida.
