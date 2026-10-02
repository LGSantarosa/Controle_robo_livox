# 062 — O Gazebo do robô 2 passa a andar reto (roda omni)

**Data**: 2026-10-02 (dev, sem Gazebo e sem robô)
**Status**: implementada e coberta por teste unitário; falta uma corrida no
Gazebo com o dono olhando
**Toca**: `robot_base/placa_simulada.py`, `robot_motion/launch/pilha.launch.py`,
`tools/linha_de_base/argumentos_launch.yaml`
**Vem de**: o dono, 02-10: "os erros laterais não existem mais, ele anda reto
agora por conta da roda omni"; precondição para investigar o plano velho no
Gazebo

## Problema

A placa simulada impunha o arco medido em 04-08 com a roda BOBA: curvatura
−0,817 1/m de frente (raio 1,22 m) e −0,098 de ré, transformada em assimetria
de roda. Em 30-09 a boba foi trocada por uma roda omni, e o robô passou a
andar reto. O Gazebo continuou modelando a máquina antiga, então o que se
testasse nele herdaria um erro lateral que o robô não tem mais.

A `pilha.launch.py` ainda passava ao compensador de rumo o feedforward antigo
(−0,817/−0,098), inclusive com `sim:=true`. Zerar só a placa faria o Gazebo
curvar para o lado OPOSTO, porque o compensador anularia um arco que não existe
mais.

## Decisão

- `placa_simulada`: `curvatura_frente` e `curvatura_re` nascem **0,0**. Os
  números de 04-08 ficam no comentário e nos testes, como histórico da boba.
- `pilha.launch.py`: `curv_frente`/`curv_re` são **0,0 com `sim:=true`**. No
  robô real o padrão continua o do nó (−0,817/−0,098).
- O aviso de log do arco dividia frente por ré e derrubaria o nó com zero.
  Agora vira `aviso_do_arco()`, que diz "anda reto" quando as duas são zero.
- Ficam intactos: patamar, latência, retenção de desliga e `rendimento_giro`,
  que não dependem da roda.

## O zero vem de observação, não de medida

O dono viu o robô andar reto com a omni. Ninguém mediu a curvatura dela. A
medida (`medir.py curvatura`) fica para o chão. Se não der zero, o número entra
nos dois lugares acima.

⚠️ **O robô real NÃO muda aqui.** O `bin/sobe-robo` ainda passa
`curv_frente:=-0.8365` (medido em 20-08, com a boba). Com a omni esse
feedforward pode estar puxando o robô para o lado contrário. É para conferir no
chão, no lab, e não para mudar às cegas.

## Teste

`test_placa_simulada.py`: a placa nasce reta (assimetria e curvatura zero
nos dois sentidos), e o aviso não cai com zero. Os testes do arco de 04-08
continuam, com os números passados à mão (`BOBA`).
`test_configs_coerentes.py`: o padrão do robô real é igual ao do nó, e o do
simulador é zero.
