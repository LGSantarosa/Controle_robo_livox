# 060 — Progresso é o que falta pelo plano, não a reta até o objetivo

**Data**: 2026-10-02 (dev, robô desligado)
**Status**: implementada e coberta por teste unitário; falta validar no chão
**Toca**: `robot_motion/lei_de_seguimento.py`, `robot_motion/path_follower.py`
**Vem de**: ré do corredor no robô 2, 01-10 (diário e
`docs/dados/2026-10-01-robo2-curva-forte/`)

## Problema

O gatilho de emperrado (`ProgressoDeAvanco`, 4 s sem cair 5 cm) era alimentado
com a distância em linha reta até o objetivo. Numa curva longa essa distância
SOBE com o robô andando certo: em 01-10 (t=1790891749), a 0,5 m/s e com o rumo
indo de 177° para 103° em 5 s, ela foi de 20,92 para 21,83 m. O gatilho
disparou escape e depois ré no corredor, e a traseira varreu a parede. A mesma
corrida teve cerca de 15 escapes para frente pelo mesmo motivo.

## Alternativas

1. **Emperrado = o robô não se mexeu** (deslocamento < 5 cm em 4 s).
   Descartada: deixa de pegar a órbita (anda sem avançar), que é a razão de o
   detector existir. Além disso, o movimento seguraria a liberação do
   replanejamento (`aceita_replano`), o que piora o plano velho.
2. **Progresso pelo arco do plano** — escolhida. `restante_pelo_plano(plano,
   i0)` soma os trechos do ponto do plano mais próximo do robô até o fim. Na
   curva esse valor cai. Na órbita, o ponto mais próximo não avança e o valor
   não cai.

## Decisão

- `i0` passa a ser calculado uma vez por ciclo, antes da guarda de plano
  velho. As duas chamadas do gatilho passam a usar o restante pelo plano.
- `cb_plano`: um plano **aceito** reinicia o progresso, porque a rota nova tem
  outro comprimento. Um replano **recusado** (rota travada) não reinicia;
  senão o replano de 5 s do BT zeraria o relógio de 4 s para sempre.
- O parâmetro do `atualiza` passa a se chamar `restante`.
- **Fora desta mudança**: a `dist_antes_da_re` (dívida da ré) continua na reta.

## Limites conhecidos

- No modo de plano cru sem suavizador (`aceita_plano_cru`), a rota não trava
  e todo plano é aceito. Assim, cada replano zera o relógio. Com o BT a 5 s e
  o gatilho a 4 s ele ainda dispara, mas com pouca folga.
- Num plano que cruza a si mesmo, o índice mais próximo pode saltar. Os
  planos do Smac suavizado não fazem isso no uso atual.

## Teste

`test_lei_de_seguimento.py`: a curva de 01-10 reproduzida num plano em U. A
métrica antiga dispara e a nova não. A órbita de raio 0,3 m continua
disparando. `test_plano_suavizado.py`: plano aceito reinicia o progresso e o
recusado não.
