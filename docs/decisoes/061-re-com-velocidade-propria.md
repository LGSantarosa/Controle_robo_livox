# 061 — A ré ganha velocidade própria (`re_v` 0,24 m/s), separada do `v_piso`

**Data**: 2026-10-02 (dev, robô desligado)
**Status**: implementada e coberta por teste unitário; falta validar no chão
**Toca**: `robot_motion/path_follower.py`, `config/perfil_robo3.yaml`
**Vem de**: ré do corredor no robô 2, 01-10; segue a 060

## Problema

A ré do seguidor andava no `v_piso`. Em 01-10 o `v_piso` subiu de 0,24 para
0,36 m/s junto com o `wz_max` 2,2, porque o piso inclui `wz_max·bitola/2`.
Esse acoplamento não foi separado na hora. Sem freio linear, a placa ainda
empurra ~0,5 s depois do corte, e a ré do corredor varreu a parede com a
traseira.

## Alternativas

1. Voltar o `v_piso` para 0,24: desfaz a curva forte aprovada no chão.
   Descartada.
2. **Parâmetro próprio da ré** — escolhida. A ré anda reta, então não precisa
   do piso que existe para caber o giro.

## Decisão

- `re_v` = 0,24 m/s, o valor do `v_piso` antes de 01-10. Usado só quando
  `sentido < 0`.
- O escape para frente continua no `v_piso`.
- A invariante da janela de `/scan` vencido passa a usar `re_v`:
  0,8 s × 0,24 = 0,19 m às cegas, contra 0,30 m de folga.
- Perfil do robô 3: `re_v` entra como herdado do robô 2 (etapa 8).

## Teste

`test_re_desligada.py`: com `v_piso` 0,3648 ou 0,50, a ré publica −0,24, e o
escape para frente publica o `v_piso`. `test_configs_coerentes.py`: cegueira
do `/scan` calculada com `re_v`.
