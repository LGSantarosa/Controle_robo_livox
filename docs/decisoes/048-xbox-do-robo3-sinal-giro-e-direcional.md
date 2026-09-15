# 048 — Xbox do robô 3: sinal 1.0, giro 4,0/7,5 e direcional de reta pura

**Data**: 2026-09-14 (notebook do robô 3, rodas suspensas)
**Status**: aplicada (`b9dcbeb`, `c5376e2`, `b4d2c39`, `877d632`), aprovada pelo
dono no teste.
**Toca**: `config/teleop_xbox_robo3.yaml`, `config/twist_mux_robo3.yaml`,
`launch/controle_robo3.launch.py`, `robot_nav/dpad_reto.py`, `bin/sobe-robo3`.
**Vem de**: decisão 046 (Xbox pela MEGA). Os números de lá eram de partida e
nunca tinham rodado com a placa respondendo; o GND bom só veio em 14-09 ~20:25.

---

## 1. Os três problemas do primeiro teste

1. **Giro fraco**, normal e turbo: as rodas quase não se mexiam.
2. **Giro para o lado invertido** com `sinal:=-1.0`.
3. O dono não sabia se um desvio na reta era do robô ou do polegar no analógico.

## 2. O que foi medido

- Conta da cadeia: `steer = yaw × bitola/2 × escala = yaw × 64,5`. Com 1,5/2,3 o
  Xbox mandava `steer` 97/148.
- `teclado_20260914_202729.csv` (PC dev, GND bom, rodas no ar): `steer 150` em
  pivô → ~17 rpm; `speed 250` reto → ~117 rpm. A placa entrega o `steer` bem mais
  fraco que o `speed` (mecanismo dentro do firmware da placa, não medido).
- `sinal` é aplicado às duas rodas: inverter as duas troca o `speed` **e** o
  `steer` (o próprio launch já avisava). O −1 veio de 10-09 ("speed>0 = ré").
- `joy_dpad_204304.csv` (notebook): direcional cima/baixo = eixo 7 (+1 cima),
  esquerda/direita = eixo 6 (+1 esquerda); LB = 6, RB = 7.

## 3. Alternativas

| problema | alternativa | por que não |
|---|---|---|
| giro fraco | `wheel_scale` no `mega_bridge` | multiplica a reta junto, que estava boa |
| giro fraco | compensar o `steer` dentro do `mega_bridge` | muda a semântica do setpoint que a odometria vai usar; sem medir o mecanismo da placa |
| giro fraco | **subir `scale_angular` no YAML** | **escolhida**: uma linha, reversível, sem recompilar código |
| giro invertido | escala de giro negativa, mantendo `sinal:=-1` | proposta; o dono preferiu "a frente é o que as rodas acreditam" |
| giro invertido | **`sinal` 1.0 como padrão** | **escolhida pelo dono** |
| giro invertido | trocar os cabos das rodas | pela conta inverte só frente/ré (placa espelha os canais); fica como opção física, não aplicada |
| reta ambígua | limitar o giro do analógico perto do centro | ainda deixa desvio pequeno passar e muda o manche inteiro |
| reta ambígua | **nó `dpad_reto`, LB + direcional, `angular.z = 0`, acima do analógico no mux (110 > 100, timeout 0,3 s)** | **escolhida**: comando exato separado, sem mexer no teleop |

## 4. Resultado

- Giro 4,0/7,5 com `sinal` 1.0: dono aprovou força e direção.
- Direcional: "andou reto, não 100%, mas aí é erro dele". **Desvio não medido**:
  o bag `controle_20260914_205608` ficou sem índice.

## 5. O que isto NÃO decide

- O "rad/s" do teleop deixou de ser físico; a calibração real vem com o Livox.
- Tudo foi com as rodas no ar. No chão o giro pode pedir mais.
- A frente com `sinal` 1.0 é a das rodas; se a frente do corpo for a outra, é
  escolha de montagem (ou trocar os cabos das rodas), a registrar quando o Livox
  entrar.
- Não mede se a reta desviando é da placa, da roda ou do chassi.
