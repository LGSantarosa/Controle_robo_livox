# 049 — Robô 3 de costas: trocar frente e ré sem mexer no giro

**Data**: 2026-09-16 (PC de dev, robô desligado — nada testado no robô ainda)
**Status**: aplicada em código, **NÃO testada no robô**. É o **PADRÃO** do robô
3 desde 16-09, por decisão do dono: `bash bin/sobe-robo3` já sobe com a antiga
ré como frente, sem argumento nenhum. `frente:=1.0` volta a frente antiga.
**Toca**: `robot_nav/cmd_vel_to_wheels.py`, `launch/controle_robo3.launch.py`,
`bin/sobe-robo3`.
**Vem de**: decisão 048 (números do Xbox) e da pergunta aberta do
`ESTADO_PROJETO`: *por que a frente puxa para a direita e a ré vai reta?*

---

## 1. O que o dono pediu

Contornar, não explicar: *"inverte aí a ré e a frente, quero só que esse bixo
funcione, não descobrir o porquê ele anda reto errado"*. Como a ré já anda reto
(robôs 2 e 3, 15-09), promover a ré a frente é um contorno legítimo enquanto a
causa não é investigada.

Isto **não fecha** a pergunta de 16-09 do `ESTADO_PROJETO` — só a adia.

## 2. Por que o `sinal:=-1.0` que já existia não serve

`sinal` aplica o mesmo fator às DUAS rodas. Isso é uma **reflexão**, não uma
rotação: inverte `speed` e `steer` juntos, e o giro sai trocado para quem
dirige. Foi exatamente o problema 2 da decisão 048, e o motivo de o dono ter
fixado `sinal` em 1.0.

Virar o robô de costas é uma **rotação de 180° do `base_link`**, e ela troca
duas coisas ao mesmo tempo: o sentido da frente **e** qual roda é a esquerda.
Na cinemática:

    frente nova = −frente velha ,  esquerda nova = roda direita velha

    v_esq_novo  = v − ω·b/2   com v ← −v   →  saída esquerda = −v − ω·b/2
    v_dir_novo  = v + ω·b/2   com v ← −v   →  saída direita  = −v + ω·b/2

que é **idêntico** a negar só o `linear` e deixar o `angular` em paz — as duas
inversões (sentido e lado) se cancelam no termo do giro. Daí o parâmetro novo
`linear_sign`, e não um segundo `sinal`.

Resultado prático: frente ↔ ré trocadas, **esquerda continua esquerda** para
quem está com o controle na mão.

## 3. Alternativas

| alternativa | por que não |
|---|---|
| `sinal:=-1.0` (já existia) | espelho: inverte o giro junto — reprovado pelo dono em 14-09 |
| inverter o eixo linear no `teleop_xbox_robo3.yaml` | pega o analógico mas **não** o `dpad_reto` (que publica Twist direto), e nada do que vier por nav/web |
| inverter dentro do `dpad_reto` também | dois lugares para manter em sincronia; o `cmd_vel_to_wheels` é o funil por onde tudo passa |
| trocar os cabos das rodas na placa | pela conta da decisão 048 dá o mesmo efeito, mas é mudança física, não reversível por argumento de launch, e some do registro |
| **`linear_sign` no `cmd_vel_to_wheels`, exposto como `frente:=` no launch** | **escolhida**: um ponto só, reversível sem recompilar, default neutro |

## 4. O que isto NÃO decide e NÃO resolve

- **Não explica** por que a frente puxa para a direita. A hipótese do sentido
  físico da roda (pivô parcial de 15-09) segue aberta.
- **Se a causa for do lado, e não do sentido**, o desvio pode simplesmente
  trocar de lado junto com a frente — e aí o contorno falha. É o primeiro
  resultado a olhar no teste.
- **Vale só para o robô 3**, que hoje é teleop puro: sem Livox, sem
  `pose_estimator`, sem `/odom`. O robô 2 fica **fora** desta decisão — lá o
  Livox define a odometria e o giro de 180° teria de ir junto no URDF e nos
  calibres de frente/ré, que são separados (`curv_frente` × `curv_re`).
- **O nav2 do robô 3 ainda não existe** — vai ser adaptado. Quando for, ele
  passa pelo mesmo `cmd_vel_to_wheels`: basta o launch novo mandar
  `linear_sign: -1.0` junto. ⚠️ Não mudar o default DO NÓ para -1.0: ele é
  compartilhado com o `robot.launch.py`, e isso viraria a frente do robô 2 sem
  ninguém pedir.
- O `robo3.urdf.xacro` (simulador) **não foi tocado**: lá a frente continua a
  antiga. Enquanto o robô 3 não tiver pilha de navegação, isso não morde.
- Cobre tudo o que dirige o robô 3 hoje — analógico e `dpad_reto` passam os
  dois pelo `cmd_vel_to_wheels`.
- Nada foi ao robô: o teste é `bash bin/sobe-robo3` (sem argumento) e LB +
  direcional cima, com o bag de sempre.
