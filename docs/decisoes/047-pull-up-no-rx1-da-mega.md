# 047 — Pull-up no RX1 (pino 19) do `mega_bridge`

**Data**: 2026-09-14 (bancada, robô 3, rodas suspensas)
**Status**: aplicada (`144b739`), confirmada por A/B no robô.
**Toca**: `firmware/mega_bridge/src/main.cpp` (uma linha no `setup()`).
**Vem de**: decisão 046. O robô 3 não armava a placa pelo `mega_bridge`, nem
pelo ROS nem pelo teclado, e armava pelo teclado com a `hover_ponte`.

---

## 1. O problema

Com o `mega_bridge` gravado, a placa ligava sempre com o beep de travada, com
ou sem comando. Pela `hover_ponte`, a mesma placa armava quando ligada
recebendo um comando diferente de zero (o `S` segurado no teclado).

## 2. O que foi medido antes de mexer

Instrumento `FT_DEBUG` → `/mega/debug` (`384d20b`): o que a MEGA aceitou do PC
e o que escreveu no `Serial1`.

| corrida | firmware | o que a MEGA escreveu na placa | placa |
|---|---|---|---|
| ROS, placa desligada (`/wheel_vel_setpoints` +200) | `mega_bridge` | 20,2 de 20 comandos/s aceitos, 50,2 Hz no `Serial1`, 0 checksum errado, `speed=200` | — |
| ROS, LB+RB+analógico para trás ao ligar | `mega_bridge` | 538 setpoints positivos até +200 no bag | não armou |
| teclado `--mega`, `S` ao ligar (`teclado_mega_20260914_191731.csv`) | `mega_bridge` | os mesmos +250/−250 e rampas do teclado, 49,7 Hz, 52 s, nenhum recusado | **não armou** |
| teclado, `S` ao ligar | `hover_ponte` | (sem instrumento; mesmos quadros 0xABCD a 115200) | **armou** |

Os bytes e o ritmo iguais descartam o ROS, o conteúdo e o ritmo do comando. A
diferença tinha de estar no firmware da MEGA, mas fora dos bytes enviados.

## 3. A diferença encontrada

A `hover_ponte` faz `pinMode(19, INPUT_PULLUP)` desde o primeiro commit
(`7988d1e`), porque em 10-09 o 19 flutuava e virava ruído (`hover_sniff`). O
`mega_bridge` não faz, e o `HardwareSerial::begin` do núcleo AVR só liga
`RXEN/TXEN/RXCIE` — não liga pull-up
(`framework-arduino-avr/cores/arduino/HardwareSerial.cpp`). Todas as vezes em
que a placa armou pela MEGA, o 19 tinha pull-up.

## 4. Alternativas

| alternativa | por que não |
|---|---|
| voltar o robô 3 para a `hover_ponte` | perde o comando a 50 Hz na MEGA, o watchdog e o instrumento; o ROS não fala 0xABCD por ela |
| resistor de pull-up externo no 19 | mesmo efeito, mas é mudança de hardware sem ninguém no lab para isso, e some se a MEGA for trocada |
| investigar outros pinos que o `mega_bridge` configura (I²C 20/21, SPI 50–53, 7/8/9, `Serial2` 16/17) | nenhum fio conhecido do robô 3 passa por eles; o 19 é o único pino da cadeia da placa que difere |
| **`pinMode(19, INPUT_PULLUP)` depois do `Serial1.begin`** | **escolhida**: uma linha, igual à `hover_ponte`, reversível |

## 5. Resultado

`teclado_mega_20260914_192125.csv`, mesmo ritual: **armou como pela
`hover_ponte`**, o `W` girou as rodas. Depois parou de novo, com a MEGA ainda
escrevendo ±250 a 49,7 Hz.

## 6. O que isto NÃO decide

- **O mecanismo não foi medido.** Não se sabe por que o nível do RX da MEGA
  (o fio azul, saída da placa) muda o arme da placa. O A/B mostra o efeito, não
  a causa física.
- **"Roda ~1 s e para" continua**, pelos dois firmwares, com comando contínuo
  chegando ao fio. É da placa, e segue sem medida porque o retorno dela (bateria,
  rpm) nunca chegou — nem com o pull-up, numa escuta logo depois da corrida em
  que não se sabe se a placa ainda estava ligada.
- Não mexe no robô 1: este repositório é independente.
