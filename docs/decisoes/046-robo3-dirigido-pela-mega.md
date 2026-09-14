# 046 — O robô 3 é dirigido pela MEGA, não pelo ros2_control

**Data**: 2026-09-14 (dev; provado sem hardware, com MEGA fingida)
**Status**: pronto para o primeiro teste no laboratório, com as rodas suspensas.
**Toca**: `robot_nav/launch/controle_robo3.launch.py`,
`robot_nav/config/teleop_xbox_robo3.yaml`, `robot_nav/config/twist_mux_robo3.yaml`,
`bin/sobe-robo3`. Firmware `firmware/mega_bridge` **sem alteração**.
**Vem de**: o dono quer mover o robô 3 no controle antes de montar o Livox. A
bancada de 10-09 terminou com a placa respondendo de forma intermitente e sem
nenhum frame de volta.
**Plano**: `docs/PLANO_CONTROLE_ROBO3.md`.

---

## 1. O problema

O robô 3 tem uma placa de hoverboard e uma MEGA. Para dirigir no controle
existiam duas cadeias neste repo, e a óbvia (a do robô 2) falharia **em
silêncio** de quatro jeitos, todos lidos no código:

1. **Placa sem resposta → controlador recusa comando.** O `hoverboard_driver` só
   tira a posição das rodas de NaN quando chega um frame da placa. Com
   `open_loop: false` (a config de caracterização do robô 3), o
   `diff_drive_controller` lê NaN e não comanda. Em 10-09 não chegou nenhum frame.
2. **Zona morta.** A compensação está desligada no robô 3 (correto para
   caracterizar). 0,30 m/s viram ~35 unidades na placa, e o motor só vence a
   inércia acima de ~60–80.
3. **Sentido.** Em 10-09, `speed > 0` andou de ré. O driver não tem sinal de comando.
4. **Ritmo.** O driver escreve a 10 Hz a partir do PC. A placa trava depois de
   um silêncio (bancada 01-09 §5), e o que girou as rodas em 10-09 foi 50 Hz sem
   pausa.

## 2. Alternativas

| alternativa | por que não |
|---|---|
| ros2_control com uma config "de teleop" (`open_loop: true`, zona morta ligada, 50 Hz) + parâmetro novo de sinal no driver em C++ | resolve 1, 2 e 4 por config, mas 3 exige mudar o driver; e continua dependendo do PC manter o ritmo da placa. Mais peças para um teste de "ele responde?" |
| `tools/teclado_placa.py` pela `hover_ponte` | já provado não confiável em 10-09, e sem ver a resposta da placa |
| **cadeia da MEGA com protocolo próprio** (`mega_bridge`) | **escolhida** |

## 3. Por que a MEGA

O firmware `mega_bridge` mora na MEGA e resolve do lado certo o que o PC não
controla:

- manda à placa a **50 Hz fixos**, com o último setpoint ou zero; o watchdog
  zera em 0,5 s sem PC. A placa nunca fica sem comando, então a trava por
  silêncio não tem como acontecer;
- **lê a resposta da placa** (pino 19) e publica bateria, rpm e placa muda há
  mais de 200 ms (`/battery/front.present`). É a pendência nº 1 de 10-09;
- watchdog de 2 s e timeout no I²C: sem IMU e sem sensor de fluxo montados, a
  leitura só falha e tenta de novo.

No PC, `cmd_vel_to_wheels` converte m/s em unidades da placa com
`left/right_wheel_sign`. Isso resolve o sentido sem tocar em C++. A escala é
direta, sem compensação: 0,30 m/s × 400 = 120, acima da zona morta.

A cópia do firmware e do `mega_bridge.py` neste repo difere da versão mais nova
da mesma cadeia **só** pelo suporte a BNO055, que o robô 3 não tem. Nada foi
copiado. O `platformio.ini` também não mudou: `--upload-port` na linha de
comando substitui o `/dev/mega` fixo.

## 4. Os dois pares que precisam casar

- `publish_stamped_twist: false` (teleop) ↔ `use_stamped: false` (mux) ↔
  `Twist` cru (`cmd_vel_to_wheels`). Se um só mudar, o DDS recusa por type hash
  e o robô não anda, sem erro nenhum.
- Por isso o `bin/robot-key` deste repo (TwistStamped) **não** entra no mux do
  robô 3.

## 5. Prova no dev

MEGA fingida numa pty; `/joy` publicado com LB apertado:

```
analógico frente (eixo 1 = 1,0)     31 frames  steer=0    speed=-120
analógico esquerda (eixo 0 = 1,0)   39 frames  steer=97   speed=0
LB solto                                       steer=0    speed=0
```

Bate com a conta: 0,30 × 400 × (−1) = −120; 1,5 × 0,3225/2 × 400 = 96,75.
`bin/sobe-robo3` contra uma MEGA fingida que manda frame de estado (36,50 V,
placa respondendo): conferências 🟢 e bag gravado; `--mata` limpa.

## 6. O que isto NÃO decide

- **Sinal −1 e sentido do giro não foram vistos no robô.** O −1 vem de uma
  observação de 10-09 feita por outra cadeia (teclado direto na placa). O teste
  suspenso confirma.
- Escala 400 e giro 1,5/2,3 rad/s são **de partida**. Nenhum número de
  movimentação sai daqui; isso é trabalho com o Livox.
- Não resolve placa intermitente. Se ela voltar, agora aparece no bag
  (`present`) em vez de ser palpite.
- O ros2_control continua sendo o caminho da caracterização
  (`robo3-bancada-validacao`, decisão 045).
