# Plano — dirigir o robô 3 no controle Xbox (sem Livox)

> Escrito em 2026-09-14, antes de qualquer código. Objetivo único: **ligar o
> robô 3 e mover ele no controle**, para saber se ele responde certo. Calibrar
> movimentação fica para depois, com o Livox montado.

## Por que o caminho da MEGA com protocolo próprio, e não o ros2_control

Duas cadeias de comando existem neste repo:

| | ros2_control (`hoverboard_driver`) | MEGA com protocolo próprio (`mega_bridge`) |
|---|---|---|
| quem fala com a placa | o PC, frame a frame, a 10 Hz | a MEGA, a 50 Hz fixos |
| silêncio do PC | a placa fica sem comando e **trava** (bancada 01-09 e 10-09) | a MEGA segue mandando zero; watchdog de 0,5 s |
| resposta da placa | só no PC; sem frame, posição NaN e o controlador **recusa comando** com `open_loop: false` | lida na MEGA e enviada como bateria, rpm e aviso de placa muda |
| zona morta | compensação desligada no robô 3 → 0,30 m/s vira ~35 e o robô **fica parado em silêncio** | escala direta em unidades da placa; 0,30 m/s → 120 |
| sentido invertido (10-09: `speed>0` anda de ré) | sem parâmetro para inverter o comando | `left/right_wheel_sign` já existem |
| camadas | mux, compensador de rumo, ganho de giro, retenção | joy → teleop → mux → rodas |

O robô 3 tem MEGA e uma placa só. A cadeia da MEGA já trata os quatro defeitos
que derrubariam a outra hoje, e o que está em `firmware/mega_bridge` e em
`robot_nav/mega_bridge.py` cobre este caso. A única diferença para a versão mais
nova da mesma cadeia é o suporte a BNO055, que o robô 3 não tem. **Nada é
copiado; só se configura.**

O ros2_control continua sendo o caminho da caracterização
(`robo3-bancada-validacao`). Esta decisão é só para o teste de hoje.

## A cadeia

```
Xbox ─/dev/input/jsN─> joy_node ─/joy─> teleop_twist_joy ─/joy_vel─> twist_mux ─/cmd_vel─>
   cmd_vel_to_wheels ─/wheel_vel_setpoints─> mega_bridge ─USB 230400─> MEGA ─Serial1 (18/19)─> placa
```

Feedback de volta: `/battery/front`, `/hoverboard/wheel_velocities`, `/system/health`.

## Números de partida (NÃO calibrados)

| parâmetro | valor | de onde |
|---|---|---|
| `wheel_base` | 0,3225 m | bitola do robô 3 (URDF, fechada em 02-09) |
| `linear_scale` | 400 un/(m/s) | mesmas rodas 6,5"; 0,30 m/s → 120, acima da zona morta de ~60–80 do motor |
| `left_wheel_sign` / `right_wheel_sign` | −1 / −1 | 10-09: `speed>0` andou de ré. Inverter os dois inverte speed **e** steer, o que corresponde a uma reversão completa |
| linear normal / turbo | 0,30 / 0,50 m/s | limites conservadores |
| giro normal / turbo | 1,5 / 2,3 rad/s | pivô a 1,5 → ±97 na placa, perto do que o teclado usou (150). **Não** é o 6,0 do chassi 4 rodas |
| botões Xbox | LB = 6 homem-morto, RB = 7 turbo | medido por ioctl; reconferir com `scripts/js_mapping.py` |

⚠️ O giro não tem zona morta medida neste robô. Se o pivô não sair do lugar em
1,5, é zona morta aparecendo, não defeito. Sobe-se o número e anota-se.

## Passos (um commit cada)

1. **Este plano.**
2. **`firmware/mega_bridge/platformio.ini`**: a porta de gravação vem de
   `--upload-port` na linha de comando. O `/dev/mega` fixo é regra udev de outra
   máquina.
3. **`robot_nav/config/teleop_xbox_robo3.yaml`** e **`twist_mux_robo3.yaml`**:
   `publish_stamped_twist: false` e `use_stamped: false`, os dois juntos. Se só
   um mudar, o DDS recusa a ligação por type hash e o robô não anda. O mux tem
   **só o controle**: o `robot-key` deste repo publica stamped e não entra aqui.
4. **`robot_nav/launch/controle_robo3.launch.py`**: sobe mega_bridge,
   cmd_vel_to_wheels, joy_node (autorepeat 20 Hz, deadzone 0,10), teleop e mux.
   Os sinais, a escala e a porta viram argumentos, para inverter no lab sem
   recompilar.
5. **`bin/sobe-robo3`**: recusa se já houver instância dele ou conflito no
   grafo ROS (desde 22-09, etapa 4 passo 6a: não mata mais nada por nome; o
   `--mata` só derruba o que ele próprio subiu), sobe a launch e confere a
   bateria da placa, se a placa responde e o `/joy`. Grava bag em
   `~/bancada_robo3/controle_<data>/` com `/joy /joy_vel /cmd_vel
   /wheel_vel_setpoints /hoverboard/wheel_velocities /battery/front
   /system/health`.
6. **Documentação**: decisão 046, DIÁRIO e ESTADO.

Validação no dev antes do deploy: build do `robot_nav`, launch sobe sem MEGA
(erro de porta esperado e legível), `bash -n` no script, `pio run` compila.

## Roteiro no laboratório

| # | estado do robô | o que fazer | passa se |
|---|---|---|---|
| 0 | DESLIGADO | `git fetch && git reset --hard` + `colcon build --packages-select robot_nav` | build ok |
| 1 | placa DESLIGADA, MEGA no USB | `pio run -d firmware/mega_bridge -t upload --upload-port /dev/ttyACM0` | upload ok (substitui a `hover_ponte`) |
| 2 | **rodas SUSPENSAS**, placa LIGADA | `bash bin/sobe-robo3` | bateria > 0 e placa respondendo |
| 3 | suspenso | LB + analógico para a frente | as duas rodas giram no sentido "frente" |
| 4 | suspenso | LB + analógico para a esquerda | esquerda para trás e direita para a frente |
| 5 | suspenso | soltar o LB no meio | as rodas param em < 0,5 s |
| 6 | NO CHÃO | frente, ré, pivô e curvas devagar; RB só depois | robô responde ao controle |

Frente trocada: `sinal:=1.0`. Frente certa e giro trocado:
`bitola:=-0.3225`. A conta é direta: em
`v_left = v - wz·bitola/2` e `v_right = v + wz·bitola/2`, trocar o sinal da
bitola inverte somente os termos de giro e preserva o comando reto. Usar sinais
opostos em `left_wheel_sign` e `right_wheel_sign` estaria errado: transformaria
o comando reto em pivô. Nos dois casos, anotar o que se viu.

**Arme da placa:** com a `sobe-robo3` de pé a MEGA já manda zero a 50 Hz. Se a
placa não armar, religue-a com tudo rodando.

## O que este teste NÃO prova

- Nada de precisão de movimento: escala e giro são de partida.
- Se a placa voltar a responder de forma intermitente como em 10-09, o controle
  também vai. A diferença é que agora o bag mostra se o comando saiu e se a
  placa respondeu.
- O `teclado_placa.py` fala com a `hover_ponte`; com o `mega_bridge` gravado ele
  não funciona.
