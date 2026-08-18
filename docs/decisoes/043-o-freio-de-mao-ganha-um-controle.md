# 043 — O freio de mão ganha um controle Xbox

**Data**: 2026-08-18 (dev, robô desligado)
**Status**: implementado e testado **no dev**; nada verificado no robô ainda —
a etapa de campo está descrita em §7 e é o que falta.
**Toca**: `ros2_packages/robot_motion/config/teleop_xbox.yaml` (novo),
`config/twist_mux.yaml` (faixa nova), `launch/joystick.launch.py` (novo),
`test/test_joystick_coerente.py` (novo), `bin/sobe-robo` (passo 4/5 e o
`vivos()`), `pair-xbox.sh` + `scripts/_bluez_fixes.sh` + `scripts/js_mapping.py`
(portados), `scripts/setup_headless.sh`.
**Vem de**: o pedido do dono — adaptar o robô 2 ao controle Xbox *"assim como
foi feito recentemente para o robô 1"* — e do commit `ecd6e70` do
`Controle_robo_web`, que é a receita medida numa Raspberry Pi em 2026-08-11.

---

## 1. O que o robô 2 tinha antes disto: nada

O `joy_node` e o `teleop_twist_joy` existiam no repo, mas só dentro de
`ros2_packages/robot_nav/launch/robot.launch.py` — herança morta do clone do
robô 1. O `bin/sobe-robo`, que é o que sobe a máquina de verdade, nunca chamou
aquilo, e o `twist_mux` da pilha viva (`robot_motion/config/twist_mux.yaml`)
não tinha faixa de joystick nenhuma:

```
teclado 90   web 50   desencalhe 30   autonomia 10
```

Ou seja: os dois canais humanos deste robô eram um terminal SSH e um navegador.
Nenhum deles é operável **ao lado da máquina**, olhando para ela.

## 2. Por que prioridade 100, acima do teclado

O `CLAUDE.md` diz que "humano tem prioridade sobre goal" vale nos dois robôs. O
que ele não resolve é a ordem **entre humanos**, e agora são três.

O critério escolhido é **quem enxerga o robô**. Teclado e web são operados de
onde a pessoa estiver — outro cômodo, outro andar. O joystick é operado por
quem está do lado, vendo para onde a máquina aponta. Entre duas mãos humanas
discordando, ganha a que tem a informação.

⚠️ E prioridade 100 só é defensável porque este canal tem **freio de mão
físico**: `require_enable_button: true` faz o `teleop_twist_joy` publicar
apenas enquanto o LB está apertado. Sem isso, prioridade máxima seria um robô
sequestrado por um analógico encostado na mochila. Há teste travando o par.

## 3. Três números que NÃO atravessaram do robô 1

Copiar o `teleop_xbox.yaml` de lá teria dado três defeitos, e **o primeiro não
dá sintoma**.

### 3.1 `publish_stamped_twist` — o silencioso

```
robô 1:  twist_mux use_stamped: false   ->  teleop publish_stamped_twist: false
robô 2:  twist_mux use_stamped: true    ->  teleop publish_stamped_twist: TRUE
```

A cadeia inteira do robô 2 fala `TwistStamped` (`diff_drive_controller`, placa
fingida, `compensador_rumo`). Com o valor do robô 1 aqui, o nó sobe, o `/joy`
publica, o `/joy_vel` aparece em `topic list` — e o DDS recusa a ligação por
*type hash*. O mux fica sem publisher naquela entrada e **o robô ignora o
controle sem uma linha de erro**.

É a mesma classe de defeito da 040 (frame validado no simulador onde ele por
acaso batia) e do de 29-07 (bitola divergente entre quatro arquivos): a peça
está certa em si e errada em relação à vizinha. Por isso ela virou teste, e o
teste foi conferido quebrando o arquivo de propósito.

### 3.2 `scale_angular: 6.0` — o knob que o CLAUDE.md proíbe herdar

Aqueles 6,0 rad/s eram anti-skid: o chassi de 4 rodas do robô 1 não apoiava
uniformemente, as rodas patinavam e comando angular baixo não girava a máquina.
O `CLAUDE.md` lista isso por escrito entre os "cuidados herdados que NÃO valem
aqui".

Os tetos deste robô estão em `movimentacao.yaml` e são `wz_max: 1.0` e
`v_max: 0.5`. O teleop fica dentro deles: **mão humana não é o lugar de
estrear velocidade que a autonomia não usa**. Ficou `0,30 / 0,50` linear e
`0,80 / 1,00` angular (normal/turbo).

### 3.3 Os botões

`LB = 6` e `RB = 7`, contra `L1 = 4` e `R1 = 5` do PS4 — medido no robô 1 por
ioctl `JSIOCGBTNMAP`. Com os números do PS4 o homem-morto cai num botão que
ninguém aperta e o robô não anda. **Estes ainda vão ser reconferidos neste
controle** (§7); é para isso que o `scripts/js_mapping.py` veio junto.

## 4. O que nenhum número no YAML conserta, e o operador vai sentir

Girar **parado** neste robô não é proporcional. A placa entrega um módulo de
giro só — 2,204 rad/s medidos — e segura a saída cheia por ~0,52 s depois do
corte, varrendo 93–101° (decisão 023, e a observação do dono no DIARIO de
14-08: *"o pivô dele tá forte demais, ele gira com um simples toque"*).

```
robô ANDANDO   o arco preserva a razão entre as rodas   ->  proporcional
robô PARADO    um toque lateral no analógico            ->  quantum de ~95°
```

➡️ Quem dirigir vai ler isso como "o `scale_angular` está alto". **Não está.**
Baixar o número não resolve o giro parado (a placa ignora o teto) e piora o
giro em movimento, que funciona. Está escrito no cabeçalho do
`teleop_xbox.yaml` para ninguém gastar sessão perseguindo o sintoma.

## 5. O compensador de rumo pega o joystick de graça — mas só metade dele

Decisão do dono nesta sessão, textual: *"se eu estiver mandando ele ir reto eu
quero o filtro de ir reto sim"*.

E ele já vem: o `compensador_rumo` fica **depois** do mux e atende "QUALQUER
comandante — bancada, teleop, seguidor, Nav2". Só que ele tem duas metades, e
na pilha só uma está ligada:

| metade | o que faz | com o joystick |
|---|---|---|
| feedforward (`curv_frente`) | cancela o arco do corpo (círculo de 1,22 m) | ✅ liga sempre |
| malha PI (`segura_rumo`) | segura o rumo capturado contra deriva residual | ❌ `False` na pilha |

O `segura_rumo: False` de `pilha.launch.py` existe por um motivo que é **só da
autonomia**: com o `heading_controller` acima, os dois disputam a direção e o
de cima tem de pivotar para desfazer — o sintoma de 05-08 (*"parece que ele
está sem o PID, tá pendendo pra direita"*).

⚠️ Quando o humano dirige, esse motivo não se aplica. Mas `segura_rumo` é
parâmetro do **nó**, não da fonte: não dá para ligar só para o joystick sem
ligar para a pilha inteira e trazer a briga de volta. Fazê-lo por fonte exigiria
o compensador saber quem venceu o mux, e o `twist_mux` 4.5 não anuncia isso em
tópico nenhum — seria nó novo, não config.

**Escolha do dono, com as três opções na mesa: fica como está.** O feedforward
é o grosso, e se sobrar deriva no campo a gente mede o quanto é antes de
escrever código. Registrado aqui para ninguém reabrir sem número na mão.

## 6. Duas decisões de entorno

**Launch separada, não dentro da `pilha.launch.py`.** O controle é do operador,
não da missão. A pilha sobe e desce por causa de mapa, localização e Nav2; o
freio de mão tem de sobreviver a isso e, principalmente, tem de poder subir
sozinho com a pilha morta, para tirar o robô de um lugar ruim. Amarrar o freio
ao ciclo de vida daquilo de que ele é o freio seria o erro.

**Sem `bin/robot-pair-xbox`.** O robô 1 tem esse wrapper e ele depende de mDNS
(`robo-desktop.local`) e do usuário `robo`. Aqui o usuário é `bara`, o IP muda
a cada sessão e o mDNS não resolve entre redes (ESTADO 14-08). Um wrapper que
assume o contrário seria mentira; o `pair-xbox.sh` roda no NUC e o cabeçalho
dele traz a linha de `ssh` pronta.

**Achado de brinde:** `scripts/setup_headless.sh` fazia `source` do
`scripts/_bluez_fixes.sh`, que **não existia neste repo** — veio no clone sem o
arquivo junto. Com `set -e` no topo, isso derrubava o setup headless inteiro.
O port do Bluetooth conserta.

## 7. O que NÃO está verificado — e é o que falta

Nada disto passou por robô. No dev: 340 testes verdes, build limpo, a
auto-detecção de device exercida, e os 8 defeitos que o teste novo deveria
pegar conferidos por mutação.

Protocolo de campo, com o robô **LIGADO**:

1. `./pair-xbox.sh` no NUC — sucesso é `/dev/input/jsN` aparecer, não o BlueZ
   dizer "Connected";
2. `./scripts/js_mapping.py /dev/input/jsN` — **confirmar LB=6 e RB=7**. Se
   divergir, o config muda antes de qualquer corrida;
3. `bash bin/sobe-robo` — o passo 4/5 tem de imprimir o `js` escolhido e a taxa
   do `/joy`;
4. homem-morto: com o LB **solto**, mexer o analógico e confirmar robô parado;
5. só então dirigir.

E uma medida que vale registrar enquanto se dirige, porque responde a §5 sem
código novo: empurrar reto ~3 m e ler o desvio lateral no CSV. Se o
feedforward sozinho segurar, a conversa do `segura_rumo` está encerrada.
