#!/bin/bash
# Pareamento automatizado do controle Xbox Series X|S no NUC, sem tela.
#
# Portado do robô 1 (`pair-xbox.sh`, commit ecd6e70), onde a receita foi
# MEDIDA numa Raspberry Pi em 2026-08-11. O protocolo é do BlueZ e do
# controle, não da máquina, então ele atravessa — o que muda é o entorno,
# e está anotado no fim deste cabeçalho.
#
# ─── AS DUAS PEGADINHAS QUE DEFINEM ESTE SCRIPT
#
#   A) O Series é BLE (HID over GATT), não Bluetooth clássico. Com
#      `ControllerMode = bredr` a stack LE fica desligada e o controle NEM
#      APARECE no scan. Aqui usamos `dual` (BR/EDR + LE juntos), que também é
#      o default do `scripts/_bluez_fixes.sh` deste repo — não há PS4 neste
#      robô para brigar pela linha, ao contrário do robô 1.
#
#   B) O agent tem que ser NoInputNoOutput. Com `KeyboardDisplay` (que o PS4
#      do robô 1 exigia) o BlueZ pede MITM que o controle não sabe fazer, e o
#      pareamento morre com uma linha só:
#          hci0 ... auth failed with status 0x05 (Authentication Failed)
#      E o agent PRECISA sobreviver ao pair: com heredoc ou pipe simples o
#      bluetoothctl sai antes da autenticação e o bluetoothd loga
#          src/device.c:new_auth() No agent available for request type 2
#      Daí o FIFO.
#
#   E não existe `hid_playstation` aqui: o caminho é HID over GATT →
#   hid_generic → joydev. Modprobe dos dois antes do pair, senão conecta em BT
#   e /dev/input/jsN nunca materializa.
#
# ─── COMO RODAR NESTE ROBÔ
#
# Ele roda NO NUC. O usuário é `bara`, o IP MUDA a cada sessão e
# `robo-desktop.local` NÃO resolve (ESTADO_PROJETO 14-08) — por isso este repo
# não tem o wrapper `robot-pair-xbox` que o robô 1 tem: um script que assume
# mDNS aqui seria mentira. Do seu PC:
#
#     ssh -t bara@<ip-do-nuc> "sudo -v && cd ~/Controle_robo_livox && \
#         bash pair-xbox.sh --no-prompt"
#
# Ou, já logado no NUC:
#
#     cd ~/Controle_robo_livox && ./pair-xbox.sh
#
# ⚠️ O `sudo -v` e o script precisam do MESMO PTY — o cache do sudo é por
# terminal. Separar em dois `ssh` faz o script pedir senha e travar.
#
# Uso:
#   ./pair-xbox.sh                # interativo (pede Enters)
#   ./pair-xbox.sh --no-prompt    # non-interactive (via SSH); sudo cacheado
#
# Modo pareamento do Xbox Series X|S:
#   - Ligue o controle no botão Xbox (fica aceso fixo).
#   - Segure o botão PAIR (o pequeno na aresta de cima, ao lado do USB-C)
#     por ~3-5 s, até o botão Xbox piscar RÁPIDO.
#   - Solte; continua piscando rápido sozinho.

set -e

# ------------------------------------------------------------------
# Args
# ------------------------------------------------------------------
NO_PROMPT=0
for arg in "$@"; do
    case "$arg" in
        --no-prompt) NO_PROMPT=1 ;;
        -h|--help)
            sed -n '2,55p' "$0" | sed 's/^# \?//'
            exit 0 ;;
    esac
done

_read_or_skip() {
    if [ "$NO_PROMPT" = 1 ]; then
        return
    fi
    read -r -p "$1" _
}

if [ "$NO_PROMPT" = 1 ] && ! sudo -n true 2>/dev/null; then
    echo "ERRO: --no-prompt exige sudo cacheado. Rode 'sudo -v' antes (ou via SSH:"
    echo "      ssh <user>@<host> sudo -v)."
    exit 1
fi

# ------------------------------------------------------------------
# 0) Pré-checagens & fixes de sistema (idempotente)
# ------------------------------------------------------------------

if ! command -v bluetoothctl >/dev/null 2>&1; then
    echo "ERRO: bluetoothctl não encontrado. Instale com: sudo apt install -y bluez"
    exit 1
fi

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# `dual` (BR/EDR + LE) é o DEFAULT do _bluez_fixes.sh deste repo — ver o
# cabeçalho de lá. Fica explícito aqui assim mesmo: o dia em que alguém mudar
# o default, este script tem de continuar pedindo o que o Series precisa.
export BLUEZ_CONTROLLER_MODE=dual
# shellcheck source=scripts/_bluez_fixes.sh
source "$SELF_DIR/scripts/_bluez_fixes.sh"
bluez_apply_persistent_fixes

# ERTM: o Xbox não precisa disso (é vício de DualShock), mas desligar não
# atrapalha e o fix já é persistente na máquina. Só avisa se o kernel recusar.
bluez_apply_ertm_runtime || echo "  (aviso: ERTM segue ligado; inofensivo pro Xbox)"

if [ "$BLUEZ_NEED_RESTART" = 1 ]; then
    echo "→ Reiniciando bluetoothd pra aplicar ControllerMode=dual..."
    sudo systemctl restart bluetooth
    sleep 3
fi

bluez_unblock_rfkill
sleep 1

if ! bluetoothctl show 2>/dev/null | grep -q "Powered: yes"; then
    echo "→ Ligando adaptador BT..."
    bluetoothctl power on >/dev/null 2>&1 || true
    sleep 1
fi

if ! bluetoothctl show 2>/dev/null | grep -q "Powered: yes"; then
    echo "ERRO: adaptador BT continua desligado. Diagnóstico:"
    rfkill list bluetooth
    bluetoothctl show | head -10
    exit 1
fi

# Confere que o modo realmente ficou dual — se o bluetoothd ignorou, o scan
# LE não roda e o resto do script falharia com "não apareceu no scan".
if ! grep -qE '^[[:space:]]*ControllerMode[[:space:]]*=[[:space:]]*dual' /etc/bluetooth/main.conf 2>/dev/null; then
    echo "ERRO: ControllerMode não ficou 'dual' em /etc/bluetooth/main.conf."
    echo "      O Xbox Series é BLE e não aparece no scan sem isso."
    exit 1
fi

# HID over GATT → hid_generic → joydev.
for mod in hid_generic joydev; do
    if ! lsmod | grep -q "^${mod}"; then
        sudo modprobe "$mod" 2>/dev/null || true
    fi
done

echo "✓ Sistema pronto (ControllerMode=dual, rfkill OK, adapter ON, joydev OK)"
echo

# Snapshot dos joysticks que JÁ existem: o novo device pode nascer js1, não
# js0 — o índice do joydev depende da ORDEM de conexão, não do controle. Sem
# este snapshot o script declararia sucesso por causa de um device velho.
JS_BEFORE="$(ls /dev/input/js* 2>/dev/null | sort | tr '\n' ' ')"
[ -n "$JS_BEFORE" ] && echo "  (joysticks já presentes: $JS_BEFORE)"

# ------------------------------------------------------------------
# 1) Modo pareamento
# ------------------------------------------------------------------

cat <<'EOF'
=== Pareamento do controle Xbox Series X|S ===

Coloque o controle em modo pareamento AGORA:
  1) Ligue no botão Xbox (luz acesa fixa).
  2) Segure o botão PAIR (aresta de cima, ao lado do USB-C) por ~3-5s,
     até o botão Xbox piscar RÁPIDO.
  3) Solte — deve continuar piscando rápido sozinho.

EOF
_read_or_skip "Pronto? Enter pra começar... "

# ------------------------------------------------------------------
# 2) Limpa pareamento velho do MESMO controle (se houver)
# ------------------------------------------------------------------
# Casa pelo nome anunciado. Só remove o que se chama "Xbox Wireless
# Controller" — teclado, mouse ou fone pareados no NUC não são tocados.
OLD_MAC="$(bluetoothctl devices 2>/dev/null | awk '/Xbox Wireless Controller/{print $2; exit}')"
if [ -n "$OLD_MAC" ]; then
    echo "→ Removendo pareamento antigo do Xbox ($OLD_MAC)..."
    bluetoothctl remove "$OLD_MAC" >/dev/null 2>&1 || true
    sleep 1
fi

# ------------------------------------------------------------------
# 3) bluetoothctl vivo em background com agent NoInputNoOutput
# ------------------------------------------------------------------
# NoInputNoOutput, e não KeyboardDisplay. O Xbox Series faz emparelhamento BLE
# "Just Works"; com KeyboardDisplay o BlueZ pede MITM que o controle não sabe
# fazer — resultado medido no robô 1 em 2026-08-11:
#     hci0 ... auth failed with status 0x05 (Authentication Failed)
# NoInputNoOutput é o que casa com a capability real do controle.
#
# E o agent PRECISA continuar vivo até o fim do pair: com heredoc ou pipe
# simples o bluetoothctl sai antes da autenticação e o bluetoothd loga
#     src/device.c:new_auth() No agent available for request type 2
# — daí o FIFO.
AGENT_LOG=$(mktemp)
AGENT_FIFO=$(mktemp -u)
mkfifo "$AGENT_FIFO"
exec 3<>"$AGENT_FIFO"
bluetoothctl <&3 >"$AGENT_LOG" 2>&1 &
AGENT_PID=$!
trap 'echo quit >&3 2>/dev/null; sleep 1; kill $AGENT_PID 2>/dev/null; exec 3>&- 2>/dev/null; rm -f "$AGENT_FIFO" "$AGENT_LOG"' EXIT
sleep 1
echo "agent NoInputNoOutput" >&3
sleep 1
echo "default-agent" >&3
sleep 2

if ! grep -qE "Agent registered|Agent is already registered" "$AGENT_LOG"; then
    echo "ERRO: nenhum agent registrou. Sem agent vivo o pair falha com"
    echo "      'auth failed with status 0x05'. Log do agent:"
    tail -10 "$AGENT_LOG" | sed 's/^/    /'
    exit 1
fi

# ------------------------------------------------------------------
# 4) Scan até achar o Xbox
# ------------------------------------------------------------------
echo "→ Escaneando por até 30s (procurando 'Xbox Wireless Controller')..."
bluetoothctl --timeout 30 scan on >/dev/null 2>&1 &
SCAN_PID=$!

XBOX_MAC=""
for i in $(seq 1 30); do
    sleep 1
    XBOX_MAC=$(bluetoothctl devices 2>/dev/null | awk '/Xbox Wireless Controller/{print $2; exit}')
    if [ -n "$XBOX_MAC" ]; then
        echo "✓ Xbox encontrado: $XBOX_MAC"
        break
    fi
done

kill $SCAN_PID 2>/dev/null || true
wait $SCAN_PID 2>/dev/null || true
bluetoothctl scan off >/dev/null 2>&1 || true

if [ -z "$XBOX_MAC" ]; then
    cat <<EOF
ERRO: Xbox não apareceu no scan em 30s.

Diagnóstico:
  - O botão Xbox está piscando RÁPIDO agora? Se parou de piscar, ele saiu do
    modo pareamento: segure PAIR por 3-5s de novo e rode outra vez.
  - Console/PC/celular que já pareou com esse controle ligado por perto? Ele
    prefere o último host — desligue ou afaste.
  - Controle com firmware antigo: alguns Series só falam BT depois de
    atualizar pelo app Xbox Accessories (num PC/console Windows).
  - Confirme que o scan LE está de fato ativo:
      grep ControllerMode /etc/bluetooth/main.conf     # deve ser 'dual'
EOF
    exit 1
fi

# ------------------------------------------------------------------
# 5) Pair (polling de estado, não confia no retorno)
# ------------------------------------------------------------------
echo "→ Pairing... (até 30s, espera Bonded=yes — não só Paired)"
timeout 25 bluetoothctl pair "$XBOX_MAC" 2>&1 | tail -5 || true

BONDED=0
for i in $(seq 1 30); do
    if bluetoothctl info "$XBOX_MAC" 2>/dev/null | grep -q "Bonded: yes"; then
        BONDED=1
        break
    fi
    sleep 1
done

if [ "$BONDED" != 1 ]; then
    echo "ERRO: bonding não completou em 30s. Última info:"
    bluetoothctl info "$XBOX_MAC" 2>/dev/null | grep -E "Paired|Bonded|Trusted|Connected" || true
    echo
    echo "  Agent log:"
    tail -10 "$AGENT_LOG" | sed 's/^/    /'
    exit 1
fi
echo "✓ Paired + Bonded"

# ------------------------------------------------------------------
# 6) Trust
# ------------------------------------------------------------------
bluetoothctl trust "$XBOX_MAC" >/dev/null 2>&1
echo "✓ Trusted"

# ------------------------------------------------------------------
# 7) Connect
# ------------------------------------------------------------------
echo "→ Aguardando conexão (até 25s)..."
CONNECTED=0
for i in $(seq 1 25); do
    if bluetoothctl info "$XBOX_MAC" 2>/dev/null | grep -q "Connected: yes"; then
        CONNECTED=1
        break
    fi
    if [ "$i" = 6 ]; then
        echo "  ... auto-connect não disparou, forçando connect..."
        timeout 10 bluetoothctl connect "$XBOX_MAC" >/dev/null 2>&1 || true
    fi
    sleep 1
done

if [ "$CONNECTED" != 1 ]; then
    echo "ERRO: controle não conectou em 25s. Diagnóstico:"
    bluetoothctl info "$XBOX_MAC" 2>/dev/null | grep -E "Paired|Trusted|Connected|Blocked" || true
    echo
    echo "Últimas linhas do bluetoothd:"
    sudo journalctl -u bluetooth --since "30 seconds ago" --no-pager | tail -15
    exit 1
fi
echo "✓ Connected"

# ------------------------------------------------------------------
# 8) Espera um joystick NOVO materializar (HID over GATT → joydev)
# ------------------------------------------------------------------
echo "→ Aguardando /dev/input/jsN novo (até 30s; disconnect+reconnect aos 15s)..."
JS_NEW=""
JS_RETRY_DONE=0
for i in $(seq 1 30); do
    for js in $(ls /dev/input/js* 2>/dev/null | sort); do
        case " $JS_BEFORE " in
            *" $js "*) ;;                # já existia antes, ignora
            *) JS_NEW="$js" ;;
        esac
    done
    if [ -n "$JS_NEW" ]; then
        echo "  apareceu em ${i}s: $JS_NEW"
        break
    fi
    if [ "$i" = 15 ] && [ "$JS_RETRY_DONE" = 0 ]; then
        echo "  ... 15s sem joystick, forçando disconnect+reconnect pro HID subir..."
        bluetoothctl disconnect "$XBOX_MAC" >/dev/null 2>&1 || true
        sleep 2
        timeout 10 bluetoothctl connect "$XBOX_MAC" >/dev/null 2>&1 || true
        JS_RETRY_DONE=1
    fi
    sleep 1
done

if [ -z "$JS_NEW" ]; then
    cat <<EOF
AVISO: BT conectou (Connected=yes, Bonded=yes), mas nenhum /dev/input/jsN novo apareceu.

Estado no BlueZ:
$(bluetoothctl info "$XBOX_MAC" 2>/dev/null | grep -E "Paired|Bonded|Trusted|Connected|UUIDs" | sed 's/^/  /')

Modulos:
$(lsmod | grep -E "^hid_generic|^joydev|^hidp|^uhid" | sed 's/^/  /')

Logs do bluetoothd nos últimos 30s:
$(sudo journalctl -u bluetooth --since "30 seconds ago" --no-pager 2>&1 | grep -iE "input|hid|rejected|gatt" | tail -8 | sed 's/^/  /')

Causa mais comum aqui: o perfil HID over GATT não subiu. Tente desligar o
controle (segurar Xbox ~6s), ligar de novo e rodar o script outra vez.
EOF
    exit 1
fi

# ------------------------------------------------------------------
# 9) Sucesso
# ------------------------------------------------------------------
JS_INDEX="${JS_NEW##*/js}"

cat <<EOF

==========================================================
✓ Xbox pareado, conectado e visível como joystick.
==========================================================

Status:
$(bluetoothctl info "$XBOX_MAC" | grep -E "Name|Paired|Trusted|Connected" | sed 's/^/  /')

Dispositivo:
  $JS_NEW   (js${JS_INDEX})

⚠️ CONFIRA O MAPA DE BOTÕES ANTES DE CONFIAR NO HOMEM-MORTO.

  O número do botão muda entre controles e entre drivers — o mesmo LB pode
  ser 4 no xpad e 6 no HID genérico por Bluetooth. O
  \`config/teleop_xbox.yaml\` traz LB=6 / RB=7, que foi o MEDIDO no robô 1
  por Bluetooth. Se este controle discordar, o homem-morto cai num botão que
  ninguém aperta e o robô simplesmente não anda.

    ./scripts/js_mapping.py $JS_NEW

  Aperte LB e RB e leia os números. Se derem 6 e 7, o config serve como está.
  Se derem outros, corrija \`enable_button\` e \`enable_turbo_button\` em
  \`ros2_packages/robot_motion/config/teleop_xbox.yaml\` — e o teste
  \`test_joystick_coerente.py\` continua valendo.

Testes rápidos:
  ros2 run joy joy_enumerate_devices        # o ROS enxerga o joystick?
  ros2 topic echo /joy                      # com o joy_node no ar

Subir a pilha (o joystick sobe junto, passo 4/5):
  bash bin/sobe-robo

Dirigir: segure o LB e mexa o analógico esquerdo. RB = turbo.
Soltou o LB, ele para — é o homem-morto, e é de propósito.
EOF

if [ "$JS_INDEX" != "0" ]; then
    cat <<EOF

ℹ️  O controle nasceu em js${JS_INDEX}, não js0 — e aqui isso NÃO é problema.
    A \`joystick.launch.py\` deste repo detecta o device pelo NOME que o
    driver anuncia (ioctl JSIOCGNAME), não pelo índice, justamente porque a
    numeração do joydev depende da ordem de conexão no boot. Se algum dia a
    heurística errar, o override é \`ROBOT_JOY_DEVICE=${JS_INDEX}\` ou
    \`ros2 launch robot_motion joystick.launch.py dev:=${JS_INDEX}\`.
EOF
fi
