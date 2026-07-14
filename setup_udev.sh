#!/bin/bash
# Configura nomes USB estáveis via IDENTIDADE do dispositivo (serial / VID:PID).
# 2026-07-01 (robô 1): era por PORTA FÍSICA (KERNELS) e invertia quando os
# cabos trocavam de porta USB. Agora casa a MEGA pelo SERIAL (imune a troca
# de porta; fallback pra porta física só se faltar serial).
# Uso: sudo ./setup_udev.sh
#
# Identifica:
#   /dev/mega   — Arduino MEGA 2560 (ponte pra placa de hoverboard)
# (O Livox Mid-360 é Ethernet — não passa por udev serial.)

set -e

case "${1:-}" in
    --help|-h)
        sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
esac

if [ "$EUID" -ne 0 ]; then
    echo "ERRO: Execute com sudo: sudo ./setup_udev.sh"
    exit 1
fi

RULES_FILE="/etc/udev/rules.d/99-robot-usb.rules"

get_devpath() {
    # Resolve a porta USB FÍSICA (ex: "1-7" ou "1-2.3") andando pela árvore
    # de pais em /sys. Funciona pra ttyUSB* (FTDI/CH340) e ttyACM*
    # (CDC-ACM, Arduino genuíno) sem depender de awk avançado.
    local dev="$1"
    local devname
    devname=$(basename "$dev")
    local syspath
    syspath=$(readlink -f "/sys/class/tty/$devname/device" 2>/dev/null)
    while [ -n "$syspath" ] && [ "$syspath" != "/" ]; do
        local base
        base=$(basename "$syspath")
        if [[ "$base" =~ ^[0-9]+-[0-9]+(\.[0-9]+)*$ ]]; then
            echo "$base"
            return 0
        fi
        syspath=$(dirname "$syspath")
    done
}

get_vidpid() {
    local dev="$1"
    local vid pid
    vid=$(udevadm info "$dev" 2>/dev/null | awk -F= '/ID_VENDOR_ID/{print $2}')
    pid=$(udevadm info "$dev" 2>/dev/null | awk -F= '/ID_MODEL_ID/{print $2}')
    echo "${vid}:${pid}"
}

get_serial() {
    # Serial USB estável (ID_SERIAL_SHORT). Arduino genuíno tem; adaptadores
    # baratos (PL2303/CH340 genéricos) frequentemente NÃO — nesse caso volta vazio
    # e o chamador cai no fallback por porta física.
    local dev="$1"
    udevadm info "$dev" 2>/dev/null | awk -F= '/ID_SERIAL_SHORT/{print $2}'
}

# Emite uma regra udev pra um dispositivo, preferindo a IDENTIDADE estável:
#   serial presente        -> ATTRS{idVendor}+{idProduct}+{serial}  (imune a porta)
#   sem serial, com VID:PID -> ATTRS{idVendor}+{idProduct}          (imune a porta)
#   sem nada                -> KERNELS (porta física, frágil — último recurso)
emit_rule() {
    local name="$1" vidpid="$2" serial="$3" path="$4"
    local vid="${vidpid%%:*}" pid="${vidpid##*:}"
    if [ -n "$serial" ] && [ -n "$vid" ] && [ -n "$pid" ]; then
        echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", ATTRS{serial}==\"$serial\", SYMLINK+=\"$name\", MODE=\"0660\", GROUP=\"dialout\""
    elif [ -n "$vid" ] && [ -n "$pid" ]; then
        echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"$vid\", ATTRS{idProduct}==\"$pid\", SYMLINK+=\"$name\", MODE=\"0660\", GROUP=\"dialout\""
    else
        echo "SUBSYSTEM==\"tty\", KERNELS==\"$path\", SYMLINK+=\"$name\", MODE=\"0660\", GROUP=\"dialout\""
    fi
}

list_tty_ports() {
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || true
}

settle_udev() {
    udevadm settle --timeout=5 >/dev/null 2>&1 || true
}

echo "========================================================"
echo "  Configuração de porta USB fixa — MEGA"
echo "========================================================"
echo ""

# Daqui pra baixo o script faz I/O com hardware imprevisível (udev, USB).
# Desligo o 'set -e' pra não morrer silencioso em substituição de comando.
set +e

# ---- Identificar porta da MEGA (obrigatória) ----
echo "Arduino MEGA 2560 (obrigatória)"
echo "  Plugue a MEGA agora."
read -r -p "  Pressione ENTER quando estiver pronto... " _DUMMY

echo "  Aguardando 1s e estabilizando eventos udev..."
sleep 1
settle_udev

PORTS_ALL=$(list_tty_ports)
if [ -z "$PORTS_ALL" ]; then
    echo "ERRO: Nenhum /dev/ttyUSB* ou /dev/ttyACM* encontrado. Verifique a conexão da MEGA."
    exit 1
fi

# Lista portas candidatas. Se houver mais de uma (modem 3G/4G, debugger,
# etc.) pergunta interativamente em vez de pegar a primeira.
CAND_MEGA=$(echo "$PORTS_ALL" | xargs)  # trim
CAND_COUNT=$(echo "$CAND_MEGA" | wc -w)

if [ "$CAND_COUNT" -eq 0 ]; then
    echo "ERRO: Não detectei nenhuma porta serial. A MEGA está plugada?"
    exit 1
elif [ "$CAND_COUNT" -eq 1 ]; then
    MEGA_PORT="$CAND_MEGA"
else
    echo "  Múltiplas portas candidatas detectadas:"
    for p in $CAND_MEGA; do
        vidpid=$(get_vidpid "$p")
        path=$(get_devpath "$p")
        echo "    $p  [VID:PID=$vidpid  USB path=$path]"
    done
    read -r -p "  Qual é a porta da MEGA? (ex: /dev/ttyACM0): " MEGA_PORT
fi

MEGA_PATH=$(get_devpath "$MEGA_PORT")
MEGA_VIDPID=$(get_vidpid "$MEGA_PORT")
MEGA_SERIAL=$(get_serial "$MEGA_PORT")

echo "  MEGA identificada: $MEGA_PORT → VID:PID=$MEGA_VIDPID serial=${MEGA_SERIAL:-<nenhum>} path=$MEGA_PATH"
echo ""

# Sem o USB path a regra udev vira `KERNELS==""` (casa com nada, ou pior,
# casa com tudo dependendo do kernel). Aborta antes de gravar lixo em /etc.
if [ -z "$MEGA_PATH" ]; then
    echo "ERRO: não consegui extrair o USB path da MEGA ($MEGA_PORT). Aborto."
    exit 1
fi
# Reativa set -e pra escrita do arquivo / udevadm reload (essas devem dar certo).
set -e

# ---- Cria as regras udev ----
echo "Criando $RULES_FILE ..."

{
    cat << EOF
# Regras udev para nomes estáveis — Arduino MEGA.
# Casa por IDENTIDADE do dispositivo (serial / VID:PID) — imune a troca de
# porta USB. Só cai pra porta física (KERNELS) se faltar serial E VID:PID.
# Gerado por setup_udev.sh em $(date).
#
# Para regenerar: sudo ~/Workspace/Controle_robo_web/setup_udev.sh

# Arduino MEGA 2560 — ponte 2 placas hoverboard + sensores
# VID:PID=$MEGA_VIDPID serial=${MEGA_SERIAL:-<nenhum>} (porta física era $MEGA_PATH)
$(emit_rule mega "$MEGA_VIDPID" "$MEGA_SERIAL" "$MEGA_PATH")
EOF

} > "$RULES_FILE"

echo ""
echo "Arquivo criado:"
cat "$RULES_FILE"

echo ""
echo "Recarregando regras udev..."
udevadm control --reload-rules
udevadm trigger
sleep 1

echo ""
echo "=== Verificando symlinks ==="
EXPECTED_LINKS="/dev/mega"
if ! ls -la $EXPECTED_LINKS 2>/dev/null; then
    echo "AVISO: Symlinks não apareceram ainda — desplugue e replugue os dispositivos."
fi

echo ""
echo "=== Pronto! ==="
echo ""
echo "IMPORTANTE: symlinks agora casam por SERIAL/VID:PID — imunes a troca de"
echo "porta USB. Só rode de novo se TROCAR a placa MEGA."
echo ""
echo "Próximo passo: ./launch.sh (rebuild incremental automático)."
echo "Pra recompilar tudo do zero (raro), apague o install/ e rode ./setup_pi.sh ou ./setup.sh."
