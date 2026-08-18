# shellcheck shell=bash
# Library de fixes do BlueZ para o controle ficar pareado e conectado no Linux
# sem tela. Sourceada por `pair-xbox.sh`.
#
# Portada do robô 1 (`scripts/_bluez_fixes.sh`, commit ecd6e70) com UMA mudança
# de valor: aqui o default do `ControllerMode` é **dual**, não `bredr`.
#
#   No robô 1 o default era `bredr` porque o controle do dia a dia era um PS4
#   (Bluetooth clássico), e o `pair-xbox.sh` de lá pedia `dual` por override.
#   Os dois brigavam pela mesma linha do `main.conf`: rodar o pareamento do PS4
#   depois do Xbox quebrava o Xbox, e o GUIA_RAPIDO de lá avisa isso.
#
#   Neste robô NÃO EXISTE PS4 — nunca houve, e a pilha não tem canal para ele.
#   Então a briga não existe, e o valor certo é o único que o Xbox Series
#   aceita. Herdar o `bredr` só para "ficar igual ao robô 1" seria copiar a
#   restrição sem copiar o motivo dela.
#
# As 4 pegadinhas cobertas aqui:
#   1) ERTM: alguns controles não falam ERTM; desligar via modprobe option.
#      O Xbox Series não sofre disso, mas o fix é inofensivo e persistente.
#   2) ControllerMode = dual: o Series é BLE (HID over GATT). Em `bredr` a
#      stack LE fica desligada e o controle NEM APARECE no scan.
#   3) AutoEnable = true: o adaptador acorda "Powered: yes" no boot. O NUC é
#      headless — ninguém vai clicar em nada para ligar o Bluetooth.
#   4) ClassicBondedOnly = false: sem isso o bluetoothd recusa HID de device
#      sem bonding completo — conecta em BT e o /dev/input/jsN nunca aparece.
#
# Uso típico:
#     source "$REPO/scripts/_bluez_fixes.sh"
#     bluez_apply_persistent_fixes
#     [ "$BLUEZ_NEED_RESTART" = 1 ] && sudo systemctl restart bluetooth
#     bluez_unblock_rfkill

# Aplica os 4 fixes persistentes (arquivos em /etc).
# Sets:
#   BLUEZ_NEED_RESTART=1   se algum arquivo em /etc/bluetooth/ mudou
#   BLUEZ_NEED_REBOOT=1    se o config persistente do ERTM foi escrito agora
bluez_apply_persistent_fixes() {
    BLUEZ_NEED_RESTART=0
    BLUEZ_NEED_REBOOT=0

    # Fix 1: ERTM via /etc/modprobe.d/.
    local ertm_conf=/etc/modprobe.d/bluetooth-disable-ertm.conf
    if [ ! -f "$ertm_conf" ]; then
        echo "  → Desligando ERTM no kernel (persistente em $ertm_conf)"
        echo 'options bluetooth disable_ertm=Y' | sudo tee "$ertm_conf" >/dev/null
        BLUEZ_NEED_REBOOT=1
    fi

    # Fix 2 + 3: /etc/bluetooth/main.conf (ControllerMode, AutoEnable).
    local mode="${BLUEZ_CONTROLLER_MODE:-dual}"
    local main_conf=/etc/bluetooth/main.conf
    if [ -f "$main_conf" ]; then
        if ! grep -qE "^[[:space:]]*ControllerMode[[:space:]]*=[[:space:]]*${mode}\$" "$main_conf"; then
            echo "  → Forçando ControllerMode = $mode em $main_conf"
            sudo cp "$main_conf" "$main_conf.bak.bluez_fixes"
            if grep -qE '^[[:space:]]*#?[[:space:]]*ControllerMode' "$main_conf"; then
                sudo sed -i "s|^[[:space:]]*#\?[[:space:]]*ControllerMode[[:space:]]*=.*|ControllerMode = ${mode}|" "$main_conf"
            elif grep -q '^\[General\]' "$main_conf"; then
                sudo sed -i "/^\[General\]/a ControllerMode = ${mode}" "$main_conf"
            else
                printf '\n[General]\nControllerMode = %s\n' "$mode" | sudo tee -a "$main_conf" >/dev/null
            fi
            BLUEZ_NEED_RESTART=1
        fi

        if ! grep -qE '^[[:space:]]*AutoEnable[[:space:]]*=[[:space:]]*true' "$main_conf"; then
            echo "  → Habilitando AutoEnable = true em $main_conf"
            if grep -qE '^[[:space:]]*#?[[:space:]]*AutoEnable' "$main_conf"; then
                sudo sed -i 's|^[[:space:]]*#\?[[:space:]]*AutoEnable[[:space:]]*=.*|AutoEnable = true|' "$main_conf"
            elif grep -q '^\[Policy\]' "$main_conf"; then
                sudo sed -i '/^\[Policy\]/a AutoEnable = true' "$main_conf"
            else
                printf '\n[Policy]\nAutoEnable = true\n' | sudo tee -a "$main_conf" >/dev/null
            fi
            BLUEZ_NEED_RESTART=1
        fi
    fi

    # Fix 4: ClassicBondedOnly=false em /etc/bluetooth/input.conf.
    local input_conf=/etc/bluetooth/input.conf
    if [ -f "$input_conf" ] && ! grep -qE '^[[:space:]]*ClassicBondedOnly[[:space:]]*=[[:space:]]*false' "$input_conf"; then
        echo "  → Setando ClassicBondedOnly = false em $input_conf"
        if grep -qE '^[[:space:]]*#?[[:space:]]*ClassicBondedOnly' "$input_conf"; then
            sudo sed -i 's|^[[:space:]]*#\?[[:space:]]*ClassicBondedOnly[[:space:]]*=.*|ClassicBondedOnly = false|' "$input_conf"
        elif grep -q '^\[General\]' "$input_conf"; then
            sudo sed -i '/^\[General\]/a ClassicBondedOnly = false' "$input_conf"
        else
            printf '\n[General]\nClassicBondedOnly = false\n' | sudo tee -a "$input_conf" >/dev/null
        fi
        BLUEZ_NEED_RESTART=1
    fi
}

# Tenta aplicar ERTM em runtime via sysfs. Retorna 0 se ficou em Y.
bluez_apply_ertm_runtime() {
    local cur
    cur="$(cat /sys/module/bluetooth/parameters/disable_ertm 2>/dev/null || echo N)"
    if [ "$cur" = "Y" ]; then return 0; fi
    echo Y | sudo tee /sys/module/bluetooth/parameters/disable_ertm >/dev/null 2>&1 || true
    cur="$(cat /sys/module/bluetooth/parameters/disable_ertm 2>/dev/null || echo N)"
    [ "$cur" = "Y" ]
}

# Destrava bluetooth no rfkill (runtime) e persiste via systemd-rfkill.
bluez_unblock_rfkill() {
    if rfkill list bluetooth 2>/dev/null | grep -q "Soft blocked: yes"; then
        echo "  → Destravando bluetooth no rfkill"
        sudo rfkill unblock bluetooth
    fi
    sudo systemctl enable systemd-rfkill.service >/dev/null 2>&1 || true
}
