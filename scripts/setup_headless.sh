#!/bin/bash
# Tooling de operação headless (acessar/operar o robô de outro PC).
# Chamado por setup.sh e setup_pi.sh. PLANO_HEADLESS_2026-05-22 §4.
# Idempotente — pode rodar de novo sem problema.
#
# Deixa a máquina pronta pra ser TANTO o robô (anuncia <hostname>.local, aceita
# SSH, roda a stack no tmux) QUANTO o "outro PC" (resolve *.local, robot-connect).
#
# Espera REPO_DIR no ambiente; senão deduz do caminho deste script.
set -e

REPO_DIR="${REPO_DIR:-$(cd "$(dirname "$0")/.." && pwd)}"

echo "=== Headless: SSH + mDNS (<hostname>.local) + tmux + bluez ==="
# openssh-server : aceita SSH de outro PC (lado robô).
# avahi-daemon   : anuncia este host como <hostname>.local na LAN (lado robô).
# libnss-mdns    : resolve *.local sem caçar IP (lado cliente; o postinst já
#                  insere o mdns no /etc/nsswitch.conf).
# tmux           : a stack sobrevive à queda do SSH (detach/attach).
# bluez          : pareamento do controle Xbox Series X|S (pair-xbox.sh).
# rfkill         : destravar bluetooth bloqueado por software.
# joystick       : utilitário jstest pra validar o controle.
sudo apt install -y openssh-server avahi-daemon libnss-mdns tmux bluez rfkill joystick
sudo systemctl enable --now ssh avahi-daemon bluetooth

# Atalhos no PATH. Symlinks pro repo: editar bin/ reflete na hora, sem reinstalar.
# robot-up/robot-key servem ao robô; robot-connect serve ao outro PC;
# idempotente instalar nos dois lados (cada lado só usa os relevantes).
#
# ⚠️ Não há `robot-pair-xbox` aqui, ao contrário do robô 1: o pareamento roda
# NO NUC (`pair-xbox.sh`), porque o wrapper de lá depende de mDNS e neste robô
# `robo-desktop.local` não resolve (ESTADO_PROJETO 14-08).
for tool in robot-up robot-key robot-connect; do
    src="$REPO_DIR/bin/$tool"
    dst="/usr/local/bin/$tool"
    [ -f "$src" ] || continue
    if [ -e "$dst" ] && [ ! -L "$dst" ]; then
        echo "  AVISO: $dst é arquivo real (não symlink), pulando — apague-o se quer atualizar."
        continue
    fi
    chmod +x "$src"
    sudo ln -sf "$src" "$dst"
    echo "  $dst -> $src"
done

# --- Fixes do BlueZ pro controle funcionar ---
# Sem isso o controle pareia mas o connect cai em loop (Connected: yes → no),
# ou /dev/input/jsN nunca materializa. Helpers compartilhados com pair-xbox.sh.
#
# ⚠️ Até 2026-08-18 este `source` apontava para um arquivo que NÃO EXISTIA
# neste repo — veio no clone do robô 1 sem o `_bluez_fixes.sh` junto, e com
# `set -e` no topo isso derrubava o setup_headless inteiro aqui. O arquivo
# entrou junto com o suporte ao Xbox.
# shellcheck source=_bluez_fixes.sh
source "$REPO_DIR/scripts/_bluez_fixes.sh"
bluez_apply_persistent_fixes
if [ "$BLUEZ_NEED_RESTART" = 1 ]; then
    sudo systemctl restart bluetooth
fi
bluez_unblock_rfkill

HOSTNAME_SHORT="$(hostname)"
echo
echo "  Pronto. Este host se anuncia como '${HOSTNAME_SHORT}.local'."
echo "  Do OUTRO PC (com este mesmo setup):  robot-connect <modo>"
echo "  Como robô: pareie o controle Xbox com  ./pair-xbox.sh"
