# Calços do teste do `bin/sobe-robo3` — lido pelo bash NÃO interativo (BASH_ENV)
# antes do script. Só existe no teste; o script em produção nunca o vê.
#
# Função tem precedência sobre builtin e sobre o PATH, e vale dentro de
# `source`. O que isto fecha:
#   kill           → o calço de kill (lista de permitidos do teste)
#   pkill/killall  → violação, nada acontece
#   source ROS     → nada: o PATH falso não pode ser sobrescrito pelo
#                    /opt/ros/*/setup.bash, senão o `ros2` real entraria
#   sleep          → curto (as esperas do script são para hardware)
#
# O que isto NÃO fecha — `builtin kill`, `command kill`, `/bin/kill`… — é
# proibido no fonte por teste estático.

kill() { "$CALCO_BIN/kill" "$@"; }
pkill() { echo "pkill $*" >> "$CALCO/violacoes"; return 1; }
killall() { echo "killall $*" >> "$CALCO/violacoes"; return 1; }
source() {
  case "$1" in
    # Relativo também: o script faz `source install/setup.bash`. Um setup
    # que escapasse poria o /opt/ros na frente do PATH e o `ros2` REAL
    # entraria (aconteceu em 22-09: a pilha real subiu 56 vezes no dev).
    *setup.bash|*setup.sh|*local_setup.*|/opt/ros/*) return 0 ;;
  esac
  builtin source "$@"
}
sleep() { command sleep 0.05; }
