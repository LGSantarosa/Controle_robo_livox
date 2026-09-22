#!/usr/bin/env bash
# Sentido bridge -> MEGA: um WheelSpeeds conhecido tem de virar um
# FT_SET_SPEED exato. Encerramento normal (vai + fim) para os arquivos
# fecharem. Etapa 5, passo 5.
set +u
R=/home/rbe-luis/Workspace/Controle_robo_livox
SAIDA="$HOME/validacao_etapa5/$(date +%Y%m%d_%H%M%S)_diag-bridge"
mkdir -p "$SAIDA"; cp "$0" "$SAIDA/"; cp "$R/tools/valida_etapa5/fases.yaml" "$SAIDA/"
exec > >(tee -a "$SAIDA/console.txt") 2>&1
export ROS_DOMAIN_ID=48 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST VALIDA_ETAPA4_MARCA="$SAIDA"
cd "$R" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
{ echo "commit: $(git rev-parse HEAD)"; git status --porcelain; } > "$SAIDA/git.txt"
d="$SAIDA/rodada"; mkdir -p "$d"
setsid python3 tools/valida_etapa5/mega_fingida5.py "$d" padrao_hoje > "$d/mega.log" 2>&1 < /dev/null &
MEGA=$!
for i in $(seq 1 100); do [ -s "$d/pty.txt" ] && break; sleep 0.1; done
PTY="$(cat "$d/pty.txt")"; echo "pty=$PTY mega(pgid)=$MEGA"
setsid ros2 run robot_nav mega_bridge --ros-args -r __node:=mega_bridge \
    -p port:="$PTY" -p baud:=230400 > "$d/bridge.log" 2>&1 < /dev/null &
BRIDGE=$!
sleep 5
echo "== um WheelSpeeds conhecido: left=-120 right=-120  (esperado steer=0 speed=-120)"
timeout 10 ros2 topic pub --once --no-daemon /wheel_vel_setpoints wheel_msgs/msg/WheelSpeeds \
    '{left_wheel: -120.0, right_wheel: -120.0}' > "$d/pub.txt" 2>&1; echo "   pub rc=$?"
sleep 1
echo "== encerrando normalmente (vai + fim), para os arquivos fecharem"
touch "$d/vai"
for i in $(seq 1 900); do [ -s "$d/mega.yaml" ] && break; sleep 0.1; done
touch "$d/fim"
for i in $(seq 1 200); do kill -0 "$MEGA" 2>/dev/null || break; sleep 0.1; done
kill -INT -- -"$BRIDGE" 2>/dev/null; sleep 2; kill -KILL -- -"$BRIDGE" 2>/dev/null
echo "== frames FT_SET_SPEED recebidos pela MEGA fingida:"
awk -F, 'NR>1 && $3=="0x01" {print "   t="$1" fase="$2" hex="$5" steer="$6" speed="$7}' "$d/frames.csv" | head -5
echo "   total de frames: $(( $(wc -l < "$d/frames.csv") - 1 ))  | bytes.bin: $(stat -c%s "$d/bytes.bin") bytes"
python3 tools/valida_etapa5/processos.py sinaliza_marcados INT > /dev/null 2>&1; sleep 2
python3 tools/valida_etapa5/processos.py marcados && echo "0 marcados" || echo "SOBRARAM"
echo "pasta: $SAIDA"
