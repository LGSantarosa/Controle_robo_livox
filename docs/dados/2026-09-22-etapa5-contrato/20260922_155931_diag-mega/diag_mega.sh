#!/usr/bin/env bash
# Diagnóstico: MEGA fingida + mega_bridge sozinhos (etapa 5, passo 5 reprovado
# em 20260922_154816). Sem o resto da pilha. Três partidas frias.
set +u
R=/home/rbe-luis/Workspace/Controle_robo_livox
CARIMBO="$(date +%Y%m%d_%H%M%S)"
SAIDA="$HOME/validacao_etapa5/${CARIMBO}_diag-mega"
mkdir -p "$SAIDA"; cp "$0" "$SAIDA/diag_mega.sh"
exec > >(tee -a "$SAIDA/console.txt") 2>&1
export ROS_DOMAIN_ID=47 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export VALIDA_ETAPA4_MARCA="$SAIDA"
cd "$R" || exit 1
source /opt/ros/jazzy/setup.bash; source install/setup.bash
{ echo "commit: $(git rev-parse HEAD)"; git status --porcelain; } > "$SAIDA/git.txt"

# Cópia instrumentada: conta os writes de STATE e NÃO engole o erro.
python3 - "$SAIDA" <<'PY'
import sys
src = open('tools/valida_etapa5/mega_fingida5.py').read()
old = """    def estado():
        q = frame_state()
        while not parar.is_set():
            try:
                os.write(mestre, q)
            except OSError:
                return
            time.sleep(0.02)"""
new = """    def estado():
        q = frame_state()
        n = 0
        diag = open(os.path.join(pasta, 'diag_state.txt'), 'w', buffering=1)
        diag.write(f'frame STATE = {q.hex()} ({len(q)} bytes)\\n')
        while not parar.is_set():
            try:
                os.write(mestre, q)
                n += 1
                if n in (1, 10, 50, 250, 1000):
                    diag.write(f'{agora():.3f}s writes={n}\\n')
            except OSError as e:
                import traceback
                diag.write(f'{agora():.3f}s OSError depois de {n} writes: {e!r}\\n')
                traceback.print_exc(file=diag)
                return
            time.sleep(0.02)
        diag.write(f'{agora():.3f}s fim, writes={n}\\n')"""
assert old in src
open(sys.argv[1] + '/mega_instrumentada.py', 'w').write(src.replace(old, new))
PY
chmod +x "$SAIDA/mega_instrumentada.py"
cp tools/valida_etapa5/fases.yaml "$SAIDA/"   # a cópia lê o fases.yaml ao lado dela

for n in 1 2 3; do
  d="$SAIDA/partida_$n"; mkdir -p "$d"
  echo "===== partida fria $n ====="
  setsid python3 "$SAIDA/mega_instrumentada.py" "$d" padrao_hoje > "$d/mega.log" 2>&1 < /dev/null &
  MEGA=$!
  for i in $(seq 1 100); do [ -s "$d/pty.txt" ] && break; sleep 0.1; done
  PTY="$(cat "$d/pty.txt")"; echo "pty=$PTY mega_pid=$MEGA"
  setsid ros2 run robot_nav mega_bridge --ros-args -r __node:=mega_bridge \
      -p port:="$PTY" -p baud:=230400 > "$d/bridge.log" 2>&1 < /dev/null &
  BRIDGE=$!
  sleep 5
  echo "--- /battery/front (mensagem inteira):"
  timeout 8 ros2 topic echo /battery/front --once > "$d/battery.txt" 2>&1; echo "rc=$?"
  sed 's/^/   /' "$d/battery.txt"
  echo "--- taxas (MEGA -> bridge):"
  timeout 12 ros2 topic hz /hoverboard/wheel_velocities --window 30 > "$d/hz_wheels.txt" 2>&1
  grep -m1 "average rate" "$d/hz_wheels.txt" | sed 's/^/   wheel_velocities /'
  timeout 12 ros2 topic hz /battery/front --window 5 > "$d/hz_battery.txt" 2>&1
  grep -m1 "average rate" "$d/hz_battery.txt" | sed 's/^/   battery          /'
  echo "--- um WheelSpeeds conhecido (bridge -> MEGA):"
  timeout 8 ros2 topic pub --once /wheel_vel_setpoints wheel_msgs/msg/WheelSpeeds \
      '{left_wheel: -120.0, right_wheel: -120.0}' > "$d/pub.txt" 2>&1; echo "rc=$?"
  sleep 1
  touch "$d/fim"; sleep 2
  kill -INT -- -"$MEGA" 2>/dev/null; kill -INT -- -"$BRIDGE" 2>/dev/null; sleep 2
  kill -KILL -- -"$MEGA" 2>/dev/null; kill -KILL -- -"$BRIDGE" 2>/dev/null; sleep 1
  echo "--- frames recebidos pela MEGA fingida (bridge -> MEGA):"
  awk -F, 'NR>1 {print "   ", $1, $3, $6, $7}' "$d/frames.csv" 2>/dev/null | head -3
  echo "   linhas em frames.csv: $(( $(wc -l < "$d/frames.csv" 2>/dev/null || echo 1) - 1 ))"
  echo "--- diag da thread de STATE:"; sed 's/^/   /' "$d/diag_state.txt" 2>/dev/null | head -8
done
python3 tools/valida_etapa5/processos.py marcados && echo "0 processos marcados" || echo "SOBRARAM marcados"
echo "pasta: $SAIDA"
