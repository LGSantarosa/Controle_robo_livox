#!/usr/bin/env bash
# Reprodutor mínimo do SIGSEGV do collision_monitor no teardown (decisão 057,
# adendo de 28-09 noite). Sem Gazebo, sem pilha, sem overlay do workspace:
# só /opt/ros/jazzy (Nav2 1.3.12 inalterado) + tools/repro_cm_sigsegv/auxiliar.py.
#
#   bash tools/repro_cm_sigsegv/roda.sh [N]      N tentativas (1..3, padrão 1)
#
# Cada tentativa: varredura (nada vivo, nenhuma marca, zero SHM Fast DDS) →
# collision_monitor sob gdb (para no SIGINT e no SIGSEGV) → auxiliar → espera o
# CMD_PUBLICADO + 2 s → confere que nenhum erro de TF saiu → SIGINT por PID no
# collision_monitor → gdb registra as threads no SIGINT, entrega o sinal e
# registra o SIGSEGV → SIGINT no auxiliar → inventário e limpeza de SHM.
# Qualquer resíduo não recuperado interrompe tudo. Pasta: ~/repro-cm-sigsegv/<carimbo>/.
#
# VERMELHO = (1) no SIGINT, a thread principal está em tf2_ros::Buffer::canTransform
# dentro de CollisionMonitor::process, (2) nenhum "Failed to get" antes do
# SIGINT, (3) SIGSEGV com #0 get_subscription_count, #1 process,
# #2 cmdVelInCallbackStamped e rdi = 0x10.
set +u

if [ -z "$REPRO_AMBIENTE_LIMPO" ]; then
  exec env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH \
           -u PYTHONPATH -u LD_LIBRARY_PATH -u ROS_DISTRO -u VALIDA_ETAPA4_MARCA \
           REPRO_AMBIENTE_LIMPO=1 bash "$0" "$@"
fi

N="${1:-1}"
[[ "$N" =~ ^[1-3]$ ]] || { echo "uso: bash tools/repro_cm_sigsegv/roda.sh [N: 1..3]"; exit 2; }

R="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
D="$R/tools/repro_cm_sigsegv"
RS="$R/tools/subidas_robo3/shm_recupera.py"
CM=/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor
SHM=/dev/shm
DOMINIO=53
BASE="$HOME/repro-cm-sigsegv/$(date +%Y%m%d_%H%M%S)"
[ -e "$BASE" ] && { echo "🔴 $BASE já existe"; exit 1; }
mkdir -p "$BASE" || exit 1
exec > >(tee -a "$BASE/console.txt") 2>&1
# O tee fica fora de qualquer sinal: ninguém sinaliza o grupo deste script.

source /opt/ros/jazzy/setup.bash || { echo "🔴 source /opt/ros/jazzy falhou"; exit 1; }
export ROS_DOMAIN_ID="$DOMINIO"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export REPRO_CM_MARCA="$BASE"
export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] [{time}] [{name}]: {message}'
export RCUTILS_LOGGING_BUFFERED_STREAM=0
export PYTHONUNBUFFERED=1

{ echo "data: $(date --iso-8601=seconds)"
  echo "git: $(git -C "$R" rev-parse HEAD) $(git -C "$R" status --porcelain | wc -l) alteração(ões)"
  dpkg-query -W ros-jazzy-nav2-collision-monitor ros-jazzy-nav2-util ros-jazzy-rclcpp ros-jazzy-tf2-ros ros-jazzy-bondcpp
  sha256sum "$CM" /opt/ros/jazzy/lib/libcollision_monitor_core.so
  echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
  echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE"
} > "$BASE/ambiente.txt"
case "$AMENT_PREFIX_PATH" in *"$R"*) echo "🔴 overlay do workspace no ambiente"; exit 1 ;; esac

conta_shm() { ls "$SHM" 2>/dev/null | grep -ciE 'fastrtps|fast_datasharing'; }
vivos() {   # lista processos ROS/Gazebo ou com marca (exceto este script)
  local p cmd
  for p in /proc/[0-9]*; do
    [ "${p#/proc/}" = "$$" ] && continue
    cmd="$(tr '\0' ' ' < "$p/cmdline" 2>/dev/null)" || continue
    [ -z "$cmd" ] && continue
    if grep -qaE 'REPRO_CM_MARCA|VALIDA_ETAPA4_MARCA' "$p/environ" 2>/dev/null \
       || [[ "$cmd" =~ /opt/ros/|ros2|gz\ sim|gzserver|ruby.*gz ]]; then
      case "$cmd" in *tee\ -a*|*roda.sh*) continue ;; esac
      echo "${p#/proc/} $cmd"
    fi
  done
}
varre() {   # varre <arquivo> — 0 se limpo
  vivos > "$1"; echo "shm_fastdds=$(conta_shm)" >> "$1"
  [ "$(wc -l < "$1")" -eq 1 ] && [ "$(conta_shm)" -eq 0 ]
}
espera_linha() {   # espera_linha <arquivo> <regex> <segundos>
  local fim=$((SECONDS + $3))
  while [ $SECONDS -lt $fim ]; do grep -qE "$2" "$1" 2>/dev/null && return 0; sleep 0.2; done
  return 1
}
espera_morte() {   # espera_morte <pid> <segundos>
  local fim=$((SECONDS + $2))
  while [ $SECONDS -lt $fim ]; do kill -0 "$1" 2>/dev/null || return 0; sleep 0.2; done
  return 1
}

INTERROMPIDA=0
tentativa() {
  local i="$1" S="$BASE/tentativa_0$1" gdbpid cmpid auxpid motivo="" cls
  mkdir -p "$S"; echo; echo "== tentativa $i/$N =="
  varre "$S/varredura_antes.txt" || { echo "   🔴 resíduo antes (varredura_antes.txt)"; return 1; }
  python3 "$RS" inventaria "$SHM" "$S/shm_antes.tsv" || return 1

  cat > "$S/gdb.cmd" <<EOF
set pagination off
set confirm off
set debuginfod enabled off
set print thread-events off
handle SIGINT stop print pass
run --ros-args -r __node:=collision_monitor --params-file $D/perfil.yaml > $S/cm.out 2> $S/cm.log
echo \n=== PARADA 1 (esperada: SIGINT) ===\n
info program
thread apply all bt 16
echo \n=== CONTINUA: sinal entregue ===\n
continue
echo \n=== PARADA 2 (esperada: SIGSEGV) ===\n
info program
bt 16
info registers rdi rip
thread apply all bt 8
continue
echo \n=== FIM ===\n
EOF
  ROS_LOG_DIR="$S/roslog" setsid gdb -batch -nx -x "$S/gdb.cmd" "$CM" > "$S/gdb.txt" 2>&1 &
  gdbpid=$!
  echo "   gdb pid=$gdbpid"
  if ! espera_linha "$S/cm.log" 'Activating' 30; then
    motivo="collision_monitor não ativou em 30 s"
  else
    sleep 0.5
    cmpid="$(pgrep -P "$gdbpid" -x collision_monit | head -1)"
    [ -z "$cmpid" ] && cmpid="$(pgrep -P "$gdbpid" | head -1)"
    echo "   collision_monitor pid=$cmpid"
    ROS_LOG_DIR="$S/roslog" setsid python3 "$D/auxiliar.py" > "$S/aux.log" 2>&1 &
    auxpid=$!
    echo "   auxiliar pid=$auxpid"
    if ! espera_linha "$S/aux.log" '^CMD_PUBLICADO' 20; then
      motivo="auxiliar não publicou o cmd_vel em 20 s"
    else
      sleep 2
      grep -n 'Failed to get' "$S/cm.log" > "$S/tf_erro_antes_do_sigint.txt"
      echo "sigint wall=$(date +%s.%N)" > "$S/sigint.txt"
      kill -INT "$cmpid" && echo "   SIGINT → $cmpid"
    fi
  fi
  espera_morte "$gdbpid" 60 || { echo "   🔴 gdb vivo depois de 60 s"; motivo="${motivo:-gdb não terminou}"; }
  if [ -n "$auxpid" ]; then
    kill -INT "$auxpid" 2>/dev/null
    espera_morte "$auxpid" 15 || { echo "   🔴 auxiliar não saiu com SIGINT"; motivo="${motivo:-auxiliar não saiu}"; }
  fi
  if [ -n "$motivo" ]; then
    echo "   🔴 $motivo — nada mais é sinalizado; parar e trazer ao dono"; return 1
  fi

  # ── classificação ──
  python3 - "$S" > "$S/resultado.txt" <<'PY'
import re, sys, pathlib
S = pathlib.Path(sys.argv[1])
g = (S / 'gdb.txt').read_text(errors='replace')
p1, _, resto = g.partition('=== CONTINUA')
p2 = resto.partition('=== PARADA 2')[2]
sigint = 'received signal SIGINT' in p1
main_t = re.search(r'Thread 1 .*?(?=\nThread \d|\Z)', p1, re.S)
main_t = main_t.group(0) if main_t else ''
espera = 'canTransform' in main_t and 'CollisionMonitor::process' in main_t
tf_antes = (S / 'tf_erro_antes_do_sigint.txt').read_text().strip() == ''
segv = 'received signal SIGSEGV' in p2
bt = p2.partition('info registers')[0]
quadros = ('get_subscription_count' in bt and 'CollisionMonitor::process' in bt
           and 'cmdVelInCallbackStamped' in bt)
rdi = re.search(r'^rdi\s+(0x[0-9a-f]+)', p2, re.M)
rdi = rdi.group(1) if rdi else '-'
v = sigint and espera and tf_antes and segv and quadros and rdi == '0x10'
print(f'parou_no_sigint={int(sigint)}')
print(f'thread_principal_na_espera_de_tf={int(espera)}')
print(f'sem_erro_de_tf_antes_do_sigint={int(tf_antes)}')
print(f'sigsegv={int(segv)}')
print(f'quadros_iguais={int(quadros)}')
print(f'rdi={rdi}')
print('classificacao=' + ('VERMELHO' if v else 'NAO_VERMELHO'))
PY
  cls="$(grep '^classificacao=' "$S/resultado.txt" | cut -d= -f2)"
  sed 's/^/   /' "$S/resultado.txt"
  grep -nE '\] \[[0-9.]+\] \[collision_monitor\]: (Running Nav2|Deactivating|Cleaning up|Shutting down|Destroying)|Failed to get' "$S/cm.log" \
    | sed 's/^/   log: /' | cut -c1-150

  # ── SHM: inventário e limpeza ──
  local orf; orf="$(conta_shm)"
  echo "   shm fastdds depois: $orf"
  if [ "$orf" -gt 0 ]; then
    ls -la --time-style=full-iso "$SHM" | grep -iE 'fastrtps|fast_datasharing' > "$S/shm_inventario.txt"
    python3 "$RS" inventaria "$SHM" "$S/shm_depois.tsv" || { echo "   🔴 inventário falhou"; return 1; }
    if [ "$(vivos | wc -l)" -ne 0 ]; then
      vivos > "$S/vivos_antes_do_shm_clean.txt"; echo "   🔴 algo vivo com SHM sobrando"; return 1
    fi
    fastdds shm clean > "$S/shm_clean.txt" 2>&1
    if [ "$(conta_shm)" -gt 0 ]; then
      ls -la --time-style=full-iso "$SHM" >> "$S/shm_clean.txt"
      if ! python3 "$RS" remove "$SHM" "$S/shm_antes.tsv" "$S/shm_depois.tsv" > "$S/shm_remocao_manual.txt" 2>&1 \
         || [ "$(conta_shm)" -gt 0 ]; then
        echo "   🔴 resíduo de SHM não recuperado (shm_remocao_manual.txt)"; return 1
      fi
      echo "   🟠 remoção manual controlada: $(grep -c '^  ' "$S/shm_remocao_manual.txt") arquivo(s)"
    else
      echo "   🟠 fastdds shm clean zerou"
    fi
  fi
  varre "$S/varredura_depois.txt" || { echo "   🔴 resíduo depois (varredura_depois.txt)"; return 1; }
  echo "   ✔ limpo depois — $cls"
}

echo "reprodutor collision_monitor — $BASE — N=$N, domínio $DOMINIO"
for i in $(seq 1 "$N"); do
  tentativa "$i" || { INTERROMPIDA=1; break; }
done
( cd "$BASE" && find . -type f ! -name console.txt ! -name SHA256SUMS | sort | xargs sha256sum > SHA256SUMS )
if [ "$INTERROMPIDA" -ne 0 ]; then echo "🔴 INTERROMPIDO — $BASE"; exit 1; fi
echo "✔ $N tentativa(s) — $BASE"; grep -h '^classificacao=' "$BASE"/tentativa_*/resultado.txt
