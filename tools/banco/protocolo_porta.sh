#!/usr/bin/env bash
# Protocolo da porta: N corridas com CONDIÇÃO INICIAL IDÊNTICA.
#
# Por que pilha nova a cada corrida, e não um goal atrás do outro: o robô
# termina onde parou, e a geometria de aproximação da porta é o que está sob
# teste. Duas corridas seguidas sem reset medem coisas diferentes — foi assim
# que 14-08 quase concluiu que "às vezes passa" quando na verdade a condição
# inicial é que mudava.
#
#   bash tools/banco/protocolo_porta.sh 5 docs/dados/AAAA-MM-DD-nome
# ⚠️ sem `set -u`: o `install/setup.bash` do colcon lê COLCON_TRACE sem
# default e morre com "unbound variable". Custou a 1ª rodada deste protocolo.
set -e
set +u
N="${1:-5}"
SAIDA="${2:-docs/dados/2026-08-14-porta-gazebo/protocolo}"
EXTRA="${3:-}"        # argumentos extras de launch (para A/B)
R="/home/luiz-santarosa/Workspace/Controle_robo_livox"
S="/tmp/claude-1000/-home-luiz-santarosa-Workspace-Controle-robo-livox/818d5d95-96b2-4997-bdd6-5ecc16c16121/scratchpad"
ALVO_X=6.14
ALVO_Y=3.78
TETO=90

mkdir -p "$R/$SAIDA"
cd "$R" || exit 1

mata() {
  ps -eo pid,args | grep -E "Controle_robo_livox|gz sim|ros2 bag|ros2 launch|/opt/ros/jazzy/lib" \
    | grep -v grep | awk '{print $1}' | sort -u | xargs -r kill -9
  sleep 5
}

for i in $(seq 1 "$N"); do
  echo "=== corrida $i/$N ==="
  mata
  source "$R/install/setup.bash"
  export ROS_DOMAIN_ID=42
  ros2 launch robot_motion pilha.launch.py sim:=true gui:=false rviz:=false \
      mundo:="$R/worlds/sala_andar3.sdf" \
      mapa:="$R/maps/sala_andar3/sala_andar3.yaml" \
      localizacao:=amcl pose_x:=0.0 pose_y:=0.0 pose_yaw:=0.0 \
      log_dir:="$R/$SAIDA/logs_$i" $EXTRA \
      > "$S/protocolo_$i.log" 2>&1 &

  # espera ativar (ou desistir)
  for _ in $(seq 1 60); do
    grep -q "Managed nodes are active" "$S/protocolo_$i.log" && break
    grep -q "Aborting bringup" "$S/protocolo_$i.log" && break
    sleep 3
  done
  if ! grep -q "Managed nodes are active" "$S/protocolo_$i.log"; then
    echo "corrida $i: BRINGUP FALHOU, pulando"
    continue
  fi
  sleep 5   # deixa o AMCL assentar antes de mandar objetivo

  timeout $((TETO + 40)) python3 tools/banco/corrida_nav.py \
      --alvo $ALVO_X $ALVO_Y --teto-s $TETO \
      --csv "$R/$SAIDA/corrida_$i.csv" > "$SAIDA/veredito_$i.txt" 2>&1
  echo "--- veredito $i ---"
  tail -8 "$SAIDA/veredito_$i.txt"
done

mata
echo "=== protocolo terminado: $N corridas em $SAIDA ==="
