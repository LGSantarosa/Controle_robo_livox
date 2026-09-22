#!/usr/bin/env bash
# Confirmação isolada: o scan_2d cria EXATAMENTE um /transform_listener_impl_*?
# Etapa 4, passo 7, achado do cenário 2 (20260922_130525_robo3-gazebo).
# Domínio 45, só localhost, sem Gazebo; só scan_2d.launch.py. Três consultas
# ao grafo; esperado: /scan_2d + 1 ocorrência de ^/transform_listener_impl_[0-9a-f]+$.
set +u
R=/home/rbe-luis/Workspace/Controle_robo_livox
V="$R/tools/valida_etapa4"
CARIMBO="$(date +%Y%m%d_%H%M%S)"
SAIDA="$HOME/validacao_etapa4/${CARIMBO}_scan2d-isolado"
[ -e "$SAIDA" ] && exit 1
mkdir -p "$SAIDA"; cp "$0" "$SAIDA/confirma_scan2d.sh"
exec > >(tee -a "$SAIDA/console.txt") 2>&1
GRUPOS="$SAIDA/grupos_wrapper.txt"; : > "$GRUPOS"
RESULTADO="$SAIDA/resultado.txt"; : > "$RESULTADO"
export ROS_DOMAIN_ID=45 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
DOMINIOS_USADOS=45
source "$V/lib.sh"
{ echo "commit: $(git -C "$R" rev-parse HEAD)"; git -C "$R" status --porcelain; } > "$SAIDA/git.txt"
PRE="$(ps -eo pid,user,args | grep -E '(^|[ /])(ros2|gz|rviz2|_ros2_daemon)( |$)|/opt/ros/[a-z]+/lib/|/install/[a-z_]+/lib/|ros2cli\.daemon' | grep -vE 'grep -E|confirma_scan2d')"
[ -n "$PRE" ] && { echo "🔴 ROS de pé:"; echo "$PRE"; exit 1; }
export VALIDA_ETAPA4_MARCA="$SAIDA"
instala_limpeza
cd "$R" || exit 1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
printenv | grep -E '^(ROS|RMW|AMENT_PREFIX|VALIDA)' | sort > "$SAIDA/ambiente.txt"

sobe_grupo scan_2d "$SAIDA/launch.log" ros2 launch robot_base scan_2d.launch.py || exit 1
sleep 5
OK=1
for n in 1 2 3; do
  f="$SAIDA/consulta_$n.txt"
  nos_do_dominio 45 "$f"; rc=$?
  lis="$(grep -cE '^/transform_listener_impl_[0-9a-f]+$' "$f")"
  outros="$(grep -vxE '/scan_2d|/transform_listener_impl_[0-9a-f]+' "$f" | tr '\n' ' ')"
  scan="$(grep -cx '/scan_2d' "$f")"
  echo "consulta $n: código $rc, /scan_2d=$scan, listeners=$lis, outros=[${outros}]" | tee -a "$SAIDA/consultas.txt"
  [ "$rc" -eq 0 ] && [ "$scan" -eq 1 ] && [ "$lis" -eq 1 ] && [ -z "$outros" ] || OK=0
  sleep 2
done
[ "$OK" -eq 1 ] && anota "scan_2d isolado: /scan_2d + 1 transform_listener_impl, 3/3 consultas" APROVADO \
                || anota "scan_2d isolado: /scan_2d + 1 transform_listener_impl, 3/3 consultas" REPROVADO "(consultas.txt)"
limpa
cat "$RESULTADO"
grep -q " REPROVADO" "$RESULTADO" && exit 1
exit 0
