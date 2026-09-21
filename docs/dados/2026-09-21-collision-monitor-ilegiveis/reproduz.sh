#!/usr/bin/env bash
# Reprodução isolada dos ilegíveis do collision_monitor (21-09). Sem Gazebo,
# sem robô: domínio ROS próprio, só localhost. Rodar da raiz do repo.
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=217 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
setsid ros2 run nav2_collision_monitor collision_monitor --ros-args \
  --params-file ros2_packages/robot_motion/config/collision_monitor.yaml > /dev/null 2>&1 &
PID=$!
sleep 3
ros2 lifecycle set /collision_monitor configure
ros2 lifecycle set /collision_monitor activate
ros2 lifecycle get /collision_monitor
echo "--- ros2 param dump (o CLI tem o mesmo defeito):"
timeout 20 ros2 param dump /collision_monitor
python3 - <<'PY'
import rclpy
from rclpy.parameter_client import AsyncParameterClient
rclpy.init(); n = rclpy.create_node('_reproducao')
cli = AsyncParameterClient(n, '/collision_monitor'); cli.wait_for_services(3.0)
def espera(f):
    rclpy.spin_until_future_complete(n, f, timeout_sec=3.0); return f.result()
nomes = sorted(espera(cli.list_parameters(depth=None)).result.names)
lote = espera(cli.get_parameters(nomes))
print(f'--- list_parameters: {len(nomes)} nomes; get_parameters em lote: {len(lote.values)} valores')
for x in nomes:
    um = espera(cli.get_parameters([x]))
    vals = list(um.values) if um else []
    if len(vals) != 1 or vals[0].type == 0:
        print(f'ILEGIVEL /collision_monitor:{x} -> {len(vals)} valor(es)' +
              (f', tipo {vals[0].type}' if vals else ''))
PY
kill -INT -- -$PID 2>/dev/null || kill -INT $PID
