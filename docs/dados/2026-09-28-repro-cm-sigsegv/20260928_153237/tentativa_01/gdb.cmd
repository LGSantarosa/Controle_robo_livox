set pagination off
set confirm off
set debuginfod enabled off
set print thread-events off
handle SIGINT stop print pass
run --ros-args -r __node:=collision_monitor --params-file /home/rbe-luis/Workspace/Controle_robo_livox/tools/repro_cm_sigsegv/perfil.yaml > /home/rbe-luis/repro-cm-sigsegv/20260928_153237/tentativa_01/cm.out 2> /home/rbe-luis/repro-cm-sigsegv/20260928_153237/tentativa_01/cm.log
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
