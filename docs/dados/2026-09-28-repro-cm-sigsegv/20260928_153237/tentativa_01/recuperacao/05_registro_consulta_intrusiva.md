# Registro: a consulta de nós removeu 6 arquivos do /dev/shm

Registrado em 2026-09-28T15:47:27-03:00. Decisão do dono: `ros2 node list` deixa
de ser gate de limpeza; aqui vale só como observação intrusiva.

- Comando: `env -u REPRO_CM_MARCA ROS_DOMAIN_ID=53 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ros2 node list --no-daemon --spin-time 5`
  (15:42:45 → 15:42:51), código 0, saída vazia (zero nós), stderr vazio (`02_*`).
- Inventário antes: `01_inventario.tsv` (idêntico ao `shm_depois.tsv` da tentativa).
  Inventário depois: `03_inventario_pos_consulta.tsv`. Diferença: `03_diff_pos_consulta.txt`.
- **Removidos pela consulta (6)**, nenhum arquivo novo:
  - fastrtps_port20661
  - fastrtps_port20661_el
  - fastrtps_port7000
  - fastrtps_port7000_el
  - sem.fastrtps_port20661_mutex
  - sem.fastrtps_port7000_mutex
- São exatamente as portas com trava `_el` (e seus mutex): o participante Fast DDS
  novo limpa portas zumbis ao iniciar. Não cria nó nem daemon observável
  (`pgrep` sem daemon depois).
- Consequência: esses 6 não são atribuíveis ao `fastdds shm clean` nem à remoção
  manual. A recuperação não fica invalidada: antes/depois existem e nada novo apareceu.
