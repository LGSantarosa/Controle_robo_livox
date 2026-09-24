# Evidência — primeiro LIO completo do robô 3 parado

**Data:** 2026-09-24, laboratório.  
**Notebook:** `ubuntu-Latitude-3490`, repo em `fba6d39`.  
**Hardware:** placa e motores desligados; somente notebook, Ethernet e Mid-360.
O robô não recebeu comando e não se moveu.

## Configuração viva

- `enp1s0`: `192.168.1.5/24`;
- Mid-360: `192.168.1.169`, MAC `e4:7a:2c:90:1d:f1`;
- JSON ativo: cópia ignorada pelo git dentro do `livox_ros_driver2`, host `.5`
  e sensor `.169`;
- o JSON versionado `.2/.158` não foi alterado.

## Sequência

1. O driver isolado que sobrevivera à primeira pausa retomou a publicação:
   `/livox/lidar` 9,97 Hz e `/livox/imu` 200,1 Hz, um publicador de cada.
2. Ele foi encerrado com `SIGINT`.
3. Subiu `robot_base/localizacao.launch.py`, sem tração. FAST-LIO e a conversão
   de nuvem funcionaram; `tf_odom` recusou corretamente a árvore sem RSP.
4. Subiu somente `robot_state_publisher` com o `robo3.urdf.xacro`, sem serial,
   mux ou atuador. A TF fechou e `/scan` passou a publicar.
5. Uma assinatura própria mediu `/Odometry` e `/scan` por 15 s, com o robô
   parado. Saída integral em `metricas_parado.txt`.
6. Todos os processos foram encerrados por PGID, sem `SIGKILL`; processos e
   grafo ROS ficaram vazios antes de o dono desligar o lidar.

## Arquivos

| arquivo | origem |
|---|---|
| `driver_isolado.log` | `/tmp/livox.log`, driver que retomou após religar |
| `localizacao.log` | launch com Livox, FAST-LIO, `tf_odom`, ponte e `scan_2d` |
| `rsp.log` | RSP avulso, somente para fechar a árvore |
| `odom_hz.txt` | `ros2 topic hz /Odometry --window 30` |
| `pontos_hz.txt` | `ros2 topic hz /livox/pontos --window 30` |
| `scan_hz.txt` | tentativa **anterior ao RSP**; a ausência de scan é evidência da TF faltante |
| `metricas_parado.txt` | janela final de 15 s e carga dos processos |
| `SHA256SUMS` | integridade dos arquivos anteriores |

## Limites

- A pose do Livox usada pela TF é o valor provisório do URDF, não medida 6D.
- A janela de 15 s caracteriza deriva curta; não calibra o LIO.
- Ter 352–355 raios finitos não prova casamento com mapa nem visão correta
  junto ao corpo.
- RViz do launch upstream morreu no headless por Qt; os nós úteis continuaram.
- Mapa, AMCL, Nav2, percepção/reflexo e atuador não foram exercitados.

