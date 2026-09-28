# Procedência — reprodutor do SIGSEGV do `collision_monitor` (decisão 057)

Cópia fiel de `~/repro-cm-sigsegv/20260928_153237/` (PC de dev, 28-09-2026),
feita depois da recuperação; os dois manifestos conferem na origem e na cópia.
Robô, lidar e Gazebo desligados em toda a execução.

## Execução

- Comando: `bash tools/repro_cm_sigsegv/roda.sh 1`, no commit **`e346af3`**
  (árvore limpa — `ambiente.txt`). Domínio 53, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`,
  só `/opt/ros/jazzy` (sem overlay), Nav2 **1.3.12 inalterado** (sha256 do
  binário e da `libcollision_monitor_core.so` em `ambiente.txt`).
- **Tentativa 1 = VERMELHO válido, classificado MANUALMENTE.** O
  `resultado.txt` automático diz `NAO_VERMELHO` por três defeitos do
  instrumento (abaixo); ele fica como foi gerado, não foi editado.
- **Nenhuma tentativa 2 ou 3** foi rodada.

## Os três defeitos do instrumento (`roda.sh` em `e346af3`)

1. `thread_principal_na_espera_de_tf=0`: a regex pegou a linha
   `Thread 1 "collision_monit" received signal SIGINT` em vez do bloco da
   thread 1 do `thread apply all bt`.
2. `sigsegv=0`: a linha `received signal SIGSEGV` sai antes do marcador
   `PARADA 2`, e a busca começou depois dele.
3. "algo vivo com SHM sobrando": o `wc -l` de `$(vivos | wc -l)` é filho do
   script e carrega `REPRO_CM_MARCA`; a varredura acusou o próprio script
   (`vivos_antes_do_shm_clean.txt` está vazio). Por isso a tentativa parou
   antes da limpeza.

## Classificação manual (`tentativa_01/gdb.txt`, `cm.log`, `aux.log`, `sigint.txt`)

| critério | evidência |
|---|---|
| callback na espera de TF antes do SIGINT | parada no SIGINT, thread 1: `nanosleep ← tf2_ros::Buffer::canTransform ← lookupTransform ← nav2_util::getTransform ← Source::getTransform ← PointCloud::getData ← CollisionMonitor::process ← cmdVelInCallbackStamped ← execute_subscription ← SingleThreadedExecutor::spin` |
| nenhum erro de TF antes do SIGINT | `tf_erro_antes_do_sigint.txt` vazio |
| mesmo SIGSEGV | `#0 get_subscription_count()+22`, `#1 process` (retorno `…cf9b`), `#2 cmdVelInCallbackStamped` — mesmos deslocamentos do core de 14h11 (bateria `141047`) |
| `this` inválido | `rdi = 0x10` |

Linha do tempo (relógio de parede, `aux.log`/`sigint.txt`/`cm.log`):
TF para 364,995 → `TwistStamped` 365,496 → SIGINT 367,622 → preshutdown,
`Deactivating`, `Cleaning up` 367,687 → erro de TF 367,696 (depois do
cleanup, quando o `rcl_shutdown` torna `rclcpp::ok()` falso) → SIGSEGV.

Desvios deliberados do perfil real (`tools/repro_cm_sigsegv/perfil.yaml`):
`transform_tolerance` 30 s (real 0,5 s — a parada do gdb comeria a janela),
sem lifecycle manager e sem bond (`bond_heartbeat_period: 0`), tópicos
`/repro/*`. Mudam o tamanho da janela, não o caminho do código.

## Recuperação do SHM (`tentativa_01/recuperacao/`, manifesto `recuperacao.SHA256SUMS`)

Executada **depois**, por autorização do dono, com `shm_recupera.py`
corrigido em **`e75a13b`** (reconferência interna de processos, das duas
marcas e de donos por `maps`/`fd`, só `/proc`, sem ROS).

Atribuição dos **12** arquivos Fast DDS deixados pela tentativa:

| quem removeu | arquivos | evidência |
|---|---|---|
| `ros2 node list --no-daemon` (consulta **intrusiva**: o participante novo limpa portas zumbis com `_el`) | **6**: `fastrtps_port20661`, `_el`, `sem.…20661_mutex`, `fastrtps_port7000`, `_el`, `sem.…7000_mutex` | `02_*`, `03_diff_pos_consulta.txt`, `05_registro_consulta_intrusiva.md` |
| `fastdds shm clean` (código 0, "1 zombie segments cleaned") | **2**: `fastrtps_c4feb5a20bf45379`, `_el` | `06_*`, `07_*`, `08_*` |
| `shm_recupera.py remove` (`e75a13b`), candidatos exatos `shm_depois − shm_antes`, gate interno | **4**: `fastrtps_port20663`, `fastrtps_port7001`, `sem.…20663_mutex`, `sem.…7001_mutex` | `09_*`, `10_*`, `11_*` |

Verificação final (`13_verificacao_final.txt`): 0 segmento Fast DDS, resto
do `/dev/shm` igual ao de antes da tentativa, `confere` sem processo, marca
ou dono, 0 marca. `ros2 node list` não foi executado na verificação.

## Integridade

- `20260928_153237/SHA256SUMS` — gerado pelo `roda.sh` ao fim da execução
  (16 arquivos; `console.txt` fora por projeto).
- `20260928_153237/tentativa_01/recuperacao.SHA256SUMS` — 26 arquivos da
  recuperação, gerado depois da verificação final.
- `console.txt` (fora dos dois): sha256
  `3a8003d9ffbe41310412d6af35287d16ebd284c07563a75903e18887165c5e59`.
- O Apport não gerou relatório novo (o `/var/crash` segue só com os de
  13h56 e 14h11).
