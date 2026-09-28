# Sessão manual do robô 3 no Gazebo — 28-09-2026

Sessão pedida pelo dono para dirigir o robô pelo RViz e observar a pilha em um
percurso longo. Rodou no PC de desenvolvimento, com robô e lidar físicos
desligados, na branch `etapa6-pilha-robo3`, commit `71f53c4`, domínio ROS 50.
O objetivo foi clicado no RViz; não foi uma corrida dos validadores das
decisões 060 ou 061.

## Resultado

O objetivo enviado foi `(10,5034; 6,6586)`, yaw `1,48406 rad`, partindo de
`(2,000; 5,000)`. O Nav2 declarou `SUCCEEDED` em aproximadamente **57,1 s** de
parede. A trajetória verdadeira registrada em `/Odometry` mediu **13,524 m**.

No instante de `Reached the goal`, a pose era aproximadamente
`(10,383; 6,445)`, a **0,245 m** do alvo, dentro da tolerância viva de 0,25 m.
Depois da inércia/frenagem da planta, a pose assentou em aproximadamente
`(10,449; 6,568)`, a **0,105 m** do alvo. O dono observou que o robô passou por
todo o percurso, mas ficou "meio burrinho" ao entrar na segunda porta.

## O que aconteceu na segunda porta

A segunda passagem é o aperto de **0,80 m** de `tools/mundo/gera_pista.py`
(`x=8,9…9,1`, vão `y=3,2…4,0`). Não foi incapacidade do planner de encontrar
um caminho. A entrada ocorreu curvada e pouco alinhada; o envelope de parada
tem 0,48 m de largura e exige pelo menos dois pontos.

| parede | evento | pose verdadeira mais próxima | leitura |
|---:|---|---|---|
| 1790622029,797905 | `APPROACH:PolygonApproach` | `(8,850; 3,688)`, yaw ≈ `+0,145 rad` | aproximação reduzida |
| 1790622029,974314 | `DO_NOTHING` | — | aproximação liberada |
| 1790622030,506384 | `STOP:PolygonStop` | `(9,047; 3,675)`, yaw ≈ `−0,243 rad` | o seguidor pedia `+0,50 m/s`; `/auto_vel` foi zerado |
| 1790622035,788756 | `DO_NOTHING` | `(9,272; 3,574)`, yaw ≈ `−0,550 rad` | liberação depois de **5,282 s** |

A planta levou cerca de **1,4 s** para assentar depois do corte. O
`heading_controller` acusou robô parado com comando às 2032,572; o
`path_follower` iniciou escape reto às 2034,654, com frente livre de 2,69 m.
O reflexo liberou às 2035,789 e o escape terminou às 2036,010, depois de
0,21 m em 1,2 s. O robô então corrigiu o rumo e concluiu o objetivo.

O `/scan` gravado corrobora a proximidade: durante o `STOP` houve até dez
pontos dentro do polígono, concentrados no quadrante traseiro esquerdo do
robô. Isso é diagnóstico, não prova exata da entrada do monitor: o
`collision_monitor` usava `/livox/pontos`, que não foi incluído no bag.

Também houve um escape curto na primeira porta, às 2004,621–2005,572 (0,21 m
em 0,8 s, frente livre de 1,80 m), sem transição do `collision_monitor`.

## Evidência versionada

- `ros_logs/freeze_capture.csv`: cadeia de comandos, odometria, status da ação
  e as quatro transições do monitor;
- `ros_logs/freeze_diag.csv`: diagnóstico periódico de pose, obstáculo à
  frente e comando;
- `ros_logs/seguidor_2026-09-28_155821.csv`: 1 041 amostras do seguidor;
- `ros_logs/corrida_2026-09-28_155820/`: parâmetros materializados do Nav2 e
  do monitor;
- `launch.log` e `rviz.log`: bringup, objetivo, escapes, `STOP` e sucesso;
- `bag_info.txt`: inventário do bag bruto;
- `ORIGEM_SHA256SUMS`: hashes dos nove arquivos da pasta original.

O bag bruto permanece, pela convenção de `.gitignore`, fora do repositório:

`~/sessao-manual-robo3/20260928_160300/bag_movimento/bag_movimento_0.mcap`

SHA-256: `9f1534516aace588a469172325c640a6114d5267a2d9942140ba45c0b73d14c7`.
Ele tem 23,1 MiB, 188,923 s, 206 893 mensagens e 16 tópicos. O gravador
informou **uma mensagem perdida no transporte**, sem identificar o tópico.

## Limites da conclusão

- é Gazebo; não prova hardware nem a geometria real do robô 3;
- `/livox/pontos` não foi gravado, portanto o conjunto exato de pontos que
  disparou o polígono não pode ser reconstituído;
- o `launch.log` terminou quando o `tee` recebeu `Ctrl-C` e não contém o
  teardown. Ao final foram conferidos zero processo ROS/Gazebo, zero marca e
  zero segmento Fast DDS, mas esta sessão **não classifica** o SIGSEGV
  intermitente da decisão 057 como presente ou ausente;
- nenhuma correção de produção foi feita nesta sessão.
