# Robô 3 sem freio — entrada da salinha

Corrida manual no Gazebo em 29-09-2026, iniciada às 14:44, no commit
`4f507ea`. Pilha subida com `robo:=3 sim:=true bag:=false` e
`freio_linear:=false`; objetivo enviado pelo RViz para
`(11,01; 1,52326)`. Não houve bag: os dados preservados são os CSVs do
`freeze_capture`, o CSV do seguidor, os perfis materializados e o `launch.log`.

Observação do dono: o pulinho para trás desapareceu, mas o robô parou cerca de
três vezes na porta da salinha e demorou demais para voltar a andar.

Resultado conferido, sem mudança de código: o objetivo terminou com
`Goal succeeded`, a 4 cm do alvo em repouso. O freio não atuou, não houve ré e
o rebote desapareceu.

Na porta, o `collision_monitor` acionou `PolygonStop` repetidamente. A primeira
recuperação só começou 9,91 s depois; ela foi abortada porque a medida de
`/scan` ficou velha. O segundo `PolygonStop` esperou mais 7,73 s até uma nova
tentativa, que avançou 0,21 m e liberou o robô. Depois ocorreram duas atuações
curtas, de 0,57 s e 0,15 s. Portanto, a demora observada não veio de uma
redução da velocidade: veio da espera de recuperação (`re_parado_s=4,0`) somada
às perdas intermitentes do Livox, que abortaram a primeira saída e fizeram o
ciclo começar novamente. Durante esse trecho o Gazebo rodou a aproximadamente
0,5x do tempo real; por isso os 4 s do relógio simulado pareceram cerca de 8 s
para quem estava olhando, e o movimento inteiro também pareceu mais lento.

Arquivos brutos: `ros_logs/`.
