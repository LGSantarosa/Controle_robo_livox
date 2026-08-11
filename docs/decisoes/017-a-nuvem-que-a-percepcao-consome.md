# 017 — A percepção consome `/livox/pontos`, não o tópico do driver

**Data**: 2026-08-11 · **Estado**: implementado, NÃO visto no robô
**Vem de**: o pré-voo de 10-08 · **Toca**: `robot_base` (nó novo), as duas
launches, `nav2.yaml`, `collision_monitor.yaml`, `checa_pilha.py`

## O defeito, medido

```
/livox/lidar   publisher:  livox_ros_driver2/msg/CustomMsg   (1 publisher)
costmaps, collision_monitor e o pré-voo assinam  sensor_msgs/msg/PointCloud2
```

Quem assina `PointCloud2` **nunca recebeu nada** no robô. O FAST-LIO lê
CustomMsg — por isso a localização sempre foi bem (deriva de 1,8 mm em 15 s) e a
percepção era zero: os dois costmaps com **0 células letais** enquanto o
`ros2 topic hz /livox/lidar` mostrava **9,96 Hz**.

**A decisão 014 estava inerte desde que nasceu**, e o reflexo de colisão nunca
funcionou no robô — o que explica o teste D de 06-08 sem precisar de outra
hipótese além da TF que faltava.

## A causa raiz é de contrato, não de configuração

O comentário da ponte do Gazebo dizia, com todas as letras:

> *"ela é remapeada para `/livox/lidar`, o MESMO nome do driver real, como o
> /Odometry: quem consome não sabe a diferença"*

A intenção era boa — um só nome nos dois mundos, consumidor sem `if`. Só que
**um tópico é (nome, tipo)**. Igualar o nome sobre tipos diferentes não elimina
a diferença: esconde. No simulador o `gpu_lidar` publica `PointCloud2` naquele
nome, então tudo funcionava; a decisão 014 foi escrita, medida e aceita num
ambiente onde o defeito não existe.

⚠️ **Nenhum dos testes de config pegou isto**, e eles são muitos. Todos
conferiam que os consumidores concordavam **entre si** — e concordavam: os três
liam `/livox/lidar`. Faltava perguntar **quem publica aquilo, e em que tipo**.

## Decisão

```
/livox/lidar    CRU, tipo dependente do mundo:
                robô      -> CustomMsg      (o que o FAST-LIO come)
                simulador -> PointCloud2    (o gpu_lidar do Gazebo)

/livox/pontos   PointCloud2 SEMPRE — o contrato da percepção
```

- no **robô**, um nó novo converte (`robot_base/nuvem_pontos.py`), subindo junto
  na `localizacao.launch.py`;
- no **simulador**, a ponte do Gazebo passa a remapear direto para
  `/livox/pontos` — lá não há conversão nenhuma a fazer;
- `nav2.yaml` (as duas camadas), `collision_monitor.yaml` e o `checa_pilha.py`
  passam a ler `/livox/pontos`.

**Três testes novos fecham o buraco que deixou isto passar**: que a ponte do
simulador publica no tópico que a percepção lê, que a launch do robô sobe o
conversor, e que ninguém volta a consumir o tópico cru.

### O ponto (0,0,0) é descartado, e isso não é detalhe

O Mid-360 emite `(0,0,0)` quando o raio não volta. No frame do sensor esse ponto
é **o próprio robô**: deixá-lo passar marcaria célula letal em cima dele a cada
quadro, e o planejador recusaria todo caminho por estar "dentro" de um
obstáculo — com a nuvem parecendo perfeita. Só o ponto **inteiro** zerado sai;
`(0, 3, 0)` é parede ao lado do robô e fica.

## Alternativas consideradas, e por que não

**(a) Trocar o `xfer_format` do driver para PointCloud2.** Um tópico a menos e
nó nenhum. **Descartado**: o FAST-LIO deste projeto lê Livox `CustomMsg`
(`lidar_type: 1`); mudar o formato quebra a localização, que é a única coisa que
funciona bem hoje. Trocar percepção quebrada por localização quebrada não é
progresso.

**(b) Subir um segundo driver com o outro formato.** Dois processos falando com
um sensor, contra a regra de "uma pilha só" que já custou horas em 07-31 e
07-08 (três `fastlio_mapping` órfãos, três pilhas empilhadas).

**(c) Ensinar os consumidores a ler CustomMsg.** Não são nossos — `nav2_costmap_2d`
e `nav2_collision_monitor` são upstream.

**(d) Manter o nome `/livox/lidar` e converter em cima dele.** Publicar
PointCloud2 no mesmo nome onde já há CustomMsg deixaria o tópico com dois tipos
vivos, que é exatamente a ambiguidade que criou o problema — e o `ros2 topic
info` do robô já mostra os dois tipos listados, o que confunde o diagnóstico.

## Como isto se prova no robô

```bash
ros2 topic hz /livox/pontos                   # ~10 Hz
ros2 topic info /livox/pontos                 # UM tipo: sensor_msgs/msg/PointCloud2
python3 tools/banco/checa_pilha.py            # a linha da nuvem tem de passar
```

⚠️ **PREVISÃO FALSIFICÁVEL, e ela diz que isto sozinho NÃO basta:** os costmaps
devem continuar com **0 células letais** depois desta mudança. Há um segundo
defeito em série, medido em 10-08 — com o `tf_odom` compondo a pose do sensor, o
`odom` fica na altura do sensor (`tf2_echo odom base_link` deu z = −0,477 m), o
chão vai para z ≈ −0,42 e a faixa de altura da camada (0,10–0,50 m, aplicada no
frame global) rejeita tudo.

Se os costmaps **marcarem** já com esta mudança, a previsão está errada e a
faixa de altura não é aplicada no frame global como eu suponho — o que muda o
conserto seguinte. É o próximo item da fila.

## Referências

- `docs/DIARIO.md`, 10-08 (o pré-voo) e 11-08
- `docs/decisoes/014-costmap-enxerga-a-nuvem.md` — a decisão que estava inerte
- `docs/dados/2026-08-10-sessao/ambiente.txt` — os dois pré-voos, 10/19 e 16/19
