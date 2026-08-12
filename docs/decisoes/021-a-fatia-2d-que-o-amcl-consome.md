# 021 — A fatia 2D que a localização contra mapa consome

**Data**: 2026-08-12 (dev, robô desligado) · **Branch**: `slam-meu-mapa`
**Status**: implementada, não vista no simulador nem no robô
**Toca**: `robot_base/config/scan_2d.yaml`, `launch/scan_2d.launch.py`,
`localizacao.launch.py`, `sim.launch.py`

## Contexto

Existe um mapa do andar, feito por SLAM, e ele cobre a sala, o corredor e um
bom pedaço do piso. Ele resolveria a limitação que a decisão 015 aceitou de
propósito: sem mapa, o costmap global é uma janela de 20 × 20 m e **o que o robô
nunca viu conta como livre** — memória curta, que basta para 2 m e não para o
corredor.

Mas mapa sozinho piora as coisas. Hoje a `pilha.launch.py` publica `map → odom`
**fixo, identidade**: com um mapa de verdade, o robô acreditaria estar
eternamente na origem dele, e desde a 014 essas paredes fantasma ainda se
misturam com marcação real do Livox. **Mapa e localização-contra-mapa são
pacote fechado.**

A localização que fecha esse pacote é o AMCL, e o AMCL é **2D**: ele casa um
`sensor_msgs/LaserScan` contra um mapa de ocupação. O nosso único sensor é uma
nuvem 3D. Então alguém tem de escolher **qual fatia do mundo vira "parede"** —
e é essa escolha que esta decisão registra.

## A decisão

Um `pointcloud_to_laserscan` consumindo **`/livox/pontos`** (o tópico que a
percepção lê nos dois mundos, decisão 017) e publicando `/scan`, com a fatia
medida **a partir do chão** (`target_frame: base_link`, cuja origem está no
chão).

```
min_height   0,15 m     range_min   0,35 m      360°, passo de 1°
max_height   1,00 m     range_max  20,0  m      10 Hz
```

Cada número sai de uma medida da máquina:

| número | de onde vem |
|---|---|
| `min_height` 0,15 | o raio de −7° bate no piso a `0,42/tan(7°)` = **3,4 m**. Piso lido como obstáculo põe um **anel de parede** em volta do robô, que a planta não tem — e o AMCL não fecha |
| `max_height` 1,00 | o Mid-360 olha para **cima** (+52°): a 0,45 m ele já alcança 1,00 m. Sem corte, teto e luminária viram parede. O mapa é uma planta |
| `range_min` 0,35 | o raio do robô é **0,32** (`nav2.yaml`) e a caixa fica a 0,23 m — **dentro da fatia**. Sem corte ele lê a própria carcaça, em todos os quadros e direções |
| `range_max` 20,0 | o mapa tem 23,5 × 67,2 m; ler além dele não ajuda a casar e traz retorno de fora da planta |

⚠️ **A fatia precisa ser alcançável, não só razoável**: `min_height` acima de
0,42 (a altura do sensor) deixaria o robô cego para parede próxima, sem aviso.
Há teste para isso.

## Alternativas consideradas

**(a) Consumir `/cloud_registered_body` (a nuvem já registrada pelo LIO).**
Descartada porque **ela não existe no simulador** — lá o `/Odometry` é a pose
verdadeira do Gazebo e não roda FAST-LIO. Usar uma entrada que só existe no robô
tira do simulador justamente a capacidade de provar a fatia, que é a parte mais
fácil de errar.

**(b) Escrever a projeção nós mesmos, em Python.** Tentador porque o
`nuvem_pontos` já percorre todos os pontos e poderia cuspir os dois formatos num
passe só. Descartada por CPU: em Python ele já come ~91% de um core, e um
segundo nó dobraria isso. O `pointcloud_to_laserscan` é C++ e resolvido.

⚠️ **Isto adiciona uma dependência de sistema** (`ros-jazzy-pointcloud-to-laserscan`),
e dependência nova neste projeto exige conferir o que ela arrasta — em 05-08
ficou provado que subir um pacote só pode quebrar a pilha (o símbolo do
`diagnostic_updater` que **desaparece** entre versões). Conferido antes de
escolher:

```
apt-get install --dry-run ros-jazzy-pointcloud-to-laserscan
    0 upgraded, 1 newly installed, 0 to remove
```

Instalação nova, zero upgrades. O perigo de 05-08 é de *upgrade* e não tem essa
forma aqui.

**(c) Deixar o simulador com uma fatia e o robô com outra** (por exemplo,
aproveitando que no Gazebo o chão é perfeito). Descartada sem discussão: é o
defeito da bitola de 29-07, que custou semanas — a bancada parece boa e o robô
piora. Um launch só, um YAML só, e teste travando que os dois mundos sobem o
mesmo.

## Como isto se prova

**Sete testes**, que leem os arquivos que o robô carrega — nenhum sobe ROS:
a fatia não lê o chão; não lê o teto; o robô não se lê como parede; a fatia é
alcançável pelo campo de visão; o scan sai de `/livox/pontos` e não do tópico
cru; e os **dois mundos** sobem a mesma fatia.

🔧 **O teste do nome do nó existe por um modo de falha silencioso**: parâmetro
que não casa com o nome do nó é **ignorado**, e o nó sobe com os defaults do
upstream — fatia de −1 a +1 m, que lê chão **e** teto. Nada no log diria isso.

🔧 **E um dos testes nasceu errado, de novo.** O "os dois mundos sobem a mesma
fatia" procurava o nome do arquivo no TEXTO da launch. A mutação (tirar
`scan_2d` da lista do `LaunchDescription`, deixando a variável definida e sem
uso) **passou** — o simulador subiria sem fatia nenhuma com a suíte verde. A
versão que ficou pergunta por AST se o include chega mesmo à `LaunchDescription`,
direto ou por variável usada na lista. **É a terceira vez que este projeto pega
a mesma forma de teste ruim** (017, 019, e agora aqui): teste que reconstrói ou
tangencia o alvo não testa o alvo.

⏳ **Nada rodou ainda** — nem no simulador. O que o Gazebo responde a seguir, e
é o que mais dá erro: o `/scan` casa com o `meu_mapa`? O que ele **não**
responde é convergência do AMCL contra odometria que deriva, porque lá a pose é
verdadeira.

## Referências

- `docs/decisoes/015-nav2-sem-mapa-no-robo-real.md` — a memória curta que isto
  vem resolver
- `docs/decisoes/017-a-nuvem-que-a-percepcao-consome.md` — por que a entrada é
  `/livox/pontos`
- `docs/decisoes/019-o-default-e-o-caso-seguro.md` — a lição de teste que este
  repetiu
- `docs/MODELO_ROBO2.md` e `docs/dados/2026-08-05-lidar-campo-de-visao/` — a
  altura de 0,42 m e o campo de visão de −7° a +52°
