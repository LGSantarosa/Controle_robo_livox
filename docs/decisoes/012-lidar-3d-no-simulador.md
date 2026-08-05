# 012 — LiDAR 3D no simulador: raster no lugar da roseta

**Data**: 2026-08-04 · **Status**: aceita pelo dono (pedido em sessão)

## Contexto

O simulador reproduz a dinâmica de rumo do robô dentro da dispersão da
máquina (aceitação de 04-08, validada a olho pelo dono). Mas **não tem
sensor nenhum**: o `/Odometry` é pose verdadeira do Gazebo, e obstáculo só
existe se vier de mapa estático. Isso trava a fatia B inteira (desviar de
obstáculo) e o item 11 do ESTADO.

O robô tem **um** sensor externo: o Livox Mid-360.

## O que decide

Entra um **`gpu_lidar` do Gazebo** montado no topo da caixa, publicando
`PointCloud2` em `/livox/lidar` — **mesmo tópico e mesmo papel do driver
real**, pelo mesmo princípio que fez o `/Odometry` se chamar `/Odometry`:
quem consome não deve saber a diferença.

Parâmetros copiados da folha do Mid-360:

    FOV horizontal   360°
    FOV vertical     −7° a +52°
    alcance          0,1 a 40 m
    taxa             10 Hz
    ruído            gaussiano, σ = 2 cm

### ⛔ A ressalva que define o que este sensor prova

**O Mid-360 não varre em raster — varre em ROSETA não repetitiva.** Cada
quadro cobre uma fração do campo, e a cobertura ADENSA com o tempo de
integração. O `gpu_lidar` do Gazebo só sabe varrer raster: grade uniforme,
cobertura idêntica todo quadro.

Consequência, e ela é otimista: **o simulador vai enxergar melhor do que o
robô** num quadro só. Um obstáculo fino (perna de cadeira, pé de mesa) que
a roseta pode perder num quadro e achar no seguinte, aqui aparece sempre.

Então este sensor serve para:
- geometria de campo de visão, zonas cegas e alcance;
- amarrar a cadeia `nuvem → costmap → Nav2` e achar defeito de integração;
- desenvolver a lógica de desvio.

E **não** serve para:
- prever taxa de detecção de obstáculo fino;
- dimensionar tempo de integração;
- qualquer conclusão sobre **FAST-LIO**, que se alimenta do padrão de varredura.

Contramedida barata quando isso importar: baixar a densidade angular e ver
se a lógica sobrevive à nuvem esparsa. Fica registrado como o teste a fazer
antes de confiar em qualquer número de detecção.

### O que este sensor NÃO vai alimentar (por ora)

**O `/Odometry` continua sendo pose verdadeira do Gazebo.** Rodar FAST-LIO
sobre a nuvem simulada é outra decisão, com outro custo, e a roseta ausente
é exatamente o que mais pesaria ali. A decisão 004 fica intacta.

## O achado que já sai da geometria, antes de rodar

O Mid-360 quase não olha para baixo (−7°). Montado com o centro óptico a
`h`, o chão só começa a ser visto a `h/tan(7°) ≈ 8,1·h`. Com a caixa medida
com trena (topo a 0,230 m) e o sensor logo acima:

    centro óptico ~0,27 m  ->  chão invisível num raio de ~2,2 m

E obstáculo a distância `d` só aparece se for mais alto que `h − d·tan 7°`:

    a 0,5 m   precisa ter mais de ~21 cm
    a 1,0 m   precisa ter mais de ~15 cm
    a 1,5 m   precisa ter mais de ~9 cm
    a 2,2 m   o chão aparece

**Um robô de 45 cm de largura com 2,2 m de cegueira no chão à sua volta.**
Isto é do SENSOR, não do simulador — e é dado de projeto para a fatia B:
obstáculo baixo e perto é invisível por geometria, e nenhuma sintonia
resolve. Some com o BO-3 (parar em silêncio) como par de riscos a delatar.

⚠️ **A altura de montagem é PROVISÓRIA** — mesmo estatuto do `boba_raio`.
Não foi medida com trena; está no arquivo como valor único e nomeado, para
trocar num lugar só. Como a zona cega escala com ela, **medir a altura do
Mid-360 é item de bancada**, e barato.

## Alternativas descartadas

1. **Plugin de Livox de terceiros** (`livox_laser_simulation` e portes) —
   reproduzem a roseta, mas os que existem são de **Gazebo Classic**; para
   o Harmonic o suporte é imaturo e viraria dependência de terceiro no
   caminho crítico, com o mesmo risco que o `setup_livox.sh` já administra
   com commits fixados. Fica como evolução se a roseta virar bloqueio real.
2. **Ray lidar de CPU** — mesma limitação de raster, muito mais lento.
3. **Não modelar sensor e seguir com mapa estático** — é o estado de hoje, e
   é ele que trava a fatia B.
4. **Montar o sensor tiltado para baixo** para matar a zona cega — muda o
   robô real, não o modelo. Se virar proposta, é decisão de hardware do
   dono, e o simulador passa a ser onde ela se avalia. **O modelo tem de
   refletir o robô que existe.**

## Referências

- Geometria da caixa: trena de 07-29, `ESTADO_PROJETO.md`.
- Papel do `/Odometry` e o princípio de mesmo-tópico: decisão 004.
- Fatia B e o que falta para desviar: decisão 006, item 11 do ESTADO.
