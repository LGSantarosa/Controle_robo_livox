# 015 — Nav2 sem mapa: no robô real, `mapa:=nenhum`

**Data:** 2026-08-07
**Estado:** implementada e medida no simulador; **nunca rodou no robô**
**Depende de:** [003 — base ros2_control e LIO](003-base-ros2control-e-lio.md),
[014 — o costmap enxerga a nuvem](014-costmap-enxerga-a-nuvem.md)

## O que estava errado, e estava desde que a pilha nasceu

A decisão 003 tirou o mapa e o AMCL do projeto: a localização é LIO, o robô não
precisa de mapa para saber onde está. Mas a `pilha.launch.py` continuou subindo
um `map_server` apontado para `maps/pista_obstaculos.yaml` — **a planta da pista
SIMULADA**, uma sala de 12 × 8 m que não existe em lugar nenhum — e o
`global_costmap` continuou montado sobre uma `StaticLayer`.

O cabeçalho da própria launch já dizia isso em voz alta:

> *"No robô real isto não vale, e é por isso que o `mapa` é argumento: sem mapa
> que corresponda ao lugar, o costmap global inventa obstáculo onde não tem."*

Era um aviso sem saída: não existia o "sem mapa" para escolher.

**A decisão 014 (no mesmo dia) piorou o quadro.** Com a nuvem entrando nos
costmaps, o global passou a MISTURAR as paredes fantasma do mapa errado com
marcação real e permanente do Livox. Meio mapa é mais difícil de diagnosticar
que mapa nenhum — e diagnóstico difícil com a bateria correndo é o que este
projeto mais evita.

## O que entra

`ros2 launch robot_motion pilha.launch.py mapa:=nenhum`

- **sem `map_server`** — e ele sai também da lista do `lifecycle_manager`, que
  aborta o bringup inteiro se um servidor da lista não responder (foi assim que
  o `collision_monitor` morreu junto com o Nav2 em 06-08 e o teste D caiu);
- **sem `static_layer`** nos dois costmaps;
- **`global_costmap` vira janela rolante de 20 × 20 m**, alimentada só pelo
  Livox. É o mesmo princípio do local (4 × 4 m), com outro tamanho e outro
  propósito: o local existe para desviar do que está no caminho, o global para
  escolher **rota** — e rota se escolhe com o que está longe.

Tudo isso vive em `config/nav2_sem_mapa.yaml`, que é um **overlay**: entra
depois do `nav2.yaml` e sobrescreve só o que muda. Copiar o `nav2.yaml` inteiro
e editar recriaria o defeito da bitola de 29-07 num lugar novo — dois arquivos
com o mesmo número, um envelhecendo sozinho, e o robô rodando com um valor
diferente do que a bancada julgou. Há teste que falha se o overlay redefinir
geometria.

⚠️ **O frame continua `map`.** A TF `map → odom` que a launch publica é a
identidade, então `map ≡ odom` e o goal do RViz chega onde deve. Trocar o frame
para `odom` seria mais honesto no nome e obrigaria a mexer no `bt_navigator`, no
RViz e em quem publica goal, sem ganho nenhum enquanto a TF for identidade.
Quando existir localização que case `map` com `odom` de verdade, é ali que ela
entra.

## O que foi medido

Simulador, mundo `pista_surpresa.sdf`, robô parado em (2,0 · 5,0), caminho
pedido de lá para (2,0 · 7,2) — o mesmo par de sempre.

**Sem mapa, o costmap global é feito só do que o sensor viu**, e as manchas
batem com a planta real:

```
(0,19 · 5,00) parede oeste (real em 0,20)     0,553 m²
(3,91 · 5,10) divisória    (real em 3,90)     0,505 m²
(1,19 · 7,81) e (2,84 · 7,80) parede norte (real em 7,80)
(2,00 · 6,28) a caixa surpresa                0,075 m²
(3,79 · 2,64) a surpresa do vão da porta      0,025 m²
```

**E o plano contorna:**

```
                    caminho / reta   folga do centro do obstáculo
com mapa              2,74 / 2,20         0,530 m    CONTORNA
sem mapa nenhum       2,78 / 2,25         0,530 m    CONTORNA
```

O robô planeja **sem mapa**, só com o que enxerga, e chega ao mesmo desvio. O
controle com mapa foi rodado depois da mudança e reproduz o número anterior
exato — a refatoração da launch não mexeu naquele caminho.

## O que isso custa, e precisa estar claro para quem opera

**O robô passa a planejar com MEMÓRIA CURTA.** Fora da janela de 20 m ele não
sabe de nada, e o que nunca viu conta como **livre** (`track_unknown_space:
false`, herdado do `nav2.yaml`). O plano pode passar por dentro de uma parede
que está atrás de uma quina; quem responde por isso é o replanejamento a 1 Hz e
o reflexo de colisão.

Isso **não é regressão**: com o mapa errado o comportamento era pior, porque a
parede inventada estava no lugar errado — e uma parede fantasma no lugar errado
não é conservadora, é aleatória.

## Ressalvas

1. **Nada disto rodou no robô.**
2. A dívida da 014 continua: **no global costmap a marcação é permanente**, e
   com `map→odom` fixa a deriva do LIO vira obstáculo fantasma acumulado. Sem
   mapa isso fica *mais* visível, porque não há mais nada no costmap para
   confundir com fantasma. `clear_entirely_global_costmap` é a saída de campo.
3. **20 × 20 m é escolha, não medida.** Cobre a sala de teste inteira com folga;
   num corredor longo pode ser pouco. É um número para revisitar com o robô.

## Referências

- `docs/DIARIO.md`, 07-08 — a sessão
- `ros2_packages/robot_motion/config/nav2_sem_mapa.yaml` — o racional por chave
- `ros2_packages/robot_motion/launch/pilha.launch.py` — o cabeçalho diz qual
  perfil usar em qual máquina
