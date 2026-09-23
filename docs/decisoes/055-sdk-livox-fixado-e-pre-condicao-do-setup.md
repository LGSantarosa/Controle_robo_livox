# 055 — O SDK nativo da Livox é fixado, e o `setup_livox.sh` para cedo

**Data**: 2026-09-23 (PC de dev, fora do laboratório; robô desligado o tempo todo)
**Status**: aplicada (`c6a7b46`); **nada testado contra hardware** — o lidar não
foi ligado, nem a rede dele tocada.
**Toca**: `setup_livox.sh` (passo 0/5 novo e o passo 1/5).
**Vem de**: fechamento da etapa 5 e preparo da etapa 6 — a pilha de
localização não compilava neste PC.

---

## 1. Contexto

Ao preparar a etapa 6 (a pilha com `robo:=3`), o `colcon build` reprovou em
`Could not find LIVOX_LIDAR_SDK_LIBRARY`. O diagnóstico mostrou três coisas
neste PC, todas anteriores a hoje:

1. o **SDK nativo estava compilado desde 24-07** (`third_party/Livox-SDK2`),
   mas **nunca instalado** em `/usr/local` — o `sudo cmake --install` não
   completou;
2. o `install/livox_ros_driver2` que existia era uma **casca de 6 arquivos**,
   só escrituração do colcon: zero biblioteca, zero mensagem, sem
   `ament_index`. Não era build velho: era o rastro de um build que falhou;
3. o `MID360_config.json` **dentro do clone** ainda tinha `192.168.1.169` — o
   IP que, pela varredura de 15-09, **não respondeu**. O versionado tem
   `.158` desde `56e6bda`. O passo 4 do setup nunca rodou depois disso.

O (3) é o mais perigoso dos três, e é falha sem sintoma no lugar errado: IP
errado = `bind failed` = FAST-LIO sem nuvem = `/Odometry` nunca publicado.
Quem investiga olha o FAST-LIO; o defeito está num JSON dentro de um clone
descartável.

## 2. Decisão

**(a) O SDK nativo passa a ser fixado como os dois drivers**: `SDK_TAG=v1.3.1`
e `SDK_COMMIT=f5d9375f84efe2b15bc0a052d3e18482ed13adf4`, que é a revisão
compilada neste PC. A tag existe para leitura humana; **o SHA é quem fecha**,
porque tag pode ser movida no upstream — o clone novo confere os dois.

**(b) Revisão diferente é recusada, não corrigida.** Se o clone existir em
outro commit, o script sai com código 1 e manda decidir: apagar o diretório ou
mudar o valor no script. Não atualiza, não reseta.

**(c) A conferência da revisão roda antes da guarda do `$SDK_LIB`** — se fonte
e instalado divergirem, a gente quer saber mesmo com a lib já em `/usr/local`,
que é justamente o caso silencioso.

**(d) Passo 0/5: pré-condição de dependências ROS**, antes de tocar em
`/usr/local` e antes de clonar. Lista fixa (`pcl_ros`, `pcl_conversions`),
porque a conferência roda antes do clone — em máquina nova os `package.xml`
ainda não existem. Faltando, imprime o `apt install` exato e sai com 1.

## 3. Alternativas descartadas

| alternativa | por que não |
|---|---|
| **deixar `--depth 1` sem revisão** | o que vai para `/usr/local` é código nativo, fora do git, que **nenhum teste nosso cobre**. Em máquina nova seria outra lib, sem uma linha de aviso. É a mesma família da zona morta: diferença real, sintoma nenhum |
| **formalizar `--packages-skip livox_ros_driver2 fast_lio`** | resolvia o build, mas deixava a pilha de localização não-construível neste PC — e a etapa 6 começaria apoiada num `install/` que, como se viu, era uma casca vazia |
| **o script atualizar o clone sozinho** (`fetch` + `checkout`) | mexer em fonte de terceiro é mudança deliberada, com registro. Rodar o setup não pode ser o gatilho |
| **ler as dependências dos `package.xml`** em vez de lista fixa | a conferência tem de rodar **antes do clone**; em máquina nova não há `package.xml` para ler |
| **`rosdep install --from-paths`** | instala um conjunto que a gente não escolheu, e ainda precisa de sudo. A lista explícita diz exatamente o que falta e por quê |

## 4. O que foi provado, e o que não foi

**Provado hoje, neste PC:**

- recusa por revisão errada: sandbox com commit diferente → para em 1/5,
  código 1, **nada clonado nem construído**;
- recusa por dependência faltando: **defeito real**, com o `pcl_ros` ausente →
  para em 0/5, código 1, aponta só o `pcl_ros` (o `pcl_conversions` existe) e
  imprime `sudo apt install ros-jazzy-pcl-ros`;
- caminho positivo: `rc=0`, SDK instalado (`ldconfig` confirma), `.158`
  copiado (`cmp` idêntico), `livox_ros_driver2` e `fast_lio` com prefixo real
  (`ros2 pkg prefix`), mensagem `CustomMsg` visível no grafo;
- suíte: **1239 passed**, código 0 — o driver compilado não mudou a suíte.

**NÃO provado, e não se pretende:**

- 🔴 **nada de hardware.** O lidar não foi ligado. Que o `.158` responda é
  crença baseada na varredura de 15-09, não medição de hoje;
- 🔴 **o FAST-LIO publicar `/Odometry`** — compilar não é localizar;
- 🔴 **o NUC.** Isto conserta o clone **deste PC**. O robô pode estar com
  outra revisão de SDK, sem `pcl_ros`, e com o `.169` velho. Vira tarefa
  explícita de deploy da etapa 6.

## 5. Referências

- `setup_livox.sh`, cabeçalho e passos 0/5 e 1/5.
- `ros2_packages/robot_base/config/README.md` — a varredura que achou o
  `.158` e descartou o `.169`.
- Diário de 2026-09-23.
