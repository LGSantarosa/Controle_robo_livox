# 003 — Base do robô 2: tração por `ros2_control` e localização por LIO

**Data:** 2026-07-24
**Estado:** aceita
**Revisa:** [001](001-gui-2d-localizacao-3d.md) e [002](002-rota-2d-primeiro-lio-depois.md)
**Fecha:** BO-1 (arquitetura da ponte PC↔placa hover)

---

## Contexto

Até aqui o robô 2 estava inerte por software e duas perguntas travavam tudo:

1. **BO-1** — como o PC fala com a placa de hoverboard? Havia duas candidatas
   (serial direta vs. reintroduzir um microcontrolador no meio) e a decisão
   estava suspensa esperando bancada.
2. **Localização** — a decisão 002 propunha derivar um `/scan` 2D da nuvem do
   Mid-360 e rodar AMCL + Nav2, replicando a receita do robô 1, deixando LIO 3D
   como evolução posterior.

O plano de reforma (`MIGRACAO_LIVOX.md`) previa adaptar dois nós herdados do
robô 1 para isso: `mega_bridge.py` (ponte serial via microcontrolador) e
`cmd_vel_to_wheels.py` (cinemática diferencial caseira).

Nesse meio tempo a base foi **efetivamente colocada para funcionar em hardware
neste robô**: as rodas giram sob comando e o robô sabe onde está. Isso responde
as duas perguntas com evidência em vez de projeto, e é o que motiva esta
decisão.

## Decisão

Adotar como **base** do robô 2:

- **Tração:** `ros2_control` com uma interface de hardware que fala serial
  direta com a placa hover, acionando um `diff_drive_controller`. Pacote
  `ros2_packages/hoverboard_driver/`.
- **Localização:** `livox_ros_driver2` (Mid-360) + **FAST-LIO**, publicando
  `/Odometry`. Sem `/scan` 2D, sem mapa, sem AMCL.
- **Movimentação:** escrita **do zero** neste projeto. Não se herda controlador
  de rumo de lugar nenhum.

O pacote `ros2_packages/robot_base/` amarra tração + localização
(`base.launch.py`) e é o único ponto de entrada da base.

## Fatos técnicos que sustentam a decisão

Medidos neste robô, não deduzidos:

- **A placa hover conversa direto com o PC por serial**, sem microcontrolador
  intermediário. Protocolo `0xABCD` a 115200.
- **O quadro de realimentação da placa tem 18 bytes**
  (`start, cmd1, cmd2, speedR, speedL, batVoltage, boardTemp, cmdLed, checksum`).
  Isto não é detalhe cosmético: com a estrutura errada o checksum nunca fecha,
  a posição das juntas fica `NaN`, o controlador rejeita **todo** comando
  ("non-finite error value") e **as rodas simplesmente não giram** — sem erro
  óbvio apontando para a causa. Foi a falha que mais custou tempo.
- **O motor tem zona-morta alta**: abaixo de ~`speed 100` (de 1000) ele não
  vence o atrito e a roda fica parada "tentando". Por isso o comando é escalado
  até a roda de maior magnitude cruzar esse piso — **escalando as duas juntas**,
  nunca cada uma por si, senão a roda interna de uma curva também sobe ao piso,
  a razão entre as rodas se perde e o robô abre a curva em vez de fechá-la.
- **A placa reporta velocidade, não contagem de encoder** — a posição das
  juntas é obtida integrando a velocidade.
- **A localização por LIO funciona neste robô** e a origem dela zera a cada
  boot: a pose é relativa ao ponto de partida, não a um mapa global.

## Alternativas descartadas

| Alternativa | Por que não |
|---|---|
| **Reintroduzir microcontrolador entre PC e placa** (candidata B do BO-1) | A serial direta foi verificada funcionando. Um elo a mais só adiciona firmware para manter, latência e um ponto de falha, sem resolver nada que já não esteja resolvido. |
| **`mega_bridge.py` + `cmd_vel_to_wheels.py`** (ADAPTA 1–2 do plano de reforma) | Pressupunham o microcontrolador e uma cinemática diferencial escrita à mão. O `diff_drive_controller` do `ros2_control` é o caminho padrão, já resolve limites de velocidade/aceleração e odometria de roda, e é o que está verificado. Manter os dois seria manter duas cinemáticas concorrentes. |
| **Derivar `/scan` 2D + AMCL + Nav2** (decisão 002) | O LIO já entrega pose 6-DoF funcionando com o sensor que temos. Achatar uma nuvem 3D para 2D só para alimentar um localizador que precisa de mapa prévio joga informação fora e **acrescenta** trabalho (mapear o ambiente, manter o mapa) em vez de remover. A 002 apostava que o caminho 2D seria mais rápido de chegar; a evidência é o contrário. |
| **Herdar o controlador de rumo junto com a base** | Rejeitado explicitamente — ver abaixo. |

## Por que a movimentação NÃO vem junto

A camada que decide *para onde ir* foi avaliada e recusada. O motivo é
estrutural, não de ajuste fino: ela empilhava **ganhos multiplicativos em série**
em três nós diferentes antes de chegar no controlador, que por sua vez tinha
tetos de velocidade. O resultado é que o teto engolia a modulação — a regra
"desacelere quando estiver desalinhado" produzia um valor mínimo já acima do
teto, então **o robô andava sempre na velocidade máxima**, alinhado ou não,
enquanto o giro saturava. Andar reto a fundo com giro saturado é exatamente a
receita de zigue-zague em S e de curva que não fecha ("balão").

O sintoma real observado no robô é esse: **o robô anda em S**. E a geometria
piora: **as rodas motrizes são dianteiras e a roda boba é traseira**, então a
traseira só acompanha por arrasto e amplifica qualquer oscilação de rumo — o
oposto de um carrinho de supermercado, que é estável porque a boba vai à frente.

Consequência de projeto para o que vamos escrever:

> O controlador de movimentação publica velocidade em **unidades SI reais**
> (m/s, rad/s) direto no `diff_drive_controller`, **sem nenhuma escada de ganhos
> intermediária**. Se um valor precisa ser limitado, o limite mora em um lugar
> só. Sem isso, nenhum ajuste é interpretável.

## Consequências

- **BO-1 fechado**: serial direta, sem microcontrolador.
- **Decisão 002 revisada**: a rota "2D primeiro" cai. O baseline 2D vs LIO que
  ela previa como resultado do artigo deixa de ser o eixo — ver "Em aberto".
- **`ros2_packages/robot_nav/` vira legado**: `mega_bridge.py` e
  `cmd_vel_to_wheels.py` deixam de ser caminho. Não foram apagados nesta leva
  de propósito — a GUI e a suíte de testes ainda os referenciam, e remover tudo
  junto quebraria as duas de uma vez. A remoção é uma fatia própria, depois que
  a base nova estiver rodando aqui.
- Os drivers de terceiros (`livox_ros_driver2`, `FAST_LIO`) e o SDK nativo da
  Livox **não são versionados**: `setup_livox.sh` os traz em commits fixados.
  Razão em comentário no próprio script.
- O ambiente não precisa mais ser mapeado antes de o robô andar.

## Em aberto

- **O que o artigo compara**, já que o baseline 2D saiu do caminho. Candidatos:
  comparar métodos de LIO entre si, ou comparar estratégias de controle de rumo
  para esta geometria (motriz dianteira + boba traseira), que é o problema real
  que temos em mãos.
- `wheel_separation` e `wheel_radius` do controlador diferencial ainda são
  valores herdados, **não medidos neste robô**. Erram a odometria de roda e a
  conversão de `rad/s` comandado. Medir com trena antes de calibrar qualquer
  coisa de movimentação.
- A pose do LIO zera a cada boot; se em algum momento for preciso repetir
  trajeto entre sessões, isso vira um problema a resolver.
