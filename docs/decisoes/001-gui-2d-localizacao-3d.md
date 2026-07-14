# 001 — Conceito de arquitetura: interface humana 2D, localização 3D

**Data**: 2026-07-14 · **Status**: conceito norteador aprovado pelo dono;
escolhas concretas (qual LIO, papel do Nav2, geração do mapa 2D) ficam para
decisões futuras, cada uma com literatura.

## O conceito

> "Minha ideia é deixar a visão do GUI 2D, mas a localização real usada pelo
> robô ser 3D. Eu mando ele pro lugar via mapa 2D, porque é fácil pra humano,
> e ele funciona usando o 3D." — dono, 2026-07-14

Duas camadas com contratos distintos:

1. **Camada humana (2D)** — a GUI web herdada do robô 1: mapa 2D no browser,
   clique/rota para mandar o robô, trilha, HUD, métricas. Um mapa 2D é o que
   um humano lê e opera com facilidade; isso o robô 1 já provou em campo.
2. **Camada do robô (3D)** — localização e percepção reais vêm da nuvem do
   Mid-360 (odometria/localização LiDAR-inercial 3D; candidatos FAST-LIO2 /
   Point-LIO / LIO-SAM em `docs/REFERENCIAS.md`). O robô NÃO depende de AMCL
   2D nem de odometria de roda como fonte primária de pose.

A costura entre as camadas é uma **projeção**: o mundo 3D precisa gerar um
mapa 2D pra GUI (ex.: fatia/projeção da nuvem na faixa de altura do robô), e
um clique 2D `(x, y)` da GUI precisa virar goal no mundo 3D (trivial se o
mapa 2D é projeção consistente do 3D: mesmo frame, z do chão).

## O que isso decide (e o que NÃO decide)

**Decide:**
- A GUI web continua sendo a interface — não haverá RViz/ferramenta 3D como
  operação normal.
- O investimento de pesquisa vai pra localização 3D; não vamos "portar" o
  AMCL/stack 2D do robô 1 como solução definitiva.
- Nav2, localização e movimentação serão repensadas do zero, com literatura.
  Reuso direto só do que é comprovadamente igual: ponte de rodas/cinemática
  diferencial, GUI, instrumentação CSV, método de trabalho.

**NÃO decide (decisões futuras, uma a uma):**
- Qual LIO (FAST-LIO2 vs Point-LIO vs LIO-SAM vs outro) — exige leitura e
  possivelmente benchmark próprio (bom material de artigo).
- Se o Nav2 continua como framework de navegação (com costmap 2D alimentado
  pela projeção da nuvem) ou se outra abordagem serve melhor.
- Como gerar e manter o mapa 2D da GUI (projeção online da nuvem? mapa 2D
  derivado do mapa 3D do LIO? faixa de altura de corte?).
- Se `/scan` 2D (pointcloud_to_laserscan) sobrevive como insumo de camadas
  de segurança (collision/motion_guard) mesmo com localização 3D.

## Consequências imediatas no repo

- O encanamento do LiDAR serial (LD06) foi removido; `launch.sh` tem um
  placeholder explícito onde o driver do Mid-360 entra (Ethernet).
- A stack Nav2/AMCL herdada fica no repo como REFERÊNCIA e possível
  componente (costmaps/planner são candidatos a reuso), não como caminho
  assumido — nada dela é considerado "pronto" pro robô 2.

## Alternativa descartada

**"Fase 1 clone barato"** (Livox → `/scan` 2D → stack 2D herdada, AMCL e
tudo, robô andando em dias): descartada como ALVO pelo dono — mascararia a
pesquisa (o projeto é PIBIT, o valor está em pensar a navegação 3D do zero)
e criaria inércia em cima de uma arquitetura que não é a final. A projeção
2D continua no radar apenas como possível insumo de segurança/visualização.
