# 002 — Rota de navegação: clone 2D primeiro, LIO 3D depois (revisa a 001)

**Data**: 2026-07-15 · **Status**: aprovada pelo dono.
**Revisa**: a decisão [001](001-gui-2d-localizacao-3d.md) — o *conceito*
(humano opera em 2D, pesquisa de localização 3D) permanece; o que muda é a
**ordem de execução**: a "fase 1 clone barato", descartada como ALVO na 001,
volta como **ETAPA** deliberada no caminho até o LIO.

## Contexto novo (o que mudou desde a 001)

1. **Meta final explicitada pelo dono**: o robô 2 deve ir de um ponto a
   outro sem bater, igual ao robô 1. Esse é o critério de sucesso do
   projeto — não há requisito de capacidade além disso.
2. **Time**: o trabalho passa a ser de 3 pessoas. Duas nunca usaram ROS;
   o dono tem ~6 meses de experiência e se considera iniciante. Divisão de
   tarefas e aprendizado viram requisitos do método, não detalhes.
3. Robô 2 ainda não anda nem sente (bridge e driver Livox pendentes) —
   qualquer rota passa pelos mesmos marcos iniciais.

## A decisão

Chegar à meta (ponto a ponto sem colisão) **primeiro pela rota 2D
comprovada** no robô 1, e só então evoluir a localização para LIO 3D,
comparando as duas — a comparação vira resultado do artigo.

Escada de marcos (cada um demonstrável):

- **M0 — o robô existe pra gente**: NUC (Ubuntu/Jazzy, ssh) + inspeção de
  bancada (checklist no DIARIO 07-15; destrava BO-1, a ponte PC↔placa).
- **M1 — anda na mão**: ponte PC↔hover + calibração da cinemática
  diferencial. Entrega: dirigir pela GUI/teclado.
- **M2 — sente**: `livox_ros_driver2` + nuvem no RViz +
  `pointcloud_to_laserscan` → `/scan`. Independente do M1 (paralelizável).
- **M3a — sabe onde está (2D)**: odometria de roda + slam_toolbox/AMCL
  sobre o `/scan` derivado — a receita do robô 1.
- **M4 — ponto a ponto sem bater**: Nav2 + collision monitor. **Meta
  atingida.**
- **M5 — evolução de pesquisa (o coração PIBIT)**: LIO 3D (FAST-LIO2 /
  Point-LIO / LIO-SAM — decisão futura com literatura) substituindo o AMCL
  como fonte de pose, com a GUI 2D intacta (conceito da 001). Benchmark
  M3a vs M5 no mesmo robô/ambiente = material de artigo.

## Por que revisar a 001

A 001 descartou o clone 2D como alvo por medo de que ele "mascarasse a
pesquisa" e criasse inércia. Os fatos novos invertem o risco dominante:
com time iniciante e o robô ainda inerte, o risco maior passa a ser
**nunca chegar ao LIO por falta de base** (robô que não anda, time sem
tração, projeto parado em leitura). A rota 2D:

- dá vitórias demonstráveis cedo (motivação e aprendizado de ROS na
  prática pros 2 novatos);
- constrói exatamente a infraestrutura que o LIO vai precisar (base
  andando, driver Livox, TF, Nav2, GUI);
- e transforma o clone de "concorrente da pesquisa" em **baseline
  experimental**: sem M3a funcionando, a comparação do artigo não existe.

## Alternativas descartadas

- **LIO 3D direto (ordem original da 001)**: máxima fidelidade à pesquisa,
  mas meses sem robô andando, curva de aprendizado brutal pra quem nunca
  viu ROS, e sem baseline pra comparar o LIO depois.
- **Decidir só depois do M2**: adiaria a organização do time agora, que é
  o gargalo real — e M0–M2 são idênticos nas duas rotas mesmo, então
  esperar não compraria informação que mude a escolha.

## Consequências

- A stack Nav2/AMCL herdada deixa de ser só "referência" (001) e volta a
  ser **caminho de execução do M3a/M4** — com os params marcados como
  "constante de skid a recalibrar" (MIGRACAO_LIVOX item 5).
- A fila de leitura LIO (docs/REFERENCIAS.md) continua, mas sem bloquear
  M0–M4; vira a preparação do M5.
- Divisão por frentes (A base/motores, B percepção/Livox, C infra/GUI)
  registrada no ESTADO; detalhes de trabalho a 3 (branches, revisão) a
  combinar com o time.
