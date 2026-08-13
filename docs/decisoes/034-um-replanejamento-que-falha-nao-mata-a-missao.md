# 034 — Um replanejamento que falha não pode matar a missão

**Data**: 2026-08-14 (dev, robô carregando; medido no Gazebo)
**Status**: aceita e implementada, verificada em 3 corridas
**Toca**: `behavior_trees/replanejamento_com_suavizacao.xml`
**Vem de**: três corridas seguidas morrendo no mesmo ponto, sempre pelo mesmo
motivo

## O defeito

Na árvore, `ComputePathToPose` estava solto dentro de uma `Sequence`. Falha dele
= falha da sequência = falha do `RateController` = falha da `PipelineSequence` =
**`Goal failed`**. Um replanejamento ruim de meio segundo matava a navegação
inteira.

E ele acontece, com nome e lugar fixos. Medido três vezes:

```
GridBased plugin failed to plan from (4.39, 1.63) to (6.24, 3.51):
"Either of the start or goal pose are occupied"
```

O robô entra numa quina com 0,30 m de folga no mapa estático; as marcas **ao
vivo** do Livox fecham a célula em que ele está; o planejamento falha por um
ciclo. Resultado antes desta decisão: `Goal failed`, e o seguidor parado para
sempre a 2,3 m do alvo — em uma das corridas, 46 s imóvel até o teto do teste.

## A decisão

`ComputePathToPose` passa a viver dentro de um `RecoveryNode` com
`AlwaysSuccess` de reserva — exatamente o padrão que o `SmoothPath` já usava na
mesma árvore, e pelo mesmo motivo:

```xml
<RecoveryNode number_of_retries="1" name="PlanejaSePuder">
  <ComputePathToPose .../>
  <AlwaysSuccess/>
</RecoveryNode>
```

Falhou, `{path}` fica com o último plano válido e o próximo tique (1 Hz) tenta
de novo.

### 🔴 E é isto que torna a ré ALCANÇÁVEL

Este é o ponto que só apareceu ligando as duas coisas. Desde a decisão 031 a ré
do seguidor exige **objetivo vivo**. Se a árvore aborta no mesmo instante em que
o robô encalha, a recuperação nunca chega a existir: quando ela seria útil, o
objetivo já morreu.

```
antes   encalha -> planner falha -> árvore aborta -> objetivo morto -> ré inerte
agora   encalha -> planner falha -> árvore SEGURA  -> objetivo vivo  -> ré pode
```

"Objetivo vivo + emperramento" é exatamente a condição que a 031 espera. As duas
decisões só funcionam juntas.

### Por que não vira loop infinito

Duas travas já existentes, nenhuma nova:

- o seguidor **para sozinho** com plano velho de 2 s (`timeout_plano`);
- o `progress_checker` do `FollowPath` derruba o objetivo depois de 30 s sem
  avanço (`movement_time_allowance`).

Ou seja: tolera o tropeço, não tolera o travamento permanente.

## Alternativa considerada

**Usar uma árvore de fábrica com recuperação** (`navigate_to_pose_w_replanning_
and_recovery.xml`). Descartada pela mesma razão de 29-07, que continua válida: a
recuperação de fábrica é `spin` (pivô — a placa não entrega módulo, decisão 023)
e `backup` (a ré que a decisão 009 tirou do Nav2). Pior: as duas seriam **no-op**,
porque o `cmd_vel` delas sai pelo tópico ignorado, e a árvore acharia que
recuperou sem nada ter acontecido. Quem recupera é o seguidor.

## A medida

Antes: 3 corridas, 3 mortes por `Goal failed`, nenhuma chegou.
Depois (junto com as decisões 032 e 033): 3 corridas, **3 chegadas**, em
39,7 / 40,0 / 40,4 s, com folga mínima de 0,35 m.

⚠️ Sozinha esta decisão não faz o robô chegar — ela faz o robô continuar
tentando. Quem destravou a quina foi a 033.

## Referências

- decisão 031 — a ré só existe com objetivo vivo (o par desta)
- decisão 026 — o plano passa a ser suavizado (de onde veio esta árvore)
- dados: `docs/dados/2026-08-14-sim-porta/`
