# 035 — A árvore não desiste, e a recuperação não move roda

**Data**: 2026-08-14 (2ª leva; dev + Gazebo, com o dono olhando)
**Status**: implementada, **não medida em corrida completa** — o dia acabou antes
**Toca**: `behavior_trees/replanejamento_com_suavizacao.xml`, `config/nav2.yaml`
**Vem de**: o dono, vendo o robô abandonar objetivo: *"faça essa PORRA desse robô
não desistir dos goals, leia como o robo do Controle_robo_web faz e faça igual"*

## O defeito

Objetivo morria por três caminhos, e cada um matava a missão inteira:

```
1. progress_checker      "Failed to make progress" -> FollowPath aborta
2. ComputePathToPose     falha em rajada (marca ao vivo fecha a célula do robô)
3. FollowPath            aborta por qualquer outro motivo
```

E morrer é pior do que parece: **objetivo morto desliga a recuperação**. A ré do
seguidor exige objetivo vivo desde a decisão 031 — se a árvore aborta no mesmo
instante em que o robô encalha, a única recuperação que existe nunca acontece.

## As duas tentativas que FALHARAM antes da certa

Registradas porque as duas parecem razoáveis e as duas pioraram:

**(a) `RecoveryNode number_of_retries="1"` só no `ComputePathToPose`.** Tolera
UMA falha, e o planejador falha em rajada: duas `Aborting handle` seguidas e a
árvore morria igual.

**(b) `ForceSuccess` no planejamento + `RetryUntilSuccessful` no `FollowPath`.**
O decorador no `FollowPath`, dentro de uma `PipelineSequence`, quebra o halt do
cliente de ação:

```
Failed to get result for follow_path in node halt!
Goal failed
```

**A tentativa de "nunca desistir" fez desistir mais cedo.** E com o
`ForceSuccess` sozinho, quem passou a matar o objetivo foi o `FollowPath`
abortando — que a proteção do planejamento não cobre.

Lição: eu estava remendando GALHO. A peça certa é estrutural.

## A decisão: a estrutura do robô 1

Fui ler o `navigate_w_backup_first_recovery.xml` do `Controle_robo_web`, como o
dono mandou. A peça que faz a diferença é uma só:

```
RecoveryNode number_of_retries="1000"  envolvendo a NAVEGAÇÃO INTEIRA
  └ PipelineSequence
      ├ RateController(1 Hz)
      │   └ RecoveryNode(1) Planeja -> recuperação: limpa costmap GLOBAL
      └ RecoveryNode(1)     Segue   -> recuperação: limpa costmap LOCAL
  └ recuperação do conjunto: limpa os DOIS costmaps
```

Mais o `progress_checker` desligado (`movement_time_allowance` 30 s → 1e9): ele
julgava o robô contra o `cmd_vel` de um servidor que **não move roda nenhuma**
nesta máquina.

### O que NÃO foi copiado do robô 1, e por quê

O `RoundRobin` de lá tem `BackUp`, `Wait` e `Spin` além da limpeza. Os três
ficaram fora:

- **`Spin`** é pivô, e a placa não entrega módulo (023);
- **`BackUp`** é a ré do Nav2, que a decisão 009 tirou do caminho — quem recua é
  o seguidor, por gatilho, pelo canal que fura o reflexo (025);
- **`Wait`** exige o `behavior_server`, que esta pilha não sobe.

E os três seriam **no-op**: o `cmd_vel` deles sai pelo tópico que a nossa cadeia
ignora, então a árvore acharia que recuperou sem nada ter acontecido — o BO-3
exato, a classe de defeito que mais custou tempo neste projeto.

### 🟢 A limpeza de costmap resolve um defeito MEDIDO

Não é enfeite copiado. Três corridas de 14-08 morreram com

```
GridBased failed to plan: "Either of the start or goal pose are occupied"
```

porque marcas **ao vivo** do Livox fecharam a célula onde o robô estava. Limpar
o costmap é exatamente o remédio disso, **não move o robô**, e é a primeira
recuperação do robô 1 pelo mesmo motivo.

## O preço, e ele é escolha do dono

Robô genuinamente encalhado agora **tenta para sempre** em vez de reportar
falha. Foi pedido com todas as letras, e é a escolha certa aqui: quem recupera é
a ré, e ela precisa do objetivo vivo.

## O que falta

Rodar uma corrida completa com esta árvore e medir. Ela subiu e a pilha ativou;
o comportamento em corrida **não foi verificado** — a sessão virou para o freio
de giro e depois para o salto de pose do AMCL.

## Referências

- decisão 031 — a ré só existe com objetivo vivo (o par desta)
- decisão 034 — a versão anterior desta, que era o remendo (a)
- `Controle_robo_web`: `robot_nav/behavior_trees/navigate_w_backup_first_recovery.xml`
