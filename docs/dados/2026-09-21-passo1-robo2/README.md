# Passo 1 da etapa 4 — prova no Gazebo: `footprint_padding` explícito, zero diferença

Capturada em 2026-09-21, commit **`f6e661f`** (`captura/commit.txt`), robô
físico desligado, com o mesmo roteiro e o mesmo comando do passo 0:

```bash
bash bin/linha-de-base-robo2      # pilha.launch.py sim:=true, sem outro argumento
```

## O que mudou no código

Os dois costmaps do `robot_motion/config/nav2.yaml` passaram a declarar
`footprint_padding: 0.009999999776482582`: o valor vivo que o Nav2 já usava
(o `0.01f` padrão, float32 promovido a double), medido no passo 0. **Não**
`0.01`: com ele o parâmetro vivo mudaria em ~2e-10 e a comparação acusaria
diferença. Decisão do dono: o passo 1 fecha **sem nenhuma permissão**.

## Resultado (`comparacao.txt`)

- captura: **APROVADO**, 30/30 nós, 25 dumps, nada duplicado nem a mais;
- `footprint_padding` vivo nos dois costmaps: `0.009999999776482582`;
- parâmetros normalizados contra a linha de base
  (`../2026-09-21-baseline-robo2/02-baseline-aprovada/`): **APROVADO, zero
  diferenças, nenhuma permissão**;
- `grafo_normalizado.txt`: idêntico ao do passo 0;
- `parametros_brutos.yaml`: idêntico byte a byte ao do passo 0 — a
  normalização não precisou esconder nada;
- `logs/erros_launch.txt`: só o mesmo shader GLSL do RViz do passo 0, sem
  impacto; nenhum processo sobrou no desligamento.

## O que isto prova e o que não prova

Prova que tornar o padding explícito **não mudou** nenhum parâmetro efetivo
nem o grafo do robô 2 no Gazebo. Não prova desempenho, geometria nem nada de
hardware; o robô 2 físico não foi tocado (plano da etapa 4, §10).
