# Passo 3 da etapa 4 — prova no Gazebo: o robô 2 montado pelo perfil, zero diferença

Capturada em 2026-09-21, commit **`5659dd2`** (`captura/commit.txt`), robô
físico desligado, mesmo roteiro dos passos anteriores (`bash
bin/linha-de-base-robo2`, `pilha.launch.py sim:=true` sem outro argumento).

## O que mudou no código

`robot_motion/perfil.py`: `parametros(2, share)` devolve o que cada nó recebe
— `nav2` e `collision_monitor` (os mesmos YAMLs de antes), `nav2_rewrites: {}`
e `path_follower: {}`. Qualquer outro robô levanta `RoboSemPerfil`.

`pilha.launch.py`: monta o robô 2 pelo perfil, **estático** (escolher o perfil
pelo argumento é a etapa 6); ganhou o argumento `robo` (default `"2"`) e o
`_recusa_robo`, antes de qualquer ação: só o texto exato `"2"` passa, `"3"`
recusa dizendo que a pilha do robô 3 é a etapa 6, o resto recusa como robô
inexistente. `nav2_rewrites` não vazio derruba a subida (a pilha ainda não
as aplica); a sobreposição vazia do `path_follower` deixa a lista de
parâmetros dele literalmente a de antes.

## Resultado (`comparacao.txt`), contra a baseline v2, SEM permissão

A v2 já contém o `avanco_para_choque` do passo 2, então este passo não leva
permissão nenhuma.

- captura: **APROVADO**, 30/30 nós, 25 dumps, `dumps_vazios: []`; padding dos
  dois costmaps preservado (`0.009999999776482582`);
- ilegíveis: os seis conhecidos (2 do `collision_monitor`, 4 do
  `controller_manager`), todos permitidos; nenhum inesperado, nenhuma exceção
  sem uso; `ilegiveis.yaml` idêntico ao da v2;
- parâmetros normalizados: **APROVADO**, nenhuma diferença;
- `grafo_normalizado.txt`: idêntico;
- `parametros_brutos.yaml`: idêntico (mais forte que o critério, que era o
  normalizado).

Na trava do §8 (`tools/linha_de_base/argumentos_launch.yaml`, não regerada), a
única diferença da extração é a entrada nova permitida, `robo: '2'` na pilha.

## Anotações do console, ambas já presentes na v2

- `logs/erros_launch.txt`: uma linha, o shader GLSL do RViz de sempre
  (`glsl120/indexed_8bit_image.vert`);
- na derrubada, `ros2 launch` e `ros2 bag record` precisaram de `kill -9` do
  wrapper — igual à v2; terminou `🟢 limpo`.

## O que isto prova e o que não prova

Prova que passar o robô 2 pelo perfil não mudou nenhum parâmetro nem nó no
Gazebo, e que o argumento novo nasceu com o default certo. Não prova a recusa
de `robo:=3`/inválido na subida real (isso é `test_pilha_robo.py`, num
`LaunchContext` de verdade), nem desempenho, geometria ou hardware — nenhum
objetivo foi enviado.
