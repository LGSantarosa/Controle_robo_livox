# Passo 2 da etapa 4 — prova no Gazebo: `avanco_para_choque`, uma diferença permitida

Capturada em 2026-09-21, commit **`86ca1ed`** (`captura/commit.txt`), robô
físico desligado, mesmo roteiro do passo 0 (`bash bin/linha-de-base-robo2`,
`pilha.launch.py sim:=true` sem outro argumento).

## O que mudou no código

O `path_follower` ganhou `avanco_para_choque` (default 0,28) e o
`vao_frente()` passou a medir dele, em vez de usar `re_recuo_para_choque` (o
para-choque de TRÁS). No robô 2 os dois valem 0,28; a diferença só aparece no
robô 3 (0,0825 contra 0,2913, decisão 052).

## Resultado (`comparacao.txt`), contra a linha de base do passo 0

- captura: **APROVADO**, 30/30 nós, 25 dumps; padding dos dois costmaps
  preservado (`0.009999999776482582`);
- parâmetros normalizados: **APROVADO com uma única diferença**, a permitida
  por `tools/linha_de_base/permitidas/passo2_robo2.yaml` (versionada ANTES da
  captura): `/path_follower:avanco_para_choque`, `entrou`, 0,28. Nenhuma
  permissão sem uso, nenhuma diferença não autorizada;
- `grafo_normalizado.txt`: idêntico;
- `parametros_brutos.yaml`: uma linha nova, `avanco_para_choque: 0.28`, e mais
  nada;
- `logs/erros_launch.txt`: só o shader GLSL do RViz de sempre; nenhum processo
  sobrou.

## O que isto prova e o que não prova

Prova que o parâmetro novo entrou com o valor de hoje e **nada mais** mudou no
robô 2 no Gazebo. Não prova o `vao_frente()` com avanço ≠ recuo — isso é o
teste unitário `test_avanco_para_choque.py`; no robô 2 os dois são iguais, e o
comportamento de navegação não foi exercido (sem objetivo enviado).
