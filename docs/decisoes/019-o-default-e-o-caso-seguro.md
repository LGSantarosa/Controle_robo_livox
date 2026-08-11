# 019 — O default é o caso seguro; o perigoso exige intenção

**Data**: 2026-08-11 (dev, robô desligado)
**Status**: aceita e implementada
**Contexto**: abre a leva de navegação, pedida pelo dono para andar mais rápido

## O problema

Duas configurações deste repo eram **obrigatórias no robô real e opcionais na
sintaxe**, o que na prática significa "alguém vai esquecer":

| o que | o que acontecia ao esquecer | quem avisava |
|---|---|---|
| `tf_odom -p frame_da_pose:=livox_frame` | o nó sobe, **não publica TF**, os quatro servidores do Nav2 não ativam e o `lifecycle_manager` aborta o bringup | uma linha de ERROR no meio do log da base |
| `pilha.launch.py mapa:=nenhum` | o robô real ganha o mapa da **pista simulada** (12 × 8 m que não existem) e mistura parede fantasma com marcação real do Livox | ninguém |

Nenhum dos dois é hipótese: o `tf_odom` subiu inerte em **três sessões** (06-08,
08-10 e 11-08) e nas duas últimas alguém teve de matá-lo e subir na mão. O mapa
fantasma foi diagnosticado na 015 e a decisão parou em criar a opção — deixou a
escolha com o operador.

⚠️ **O padrão comum é mais importante que os dois casos**: o valor perigoso era
o default, e o seguro exigia digitação. É a mesma forma do defeito da bitola
(29-07) e do `ff HERDADO` (07-08) — configuração que envelhece ou falta e não dá
sintoma no lugar onde é decidida.

## Alternativas consideradas

1. **Documentar melhor** (o que a 015 fez, e o que o roteiro repetia em três
   lugares). Descartada: o roteiro já dizia, com destaque vermelho, e o erro
   aconteceu mesmo assim em duas sessões seguidas. Documento não é trava.
2. **Fazer o nó falhar duro** (`tf_odom` abortando a subida sem frame válido).
   Descartada por ora: derrubaria a base inteira por uma escolha que tem um
   default correto óbvio, e o nó **já faz a coisa certa** ao se recusar a
   publicar TF errada — o defeito não era dele.
3. **Default seguro dependente do `sim`** (a escolhida): a launch decide pelo
   contexto, e quem quiser o caso perigoso passa o argumento explicitamente.

## O que entrou

```
localizacao.launch.py   frame_da_pose  default 'livox_frame' (era vazio)
pilha.launch.py         mapa           default 'nenhum' se sim:=false
pilha.launch.py         rviz           default 'false'  se sim:=false
```

O `rviz` veio junto pelo mesmo raciocínio: o NUC não tem tela e já gasta CPU
convertendo ~11 mil pontos por quadro.

**Seis testes novos**, e a história deles é o registro mais útil desta decisão:

🔧 **A primeira versão do teste de resolução PASSOU com o condicional
invertido.** Ela montava a expressão `PythonExpression` dentro do próprio teste
para provar o sentido do `if` — ou seja, testava uma cópia, não o alvo. A
mutação (trocar os ramos do condicional na launch) não derrubou nada. A versão
que ficou lê o `default_value` **do arquivo**, via AST, e o resolve com um
`LaunchContext` de verdade; com ela a mesma mutação falha.

➡️ **Teste que reconstrói o alvo não testa o alvo.** Vale para toda checagem de
config deste repo — a 017 caiu num parente disso (todos os testes conferiam que
os consumidores concordavam entre si, e nenhum perguntava quem publicava).

## Consequências

- a base sobe com a árvore TF fechada **sem intervenção**, que é a pré-condição
  de tudo em navegação;
- `ros2 launch robot_motion pilha.launch.py` no robô real passa a ser seguro
  sem argumento nenhum;
- ⚠️ **dívida que continua aberta**: `livox_frame` embute os ~5 cm entre a IMU
  (`body`) e o lidar. O conserto certo é o URDF descrever `body`, e aí o default
  passa a ser `body`.

## Referências

- `docs/decisoes/015-nav2-sem-mapa-no-robo-real.md` — criou o `mapa:=nenhum`
- `docs/decisoes/017-a-nuvem-que-a-percepcao-consome.md` — o parente do buraco
  de teste
- `docs/DIARIO.md`, 06-08, 08-10 e 11-08 — as três sessões em que o `tf_odom`
  subiu inerte
