# 022 — O AMCL substitui a TF fixa, e os dois nunca convivem

**Data**: 2026-08-12 (dev, robô desligado) · **Branch**: `slam-meu-mapa`
**Status**: implementada, **não rodou em lugar nenhum** — nem no simulador
**Toca**: `pilha.launch.py`, `config/localizacao_amcl.yaml`
**Vem de**: decisão 021 (a fatia 2D), e fecha o par com ela

## Contexto

A decisão 015 pôs o robô real para navegar **sem mapa**, e isso foi certo: o
único mapa que a pilha tinha era o da pista simulada. O custo aceito na época
foi **memória curta** — janela rolante de 20 × 20 m, e o que o robô nunca viu
conta como livre. Para 2 m na sala, basta. Para o corredor e o andar, não.

Existe um mapa do andar feito por SLAM. Mas ele não entra sozinho: hoje a pilha
publica `map → odom` **fixo, identidade** (`tf_map_odom`), o que com um mapa de
verdade significa *o robô acredita estar eternamente na origem do mapa*. Paredes
fantasma no lugar errado, misturadas com marcação real do Livox desde a 014.

**Mapa e localização-contra-mapa são pacote fechado**, e esta decisão é a
segunda metade dele.

## A decisão

`localizacao:=fixa|amcl` na `pilha.launch.py`.

```
fixa   tf_map_odom publica identidade          (o que existe hoje)
amcl   nav2_amcl casa /scan contra o mapa e publica map → odom de verdade
```

**Default `fixa` nos dois mundos.** Nada do AMCL rodou ainda, nem no simulador —
então ele entra **opt-in**, exatamente como entraram o preditor de Smith (06-08)
e o estimador do ff (016). Os dois foram medidos depois, e um dos dois perdeu.

⚠️ **Quando estiver provado, o default deveria SEGUIR O MAPA**, pela regra da
019: o caso perigoso é que exige intenção, e "mapa sem localização" é o caso
perigoso. Hoje ele é seguro apenas por acidente — o único mapa que sobe por
padrão é o da pista simulada, cujo mundo sai da mesma planta.

### As três travas, e cada uma tem teste

🔴 **1. Exclusão mútua.** O AMCL e o `tf_map_odom` publicam a **mesma**
transformada. Juntos, a pose **pisca** entre "identidade" e a verdade a cada
consulta — e o sintoma (robô em ziguezague no RViz, plano que salta) não aponta
para TF nenhuma. A launch condiciona um ao complemento do outro.

🔴 **2. O AMCL entra na lista do `lifecycle_manager`.** Ele é nó de ciclo de
vida e **não ativa sozinho**: fora da lista ele sobe, fica em `unconfigured` e
não publica TF — e como o publicador fixo também não subiu (trava 1), a árvore
fica partida e o Nav2 inteiro não ativa. **Silêncio total**, que é o modo de
falha mais caro deste projeto (foi assim que o teste D morreu em 06-08).

🔴 **3. `amcl` sem mapa morre na subida.** Não é pilha degradada, é pilha que
não funciona: sem `map_server` o AMCL espera um mapa que nunca vem. A launch
recusa a combinação com a frase certa, em vez de abortar trinta segundos depois
com cara de bug de código.

### A pose inicial vem por parâmetro, e isso não é conveniência

🔴 **O NUC não tem tela.** O jeito normal de dizer ao AMCL onde o robô está é
clicar "2D Pose Estimate" no RViz — que no robô real não existe (desde a 019 o
rviz nem sobe por padrão lá). Sem pose inicial o filtro nasce espalhado pelo
mapa inteiro e converge para qualquer lugar, ou para lugar nenhum.

```
pose_x  pose_y  pose_yaw     [m, m, rad], default = origem do mapa
```

### Os números do filtro, e de onde saem

- **`DifferentialMotionModel`** — 2 rodas e uma boba. O modelo omnidirecional
  gastaria partícula em pose que este robô não alcança;
- **`alpha1..4 = 0,1`**, baixos porque o nosso `odom` é **LIO**, não roda (a
  odometria de roda deste robô é `open_loop` e mente). Baixos, **não zero**: em
  10-08 a planta derivou 19% em 5 min, e desconfiança zero faz o filtro colapsar
  num ponto e nunca se recuperar;
- **`laser_min/max_range` = 0,35 / 20,0**, iguais aos cortes da fatia 2D. Pedir
  mais longe que o scan entrega faz o AMCL tratar "não medi" como "medi longe" —
  feixe fantasma atravessando parede. Os dois números moram em **pacotes
  diferentes**, que é a distância em que número copiado envelhece: há teste
  lendo os dois arquivos.

## Alternativas consideradas

**(a) `slam_toolbox` em modo localização, em vez de AMCL.** Casa scan contra o
grafo em vez de contra a grade, e dispensaria a conversão para PGM. Descartada
por ora: é uma peça a mais para diagnosticar numa cadeia que ainda não rodou
inteira uma vez. O AMCL é o caminho mais curto entre "temos mapa" e "o robô se
acha", e o mapa já está em PGM.

**(b) Deixar o AMCL como default assim que entrar.** Descartada pela regra do
projeto: nada vira padrão antes de rodar. Ver o estimador do ff, cuja previsão
falsificável **passou** e cujo veredito ainda foi negativo.

**(c) Manter o `tf_map_odom` de pé "por segurança", como reserva.** É
exatamente o defeito que a trava 1 impede. Reserva que publica a mesma TF não é
reserva, é disputa.

## Como isto se prova

Nove testes novos, quatro verificados por mutação: o publicador fixo volta a
subir sempre; o AMCL sai da lista do lifecycle; o alcance diverge da fatia; a
recusa deixa passar `amcl` sem mapa. Os quatro derrubam o teste esperado.

⏳ **Nada rodou.** O próximo passo é o simulador, e vale saber de antemão o que
ele responde:

| pergunta | o Gazebo responde? |
|---|---|
| o `/scan` casa com o `meu_mapa`? | 🟢 sim — e é o que mais dá erro |
| a pilha sobe com `map_server` + AMCL sem brigar? | 🟢 sim |
| o AMCL converge com odometria que **deriva**? | 🔴 **não** |

A terceira não, e é importante não vendê-la: no simulador o `/Odometry` é a pose
**verdadeira** do Gazebo. Com odometria perfeita o AMCL não tem o que corrigir e
vai parecer ótimo — é o simulador otimista de sempre, na pior versão. Para
atacar essa terceira seria preciso injetar deriva na pose simulada de propósito,
e isso é decisão própria, ainda não tomada.

## Referências

- `docs/decisoes/021-a-fatia-2d-que-o-amcl-consome.md` — a outra metade do par
- `docs/decisoes/015-nav2-sem-mapa-no-robo-real.md` — a memória curta que isto
  vem resolver
- `docs/decisoes/019-o-default-e-o-caso-seguro.md` — a regra que decidirá o
  default quando isto estiver provado
