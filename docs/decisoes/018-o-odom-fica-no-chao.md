# 018 — O `odom` fica no chão, não na altura do sensor

**Data**: 2026-08-11 · **Estado**: implementado, NÃO visto no robô
**Vem de**: a previsão da decisão 017 · **Toca**: `tf_odom.py`,
`transformadas.py`

## O defeito

Medido no robô em 10-08, logo depois de o `tf_odom` passar a publicar:

```
tf2_echo odom base_link  ->  z = −0,477 m
```

O robô 48 cm **abaixo** da origem do próprio `odom` — os 42 cm da trena mais a
deriva de z do LIO. A causa é a composição: o `tf_odom` compõe a pose do sensor
com o URDF **só à direita**,

```
T(odom → base_link) = P(t) ∘ T(sensor → base_link)
```

o que põe o `base_link` no lugar certo *em relação ao sensor* e deixa a **origem
do `odom`** onde o FAST-LIO a criou: em cima do Mid-360.

A TF não está errada como geometria — ela descreve corretamente onde o corpo
está. Está errada como **convenção**: com ela, o chão fica em z ≈ −0,42.

## Por que isso quebra a percepção, e em dois lugares independentes

Os dois no frame **global**, que é onde o Nav2 filtra:

1. `min_obstacle_height` / `max_obstacle_height` (0,10–0,50 m no nosso YAML)
   rejeitam a nuvem inteira, que passa a chegar em −0,42 ± altura do obstáculo;
2. a `VoxelLayer` descarta o que está abaixo do `origin_z` dela.

**Sintoma: costmap com ZERO células letais e nuvem perfeita** — exatamente o que
o pré-voo de 10-08 mostrou, e que naquele momento tinha uma segunda causa
suficiente (a nuvem em `CustomMsg`, decisão 017). Consertar só a 017 deixaria o
sintoma igual, e é por isso que a 017 registrou esta previsão antes de mexer.

## Decisão

`odom` passa a ser **a pose do `base_link` na largada**, pré-compondo com a
inversa:

```
T(odom → base_link) = T(sensor → base_link)⁻¹ ∘ P(t) ∘ T(sensor → base_link)
```

No plano isso é "tirar os 42 cm", mas a conta é feita com a inversa de verdade
porque **em rampa ela não é uma subtração** — e porque o Mid-360 poderia vir a
ser montado torto.

A propriedade que define a escolha, e que está travada em teste: **qualquer que
seja a montagem do sensor, quando a pose do LIO é identidade o `base_link` está
na origem do `odom`.** É o que põe o chão em z ≈ 0.

⚠️ **A altura MEDIDA continua aparecendo.** Sai o offset do URDF, não o dado: se
o LIO diz que o sensor subiu 3 cm (rampa, deriva), a TF mostra 3 cm.

## Alternativas consideradas, e por que não

**(a) Descer a faixa de altura do costmap em 0,42 m.** Um número em dois YAML e
nenhum código. **Descartado**: enterra a altura do sensor dentro de parâmetros
de percepção, que passam a mentir sobre o que descrevem ("obstáculo entre −0,32
e +0,08 m" não é altura de obstáculo nenhum). Trocar a trena do sensor, ou
montar o Livox mais alto, quebraria a percepção sem tocar em nada de percepção —
a classe de defeito da bitola (29-07).

**(b) Deixar o `odom` onde está e publicar um `base_footprint` no chão.** É a
convenção de muitos robôs, e não resolve: os costmaps trabalham com o
`robot_base_frame`, que é `base_link` em todo o nosso Nav2, e a origem do odom
continuaria alta.

**(c) Não compor nada (publicar a pose do LIO como se fosse do corpo).** É o
erro que o `tf_odom` foi escrito para impedir, e ele é silencioso: 42 cm de erro
em toda a navegação sem uma linha de log.

## Como isto se prova no robô

```bash
ros2 run tf2_ros tf2_echo odom base_link      # z tem de ficar em ~0,00, não −0,48
```

E o teste que junta 017 e 018, que é o único que importa:

```bash
python3 tools/banco/checa_pilha.py            # "local/global costmap marcando"
```

⚠️ **Se os costmaps continuarem em zero com as duas mudanças de pé**, a hipótese
da altura estava errada e a próxima suspeita é a `VoxelLayer` (`origin_z`,
`z_voxels`, `mark_threshold`), não a faixa de altura.

## Referências

- `docs/decisoes/017-a-nuvem-que-a-percepcao-consome.md` — o defeito em série
- `docs/DIARIO.md`, 10-08 (o `z = −0,477` medido) e 11-08
- `ros2_packages/robot_base/test/test_tf_odom.py` — a composição travada, com
  oráculo em matriz e sensor inclinado
