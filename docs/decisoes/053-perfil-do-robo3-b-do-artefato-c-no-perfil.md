# 053 — Perfil do robô 3: geometria lida do artefato, política em chaves próprias

**Data**: 2026-09-22 (PC de dev, robô desligado)
**Status**: aplicada no perfil (`ca973c9`, `7af0ce1`, `d75b65f`); **ainda não
sobe** — a pilha recusa `robo:=3` até a etapa 6 (D1), e nenhum costmap do
robô 3 roda nesta etapa.
**Toca**: `robot_base/config/geometria_robo3.yaml` (`raio_varrido_pivo`),
`robot_motion/robot_motion/perfil.py` (ramo `3`, `aplica_reescritas`),
`robot_motion/config/perfil_robo3.yaml` (novo), `robot_motion/launch/pilha.launch.py`,
`robot_motion/package.xml` (`python3-yaml`), testes `test_urdf_robo3.py`,
`test_reescritas.py`, `test_perfil_robo3.py`, `test_perfil.py`, `test_pilha_robo.py`.
**Vem de**: decisão 052 (artefato geométrico), `PLANO_ETAPA4_ROBO3.md` §§2–5 e
§9 passo 4, D2 e D3 aprovadas pelo dono.

---

## 1. O que foi decidido

1. **(b) nunca mora no perfil.** O `perfil_robo3.yaml` só tem a *referência*
   ao artefato da 052 (`geometria.arquivo`), sem nenhum número. Footprint,
   polígonos do reflexo e os seis do `path_follower` são **calculados** pelo
   `perfil.py` a partir do artefato + das margens de (c).
2. **(c) em chaves separadas, uma por função**, cada uma com `classe`,
   `origem` e a etapa que `fecha`: `footprint_padding` (D2), `approach_margem`,
   `stop_margem` (D3), `passagem_margem`, `desencalhe_pivo_margem`. Nenhuma é
   somada à geometria e guardada como número só.
3. **Os costmaps e o reflexo recebem por reescrita do YAML, por caminho
   completo.** Os costmaps são nós dentro dos servidores do Nav2: dicionário
   passado ao `Node` do servidor não chega neles (plano §3).
   `aplica_reescritas` troca só **folha que já existe**; caminho inexistente,
   que para num ramo ou que atravessa folha reprova o lote inteiro, sem
   efeito parcial. Criar chave em silêncio seria o nó lendo um parâmetro que
   ninguém declarou.
4. **A pilha ainda não aplica reescrita e reprova se o perfil pedir uma** —
   agora também para `collision_monitor_rewrites`, não só `nav2_rewrites`.
   Reescrita pedida e não aplicada seria costmap ou reflexo lendo o arquivo
   sem ela, em silêncio.
5. **Partição fechada da base.** Toda folha numérica do `nav2.yaml`, do
   `collision_monitor.yaml` e dos defaults do `path_follower` que o perfil não
   sobrescreve está em `herdados_provisorios` (medida/sintonia do robô 2, com
   a etapa que fecha: 7, 8, 9 ou 10) ou em `independentes_do_robo` (com
   motivo). Parâmetro novo na base sem classificação reprova.
6. **`python3-yaml` declarado** no `package.xml`: o `perfil.py` lê YAML em
   produção, e dependência transitiva do ambiente não é contrato (apontado na
   revisão).

## 2. O `raio_varrido_pivo` (entrou no artefato da 052, justificado aqui)

O `desencalhe_pivo_folga` precisa do raio que o **corpo** varre girando no
`base_link`. É geometria (b), então foi para o artefato, sem folga:

    bobas:   √(0,2485² + 0,105²) + 0,04272 = 0,31249 → 0,3125 (arredondado para FORA)
    corpo rígido (quina traseira da caixa): 0,2760 — não alcança

**Não é a quina do footprint** (0,348). Aquela é área vazia da caixa
delimitadora (052 §2.2), e usá-la seria 3,5 cm de folga escondida. O nome é
diferente de `raio_min_curva` de propósito: um é o que o corpo ocupa, o outro
é a curva da trajetória. O teste calcula o raio do URDF e reprova se o
artefato ficar abaixo da envolvente, com folga embutida, ou a menos de 3 cm
da quina vazia.

Não virou decisão própria: é consequência direta da 052 e já estava no plano
(§5, linha do pivô).

## 3. Os valores montados (conferidos na revisão do dono)

| consumidor | (b) do artefato | (c) do perfil | montado |
|---|---|---|---|
| footprint, 2 costmaps | polígono da 052 | `footprint_padding` 0,009999999776482582 | polígono da 052 |
| `PolygonApproach` | footprint | +0,03 por face; 0,75 s | x +0,1125 / −0,3213, y ±0,220 |
| `PolygonStop` | footprint | frente 0,133, lados 0,05, trás 0,05 | x +0,2155 / −0,3413, y ±0,240 |
| `passagem_meia_largura` | 0,190 | `passagem_margem` 0,03 (chave própria) | 0,190 |
| `re_largura` | 2 × 0,190 | + 2 × lado do Stop | 0,480 |
| `re_recuo_para_choque` | 0,2913 | folga fica no `re_folga` | 0,2913 |
| `avanco_para_choque` | 0,0825 | — | 0,0825 |
| `desencalhe_pivo_folga` | 0,3125 | + 0,020 | 0,3325 |

Montado também a partir do `install/` (via `ament_index`), com os mesmos
números.

## 4. O que isto prova e o que NÃO prova

- **Prova:** o que chega ao YAML reescrito — o arquivo que o nó carregaria —
  e que nenhuma outra folha dos dois YAMLs muda.
- **Não prova** que o Nav2 aceita o footprint nem o padding (texto × lista,
  float32). Isso é critério herdado da **etapa 6**: `ros2 param get` vivo nos
  dois costmaps quando a pilha do robô 3 subir no Gazebo.
- **Não prova nada sobre o robô físico.** Todas as margens de (c) são
  herdadas do robô 2 e provisórias até a etapa 8 medir a frenagem.

## 5. Alternativas

| alternativa | por que não |
|---|---|
| `nav2_robo3.yaml` inteiro, copiado e editado | dois arquivos divergem em silêncio; vértice redigitado (052 §2.5 proíbe) |
| dicionário de parâmetros no `Node` do servidor | não chega nos costmaps, que são nós próprios (plano §3) |
| `RewrittenYaml` do `nav2_common` já agora | a pilha ainda não sobe o robô 3; aplicar fica para quando houver o que provar vivo. A função pura dá o teste sem ROS |
| reescrita que cria a chave se faltar | parâmetro não declarado e valor sumindo sem aviso |
| margem somada à geometria num número só | mistura (b) com (c); some dos dois lugares (plano geral §5) |
| quina do footprint como raio do pivô | área vazia da caixa: 3,5 cm de folga escondida |
