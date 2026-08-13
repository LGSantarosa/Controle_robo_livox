# 028 — O robô desenha o próprio mapa, porque o do estágio foi limpo

**Data**: 2026-08-13 (robô ligado a sessão inteira)
**Status**: aceita, implementada e **medida no robô**
**Toca**: `robot_motion/launch/mapeia.launch.py` (novo), `maps/sala_andar3/`
**Vem de**: o passo 6 do `docs/ROTEIRO_NAV2_NO_ROBO.md` (mapa + AMCL)

## O problema, medido

O roteiro mandava localizar contra `maps/andar3/scan_andar3_ajustado.yaml`,
escolhido em 12-08 **por medida** entre os mapas que o robô do estágio usa. Com
a pilha de pé, o AMCL `active` e a pose semeada onde o dono disse que o robô
estava, o casamento entre a fatia 2D e a planta deu:

```
pose semeada (-6,5 · -2,5)   47,2% dos feixes sem parede nenhuma perto
                             erro mediano 0,300 m
```

A suspeita óbvia era pose errada. Para separar "pose errada" de "mapa errado"
sem depender de chute, varri **todas** as poses possíveis dentro da sala
(transformada de distância do mapa, (x, y) a cada 10 cm, yaw a cada 3°,
refinamento a 2 cm e 0,5°), pontuando pela fração de feixes a menos de 10 cm de
alguma parede:

```
melhor pose POSSÍVEL na sala   41,4% a menos de 0,15 m, erro mediano 0,300 m
esperado (12-08, simulador)    100% dentro de 0,15 m, mediano 0,050 m
```

**Se a melhor pose possível reprova, o problema não é a pose.**

## A causa

O `scan_andar3_ajustado` é a versão **limpa** do `scan_andar3` cru. O próprio
`maps/andar3/README.md` registra a limpeza como virtude: folga p10 de 0,212 →
0,400 m, "corpo não cabe" de 13,3% → 7,5%. O que ninguém tinha escrito é **o
que foi limpo**: a mobília.

A sala real do andar 3 está cheia de coisa. O dono, quando eu ainda insistia em
achar a pose: *"ela ta cheia de coisa, mas no mapa foi tirado"*.

Planta limpa contra sala mobiliada não fecha em pose nenhuma. Cada móvel é um
feixe que o mapa não explica, e o AMCL não tem o que casar.

## A decisão

**O robô desenha o próprio mapa da sala onde vai andar**, por teleop, com
`slam_toolbox`. Nasce `robot_motion/launch/mapeia.launch.py`.

O que ela sobe, e o que ela recusa a subir:

| sobe | não sobe | por quê |
|---|---|---|
| `slam_toolbox` (mapping) | `amcl`, `map_server` | os dois publicariam `map → odom` junto com o SLAM |
| `twist_mux` | `tf_map_odom` | idem: identidade brigando com a correção do SLAM |
| `compensador_rumo` | Nav2, `path_follower`, `heading_controller` | autonomia enquanto se mapeia é robô perseguindo mapa que muda debaixo dele |

O `compensador_rumo` fica porque o robô comandado reto arca −0,91 1/m (decisão
011): sem ele o operador luta contra a curva do próprio robô enquanto tenta
desenhar uma parede reta, e o mapa sai torto **por causa do atuador**.

## O resultado, contra o mesmo scan

```
                             <0,15 m de parede   erro mediano
mapa do estágio (limpo)           41,4%             0,300 m
mapa desenhado hoje               97,7%             0,050 m
depois, com AMCL travado          99,7%             0,000 m
```

## As alternativas descartadas

**Editar o mapa do estágio à mão, pondo a mobília de volta.** Descartada: a
mobília muda de lugar, e mapa mantido à mão envelhece em silêncio. O robô
redesenhar leva 10 minutos e é repetível.

**Usar o `mapa_3_andar`, o outro recorte.** Descartada pelo mesmo motivo: é
outro corte da MESMA planta limpa.

**Subir a tolerância do AMCL até o mapa limpo "fechar".** Descartada, e é a pior
das três: afrouxar o modelo de medida para aceitar um mapa que não descreve o
mundo compra convergência com localização que mente. Reflexo que dispara sempre
não quer dizer nada (027); AMCL que aceita tudo, idem.

**Ficar no `localizacao:=fixa` e navegar sem mapa.** Descartada porque é o que
já se fazia: sem mapa global o costmap é janela de 20 m, e foi por isso que o
plano para 4 m deu VAZIO em 12-08.

## As armadilhas que esta decisão paga (as três custaram tempo hoje)

1. **`use_lifecycle_manager: False` NÃO faz o `slam_toolbox` se auto-ativar** no
   Jazzy. Ele nasce `unconfigured` e fica lá, **calado** — sem erro, sem aviso,
   e o único sintoma é `/map` que nunca aparece. Entrou um
   `nav2_lifecycle_manager` com lista de **um** nome (lista curta de propósito:
   servidor da lista que não responde derruba o bringup inteiro, 06-08).
2. **O `map_saver_cli` estoura o timeout default de 2 s** neste NUC:
   `--ros-args -p save_map_timeout:=30.0`. O erro é `Failed to spin map
   subscription`, que não parece timeout.
3. **A pose inicial envelhece.** Uma correção calculada de um scan de minutos
   antes foi rejeitada pelo filtro (o AMCL assentou 0,26 m ao lado). Capturar,
   buscar e empurrar `/initialpose` **sem intervalo**, com o robô parado.

## Como se falsifica

Rodar `tools/banco/casa_scan.py --pose amcl` contra o mapa novo, com o robô
parado em ponto qualquer da sala. Se cair abaixo de ~90% dentro de 0,15 m, o
mapa envelheceu (mobília mudou de lugar) e se redesenha. Custa 10 minutos.

## Referências

- decisão 021 (a fatia 2D que o AMCL come) — os dois números de altura que
  decidem o que vira "parede";
- decisão 022 (AMCL) e 019 (a TF que faltava);
- `maps/andar3/README.md` — a régua que escolheu o mapa do estágio, e que esta
  decisão aposenta para ESTA sala.
