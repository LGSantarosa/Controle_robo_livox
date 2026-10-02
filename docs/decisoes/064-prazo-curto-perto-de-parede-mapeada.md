# 064 — Parede mapeada e frente livre encurtam o detector para 2 s

**Data**: 2026-10-02 (robô 2 e dev)
**Status**: implementada no PC; falta enviar ao NUC e repetir a mesma rota no
chão
**Toca**: somente o prazo do detector de falta de progresso; não altera o
`PolygonStop`, a velocidade nem a manobra de escape
**Vem de**: decisões 060/061 e parada medida na volta da rota longa de 02-10

## O que aconteceu no chão

Sem o compensador real (063), o robô completou várias idas aos pontos 1 e 2 e
uma rota longa de ida e volta. Na volta, o `PolygonStop` zerou o comando final
na porta em `t=1790979417,340`, embora o seguidor continuasse pedindo movimento.
O robô estava praticamente sobre o plano (erro lateral de 0 a 1,5 cm) e o
`/scan` media 2,49 m livres no corredor frontal.

O detector genérico só liberou a recuperação em `t=1790979422,053`: **4,71 s**
depois. O escape reto avançou 0,21 m e o seguimento normal voltou. A ação foi
correta; a espera é que fazia o robô parecer indeciso diante de uma parede que
já existe no mapa.

O robô 1 já separa esse caso: usa 2 s quando há parede mapeada próxima e
conserva o prazo longo para obstáculo desconhecido. O raio usado lá é 0,60 m.

## Alternativas

1. Manter 4 s em todos os casos. É seguro, mas conserva a pausa observada.
2. Trocar o prazo global de 4 para 2 s. Descartada: a ré mais longa medida dura
   2,8 s; um prazo global curto recria a realimentação que produziu a fuga de
   12-08.
3. Afrouxar ou desligar o `PolygonStop`. Descartada: ele é a última proteção
   física e já evitou batidas. O defeito medido não foi ele parar, foi a demora
   para responder depois da parada.
4. Agir imediatamente porque a parede está no mapa. Descartada: mapa e
   localização têm erro. Dois segundos confirmam falta de progresso antes de
   furar o reflexo.

## Decisão

- `re_parado_s` continua **4,0 s** para o caso genérico.
- O prazo cai para `re_parado_mapeado_s = 2,0 s` somente quando:
  1. há célula ocupada do mapa estático a até `re_mapeado_raio = 0,60 m` do
     ponto correspondente no plano aceito; e
  2. o `/scan` está fresco e mede espaço para o escape frontal inteiro:
     0,20 m de avanço + 0,10 m de folga.
- Célula desconhecida, ponto fora do mapa, frames incompatíveis, `/scan`
  ausente/velho ou frente com menos de 0,30 m conservam os 4 s.
- O mapa só muda **quando** o detector libera a recuperação. A função existente
  continua escolhendo a ação e remedindo o vão durante os 20 cm; ré, pivô,
  velocidade e `PolygonStop` não mudam.

O índice do plano transformado para `odom` corresponde ao mesmo índice do
plano original em `map`; por isso a consulta usa esse ponto original, sem
aproximar `odom ≈ map`. Se os frames não conferirem, a otimização se desliga.

## Limites e teste no chão

A pausa por joystick encontrada mais tarde na mesma corrida é outro defeito: o
comando manual zero, de prioridade maior, segurou a autonomia por 13,45 s. Ela
não entra nesta decisão.

Testes puros cobrem mapa ocupado, desconhecido, origem girada, frente livre e
insuficiente, além do prazo contextual sem mudar o padrão. A validação final é
repetir a mesma rota de volta e medir se a porta passa de 4,71 s para cerca de
2 s mais a última atualização de progresso, sem criar escape em espaço aberto.
Suíte completa no PC: **1324 testes passam**.
