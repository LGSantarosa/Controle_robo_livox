#!/usr/bin/env python3
"""Seguidor de caminho do robô 2 — executa o que o Nav2 planejou.

    o Nav2 diz POR ONDE ir          (/plan, Smac Hybrid-A* em Dubins)
    este nó diz PARA ONDE OLHAR     (~/rumo_alvo, ~/velocidade_alvo)
    a movimentação diz O QUE CABE   (lei_de_rumo, decisão 005)

A lei está em `lei_de_seguimento.py`, testável sem ROS; aqui é a cola de I/O e a
máquina de dois estados (SEGUINDO / RÉ). O racional, com os números que o
justificam, está em `docs/decisoes/008-nav2-planeja-nos-seguimos.md` e
`docs/decisoes/009-re-por-gatilho-nao-por-plano.md`.

Tópicos:
    entra  /plan              nav_msgs/Path      quem pede é a GUI, via
                                                 bt_navigator; o replanejamento
                                                 vem dele, não daqui
           /Odometry          nav_msgs/Odometry  pose (FAST-LIO ou Gazebo)
    sai    ~/rumo_alvo        std_msgs/Float64   [rad]
           ~/velocidade_alvo  std_msgs/Float64   [m/s] — NEGATIVA aciona a ré na
                                                 movimentação, sem tópico novo

⚠️ Este nó NÃO conhece zona morta nem `a_dec` de giro, e não fala com roda. Isso
é da movimentação, onde já está caracterizado. O único número dela que entra
aqui é o `v_piso`, e só para calcular o raio de chegada — ver o aviso de subida.
"""
import csv
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf2_ros import Buffer, TransformException, TransformListener

from robot_motion.lei_de_seguimento import (
    CorrecaoDeDesvio,
    MiraAdaptativa,
    ProgressoDeAvanco,
    carrot,
    chegou,
    comando_de_parada,
    curvatura_adiante,
    desvio_lateral,
    folga_radial,
    indice_mais_proximo,
    lookahead_de,
    orcamento_de_re,
    passagens_estreitas,
    raio_de_chegada_minimo,
    re_esgotada,
    rumo_para,
    alvo_estavel_de_passagem,
    vao_no_corredor_frontal,
    vao_no_corredor_traseiro,
    velocidade_de_seguimento,
)


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        p = self.declare_parameters('', [
            # Raio que a MÁQUINA fecha [m]. Governa o lookahead. Medido em
            # 29-07: 0,370 no perfil otimista de zona morta, 0,463 no
            # pessimista. Mesmo número que o planner recebe.
            ('raio_min_curva', 0.37),
            # ⚠️ 1,0 desde 05-08, era 1,5. Com 1,5 o lookahead dava 0,555 m —
            # mais da metade do vão da porta (0,90 m). A cenoura caía DEPOIS
            # da porta e o robô cortava a quina para alcançá-la: o pior ponto
            # das quatro corridas de 05-08 caiu sempre na ombreira, com folga
            # de 0,29 m contra os 0,314 do corpo.
            #
            # 1,5 vinha de um seguidor que NÃO pivotava e precisava de mira
            # longa para não oscilar. Este pivota (fatia 3 da 011) e tem o
            # compensador cancelando o arco: mira curta deixou de custar
            # oscilação. Com 1,0 o lookahead vira 0,37 m — cabe dentro do vão.
            # 19-08: a tentativa de 0,15 m foi reprovada na tela — amplificou
            # a irregularidade local e o robô passou a se perder do plano.
            # Volta à mira curta comprovada de 0,37 m; a regra angular acima
            # continua impedindo a mira LONGA de 1 m perto de curva.
            ('lookahead_fator', 1.0),
            ('lookahead_piso', 0.30),
            # --- MIRA ADAPTATIVA (decisão 040) ---
            #
            # A mira fixa de 0,37 m é a causa medida do S. O rumo pedido muda
            # por `atan(salto_do_plano / mira)`, e o plano salta de lado no
            # replanejamento (p90 5,8 cm, max 15,1 cm, medido em 14-08):
            #
            #   com 0,37 m   15 cm viram 22°     <- amplitude p90 medida: 20,0°
            #   com 1,00 m   15 cm viram  8,6°
            #
            # O robô 1 (`Controle_robo_web`) chegou nisto antes e por outro
            # caminho: *"carrot 0.6 amplifica ruído de pose (12 cm = 12°) ->
            # 184 giros no lugar, zigue-zague em corredor; a 1.5 m os mesmos
            # 12 cm = ~4,6° -> segue reto"*. A mira dele em curva é 0,6 — a
            # nossa era 0,37, mais curta ainda.
            #
            # ⚠️ O esticão é CONDICIONAL, e essa é a parte que o robô 1
            # aprendeu errando: mira longa corta curva por dentro. Estica só em
            # trecho reto E com espaço medido pelo SCAN. Passagem apertada é
            # parede perto por definição, e ali a mira curta é que segura a
            # linha — é o caso da porta.
            ('mira_longa', 1.00),
            # Limiares da geometria, não de varredura (`desvio_da_corda` sobre
            # 1,0 m): raio 2,0 m desvia 0,068; raio 1,0 m desvia 0,125.
            # Estica em curva suave (>= ~2 m). A corrida de 19-08 mostrou que
            # manter a mira longa ate 0,12 m fazia o carrot atravessar uma
            # curva logo depois da reta e cortar a porta. Encolhe em 0,08 m;
            # sobra 1 cm de histerese contra ruido sem esconder a quina.
            # 20-08, corredor do Gazebo: estes limiares estreitos deixaram a
            # mira curta em 97% da corrida. O plano suave fazia um meandro
            # largo, e a regra o confundia com manobra: o seguidor obedecia ao
            # S em vez de filtrá-lo. A quina da porta muda >40° no mesmo metro
            # (teste de regressão abaixo), portanto ela continua encolhendo a
            # mira com bastante margem.
            ('mira_tol_estica', 0.15),
            ('mira_tol_encolhe', 0.20),
            ('mira_rumo_estica_deg', 15.0),
            ('mira_rumo_encolhe_deg', 20.0),
            # Só estica com este vão livre à frente, medido no corredor
            # retangular do corpo. `None` (scan velho ou ausente) = não estica.
            ('mira_folga_min', 0.60),
            # --- realimentação do DESVIO LATERAL (decisão 039) ---
            #
            # Até 14-08 a lei era só de rumo: `rumo_para(x, y, carrot)`. O
            # desvio lateral só voltava ao comando de forma implícita e com
            # atraso, e pure pursuit tem erro permanente conhecido em curva —
            # a mira à frente corta a curva por dentro. A entrada da porta vem
            # logo depois de uma curva, e foi exatamente ali que doeu:
            #
            #   14-08, Gazebo    plano deixa 0,175 m p/ o corpo, robô deixa 0,065
            #   13-08, robô real plano deixa 0,118 m,             robô deixa 0,036
            #
            # Nos dois o PLANO estava bom e quem saiu dele foi o seguidor. Com
            # 2/3 da margem comida, o reflexo dispara contra parede MAPEADA —
            # que é o critério de aceitação que o dono fixou em 12-08.
            #
            # `k_lat` escolhido pela dinâmica que impõe, não por varredura: o
            # erro decai com constante de tempo 1/k s (ver `CorrecaoDeDesvio`).
            # Com 1,0, os 11 cm medidos viram ~1 cm em 3 s — 0,90 m de caminho
            # a 0,30 m/s, que cabe folgado na reta de aproximação.
            #
            # 🔴 REPROVADO EM 1,0 NA CORRIDA B DE 14-08 — o robô fez
            # zigue-zague e o resultado PIOROU:
            #
            #                       k_lat=0    k_lat=1,0
            #   amplitude p90        20,0°       39,8°
            #   pico                 50,3°       88,1°
            #   período p50           1,00 s      0,70 s
            #
            # E a causa não é só ganho alto: o Nav2 republica o plano ~1 Hz,
            # às vezes deslocado de lado. Isso é DEGRAU em `e_lat`, e o termo
            # o converte em degrau de rumo contra uma placa com ~0,5 s de
            # tempo morto. Degraus grandes de referência dobraram (7 → 14) e o
            # maior foi de 26° para 62°.
            #
            # ↑ 14-08, DEPOIS: o termo ganhou limite de taxa
            # (`desvio_taxa_deg_s`, ver `CorrecaoDeDesvio`), que é o conserto
            # da causa — baixar o ganho sozinho não resolveria, porque o degrau
            # continuaria degrau, só menor.
            #
            # ⚠️ MESMO ASSIM O DEFAULT FICA EM 0,0, e desta vez de propósito:
            # o valor 1,0 foi para a corrida B como default e o dono descobriu
            # o defeito na tela. Ganho só sobe por argumento explícito de
            # launch, e só vira default depois de uma corrida que o aprove.
            ('k_lat', 0.0),
            # Piso do denominador do termo de Stanley [m/s]. Em `v -> 0` ele
            # pediria 90° e o robô giraria parado em cima do caminho.
            ('desvio_v_ref', 0.20),
            # Acima disto não é correção, é manobra — e manobra tem dono (o
            # pivô da 036, a ré da 025).
            ('desvio_teto_deg', 30.0),
            # 🔴 O LIMITE DE TAXA, e ele é o que a corrida B de 14-08 comprou.
            # O plano salta de lado no replanejamento (p90 5,8 cm, max 15,1 cm)
            # e sem limite isso virava degrau de rumo no mesmo ciclo, contra
            # uma placa com ~0,5 s de tempo morto.
            #
            # 15°/s: o teto de 30° se completa em 2 s = 4× o tempo morto (a
            # malha vê movimento lento, que é a condição de não oscilar), e
            # fica 3,8× abaixo dos ~57°/s que a máquina fecha com `wz_max` 1,0.
            ('desvio_taxa_deg_s', 15.0),
            ('v_max', 0.5),
            ('a_lin', 0.3),
            ('wz_max', 1.0),
            # --- travessia DETERMINÍSTICA de gargalo (20-08) ---
            #
            # O mesmo /plan_smoothed nas corridas de 11:34 e 11:39 passou no
            # centro dos dois vãos com diferença menor que 1 cm. Mesmo assim,
            # no aperto de 0,80 m o robô chegava a x=8,5 com rumos entre 23° e
            # 45°: a placa ainda executava a curva anterior quando o carrot
            # curto já havia mudado para o outro lado. A 0,50 m/s, os 0,52 s
            # de retenção valem 26 cm — quase a travessia inteira.
            #
            # O mapa só detecta "aqui há duas paredes"; não marca portas e
            # não inventa rota. Ao entrar na janela, o próprio plano aceito
            # fornece um eixo que fica travado até a saída. A velocidade cai
            # apenas nessa janela, para a mudança de giro caber antes do
            # batente. Sem /map, o comportamento anterior permanece inteiro.
            ('passagem_estreita_habilitada', True),
            ('passagem_largura_min', 0.55),
            ('passagem_largura_max', 1.10),
            ('passagem_antecipacao', 1.00),
            # O alvo fica bem adiante para nunca virar uma singularidade ao
            # lado do robô. O latch é solto antes, assim que o corpo inteiro
            # passou da parede; nesse instante ainda restam 0,60 m de mira.
            ('passagem_saida', 1.00),
            ('passagem_liberacao', 0.40),
            # Só muda de "entrar" para "atravessar" quando POSE e RUMO reais
            # estão no eixo. A corrida continua_01 provou que conferir apenas
            # a reta projetada liberava com 53° de guinada residual.
            ('passagem_alinha_lateral', 0.08),
            ('passagem_alinha_rumo_deg', 10.0),
            # 🔴 20-08: O TETO DE 0,25 ERA A CAUSA DA TRAVADA NA PORTA, e ele
            # anda para trás da própria intenção. Foi posto para "dar tempo de
            # corrigir dentro do vão"; o que ele faz é jogar o robô na faixa
            # de velocidade em que o atuador é MENOS fiel, justamente onde a
            # folga é de 12 cm.
            #
            # Ganho da movimentação (`cmd_vel_bruto` -> roda) e curvatura
            # parasita, medidos no bag de `continua_02`:
            #
            #     v = 0,40-0,60 m/s   ganho 0,16   curvatura  -0,12 1/m
            #     v = 0,22-0,28 m/s   ganho 0,45   curvatura  +0,55 1/m
            #
            # 4,6x maior e de SINAL OPOSTO. E a virada aparece no yaw, no
            # ciclo seguinte ao teto armar (mesma corrida, porta do `aperto`):
            #
            #     x=8,03  v 0,50  yaw +15°      <- subindo suave
            #     x=8,14  v 0,25  yaw +17°      <- o teto arma aqui
            #     x=8,23  v 0,25  yaw +30°
            #     x=8,40  v 0,25  yaw +46°      <- entra na porta girando
            #
            # É o cuidado do robô 1 que o CLAUDE.md diz valer DOS DOIS LADOS:
            # nunca escalar wz parcialmente sem conhecer a zona-morta do
            # atuador. Aqui a zona morta é por RODA, então baixar a linear
            # aproxima as duas rodas do limiar e a movimentação passa a
            # amplificar o giro para vencê-lo.
            #
            # Igual a `v_max` = teto NEUTRO: o `min()` abaixo não corta nada,
            # e a detecção de gargalo (alvo fixo, eixo, liberação) continua
            # inteira. O knob fica porque um teto medido pode voltar a fazer
            # sentido — mas só depois de a zona-morta de giro ser levantada
            # com `tools/banco/`, e nunca de novo por hipótese.
            ('passagem_v_max', 0.5),
            # Corpo físico medido (0,455 m) e margem geométrica por lado.
            # Não reutiliza a caixa do reflexo: ela é maior porque também
            # responde por distância de parada, não só por caber no vão.
            ('passagem_meia_largura', 0.2275),
            ('passagem_margem', 0.03),
            # Piso de linear da MOVIMENTAÇÃO. Não é usado para comandar — ela
            # se defende sozinha — mas sem ele não dá para saber se o raio de
            # chegada pedido é possível. TEM QUE BATER com o que a movimentação
            # calcula: zona_morta + wz_max·bitola/2 + margem.
            #
            # 0,0178 + 1,0·0,270/2 + 0,05 = 0,203, com a zona morta MEDIDA
            # (decisão 020). Era 0,335, que vinha do chute de 0,15 — e o "tem
            # que bater" acima era só comentário: os dois arquivos andaram
            # separados por doze dias porque nada conferia. Agora confere
            # (`test_o_piso_do_seguidor_sai_da_movimentacao`).
            ('v_piso', 0.203),
            ('raio_chegada', 0.25),
            # --- chegada em DUAS FASES (05-08) ---
            # A decisão 006 tirou o rumo de chegada com esta razão: "girar
            # depois de chegar arrasta o robô para fora do ponto (0,06 m
            # viraram 0,27 m em 27-07)". Aquilo era verdade para um robô que
            # só sabia ARCAR: girar significava andar em círculo.
            #
            # Com o pivô (fatia 3 da 011) girar parado custa v=0 — o robô não
            # sai do lugar. Então a chegada volta a ter duas fases: vai até o
            # ponto reto, e SÓ LÁ acerta o ângulo. É também o que torna o
            # Theta* utilizável: o plano não precisa mais chegar apontado.
            #
            # 🔴 FALSE DESDE a 4ª leva de 12-08 (decisão 023), consequência direta:
            # a fase 2 pedia `v=0` mais um ângulo, e quem entregava isso era o
            # pivô por corte — que saiu do caminho por não fechar contra a
            # retenção da placa. A lei contínua assume e NÃO arrasta (medido:
            # `v` = 0,000 de 2° a 150° com `v_alvo=0`), mas também não gira
            # abaixo de 90° no perfil do simulador, onde a zona morta crida é
            # 0,10. Deixar ligado seria pendurar a chegada esperando um ângulo
            # que a máquina não sabe fechar — parado, em silêncio, que é o BO-3.
            #
            # E o requisito nunca foi do seguidor: `comando_de_parada` já diz
            # que "rumo na chegada não é requisito deste seguidor, e persegui-lo
            # custa a própria chegada", e o `nav2.yaml` já roda com
            # `use_final_approach_orientation: false`. Religa junto com o pivô.
            ('aponta_no_fim', False),
            # Folga sobre a tolerância do pivô (~6°): pedir mais fino que a
            # manobra consegue entregar é laço que não fecha.
            ('tolerancia_rumo_final', 0.15),
            # --- ré por gatilho (decisão 009) ---
            #
            # ⚠️ DESLIGADA POR PADRÃO desde 05-08, e isto revisa a premissa da
            # própria 009. Ela escolheu ré-por-sintoma com estas palavras: "o
            # que sustenta a ré por gatilho é o PIVÔ, e o robô 1 pivota. O robô
            # 2 não, com os parâmetros de hoje." Essa premissa CAIU: com a lei
            # do pivô (fatia 3 da 011) o robô 2 pivota — 3 manobras iniciadas,
            # 3 fechadas, resíduo de 0,0°/0,0°/0,1°.
            #
            # E a ré nunca resolveu o problema que a disparava: ela dispara por
            # "não progrediu", que num robô mal-apontado significa RUMO errado
            # — e recuar RETO não muda rumo (medido em 29-07, 7ª leva:
            # "recuar NÃO salva o plano, hipótese testada e derrubada").
            # Pior, ela atropelava o pivô: em 05-08, 2 de 3 manobras morreram
            # assim, com o pivô acusando BO-3 por não conseguir girar.
            #
            # Fica no código, com orçamento e voz, para o caso que só ela
            # resolve: geometria fechada de verdade (nariz contra a parede,
            # sem espaço para girar).
            #
            # 🔴 TRUE DESDE a 4ª leva de 12-08 (decisão 024) — e é exatamente
            # esse caso que apareceu. Das duas objeções acima, uma morreu e a
            # outra não se aplica:
            #
            # · "ela atropelava o pivô" — não há mais pivô para atropelar
            #   (023, algumas horas antes desta linha);
            # · "recuar reto não muda RUMO" — continua verdade, e continua
            #   sendo motivo para não usar a ré como conserto de rumo. Aqui o
            #   emprego é OUTRO: tirar o robô de uma CÉLULA que o planner
            #   recusa (`Start occupied`). Isso é posição, não rumo, e recuar
            #   reto muda posição. A distinção é o que sustenta esta linha.
            #
            # O reflexo cobre a traseira: o polígono do `collision_monitor` vai
            # de +0,49 a −0,28 m em x, e o Mid-360 é 360°. Recuar não é cego
            # contra obstáculo — é cego contra buraco, e disso nenhum sensor
            # nosso protege.
            ('re_habilitada', True),
            # Quantas rés cabem sem que um plano novo chegue. UMA, e o teto é
            # o ponto: recuar tira o robô da célula que o planner recusa, mas
            # não ressuscita um objetivo abortado. Sem teto, o robô atravessa
            # a sala de ré em passos de 0,30 m — movimento que parece
            # recuperação e não é.
            ('re_max_sem_plano', 1),
            # --- o vão traseiro (decisão 025) ---
            # Largura do CORREDOR que o corpo varre dando ré [m]. A trena de
            # 29-07 deu caixa 0,433 × 0,455; 0,50 dá 2 cm de folga por lado
            # sobre a maior dimensão. NÃO é o `robot_radius` do Nav2 (0,32,
            # que é raio) nem a bitola (0,270, que é entre-eixos de roda).
            # 19-08: acompanha o contorno do planner/reflexo (0,455 m de
            # corpo + 3 cm de cada lado). A traseira nao pode varrer uma faixa
            # mais larga do que aquela que a recuperacao mede.
            ('re_largura', 0.555),
            # Do centro do robô ao para-choque traseiro [m]. Mesma referência
            # do polígono do reflexo, que vai a −0,28.
            ('re_recuo_para_choque', 0.28),
            # `/scan` mais velho que isto = traseira BLOQUEADA, não "livre".
            # Leitura que não existiu não pode virar permissão para recuar.
            #
            # ⚠️ 0,8 s CORRIGIDO POR MEDIDA. Começou em 0,5 e cairia em cima do
            # pior caso: 245 quadros medidos no simulador deram p50 0,103,
            # p90 0,207, p99 0,317 e MÁX 0,513 s. Com 0,5 a ré abortaria por
            # falso alarme, e o sintoma seria um robô que se recusa a se
            # desencalhar — parecido demais com defeito de lógica.
            #
            # E o teto vem de uma conta, não de gosto: recuando a `v_piso`, uma
            # janela vencida inteira gasta 0,8 × 0,203 = 0,16 m às cegas, e a
            # `folga` que o orçamento já desconta do vão medido é 0,30 m. A
            # cegueira cabe DENTRO da margem. Há teste travando este par.
            ('re_scan_velho_s', 0.8),
            # Folga entre o vão medido e o que a ré se permite gastar [m].
            # Explícita (era o default de `orcamento_de_re`) porque é ela que
            # cobre a janela de `/scan` vencido — e um número que sustenta uma
            # invariante de segurança não pode viver só num default.
            ('re_folga', 0.30),
            # 🔴 4,0 s DESDE a 6ª leva de 12-08, era 1,5 — e 1,5 fabricava uma
            # FUGA. Medido na corrida da porta: 9 rés seguidas levaram o robô
            # de 2,50 m para 5,10 m do objetivo, andando de costas em linha
            # reta até o vão traseiro acabar (3,17 m -> 0,31 m).
            #
            # A conta que explica: a própria ré dura 1,6–2,8 s (medido) e
            # recua 0,30 m. Para zerar o relógio o robô precisa BATER o melhor
            # de sempre, que ficou 0,30 m atrás — ou seja, ~1 s só para voltar
            # ao ponto de partida, mais o avanço. Com o relógio em 1,5 s a ré
            # rearma ANTES de o robô ter tempo físico de aproveitar a
            # anterior. Realimentação positiva, e o sintoma é um robô que "dá
            # ré à toa" estando apontado certo.
            #
            # 4,0 s é maior que a manobra mais longa medida (2,8 s) mais o
            # retorno (~1 s), com folga. Há teste travando o par.
            ('re_parado_s', 4.0),
            ('re_avanco_min', 0.05),
            ('re_orcamento_cego', 0.30),
            ('re_teto_s', 8.0),
            # Mesmo plano B do robô 1: se a traseira está bloqueada mas a
            # frente está livre, sair 20 cm para FRENTE pelo canal de
            # desencalhe. O vão frontal é medido antes e durante a manobra.
            ('desencalhe_frente_dist', 0.20),
            ('desencalhe_frente_folga', 0.10),
            # Quando frente E traseira estão bloqueadas, translação nenhuma
            # existe. Ainda pode caber um pivô: a caixa física de
            # 0,433 x 0,455 m varre raio de 0,314 m. O footprint de 0,6165 x
            # 0,555 m visto no RViz é a envolvente de parada, não o corpo.
            # Mede o círculo físico + 2 cm; nunca gira só porque os corredores
            # retangulares disseram zero.
            ('desencalhe_pivo_habilitado', True),
            ('desencalhe_pivo_folga', 0.334),
            ('desencalhe_pivo_angulo_deg', 25.0),
            ('desencalhe_pivo_wz', 1.0),
            ('desencalhe_pivo_teto_s', 4.0),
            # Ré só é recuperação quando existe bloqueio físico à frente.
            # Rumo/pivô/replanejamento também podem ficar 4 s sem progresso e
            # não autorizam andar para trás.
            ('re_bloqueio_frente_max', 0.20),
            # Quantas rés SEGUIDAS sem melhorar o melhor. É o teto estrutural
            # contra a fuga, e ele não depende de sintonia: recuo que não
            # aproxima o robô do objetivo não é recuperação, e repeti-lo é
            # andar de costas com cara de recuperação. Zera assim que o robô
            # bate a melhor distância que tinha antes da ré.
            ('re_max_seguidas', 2),
            # Regra do dono, 20-08: objetivo vivo não pode terminar em robô
            # parado porque um contador acabou. Os tetos antigos continuam
            # disponíveis para bancada com este knob em false; em produção a
            # recuperação repete, sempre limitada pelo scan em cada manobra.
            ('recuperacao_infinita_com_objetivo', True),
            # 🔴 RÉ SÓ COM OBJETIVO VIVO (14-08, decisão 031). Requisito do
            # dono, com as palavras dele: *"a ré é para desencalhar, mas quando
            # ele ENCALHA por conta de um erro, é pra desencalhar E IR ATÉ UM
            # PONTO"*. Recuo sem objetivo não é recuperação de nada — não há
            # para onde voltar depois, e o robô só anda de costas.
            #
            # Default `True` pela regra da decisão 019 (o default é o caso
            # seguro): entre "recuar sem ninguém ter pedido" e "não recuar", o
            # perigoso é o primeiro — foi ele que apareceu no robô e no Gazebo.
            #
            # ⚠️ Consequência que é FEATURE, não efeito colateral: dirigindo o
            # seguidor por `/plan` cru (o `tools/banco/plano.py`, sem ação do
            # Nav2) a ré fica INERTE. Foi assim que ela fez o robô recuar do
            # nada na bancada, e o dono nomeou isso como defeito.
            ('re_exige_objetivo', True),
            ('taxa', 20.0),
            # Sem plano novo por este tempo, para. Desde 19-08 o planejador
            # roda a cada 5 s para nao balancar a referencia; 7 s tolera um
            # ciclo atrasado sem manter plano morto indefinidamente.
            ('timeout_plano', 7.0),
            # 🔴 O TÓPICO DO PLANO — o suavizado, não o cru (decisão 042). O
            # porquê, com número, está na assinatura lá embaixo. O cru fica
            # declarado porque ele é a QUEDA quando o suavizador recusa, e
            # porque um dia pode ser preciso voltar atrás por parâmetro.
            ('topico_plano', '/plan_smoothed'),
            ('topico_plano_cru', '/plan'),
            # Segurança determinística: na produção o plano cru nunca dirige.
            # Em 20-08 ele chegou 8 ms antes do suave, foi congelado pela trava
            # de estabilidade e levou o robô inclinado ao aperto. Bancadas sem
            # smoother podem religar explicitamente; navegação real espera.
            ('aceita_plano_cru', False),
            # 🔴 14-08: LOG DE TODA CORRIDA, POR PADRÃO (pedido do dono).
            #
            # Até hoje `csv` nascia vazio E `grava()` NUNCA era chamado — o nó
            # sabia gravar e não gravava, em nenhuma corrida. Foi por isso que
            # o dia inteiro dependeu de eu lembrar de subir um `ros2 bag` à
            # mão, e a primeira corrida BOA do dia ficou sem registro.
            #
            # Agora: `log_dir` não-vazio faz o nó abrir um CSV com carimbo de
            # tempo a cada subida, sem ninguém pedir. `csv` continua existindo
            # como caminho explícito, para a bancada escolher o nome.
            ('csv', ''),
            ('log_dir', ''),
            ('log_periodo_s', 5.0),
        ])
        self.par = {x.name: x.value for x in p}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_rumo = self.create_publisher(Float64, '~/rumo_alvo', qos)
        self.pub_vel = self.create_publisher(Float64, '~/velocidade_alvo', qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, qos)
        # ══════════════════════════════════════════════════════════════════
        # 🔴 O SEGUIDOR SEGUE O PLANO SUAVIZADO (decisão 042). Até 14-08 ele
        # assinava `/plan` — o Theta* CRU — e a suavização da 026 nunca chegou
        # nele. Medido nas 5 corridas do protocolo, na aproximação da porta:
        #
        #   /plan            raio mínimo exigido  0,215 a 0,275 m   NÃO CABE
        #   /plan_smoothed                        0,402 a 0,477 m   cabe
        #                    a máquina fecha 0,37 m (`raio_min_curva`)
        #
        # O plano cru pede curva mais fechada do que o robô sabe fazer, bem na
        # boca de um vão de 0,85 m. Ele chega ainda girando, e o resíduo de
        # rumo come 7 cm dos 22 cm de folga por lado.
        #
        # A árvore JÁ suaviza e escreve o suavizado de volta em `{path}` — quem
        # não recebia era este nó, que lia o tópico do PLANEJADOR em vez do
        # tópico do SUAVIZADOR.
        #
        # ⚠️ E POR ISSO A QUEDA PARA O CRU EXISTE: se o `SmoothPath` recusar, a
        # árvore segue com o plano cru e `/plan_smoothed` PARA de sair. Sem a
        # queda, o sintoma seria o pior deste projeto — "objetivo aceito, plano
        # desenhado, robô parado, ninguém culpado no log" (13-08). O cru só
        # entra quando o suave está mais velho que `timeout_plano`.
        # ══════════════════════════════════════════════════════════════════
        self.create_subscription(Path, self.par['topico_plano'],
                                 lambda m: self.cb_plano(m, suave=True), qos)
        self.create_subscription(Path, self.par['topico_plano_cru'],
                                 lambda m: self.cb_plano(m, suave=False), qos)
        # O RViz deve mostrar a rota que de fato dirige, não cada candidato
        # instável publicado pelo planner e recusado pela trava abaixo.
        self.pub_plano_aceito = self.create_publisher(
            Path, '~/plano_aceito',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # 🔴 QUEM DIZ QUE EXISTE OBJETIVO VIVO — e sem isto a ré recua sozinha.
        #
        # Defeito medido em 13-08 (robô) e reproduzido em 14-08 no Gazebo: o
        # `/plan` fica RETIDO. Terminado o objetivo — cancelado, abortado, ou o
        # dono simplesmente parou de clicar — `self.plano` continua cheio, o
        # robô parado passa `re_parado_s` sem avançar, o plano vence, e o
        # gatilho de emperramento chama a ré. O robô anda de costas sem que
        # nada esteja acontecendo. Palavras do dono: *"ontem ela ativava do
        # nada sem nada estar acontecendo, e pior, aconteceu no gazebo também"*.
        #
        # Os dois tópicos, e não só o primeiro, porque a árvore atende às duas
        # ações; `unstuck_supervisor` e `freeze_capture` já leem este mesmo par.
        self._objetivo = {}
        for topico in ('navigate_to_pose/_action/status',
                       'navigate_through_poses/_action/status'):
            self.create_subscription(
                GoalStatusArray, topico,
                lambda msg, t=topico: self.cb_status(msg, t), qos)

        # 🔴 O CANAL QUE FURA O REFLEXO (decisão 025). Entra no `twist_mux`
        # DEPOIS do `collision_monitor`, com prioridade 30 — acima da
        # autonomia (10) e abaixo do humano (teclado 90, web 50).
        #
        # Existe porque o `PolygonStop` é cego para DIREÇÃO: medido em 12-08,
        # 831 de 831 amostras de ré foram vetadas pelo mesmo reflexo que tinha
        # acabado de salvar o robô de bater na ombreira. O furo é no bloqueio,
        # NUNCA na percepção: quem publica aqui já mediu o vão de trás.
        self.pub_desencalhe = self.create_publisher(
            TwistStamped, '/unstuck_vel', qos)
        # O `/scan` da decisão 021 é o que torna a ré não-cega. Best effort:
        # é sensor de alta taxa, e perder quadro é normal — o que não pode é
        # quadro VELHO passar por medida (ver `vao_traseiro`).
        self.create_subscription(
            LaserScan, '/scan', self.cb_scan,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        # O mapa é usado somente para localizar gargalos sobre a rota já
        # aceita. Transient-local garante a cópia publicada pelo map_server
        # antes deste nó terminar de subir.
        self.create_subscription(
            OccupancyGrid, '/map', self.cb_mapa,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        self.pose = None
        self.plano = []
        # Frame em que o plano chegou (`map` com AMCL). Sem ele não dá
        # para saber se a transformada é necessária — ver `plano_em_odom`.
        self.plano_frame = None
        self.mapa = None
        self.passagens = []
        self.passagem_ativa = None
        self.passagem_fase = ''
        # Quando o plano SUAVIZADO chegou pela última vez. `None` = nunca veio,
        # e aí o cru é aceito sem discussão: é o caso da bancada e de qualquer
        # pilha que não suba o `smoother_server` (042).
        self.t_plano_suave = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.t_plano = None
        # Replanejar continua ativo no Nav2, mas uma rota que está funcionando
        # não pode ser trocada de lado a cada ciclo. Só falta de progresso
        # libera a substituição para o MESMO objetivo; objetivo novo entra já.
        self.aceita_replano = True
        self.estado = 'ocioso'
        self.progresso = ProgressoDeAvanco(self.par['re_parado_s'],
                                           self.par['re_avanco_min'])
        self.re_desde = None
        self.re_origem = None
        self.re_sentido = -1
        self.re_orcamento_atual = 0.0
        self.pivo_desde = None
        self.pivo_rumo_inicial = None
        self.pivo_sentido = 1
        # Quantas rés já foram gastas SEM que um plano novo chegasse. Zera no
        # `cb_plano`: plano novo é a prova de que a recuperação serviu.
        self.res_sem_plano = 0
        self.scan = None
        self.t_scan = None
        # Contabilidade da fuga: quantas rés seguidas não melhoraram nada, e
        # qual era a melhor distância antes da última delas.
        self.res_seguidas = 0
        self.dist_antes_da_re = None
        self.rumo_objetivo = None

        self.linhas = []
        # `dt` do laço é fixo porque o `passo` roda por timer de taxa fixa —
        # é o mesmo que a correção lateral usa para limitar a taxa.
        self.dt = 1.0 / self.par['taxa']
        self.correcao = CorrecaoDeDesvio(
            k_lat=self.par['k_lat'], v_ref=self.par['desvio_v_ref'],
            teto=math.radians(self.par['desvio_teto_deg']),
            taxa_max=math.radians(self.par['desvio_taxa_deg_s']))
        self.mira = MiraAdaptativa(
            curto=lookahead_de(self.par['raio_min_curva'],
                               self.par['lookahead_fator'],
                               self.par['lookahead_piso']),
            longo=self.par['mira_longa'],
            tol_estica=self.par['mira_tol_estica'],
            tol_encolhe=self.par['mira_tol_encolhe'],
            folga_min=self.par['mira_folga_min'],
            rumo_estica=math.radians(self.par['mira_rumo_estica_deg']),
            rumo_encolhe=math.radians(self.par['mira_rumo_encolhe_deg']))
        self.create_timer(1.0 / self.par['taxa'], self.passo)
        # Caminho do CSV: explícito ganha; senão, carimbo de tempo no log_dir.
        if not self.par['csv'] and self.par['log_dir']:
            os.makedirs(self.par['log_dir'], exist_ok=True)
            carimbo = time.strftime('%Y-%m-%d_%H%M%S')
            self.par['csv'] = os.path.join(
                self.par['log_dir'], f'seguidor_{carimbo}.csv')
        if self.par['csv']:
            # Reescreve periodicamente: corrida que termina em Ctrl-C, queda de
            # bateria ou `kill -9` (as três aconteceram este mês) não pode
            # perder o registro por estar tudo em memória.
            self.create_timer(self.par['log_periodo_s'], self.grava)
            self.get_logger().info(f"log da corrida -> {self.par['csv']}")
        self.avisa_de_saida()

    # ------------------------------------------------------------- subida
    def avisa_de_saida(self):
        """Diz com que números está trabalhando e o que eles impedem.

        O raio de chegada é o caso onde isso morde: pedir um raio menor que a
        distância de parada a partir do piso faz o robô ORBITAR o ponto para
        sempre — e o sintoma parece defeito de controle, não de configuração.
        Melhor gritar na subida do que descobrir isso rodando.
        """
        minimo = raio_de_chegada_minimo(self.par['v_piso'], self.par['a_lin'])
        la = lookahead_de(self.par['raio_min_curva'],
                          self.par['lookahead_fator'],
                          self.par['lookahead_piso'])
        self.get_logger().info(
            f"seguidor de pé — raio da máquina {self.par['raio_min_curva']:.2f} m, "
            f'mira {la:.2f} m em curva e {self.par["mira_longa"]:.2f} m em reta '
            f'(040), piso de linear {self.par["v_piso"]:.3f} m/s')
        if self.par['raio_chegada'] < minimo:
            self.get_logger().error(
                f"raio_chegada={self.par['raio_chegada']:.3f} m é MENOR que a "
                f'distância de parada a partir do piso ({minimo:.3f} m). O robô '
                'vai ORBITAR o ponto sem nunca fechar. Suba o raio de chegada '
                'ou meça a zona morta — ela é quem manda no piso.')
        else:
            self.get_logger().info(
                f'raio de chegada {self.par["raio_chegada"]:.2f} m '
                f'(mínimo viável {minimo:.2f} m)')

    # --------------------------------------------------------- callbacks
    def cb_odom(self, msg):
        self.pose = msg

    def cb_mapa(self, msg):
        self.mapa = msg
        self.atualiza_passagens()

    def atualiza_passagens(self):
        """Extrai gargalos do mapa sobre o plano congelado atual."""
        self.passagens = []
        if (not self.par['passagem_estreita_habilitada'] or self.mapa is None
                or len(self.plano) < 2):
            return
        mapa_frame = self.mapa.header.frame_id
        if (mapa_frame and self.plano_frame
                and mapa_frame != self.plano_frame):
            self.get_logger().warn(
                f'não detecto gargalo: /map está em {mapa_frame} e o plano '
                f'em {self.plano_frame}', throttle_duration_sec=10.0)
            return
        o = self.mapa.info.origin
        origem = (o.position.x, o.position.y, yaw_de(o.orientation))
        self.passagens = passagens_estreitas(
            self.plano, self.mapa.data,
            self.mapa.info.width, self.mapa.info.height,
            self.mapa.info.resolution, origem=origem,
            largura_min=self.par['passagem_largura_min'],
            largura_max=self.par['passagem_largura_max'])
        if self.passagens:
            larguras = ', '.join(f'{p.largura:.2f} m'
                                 for p in self.passagens)
            self.get_logger().info(
                f'{len(self.passagens)} gargalo(s) fixado(s) no plano aceito: '
                f'{larguras}')

    @staticmethod
    def comprimentos_do_plano(plano):
        s = [0.0]
        for a, b in zip(plano, plano[1:]):
            s.append(s[-1] + math.dist(a, b))
        return s

    def passagem_para(self, plano, i0):
        """Passagem ativa/próxima, latched até o corpo sair do vão."""
        if not self.passagens or not plano:
            self.passagem_ativa = None
            self.passagem_fase = ''
            return None
        s = self.comprimentos_do_plano(plano)

        if self.passagem_ativa is not None:
            p = self.passagem_ativa
            # Solta só depois do fim estreito + margem reta. Enquanto uma ré
            # traz o robô para trás, o mesmo eixo continua dono da tentativa.
            if s[i0] <= s[p.centro] + self.par['passagem_liberacao']:
                return p
            self.get_logger().info('gargalo concluído — devolvendo a mira normal')
            self.passagem_ativa = None
            self.passagem_fase = ''

        for p in self.passagens:
            falta = s[p.inicio] - s[i0]
            entrando = s[p.inicio] < s[i0] <= s[p.centro]
            if (0.0 <= falta <= self.par['passagem_antecipacao']
                    or entrando):
                self.passagem_ativa = p
                self.passagem_fase = ''
                self.get_logger().info(
                    f'GARGALO FIXADO: largura {p.largura:.2f} m, '
                    f'eixo mantido até o corpo avançar '
                    f'{self.par["passagem_liberacao"]:.2f} m depois do vão; '
                    f'alvo fixo a {self.par["passagem_saida"]:.2f} m')
                return p
        return None

    def plano_em_odom(self):
        """O plano trazido para o frame da POSE. `None` se não der.

        🔴 O DEFEITO DE 14-08, E ELE EXPLICA O DIA INTEIRO.

        Até esta data este nó comparava `/plan` (frame `map`) com `/Odometry`
        (frame `odom`) **sem nunca aplicar a transformada entre os dois** — não
        havia `TransformListener` no arquivo. A diferença entre os dois frames
        é exatamente a correção do AMCL, e ela foi medida nas corridas de hoje:

            corrida A   salto p90 14,3 cm   deriva total   76 cm
            corrida C   salto p90  9,6 cm   deriva total   56 cm
            corrida E   salto p90  100 cm   deriva total  833 cm  (o AMCL fugiu)

        Ou seja: o seguidor se achava fora do caminho por uma quantidade que
        era puro erro de frame, e dirigia para corrigir um desvio que não
        existe — que pula a cada atualização do AMCL. É o S, é a entrada torta
        na porta, e é por que toda melhoria de responsividade PIORAVA o
        resultado: mais fidelidade ao sinal errado.

        Traz o PLANO para `odom` (e não a pose para `map`) porque o
        `heading_controller` mede o rumo no referencial do `/Odometry` — está
        escrito na primeira linha da docstring dele. Mexer no frame da pose
        obrigaria a girar o `rumo_alvo` junto, e seria a mesma armadilha de
        frame trocada de lugar.
        """
        if self.plano is None or self.pose is None:
            return None
        destino = self.pose.header.frame_id or 'odom'
        if not self.plano_frame or self.plano_frame == destino:
            return self.plano          # já no mesmo frame: nada a fazer
        try:
            t = self.tf_buffer.lookup_transform(
                destino, self.plano_frame, rclpy.time.Time())
        except TransformException as e:
            # Não cair no comportamento antigo: usar o plano cru aqui é
            # exatamente o defeito. Sem transformada, não se dirige.
            self.get_logger().warn(
                f'sem TF {destino}<-{self.plano_frame} ({e}); o plano não pode '
                'ser usado — dirigir com ele seria o defeito de 14-08 de volta',
                throttle_duration_sec=5.0)
            return None
        dx = t.transform.translation.x
        dy = t.transform.translation.y
        dth = yaw_de(t.transform.rotation)
        c, s = math.cos(dth), math.sin(dth)
        return [(c * x - s * y + dx, s * x + c * y + dy) for (x, y) in self.plano]

    def cb_scan(self, msg):
        self.scan = msg
        self.t_scan = self.agora()

    def vao_traseiro(self):
        """Vão livre atrás do para-choque [m], ou `None` se não dá para saber.

        `None` é diferente de zero e a diferença é a segurança inteira: zero é
        "medi e não há espaço", `None` é "não medi". Quem chama trata os dois
        como proibição de recuar, mas o log precisa dizer qual dos dois foi —
        robô parado sem motivo escrito é o BO-3.
        """
        if self.scan is None or self.t_scan is None:
            return None
        if self.agora() - self.t_scan > self.par['re_scan_velho_s']:
            return None
        return vao_no_corredor_traseiro(
            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
            self.par['re_largura'], self.par['re_recuo_para_choque'],
            alcance_max=self.scan.range_max)

    def vao_frente(self):
        """Vão livre à frente do para-choque [m], ou `None` se não dá para saber.

        Gate da mira adaptativa (040). Mesmo contrato do `vao_atras`: `None` é
        "não medi", e quem chama trata como "não estica" — o lado seguro, que
        aqui é a mira curta.
        """
        if self.scan is None or self.t_scan is None:
            return None
        if self.agora() - self.t_scan > self.par['re_scan_velho_s']:
            return None
        return vao_no_corredor_frontal(
            self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
            self.par['re_largura'], self.par['re_recuo_para_choque'],
            alcance_max=self.scan.range_max)

    def vao_giro(self):
        """Raio livre ao redor do centro, ou ``None`` com scan ausente/velho."""
        if self.scan is None or self.t_scan is None:
            return None
        if self.agora() - self.t_scan > self.par['re_scan_velho_s']:
            return None
        return folga_radial(self.scan.ranges, alcance_max=self.scan.range_max)

    def publica_desencalhe(self, v, wz=0.0):
        """Ré pelo canal que fura o reflexo. `v` negativo, giro ZERO.

        Reto por decisão (009): andando para trás a boba deixa de ser
        arrastada e passa a ser empurrada, que é a configuração instável do
        carrinho de supermercado. Curvar assim é a manobra sobre a qual não
        existe medida nenhuma neste robô.
        """
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(wz)
        self.pub_desencalhe.publish(m)

    # Os três status ATIVOS do `action_msgs/GoalStatus`: 1 ACCEPTED,
    # 2 EXECUTING, 3 CANCELING. Mesma tripla que o `unstuck_supervisor` e o
    # `freeze_capture` já usam — se um dia mudar, muda nos três.
    ATIVOS = {1, 2, 3}

    def cb_status(self, msg, topico):
        self._objetivo[topico] = any(s.status in self.ATIVOS
                                     for s in msg.status_list)

    def tem_objetivo(self):
        """Existe objetivo de navegação vivo AGORA?

        Sem timeout de propósito: o `GoalStatusArray` é publicado a cada
        transição e o último estado vale até a próxima. Terminado o objetivo
        ele vira 4/5/6 (SUCCEEDED/CANCELED/ABORTED) e esta função passa a
        responder False sozinha — não há estado velho para expirar.

        Ninguém publicou nada ainda = **não há objetivo**. É o caso do
        seguidor dirigido por `/plan` cru, e é onde a ré tinha de ficar quieta.
        """
        return any(self._objetivo.values())

    def cb_plano(self, msg, suave=True):
        novo = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
        if len(novo) < 2:
            return
        agora = self.agora()
        if suave:
            self.t_plano_suave = agora
        elif not self.par['aceita_plano_cru']:
            self.get_logger().warn(
                'plano CRU descartado — segurança exige /plan_smoothed; '
                'aguardando o suavizador', throttle_duration_sec=10.0)
            return
        elif (self.t_plano_suave is not None
              and agora - self.t_plano_suave <= self.par['timeout_plano']):
            # o suavizado está vivo: o cru é a mesma missão, com curva que a
            # máquina não fecha. Descartar aqui é o ponto inteiro da 042.
            return
        elif self.t_plano_suave is not None:
            self.get_logger().warn(
                f'plano suavizado calado há '
                f'{agora - self.t_plano_suave:.1f} s — seguindo o CRU, que '
                'pede curva mais fechada do que a máquina fecha',
                throttle_duration_sec=5.0)

        # 20-08: o Smac produziu cinco rotas incompatíveis em sequência para
        # o mesmo objetivo. Cada uma começava perto da pose atual, então o
        # carrot curto mandava virar alternadamente para os dois lados. Trava
        # a rota aceita enquanto há progresso. O planner continua calculando
        # (e provando que está vivo), mas só assume novamente se o seguidor
        # declarar falta de progresso. Um objetivo diferente nunca herda a
        # trava da missão anterior.
        mesmo_objetivo = (bool(self.plano)
                          and math.dist(novo[-1], self.plano[-1]) <= 0.30)
        if (suave and mesmo_objetivo and self.tem_objetivo()
                and (not self.aceita_replano
                     or self.passagem_ativa is not None)):
            self.t_plano = agora
            return

        if not mesmo_objetivo:
            self.passagem_ativa = None
            self.passagem_fase = ''
        self.plano = novo
        self.aceita_replano = False
        if hasattr(self, 'pub_plano_aceito'):
            self.pub_plano_aceito.publish(msg)
        # 🔴 14-08: GUARDAR O FRAME DO PLANO. Até esta data ele era ignorado, e
        # o plano (`map`) era comparado direto contra a pose (`odom`) — ver
        # `plano_em_odom`, que é onde o defeito está descrito.
        self.plano_frame = msg.header.frame_id
        self.atualiza_passagens()
        # O ângulo de chegada sai do ÚLTIMO ponto do plano, e não de uma
        # assinatura própria de `/goal_pose`.
        #
        # A primeira versão assinava `/goal_pose` e ERA FRÁGIL: nó que publica
        # o alvo e sai pode ser descoberto pelo `bt_navigator` e não por este
        # nó, e então o robô navega para o ponto certo e gira para o ângulo do
        # alvo ANTERIOR. Aconteceu na demonstração de 05-08 — chegou a 0,09 m
        # do ponto com 98° de erro, e o log mostrava "ângulo acertado" porque
        # ele acertou o objetivo velho.
        #
        # Ler do plano elimina a corrida: sem plano este nó não faz nada
        # mesmo, então a informação chega junto com o trabalho.
        #
        # ⚠️ São só os pontos INTERMEDIÁRIOS do Theta* que vêm com orientação
        # zerada (29-07). O ÚLTIMO carrega o rumo pedido — conferido em 05-08:
        # alvo de +45,0° e último ponto do plano com +45,0°.
        self.rumo_objetivo = yaw_de(msg.poses[-1].pose.orientation)
        self.t_plano = self.agora()
        # Plano novo = a recuperação funcionou. O orçamento de ré volta ao
        # cheio; sem isto, um travamento no começo da missão deixaria o robô
        # sem recuperação pelo resto dela.
        self.res_sem_plano = 0
        if self.estado == 'ocioso':
            self.estado = 'seguindo'
            self.progresso.reinicia()

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publica(self, rumo, v):
        m = Float64()
        m.data = float(rumo)
        self.pub_rumo.publish(m)
        m = Float64()
        m.data = float(v)
        self.pub_vel.publish(m)

    # -------------------------------------------------------------- ciclo
    def passo(self):
        if self.pose is None or not self.plano:
            return
        # 🔴 O plano vem de `/plan` em `map`; a pose vem de `/Odometry` em
        # `odom`. Comparar os dois crus é o defeito de 14-08 — ver
        # `plano_em_odom`. Daqui para baixo, `plano` é o único que se usa.
        plano = self.plano_em_odom()
        if not plano:
            return
        t = self.agora()
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        rumo = yaw_de(self.pose.pose.pose.orientation)
        objetivo = plano[-1]
        dist = math.hypot(objetivo[0] - x, objetivo[1] - y)

        # ⚠️ A CHEGADA VEM ANTES DO FRESCOR DO PLANO, e a ordem é o conserto de
        # 05-08. O Nav2 declara `Goal succeeded` assim que o robô entra no raio
        # e PARA de replanejar; o plano vence 2 s depois. Com a checagem de
        # plano velho antes desta, o seguidor entrava em `parado (plano velho)`
        # e ficava trancado lá — nunca alcançava a fase de apontar, e o robô
        # parava no ponto com o ângulo errado. Medido: 2 de 3 alvos chegaram a
        # 0,10–0,20 m do ponto com 84–90° de erro de rumo.
        if chegou(dist, self.par['raio_chegada']) or self.estado == 'apontando':
            if self.aponta(rumo):
                return
            self.para('chegou')
            return

        # 🔴 A RÉ EM CURSO VEM ANTES DA GUARDA DE PLANO VELHO (4ª leva de
        # 12-08), e a ordem não é estilo: o plano vence JUSTAMENTE enquanto o
        # robô recua — ninguém replaneja para um robô emperrado. Com a guarda
        # antes, ela abortava a manobra no primeiro ciclo dela.
        if self.estado == 're':
            self.passo_de_re(t, x, y, rumo, dist)
            return
        if self.estado == 'pivo_escape':
            self.passo_de_pivo_escape(t, rumo, dist)
            return

        if (self.t_plano is not None
                and t - self.t_plano > self.par['timeout_plano']):
            # ⚠️ PLANO VELHO COM O ROBÔ EMPERRADO NÃO É MOTIVO PARA DESISTIR —
            # é o SINTOMA que a decisão 009 escolheu como gatilho da ré. Era
            # aqui que a única recuperação do seguidor ficava inalcançável por
            # construção: este `return` vinha antes da checagem de progresso lá
            # embaixo, e o plano vence exatamente quando o robô trava.
            #
            # Medido na 4ª leva de 12-08, alvo (6,0 · 1,5) pela porta de
            # 0,90 m: o reflexo parou o robô a 0,34 m da ombreira, o planner
            # recusou com `Start occupied`, o `bt_navigator` abortou o objetivo
            # e o seguidor parou PARA SEMPRE a 2,49 m do alvo — 87 s de CSV
            # com a pose imóvel na mesma casa decimal.
            if self.progresso.atualiza(t, dist):
                if (self.par['recuperacao_infinita_com_objetivo']
                        or self.res_sem_plano < self.par['re_max_sem_plano']):
                    self.entra_na_re(t, x, y, dist)
                    if self.estado == 're':
                        self.res_sem_plano += 1
                        self.passo_de_re(t, x, y, rumo, dist)
                        return
                else:
                    # ⚠️ E AQUI A RÉ PARA DE INSISTIR, DE PROPÓSITO. Recuar
                    # tira o robô da célula que o planner recusa, mas NÃO traz
                    # plano de volta: quem desistiu foi o objetivo, lá no
                    # `bt_navigator`. Sem este teto o robô atravessaria a sala
                    # de ré, 0,30 m por vez, para sempre — movimento que
                    # parece recuperação e não é.
                    self.get_logger().error(
                        f'recuei {self.res_sem_plano}x e nenhum plano novo '
                        f'chegou em {t - self.t_plano:.0f} s. Quem abortou foi '
                        'o OBJETIVO (bt_navigator), não o seguidor — sair do '
                        'lugar não traz plano de volta, e o objetivo precisa '
                        'ser mandado de novo.',
                        throttle_duration_sec=10.0)
            self.para('plano velho')
            return

        # --- seguindo ---
        i0 = indice_mais_proximo(plano, x, y)
        # Decisão 040: a mira ESTICA em reta e encolhe em curva. Mira fixa de
        # 0,37 m amplificava o salto do plano (p90 5,8 cm, max 15,1 cm) em até
        # 22° de referência — a amplitude p90 medida em 14-08 foi 20,0°.
        # `vao_frente` é o gate: passagem apertada volta para a mira curta.
        passagem = self.passagem_para(plano, i0)
        if passagem is None:
            la = self.mira.passo(plano, i0, self.vao_frente())
            _, alvo = carrot(plano, i0, la)
        else:
            alvo, fase = alvo_estavel_de_passagem(
                plano, passagem, x, y, rumo,
                saida=self.par['passagem_saida'],
                meia_largura=self.par['passagem_meia_largura'],
                margem=self.par['passagem_margem'],
                tolerancia_lateral=self.par['passagem_alinha_lateral'],
                tolerancia_rumo=math.radians(
                    self.par['passagem_alinha_rumo_deg']),
                eixo_comprometido=self.passagem_fase == 'eixo')
            la = math.hypot(alvo[0] - x, alvo[1] - y)
            if fase != self.passagem_fase:
                self.passagem_fase = fase
                self.get_logger().info(
                    f'gargalo: alvo fixo mudou para {fase} '
                    f'({alvo[0]:.2f}, {alvo[1]:.2f})')
        raio = curvatura_adiante(
            plano, i0, janela=max(self.mira.curto, min(la, self.mira.longo)))
        v = velocidade_de_seguimento(dist, raio, self.par['v_max'],
                                     self.par['a_lin'], self.par['wz_max'])
        if passagem is not None:
            v = min(v, self.par['passagem_v_max'])
        # Decisão 039: o rumo do carrot MAIS a realimentação do desvio lateral.
        # Só o carrot deixa erro permanente em curva (pure pursuit corta por
        # dentro), e foi ele que comeu 11 cm da margem da porta em 14-08.
        e_lat = desvio_lateral(plano, i0, x, y)
        if passagem is None:
            rumo_alvo = self.correcao.passo(rumo_para(x, y, alvo), e_lat, v,
                                            self.dt)
        else:
            # O alvo do gargalo já incorpora centralização e eixo. Somar aqui
            # a correção lateral do plano criaria um segundo dono e faria a
            # referência voltar a trocar de lado dentro da porta.
            self.correcao.reset()
            rumo_alvo = rumo_para(x, y, alvo)
        self.publica(rumo_alvo, v)
        self.registra(t, x, y, rumo, rumo_alvo, v, dist, raio, e_lat,
                      mira=la, alvo=alvo,
                      modo=('gargalo_' + self.passagem_fase
                            if passagem is not None else 'normal'))

        # Progresso de verdade apaga a dívida: se o robô chegou mais perto do
        # que estava antes da última ré, aquela ré cumpriu o papel dela.
        if self.dist_antes_da_re is not None and dist < self.dist_antes_da_re:
            self.res_seguidas = 0
            self.dist_antes_da_re = dist

        if self.progresso.atualiza(t, dist):
            self.entra_na_re(t, x, y, dist)

    def aponta(self, rumo):
        """Fase 2 da chegada: no ponto, acerta o ângulo. Devolve True se ainda
        está trabalhando nisso.

        Quem gira é o PIVÔ da movimentação (v=0), então o robô não sai do
        lugar — é isso que torna esta fase possível sem repetir o defeito de
        27-07, quando girar depois de chegar arrastava 0,06 m para 0,27 m.
        """
        if not self.par['aponta_no_fim'] or self.rumo_objetivo is None:
            return False
        erro = math.atan2(math.sin(self.rumo_objetivo - rumo),
                          math.cos(self.rumo_objetivo - rumo))
        if abs(erro) <= self.par['tolerancia_rumo_final']:
            if self.estado == 'apontando':
                self.get_logger().info(
                    f'ângulo acertado: {math.degrees(erro):+.1f}° do pedido')
            return False
        if self.estado != 'apontando':
            self.get_logger().info(
                f'no ponto — girando {math.degrees(erro):+.0f}° para acertar '
                f'o ângulo pedido')
            self.estado = 'apontando'
        # Velocidade ZERO: é o pivô que responde por isto.
        self.publica(self.rumo_objetivo, 0.0)
        return True

    def entra_na_re(self, t, x, y, dist):
        # 🔴 PRIMEIRA GUARDA: SEM OBJETIVO VIVO NÃO SE RECUA (decisão 031).
        #
        # Vem antes de tudo porque é a pergunta mais fundamental: as outras
        # guardas decidem SE ESTA ré cabe; esta decide se recuar faz sentido
        # ALGUM. O `/plan` fica retido depois que o objetivo morre, então sem
        # este cheque o robô parado recua sozinho 4 s depois de qualquer
        # objetivo terminar — o defeito de 13-08 no robô, repetido no Gazebo.
        #
        # `progresso.reinicia()` junto: sem ele o gatilho fica verdadeiro em
        # todo ciclo e o log vira enxurrada de 20 Hz.
        if self.par['re_exige_objetivo'] and not self.tem_objetivo():
            self.get_logger().warn(
                f'sem progresso a {dist:.2f} m do fim do plano, mas NÃO HÁ '
                'objetivo de navegação vivo — não recuo. Recuar sem objetivo '
                'não é desencalhe: não há para onde voltar depois. Mande o '
                'ponto de novo.', throttle_duration_sec=5.0)
            self.progresso.reinicia()
            return
        # Falta de progresso é a única autorização para substituir uma rota
        # ainda viva pelo próximo replanejamento do MESMO objetivo. Vale mesmo
        # quando a recuperação física (ré/escape/pivô) acabar sendo recusada.
        self.aceita_replano = True
        # ⚠️ `re_max_seguidas <= 0` entra AQUI, junto com o desligamento
        # explícito, e isso é conserto de 13-08: teto zero caía na guarda lá
        # embaixo, que formata `dist_antes_da_re` — e esse valor só existe
        # DEPOIS da primeira ré. Com teto zero não há primeira ré, então o nó
        # morria com `TypeError: unsupported format string passed to
        # NoneType.__format__` na primeira vez que o robô emperrasse. Nó morto
        # não dirige: o sintoma no robô foi objetivo aceito, plano desenhado e
        # robô parado, sem nenhuma mensagem culpando ninguém.
        if not self.par['re_habilitada'] or self.par['re_max_seguidas'] <= 0:
            # Quem conserta rumo agora é o pivô, lá na movimentação. Falar uma
            # vez a cada 5 s é o suficiente: se o robô ficar de fato emperrado
            # com a ré desligada, isto é a pista.
            self.get_logger().warn(
                f'sem progresso a {dist:.2f} m do objetivo — ré DESLIGADA '
                '(o pivô responde por rumo desde 05-08); se ele não sair '
                'daqui, a geometria é fechada e a ré precisa voltar',
                throttle_duration_sec=5.0)
            self.progresso.reinicia()
            return
        # 🔴 A RÉ DEIXOU DE SER CEGA (decisão 025). O `/scan` da 021 mede o
        # vão real atrás do para-choque, e ele é quem decide se a manobra
        # existe. Sem medida, NÃO recua: leitura que não existiu não vira
        # permissão.
        # ⚠️ RECUO QUE NÃO APROXIMA NÃO É RECUPERAÇÃO. Se a ré anterior não
        # levou o robô a bater a distância que ele já tinha antes dela, ela
        # não serviu — e repetir produz FUGA: medido em 12-08, 9 rés seguidas
        # levaram o robô de 2,50 m para 5,10 m do objetivo, de costas, até o
        # vão traseiro acabar. O teto é estrutural: não depende de sintonia,
        # só de a recuperação ter melhorado alguma coisa.
        if self.dist_antes_da_re is not None and dist < self.dist_antes_da_re:
            self.res_seguidas = 0          # a anterior serviu: crédito renovado
        if (not self.par['recuperacao_infinita_com_objetivo']
                and self.res_seguidas >= self.par['re_max_seguidas']):
            # `dist_antes_da_re` não pode ser None aqui (só se chega com
            # `res_seguidas >= 1`, e quem incrementa também grava a distância),
            # mas formatar None mata o nó — e nó morto não dirige. Cinto.
            antes = ('?' if self.dist_antes_da_re is None
                     else f'{self.dist_antes_da_re:.2f}')
            self.get_logger().error(
                f'{self.res_seguidas} rés seguidas e o robô não chegou mais '
                f'perto que {antes} m — recuar não está '
                'resolvendo, e insistir é andar de costas. Parado até o plano '
                'mudar.', throttle_duration_sec=10.0)
            return

        vao = self.vao_traseiro()
        if vao is None:
            self.get_logger().warn(
                'emperrado, mas SEM medida do vão traseiro (/scan ausente ou '
                f'mais velho que {self.par["re_scan_velho_s"]:.1f} s) — não '
                'recuo às cegas', throttle_duration_sec=5.0)
            return
        orcamento = min(orcamento_de_re(vao_traseiro=vao,
                                        folga=self.par['re_folga'],
                                        cego=self.par['re_orcamento_cego']),
                        self.par['re_orcamento_cego'])
        if orcamento <= 0.0:
            frente = self.vao_frente()
            margem = self.par['desencalhe_frente_folga']
            alvo = self.par['desencalhe_frente_dist']
            if frente is None or frente <= margem:
                medido = ('sem medida' if frente is None else f'{frente:.2f} m')
                if self.entra_no_pivo_escape(t, x, y, dist):
                    return
                self.get_logger().warn(
                    f'emperrado sem saída segura: atrás há {vao:.2f} m e '
                    f'na frente {medido}', throttle_duration_sec=5.0)
                return
            orcamento = min(alvo, frente - margem)
            self.re_sentido = 1
        else:
            frente = self.vao_frente()
            if (frente is None
                    or frente > self.par['re_bloqueio_frente_max']):
                medido = ('sem medida' if frente is None
                          else ('livre' if math.isinf(frente)
                                else f'{frente:.2f} m'))
                # 20-08: o PolygonStop é propositalmente conservador, mas é
                # cego à direção. Um retorno ao lado do corpo pode vetar tudo
                # mesmo com vários metros comprovadamente livres no corredor
                # reto. O seguidor já possui o canal de escape que fura esse
                # veto e REMEDE o vão frontal a cada ciclo; use-o por apenas
                # 20 cm, sem transformar folga lateral em permissão genérica.
                if frente is None:
                    self.get_logger().warn(
                        'sem progresso e sem medida frontal — não dou ré nem '
                        'avanço às cegas', throttle_duration_sec=5.0)
                    self.progresso.reinicia()
                    return
                orcamento = min(self.par['desencalhe_frente_dist'],
                                frente - self.par['desencalhe_frente_folga'])
                if orcamento <= 0.0:
                    self.progresso.reinicia()
                    return
                self.re_sentido = 1
            else:
                self.re_sentido = -1
        self.estado = 're'
        self.re_desde = t
        self.re_origem = (x, y)
        self.re_orcamento_atual = orcamento
        self.res_seguidas += 1
        if self.dist_antes_da_re is None or dist < self.dist_antes_da_re:
            self.dist_antes_da_re = dist
        if self.re_sentido < 0:
            self.get_logger().warn(
                f'EMPERRADO a {dist:.2f} m do objetivo — ré de até '
                f'{orcamento:.2f} m (vão medido atrás: {vao:.2f} m)')
        elif self.re_sentido > 0:
            self.get_logger().warn(
                f'EMPERRADO com frente livre ({frente:.2f} m) — escape RETO '
                f'para frente de até {orcamento:.2f} m, remedindo a cada ciclo')

    def entra_no_pivo_escape(self, t, x, y, dist):
        """Tenta o último recurso: pivô físico quando não cabe transladar."""
        if not self.par['desencalhe_pivo_habilitado']:
            return False
        folga = self.vao_giro()
        minimo = self.par['desencalhe_pivo_folga']
        if folga is None or folga < minimo:
            medido = 'sem medida' if folga is None else f'{folga:.2f} m'
            self.get_logger().warn(
                f'frente e traseira bloqueadas; pivô também não cabe '
                f'(raio livre {medido}, precisa {minimo:.2f} m)',
                throttle_duration_sec=5.0)
            return False

        rumo = yaw_de(self.pose.pose.pose.orientation)
        plano = self.plano_em_odom()
        sentido = 1
        if plano:
            i0 = indice_mais_proximo(plano, x, y)
            _, alvo = carrot(plano, i0, 0.60)
            erro = math.atan2(math.sin(rumo_para(x, y, alvo) - rumo),
                              math.cos(rumo_para(x, y, alvo) - rumo))
            if abs(erro) > math.radians(3.0):
                sentido = 1 if erro > 0.0 else -1
        self.estado = 'pivo_escape'
        self.pivo_desde = t
        self.pivo_rumo_inicial = rumo
        self.pivo_sentido = sentido
        self.get_logger().warn(
            f'ENCURRALADO a {dist:.2f} m do objetivo — pivô de escape '
            f'{sentido * self.par["desencalhe_pivo_angulo_deg"]:+.0f}° '
            f'(raio livre {folga:.2f} m)')
        return True

    def passo_de_pivo_escape(self, t, rumo, dist):
        """Fecha o pivô pela pose e revalida a varredura em todo ciclo."""
        folga = self.vao_giro()
        minimo = self.par['desencalhe_pivo_folga']
        girou = abs(math.atan2(math.sin(rumo - self.pivo_rumo_inicial),
                               math.cos(rumo - self.pivo_rumo_inicial)))
        alvo = math.radians(self.par['desencalhe_pivo_angulo_deg'])
        acabou = (girou >= alvo
                  or t - self.pivo_desde >= self.par['desencalhe_pivo_teto_s'])
        inseguro = folga is None or folga < minimo
        if acabou or inseguro:
            self.publica_desencalhe(0.0, 0.0)
            motivo = ('folga fechou' if inseguro else
                      f'girou {math.degrees(girou):.1f}°')
            self.get_logger().warn(f'fim do pivô de escape: {motivo}')
            self.estado = 'seguindo'
            self.progresso.reinicia()
            return
        self.publica(rumo, 0.0)
        self.publica_desencalhe(
            0.0, self.pivo_sentido * self.par['desencalhe_pivo_wz'])
        self.registra(t, self.pose.pose.pose.position.x,
                      self.pose.pose.pose.position.y, rumo, rumo, 0.0, dist,
                      float('inf'))

    def passo_de_re(self, t, x, y, rumo, dist):
        recuado = math.hypot(x - self.re_origem[0], y - self.re_origem[1])
        sentido = getattr(self, 're_sentido', -1)

        # ⚠️ O VÃO É REMEDIDO A CADA CICLO, e não só na largada da manobra.
        # Vão que some no MEIO da ré é o caso que o para-choque não perdoa: o
        # mundo tem gente andando, e uma medida de 8 s atrás não descreve o
        # que está atrás agora. Some ou não-medível -> PARA, na hora.
        vao = self.vao_traseiro() if sentido < 0 else self.vao_frente()
        margem = (0.0 if sentido < 0
                  else self.par['desencalhe_frente_folga'])
        if vao is None or vao <= margem:
            self.publica_desencalhe(0.0)
            self.get_logger().warn(
                'desencalhe ABORTADO no meio: ' +
                ('o vão escolhido fechou' if vao is not None
                 else 'perdi a medida do /scan'))
            self.estado = 'seguindo'
            self.progresso.reinicia()
            return

        if sentido < 0:
            orcamento = min(orcamento_de_re(vao_traseiro=vao,
                                            folga=self.par['re_folga'],
                                            cego=self.par['re_orcamento_cego']),
                            getattr(self, 're_orcamento_atual',
                                    self.par['re_orcamento_cego']))
        else:
            orcamento = min(self.re_orcamento_atual,
                            vao - self.par['desencalhe_frente_folga'])
        if re_esgotada(recuado, orcamento, t - self.re_desde,
                       self.par['re_teto_s']):
            # Zero EXPLÍCITO no canal: o mux segura o último comando até o
            # timeout, e sair da manobra sem zerar deixaria 0,5 s de ré órfã.
            self.publica_desencalhe(0.0)
            manobra = ('ré' if sentido < 0 else 'escape para frente')
            self.get_logger().warn(
                f'fim do {manobra}: percorreu {recuado:.2f} m em '
                f'{t - self.re_desde:.1f} s')
            self.estado = 'seguindo'
            self.progresso.reinicia()
            return
        # 🔴 A ré sai pelo CANAL QUE FURA (025), não pela cadeia normal: na
        # cadeia normal o reflexo a veta (831/831 medido em 12-08). E a
        # movimentação recebe ZERO enquanto isso, para não haver duas fontes
        # disputando a mesma roda.
        self.publica(rumo, 0.0)
        v_escape = sentido * self.par['v_piso']
        self.publica_desencalhe(v_escape)
        self.registra(t, x, y, rumo, rumo, v_escape, dist, float('inf'))

    def para(self, motivo):
        v, _ = comando_de_parada()
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        rumo = yaw_de(self.pose.pose.pose.orientation)
        # Rumo alvo = o rumo ATUAL: pedir outro faria a movimentação girar, e
        # girar depois de chegar arrasta o robô para fora do ponto (0,06 m
        # viraram 0,27 m em 27-07).
        self.publica(rumo, v)
        if self.estado != 'ocioso':
            self.get_logger().info(f'parado ({motivo})')
            self.estado = 'ocioso'
            self.progresso.reinicia()
            # Correção lateral acumulada não sobrevive a uma parada: aplicada
            # ao caminho seguinte ela é comando sem dono.
            self.correcao.reset()

    # ------------------------------------------------------------ registro
    def registra(self, t, x, y, rumo, rumo_alvo, v, dist, raio, e_lat=0.0,
                 mira=None, alvo=None, modo=''):
        """CSV de diagnóstico — o dono só roda, os números vêm por ssh.

        `rumo_alvo` está aqui de propósito: o plano salta entre replanejamentos,
        e seguidor que persegue esse salto oscila. Se isso aparecer neste robô,
        quero o número na mão em vez de adivinhar — e só então decidir se cabe
        filtro, não antes.
        """
        if not self.par['csv']:
            return
        self.linhas.append({
            't': round(t, 3), 'estado': self.estado,
            'modo': modo,
            'x': round(x, 4), 'y': round(y, 4),
            'rumo': round(rumo, 4), 'rumo_alvo': round(rumo_alvo, 4),
            'erro_rumo': round(math.atan2(math.sin(rumo_alvo - rumo),
                                          math.cos(rumo_alvo - rumo)), 4),
            'v_alvo': round(v, 4), 'dist': round(dist, 4),
            'raio_curva': ('inf' if math.isinf(raio) else round(raio, 4)),
            # 19-08: prova se a mira ficou curta diante de uma curva futura.
            'mira': '' if mira is None else round(mira, 4),
            'alvo_x': '' if alvo is None else round(alvo[0], 4),
            'alvo_y': '' if alvo is None else round(alvo[1], 4),
            # 039: com sinal (+ à esquerda). É a régua do conserto da porta —
            # sem ele o desvio só aparecia medindo o bag contra o mapa depois.
            'desvio_lateral': round(e_lat, 4),
        })

    def grava(self):
        if not self.par['csv'] or not self.linhas:
            return
        with open(self.par['csv'], 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.linhas[0].keys()))
            w.writeheader()
            w.writerows(self.linhas)
        self.get_logger().info(
            f"{len(self.linhas)} amostras -> {self.par['csv']}",
            throttle_duration_sec=30.0)


def main():
    rclpy.init()
    no = PathFollower()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        no.grava()
        no.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
