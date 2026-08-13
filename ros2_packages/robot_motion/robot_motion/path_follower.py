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

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

from robot_motion.lei_de_seguimento import (
    ProgressoDeAvanco,
    carrot,
    chegou,
    comando_de_parada,
    curvatura_adiante,
    indice_mais_proximo,
    lookahead_de,
    orcamento_de_re,
    raio_de_chegada_minimo,
    re_esgotada,
    rumo_para,
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
            ('lookahead_fator', 1.0),
            ('lookahead_piso', 0.30),
            ('v_max', 0.5),
            ('a_lin', 0.3),
            ('wz_max', 1.0),
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
            ('re_largura', 0.50),
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
            # Quantas rés SEGUIDAS sem melhorar o melhor. É o teto estrutural
            # contra a fuga, e ele não depende de sintonia: recuo que não
            # aproxima o robô do objetivo não é recuperação, e repeti-lo é
            # andar de costas com cara de recuperação. Zera assim que o robô
            # bate a melhor distância que tinha antes da ré.
            ('re_max_seguidas', 2),
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
            # Sem plano novo por este tempo, para. Plano velho é plano perigoso
            # — mesma regra do `timeout_alvo` da movimentação.
            ('timeout_plano', 2.0),
            ('csv', ''),
        ])
        self.par = {x.name: x.value for x in p}

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub_rumo = self.create_publisher(Float64, '~/rumo_alvo', qos)
        self.pub_vel = self.create_publisher(Float64, '~/velocidade_alvo', qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_odom, qos)
        self.create_subscription(Path, '/plan', self.cb_plano, qos)

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

        self.pose = None
        self.plano = []
        self.t_plano = None
        self.estado = 'ocioso'
        self.progresso = ProgressoDeAvanco(self.par['re_parado_s'],
                                           self.par['re_avanco_min'])
        self.re_desde = None
        self.re_origem = None
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
        self.create_timer(1.0 / self.par['taxa'], self.passo)
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
            f'lookahead {la:.2f} m, piso de linear {self.par["v_piso"]:.3f} m/s')
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

    def publica_desencalhe(self, v):
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
        m.twist.angular.z = 0.0
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

    def cb_plano(self, msg):
        novo = [(q.pose.position.x, q.pose.position.y) for q in msg.poses]
        if len(novo) < 2:
            return
        self.plano = novo
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
        t = self.agora()
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        rumo = yaw_de(self.pose.pose.pose.orientation)
        objetivo = self.plano[-1]
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
                if self.res_sem_plano < self.par['re_max_sem_plano']:
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
        i0 = indice_mais_proximo(self.plano, x, y)
        la = lookahead_de(self.par['raio_min_curva'],
                          self.par['lookahead_fator'],
                          self.par['lookahead_piso'])
        _, alvo = carrot(self.plano, i0, la)
        rumo_alvo = rumo_para(x, y, alvo)
        raio = curvatura_adiante(self.plano, i0, janela=la)
        v = velocidade_de_seguimento(dist, raio, self.par['v_max'],
                                     self.par['a_lin'], self.par['wz_max'])
        self.publica(rumo_alvo, v)
        self.registra(t, x, y, rumo, rumo_alvo, v, dist, raio)

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
        if self.res_seguidas >= self.par['re_max_seguidas']:
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
            self.get_logger().warn(
                f'emperrado e sem vão para recuar — atrás há {vao:.2f} m e a '
                'folga exigida é maior. Parado, e é a coisa certa.',
                throttle_duration_sec=5.0)
            return
        self.estado = 're'
        self.re_desde = t
        self.re_origem = (x, y)
        self.res_seguidas += 1
        if self.dist_antes_da_re is None or dist < self.dist_antes_da_re:
            self.dist_antes_da_re = dist
        self.get_logger().warn(
            f'EMPERRADO a {dist:.2f} m do objetivo — ré de até '
            f'{orcamento:.2f} m (vão medido atrás: {vao:.2f} m)')

    def passo_de_re(self, t, x, y, rumo, dist):
        recuado = math.hypot(x - self.re_origem[0], y - self.re_origem[1])

        # ⚠️ O VÃO É REMEDIDO A CADA CICLO, e não só na largada da manobra.
        # Vão que some no MEIO da ré é o caso que o para-choque não perdoa: o
        # mundo tem gente andando, e uma medida de 8 s atrás não descreve o
        # que está atrás agora. Some ou não-medível -> PARA, na hora.
        vao = self.vao_traseiro()
        if vao is None or vao <= 0.0:
            self.publica_desencalhe(0.0)
            self.get_logger().warn(
                'ré ABORTADA no meio: ' + ('o vão traseiro sumiu'
                                           if vao is not None
                                           else 'perdi a medida do /scan'))
            self.estado = 'seguindo'
            self.progresso.reinicia()
            return

        orcamento = min(orcamento_de_re(vao_traseiro=vao,
                                        folga=self.par['re_folga'],
                                        cego=self.par['re_orcamento_cego']),
                        self.par['re_orcamento_cego'])
        if re_esgotada(recuado, orcamento, t - self.re_desde,
                       self.par['re_teto_s']):
            # Zero EXPLÍCITO no canal: o mux segura o último comando até o
            # timeout, e sair da manobra sem zerar deixaria 0,5 s de ré órfã.
            self.publica_desencalhe(0.0)
            self.get_logger().warn(
                f'fim da ré: recuou {recuado:.2f} m em {t - self.re_desde:.1f} s')
            self.estado = 'seguindo'
            self.progresso.reinicia()
            return
        # 🔴 A ré sai pelo CANAL QUE FURA (025), não pela cadeia normal: na
        # cadeia normal o reflexo a veta (831/831 medido em 12-08). E a
        # movimentação recebe ZERO enquanto isso, para não haver duas fontes
        # disputando a mesma roda.
        self.publica(rumo, 0.0)
        self.publica_desencalhe(-self.par['v_piso'])
        self.registra(t, x, y, rumo, rumo, -self.par['v_piso'], dist, float('inf'))

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

    # ------------------------------------------------------------ registro
    def registra(self, t, x, y, rumo, rumo_alvo, v, dist, raio):
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
            'x': round(x, 4), 'y': round(y, 4),
            'rumo': round(rumo, 4), 'rumo_alvo': round(rumo_alvo, 4),
            'erro_rumo': round(math.atan2(math.sin(rumo_alvo - rumo),
                                          math.cos(rumo_alvo - rumo)), 4),
            'v_alvo': round(v, 4), 'dist': round(dist, 4),
            'raio_curva': ('inf' if math.isinf(raio) else round(raio, 4)),
        })

    def grava(self):
        if not self.par['csv'] or not self.linhas:
            return
        with open(self.par['csv'], 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.linhas[0].keys()))
            w.writeheader()
            w.writerows(self.linhas)
        self.get_logger().info(
            f"{len(self.linhas)} amostras -> {self.par['csv']}")


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
