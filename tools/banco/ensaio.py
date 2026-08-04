#!/usr/bin/env python3
"""Banco de ensaios de movimentação — mede os limites do robô e grava CSV.

Roda IGUAL no robô real e no simulador: os dois falam
`/hoverboard_base_controller/cmd_vel` (TwistStamped, SI) e publicam pose em
`/Odometry` (FAST-LIO no robô, pose verdadeira do Gazebo no simulador). Por
isso os números dos dois são diretamente comparáveis — é o que permite dizer
o quanto o simulador mente, com medida em vez de opinião.

    ros2 run ... não: é script solto, roda direto.
    python3 ensaio.py --ensaio zona_morta_linear --csv zm_lin.csv

VELOCIDADE MEDIDA VEM DA POSE, não do campo `twist`. O twist do publicador de
odometria do Gazebo mostrou-se ruidoso (marcava 0,077 rad/s com o rumo
parado); a pose é limpa nos dois lados. O twist é gravado assim mesmo, em
coluna separada, para quem quiser comparar.

QUAL POSE, decidido por `--fonte`. Use `lio` (o padrão), sempre.

⛔ `--fonte roda` está QUEBRADO neste robô e só continua aqui para o simulador.
O controlador está com `open_loop: True`, então
`/hoverboard_base_controller/odom` NÃO MEDE NADA: ele integra o comando
publicado e devolve. Medir a zona morta com ele em 31-07 devolveu exatamente o
limiar de detecção de volta (0,021 m/s medidos contra LIMIAR_PARADO = 0,020) —
uma tautologia com cara de resultado, com os quatro dentes concordando. Enquanto
`open_loop` for true, odometria de roda de verdade só pelos encoders crus,
`/hoverboard/{left,right}_wheel/position`.

O texto que estava aqui acusava o LIO de fabricar movimento. Era falso: a
comparação era contra esse comando ecoado. Medido depois, com o robô parado 20 s,
o LIO deriva 0,05° de yaw e 0,009 m. O ruído era da DERIVADA (ver JANELA_S).

SEGURANÇA (o robô é real e pesa 10 kg):
  --espaco   distância máxima da origem, em metros. Estourou, para tudo.
  --dur      teto de tempo. Estourou, para tudo.
  Ctrl-C, exceção ou fim do ensaio: publica zero antes de sair, sempre.
"""
import argparse
import csv
import math
import sys
from collections import deque

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Janela da diferenciação da pose. Curta demais amplifica ruído, longa demais
# atrasa a medida e estraga justamente o que queremos medir (a rampa de giro).
#
# 0,2 s era o valor do simulador, onde a pose vem a 50 Hz — 10 amostras por
# janela. No robô o /Odometry do FAST-LIO vem a 10 Hz, e 0,2 s pegam DUAS: em
# 31-07 isso transformou 0,54° de tremor de pose (medido, robô parado 20 s) em
# 0,033 rad/s de "ruído", maior que o limiar de disparo do dente de serra. O
# sensor estava bom; a derivada é que era curta demais. Ajustar por `--janela`.
JANELA_S = 0.2

# Limiares do dente de serra da zona morta. `PAROU` é o mesmo LIMIAR_PARADO do
# medir.py de propósito: quem vira o dente e quem lê o CSV têm que concordar
# sobre o que é "imóvel", senão o ensaio inverte num ponto e a leitura acha
# outro. `SAIU` fica acima para o gatilho não disparar no ruído da pose.
SAIU = 0.03
PAROU = 0.02
CONFIRMA_SAIU = 0.15    # s de movimento contínuo para aceitar que saiu
CONFIRMA_PAROU = 0.40   # s de imobilidade para aceitar que parou (ele desliza)
PAUSA_DENTE = 1.0       # s parado entre dentes: a próxima saída tem que ser
                        # do REPOUSO, senão não é atrito estático que se mede


def yaw_de(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def norm_ang(a):
    return math.atan2(math.sin(a), math.cos(a))


class Ensaio(Node):
    def __init__(self, cfg):
        super().__init__('ensaio')
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, cfg.sim)])
        self.cfg = cfg

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(TwistStamped, cfg.topico, qos)
        self.create_subscription(Odometry, '/Odometry', self.cb_pose, qos)
        self.create_subscription(
            Odometry, '/hoverboard_base_controller/odom', self.cb_roda, qos)

        self.pose = None
        self.roda = None
        # QUANDO cada fonte falou pela última vez. Guardar só a mensagem não
        # basta: se o /Odometry morre no meio da corrida, `self.pose` congela no
        # último valor, a derivada passa a dar 0,0 e o ensaio termina inteiro
        # com pose repetida e cara de sucesso. Foi o que aconteceu em 31-07,
        # quando a rede caiu e 7 corridas fecharam com "205 amostras".
        self.t_pose = None
        self.t_roda = None
        self.janela = cfg.janela
        self.hist = deque()          # (t, x, y, yaw) para diferenciar
        self.hist_roda = deque()     # a mesma coisa, pela odometria de roda
        self.t0 = None
        self.p0 = None
        self.p0_roda = None
        self.t_ant = -1.0
        self.linhas = []
        self.fim = False
        self.motivo = 'concluído'
        self.normal = True       # o fim foi um fim, ou foi um aborto?

        # Estado do dente de serra (só os ensaios de zona morta usam).
        self.dente = 0
        self.fase = 'sobe'
        self.nivel = 0.0
        self.te_cmd = None       # te da chamada anterior, para integrar a rampa
        self.gatilho = None      # início da condição de troca de fase
        self.eventos = []        # (dente, fase, nível) — só para o resumo na tela

        self.create_timer(1.0 / cfg.taxa, self.passo)

    def cb_pose(self, msg):
        self.pose = msg
        self.t_pose = self.agora()

    def cb_roda(self, msg):
        self.roda = msg
        self.t_roda = self.agora()

    def agora(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def derivada(self, hist, t, x, y, yaw):
        """Velocidade linear e de guinada tiradas da POSE, numa janela curta.

        `hist` é a fila da fonte (LIO ou roda): as duas são diferenciadas do
        mesmo jeito, na mesma janela, para que os números do CSV sejam
        comparáveis entre si — foi essa comparação que denunciou o LIO em 31-07.
        """
        hist.append((t, x, y, yaw))
        while len(hist) > 1 and t - hist[0][0] > self.janela:
            hist.popleft()
        if len(hist) < 2:
            return 0.0, 0.0
        t1, x1, y1, yaw1 = hist[0]
        dt = t - t1
        if dt < 1e-6:
            return 0.0, 0.0
        return math.hypot(x - x1, y - y1) / dt, norm_ang(yaw - yaw1) / dt

    # ---------------- os ensaios ----------------

    def dente_de_serra(self, te, med):
        """Rampa que SOBE até o robô sair da inércia e DESCE até ele parar.

        Devolve a magnitude comandada (com sinal). É o único ensaio do banco em
        malha fechada, e é de propósito.

        Uma corrida entrega três coisas que a rampa de subida única não dava:

         · N saídas da inércia em vez de uma. Cada dente parte do REPOUSO com o
           rotor numa posição diferente, que é a fonte real de dispersão de um
           limiar de atrito estático. É a repetição do ensaio sem reposicionar
           o robô e sem gastar corrida.
         · o limiar de QUEDA — o comando em que ele para, já andando. Atrito
           dinâmico é menor que o estático, então é um número menor que o de
           saída, e é ELE que o piso de velocidade do seguidor precisa (manter
           andando o que já anda). A rampa só de subida não media isso.
         · os dois sentidos, que nesta máquina não têm por que ser iguais:
           motriz na frente, boba atrás, e de ré a boba vira roda dianteira.

        Por que virar no EVENTO e não no relógio: subindo até o teto sempre, o
        comando continua crescendo muito depois de já ter achado o número, e
        todo esse trecho é metro e segundo jogados fora. Medido antes de
        existir: com virada por tempo o ensaio linear se afastava **5,25 m** da
        origem — a trava de espaço (3 m) mataria a corrida dentro do primeiro
        dente, e o CSV traria uma saída só, pior que a versão antiga.

        A TAXA da rampa é a mesma de sempre (`rampa_ate / rampa_seg`, com
        `rampa_seg` = 20 s, que reproduz a rampa única). Não é conservadorismo:
        o limiar é lido na primeira amostra que passa de PAROU, então rampa
        mais rápida infla o número medido pelo atraso de detecção. No giro, a
        rampa de hoje já infla ~0,011 rad/s, e a decisão do pivô se joga entre
        0,10 e 0,15. Mais dentes custam TEMPO, nunca precisão.
        """
        c = self.cfg
        dt = te - self.te_cmd if self.te_cmd is not None else 0.0
        self.te_cmd = te
        if not 0.0 < dt < 0.5:            # primeira chamada, ou engasgo
            dt = 1.0 / c.taxa

        if self.dente >= c.dentes:
            self.parar(f'{c.dentes} dentes concluídos', normal=True)
            return 0.0

        taxa = c.rampa_ate / c.rampa_seg
        sinal = 1.0 if self.dente % 2 == 0 else -1.0

        def confirmou(cond, quanto):
            """Exige que a condição dure — pose tem ruído, e um pico só não é
            saída da inércia nem parada."""
            if not cond:
                self.gatilho = None
                return False
            if self.gatilho is None:
                self.gatilho = te
            return te - self.gatilho >= quanto

        if self.fase == 'sobe':
            self.nivel = min(c.rampa_ate, self.nivel + taxa * dt)
            if confirmou(abs(med) > SAIU, CONFIRMA_SAIU):
                self.eventos.append((self.dente, 'saiu', sinal * self.nivel))
                self.fase, self.gatilho = 'desce', None
            elif self.nivel >= c.rampa_ate - 1e-9:
                # Bateu o teto sem sair do lugar. Não é falha do ensaio — é o
                # resultado, e o medir.py vai dizer isso. Desce e tenta o
                # próximo dente mesmo assim.
                self.eventos.append((self.dente, 'teto', sinal * self.nivel))
                self.fase, self.gatilho = 'desce', None

        elif self.fase == 'desce':
            self.nivel = max(0.0, self.nivel - taxa * dt)
            if confirmou(abs(med) < PAROU, CONFIRMA_PAROU):
                self.eventos.append((self.dente, 'parou', sinal * self.nivel))
                self.fase, self.gatilho, self.nivel = 'pausa', te, 0.0
            elif self.nivel <= 0.0:
                self.eventos.append((self.dente, 'zerou', 0.0))
                self.fase, self.gatilho = 'pausa', te

        elif self.fase == 'pausa':
            self.nivel = 0.0
            if te - self.gatilho >= PAUSA_DENTE:
                self.dente += 1
                # Encerrar AQUI, e não na próxima passada: deixar o contador
                # andar antes de parar gravava uma linha de um dente que nunca
                # existiu, e a leitura a contava como "não saiu do lugar".
                if self.dente >= c.dentes:
                    self.parar(f'{c.dentes} dentes concluídos', normal=True)
                    return 0.0
                self.fase, self.gatilho = 'sobe', None

        return sinal * self.nivel

    def comando(self, te, v_pose, wz_pose):
        """Devolve (v, wz) do ensaio no instante te. Um lugar só."""
        c = self.cfg
        e = c.ensaio

        if e == 'zona_morta_linear':
            return self.dente_de_serra(te, v_pose), 0.0

        if e == 'zona_morta_giro':
            # Girando parado é o pior caso da zona morta: as duas rodas ficam
            # na banda proibida ao mesmo tempo.
            #
            # O gatilho compara VELOCIDADE DE BORDA DE RODA (wz·L/2), não wz.
            # Em 31-07 ele comparava wz cru contra o mesmo SAIU=0,03 da reta —
            # que na reta são 0,03 m/s de roda e no giro eram 0,0040 m/s: sete
            # vezes mais sensível. O ensaio derrubava a rampa no rastejo do
            # eixo (2° de encoder por dente, com o dono vendo o robô PARADO) e
            # devolvia 0,032 rad/s, um limiar de folga mecânica. Convertendo,
            # os dois ensaios passam a perguntar a mesma coisa da mesma roda.
            return 0.0, self.dente_de_serra(te, wz_pose * self.cfg.bitola / 2)

        if e == 'degrau_giro':
            # 2 s reto, `--liga` segundos de giro constante, resto SEM comando.
            # a_dec = wz² / (2·Δrumo depois do corte).
            #
            # `--liga` existe para a fatia 3 da decisão 011: a placa entrega um
            # wz só, então o tamanho do pivô não se escolhe pelo comando — se
            # escolhe pelo TEMPO ligado. Varrer esse tempo é o que responde
            # "existe pivô pequeno?". Padrão 2,0 = o degrau de 04-08 intacto.
            if te < 2.0:
                return c.v, 0.0
            if te < 2.0 + self.cfg.liga:
                return c.v, c.wz
            return c.v, 0.0

        if e == 'curva':
            # Curva sustentada: mede o wz REALIZADO contra o comandado e a
            # derrapada (odometria de roda contra pose). Rodar em várias
            # velocidades é o que responde "quanto ele curva a x, 2x, 3x".
            return (c.v, c.wz) if te >= 2.0 else (c.v, 0.0)

        if e == 'reta':
            # Reta com CUTUCÃO: anda, leva um pulso de giro de 0,5 s e SOLTA.
            # `--v` aceita NEGATIVO, e é assim que se mede a ré — de ré a boba
            # passa a ser a roda da frente, e boba na frente é a configuração
            # geometricamente instável (o carrinho de supermercado).
            #
            # O pulso não é enfeite: sem ele o ensaio não mede nada. O robô
            # simulado é perfeitamente simétrico num plano liso, então reta
            # pura dá desvio ZERO EXATO nos dois sentidos (medido: 0,0° e
            # 0,0 cm em 4 m, ida e ré). Instabilidade é bifurcação — só se vê
            # perturbando e olhando se o desvio volta ou cresce.
            #
            # `--wz 0` desliga o pulso, para quem quiser a reta crua no robô
            # real, onde a assimetria de verdade perturba sozinha.
            if te < 2.0:
                return 0.0, 0.0
            if 5.0 <= te < 5.5:
                return c.v, c.wz
            return c.v, 0.0

        if e == 'aceleracao_linear':
            # Degrau de linear e corte: acelera e desacelera de fato quanto?
            if te < 2.0:
                return 0.0, 0.0
            if te < 6.0:
                return c.v, 0.0
            return 0.0, 0.0

        raise SystemExit(f'ensaio desconhecido: {e}')

    # ---------------- laço ----------------

    def passo(self):
        # Espera a fonte que VIRA O DENTE. A outra é registro: se ela faltar, o
        # ensaio anda mesmo assim e o CSV fica com a coluna vazia — mas rodar
        # sem quem decide seria medir o relógio.
        if self.pose is None and self.cfg.fonte == 'lio':
            return
        if self.roda is None and self.cfg.fonte == 'roda':
            return
        t = self.agora()
        if self.t0 is None:
            self.t0 = t
            if self.pose is not None:
                self.p0 = (self.pose.pose.pose.position.x,
                           self.pose.pose.pose.position.y)
            if self.roda is not None:
                self.p0_roda = (self.roda.pose.pose.position.x,
                                self.roda.pose.pose.position.y)
        te = t - self.t0

        if te < self.t_ant - 1e-6:
            self.parar('relógio andou pra trás (mais de uma fonte de /clock?)')
            return
        self.t_ant = te

        # Trava de dado parado: a corrida já começou, então a fonte que vira o
        # dente TEM de continuar falando. Congelada, ela não devolve erro
        # nenhum — devolve velocidade zero, que é uma leitura plausível, e é
        # justamente por isso que a corrida inteira sai com cara de medida.
        # Vale só para a fonte que decide; a outra é registro, e o CSV já
        # aceita coluna vazia.
        fonte, t_fonte = (('/hoverboard_base_controller/odom', self.t_roda)
                          if self.cfg.fonte == 'roda'
                          else ('/Odometry', self.t_pose))
        if t_fonte is not None and t - t_fonte > self.cfg.sem_dado:
            self.parar(f'{fonte} calou por {t - t_fonte:.1f} s — corrida '
                       f'ABORTADA, o que já foi gravado não é medida')
            return

        x = y = yaw = None
        v_pose = wz_pose = 0.0
        if self.pose is not None:
            p = self.pose.pose.pose
            x, y = p.position.x, p.position.y
            yaw = yaw_de(p.orientation)
            v_pose, wz_pose = self.derivada(self.hist, t, x, y, yaw)

        xr = yr = yaw_r = None
        v_roda = wz_roda = 0.0
        if self.roda is not None:
            pr = self.roda.pose.pose
            xr, yr = pr.position.x, pr.position.y
            yaw_r = yaw_de(pr.orientation)
            v_roda, wz_roda = self.derivada(self.hist_roda, t, xr, yr, yaw_r)

        # Trava de espaço: o robô é real e o laboratório tem parede. Vale a
        # fonte MAIS ALARMISTA das duas, de propósito — a que erra para longe
        # só custa um ensaio interrompido; a que erra para perto custa parede.
        for pos, p0 in ((( x, y), self.p0), ((xr, yr), self.p0_roda)):
            if pos[0] is None or p0 is None:
                continue
            if math.hypot(pos[0] - p0[0], pos[1] - p0[1]) > self.cfg.espaco:
                self.parar(f'estourou o espaço de {self.cfg.espaco:.1f} m')
                return
        if te > self.cfg.dur:
            self.parar('concluído', normal=True)
            return

        if self.cfg.fonte == 'roda':
            cv, cw = self.comando(te, v_roda, wz_roda)
        else:
            cv, cw = self.comando(te, v_pose, wz_pose)
        if self.fim:                      # o dente de serra pode encerrar aqui
            return
        self.publica(cv, cw)

        self.linhas.append({
            't': round(te, 3),
            # Quem virou o dente foi o ensaio; quem MEDE o limiar é o medir.py.
            # Estas duas colunas são a costura entre os dois: sem elas a leitura
            # teria de readivinhar onde cada rampa começou e acabou.
            'dente': self.dente,
            'fase': self.fase,
            'x': round(x, 4) if x is not None else '',
            'y': round(y, 4) if y is not None else '',
            'yaw': round(yaw, 4) if yaw is not None else '',
            'cmd_v': round(cv, 4), 'cmd_wz': round(cw, 4),
            'v_pose': round(v_pose, 4), 'wz_pose': round(wz_pose, 4),
            'v_twist': (round(self.pose.twist.twist.linear.x, 4)
                        if self.pose else ''),
            'wz_twist': (round(self.pose.twist.twist.angular.z, 4)
                         if self.pose else ''),
            'x_roda': round(xr, 4) if xr is not None else '',
            'y_roda': round(yr, 4) if yr is not None else '',
            'yaw_roda': round(yaw_r, 4) if yaw_r is not None else '',
            # Derivadas da roda, na MESMA janela das do LIO. Colunas novas no
            # fim: o medir.py lê por nome, então CSV antigo continua legível.
            'v_roda': round(v_roda, 4), 'wz_roda': round(wz_roda, 4),
        })

    def parar(self, motivo, normal=False):
        """`normal=True` só para os dois fins legítimos (teto de tempo e dentes
        fechados). Todo o resto é aborto, e aborto tem de chegar ao `sessao.py`
        como código de saída — ele já para e pergunta (`sessao.py`, o `if
        roda(cmd, log) != 0`); o que faltava era alguém devolver o erro."""
        self.motivo = motivo
        self.normal = normal
        self.publica(0.0, 0.0)
        self.fim = True

    def publica(self, v, wz):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(wz)
        self.pub.publish(m)

    def grava(self):
        # Zerar de novo na saída: se o nó morreu no meio, o robô não pode
        # continuar andando com o último comando.
        for _ in range(5):
            self.publica(0.0, 0.0)
        if not self.linhas:
            fonte = ('/Odometry' if self.cfg.fonte == 'lio'
                     else '/hoverboard_base_controller/odom')
            print(f'sem dados — o {fonte} chegou?', file=sys.stderr)
            self.normal = False
            return
        with open(self.cfg.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(self.linhas[0].keys()))
            w.writeheader()
            w.writerows(self.linhas)
        print(f'{len(self.linhas)} amostras -> {self.cfg.csv} ({self.motivo})',
              file=sys.stderr)
        if self.eventos:
            # Resumo cru do que o dente de serra viu. Não é a medida (quem mede
            # é o medir.py, lendo o CSV) — é para o operador perceber ainda no
            # laboratório que um dente não fechou.
            print('  dentes: ' + '  '.join(
                f'#{d}:{q}={n:+.3f}' for d, q, n in self.eventos),
                file=sys.stderr)


ENSAIOS = ['zona_morta_linear', 'zona_morta_giro', 'degrau_giro', 'curva',
           'aceleracao_linear', 'reta']


def main():
    ap = argparse.ArgumentParser(
        description='Banco de ensaios de movimentação (robô real ou simulador)')
    ap.add_argument('--ensaio', choices=ENSAIOS, required=True)
    ap.add_argument('--csv', required=True)
    ap.add_argument('--dur', type=float, default=20.0, help='teto de tempo [s]')
    ap.add_argument('--espaco', type=float, default=4.0,
                    help='distância máxima da origem [m] — trava de segurança')
    ap.add_argument('--v', type=float, default=0.3, help='linear do ensaio [m/s]')
    ap.add_argument('--wz', type=float, default=0.5, help='giro do ensaio [rad/s]')
    ap.add_argument('--liga', type=float, default=2.0,
                    help='segundos com o giro LIGADO no degrau_giro. Varrer '
                         'isto mede o pivô mínimo (fatia 3 da decisão 011): '
                         'a placa não modula wz, só o tempo é escolhível')
    ap.add_argument('--rampa-ate', dest='rampa_ate', type=float, default=0.35,
                    help='valor final da rampa nos ensaios de zona morta')
    ap.add_argument('--dentes', type=int, default=4,
                    help='dentes de serra nos ensaios de zona morta. Cada dente '
                         'é uma saída da inércia E uma queda medidas, e o '
                         'sentido alterna a cada um. Custa tempo, não precisão')
    ap.add_argument('--rampa-seg', dest='rampa_seg', type=float, default=20.0,
                    help='segundos de 0 até --rampa-ate: é a TAXA da rampa. '
                         'Subir mais rápido infla o limiar medido pelo atraso '
                         'de detecção — mexer aqui é mexer no número')
    ap.add_argument('--sem-dado', dest='sem_dado', type=float, default=1.0,
                    help='segundos sem mensagem nova da fonte antes de ABORTAR '
                         'a corrida [s]. O /Odometry vem a 10 Hz no robô, '
                         'então 1,0 s são 10 amostras perdidas — e como a '
                         '--janela no robô é 0,5 s, um buraco desse tamanho já '
                         'invalida toda derivada da janela')
    ap.add_argument('--janela', type=float, default=JANELA_S,
                    help='janela da derivação da pose [s]. 0,2 serve para o '
                         'simulador (50 Hz); no robô, com /Odometry a 10 Hz, '
                         'use 0,5 — janela curta vira ruído inventado')
    ap.add_argument('--bitola', type=float, default=0.270,
                    help='bitola [m], da trena de 29-07. Converte o giro em '
                         'velocidade de borda de roda, que é o que o gatilho '
                         'da zona morta compara. Tem de bater com o '
                         'wheel_separation do controlador (sessao.py --checar '
                         'delata) e com o --bitola do medir.py')
    ap.add_argument('--fonte', choices=['lio', 'roda'], default='lio',
                    help='quem vira o dente de serra: a pose do LIO (padrão, '
                         'mede o CORPO e enxerga derrapagem) ou a odometria de '
                         'roda (mede o EIXO; use quando o LIO estiver ruidoso '
                         'demais para servir de gatilho). As duas vão para o '
                         'CSV sempre — isto escolhe só quem DECIDE')
    ap.add_argument('--taxa', type=float, default=50.0, help='malha do banco [Hz]')
    ap.add_argument('--sim', action='store_true',
                    help='usar tempo de simulação (no robô real, NÃO passar)')
    # No robô o comando entra no controlador e a placa DE VERDADE está depois
    # dele, no fio. No simulador a placa fingida é um nó à parte, escutando
    # `/cmd_vel_bruto` — publicar direto no controlador PASSA POR FORA dela e
    # mede um robô sem atuador. Foi o que aconteceu em 31-07 ("o Gazebo provou
    # o mecanismo, não o número"). Com isto o ensaio pode entrar pela mesma
    # porta que a navegação usa, e aí os dois lados ficam comparáveis.
    ap.add_argument('--topico', default='/hoverboard_base_controller/cmd_vel',
                    help='onde publicar o comando. No simulador, use '
                         '/cmd_vel_bruto para atravessar a placa fingida')
    cfg = ap.parse_args()

    rclpy.init()
    no = Ensaio(cfg)
    try:
        while rclpy.ok() and not no.fim:
            rclpy.spin_once(no, timeout_sec=0.1)
    except KeyboardInterrupt:
        no.motivo = 'interrompido no teclado'
        no.normal = False
    finally:
        no.grava()
        ok = no.normal
        no.destroy_node()
        rclpy.shutdown()
    # Corrida abortada sai com erro para o `sessao.py` PARAR e perguntar. Sem
    # isso ela some no meio de 27 corridas e só aparece na análise, em casa.
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
