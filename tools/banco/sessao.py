#!/usr/bin/env python3
"""Sessão de bancada: o protocolo inteiro do README, em ordem, num comando só.

    python3 tools/banco/sessao.py --checar          # só a conferência (não anda)
    python3 tools/banco/sessao.py --checar --mexer  # conferência + cutucão de sanidade
    python3 tools/banco/sessao.py                   # a sessão inteira
    python3 tools/banco/sessao.py --de 3            # retomar do passo 3

Existe porque o `ensaio.py` mede UM ensaio e o protocolo tem SEIS, cada um com
argumentos próprios, espaço próprio e uma pose de partida própria. Digitar isso
na mão, com o robô ligado e a mão no disjuntor, é onde se erra o `--wz`, se
sobrescreve um CSV ou se pula um ensaio — e o custo é voltar ao laboratório.

O que ele garante, e o `ensaio.py` sozinho não garante:

1. **Nada mede antes da conferência passar.** Sem `/Odometry` toda velocidade
   daqui sai de uma pose que não existe, e o CSV sai limpo e errado. Falha
   silenciosa é o modo de falhar desta pilha (ver BO-3 e o `bind failed` do
   lidar), então a conferência é obrigatória e bloqueia.
2. **Um CSV por ensaio, numa pasta só, com o ambiente escrito ao lado.** Piso e
   bateria mudam derrapada e zona morta; medida sem eles anotados não se compara
   com a próxima sessão nem entra no artigo.
3. **A leitura sai na hora.** Cada ensaio chama o `medir.py` logo depois e o
   número aparece ainda no laboratório — se o resultado for absurdo, dá para
   repetir com o robô ligado em vez de descobrir em casa.
"""

import argparse
import datetime
import math
import os
import subprocess
import sys
import time

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
ENSAIO = os.path.join(AQUI, 'ensaio.py')
MEDIR = os.path.join(AQUI, 'medir.py')


# --------------------------------------------------------------- o protocolo
#
# A ordem é a do README e não é gosto: a zona morta vem primeiro porque é ela
# que decide se o robô anda, e um robô que não sai do lugar invalida todo
# ensaio seguinte sem dar erro nenhum.
#
# E dentro dela, o GIRO na frente do linear (invertido em 31-07, a pedido do
# dono). Três razões, em ordem de peso:
#   1. o ensaio de giro responde a pergunta do pivô DIRETAMENTE — o menor wz
#      que gira o robô parado É o limiar do pivô, em rad/s. Pelo linear só se
#      chega lá convertendo por 2·zm/L, isto é, confiando na bitola de novo;
#   2. sessão cortada perde o que estiver por último, e em 30-07 a sessão foi
#      bloqueada sem medir nada — o risco não é hipotético;
#   3. é o mais barato de montar: gira parado, raio de 1 m, sem corredor.
# O argumento contrário ("andar reto é mais manso que girar") já está coberto
# pelo cutucão, que anda E gira antes de qualquer ensaio.

# `repete` é o número de corridas IDÊNTICAS da mesma condição, e existe porque
# uma corrida só não é medida — é uma amostra. Média de 3 mata o erro
# aleatório; o que ela NÃO mata é erro sistemático (caimento do piso, por
# exemplo), e por isso o passo 6 tem uma corrida girada 180° em vez de só mais
# uma repetição: é a única que separa "o robô puxa para a direita" de "o chão
# cai para a direita".
#
# Repetir tudo x3 dariam 33 corridas e ~50 min de bateria. A prioridade seguiu
# o que o dono decidiu em 31-07: repete o que identifica ERRO (reta, curva,
# aceleração); os ensaios de zona morta não repetem em corrida porque o dente
# de serra já entrega N saídas da inércia dentro de UMA corrida.

PASSOS = [
    dict(
        n=1, tipo='zona_morta_giro',
        titulo='Zona morta de GIRO — o item nº 1 do projeto, e o primeiro a sair',
        espaco='raio de 1 m livre em volta',
        pose='Robô parado, no meio do espaço livre. Não precisa de rumo nenhum.',
        corridas=[dict(csv='1-zona_morta_giro.csv',
                       args=['--dur', '220', '--rampa-ate', '1.5',
                             '--espaco', '1.5', '--dentes', '4'])],
        nota='PRIMEIRO de todos, e não por importância: porque ele responde a\n'
             'pergunta do pivô DIRETAMENTE. O que sai daqui é o menor wz que\n'
             'gira o robô parado, que É o limiar do pivô, em rad/s, sem passar\n'
             'por bitola nenhuma. O ensaio linear chega no mesmo lugar por\n'
             'conversão (2·zm/L), ou seja, confiando de novo num número medido.\n'
             'Some a isso o risco de sessão cortada (em 30-07 não se mediu\n'
             'NADA): o que estiver em segundo lugar é o que se perde. E é o\n'
             'ensaio mais barato de montar — gira parado, não precisa corredor.\n'
             'A FAIXA importa tanto quanto a média: a decisão do pivô se joga\n'
             'dentro dela, com folga de 4%. Por isso são 4 dentes.\n'
             'Ele vai ficar parado com o comando subindo, 4 vezes, alternando o\n'
             'sentido. Não é travamento — é a zona morta acontecendo, e é o que\n'
             'viemos medir. Deixar rodar.',
    ),
    dict(
        n=2, tipo='zona_morta_linear',
        titulo='Zona morta linear — o mesmo limiar, pela outra porta',
        espaco='~3 m em linha reta à frente (ele vai e volta, quase não sai do lugar)',
        pose='Robô parado, apontando para o lado livre mais comprido.',
        corridas=[dict(csv='2-zona_morta_linear.csv',
                       args=['--dur', '180', '--rampa-ate', '0.35',
                             '--espaco', '3.0', '--dentes', '4'])],
        nota='DENTE DE SERRA, como o passo 1. Andando reto a velocidade da roda\n'
             'é a velocidade do robô, então o que sai daqui é o limiar da roda\n'
             'em m/s DIRETO, sem bitola no meio — e é ele que entra no piso de\n'
             'velocidade do seguidor (v_piso).\n'
             'Vale também como CONFERÊNCIA do passo 1: os dois medem o mesmo\n'
             'atrito por caminhos diferentes, e têm de fechar por 2·zm/L. Se não\n'
             'fecharem, ou a bitola está errada ou as duas rodas não são iguais.\n'
             'Se NÃO sair do lugar, isso não é falha do ensaio, é o resultado;\n'
             'refazer com --rampa-ate 0.6.',
    ),
    dict(
        n=3, tipo='degrau_giro',
        titulo='Degrau de giro — quanto ele demora pra PARAR de girar (a_dec)',
        espaco='~4 m; ele termina apontando para outro lado',
        pose='Robô no ponto 0, apontando para o comprido — o MESMO ponto e o\n'
             'MESMO rumo em todas as repetições.',
        corridas=[
            dict(csv='3-degrau_wz03.csv',
                 args=['--v', '0.3', '--wz', '0.3', '--dur', '12']),
            dict(csv='3-degrau_wz06.csv', repete=3,
                 args=['--v', '0.3', '--wz', '0.6', '--dur', '12']),
            dict(csv='3-degrau_wz10.csv',
                 args=['--v', '0.3', '--wz', '1.0', '--dur', '12']),
        ],
        nota='a_dec é a causa medida do S. Os três níveis mostram se ele é\n'
             'constante ou piora com giro forte; o nível do meio vai 3x para\n'
             'dar a dispersão, que se aplica aos outros dois.',
    ),
    dict(
        n=4, tipo='curva',
        titulo='Curva sustentada — quanto ele curva a 1x, 2x, 3x de velocidade',
        espaco='círculo de raio v/wz (a 0,6 m/s e 0,5 rad/s são 1,2 m de raio)',
        pose='Robô no ponto 0, apontando para o comprido. MESMO ponto e MESMO\n'
             'rumo nas três repetições de cada velocidade.',
        corridas=[
            dict(csv='4-curva_v02.csv', repete=3,
                 args=['--v', '0.2', '--wz', '0.5', '--dur', '12']),
            dict(csv='4-curva_v04.csv', repete=3,
                 args=['--v', '0.4', '--wz', '0.5', '--dur', '12']),
            dict(csv='4-curva_v06.csv', repete=3,
                 args=['--v', '0.6', '--wz', '0.5', '--dur', '12']),
        ],
        nota='É aqui que a geometria aparece: motriz na frente, boba atrás.\n'
             'Se o giro realizado CAIR conforme a velocidade sobe, está medido,\n'
             'e vira restrição de projeto. Derrapada é a grandeza mais dispersa\n'
             'do banco — daí as três velocidades irem 3x cada.',
    ),
    dict(
        n=5, tipo='aceleracao_linear',
        titulo='Aceleração linear — arranca e freia quanto?',
        espaco='~4 m em reta',
        pose='Robô no ponto 0, apontando para o comprido.',
        corridas=[dict(csv='5-aceleracao.csv', repete=3,
                       args=['--v', '0.6', '--dur', '10'])],
        nota='Confere se os tetos do hoverboard_controllers.yaml (0,7 m/s e\n'
             '0,8 m/s²) descrevem ESTA máquina ou foram herdados sem medir.',
    ),
    dict(
        n=6, tipo='reta',
        titulo='Reta com cutucão — o rumo volta ou foge? E de ré? (BO-4)',
        espaco='~4 m em reta, nos DOIS sentidos',
        pose='Robô no MEIO do espaço: ele vai andar para frente numa corrida e\n'
             'para trás na outra. Marcar o ponto 0 COM O RUMO no chão.',
        corridas=[
            dict(csv='6-reta_frente.csv', repete=2,
                 args=['--v', '0.25', '--wz', '0.5', '--dur', '16']),
            dict(csv='6-reta_re.csv', repete=2,
                 args=['--v', '-0.25', '--wz', '0.5', '--dur', '16']),
            dict(csv='6-reta_crua.csv', repete=3,
                 args=['--v', '0.25', '--wz', '0', '--dur', '16']),
            dict(csv='6-reta_crua_180.csv', gira_180=True,
                 args=['--v', '0.25', '--wz', '0', '--dur', '16']),
        ],
        nota='FILMAR A TRASEIRA nas corridas de frente e de ré. O que se procura\n'
             'é a boba dando meia-volta na corrida de ré, e quanto o robô se\n'
             'desvia enquanto ela decide. É o ensaio que fecha o BO-4 e decide\n'
             'se a manobra de ré da decisão 007 é segura.\n'
             'A reta CRUA é sem pulso, e a leitura vai dizer "nada a comparar" —\n'
             'está certo: o medir.py resume a resposta ao cutucão, e aqui não há\n'
             'cutucão. O CSV é que interessa, e ele só vale no robô: no simulador\n'
             'a máquina é simétrica e o desvio sai zero exato.',
    ),
]


class GiroAcumulado:
    """Soma giro DESENROLADO, amostra a amostra.

    Existe porque `yaw_final − yaw_inicial` passado por `atan2` **enrola em
    ±180°**, e este robô gira muito mais que meia volta num cutucão de 2 s (o
    patamar da compensação infla o pivô 2,8 a 3,7×). Enrolado, um giro real de
    `+281,5°` é lido como `−78,5°` — com o SINAL TROCADO. Foi exatamente esse
    número que bloqueou a sessão de 30-07 como "giro espelhado", e que quase
    levou alguém a trocar as rodas de um robô que estava certo (ver DIARIO
    04-08).

    Somando entre amostras consecutivas o problema some: a 10 Hz duas amostras
    só distariam 180° se o robô girasse a 31 rad/s.
    """

    def __init__(self, yaw0):
        self.total = 0.0
        self._ant = yaw0

    def soma(self, yaw):
        self.total += math.atan2(math.sin(yaw - self._ant),
                                 math.cos(yaw - self._ant))
        self._ant = yaw
        return self.total


# --------------------------------------------------------------- conferência

# Medidos com trena NESTE robô em 2026-07-29 (ver ESTADO_PROJETO.md). Não são
# preferência de projeto: são a régua contra a qual se confere o que a base
# realmente carregou.
TRENA = {'wheel_separation': 0.270, 'wheel_radius': 0.080}

CONTROLADOR = 'hoverboard_base_controller'
PARAMS = ['wheel_separation', 'wheel_radius',
          'left_wheel_names', 'right_wheel_names']


def calibracao_viva(no, timeout=5.0):
    """Pergunta ao controlador QUE ROBÔ ele acha que está dirigindo.

    Existe porque o `ambiente.txt` gravava o **commit**, e commit descreve o
    FONTE. Quem dirige o robô é a cópia em `install/`: o `tracao.launch.py` lê
    o YAML e o xacro via `FindPackageShare`. Trocar o fonte sem recompilar deixa
    os dois divergindo em silêncio, e aí:

      · a bitola entra na conversão comando→roda, então um valor errado desloca
        TODO limiar medido (0,32 num robô de 0,270 são 18,5%);
      · pior, esse desvio é indistinguível de derrapada depois, em casa: os dois
        mexem no mesmo número em sentidos opostos e podem se cancelar.

    Também devolve os nomes de roda, que é onde vive a correção do giro
    espelhado de 30-07 — dá para ver, antes de medir, se o swap está no ar.

    NÃO bloqueia. Divergir pode ser deliberado; o que não pode é ninguém saber.
    O número vai para o `ambiente.txt`, e aí o dado continua interpretável mesmo
    se a calibração estiver errada.
    """
    import rclpy
    from rclpy.parameter import parameter_value_to_python
    from rclpy.parameter_client import AsyncParameterClient

    linhas, calib = [], {}
    cli = AsyncParameterClient(no, CONTROLADOR)
    if not cli.wait_for_services(timeout_sec=timeout):
        linhas.append(f'  [aviso] {CONTROLADOR} não respondeu ao serviço de '
                      f'parâmetros.')
        linhas.append('          Os ensaios rodam, mas fica sem registro de QUAL '
                      'calibração os gerou.')
        return linhas, calib

    fut = cli.get_parameters(PARAMS)
    rclpy.spin_until_future_complete(no, fut, timeout_sec=timeout)
    res = fut.result()
    if res is None:
        linhas.append('  [aviso] o pedido de parâmetros não voltou a tempo.')
        return linhas, calib

    for nome, valor in zip(PARAMS, res.values):
        try:
            calib[nome] = parameter_value_to_python(valor)
        except Exception:
            calib[nome] = None

    return linhas + laudo_calibracao(calib), calib


def swap_aplicado(calib):
    """As rodas estão trocadas no YAML? É onde vive a correção do giro
    espelhado achado em 30-07 (`+0,6 rad/s` girou `−78,5°`)."""
    esq = calib.get('left_wheel_names') or []
    dir_ = calib.get('right_wheel_names') or []
    if not esq or not dir_:
        return None
    return 'right' in str(esq[0]) and 'left' in str(dir_[0])


def laudo_calibracao(calib):
    """Compara o que a base carregou com a trena. Parte pura, para poder ser
    testada sem subir ROS — a lógica é o que erra, não o transporte."""
    linhas = []
    for nome, esperado in TRENA.items():
        v = calib.get(nome)
        if v is None:
            linhas.append(f'  [aviso] {nome} não veio do controlador')
        elif abs(v - esperado) < 1e-6:
            linhas.append(f'  [ok] {nome} = {v:.4f}  (bate com a trena)')
        else:
            linhas.append(f'  [ATENÇÃO] {nome} = {v:.4f}, e a trena mediu '
                          f'{esperado:.4f}')
            linhas.append(f'            desvio de {100 * (v - esperado) / esperado:+.1f}% '
                          f'— ou o build faltou, ou alguém mudou de propósito.')
            linhas.append(f'            Não estou parando a sessão: fica '
                          f'gravado no ambiente.txt e o')
            linhas.append(f'            dado continua interpretável. Mas confira '
                          f'antes de medir.')

    trocado = swap_aplicado(calib)
    if trocado is not None:
        linhas.append(f'  [info] rodas: esquerda={list(calib["left_wheel_names"])}'
                      f'  direita={list(calib["right_wheel_names"])}')
        if trocado:
            linhas.append('         -> swap esquerda/direita APLICADO (a correção '
                          'do giro espelhado de 30-07)')
        else:
            linhas.append('         -> swap NÃO aplicado (ordem original). Se o '
                          'cutucão acusar giro')
            linhas.append('            invertido, é este o arquivo a mexer: '
                          'hoverboard_controllers.yaml')
    return linhas


def confere(sim=False, mexer=False):
    """Prova que a base está de pé ANTES de qualquer medida.

    Devolve (ok, linhas). Bloqueia a sessão quando falha: um CSV gravado sem
    `/Odometry` vivo é um arquivo limpo cheio de zeros, que só se descobre em
    casa. Este é o único ponto do banco que pode dizer não.
    """
    import rclpy
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy

    linhas = []
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    rclpy.init()
    no = Node('confere_bancada')
    if sim:
        no.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

    conta = {'pose': 0, 'roda': 0}
    ultimo = {'pose': None, 'roda': None}

    def cb(qual):
        def f(msg):
            conta[qual] += 1
            ultimo[qual] = msg
        return f

    no.create_subscription(Odometry, '/Odometry', cb('pose'), qos)
    no.create_subscription(
        Odometry, '/hoverboard_base_controller/odom', cb('roda'), qos)
    pub = no.create_publisher(
        TwistStamped, '/hoverboard_base_controller/cmd_vel', qos)

    JANELA = 4.0
    print(f'  ouvindo por {JANELA:.0f} s...')
    t0 = time.monotonic()
    while time.monotonic() - t0 < JANELA:
        rclpy.spin_once(no, timeout_sec=0.1)
    dt = time.monotonic() - t0

    ok = True

    hz_pose = conta['pose'] / dt
    if conta['pose'] == 0:
        ok = False
        linhas.append('  [FALHA] /Odometry NÃO publica. Sem localização não há '
                      'medida nenhuma —')
        linhas.append('          toda velocidade do banco sai da pose. Causa '
                      'provável: IP do lidar')
        linhas.append('          errado (bind failed). Ver '
                      'ros2_packages/robot_base/config/README.md.')
    else:
        linhas.append(f'  [ok] /Odometry a {hz_pose:.1f} Hz')
        p = ultimo['pose'].pose.pose.position
        linhas.append(f'       pose atual: x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f}')
        if not all(abs(v) < 1e4 for v in (p.x, p.y, p.z)):
            ok = False
            linhas.append('  [FALHA] pose absurda — o LIO não convergiu.')

    if conta['roda'] == 0:
        linhas.append('  [aviso] /hoverboard_base_controller/odom não publica. '
                      'Os ensaios rodam, mas')
        linhas.append('          a coluna de derrapada (roda × pose) do ensaio '
                      '4 sai vazia.')
    else:
        linhas.append(f'  [ok] /hoverboard_base_controller/odom a '
                      f'{conta["roda"] / dt:.1f} Hz')

    n_sub = pub.get_subscription_count()
    if n_sub == 0:
        ok = False
        linhas.append('  [FALHA] ninguém escuta '
                      '/hoverboard_base_controller/cmd_vel — o')
        linhas.append('          diff_drive_controller não está de pé. O banco '
                      'comandaria no vazio')
        linhas.append('          e gravaria um CSV de robô parado.')
    else:
        linhas.append(f'  [ok] /hoverboard_base_controller/cmd_vel tem '
                      f'{n_sub} ouvinte(s)')

    # Que robô a base acha que está dirigindo. Vem antes do cutucão de
    # propósito: se os nomes de roda estiverem trocados, isso explica de
    # antemão um giro invertido, em vez de virar mistério com o robô andando.
    linhas.append('')
    linhas.append('  --- calibração viva (o que o controlador carregou) ---')
    l_calib, calib = calibracao_viva(no)
    linhas += l_calib

    if ok and mexer:
        linhas.append('')
        linhas.append('  --- cutucão de sanidade (o robô VAI se mexer) ---')
        linhas += _cutucao(no, pub, ultimo, sim)

    no.destroy_node()
    rclpy.shutdown()
    return ok, linhas, calib


def _cutucao(no, pub, ultimo, sim):
    """Anda um pouco e gira um pouco, e confere o SINAL do que aconteceu.

    Fiação trocada entre as rodas dá um robô que anda certo e gira ao contrário,
    e nenhum dos seis ensaios acusa isso: eles medem magnitude. Sai daqui, antes
    de gastar a bateria, ou vira meia sessão jogada fora.
    """
    import rclpy
    from geometry_msgs.msg import TwistStamped

    def yaw():
        q = ultimo['pose'].pose.pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y ** 2 + q.z ** 2))

    def pos():
        p = ultimo['pose'].pose.pose.position
        return p.x, p.y

    def solta(v, wz, seg):
        """Solta o comando por `seg`, zera, e espera a inércia acabar.

        Devolve o giro ACUMULADO no percurso inteiro, desenrolado.

        Por que acumulado, e não `yaw_final − yaw_inicial`: essa diferença passa
        por `atan2` e **enrola em ±180°**. Com o patamar da compensação este robô
        gira muito mais que meia volta num cutucão de 2 s, e o resultado era lido
        com o SINAL TROCADO — `+281,5°` virava `−78,5°`, que foi exatamente o
        falso "giro espelhado" que bloqueou a sessão de 30-07 inteira e quase
        levou alguém a trocar as rodas de um robô que estava certo (04-08).

        Somando amostra a amostra o problema some: a 10 Hz, duas amostras
        consecutivas só distariam 180° se o robô girasse a 31 rad/s.
        """
        giro = GiroAcumulado(yaw())

        def acumula():
            giro.soma(yaw())

        t0 = time.monotonic()
        while time.monotonic() - t0 < seg:
            m = TwistStamped()
            m.header.stamp = no.get_clock().now().to_msg()
            m.twist.linear.x = float(v)
            m.twist.angular.z = float(wz)
            pub.publish(m)
            rclpy.spin_once(no, timeout_sec=0.02)
            acumula()
        for _ in range(10):
            pub.publish(TwistStamped())
            rclpy.spin_once(no, timeout_sec=0.02)
            acumula()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            rclpy.spin_once(no, timeout_sec=0.05)
            acumula()
        return giro.total

    saida = []

    x0, y0 = pos()
    a0 = yaw()
    solta(0.25, 0.0, 2.0)   # o giro acumulado da reta não interessa aqui
    x1, y1 = pos()
    avanco = (x1 - x0) * math.cos(a0) + (y1 - y0) * math.sin(a0)
    lateral = -(x1 - x0) * math.sin(a0) + (y1 - y0) * math.cos(a0)
    if avanco > 0.05:
        saida.append(f'  [ok] comando de +0,25 m/s andou {avanco:+.3f} m PARA '
                     f'FRENTE (lado: {lateral:+.3f} m)')
    elif avanco < -0.05:
        saida.append(f'  [FALHA] comando de +0,25 m/s andou {avanco:+.3f} m — '
                     f'PARA TRÁS. Sinal invertido.')
    else:
        saida.append(f'  [FALHA] comando de +0,25 m/s moveu {avanco:+.3f} m: o '
                     f'robô não saiu do lugar.')
        saida.append('          Ou a zona morta é maior que 0,25 m/s (o que já '
                     'é a medida do ensaio 1),')
        saida.append('          ou a placa não está recebendo. Conferir o serial '
                     'antes de seguir.')

    giro = solta(0.0, 0.6, 2.0)
    if giro > 0.15:
        saida.append(f'  [ok] comando de +0,6 rad/s girou {math.degrees(giro):+.1f}° '
                     f'— anti-horário, como manda a regra da mão direita')
        if abs(giro) > math.pi:
            saida.append(f'         (mais de meia volta: {abs(giro) / (2 * math.pi):.2f} '
                         f'voltas. É o patamar da compensação inflando o giro —')
            saida.append('          esperado neste robô, e é por isso que este número '
                         'é ACUMULADO e não\n          a diferença entre as pontas.)')
    elif giro < -0.15:
        saida.append(f'  [FALHA] comando de +0,6 rad/s girou '
                     f'{math.degrees(giro):+.1f}° — sentido INVERTIDO.')
        saida.append('          Rodas trocadas na fiação ou no YAML. Os seis '
                     'ensaios medem magnitude e')
        saida.append('          não acusariam isso; a movimentação inteira '
                     'sairia espelhada.')
    else:
        saida.append(f'  [aviso] comando de +0,6 rad/s girou só '
                     f'{math.degrees(giro):+.1f}°: girando parado ele')
        saida.append('          não venceu a zona morta. Não é falha — é o '
                     'ensaio 2 acontecendo aqui.')

    return saida


# --------------------------------------------------------------------- fluxo

def roda(cmd, log):
    """Executa e ecoa ao mesmo tempo na tela e no arquivo de leituras."""
    log.write('\n$ ' + ' '.join(cmd) + '\n')
    log.flush()
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
    for linha in p.stdout:
        sys.stdout.write(linha)
        log.write(linha)
    p.wait()
    log.flush()
    return p.returncode


def expande(corridas, forcar=None):
    """Abre cada condição nas suas N repetições, com um CSV por corrida.

    Repetição só vira medida se as corridas forem INDEPENDENTES — daí cada uma
    ter seu arquivo em vez de somarem num só. Sufixo -a, -b, -c: quem abrir a
    pasta em casa vê na hora quantas tentativas cada condição teve.
    """
    saida = []
    for c in corridas:
        n = forcar if forcar else c.get('repete', 1)
        base = c['csv']
        for k in range(1, n + 1):
            nome = base if n == 1 else base.replace(
                '.csv', f'-{chr(ord("a") + k - 1)}.csv')
            saida.append(dict(c, csv=nome, base=base, k=k, de=n))
    return saida


def espera(texto):
    try:
        input(texto)
    except (EOFError, KeyboardInterrupt):
        print('\nsessão interrompida.')
        raise SystemExit(1)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--saida', default=None,
                    help='pasta dos CSV (padrão: docs/dados/AAAA-MM-DD-bancada-robo)')
    ap.add_argument('--de', type=int, default=1, help='começar do passo N')
    ap.add_argument('--so', type=int, default=None, help='rodar só o passo N')
    ap.add_argument('--checar', action='store_true',
                    help='só a conferência, não mede nada')
    ap.add_argument('--mexer', action='store_true',
                    help='na conferência, andar e girar um pouco (robô LIVRE)')
    ap.add_argument('--sim', action='store_true',
                    help='simulador (usa /clock). No robô real, NÃO passar.')
    ap.add_argument('--sem-perguntas', action='store_true',
                    help='não pausa entre corridas — só para ensaio no simulador')
    ap.add_argument('--repete', type=int, default=None,
                    help='força N repetições em TODA condição, ignorando o '
                         'protocolo. `--repete 1` encurta a sessão quando a '
                         'bateria está acabando — ao custo de números sem faixa')
    cfg = ap.parse_args()

    onde = 'SIMULADOR' if cfg.sim else 'ROBÔ REAL'
    print('=' * 72)
    print(f'  SESSÃO DE BANCADA — {onde}')
    print('=' * 72)

    if not cfg.sim:
        print('\n  O robô VAI ANDAR SOZINHO. Antes de seguir:')
        print('    · alguém de olho, com a mão no disjuntor;')
        print('    · área livre de pelo menos 5 × 3 m;')
        print('    · ninguém entre o robô e o fim do espaço.')
        print('  Todo ensaio tem trava de distância, e a trava NÃO substitui '
              'a mão no disjuntor.')

    print('\n--- conferência da base ---')
    ok, linhas, calib = confere(sim=cfg.sim, mexer=cfg.mexer)
    for l in linhas:
        print(l)

    if not ok:
        print('\n  A conferência REPROVOU. Nada foi medido — de propósito.')
        print('  Um CSV gravado com a base incompleta sai limpo e errado, e o')
        print('  erro só aparece em casa. Corrigir e rodar de novo.')
        raise SystemExit(2)

    print('\n  Conferência ok.')
    if cfg.checar:
        return

    # ------------------------------------------------------------- a pasta
    dia = datetime.date.today().isoformat()
    saida = cfg.saida or os.path.join(
        RAIZ, 'docs', 'dados', f'{dia}-bancada-{"sim" if cfg.sim else "robo"}')
    os.makedirs(saida, exist_ok=True)
    print(f'  CSV e leituras vão para: {saida}')

    # ------------------------------------------ o ambiente, escrito ao lado
    amb = os.path.join(saida, 'ambiente.txt')
    if not os.path.exists(amb) and not cfg.sem_perguntas:
        print('\n--- ambiente (piso e bateria mudam zona morta e derrapada) ---')
        piso = input('  piso (cimento liso / carpete / epóxi / ...): ').strip()
        bat = input('  bateria (cheia / meia / fraca / tensão se souber): ').strip()
        obs = input('  observação livre (enter para pular): ').strip()
        try:
            commit = subprocess.run(['git', '-C', RAIZ, 'rev-parse', '--short', 'HEAD'],
                                    capture_output=True, text=True).stdout.strip()
        except Exception:
            commit = '?'
        with open(amb, 'w') as f:
            f.write(f'sessão de bancada — {onde}\n')
            f.write(f'data   : {datetime.datetime.now().isoformat(timespec="seconds")}\n')
            f.write(f'commit : {commit}\n')
            f.write(f'piso   : {piso}\n')
            f.write(f'bateria: {bat}\n')
            f.write(f'obs    : {obs}\n')
            # A calibração VIVA, não a do fonte. O commit acima descreve o que
            # está no git; estas linhas descrevem o que estava dirigindo o robô
            # na hora. Sem elas, um limiar medido não tem como ser convertido
            # de volta para velocidade de roda, e vira número sem unidade.
            f.write('\n[calibração que o controlador carregou]\n')
            if calib:
                for k in PARAMS:
                    f.write(f'{k:22s}: {calib.get(k)}\n')
                for k, esperado in TRENA.items():
                    v = calib.get(k)
                    if v is not None and abs(v - esperado) > 1e-6:
                        f.write(f'*** DIVERGE DA TRENA: {k} = {v} '
                                f'(medido {esperado}) ***\n')
            else:
                f.write('(o controlador não respondeu — calibração DESCONHECIDA '
                        'nesta sessão)\n')
        print(f'  anotado em {amb}')

    # ------------------------------------------------------------ os passos
    escolhidos = [p for p in PASSOS
                  if (p['n'] == cfg.so if cfg.so else p['n'] >= cfg.de)]

    leituras = os.path.join(saida, 'leituras.txt')
    with open(leituras, 'a') as log:
        log.write(f'\n\n===== sessão {datetime.datetime.now().isoformat(timespec="seconds")} '
                  f'({onde}) =====\n')
        for passo in escolhidos:
            print('\n' + '=' * 72)
            print(f'  PASSO {passo["n"]}/6 — {passo["titulo"]}')
            print('=' * 72)
            print(f'  espaço: {passo["espaco"]}')
            print(f'  pose  : {passo["pose"]}')
            print(f'\n  {passo["nota"]}')

            corridas = expande(passo['corridas'], cfg.repete)
            for i, corrida in enumerate(corridas, 1):
                alvo = os.path.join(saida, corrida['csv'])
                print(f'\n  corrida {i}/{len(corridas)}: {corrida["csv"]}' +
                      (f'   [repetição {corrida["k"]} de {corrida["de"]}]'
                       if corrida['de'] > 1 else ''))
                if corrida.get('gira_180'):
                    print('  >> ESTA É A CORRIDA DE CONTROLE: mesmo ponto 0, robô')
                    print('     GIRADO 180°. Ela é o que separa "o robô puxa pra')
                    print('     um lado" de "o chão cai pra um lado" — média de')
                    print('     repetições não separa isso.')
                elif corrida['de'] > 1:
                    print('  >> MESMO ponto 0 e MESMO rumo da anterior.')
                if not cfg.sem_perguntas:
                    espera('  >> posicione o robô e tecle ENTER (ctrl-c aborta): ')

                cmd = [sys.executable, ENSAIO, '--ensaio', passo['tipo'],
                       '--csv', alvo] + corrida['args']
                if cfg.sim:
                    cmd.append('--sim')
                if roda(cmd, log) != 0:
                    print('  [aviso] o ensaio saiu com erro. O CSV pode estar '
                          'parcial.')
                    if not cfg.sem_perguntas:
                        espera('  >> ENTER para seguir mesmo assim, ctrl-c para '
                               'parar: ')

                print('\n  --- leitura ---')
                roda([sys.executable, MEDIR, passo['tipo'], alvo], log)

                # Fechou um grupo de repetições: mostra as N juntas, com média e
                # faixa. É o espalhamento que diz se o número serve — e aqui
                # ainda dá para repetir, com o robô ligado.
                if corrida['de'] > 1 and corrida['k'] == corrida['de']:
                    irmas = [os.path.join(saida, c['csv']) for c in corridas
                             if c['base'] == corrida['base']]
                    print(f'\n  --- as {corrida["de"]} juntas ---')
                    roda([sys.executable, MEDIR, '--resumo', passo['tipo']]
                         + irmas, log)

    print('\n' + '=' * 72)
    print('  SESSÃO ENCERRADA')
    print('=' * 72)
    print(f'  tudo em: {saida}')
    print('  Para mandar os dados: commit da pasta e push, ou')
    print(f'    rsync -av {saida} <destino>/')
    print('  As leituras da tela ficaram salvas em leituras.txt — mande junto,')
    print('  é o registro do que a máquina respondeu na hora.')


if __name__ == '__main__':
    main()
