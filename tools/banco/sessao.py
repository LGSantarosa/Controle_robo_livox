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
        n=1, tipo='zona_morta_linear',
        titulo='Zona morta linear — o robô sai do lugar com quanto?',
        espaco='~3 m em linha reta à frente (ele vai e volta, quase não sai do lugar)',
        pose='Robô parado, apontando para o lado livre mais comprido.',
        corridas=[dict(csv='1-zona_morta_linear.csv',
                       args=['--dur', '180', '--rampa-ate', '0.35',
                             '--espaco', '3.0', '--dentes', '4'])],
        nota='DENTE DE SERRA: sobe até ele sair do lugar, desce até ele parar,\n'
             'inverte o sentido e repete 4x. Uma corrida dá 4 medidas de saída\n'
             'e 4 de queda, nos dois sentidos — a repetição está DENTRO dela.\n'
             'Os 180 s são teto de tempo, não duração: ele fecha os 4 dentes\n'
             'muito antes. Se NÃO sair do lugar, isso não é falha do ensaio,\n'
             'é o resultado; refazer com --rampa-ate 0.6.',
    ),
    dict(
        n=2, tipo='zona_morta_giro',
        titulo='Zona morta de giro — e girando parado? (o item nº 1 do projeto)',
        espaco='raio de 1 m livre em volta',
        pose='Robô parado, no meio do espaço livre. Não precisa de rumo nenhum.',
        corridas=[dict(csv='2-zona_morta_giro.csv',
                       args=['--dur', '220', '--rampa-ate', '1.5',
                             '--espaco', '1.5', '--dentes', '4'])],
        nota='É o número que decide se este robô PIVOTA, e a folga é de 4%:\n'
             'com zona morta 0,10 pivotar exige 0,96 rad/s contra teto de 1,0;\n'
             'com 0,15 exige 1,48 e é impossível. Por isso a FAIXA importa\n'
             'tanto quanto a média, e por isso são 4 dentes.\n'
             'Ele vai ficar parado com o comando subindo. Não é travamento —\n'
             'é a zona morta acontecendo, e é o que viemos medir. Deixar rodar.',
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


# --------------------------------------------------------------- conferência

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

    if ok and mexer:
        linhas.append('')
        linhas.append('  --- cutucão de sanidade (o robô VAI se mexer) ---')
        linhas += _cutucao(no, pub, ultimo, sim)

    no.destroy_node()
    rclpy.shutdown()
    return ok, linhas


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
        t0 = time.monotonic()
        while time.monotonic() - t0 < seg:
            m = TwistStamped()
            m.header.stamp = no.get_clock().now().to_msg()
            m.twist.linear.x = float(v)
            m.twist.angular.z = float(wz)
            pub.publish(m)
            rclpy.spin_once(no, timeout_sec=0.02)
        for _ in range(10):
            pub.publish(TwistStamped())
            rclpy.spin_once(no, timeout_sec=0.02)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            rclpy.spin_once(no, timeout_sec=0.05)

    saida = []

    x0, y0 = pos()
    a0 = yaw()
    solta(0.25, 0.0, 2.0)
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

    a0 = yaw()
    solta(0.0, 0.6, 2.0)
    giro = math.atan2(math.sin(yaw() - a0), math.cos(yaw() - a0))
    if giro > 0.15:
        saida.append(f'  [ok] comando de +0,6 rad/s girou {math.degrees(giro):+.1f}° '
                     f'— anti-horário, como manda a regra da mão direita')
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
    ok, linhas = confere(sim=cfg.sim, mexer=cfg.mexer)
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
