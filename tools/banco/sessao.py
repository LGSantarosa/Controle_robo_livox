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

PASSOS = [
    dict(
        n=1, tipo='zona_morta_linear',
        titulo='Zona morta linear — o robô sai do lugar com quanto?',
        espaco='~3 m em linha reta à frente',
        pose='Robô parado, apontando para o lado livre mais comprido.',
        corridas=[dict(csv='1-zona_morta_linear.csv',
                       args=['--dur', '20', '--rampa-ate', '0.35',
                             '--espaco', '3.0'])],
        nota='Se ele NÃO sair do lugar em nenhum ponto da rampa, isso não é\n'
             'falha do ensaio — é o resultado. Repetir com --rampa-ate 0.6.',
    ),
    dict(
        n=2, tipo='zona_morta_giro',
        titulo='Zona morta de giro — e girando parado? (o item nº 1 do projeto)',
        espaco='raio de 1 m livre em volta',
        pose='Robô parado, no meio do espaço livre. Não precisa de rumo nenhum.',
        corridas=[dict(csv='2-zona_morta_giro.csv',
                       args=['--dur', '20', '--rampa-ate', '1.5',
                             '--espaco', '1.5'])],
        nota='É o número que decide se este robô PIVOTA. Se ele ficar parado\n'
             'com o comando subindo, é a medida do BO-3 acontecendo — deixe\n'
             'rodar os 20 s inteiros mesmo assim.',
    ),
    dict(
        n=3, tipo='degrau_giro',
        titulo='Degrau de giro — quanto ele demora pra PARAR de girar (a_dec)',
        espaco='~4 m; ele termina apontando para outro lado',
        pose='Robô no começo do espaço, apontando para o comprido.',
        corridas=[
            dict(csv='3-degrau_wz03.csv',
                 args=['--v', '0.3', '--wz', '0.3', '--dur', '12']),
            dict(csv='3-degrau_wz06.csv',
                 args=['--v', '0.3', '--wz', '0.6', '--dur', '12']),
            dict(csv='3-degrau_wz10.csv',
                 args=['--v', '0.3', '--wz', '1.0', '--dur', '12']),
        ],
        nota='O número mais importante do projeto: a_dec é a causa medida do S.\n'
             'Os três níveis mostram se ele é constante ou piora com giro forte.',
    ),
    dict(
        n=4, tipo='curva',
        titulo='Curva sustentada — quanto ele curva a 1x, 2x, 3x de velocidade',
        espaco='círculo de raio v/wz (a 0,6 m/s e 0,5 rad/s são 1,2 m de raio)',
        pose='Robô no meio do espaço livre, apontando para o comprido.',
        corridas=[
            dict(csv='4-curva_v02.csv',
                 args=['--v', '0.2', '--wz', '0.5', '--dur', '12']),
            dict(csv='4-curva_v04.csv',
                 args=['--v', '0.4', '--wz', '0.5', '--dur', '12']),
            dict(csv='4-curva_v06.csv',
                 args=['--v', '0.6', '--wz', '0.5', '--dur', '12']),
        ],
        nota='É aqui que a geometria aparece: motriz na frente, boba atrás.\n'
             'Se o giro realizado CAIR conforme a velocidade sobe, está medido,\n'
             'e vira restrição de projeto.',
    ),
    dict(
        n=5, tipo='aceleracao_linear',
        titulo='Aceleração linear — arranca e freia quanto?',
        espaco='~4 m em reta',
        pose='Robô no começo do espaço, apontando para o comprido.',
        corridas=[dict(csv='5-aceleracao.csv',
                       args=['--v', '0.6', '--dur', '10'])],
        nota='Confere se os tetos do hoverboard_controllers.yaml (0,7 m/s e\n'
             '0,8 m/s²) descrevem ESTA máquina ou foram herdados sem medir.',
    ),
    dict(
        n=6, tipo='reta',
        titulo='Reta com cutucão — o rumo volta ou foge? E de ré? (BO-4)',
        espaco='~4 m em reta, nos DOIS sentidos',
        pose='Robô no MEIO do espaço: ele vai andar para frente numa corrida e\n'
             'para trás na outra.',
        corridas=[
            dict(csv='6-reta_frente.csv',
                 args=['--v', '0.25', '--wz', '0.5', '--dur', '16']),
            dict(csv='6-reta_re.csv',
                 args=['--v', '-0.25', '--wz', '0.5', '--dur', '16']),
            dict(csv='6-reta_crua.csv',
                 args=['--v', '0.25', '--wz', '0', '--dur', '16']),
        ],
        nota='FILMAR A TRASEIRA nas duas primeiras corridas. O que se procura é\n'
             'a boba dando meia-volta na corrida de ré, e quanto o robô se\n'
             'desvia enquanto ela decide. É o ensaio que fecha o BO-4 e decide\n'
             'se a manobra de ré da decisão 007 é segura.\n'
             'A 3ª corrida é a reta CRUA, sem pulso, e a leitura dela vai dizer\n'
             '"nada a comparar" — está certo: o medir.py resume a resposta ao\n'
             'cutucão, e aqui não há cutucão. O CSV é que interessa, e ele só\n'
             'vale no robô: no simulador a máquina é simétrica e o desvio sai\n'
             'zero exato. É a assimetria natural desta máquina, medida.',
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

            for i, corrida in enumerate(passo['corridas'], 1):
                alvo = os.path.join(saida, corrida['csv'])
                print(f'\n  corrida {i}/{len(passo["corridas"])}: {corrida["csv"]}')
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
