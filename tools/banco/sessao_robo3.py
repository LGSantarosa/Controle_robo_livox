#!/usr/bin/env python3
"""Conduz a caracterização completa da planta nua do robô 3.

Um passo por vez, sempre reposicionando o robô. Pode interromper e retomar:

    python3 tools/banco/sessao_robo3.py --listar
    python3 tools/banco/sessao_robo3.py --so 0       # somente LIO, não move
    python3 tools/banco/sessao_robo3.py --so 1       # rodas SUSPENSAS
    python3 tools/banco/sessao_robo3.py --de 2       # ensaios no chão

Uma falha encerra a sessão. O objetivo é voltar do laboratório com menos
arquivos bons, nunca com uma pasta cheia de corridas posteriores a uma falha.
"""

import argparse
import datetime
import os
import platform
import subprocess
import sys


AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))
PERFIL = os.path.join(AQUI, 'perfil_robo3.py')
MEDIR = os.path.join(AQUI, 'mede_robo3.py')
BITOLA = 0.3225
RAIO = 0.0825


def corrida(nome, perfil='pulso', v=0.0, wz=0.0, liga=2.0, pausa=1.0,
            repete=1, espaco=2.5, trajeto=5.0, gira_180=False,
            permitir_sem_encoders=False, v_max=0.8, wz_max=3.0):
    return dict(nome=nome, perfil=perfil, v=v, wz=wz, liga=liga, pausa=pausa,
                repete=repete, espaco=espaco, trajeto=trajeto,
                gira_180=gira_180,
                permitir_sem_encoders=permitir_sem_encoders,
                v_max=v_max, wz_max=wz_max)


PASSOS = [
    dict(
        n=0, titulo='LIO parado — deriva, ruído e taxa antes de mover qualquer roda',
        local='Rodas sem energia. Robô imóvel por 30 s; ninguém toca na bancada.',
        nota='Pode rodar só com notebook + Livox. Se este passo reprovar, nenhum '
             'movimento posterior vira medida.',
        corridas=[corrida('0-lio-parado', perfil='parado', repete=1,
                          permitir_sem_encoders=True)]),
    dict(
        n=1, titulo='Placa, sentidos e encoders — QUATRO RODAS FORA DO CHÃO',
        local='Robô apoiado pelo chassi; pneus e bobas sem tocar em nada.',
        nota='Quatro pulsos de 0,35 s: frente, ré, giro esquerdo e giro direito. '
             'Observe o sentido físico. Este passo não autoriza pôr o robô no '
             'chão se uma roda ou encoder estiver invertido.',
        corridas=[corrida('1-suspenso', perfil='suspenso', espaco=0.30,
                          trajeto=0.50, v_max=0.40, wz_max=0.70)]),
    dict(
        n=2, titulo='Pulsos baixos — a placa modula ou transforma tudo em patamar?',
        local='Chão plano, 2 m livres à frente e atrás.',
        nota='Uma passagem curta por três módulos lineares e dois módulos de '
             'giro. É a etapa de risco baixo que decide se podemos seguir.',
        corridas=[
            corrida('2-v010-frente', v=+0.10, liga=0.60, espaco=1.0, trajeto=1.2),
            corrida('2-v010-re', v=-0.10, liga=0.60, espaco=1.0, trajeto=1.2),
            corrida('2-v025-frente', v=+0.25, liga=0.60, espaco=1.0, trajeto=1.2),
            corrida('2-v025-re', v=-0.25, liga=0.60, espaco=1.0, trajeto=1.2),
            corrida('2-v040-frente', v=+0.40, liga=0.60, espaco=1.0, trajeto=1.2),
            corrida('2-v040-re', v=-0.40, liga=0.60, espaco=1.0, trajeto=1.2),
            corrida('2-wz015-esq', wz=+0.15, liga=0.35, espaco=0.8,
                    trajeto=1.0, v_max=1.0, wz_max=3.5),
            corrida('2-wz015-dir', wz=-0.15, liga=0.35, espaco=0.8,
                    trajeto=1.0, v_max=1.0, wz_max=3.5),
            corrida('2-wz030-esq', wz=+0.30, liga=0.35, espaco=0.8,
                    trajeto=1.0, v_max=1.0, wz_max=3.5),
            corrida('2-wz030-dir', wz=-0.30, liga=0.35, espaco=0.8,
                    trajeto=1.0, v_max=1.0, wz_max=3.5),
        ]),
    dict(
        n=3, titulo='Reta curta — aceleração, frenagem e assimetria ida × ré',
        local='Mesmo ponto e rumo, 3 m livres à frente e atrás.',
        nota='Três repetições independentes por sentido. Filme as bobas em uma '
             'ida e uma ré.',
        corridas=[
            corrida('3-reta-curta-frente', v=+0.25, liga=3.0, repete=3,
                    espaco=2.0, trajeto=2.5),
            corrida('3-reta-curta-re', v=-0.25, liga=3.0, repete=3,
                    espaco=2.0, trajeto=2.5),
        ]),
    dict(
        n=4, titulo='Reta longa — curvatura natural e controle de caimento do piso',
        local='Corredor com pelo menos 4 m úteis. Marque ponto e rumo no chão.',
        nota='O par girado 180° separa defeito preso ao robô de caimento preso '
             'ao laboratório.',
        corridas=[
            corrida('4-reta-longa-frente', v=+0.25, liga=8.0, repete=3,
                    espaco=3.5, trajeto=4.0),
            corrida('4-reta-longa-re', v=-0.25, liga=8.0, repete=3,
                    espaco=3.5, trajeto=4.0),
            corrida('4-reta-longa-frente-180', v=+0.25, liga=8.0,
                    espaco=3.5, trajeto=4.0, gira_180=True),
            corrida('4-reta-longa-re-180', v=-0.25, liga=8.0,
                    espaco=3.5, trajeto=4.0, gira_180=True),
        ]),
    dict(
        n=5, titulo='Pivô — quantum, latência e retenção nos dois sentidos',
        local='Raio de 1,5 m livre. Robô sempre no mesmo ponto de partida.',
        nota='O comando é o mesmo; varia apenas o tempo ligado. Isso revela '
             'placa que escolhe módulo pelo tempo, como a do Robô 2.',
        corridas=[
            corrida(f'5-pivo-{lado}-{ms:03d}ms', wz=sinal * 0.30,
                    liga=ms / 1000.0, repete=2, espaco=1.0, trajeto=1.5,
                    v_max=1.0, wz_max=3.5)
            for lado, sinal in [('esq', 1), ('dir', -1)]
            for ms in (150, 300, 600)
        ]),
    dict(
        n=6, titulo='Curva curta — quatro combinações de marcha e lado',
        local='Quadrado livre de pelo menos 4 × 4 m.',
        nota='Frente/reversa × esquerda/direita, três vezes cada. É o núcleo '
             'da calibração de atrito e assimetria do Gazebo.',
        corridas=[
            corrida('6-curta-frente-esq', v=+0.25, wz=+0.30, liga=3.0,
                    repete=3, espaco=2.0, trajeto=2.8),
            corrida('6-curta-frente-dir', v=+0.25, wz=-0.30, liga=3.0,
                    repete=3, espaco=2.0, trajeto=2.8),
            corrida('6-curta-re-esq', v=-0.25, wz=+0.30, liga=3.0,
                    repete=3, espaco=2.0, trajeto=2.8),
            corrida('6-curta-re-dir', v=-0.25, wz=-0.30, liga=3.0,
                    repete=3, espaco=2.0, trajeto=2.8),
        ]),
    dict(
        n=7, titulo='Curva longa — regime sustentado, derrapagem e aquecimento',
        local='Quadrado livre de pelo menos 5 × 5 m.',
        nota='Arcos longos nos dois lados e nas duas marchas. Não pule a ré: '
             'as duas bobas são empurradas nela e esse é o risco novo.',
        corridas=[
            corrida('7-longa-frente-esq', v=+0.25, wz=+0.20, liga=8.0,
                    repete=3, espaco=2.8, trajeto=4.5),
            corrida('7-longa-frente-dir', v=+0.25, wz=-0.20, liga=8.0,
                    repete=3, espaco=2.8, trajeto=4.5),
            corrida('7-longa-re-esq', v=-0.20, wz=+0.20, liga=6.0,
                    repete=2, espaco=2.5, trajeto=3.5),
            corrida('7-longa-re-dir', v=-0.20, wz=-0.20, liga=6.0,
                    repete=2, espaco=2.5, trajeto=3.5),
        ]),
    dict(
        n=8, titulo='Inversão de marcha — chicote das duas bobas',
        local='3 m livres à frente e atrás. Câmera apontada para as duas bobas.',
        nota='Frente→ré e ré→frente, com pausa curta e longa. O vídeo deve '
             'mostrar quando cada pivô começa e termina os 180°.',
        corridas=[
            corrida('8-frente-re-pausa025', perfil='inversao', v=+0.25,
                    liga=2.0, pausa=0.25, repete=3, espaco=2.2, trajeto=3.5),
            corrida('8-frente-re-pausa100', perfil='inversao', v=+0.25,
                    liga=2.0, pausa=1.00, repete=3, espaco=2.2, trajeto=3.5),
            corrida('8-re-frente-pausa025', perfil='inversao', v=-0.25,
                    liga=2.0, pausa=0.25, repete=3, espaco=2.2, trajeto=3.5),
            corrida('8-re-frente-pausa100', perfil='inversao', v=-0.25,
                    liga=2.0, pausa=1.00, repete=3, espaco=2.2, trajeto=3.5),
        ]),
]


def expande(corridas, forcar=None):
    saida = []
    for c in corridas:
        n = forcar or c['repete']
        for i in range(n):
            sufixo = '' if n == 1 else f'-{chr(ord("a") + i)}'
            saida.append((dict(c), f'{c["nome"]}{sufixo}.csv', i + 1, n))
    return saida


def nome_livre(pasta, nome):
    caminho = os.path.join(pasta, nome)
    if not os.path.exists(caminho):
        return caminho
    base, ext = os.path.splitext(caminho)
    i = 2
    while os.path.exists(f'{base}-r{i}{ext}'):
        i += 1
    return f'{base}-r{i}{ext}'


def executa(cmd, log):
    log.write('\n$ ' + ' '.join(cmd) + '\n')
    log.flush()
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
    for linha in p.stdout:
        print(linha, end='')
        log.write(linha)
    p.wait()
    log.flush()
    return p.returncode


def pergunta(texto):
    try:
        return input(texto)
    except (EOFError, KeyboardInterrupt):
        print('\nSessão interrompida; o perfil ativo já recebeu zero.')
        raise SystemExit(130)


def prepara_ambiente(pasta, sem_perguntas):
    os.makedirs(pasta, exist_ok=True)
    caminho = os.path.join(pasta, 'ambiente.txt')
    if os.path.exists(caminho):
        return caminho
    respostas = {}
    if not sem_perguntas:
        print('\nRegistre a configuração que realmente está sendo testada:')
        respostas['piso'] = pergunta('  piso: ').strip()
        respostas['bateria'] = pergunta('  bateria/tensão inicial: ').strip()
        respostas['placa'] = pergunta('  placa (modelo ou "não identificada"): ').strip()
        respostas['carga'] = pergunta('  carga montada (notebook, Livox, baterias): ').strip()
        respostas['operador'] = pergunta('  quem está no corte de energia: ').strip()
        respostas['obs'] = pergunta('  observação livre: ').strip()
    commit = subprocess.run(['git', '-C', RAIZ, 'rev-parse', '--short', 'HEAD'],
                            capture_output=True, text=True).stdout.strip()
    serial = subprocess.run(
        ['bash', '-lc', 'ls -l /dev/serial/by-id 2>&1'],
        capture_output=True, text=True).stdout.strip()
    usb = subprocess.run(['lsusb'], capture_output=True, text=True).stdout.strip()
    with open(caminho, 'w') as arq:
        arq.write('sessão de validação da planta nua — ROBÔ 3\n')
        arq.write(f'data: {datetime.datetime.now().isoformat(timespec="seconds")}\n')
        arq.write(f'commit: {commit}\n')
        arq.write(f'sistema: {platform.platform()}\n')
        for k, v in respostas.items():
            arq.write(f'{k}: {v}\n')
        arq.write('\n[/dev/serial/by-id]\n' + serial + '\n')
        arq.write('\n[lsusb]\n' + usb + '\n')
    return caminho


def checa_base():
    # Reuso deliberado da conferência que já pegou build velho no robô 2.
    import sessao
    ok, linhas, calib = sessao.confere(
        sim=False, mexer=False,
        esperado={'wheel_separation': BITOLA, 'wheel_radius': RAIO})
    for linha in linhas:
        print(linha)
    if not ok:
        return False
    if abs((calib.get('wheel_separation') or 0.0) - BITOLA) > 1e-6:
        print('[FALHA] controlador não carregou a bitola do robô 3.')
        return False
    if abs((calib.get('wheel_radius') or 0.0) - RAIO) > 1e-6:
        print('[FALHA] controlador não carregou o raio do robô 3.')
        return False
    if calib.get('open_loop') is not False:
        print('[FALHA] open_loop precisa ser false: queremos encoder, não eco do comando.')
        return False
    return True


def comando(c, csv_path):
    cmd = [
        sys.executable, PERFIL,
        '--perfil', c['perfil'], '--csv', csv_path,
        '--v', str(c['v']), '--wz', str(c['wz']),
        '--liga', str(c['liga']), '--pausa', str(c['pausa']),
        '--bitola', str(BITOLA), '--raio', str(RAIO),
        '--espaco', str(c['espaco']), '--trajeto-max', str(c['trajeto']),
        '--v-real-max', str(c['v_max']), '--wz-real-max', str(c['wz_max']),
    ]
    if c['perfil'] == 'parado':
        cmd += ['--dur', '30']
    if c['permitir_sem_encoders']:
        cmd.append('--permitir-sem-encoders')
    return cmd


def listar():
    for p in PASSOS:
        total = sum(c['repete'] for c in p['corridas'])
        print(f'{p["n"]}: {p["titulo"]} ({total} corrida(s))')


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--listar', action='store_true')
    selecao = ap.add_mutually_exclusive_group()
    selecao.add_argument('--de', type=int,
                         help='executa deste passo em diante (use só após os anteriores)')
    selecao.add_argument('--so', type=int)
    ap.add_argument('--repete', type=int,
                    help='força N repetições por condição (1 = sessão rápida)')
    ap.add_argument('--saida')
    ap.add_argument('--sem-perguntas', action='store_true',
                    help='somente para ensaio controlado; não usar no chão')
    cfg = ap.parse_args(argv)
    if cfg.listar:
        listar()
        return 0
    if cfg.repete is not None and not 1 <= cfg.repete <= 26:
        ap.error('--repete precisa estar entre 1 e 26')
    escolhido = cfg.so if cfg.so is not None else cfg.de
    if escolhido is not None and not 0 <= escolhido < len(PASSOS):
        ap.error('passo precisa estar entre 0 e 8')

    dia = datetime.date.today().isoformat()
    pasta = cfg.saida or os.path.join(
        RAIZ, 'docs', 'dados', f'{dia}-bancada-robo3')
    ambiente = prepara_ambiente(pasta, cfg.sem_perguntas)
    print(f'Dados: {pasta}\nAmbiente: {ambiente}')

    # Sem seleção explícita, faz somente a prova imóvel. Nunca deixamos um
    # simples ENTER atravessar, na mesma execução, de sensor para rodas ativas.
    inicio = 0 if cfg.de is None else cfg.de
    escolhidos = [p for p in PASSOS
                  if (p['n'] == cfg.so if cfg.so is not None
                      else p['n'] >= inicio)]
    if cfg.so is None and cfg.de is None:
        escolhidos = [PASSOS[0]]
    if not escolhidos:
        ap.error('nenhum passo selecionado')

    if any(p['n'] >= 1 for p in escolhidos):
        print('\n--- conferência viva da base do robô 3 ---')
        if not checa_base():
            print('\nBase reprovada. Nenhuma roda será comandada.')
            return 2

    log_path = os.path.join(pasta, 'leituras.txt')
    obs_path = os.path.join(pasta, 'observacoes.txt')
    with open(log_path, 'a') as log, open(obs_path, 'a') as obs:
        for p in escolhidos:
            print('\n' + '=' * 78)
            print(f'PASSO {p["n"]}/8 — {p["titulo"]}')
            print('Local:', p['local'])
            print(p['nota'])
            if not cfg.sem_perguntas:
                pergunta('>> Confirme a montagem e tecle ENTER; Ctrl-C aborta: ')
            for c, nome, k, n in expande(p['corridas'], cfg.repete):
                alvo = nome_livre(pasta, nome)
                print(f'\nCorrida {os.path.basename(alvo)} ({k}/{n})')
                if c['gira_180']:
                    print('>> GIRE O ROBÔ 180° sobre o mesmo piso e preserve o eixo marcado.')
                if p['n'] == 1:
                    print('>> RODAS SUSPENSAS. Não continue se qualquer apoio tocar o chão.')
                if p['n'] == 8:
                    print('>> FILME AS DUAS BOBAS, incluindo 1 s antes e depois da inversão.')
                if not cfg.sem_perguntas:
                    pergunta('>> Posicione, arme o corte de energia e tecle ENTER: ')
                rc = executa(comando(c, alvo), log)
                executa([sys.executable, MEDIR, alvo], log)
                if not cfg.sem_perguntas:
                    nota = pergunta('>> O que você viu? (ENTER se nada anormal): ').strip()
                    obs.write(f'{datetime.datetime.now().isoformat(timespec="seconds")} '
                              f'{os.path.basename(alvo)}: {nota or "sem anormalidade anotada"}\n')
                    obs.flush()
                if rc != 0:
                    print('\nPERFIL ABORTADO. A sessão para aqui; não pule a causa.')
                    print(f'Retome depois com --de {p["n"]}.')
                    return rc

    with open(log_path, 'a') as log:
        executa([sys.executable, MEDIR, '--pasta', pasta], log)
    print('\nSessão encerrada. Mande a pasta inteira, incluindo JSON e observações:')
    print(pasta)
    return 0


if __name__ == '__main__':
    sys.exit(main())
