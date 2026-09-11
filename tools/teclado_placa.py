#!/usr/bin/env python3
"""Dirigir o robô 3 pelo teclado, direto na placa de hover (sem ROS).

Velocidade SÓ enquanto a tecla está apertada. Soltou, zero. Nada apertado,
zero — sempre.

Lê o teclado direto do kernel (/dev/input/event*): cada tecla chega como
APERTOU (1) / REPETIU (2) / SOLTOU (0), sem depender do repeat do terminal
(que não entrega "soltou") nem de janela (o Tk no Wayland perdia eventos).
Por isso precisa de sudo. Atenção: lê o teclado INTEIRO — digitar em outra
janela também dirige o robô.

Fala o protocolo 0xABCD a 115200 pela MEGA com a `firmware/hover_ponte`
(ou por um USB-TTL: --porta /dev/ttyUSB0).

Uso (no notebook, com a MEGA em /dev/ttyACM0):
    sudo python3 tools/teclado_placa.py
    sudo python3 tools/teclado_placa.py --vel 200 --giro 250

Teclas (segurando):
    w / ↑     frente            s / ↓     ré
    a / ←     pivô esquerda     d / →     pivô direita
    + / -     velocidade de frente/ré ±50
    ] / [     velocidade do pivô ±50
    i / o     inverte frente / inverte pivô (se sair trocado)
    q / Esc   sai (manda zero antes de fechar)

Por que assim (bancada 01-09 e 10-09):
  - manda comando a 50 Hz O TEMPO TODO, zero quando nada está apertado: a
    placa trava depois de um silêncio, e zeros mandados depois NÃO destravam;
  - rampa curta na subida: o degrau seco de 0 a 300 desligou a placa;
  - se o script morrer, a MEGA cala e a placa zera pelo timeout dela.

Arme: com o script rodando (já mandando zero), religue a placa ou gire as duas
rodas com a mão até o beep mudar. Só então comande.
"""

import argparse
import csv
import glob
import os
import pwd
import select
import struct
import sys
import termios
import time

import serial

START = 0xABCD
PERIODO = 0.02        # 50 Hz
RAMPA_SOBE = 50       # por ciclo: com a compensação, 0 -> 100 -> 250 em ~80 ms
RAMPA_DESCE = 50      # por ciclo: 250 -> 0 em 0,1 s
TETO = 600
# Compensação de zona morta COPIADA do hoverboard_driver (write(), deadband_speed
# do robo2.urdf.xacro): se a roda mais rápida pediria entre 1 e 100, escala as
# duas juntas até ela chegar a 100. A placa não mexe a roda abaixo de ~60-80, e
# sem isto a rampa passava ~0,3 s num comando que ela ignora. No robô 2 a
# compensação levou o arranque de 0,9 s para 0,35 s (MODELO_ROBO2.md §1).
DEADBAND = 100.0

# struct input_event (64 bits): timeval (2 x long), type u16, code u16, value s32
EV_FMT = 'llHHi'
EV_TAM = struct.calcsize(EV_FMT)
EV_KEY = 1

# código do kernel (linux/input-event-codes.h) -> nome usado aqui
TECLA = {17: 'w', 103: 'w', 31: 's', 108: 's', 30: 'a', 105: 'a', 32: 'd', 106: 'd',
         13: '+', 78: '+', 12: '-', 74: '-', 27: ']', 26: '[', 23: 'i', 24: 'o',
         16: 'q', 1: 'q'}
MOVE = {'w', 's', 'a', 'd'}
FB_TAM = 18   # start, cmd1, cmd2, speedR, speedL, bat, temp, cmdLed, checksum


def frame(steer, speed):
    return struct.pack('<HhhH', START, steer, speed,
                       (START ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF)


def extrai_feedback(buf):
    """Tira do buffer os frames de feedback válidos da placa -> (frames, resto)."""
    frames = []
    i = buf.find(b'\xcd\xab')
    while i >= 0 and len(buf) - i >= FB_TAM:
        c = struct.unpack('<HhhhhhhHH', buf[i:i + FB_TAM])
        x = 0
        for v in c[:-1]:
            x ^= v & 0xFFFF
        if x == c[-1]:
            frames.append({'cmd1': c[1], 'cmd2': c[2], 'spdR': c[3], 'spdL': c[4],
                           'bat': c[5] / 100.0, 'temp': c[6] / 10.0})
            i = buf.find(b'\xcd\xab', i + FB_TAM)
        else:
            i = buf.find(b'\xcd\xab', i + 1)
    if i < 0:
        return frames, buf[-1:]
    return frames, buf[i:]


def alvo(teclas, vel, giro, sinal_frente, sinal_giro):
    """(speed, steer) para as teclas seguradas. Pivô (a/d) tem prioridade.

    A placa distribui speedR = speed - steer, speedL = speed + steer, então
    speed=0 com steer≠0 é pivô puro."""
    a, d = 'a' in teclas, 'd' in teclas
    if a != d:
        return 0, (-giro if a else giro) * sinal_giro
    w, s = 'w' in teclas, 's' in teclas
    if w != s:
        return (vel if w else -vel) * sinal_frente, 0
    return 0, 0


def compensa(speed, steer):
    """A mesma conta do hoverboard_driver: rodas = speed ± steer/2 (a mesma
    convenção que o driver usa para montar o frame); se a maior ficar entre 1 e
    DEADBAND, escala as duas até DEADBAND, preservando a proporção."""
    esq, dir_ = speed + steer / 2.0, speed - steer / 2.0
    mx = max(abs(esq), abs(dir_))
    if 1.0 < mx < DEADBAND:
        k = DEADBAND / mx
        esq, dir_ = esq * k, dir_ * k
    sp = (esq + dir_) / 2.0
    return int(sp), int((esq - sp) * 2.0)


def aproxima(atual, desejado):
    if desejado == atual:
        return atual
    acelerando = abs(desejado) > abs(atual) and (atual == 0 or (desejado > 0) == (atual > 0))
    passo = RAMPA_SOBE if acelerando else RAMPA_DESCE
    if desejado > atual:
        return min(desejado, atual + passo)
    return max(desejado, atual - passo)


def teclados():
    """Caminhos /dev/input/eventN de tudo que o kernel marca como teclado."""
    achados, bloco = [], ''
    with open('/proc/bus/input/devices') as f:
        for linha in f.read().split('\n\n'):
            bloco = linha
            handlers = next((l for l in bloco.splitlines() if l.startswith('H: Handlers=')), '')
            ev = next((l for l in bloco.splitlines() if l.startswith('B: EV=')), '')
            if 'kbd' in handlers and ev.endswith('120013'):
                for h in handlers.split('=')[1].split():
                    if h.startswith('event'):
                        achados.append('/dev/input/' + h)
    return achados


def casa_do_usuario():
    dono = os.environ.get('SUDO_USER')
    return pwd.getpwnam(dono) if dono else pwd.getpwuid(os.getuid())


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--porta', default='/dev/ttyACM0')
    ap.add_argument('--vel', type=int, default=250, help='speed de frente/ré (padrão 250)')
    ap.add_argument('--giro', type=int, default=250, help='steer do pivô (padrão 250)')
    ap.add_argument('--teclado', action='append',
                    help='/dev/input/eventN (padrão: todos os teclados)')
    a = ap.parse_args()

    caminhos = a.teclado or teclados()
    try:
        fds = [os.open(p, os.O_RDONLY | os.O_NONBLOCK) for p in caminhos]
    except PermissionError:
        sys.exit('Sem acesso ao teclado do kernel: rode com sudo.')
    if not fds:
        sys.exit('Nenhum teclado achado em /proc/bus/input/devices.')

    usuario = casa_do_usuario()
    pasta = os.path.join(usuario.pw_dir, 'bancada_robo3')
    os.makedirs(pasta, exist_ok=True)
    caminho_csv = os.path.join(pasta, time.strftime('teclado_%Y%m%d_%H%M%S.csv'))

    print(f'Teclados: {", ".join(caminhos)}')
    print(f'Abrindo {a.porta} (a MEGA reinicia ao abrir, 2,5 s)...')
    s = serial.Serial(a.porta, 115200, timeout=0)
    time.sleep(2.5)
    s.reset_input_buffer()

    vel, giro = a.vel, a.giro
    # -1: no robô 3 com speed>0 ele anda para trás (dono, 10-09).
    sinal_frente, sinal_giro = -1, 1
    seguradas = set()
    v = g = 0
    rx, fb = b'', None

    # o terminal não ecoa as letras (senão elas viram comando no shell ao sair)
    tty_fd = sys.stdin.fileno() if sys.stdin.isatty() else None
    antigo = termios.tcgetattr(tty_fd) if tty_fd is not None else None
    if antigo:
        novo = termios.tcgetattr(tty_fd)
        novo[3] &= ~(termios.ECHO | termios.ICANON)
        termios.tcsetattr(tty_fd, termios.TCSANOW, novo)

    print(__doc__[__doc__.index('Teclas'):__doc__.index('Por que')].rstrip())
    print(f'\nMANDANDO ZERO. Arme agora (religue a placa ou gire as rodas até o beep mudar).')
    print(f'log: {caminho_csv}\n')

    fcsv = open(caminho_csv, 'w', newline='')
    log = csv.writer(fcsv)
    log.writerow(['t', 'teclas', 'alvo_speed', 'alvo_steer', 'speed', 'steer',
                  'fb_n', 'bat', 'cmd1', 'cmd2', 'spdR', 'spdL', 'temp'])
    t0 = time.time()
    prox = t0
    sair = False
    try:
        while not sair:
            espera = max(0.0, prox - time.time())
            prontos = select.select(fds, [], [], espera)[0]
            for fd in prontos:
                try:
                    dados = os.read(fd, EV_TAM * 64)
                except BlockingIOError:
                    continue
                for off in range(0, len(dados) - EV_TAM + 1, EV_TAM):
                    _, _, tipo, cod, valor = struct.unpack(EV_FMT, dados[off:off + EV_TAM])
                    if tipo != EV_KEY or cod not in TECLA:
                        continue
                    k = TECLA[cod]
                    if k in MOVE:
                        if valor == 0:
                            seguradas.discard(k)
                        else:
                            seguradas.add(k)
                    elif valor == 1:          # ajustes só no aperto, não no repeat
                        if k == 'q':
                            sair = True
                        elif k == '+':
                            vel = min(TETO, vel + 50)
                        elif k == '-':
                            vel = max(50, vel - 50)
                        elif k == ']':
                            giro = min(TETO, giro + 50)
                        elif k == '[':
                            giro = max(50, giro - 50)
                        elif k == 'i':
                            sinal_frente = -sinal_frente
                        elif k == 'o':
                            sinal_giro = -sinal_giro

            agora = time.time()
            if agora < prox:
                continue
            av, ag = alvo(seguradas, vel, giro, sinal_frente, sinal_giro)
            v, g = aproxima(v, av), aproxima(g, ag)
            vs, gs = compensa(v, g)           # o que vai de fato para a placa
            s.write(frame(gs, vs))
            rx = (rx + s.read(512))[-4096:]
            fbs, rx = extrai_feedback(rx)
            if fbs:
                fb = fbs[-1]
            f = fbs[-1] if fbs else None
            log.writerow([f'{agora - t0:.3f}', ''.join(sorted(seguradas)), av, ag, vs, gs, len(fbs)]
                         + ([f['bat'], f['cmd1'], f['cmd2'], f['spdR'], f['spdL'], f['temp']]
                            if f else [''] * 6))
            prox += PERIODO
            if prox < agora:
                prox = agora + PERIODO
            placa = f'{fb["bat"]:.1f}V' if fb else 'sem resposta'
            sys.stdout.write(f'\r segurando={"".join(sorted(seguradas)) or "-":3s} '
                             f'speed={vs:5d} steer={gs:5d}  vel={vel} pivô={giro}  placa: {placa}   ')
            sys.stdout.flush()
    finally:
        for _ in range(10):
            s.write(frame(0, 0))
            time.sleep(PERIODO)
        s.close()
        fcsv.close()
        os.chown(caminho_csv, usuario.pw_uid, usuario.pw_gid)
        for fd in fds:
            os.close(fd)
        if antigo:
            termios.tcflush(tty_fd, termios.TCIFLUSH)   # joga fora as letras digitadas
            termios.tcsetattr(tty_fd, termios.TCSANOW, antigo)
        print(f'\nParado (zero enviado). Log em {caminho_csv}')


if __name__ == '__main__':
    main()
