#!/usr/bin/env python3
"""Dirigir o robô 3 pelo teclado, direto na placa de hover (sem ROS).

Fala o protocolo 0xABCD a 115200 pela MEGA com a `firmware/hover_ponte`
(ou por um USB-TTL: --porta /dev/ttyUSB0). Feito para demonstração em
bancada/chão, não para navegação.

O que ele garante, e por quê (bancada 01-09 e 10-09):
  - manda comando a 50 Hz O TEMPO TODO, zero quando nada está apertado: a placa
    trava depois de um silêncio, e zeros mandados depois NÃO destravam;
  - SEGURAR a tecla anda, SOLTAR para: sem tecla por 0,6 s volta a zero
    (0,6 s porque o repeat do teclado demora ~0,5 s para começar);
  - rampa de velocidade: o degrau seco de 0 a 300 desligou a placa em 10-09.

Uso (no terminal do notebook, com a MEGA em /dev/ttyACM0):
    python3 tools/teclado_placa.py
    python3 tools/teclado_placa.py --vel 200 --giro 150
    python3 tools/teclado_placa.py --mega    # MEGA com firmware/mega_bridge; CSV
                                             # grava o que a MEGA pôs na placa

Teclas:
    w / s     frente / ré            (segurar)
    a / d     gira esquerda / direita (segurar; combina com w/s)
    espaço    PARA na hora
    + / -     velocidade máxima +50 / -50
    i         inverte frente/ré (se o 'w' andar para trás)
    o         inverte o giro     (se o 'a' girar para a direita)
    q         sai (manda zero antes de fechar)

Arme: com o script rodando (já mandando zero), religue a placa ou gire as duas
rodas com a mão até o beep mudar. Só então comande.
"""

import argparse
import csv
import os
import select
import struct
import sys
import termios
import time
import tty

import serial

START = 0xABCD
PERIODO = 0.02          # 50 Hz
SEM_TECLA_PARA = 0.6    # s sem tecla -> alvo zero
RAMPA = 15              # passo máximo por ciclo (750/s: 0 -> 300 em 0,4 s)
VEL_MAX_ABS = 600       # teto do '+', não passa disso


def frame(steer, speed):
    return struct.pack('<HhhH', START, steer, speed,
                       (START ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF)


# --mega: fala o protocolo do firmware/mega_bridge (0xAA 0x55, 230400) em vez
# da hover_ponte. A MEGA repete o par para a placa a 50 Hz e devolve FT_DEBUG.
# Serve para separar firmware da MEGA de cadeia ROS (robô 3, 14-09).
FT_SET_SPEED, FT_DEBUG = 0x01, 0x85


def frame_mega(steer, speed):
    payload = struct.pack('<hhhh', steer, speed, 0, 0)
    chk = FT_SET_SPEED ^ len(payload)
    for b in payload:
        chk ^= b
    return bytes([0xAA, 0x55, FT_SET_SPEED, len(payload)]) + payload + bytes([chk])


class LeDebug:
    """Acha FT_DEBUG no que a MEGA devolve; guarda o último (6 valores)."""

    def __init__(self):
        self.buf = bytearray()
        self.ultimo = ['', '', '', '', '', '']

    def alimenta(self, dados):
        self.buf += dados
        while True:
            i = self.buf.find(b'\xaa\x55')
            if i < 0:
                del self.buf[:-1]
                return
            del self.buf[:i]
            if len(self.buf) < 4 or len(self.buf) < 5 + self.buf[3]:
                return
            tipo, n = self.buf[2], self.buf[3]
            corpo = bytes(self.buf[4:4 + n])
            chk = tipo ^ n
            for b in corpo:
                chk ^= b
            if chk == self.buf[4 + n] and tipo == FT_DEBUG and n == 12:
                self.ultimo = list(struct.unpack('<hhHHHH', corpo))
                del self.buf[:5 + n]
            else:
                del self.buf[:2 if chk != self.buf[4 + n] else 5 + n]


def aproxima(atual, alvo):
    if alvo > atual:
        return min(alvo, atual + RAMPA)
    return max(alvo, atual - RAMPA)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--porta', default='/dev/ttyACM0')
    ap.add_argument('--vel', type=int, default=250, help='speed máximo (padrão 250)')
    ap.add_argument('--giro', type=int, default=150, help='steer máximo (padrão 150)')
    ap.add_argument('--mega', action='store_true',
                    help='MEGA com firmware/mega_bridge (e não hover_ponte)')
    a = ap.parse_args()

    os.makedirs(os.path.expanduser('~/bancada_robo3'), exist_ok=True)
    nome = 'teclado_mega' if a.mega else 'teclado'
    caminho_csv = os.path.expanduser(time.strftime(f'~/bancada_robo3/{nome}_%Y%m%d_%H%M%S.csv'))
    monta = frame_mega if a.mega else frame
    debug = LeDebug()

    s = serial.Serial(a.porta, 230400 if a.mega else 115200, timeout=0)
    print(f'Abrindo {a.porta}... (a MEGA reinicia ao abrir, 2,5 s)')
    time.sleep(2.5)
    s.reset_input_buffer()

    vel_max, giro_max = a.vel, a.giro
    # -1: no robô 3 com speed>0 ele anda para trás (dono, 10-09). 'i' inverte.
    sinal_frente, sinal_giro = -1, 1
    alvo_v = alvo_g = 0
    v = g = 0
    ultima_tecla = 0.0
    fd = sys.stdin.fileno()
    antigo = termios.tcgetattr(fd)

    print(__doc__[__doc__.index('Teclas:'):])
    print(f'\nMANDANDO ZERO. Arme agora (religue a placa ou gire as rodas até o beep mudar).')
    print(f'vel={vel_max} giro={giro_max}   log: {caminho_csv}\n')

    with open(caminho_csv, 'w', newline='') as fcsv:
        log = csv.writer(fcsv)
        # mega_*: último FT_DEBUG (só com --mega) — o que a MEGA escreveu na placa.
        log.writerow(['t', 'tecla', 'alvo_speed', 'alvo_steer', 'speed', 'steer',
                      'mega_steer', 'mega_speed', 'mega_set_ok', 'mega_pc_bad',
                      'mega_len_bad', 'mega_hover_tx'])
        t0 = time.time()
        prox = t0
        try:
            tty.setcbreak(fd)
            while True:
                tecla = ''
                espera = max(0.0, prox - time.time())
                if select.select([sys.stdin], [], [], espera)[0]:
                    tecla = sys.stdin.read(1).lower()

                agora = time.time()
                if tecla:
                    if tecla == 'q':
                        break
                    elif tecla == ' ':
                        alvo_v = alvo_g = v = g = 0
                    elif tecla in 'ws':
                        alvo_v = (vel_max if tecla == 'w' else -vel_max) * sinal_frente
                        ultima_tecla = agora
                    elif tecla in 'ad':
                        alvo_g = (-giro_max if tecla == 'a' else giro_max) * sinal_giro
                        ultima_tecla = agora
                    elif tecla in '+=':
                        vel_max = min(VEL_MAX_ABS, vel_max + 50)
                    elif tecla == '-':
                        vel_max = max(50, vel_max - 50)
                    elif tecla == 'i':
                        sinal_frente = -sinal_frente
                    elif tecla == 'o':
                        sinal_giro = -sinal_giro

                if agora - ultima_tecla > SEM_TECLA_PARA:
                    alvo_v = alvo_g = 0

                if agora >= prox:
                    v = aproxima(v, alvo_v)
                    g = aproxima(g, alvo_g)
                    s.write(monta(g, v))
                    volta = s.read(512)  # hover_ponte: a placa não responde legível
                    if a.mega:
                        debug.alimenta(volta)
                    log.writerow([f'{agora - t0:.3f}', tecla.strip(), alvo_v, alvo_g, v, g]
                                 + debug.ultimo)
                    prox += PERIODO
                    if prox < agora:  # atrasou (terminal travou): não dispara rajada
                        prox = agora + PERIODO
                    sys.stdout.write(f'\r speed={v:5d}  steer={g:5d}  max={vel_max:4d}  '
                                     f'frente={"+" if sinal_frente > 0 else "-"} '
                                     f'giro={"+" if sinal_giro > 0 else "-"}   ')
                    sys.stdout.flush()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, antigo)
            for _ in range(10):
                s.write(monta(0, 0))
                time.sleep(PERIODO)
            s.close()
            print(f'\nParado (zero enviado). Log em {caminho_csv}')


if __name__ == '__main__':
    main()
