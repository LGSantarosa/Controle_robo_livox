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


def aproxima(atual, alvo):
    if alvo > atual:
        return min(alvo, atual + RAMPA)
    return max(alvo, atual - RAMPA)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--porta', default='/dev/ttyACM0')
    ap.add_argument('--vel', type=int, default=250, help='speed máximo (padrão 250)')
    ap.add_argument('--giro', type=int, default=150, help='steer máximo (padrão 150)')
    a = ap.parse_args()

    os.makedirs(os.path.expanduser('~/bancada_robo3'), exist_ok=True)
    caminho_csv = os.path.expanduser(time.strftime('~/bancada_robo3/teclado_%Y%m%d_%H%M%S.csv'))

    s = serial.Serial(a.porta, 115200, timeout=0)
    print(f'Abrindo {a.porta}... (a MEGA reinicia ao abrir, 2,5 s)')
    time.sleep(2.5)
    s.reset_input_buffer()

    vel_max, giro_max = a.vel, a.giro
    sinal_frente, sinal_giro = 1, 1
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
        log.writerow(['t', 'tecla', 'alvo_speed', 'alvo_steer', 'speed', 'steer'])
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
                    s.write(frame(g, v))
                    s.read(512)  # descarta a volta: a placa não responde legível
                    log.writerow([f'{agora - t0:.3f}', tecla.strip(), alvo_v, alvo_g, v, g])
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
                s.write(frame(0, 0))
                time.sleep(PERIODO)
            s.close()
            print(f'\nParado (zero enviado). Log em {caminho_csv}')


if __name__ == '__main__':
    main()
