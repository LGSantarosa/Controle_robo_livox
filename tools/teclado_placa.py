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
    m         MARCA no CSV "girou sozinha" (não manda nada para a placa)
    q        sai (manda zero antes de fechar)

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


class LeVolta:
    """Pela hover_ponte: conta bytes crus e acha o feedback 0xABCD da placa (18 bytes).

    Robô 3, 14-09: a placa gira sozinha com o PC mandando zero, e sem a volta dela
    não se sabe o que ela entendeu (cmd1/cmd2), a bateria nem se as rodas giram.
    """

    def __init__(self):
        self.buf = bytearray()
        self.bytes_total = 0
        self.ok = 0
        self.ruim = 0
        self.ultimo = ['', '', '', '', '', '']  # cmd1 cmd2 spdR spdL bat_V temp_C

    def alimenta(self, dados):
        self.bytes_total += len(dados)
        self.buf += dados
        while True:
            i = self.buf.find(b'\xcd\xab')
            if i < 0:
                del self.buf[:-1]
                return
            del self.buf[:i]
            if len(self.buf) < 18:
                return
            cmd1, cmd2, spd_r, spd_l, bat, temp, led, chk = struct.unpack('<hhhhhhHH', self.buf[2:18])
            esperado = START
            for campo in (cmd1, cmd2, spd_r, spd_l, bat, temp):
                esperado ^= campo & 0xFFFF
            esperado ^= led
            if esperado == chk:
                self.ok += 1
                self.ultimo = [cmd1, cmd2, spd_r, spd_l, f'{bat / 100:.2f}', f'{temp / 10:.1f}']
                del self.buf[:18]
            else:
                self.ruim += 1
                del self.buf[:2]


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
    ap.add_argument('--checksum-errado', action='store_true',
                    help='só com a hover_ponte: inverte o checksum de todo quadro. '
                         'Se a placa armar mesmo assim, ela não lê o comando como serial')
    a = ap.parse_args()
    if a.mega and a.checksum_errado:
        ap.error('--checksum-errado só vale pela hover_ponte (o mega_bridge refaz o checksum)')

    os.makedirs(os.path.expanduser('~/bancada_robo3'), exist_ok=True)
    nome = 'teclado_mega' if a.mega else ('teclado_chkerrado' if a.checksum_errado else 'teclado')
    caminho_csv = os.path.expanduser(time.strftime(f'~/bancada_robo3/{nome}_%Y%m%d_%H%M%S.csv'))
    if a.mega:
        monta = frame_mega
    elif a.checksum_errado:
        # Robô 3, 14-09: separar "a placa lê o quadro" de "a placa reage ao fio".
        monta = lambda g, v: frame(g, v)[:6] + bytes(b ^ 0xFF for b in frame(g, v)[6:])
    else:
        monta = frame
    debug = LeDebug()
    volta_placa = LeVolta()

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
                      'mega_len_bad', 'mega_hover_tx',
                      # placa_*: feedback 0xABCD (só pela hover_ponte); rx_bytes acumulado
                      'rx_bytes', 'placa_ok', 'placa_ruim', 'placa_cmd1', 'placa_cmd2',
                      'placa_spdR', 'placa_spdL', 'placa_bat_V', 'placa_temp_C'])
        t0 = time.time()
        prox = t0
        try:
            tty.setcbreak(fd)
            pendentes = ''  # teclas lidas desde a última linha: não se perdem entre envios
            while True:
                tecla = ''
                espera = max(0.0, prox - time.time())
                if select.select([sys.stdin], [], [], espera)[0]:
                    tecla = sys.stdin.read(1).lower()
                    if tecla.strip():
                        pendentes += tecla

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
                    volta = s.read(512)
                    if a.mega:
                        debug.alimenta(volta)
                    else:
                        volta_placa.alimenta(volta)
                    log.writerow([f'{agora - t0:.3f}', pendentes, alvo_v, alvo_g, v, g]
                                 + debug.ultimo
                                 + [volta_placa.bytes_total, volta_placa.ok, volta_placa.ruim]
                                 + volta_placa.ultimo)
                    pendentes = ''
                    prox += PERIODO
                    if prox < agora:  # atrasou (terminal travou): não dispara rajada
                        prox = agora + PERIODO
                    sys.stdout.write(f'\r speed={v:5d}  steer={g:5d}  max={vel_max:4d}  '
                                     f'frente={"+" if sinal_frente > 0 else "-"} '
                                     f'giro={"+" if sinal_giro > 0 else "-"}   ')
                    sys.stdout.flush()
        except Exception:
            # 14-09: deu erro na tela ao ligar a placa e o traceback se perdeu.
            # Fica ao lado do CSV para ler por ssh.
            import traceback
            with open(caminho_csv + '.erro.txt', 'w') as ferr:
                ferr.write(f't={time.time() - t0:.3f}\n')
                traceback.print_exc(file=ferr)
            raise
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, antigo)
            try:
                for _ in range(10):
                    s.write(monta(0, 0))
                    time.sleep(PERIODO)
                s.close()
            except Exception:
                pass  # porta morreu: o watchdog da MEGA (0,5 s) zera sozinho
            print(f'\nParado (zero enviado). Log em {caminho_csv}')


if __name__ == '__main__':
    main()
