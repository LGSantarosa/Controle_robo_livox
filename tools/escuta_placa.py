#!/usr/bin/env python3
"""Escuta ao vivo o fio de volta da placa (azul -> pino 19), pela hover_ponte.

Robô 3, 14-09: o verde leva o comando e a placa lê (checksum invertido = roda
imóvel), mas o azul só trouxe ruído (0xBF/0xFF) e o nosso quadro vazando. Em
01-09 a mesma MEGA recebeu a bateria por esse fio. Isto é para o dono mexer no
cabo e ver, na hora, se sai algo de verdade.

A cada 0,5 s uma linha:
    SILÊNCIO        nenhum byte
    RUÍDO           bytes, mas nenhum quadro 0xABCD válido
    PLACA FALANDO   quadros válidos (bateria, rpm) — apita na primeira vez

Por padrão a MEGA NÃO transmite nada: sem comando a placa não arma e as rodas
não giram enquanto se mexe no cabo. Com --zero manda speed=0 a 50 Hz (em 01-09
a bateria chegou assim), mas aí a placa pode armar e girar sozinha.

Uso (MEGA com firmware/hover_ponte):
    python3 tools/escuta_placa.py
    python3 tools/escuta_placa.py --zero
CSV por janela em ~/bancada_robo3/escuta_<data>.csv. Ctrl+C sai.
"""

import argparse
import collections
import csv
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from teclado_placa import LeVolta, frame  # noqa: E402

JANELA = 0.5


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--porta', default='/dev/ttyACM0')
    ap.add_argument('--zero', action='store_true',
                    help='manda speed=0 a 50 Hz (a placa pode armar e girar sozinha)')
    a = ap.parse_args()

    os.makedirs(os.path.expanduser('~/bancada_robo3'), exist_ok=True)
    caminho = os.path.expanduser(time.strftime('~/bancada_robo3/escuta_%Y%m%d_%H%M%S.csv'))

    s = serial.Serial(a.porta, 115200, timeout=0)
    print(f'Abrindo {a.porta}... (a MEGA reinicia ao abrir, 2,5 s)')
    time.sleep(2.5)
    s.reset_input_buffer()
    print('MANDANDO speed=0 a 50 Hz (rodas no ar!)' if a.zero else 'MEGA CALADA: só escuta, nada vai para a placa')
    print(f'log: {caminho}\nCtrl+C sai.\n')

    volta = LeVolta()
    ja_falou = False
    with open(caminho, 'w', newline='') as fcsv:
        log = csv.writer(fcsv)
        log.writerow(['t', 'bytes', 'pct_bf_ff', 'quadros_ok', 'quadros_ruim',
                      'veredito', 'bat_V', 'spdR', 'spdL', 'cmd1', 'cmd2', 'temp_C'])
        t0 = time.time()
        prox_tx = t0
        fim_janela = t0 + JANELA
        crus = bytearray()
        ok0 = ruim0 = 0
        try:
            while True:
                agora = time.time()
                if a.zero and agora >= prox_tx:
                    s.write(frame(0, 0))
                    prox_tx += 0.02
                    if prox_tx < agora:
                        prox_tx = agora + 0.02
                dados = s.read(512)
                if dados:
                    crus += dados
                    volta.alimenta(dados)
                if agora < fim_janela:
                    time.sleep(0.002)
                    continue

                n = len(crus)
                cont = collections.Counter(crus)
                pct = 100.0 * (cont[0xBF] + cont[0xFF]) / n if n else 0.0
                ok, ruim = volta.ok - ok0, volta.ruim - ruim0
                cmd1, cmd2, spd_r, spd_l, bat, temp = volta.ultimo
                if ok:
                    veredito = 'PLACA FALANDO'
                    texto = (f'✅ PLACA FALANDO  bat={bat} V  spdR={spd_r} spdL={spd_l}  '
                             f'cmd={cmd1},{cmd2}  ({ok} quadros)')
                    if not ja_falou:
                        texto += '\a'
                        ja_falou = True
                elif n:
                    veredito = 'RUIDO'
                    texto = f'〰️  RUÍDO  {n / JANELA:5.0f} bytes/s  ({pct:.0f}% 0xBF/0xFF)'
                else:
                    veredito = 'SILENCIO'
                    texto = '·  SILÊNCIO'
                print(f'{agora - t0:6.1f}s  {texto}')
                log.writerow([f'{agora - t0:.1f}', n, f'{pct:.0f}', ok, ruim, veredito]
                             + ([bat, spd_r, spd_l, cmd1, cmd2, temp] if ok else [''] * 6))
                fcsv.flush()

                crus = bytearray()
                ok0, ruim0 = volta.ok, volta.ruim
                fim_janela += JANELA
                if fim_janela < agora:
                    fim_janela = agora + JANELA
        except KeyboardInterrupt:
            pass
        finally:
            s.close()
            print(f'\nSaiu. Log em {caminho}')


if __name__ == '__main__':
    main()
