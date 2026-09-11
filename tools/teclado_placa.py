#!/usr/bin/env python3
"""Dirigir o robô 3 pelo teclado, direto na placa de hover (sem ROS).

Abre uma janela e lê APERTOU/SOLTOU de verdade (Tk), em vez de depender do
repeat do terminal. Fala o protocolo 0xABCD a 115200 pela MEGA com a
`firmware/hover_ponte` (ou por um USB-TTL: --porta /dev/ttyUSB0).
Feito para demonstração, não para navegação.

Comportamento (pedido do dono, 10-09):
  - SEGURANDO w/s anda, SOLTOU para. Nada apertado = zero, sempre.
  - a/d é PIVÔ (uma roda para cada lado), não curva. a/d têm prioridade
    sobre w/s.
  - velocidade CONSTANTE enquanto segura; rampa curta só para não dar degrau
    seco (o degrau de 0 a 300 desligou a placa em 10-09). Soltar freia em
    ~0,1 s.
  - a janela perdeu o foco -> zero (tecla "presa" não fica andando).

Por que assim (bancada 01-09 e 10-09):
  - uma thread manda comando a 50 Hz O TEMPO TODO, zero quando nada está
    apertado: a placa trava depois de um silêncio, e zeros mandados depois
    NÃO destravam;
  - se o script morrer, a MEGA cala e a placa zera pelo timeout dela.

Uso (no notebook, com a MEGA em /dev/ttyACM0):
    python3 tools/teclado_placa.py
    python3 tools/teclado_placa.py --vel 200 --giro 250

Teclas (com a JANELA em foco):
    w / s     frente / ré      (segurando)
    a / d     pivô esq / dir   (segurando)
    espaço    zera tudo
    + / -     velocidade de w/s  ±50
    [ / ]     velocidade do pivô ±50
    i / o     inverte frente / inverte pivô (se sair trocado)
    q / Esc   sai (manda zero antes de fechar)

Arme: com a janela aberta (já mandando zero), religue a placa ou gire as duas
rodas com a mão até o beep mudar. Só então comande.
"""

import argparse
import csv
import os
import struct
import threading
import time
import tkinter as tk

import serial

START = 0xABCD
PERIODO = 0.02        # 50 Hz
RAMPA_SOBE = 15       # por ciclo: 0 -> 250 em ~0,34 s
RAMPA_DESCE = 50      # por ciclo: 250 -> 0 em 0,1 s
SOLTOU_DEBOUNCE_MS = 60   # o autorepeat do X/Wayland manda solta+aperta em rajada
TETO = 600


def frame(steer, speed):
    return struct.pack('<HhhH', START, steer, speed,
                       (START ^ (steer & 0xFFFF) ^ (speed & 0xFFFF)) & 0xFFFF)


FB_TAM = 18   # start, cmd1, cmd2, speedR, speedL, bat, temp, cmdLed, checksum


def extrai_feedback(buf):
    """Tira do buffer os frames de feedback válidos da placa.

    Devolve (frames, resto). Cada frame é o dict dos campos; o checksum é o
    XOR de todos os campos anteriores (hoverboard.h do mega_bridge)."""
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
        return frames, buf[-1:]          # guarda um byte: pode ser o 0xCD de um início
    return frames, buf[i:]


def alvo(teclas, vel, giro, sinal_frente, sinal_giro):
    """(speed, steer) desejados para o conjunto de teclas seguradas.

    A placa distribui speedR = speed - steer, speedL = speed + steer, então
    speed=0 com steer≠0 é pivô puro."""
    a, d = 'a' in teclas, 'd' in teclas
    if a != d:
        return 0, (-giro if a else giro) * sinal_giro
    w, s = 'w' in teclas, 's' in teclas
    if w != s:
        return (vel if w else -vel) * sinal_frente, 0
    return 0, 0


def aproxima(atual, desejado):
    if desejado == atual:
        return atual
    # acelerar = afastar do zero; frear = aproximar do zero (ou trocar de sinal)
    acelerando = abs(desejado) > abs(atual) and (atual == 0 or (desejado > 0) == (atual > 0))
    passo = RAMPA_SOBE if acelerando else RAMPA_DESCE
    if desejado > atual:
        return min(desejado, atual + passo)
    return max(desejado, atual - passo)


class Controle:
    def __init__(self, porta, vel, giro, caminho_csv):
        self.lock = threading.Lock()
        self.teclas = set()
        self.vel, self.giro = vel, giro
        # -1: no robô 3 com speed>0 ele anda para trás (dono, 10-09).
        self.sinal_frente, self.sinal_giro = -1, 1
        self.v = self.g = 0
        self.rodando = True
        self.s = serial.Serial(porta, 115200, timeout=0)
        time.sleep(2.5)  # a MEGA reinicia ao abrir a porta
        self.s.reset_input_buffer()
        self.fcsv = open(caminho_csv, 'w', newline='')
        self.log = csv.writer(self.fcsv)
        self.log.writerow(['t', 'teclas', 'alvo_speed', 'alvo_steer', 'speed', 'steer',
                           'fb_n', 'bat', 'cmd1', 'cmd2', 'spdR', 'spdL', 'temp'])
        self.rx = b''
        self.fb = None      # último feedback válido
        self.fb_total = 0
        self.t0 = time.time()
        self.thread = threading.Thread(target=self._envia, daemon=True)
        self.thread.start()

    def _envia(self):
        prox = time.time()
        while self.rodando:
            with self.lock:
                teclas = ''.join(sorted(self.teclas))
                av, ag = alvo(self.teclas, self.vel, self.giro,
                              self.sinal_frente, self.sinal_giro)
            self.v = aproxima(self.v, av)
            self.g = aproxima(self.g, ag)
            self.s.write(frame(self.g, self.v))
            # A volta da placa (azul -> pino 19 da MEGA). Sem o fio, só ruído
            # e fb_n fica 0; com ele, o CSV mostra se a placa aceitou (cmd1/cmd2),
            # se a roda girou (spdR/spdL) e a bateria afundando.
            self.rx = (self.rx + self.s.read(512))[-4096:]
            fbs, self.rx = extrai_feedback(self.rx)
            if fbs:
                self.fb = fbs[-1]
                self.fb_total += len(fbs)
            f = self.fb if fbs else None
            self.log.writerow([f'{time.time() - self.t0:.3f}', teclas, av, ag, self.v, self.g,
                               len(fbs)] + ([f['bat'], f['cmd1'], f['cmd2'], f['spdR'],
                                             f['spdL'], f['temp']] if f else [''] * 6))
            prox += PERIODO
            espera = prox - time.time()
            if espera > 0:
                time.sleep(espera)
            else:
                prox = time.time()  # atrasou: não dispara rajada

    def fecha(self):
        self.rodando = False
        self.thread.join(timeout=1)
        for _ in range(10):
            self.s.write(frame(0, 0))
            time.sleep(PERIODO)
        self.s.close()
        self.fcsv.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--porta', default='/dev/ttyACM0')
    ap.add_argument('--vel', type=int, default=250, help='speed de w/s (padrão 250)')
    ap.add_argument('--giro', type=int, default=250, help='steer do pivô (padrão 250)')
    a = ap.parse_args()

    os.makedirs(os.path.expanduser('~/bancada_robo3'), exist_ok=True)
    caminho_csv = os.path.expanduser(time.strftime('~/bancada_robo3/teclado_%Y%m%d_%H%M%S.csv'))
    print(f'Abrindo {a.porta} (a MEGA reinicia ao abrir, 2,5 s)...')
    c = Controle(a.porta, a.vel, a.giro, caminho_csv)
    print(f'Mandando zero. Log: {caminho_csv}')

    raiz = tk.Tk()
    raiz.title('Robô 3 — teclado')
    raiz.geometry('520x260')
    info = tk.Label(raiz, font=('monospace', 14), justify='left')
    info.pack(expand=True, fill='both', padx=16, pady=16)
    pendente = {}  # tecla -> id do after() que vai soltá-la

    def solta(k):
        pendente.pop(k, None)
        with c.lock:
            c.teclas.discard(k)

    def aperta(ev):
        k = ev.keysym.lower()
        if k in pendente:                 # era autorepeat: cancela a soltura
            raiz.after_cancel(pendente.pop(k))
        if k in ('w', 'a', 's', 'd'):
            with c.lock:
                c.teclas.add(k)
        elif k == 'space':
            with c.lock:
                c.teclas.clear()
        elif k in ('plus', 'equal', 'kp_add'):
            c.vel = min(TETO, c.vel + 50)
        elif k in ('minus', 'kp_subtract'):
            c.vel = max(50, c.vel - 50)
        elif k == 'bracketright':
            c.giro = min(TETO, c.giro + 50)
        elif k == 'bracketleft':
            c.giro = max(50, c.giro - 50)
        elif k == 'i':
            c.sinal_frente = -c.sinal_frente
        elif k == 'o':
            c.sinal_giro = -c.sinal_giro
        elif k in ('q', 'escape'):
            sair()

    def soltou(ev):
        k = ev.keysym.lower()
        if k in ('w', 'a', 's', 'd'):
            if k in pendente:
                raiz.after_cancel(pendente[k])
            pendente[k] = raiz.after(SOLTOU_DEBOUNCE_MS, solta, k)

    def perdeu_foco(_ev):
        with c.lock:
            c.teclas.clear()

    def atualiza():
        with c.lock:
            t = ''.join(sorted(c.teclas)) or '-'
        f = c.fb
        placa = (f'placa: {f["bat"]:.1f} V  cmd2={f["cmd2"]}  rodas R={f["spdR"]} L={f["spdL"]}'
                 if f else 'placa: sem resposta (azul no pino 19?)')
        info.config(text=(
            f'segurando: {t}\n{placa}\n\n'
            f'speed = {c.v:5d}   steer = {c.g:5d}\n'
            f'vel w/s = {c.vel}   pivô = {c.giro}\n'
            f'frente {"+" if c.sinal_frente > 0 else "-"}   pivô {"+" if c.sinal_giro > 0 else "-"}\n\n'
            'w/s anda  a/d pivô  +/- vel  [ ] pivô  q sai'))
        raiz.after(50, atualiza)

    def sair():
        c.fecha()
        raiz.destroy()
        print(f'Parado (zero enviado). Log em {caminho_csv}')

    raiz.bind('<KeyPress>', aperta)
    raiz.bind('<KeyRelease>', soltou)
    raiz.bind('<FocusOut>', perdeu_foco)
    raiz.protocol('WM_DELETE_WINDOW', sair)
    atualiza()
    try:
        raiz.mainloop()
    finally:
        if c.rodando:
            c.fecha()


if __name__ == '__main__':
    main()
