#!/usr/bin/env python3
"""Sentido bridge -> MEGA, sem a bancada: o mega_bridge escreve no pty?

Um WheelSpeeds conhecido (left=right=-120) tem de virar FT_SET_SPEED com
steer=0 e speed=-120. Leitor mínimo do pty, sem a MEGA fingida — as duas
tentativas anteriores falharam por erro MEU no script de diagnóstico
(`--no-daemon` inexistente no `ros2 topic pub`, e `--once` publicando antes de
a assinatura casar), não por defeito do sistema.
"""
import os, pty, subprocess, threading, time, tty

mestre, escravo = pty.openpty(); tty.setraw(escravo)
nome = os.ttyname(escravo); print('pty', nome)
lidos = bytearray()


def le():
    while True:
        try:
            lidos.extend(os.read(mestre, 4096))
        except OSError:
            return


threading.Thread(target=le, daemon=True).start()
br = subprocess.Popen(['ros2', 'run', 'robot_nav', 'mega_bridge', '--ros-args',
                       '-p', f'port:={nome}', '-p', 'baud:=230400'],
                      stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                      start_new_session=True)
time.sleep(5)
pub = subprocess.Popen(['ros2', 'topic', 'pub', '-r', '10', '/wheel_vel_setpoints',
                        'wheel_msgs/msg/WheelSpeeds',
                        '{left_wheel: -120.0, right_wheel: -120.0}'],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                       start_new_session=True)
time.sleep(4)
for p in (pub, br):
    p.terminate()
time.sleep(1)
for p in (pub, br):
    p.kill()
print('bytes lidos do pty:', len(lidos))
print('primeiro frame:', lidos[:13].hex())
