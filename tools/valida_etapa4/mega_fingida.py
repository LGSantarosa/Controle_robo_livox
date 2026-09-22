#!/usr/bin/env python3
"""MEGA fingida em pty + controle fingido + TF viva — etapa 4, passo 7 (§10.5).

    mega_fingida.py <pasta> <cenario>        (cenario: ver frames_esperados.yaml)

Ordem:
  1. abre um pty e escreve o lado escravo em <pasta>/pty.txt (é a `porta:=`
     do sobe-robo3); manda STATE a 50 Hz como a MEGA (placa respondendo,
     36,50 V) para as conferências do sobe-robo3 passarem;
  2. grava TODO byte recebido: <pasta>/bytes.bin (cru) e bytes.csv (instante,
     hex); decodifica os frames em frames.csv;
  3. publica /joy a 20 Hz, parado (LB solto), até existir <pasta>/vai — o
     wrapper cria depois de o sobe-robo3 subir e de o grafo ficar pronto;
  4. roda as fases de frames_esperados.yaml e, junto, lê a TF
     base_link → livox_frame;
  5. <pasta>/mega.yaml: por fase, os frames da janela e a comparação EXATA
     (bytes) com o esperado; a TF (xyz, roll/pitch/yaw) contra a junta
     `livox_joint` do URDF instalado;
  6. continua de pé (pty aberto, STATE, /joy parado) até existir <pasta>/fim:
     o `sobe-robo3 --mata` roda com ela viva, e ela — fora do grupo dele —
     tem de sobreviver. Sai 1 se algo reprovou.

O nó dele é OCULTO (`_mega_fingida`): fica fora do `ros2 node list` e da
comparação com a lista esperada.
"""
import csv
import math
import os
import pty
import struct
import sys
import threading
import time
import tty
import xml.etree.ElementTree as ET

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
START0, START1 = 0xAA, 0x55
FT_SET_SPEED, FT_STATE = 0x01, 0x81
LB = 6
N_EIXOS, N_BOTOES = 8, 11


# ─── protocolo (puro) ────────────────────────────────────────────────────────

def xor8(dados):
    x = 0
    for b in dados:
        x ^= b
    return x & 0xFF


def monta_frame(tipo, payload):
    cab = bytes([tipo, len(payload)])
    return bytes([START0, START1]) + cab + payload + bytes([xor8(cab + payload)])


def frame_set_speed(steer, speed):
    """O frame exato que o mega_bridge manda (traseira = frente)."""
    return monta_frame(FT_SET_SPEED, struct.pack('<hhhh', steer, speed, steer, speed))


def frame_state(bat_x100=3650):
    """Monta o STATE de 16 bytes da MEGA.

    rpm ×4, bateria F/R ×100, falhas F/R 0 (não stale), byte sem uso, flags
    (imu_ok | flow_ok).
    """
    return monta_frame(FT_STATE, struct.pack('<hhhhhhBBBB', 0, 0, 0, 0,
                                             bat_x100, bat_x100, 0, 0, 0, 0x03))


class Decodificador:
    """Máquina de estados do lado da MEGA: AA 55 tipo len payload xor."""

    def __init__(self):
        self.estado, self.buf = 0, bytearray()

    def alimenta(self, b):
        """Devolve (tipo, payload, bytes_do_frame, chk_ok) ao completar um frame."""
        if self.estado == 0:
            if b == START0:
                self.estado, self.buf = 1, bytearray([b])
            return None
        if self.estado == 1:
            if b == START1:
                self.estado = 2
                self.buf.append(b)
            elif b == START0:
                self.buf = bytearray([b])
            else:
                self.estado = 0
            return None
        self.buf.append(b)
        if len(self.buf) >= 4 and len(self.buf) == 4 + self.buf[3] + 1:
            tipo, n = self.buf[2], self.buf[3]
            payload = bytes(self.buf[4:4 + n])
            ok = xor8(self.buf[2:4 + n]) == self.buf[-1]
            quadro = bytes(self.buf)
            self.estado = 0
            return tipo, payload, quadro, ok
        if len(self.buf) > 4 + 64 + 1:
            self.estado = 0
        return None


def avalia(frames, fases, esperados, janela_s):
    """Compara, fase a fase, os frames FT_SET_SPEED da janela com o esperado.

    frames: [(t, bytes_do_frame)] só FT_SET_SPEED com chk ok.
    fases: [(nome, t_ini, t_fim)] reais. esperados: {nome: {steer, speed}}.
    Fase com LB (esperado não-zero): a janela tem de ter frames, TODOS iguais.
    Fase solta (zero): nenhum frame não-zero na janela, e o último frame
    até o fim da fase é o zero (o teleop manda um zero ao soltar e cala).
    """
    out = []
    for nome, t0, t1 in fases:
        if nome not in esperados:
            continue
        e = esperados[nome]
        alvo = frame_set_speed(e['steer'], e['speed'])
        na_janela = [q for t, q in frames if t0 + janela_s <= t <= t1]
        diferentes = sorted({q.hex() for q in na_janela if q != alvo})
        if e['steer'] == 0 and e['speed'] == 0:
            ate_fim = [q for t, q in frames if t <= t1]
            ok = not diferentes and bool(ate_fim) and ate_fim[-1] == alvo
        else:
            ok = bool(na_janela) and not diferentes
        out.append({'fase': nome, 't_ini': round(t0, 3), 't_fim': round(t1, 3),
                    'esperado': dict(e), 'esperado_hex': alvo.hex(),
                    'frames_na_janela': len(na_janela),
                    'iguais_ao_esperado': sum(q == alvo for q in na_janela),
                    'diferentes_hex': diferentes,
                    'veredito': 'APROVADO' if ok else 'REPROVADO'})
    return out


def rpy(qx, qy, qz, qw):
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return roll, pitch, yaw


def junta_livox(urdf_xml):
    """(xyz, rpy) da junta de topo base_link → livox_frame do URDF."""
    raiz = ET.fromstring(urdf_xml)
    for j in raiz.findall('joint'):
        if (j.find('parent').get('link'), j.find('child').get('link')) == \
                ('base_link', 'livox_frame'):
            o = j.find('origin')
            xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
            ang = [float(v) for v in o.get('rpy', '0 0 0').split()]
            return xyz, ang
    return None


# ─── a bancada ───────────────────────────────────────────────────────────────

def _urdf_instalado():
    import xacro
    from ament_index_python.packages import get_package_share_directory
    caminho = os.path.join(get_package_share_directory('robot_base'), 'description',
                           'robo3.urdf.xacro')
    return xacro.process_file(caminho, mappings={'sim': 'false'}).toxml()


def main(pasta, cenario):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import Joy
    import tf2_ros

    cfg = yaml.safe_load(open(os.path.join(AQUI, 'frames_esperados.yaml')))
    caso = cfg['cenarios'][cenario]

    mestre, escravo = pty.openpty()
    tty.setraw(escravo)
    nome_escravo = os.ttyname(escravo)
    with open(os.path.join(pasta, 'pty.txt'), 'w') as f:
        f.write(nome_escravo + '\n')

    t_zero = time.monotonic()
    agora = lambda: time.monotonic() - t_zero  # noqa: E731
    frames, trava, parar = [], threading.Lock(), threading.Event()
    f_bin = open(os.path.join(pasta, 'bytes.bin'), 'wb')
    f_hex = csv.writer(open(os.path.join(pasta, 'bytes.csv'), 'w', newline=''))
    f_hex.writerow(['t_s', 'hex'])
    f_fr = csv.writer(open(os.path.join(pasta, 'frames.csv'), 'w', newline=''))
    f_fr.writerow(['t_s', 'fase', 'tipo', 'chk_ok', 'hex', 'steer', 'speed',
                   'steer_rear', 'speed_rear'])
    fase_atual = ['antes']

    def le():
        dec = Decodificador()
        while not parar.is_set():
            try:
                bloco = os.read(mestre, 4096)
            except OSError:
                return
            t = agora()
            f_bin.write(bloco)
            f_hex.writerow([f'{t:.4f}', bloco.hex()])
            for b in bloco:
                r = dec.alimenta(b)
                if r is None:
                    continue
                tipo, payload, quadro, ok = r
                campos = list(struct.unpack('<hhhh', payload)) \
                    if tipo == FT_SET_SPEED and len(payload) == 8 else ['', '', '', '']
                f_fr.writerow([f'{t:.4f}', fase_atual[0], f'0x{tipo:02x}', ok,
                               quadro.hex(), *campos])
                if tipo == FT_SET_SPEED and ok:
                    with trava:
                        frames.append((t, quadro))

    def estado():
        q = frame_state()
        while not parar.is_set():
            try:
                os.write(mestre, q)
            except OSError:
                return
            time.sleep(0.02)

    threading.Thread(target=le, daemon=True).start()
    threading.Thread(target=estado, daemon=True).start()

    rclpy.init()
    no = Node('_mega_fingida')
    pub = no.create_publisher(Joy, 'joy', 10)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, no)
    comando = {'lb': False, 'eixos': [0.0] * N_EIXOS}

    def publica():
        m = Joy()
        m.header.stamp = no.get_clock().now().to_msg()
        m.axes = list(comando['eixos'])
        m.buttons = [0] * N_BOTOES
        m.buttons[LB] = 1 if comando['lb'] else 0
        pub.publish(m)

    no.create_timer(0.05, publica)
    tf_lida = []

    def le_tf():
        try:
            tr = buf.lookup_transform('base_link', 'livox_frame', Time())
        except tf2_ros.TransformException:
            return
        t, r = tr.transform.translation, tr.transform.rotation
        tf_lida.append({'t_s': round(agora(), 3), 'xyz': [t.x, t.y, t.z],
                        'quat': [r.x, r.y, r.z, r.w],
                        'rpy': list(rpy(r.x, r.y, r.z, r.w))})

    no.create_timer(0.5, le_tf)

    def gira(ate):
        while agora() < ate:
            rclpy.spin_once(no, timeout_sec=0.01)

    vai = os.path.join(pasta, 'vai')
    prazo = agora() + 600
    while not os.path.exists(vai) and agora() < prazo:
        gira(agora() + 0.2)
    if not os.path.exists(vai):
        print('🔴 o wrapper não liberou as fases em 600 s')
        return 1

    fases_reais = []
    for fase in cfg['fases']:
        comando['lb'] = fase['lb']
        comando['eixos'] = [0.0] * N_EIXOS
        if fase.get('eixo') == 'frente':
            comando['eixos'][1] = caso['eixo_frente']
        elif fase.get('eixo') == 'giro':
            comando['eixos'][0] = caso['eixo_giro']
        fase_atual[0] = fase['nome']
        t0 = agora()
        gira(t0 + fase['duracao_s'])
        fases_reais.append((fase['nome'], t0, agora()))
    fase_atual[0] = 'depois'
    comando['lb'], comando['eixos'] = False, [0.0] * N_EIXOS
    gira(agora() + 0.5)

    with trava:
        fr = [(t, q) for t, q in frames if t <= fases_reais[-1][2]]
    resultado_fases = avalia(fr, fases_reais, caso['fases'], cfg['janela_s'])

    junta = junta_livox(_urdf_instalado())
    tf_ok, tf_info = False, {'lida': tf_lida[:3], 'amostras': len(tf_lida)}
    if junta and tf_lida:
        (xyz_u, rpy_u), ult = junta, tf_lida[-1]
        dx = max(abs(a - b) for a, b in zip(ult['xyz'], xyz_u))
        da = max(abs(a - b) for a, b in zip(ult['rpy'], rpy_u))
        tf_info.update({'urdf_xyz': xyz_u, 'urdf_rpy': rpy_u, 'viva_xyz': ult['xyz'],
                        'viva_rpy_rad': ult['rpy'],
                        'viva_yaw_graus': math.degrees(ult['rpy'][2]),
                        'dif_max_xyz_m': dx, 'dif_max_rpy_rad': da})
        tf_ok = dx < 1e-9 and da < 1e-9 and abs(ult['rpy'][2]) < 1e-9
    resultado = {
        'cenario': cenario, 'argumentos': caso['argumentos'], 'pty': nome_escravo,
        'frames_set_speed': len(fr),
        'fases': resultado_fases,
        'tf_base_link_livox_frame': {'veredito': 'APROVADO' if tf_ok else 'REPROVADO',
                                     **tf_info},
    }
    com_expectativa = sum(f['nome'] in caso['fases'] for f in cfg['fases'])
    ok = tf_ok and all(f['veredito'] == 'APROVADO' for f in resultado_fases) \
        and len(resultado_fases) == com_expectativa
    resultado['veredito'] = 'APROVADO' if ok else 'REPROVADO'
    with open(os.path.join(pasta, 'mega.yaml'), 'w') as f:
        yaml.safe_dump(resultado, f, allow_unicode=True, sort_keys=False)
    for f in resultado_fases:
        print(f"   {f['veredito']:9s} {f['fase']:12s} {f['iguais_ao_esperado']}/"
              f"{f['frames_na_janela']} frames = {f['esperado']}")
    print(f"   {resultado['tf_base_link_livox_frame']['veredito']:9s} TF base_link→"
          f"livox_frame yaw {tf_info.get('viva_yaw_graus', 'sem leitura')}")
    fim = os.path.join(pasta, 'fim')
    prazo = agora() + 600
    while not os.path.exists(fim) and agora() < prazo:
        gira(agora() + 0.2)
    parar.set()
    no.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1], sys.argv[2]))
