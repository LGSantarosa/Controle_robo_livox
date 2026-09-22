#!/usr/bin/env python3
"""MEGA fingida da etapa 5 — contrato de comando (docs/PLANO_ETAPA5_ROBO3.md §3.3).

    mega_fingida5.py <pasta> <cenario>        (cenario: ver fases.yaml)

Derivada da MEGA fingida da etapa 4 (tools/valida_etapa4/mega_fingida.py,
evidência congelada, não alterada). O que muda:

  - fases com o direcional (eixo 7), com o /joy CALADO (perda do controle) e
    as conferências `timeout` e `perda` de fases.yaml;
  - tipos.yaml: para joy_vel, dpad_vel, cmd_vel e wheel_vel_setpoints, quem
    publica e quem assina, com o TIPO de mensagem, lido do grafo — é a foto
    que separa a cadeia Twist da TwistStamped.

Ordem igual à da etapa 4: pty + STATE a 50 Hz; bytes crus e frames
decodificados; /joy parado até <pasta>/vai; fases; <pasta>/mega.yaml; de pé
até <pasta>/fim. Nó OCULTO `_mega_fingida5`.
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
EIXO_FRENTE, EIXO_GIRO, EIXO_DPAD = 1, 0, 7
N_EIXOS, N_BOTOES = 8, 11
TOPICOS = ('/joy_vel', '/dpad_vel', '/cmd_vel', '/wheel_vel_setpoints')
ZERO = {'steer': 0, 'speed': 0}


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


def _janela(frames, t0, t1):
    return [q for t, q in frames if t0 <= t <= t1]


def confere(fase, t0, t1, frames, esperado, cfg, anterior=None):
    """Um veredito por fase. frames: [(t, bytes)] só FT_SET_SPEED com chk ok.

    `anterior`: o frame esperado da fase anterior (só a conferência `timeout`
    o usa — ver fases.yaml).
    """
    tipo = fase['confere']
    alvo = frame_set_speed(**esperado) if esperado else None
    zero = frame_set_speed(**ZERO)
    r = {'fase': fase['nome'], 'confere': tipo, 't_ini': round(t0, 3), 't_fim': round(t1, 3),
         'esperado': esperado, 'esperado_hex': alvo.hex() if alvo else None}
    if tipo == 'nada':
        r['veredito'] = 'ANOTADO'
        r['frames'] = len(_janela(frames, t0, t1))
        return r
    if tipo == 'igual':
        jan = _janela(frames, t0 + cfg['janela_s'], t1)
        dif = sorted({q.hex() for q in jan if q != alvo})
        r.update(frames_na_janela=len(jan), iguais=sum(q == alvo for q in jan),
                 diferentes_hex=dif)
        ok = bool(jan) and not dif
    elif tipo == 'zero':
        jan = _janela(frames, t0 + cfg['janela_s'], t1)
        ate = [q for t, q in frames if t <= t1]
        dif = sorted({q.hex() for q in jan if q != zero})
        r.update(frames_na_janela=len(jan), diferentes_hex=dif,
                 ultimo_ate_o_fim_hex=ate[-1].hex() if ate else None)
        ok = not dif and bool(ate) and ate[-1] == zero
    elif tipo == 'timeout':
        fase_fr = [(t, q) for t, q in frames if t0 <= t <= t1]
        velho = frame_set_speed(**anterior) if anterior else None
        limite = t0 + cfg['transito_s']
        i_zero = next((i for i, (t, q) in enumerate(fase_fr)
                       if q == zero and t <= limite), None)
        ok, atraso, em_transito, outros = False, None, [], []
        if i_zero is not None:
            antes, t_zero = fase_fr[:i_zero], fase_fr[i_zero][0]
            em_transito = [q.hex() for _, q in antes]
            transito_ok = all(q == velho for _, q in antes)
            depois = fase_fr[i_zero + 1:]
            outros = sorted({q.hex() for _, q in depois if q != alvo})
            if transito_ok and depois and not outros:
                atraso = depois[0][0] - t_zero
                ok = cfg['timeout_min_s'] <= atraso <= cfg['timeout_max_s']
        r.update(primeiro_hex=fase_fr[0][1].hex() if fase_fr else None,
                 em_transito_hex=em_transito, diferentes_hex=outros,
                 zero_ate_o_limite=i_zero is not None,
                 atraso_da_volta_s=None if atraso is None else round(atraso, 3),
                 frames_na_fase=len(fase_fr))
    elif tipo == 'perda':
        depois = [(t, q) for t, q in frames if t0 + cfg['perda_margem_s'] < t <= t1]
        ate = [(t, q) for t, q in frames if t <= t1]
        ultimo = ate[-1] if ate else None
        r.update(frames_depois_da_margem=len(depois),
                 ultimo_hex=ultimo[1].hex() if ultimo else None,
                 ultimo_depois_de_t0_s=None if ultimo is None else round(ultimo[0] - t0, 3))
        ok = not depois and ultimo is not None and ultimo[1] == alvo and alvo != zero
    else:
        raise ValueError(f'conferência desconhecida: {tipo}')
    r['veredito'] = 'APROVADO' if ok else 'REPROVADO'
    return r


def comando_da_fase(fase, caso):
    """(publica, lb, eixos) que o /joy fingido manda nesta fase."""
    eixos = [0.0] * N_EIXOS
    if fase.get('frente'):
        eixos[EIXO_FRENTE] = 1.0
    if fase.get('giro'):
        eixos[EIXO_GIRO] = caso['eixo_giro']
    if fase.get('dpad'):
        eixos[EIXO_DPAD] = 1.0
    return fase.get('publica', True), bool(fase['lb']), eixos


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


def _tipos(no):
    """{tópico: {publicadores: [[nó, tipo]], assinantes: [[nó, tipo]]}} do grafo."""
    out = {}
    for t in TOPICOS:
        pubs = no.get_publishers_info_by_topic(t)
        subs = no.get_subscriptions_info_by_topic(t)
        out[t] = {
            'publicadores': sorted([f'{i.node_namespace.rstrip("/")}/{i.node_name}',
                                    i.topic_type] for i in pubs
                                   if not i.node_name.startswith('_')),
            'assinantes': sorted([f'{i.node_namespace.rstrip("/")}/{i.node_name}',
                                  i.topic_type] for i in subs
                                 if not i.node_name.startswith('_')),
        }
    return out


def main(pasta, cenario):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import Joy
    import tf2_ros

    cfg = yaml.safe_load(open(os.path.join(AQUI, 'fases.yaml')))
    caso = cfg['cenarios'][cenario]
    fases = cfg['sequencias'][caso['sequencia']]

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
        n = 0
        diag = open(os.path.join(pasta, 'diag_state.txt'), 'w', buffering=1)
        diag.write(f'frame STATE = {q.hex()} ({len(q)} bytes)\n')
        while not parar.is_set():
            try:
                os.write(mestre, q)
                n += 1
                if n in (1, 10, 50, 250, 1000):
                    diag.write(f'{agora():.3f}s writes={n}\n')
            except OSError as e:
                import traceback
                diag.write(f'{agora():.3f}s OSError depois de {n} writes: {e!r}\n')
                traceback.print_exc(file=diag)
                return
            time.sleep(0.02)
        diag.write(f'{agora():.3f}s fim, writes={n}\n')

    threading.Thread(target=le, daemon=True).start()
    threading.Thread(target=estado, daemon=True).start()

    rclpy.init()
    no = Node('_mega_fingida5')
    pub = no.create_publisher(Joy, 'joy', 10)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, no)
    comando = {'publica': True, 'lb': False, 'eixos': [0.0] * N_EIXOS}

    def publica():
        if not comando['publica']:
            return
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

    with open(os.path.join(pasta, 'tipos.yaml'), 'w') as f:
        yaml.safe_dump(_tipos(no), f, allow_unicode=True, sort_keys=True)

    reais = []
    for fase in fases:
        comando['publica'], comando['lb'], comando['eixos'] = comando_da_fase(fase, caso)
        fase_atual[0] = fase['nome']
        t0 = agora()
        gira(t0 + fase['duracao_s'])
        reais.append((fase, t0, agora()))
    fase_atual[0] = 'depois'
    comando.update(publica=True, lb=False, eixos=[0.0] * N_EIXOS)
    gira(agora() + 0.5)

    with trava:
        fr = [(t, q) for t, q in frames if t <= reais[-1][2]]
    resultado_fases = [confere(f, t0, t1, fr, caso['frames'].get(f.get('espera')), cfg,
                               caso['frames'].get(f.get('anterior')))
                       for f, t0, t1 in reais]

    junta = junta_livox(_urdf_instalado())
    tf_ok, tf_info = False, {'amostras': len(tf_lida)}
    if junta and tf_lida:
        (xyz_u, rpy_u), ult = junta, tf_lida[-1]
        dx = max(abs(a - b) for a, b in zip(ult['xyz'], xyz_u))
        da = max(abs(a - b) for a, b in zip(ult['rpy'], rpy_u))
        tf_info.update(urdf_xyz=xyz_u, viva_xyz=ult['xyz'],
                       viva_yaw_graus=math.degrees(ult['rpy'][2]),
                       dif_max_xyz_m=dx, dif_max_rpy_rad=da)
        tf_ok = dx < 1e-9 and da < 1e-9
    conferidas = [f for f in resultado_fases if f['veredito'] != 'ANOTADO']
    ok = tf_ok and bool(conferidas) and all(f['veredito'] == 'APROVADO' for f in conferidas)
    resultado = {'cenario': cenario, 'argumentos': caso['argumentos'], 'pty': nome_escravo,
                 'frames_set_speed': len(fr), 'fases': resultado_fases,
                 'tf_base_link_livox_frame': {'veredito': 'APROVADO' if tf_ok else 'REPROVADO',
                                              **tf_info},
                 'veredito': 'APROVADO' if ok else 'REPROVADO'}
    with open(os.path.join(pasta, 'mega.yaml'), 'w') as f:
        yaml.safe_dump(resultado, f, allow_unicode=True, sort_keys=False)
    for f in resultado_fases:
        extra = f.get('atraso_da_volta_s', f.get('ultimo_depois_de_t0_s', ''))
        print(f"   {f['veredito']:9s} {f['fase']:12s} {f['confere']:8s} {extra}")
    print(f"   {resultado['tf_base_link_livox_frame']['veredito']:9s} TF base_link→livox_frame")

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
