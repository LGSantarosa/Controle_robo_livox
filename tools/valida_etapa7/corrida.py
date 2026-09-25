#!/usr/bin/env python3
"""A ponte entre a pasta da corrida e o juiz (decisão 060) — o que o
`bin/valida-etapa7` chama, para o bash não improvisar Python em heredoc.

    corrida.py objetivo <pose_inicial_bruta.yaml> <distância> <poses.yaml>
    corrida.py pose_final <pose_final_bruta.yaml> <poses.yaml>
    corrida.py assina <topic_info.txt>
    corrida.py julga <pasta>

`objetivo` e `pose_final` gravam o artefato NORMALIZADO `poses.yaml` (goal,
pose inicial e final em `{x, y}`), que fica na pasta para auditoria e é o que o
montador recebe. `assina` diz se o gravador já é assinante do tópico — goal
mandado antes disso perderia o começo da janela. `julga` junta os brutos da
pasta, extrai o bag, monta e julga, e imprime um item por linha
(`item<TAB>veredito<TAB>detalhe`); sai com 0 só se TODOS aprovarem.
"""
import glob
import math
import os
import sys

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)

import julga  # noqa: E402
import monta  # noqa: E402

GRAVADOR = '/rosbag2_recorder'

# Um item de coleta por fonte, SEMPRE — aprovado ou não —, para o CSV ter as
# mesmas linhas em toda corrida.
ITENS_DE_COLETA = (
    ('bag', 'coleta: bag legível'),
    (monta.ACAO, 'coleta: ação e janela (status × objetivo.log)'),
    (monta.TOL, 'coleta: tolerância viva (× materializado)'),
    (monta.PLACA, 'coleta: parâmetros da placa'),
    (monta.GRAFO, 'coleta: grafo antes = depois'),
)

ARQUIVOS = {
    'objetivo_log': 'objetivo.log',
    'dump_placa': 'placa_simulada.yaml',
    'dump_controller': 'controller_server.yaml',
    'grafo_antes': 'grafo_antes.txt',
    'grafo_depois': 'grafo_depois.txt',
}


# ── poses ───────────────────────────────────────────────────────────────────

def um_documento(texto):
    """O `ros2 topic echo --once` deste Jazzy fecha a mensagem com `---`, que
    em YAML abre um segundo documento (vazio). Exige-se EXATAMENTE um documento
    com conteúdo: dois seriam `--once` desrespeitado, e escolher um em
    silêncio seria adivinhar qual é a pose da corrida."""
    docs = [d for d in yaml.safe_load_all(texto) if d is not None]
    if len(docs) != 1:
        raise ValueError(f'esperava 1 documento, achei {len(docs)}')
    return docs[0]


def objetivo(pose, distancia):
    """`pose` = `pose.pose` do `/Odometry`. O alvo é `distancia` m no rumo
    ATUAL, preservando a orientação — "1 m à frente na mesma orientação"."""
    p, o = pose['position'], pose['orientation']
    yaw = math.atan2(2.0 * (o['w'] * o['z'] + o['x'] * o['y']),
                     1.0 - 2.0 * (o['y'] ** 2 + o['z'] ** 2))
    return {'x': p['x'] + distancia * math.cos(yaw),
            'y': p['y'] + distancia * math.sin(yaw)}, yaw


def _grava_poses(caminho, poses):
    tmp = caminho + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(poses, f, sort_keys=True)
    os.replace(tmp, caminho)


def _cmd_objetivo(bruta, distancia, poses_yaml):
    pose = um_documento(open(bruta).read())
    alvo, yaw = objetivo(pose, float(distancia))
    p, o = pose['position'], pose['orientation']
    _grava_poses(poses_yaml, {'goal': alvo,
                              'pose_inicial': {'x': p['x'], 'y': p['y']}})
    print('%.4f %.4f %.4f %.4f %.6f %.6f %.6f %.6f %.4f'
          % (p['x'], p['y'], alvo['x'], alvo['y'],
             o['x'], o['y'], o['z'], o['w'], yaw))


def _cmd_pose_final(bruta, poses_yaml):
    p = um_documento(open(bruta).read())
    poses = yaml.safe_load(open(poses_yaml))
    poses['pose_final'] = {'x': p['x'], 'y': p['y']}
    _grava_poses(poses_yaml, poses)
    g = poses['goal']
    print('%.4f %.4f %.4f' % (p['x'], p['y'],
                              math.hypot(p['x'] - g['x'], p['y'] - g['y'])))


# ── o gravador ──────────────────────────────────────────────────────────────

def gravador_assina(texto):
    try:
        return GRAVADOR in monta._grafo(texto, 'topic info')['assinantes']
    except monta._Falha:
        return False


# ── o julgamento ────────────────────────────────────────────────────────────

def _le_texto(caminho):
    try:
        with open(caminho) as f:
            return f.read()
    except OSError:
        return None


def brutos_da_pasta(pasta, le):
    """Junta os brutos. Arquivo ausente vira `None` — o montador reprova a
    fonte pelo nome. `le` é injetado: o teste não precisa de ROS nem de bag."""
    brutos = {chave: _le_texto(os.path.join(pasta, nome))
              for chave, nome in ARQUIVOS.items()}
    materializados = glob.glob(os.path.join(pasta, 'corrida_*',
                                            'perfil_nav2.yaml'))
    brutos['nav2_materializado'] = (_le_texto(materializados[0])
                                    if len(materializados) == 1 else None)
    poses = _le_texto(os.path.join(pasta, 'poses.yaml'))
    for chave, valor in ((yaml.safe_load(poses) or {}) if poses else {}).items():
        brutos[chave] = valor

    erro_bag = None
    try:
        extraido = le(os.path.join(pasta, 'bag'))
    except Exception as e:      # bag ilegível é falha de coleta, com o motivo
        extraido, erro_bag = {'amostras': [], 'status': None}, repr(e)
    brutos['amostras'] = extraido['amostras']
    brutos['status'] = extraido['status']
    return brutos, erro_bag


def vereditos(evidencia, falhas, erro_bag, avalia):
    linhas = []
    for fonte, item in ITENS_DE_COLETA:
        if fonte == 'bag':
            detalhes = [erro_bag] if erro_bag else []
        else:
            detalhes = [d for f, d in falhas if f == fonte]
        linhas.append((item, 'REPROVADO' if detalhes else 'APROVADO',
                       ' | '.join(detalhes)))
    for item, (ok, detalhe) in avalia(evidencia).items():
        linhas.append((item.strip(), 'APROVADO' if ok else 'REPROVADO',
                       str(detalhe)))
    return linhas


def _limpo(texto):
    return ' '.join(str(texto).split())


def _cmd_julga(pasta):
    import le_bag     # só aqui: precisa do ROS carregado
    brutos, erro_bag = brutos_da_pasta(pasta, le_bag.le)
    evidencia, falhas = monta.monta(brutos)
    linhas = vereditos(evidencia, falhas, erro_bag, julga.avalia)
    for item, veredito, detalhe in linhas:
        print(f'{_limpo(item)}\t{veredito}\t{_limpo(detalhe)}')
    return 0 if all(v == 'APROVADO' for _, v, _ in linhas) else 1


def main(argv):
    if len(argv) == 4 and argv[0] == 'objetivo':
        _cmd_objetivo(*argv[1:])
        return 0
    if len(argv) == 3 and argv[0] == 'pose_final':
        _cmd_pose_final(*argv[1:])
        return 0
    if len(argv) == 2 and argv[0] == 'assina':
        return 0 if gravador_assina(_le_texto(argv[1])) else 1
    if len(argv) == 2 and argv[0] == 'julga':
        return _cmd_julga(argv[1])
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
