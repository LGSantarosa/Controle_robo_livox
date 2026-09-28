#!/usr/bin/env python3
"""A ponte entre a pasta da corrida e o juiz (decisão 060) — o que o
`bin/valida-etapa7` chama, para o bash não improvisar Python em heredoc.

    corrida.py objetivo <pose_inicial_bruta.yaml> <distância> <poses.yaml>
    corrida.py pose_final <pose_final_bruta.yaml> <poses.yaml>
    corrida.py assina <topic_info.txt>
    corrida.py julga <pasta>
    corrida.py confere <julgamento.tsv> <rc do julga>

`objetivo` e `pose_final` gravam o artefato NORMALIZADO `poses.yaml` (goal,
pose inicial e final em `{x, y}`), que fica na pasta para auditoria e é o que o
montador recebe. `assina` diz se o gravador já é assinante do tópico — goal
mandado antes disso perderia o começo da janela. `julga` junta os brutos da
pasta, extrai o bag, monta e julga, e imprime um item por linha
(`item<TAB>veredito<TAB>detalhe`); sai com 0 só se TODOS aprovarem.
`confere` é a porta entre o `julga` e o `anota` do wrapper: reimprime as linhas
só se o TSV e o RC forem exatamente o esperado; senão sai com 1 e diz o motivo
no stderr, e nenhuma linha do TSV passa adiante.
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


def escalar_yaml(valor):
    """O float como escalar YAML — o MESMO texto que o `yaml.safe_dump` escreve
    no `poses.yaml`, e que o `ros2 action send_goal` relê com `yaml.safe_load`
    como o mesmo double. `repr` não serve: `1e-05` é texto no YAML 1.1."""
    return yaml.safe_dump(float(valor)).splitlines()[0]


def _cmd_objetivo(bruta, distancia, poses_yaml):
    pose = um_documento(open(bruta).read())
    alvo, yaw = objetivo(pose, float(distancia))
    p, o = pose['position'], pose['orientation']
    _grava_poses(poses_yaml, {'goal': alvo,
                              'pose_inicial': {'x': p['x'], 'y': p['y']}})
    # O alvo sai com o texto EXATO do `poses.yaml`: é ele que vai no goal, e é
    # contra ele que o juiz mede a pose final. O resto é só para o console.
    print('%.4f %.4f %s %s %.6f %.6f %.6f %.6f %.4f'
          % (p['x'], p['y'], escalar_yaml(alvo['x']), escalar_yaml(alvo['y']),
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


# Os 13 itens, na ordem do `vereditos`: 5 de coleta e os do juiz.
ITENS_ESPERADOS = tuple(item for _, item in ITENS_DE_COLETA) + tuple(
    nome.strip() for nome, _ in julga._ITENS)
VEREDITOS = ('APROVADO', 'REPROVADO')


class JulgamentoInvalido(Exception):
    """O TSV ou o RC do `julga` não são o que o contrato diz. Vira UMA linha
    conhecida REPROVADO no wrapper — nunca linhas não conferidas."""


def confere_julgamento(texto, rc):
    """Devolve `[(item, veredito, detalhe)]` se o TSV e o RC fecharem; senão
    levanta `JulgamentoInvalido` com o motivo. `rc` é o TEXTO que o bash passa.

    Fecha com: RC `0` ou `1`; toda linha terminada em quebra e com três campos;
    veredito só `APROVADO`/`REPROVADO`; os nomes formando EXATAMENTE o conjunto
    de `ITENS_ESPERADOS`, cada um uma vez; RC 0 se e só se todos aprovam.
    """
    if rc not in ('0', '1'):
        raise JulgamentoInvalido(f'RC do julga {rc!r}, esperado 0 ou 1')
    if not texto:
        raise JulgamentoInvalido('julgamento vazio')
    if not texto.endswith('\n'):
        raise JulgamentoInvalido('julgamento sem quebra de linha final '
                                 '(saída cortada?)')
    linhas = []
    for n, linha in enumerate(texto[:-1].split('\n'), 1):
        campos = linha.split('\t')
        if len(campos) != 3:
            raise JulgamentoInvalido(f'linha {n} com {len(campos)} campo(s), '
                                     'esperados 3')
        if campos[1] not in VEREDITOS:
            raise JulgamentoInvalido(f'linha {n}: veredito {campos[1]!r} fora '
                                     f'de {VEREDITOS}')
        linhas.append(tuple(campos))
    itens = [item for item, _, _ in linhas]
    repetidos = sorted({i for i in itens if itens.count(i) > 1})
    faltando = [i for i in ITENS_ESPERADOS if i not in itens]
    estranhos = [i for i in itens if i not in ITENS_ESPERADOS]
    if repetidos or faltando or estranhos:
        raise JulgamentoInvalido(
            f'itens fora do contrato: repetidos {repetidos}, faltando '
            f'{faltando}, desconhecidos {estranhos}')
    todos = all(v == 'APROVADO' for _, v, _ in linhas)
    if (rc == '0') != todos:
        raise JulgamentoInvalido(
            f'RC {rc} incoerente com os vereditos '
            f'({"todos aprovados" if todos else "há reprovação"})')
    return linhas


def _cmd_confere(caminho, rc):
    texto = _le_texto(caminho)
    try:
        if texto is None:
            raise JulgamentoInvalido(f'{caminho} ilegível')
        linhas = confere_julgamento(texto, rc)
    except JulgamentoInvalido as e:
        print(e, file=sys.stderr)
        return 1
    sys.stdout.write(''.join(f'{i}\t{v}\t{d}\n' for i, v, d in linhas))
    return 0


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
    if len(argv) == 3 and argv[0] == 'confere':
        return _cmd_confere(argv[1], argv[2])
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
