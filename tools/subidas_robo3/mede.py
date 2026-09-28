#!/usr/bin/env python3
"""As medidas de uma subida do robô 3 e o veredito da bateria (decisão 061).

    mede.py cabecalho
    mede.py linha <subida_NN/> <n>          uma linha do subidas.csv
    mede.py veredito <subidas.csv> <N> <interrompida 0|1>
    mede.py sinais <launch.log>             "<fora_da_regra> <sigsegv_teardown>"

`linha` lê, da subpasta da subida: `launch.log` (bruto), `controladores.txt`
(uma chamada `ros2 control list_controllers`), `joint_states.txt` (uma janela
de `ros2 topic echo`), `amostras.txt` (o amostrador de carga) e `medidas.env`
(o que o wrapper mediu: Nav2/TF, janela, limpeza). Arquivo ausente vale vazio:
a coluna sai vazia ou zero e a subida não é nominal — nunca inventa medida.

O veredito escreve o limite quando não há falha: 20/20 não é "taxa zero", é
taxa abaixo de ~13,9% com 95% de confiança.
"""
import csv
import os
import re
import sys

JSB = 'joint_state_broadcaster'
BASE = 'hoverboard_base_controller'
ESTADOS = ('active', 'inactive', 'unconfigured', 'finalized')

# A regra de 28-09 para sinais: só o SIGSEGV do collision_monitor INICIADO no
# teardown (dívida 057) é tolerado — registrado, e a limpeza pode ser
# recuperada. Qualquer outro sinal, ou SIGSEGV antes do teardown, interrompe.
TOLERADO = ('collision_monitor', -11)

COLUNAS = (
    'subida', 'subida_nominal',
    'nav2_tf_pronto', 't_nav2_tf_s',
    'jsb_estado', 'base_estado', 'controladores_ok',
    'joint_states_msgs', 'joint_states_janela_s',
    'switch_timeout', 'falha_ativar', 'died_antes_do_fim',
    't_ativacao_jsb_s', 't_ativacao_base_s',
    'sigsegv_teardown', 'sinais_fora_da_regra',
    'load1_max', 'psi_cpu_some_avg10_max', 'psi_cpu_some_us', 'amostras',
    'limpeza_ok', 'shm_orfaos', 'limpeza_recuperada',
)

_ANSI = re.compile(r'\x1b\[[0-9;]*m')
_MORTE = re.compile(r'\[([A-Za-z0-9_]+)-\d+\]: process has died \[pid \d+, '
                    r'exit code (-?\d+)')
_CARIMBO = re.compile(r'^\[[^\]]+\] \[[A-Z]+\] \[(\d+(?:\.\d+)?)\]')


# ── o launch.log ────────────────────────────────────────────────────────────

def _divide(texto):
    """(subida, teardown): o corte é o primeiro `signal_handler(SIGINT`."""
    linhas = texto.splitlines()
    for i, l in enumerate(linhas):
        if 'signal_handler(SIGINT' in l:
            return linhas[:i], linhas[i:]
    return linhas, []


def _ate_o_encerramento(texto):
    """Só o que veio ANTES do primeiro `signal_handler(SIGINT`: o teardown
    (decisão 057) não é subida."""
    return _divide(texto)[0]


def _sinais(linhas):
    """(nome, código) de quem morreu por SINAL (código negativo)."""
    return [(m.group(1), int(m.group(2))) for l in linhas
            for m in [_MORTE.search(l)] if m and int(m.group(2)) < 0]


def _carimbo(linha):
    m = _CARIMBO.match(linha)
    return float(m.group(1)) if m else None


def _ativacao(linhas, controlador):
    """Do `Activating controllers: [ X ]` do controller_manager ao
    `Configured and activated X` do spawner. None se não ativou."""
    inicio = fim = None
    for l in linhas:
        if inicio is None and f'Activating controllers: [ {controlador} ]' in l:
            inicio = _carimbo(l)
        elif inicio is not None and l.rstrip().endswith(
                f'Configured and activated {controlador}'):
            fim = _carimbo(l)
            break
    if inicio is None or fim is None:
        return None
    return round(fim - inicio, 6)


def analisa_log(texto):
    linhas, teardown = _divide(texto)
    antes, depois = _sinais(linhas), _sinais(teardown)
    tolerados = [s for s in depois if s == TOLERADO]
    return {
        'sigsegv_teardown': len(tolerados),
        'sinais_fora_da_regra': len(antes) + len(depois) - len(tolerados),
        'switch_timeout': sum('Switch controller timed out' in l for l in linhas),
        'falha_ativar': sum('Failed to activate controller' in l for l in linhas),
        'died_antes_do_fim': sum('process has died' in l for l in linhas),
        't_ativacao_jsb_s': _ativacao(linhas, JSB),
        't_ativacao_base_s': _ativacao(linhas, BASE),
    }


# ── list_controllers, echo e amostrador ─────────────────────────────────────

_ESTADO_DO_SERVICO = re.compile(r"ControllerState\(name='([^']*)', state='([^']*)'")


def controladores(texto):
    """Dois formatos: a resposta do serviço
    `/controller_manager/list_controllers` (a representação da mensagem, que é
    o que este PC tem — sem `ros2 control`) e a tabela do `ros2 control
    list_controllers`, onde ele existir."""
    texto = _ANSI.sub('', texto)
    saida = {n: e for n, e in _ESTADO_DO_SERVICO.findall(texto) if e in ESTADOS}
    if saida:
        return saida
    for l in texto.splitlines():
        campos = l.split()
        if len(campos) >= 3 and campos[-1] in ESTADOS and '/' in campos[1]:
            saida[campos[0]] = campos[-1]
    return saida


def conta_mensagens(texto):
    return sum(1 for l in texto.splitlines() if l.strip() == '---')


def resume_amostras(texto):
    """Linhas `epoch load1 psi_some_avg10 psi_some_total_us`."""
    amostras = []
    for l in texto.splitlines():
        campos = l.split()
        try:
            t, load1, avg10, total = (float(campos[0]), float(campos[1]),
                                      float(campos[2]), int(campos[3]))
        except (IndexError, ValueError):
            continue
        amostras.append((t, load1, avg10, total))
    if not amostras:
        return {'load1_max': None, 'psi_cpu_some_avg10_max': None,
                'psi_cpu_some_us': None, 'amostras': 0}
    return {'load1_max': max(a[1] for a in amostras),
            'psi_cpu_some_avg10_max': max(a[2] for a in amostras),
            'psi_cpu_some_us': amostras[-1][3] - amostras[0][3],
            'amostras': len(amostras)}


# ── a linha e o veredito ────────────────────────────────────────────────────

def linha(n, medidas, log, lista, echo, amostras):
    a = analisa_log(log)
    c = controladores(lista)
    jsb, base = c.get(JSB, ''), c.get(BASE, '')
    msgs = conta_mensagens(echo)
    nav2 = int(medidas.get('nav2_tf_pronto', 0) or 0)
    controladores_ok = int(jsb == 'active' and base == 'active')
    ok = int(nav2 == 1 and controladores_ok == 1 and msgs > 0
             and a['switch_timeout'] == 0)
    saida = {
        'subida': n, 'subida_nominal': ok,
        'nav2_tf_pronto': nav2, 't_nav2_tf_s': medidas.get('t_nav2_tf_s', ''),
        'jsb_estado': jsb, 'base_estado': base,
        'controladores_ok': controladores_ok,
        'joint_states_msgs': msgs,
        'joint_states_janela_s': medidas.get('js_janela_s', ''),
        'limpeza_ok': int(medidas.get('limpeza_ok', 0) or 0),
        'shm_orfaos': int(medidas.get('shm_orfaos', 0) or 0),
        'limpeza_recuperada': int(medidas.get('limpeza_recuperada', 0) or 0),
    }
    saida.update(a)
    saida.update(resume_amostras(amostras))
    return {k: ('' if saida[k] is None else saida[k]) for k in COLUNAS}


def _int(l, k):
    try:
        return int(l.get(k, 0) or 0)
    except ValueError:
        return 0


def veredito(linhas, pedidas, interrompida):
    """O rótulo é da SUBIDA (a pergunta da bateria); o teardown vem escrito à
    parte, e limpeza recuperada nunca é chamada de limpeza nominal."""
    feitas = len(linhas)
    boas = sum(_int(l, 'subida_nominal') == 1 for l in linhas)
    falhas = [str(l['subida']) for l in linhas if _int(l, 'subida_nominal') != 1]
    ruins = [str(l['subida']) for l in linhas if _int(l, 'limpeza_ok') != 1]
    recup = [str(l['subida']) for l in linhas if _int(l, 'limpeza_recuperada') == 1]
    segv = [str(l['subida']) for l in linhas if _int(l, 'sigsegv_teardown') > 0]
    teardown = (f'teardown: {len(segv)} SIGSEGV do collision_monitor '
                f'(subidas {", ".join(segv) or "nenhuma"}); '
                + (f'{len(recup)} limpeza(s) recuperada(s) por fastdds shm clean '
                   f'(subidas {", ".join(recup)})' if recup
                   else 'nenhuma limpeza recuperada'))
    if interrompida or feitas < pedidas:
        return 'INCOMPLETA', (f'INCOMPLETA: {feitas} de {pedidas} subidas '
                              f'feitas; nominais {boas}/{feitas}; limpeza '
                              f'reprovada em {ruins or "nenhuma"}; {teardown}')
    if falhas or ruins:
        return 'INSTÁVEL', (f'INSTÁVEL: {boas}/{pedidas} subidas nominais; '
                            f'falharam as subidas {", ".join(falhas) or "nenhuma"}; '
                            f'limpeza reprovada em {ruins or "nenhuma"}; {teardown}')
    limite = f'{(1 - 0.05 ** (1 / pedidas)) * 100:.1f}'.replace('.', ',')
    return 'ESTÁVEL', (f'ESTÁVEL (subida): {boas}/{pedidas} subidas nominais. '
                       f'Isto NÃO é taxa zero: com 0 falhas em {pedidas}, o '
                       f'limite superior unilateral de 95% da taxa de falha é '
                       f'{limite}%. {teardown[0].upper() + teardown[1:]}.')


# ── CLI ─────────────────────────────────────────────────────────────────────

def _le(caminho):
    try:
        with open(caminho, errors='replace') as f:
            return f.read()
    except OSError:
        return ''


def _medidas(texto):
    m = {}
    for l in texto.splitlines():
        if '=' in l:
            k, v = l.split('=', 1)
            m[k.strip()] = v.strip()
    return m


def main(argv):
    if argv == ['cabecalho']:
        print(','.join(COLUNAS))
        return 0
    if len(argv) == 3 and argv[0] == 'linha':
        p = argv[1]
        l = linha(int(argv[2]), _medidas(_le(os.path.join(p, 'medidas.env'))),
                  _le(os.path.join(p, 'launch.log')),
                  _le(os.path.join(p, 'controladores.txt')),
                  _le(os.path.join(p, 'joint_states.txt')),
                  _le(os.path.join(p, 'amostras.txt')))
        print(','.join(str(l[k]) for k in COLUNAS))
        return 0
    if len(argv) == 2 and argv[0] == 'sinais':
        a = analisa_log(_le(argv[1]))
        print(a['sinais_fora_da_regra'], a['sigsegv_teardown'])
        return 0
    if len(argv) == 4 and argv[0] == 'veredito':
        with open(argv[1]) as f:
            linhas = list(csv.DictReader(f))
        v, texto = veredito(linhas, int(argv[2]), argv[3] == '1')
        print(texto)
        return 0 if v == 'ESTÁVEL' else 1
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
