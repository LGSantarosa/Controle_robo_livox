#!/usr/bin/env python3
"""Fotos de processos, sinal só com identidade, e a prova de quem morreu.

Cópia da etapa 4 (tools/valida_etapa4/processos.py, evidência congelada), com
uma mudança: os externos são OPCIONAIS — a bancada da etapa 5 não os sobe, e
lista vazia não reprova. Etapa 5 (docs/PLANO_ETAPA5_ROBO3.md §3.3).

Identidade de processo = (pid, starttime): PID e PGID sozinhos podem ser
reusados. A marca é VALIDA_ETAPA4_MARCA=<pasta desta rodada> (o nome veio com
a cópia), herdada por todo processo que o wrapper lança.

    processos.py foto <saida.csv>
        Todo processo do usuário: pid, ppid, pgid, sid, starttime, estado,
        marcado, argv.

    processos.py marcados
        Vivos com a marca desta rodada. Sai 1 se houver algum (prova de limpeza).

    processos.py sinaliza_grupos <grupos.txt> <INT|KILL>
    processos.py vivos_grupos <grupos.txt>
        grupos.txt: PGID STARTTIME PAPEL. Classifica cada grupo ANTES de sinal:
          nosso        líder vivo (mesmo zumbi) com o STARTTIME registrado;
          nosso_orfao  líder morto, TODOS os membros com a marca desta rodada;
          reusado      líder vivo com STARTTIME diferente — não toca;
          orfao_alheio líder morto e algum membro SEM a marca — não toca;
          vazio        nenhum membro.
        Só "nosso" e "nosso_orfao" recebem sinal, e POR PID: cada membro é
        reconferido (mesmo PGID, mesmo starttime, marca) logo antes do kill.
        Membro sem marca num grupo "nosso" não recebe sinal e é listado.
        vivos_grupos imprime quantos membros sinalizáveis ainda vivem.

    processos.py sinaliza_marcados <INT|KILL>
        Quem tem a marca desta rodada, reconferido (marca + starttime) logo
        antes do kill.

    processos.py compara <antes.csv> <depois.csv> <grupos.txt> <externos.txt> <saida.yaml>
        O que o `sobe-robo3 --mata` fez, grupo a grupo: cada grupo registrado
        tem identidade na foto de antes (líder com o STARTTIME registrado, ou
        órfão com todos os membros marcados), tinha membros, todos marcados, e
        todos morreram; nenhum outro processo morreu (fora os de vida curta);
        cada externo (PID PGID STARTTIME NOME) vivo com o MESMO starttime.

"Vida curta": nasceu há menos de VIDA_CURTA_S antes da foto de antes (a
própria foto, um `ps` do wrapper) — pode sumir sem reprovar, mas é LISTADO.
"""
import csv
import os
import signal
import sys

import yaml

MARCA = 'VALIDA_ETAPA4_MARCA'
VIDA_CURTA_S = 5.0
HZ = os.sysconf('SC_CLK_TCK')


def _stat(pid):
    """(estado, ppid, pgid, sid, starttime) — campos depois do último ')'."""
    with open(f'/proc/{pid}/stat') as f:
        s = f.read()
    r = s[s.rindex(')') + 2:].split()
    return r[0], int(r[1]), int(r[2]), int(r[3]), int(r[19])


def _argv(pid):
    with open(f'/proc/{pid}/cmdline', 'rb') as f:
        return f.read().replace(b'\0', b' ').decode('utf-8', 'replace').strip()


def _marcado(pid, valor):
    if not valor:
        return False
    try:
        with open(f'/proc/{pid}/environ', 'rb') as f:
            env = f.read().split(b'\0')
    except OSError:
        return False
    return f'{MARCA}={valor}'.encode() in env


def _uptime_ticks():
    with open('/proc/uptime') as f:
        return float(f.read().split()[0]) * HZ


def foto(marca=None):
    """Lista de dicts, um por processo do usuário vivo (não zumbi)."""
    marca = os.environ.get(MARCA) if marca is None else marca
    uid = os.getuid()
    out = []
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        pid = int(d)
        try:
            if os.stat(f'/proc/{d}').st_uid != uid:
                continue
            estado, ppid, pgid, sid, start = _stat(pid)
            if estado in ('Z', 'X'):
                continue
            out.append({'pid': pid, 'ppid': ppid, 'pgid': pgid, 'sid': sid,
                        'starttime': start, 'estado': estado,
                        'marcado': _marcado(pid, marca), 'argv': _argv(pid)})
        except (OSError, ValueError, IndexError):
            continue   # morreu entre o listdir e a leitura
    return sorted(out, key=lambda p: p['pid'])


CAMPOS = ('pid', 'ppid', 'pgid', 'sid', 'starttime', 'estado', 'marcado', 'argv')


def grava(linhas, caminho):
    with open(caminho, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=CAMPOS)
        w.writeheader()
        w.writerows(linhas)


def le(caminho):
    with open(caminho, newline='') as f:
        out = []
        for r in csv.DictReader(f):
            for k in ('pid', 'ppid', 'pgid', 'sid', 'starttime'):
                r[k] = int(r[k])
            r['marcado'] = r['marcado'] == 'True'
            out.append(r)
        return out


def le_grupos(caminho):
    """[(pgid, starttime, papel)] — o formato do registro do sobe-robo3."""
    out = []
    with open(caminho) as f:
        for linha in f:
            if linha.strip():
                pgid, start, papel = linha.split()
                out.append((int(pgid), int(start), papel))
    return out


def le_externos(caminho):
    """[(pid, pgid, starttime, nome)]."""
    out = []
    with open(caminho) as f:
        for linha in f:
            if linha.strip():
                pid, pgid, start, nome = linha.split()
                out.append((int(pid), int(pgid), int(start), nome))
    return out


def classifica(pgid, start, processos):
    """(classe, membros) de um grupo registrado, sobre uma foto (lista de dicts).

    Classe: nosso, nosso_orfao, reusado, orfao_alheio, vazio (ver o topo).
    """
    membros = [p for p in processos if p['pgid'] == pgid]
    lider = [p for p in processos if p['pid'] == pgid]
    if lider:
        return ('nosso' if lider[0]['starttime'] == start else 'reusado'), membros
    if not membros:
        return 'vazio', []
    return ('nosso_orfao' if all(p['marcado'] for p in membros) else 'orfao_alheio'), membros


def _foto_com_zumbis(marca):
    """Como foto(), mas com zumbis: líder zumbi ainda segura o PID."""
    uid, out = os.getuid(), []
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            if os.stat(f'/proc/{d}').st_uid != uid:
                continue
            estado, ppid, pgid, sid, start = _stat(int(d))
            out.append({'pid': int(d), 'pgid': pgid, 'starttime': start, 'estado': estado,
                        'marcado': estado not in ('Z', 'X') and _marcado(int(d), marca)})
        except (OSError, ValueError, IndexError):
            continue
    return out


def _mata_se_ainda_for(p, sig, marca, exige_pgid=None):
    """Reconfere pid + starttime + marca (+ pgid) logo antes do sinal."""
    try:
        estado, _, pgid, _, start = _stat(p['pid'])
    except (OSError, ValueError, IndexError):
        return 'sumiu'
    if estado in ('Z', 'X'):
        return 'zumbi'
    if start != p['starttime'] or (exige_pgid is not None and pgid != exige_pgid):
        return 'outro processo'
    if not _marcado(p['pid'], marca):
        return 'sem marca'
    try:
        os.kill(p['pid'], sig)
    except ProcessLookupError:
        return 'sumiu'
    return 'sinalizado'


SINAIS = {'INT': signal.SIGINT, 'KILL': signal.SIGKILL}


def sinaliza_grupos(grupos, sig, marca, log=print):
    """Sinaliza, por PID, os membros marcados dos grupos com identidade."""
    vivos = 0
    for pgid, start, papel in grupos:
        foto_ = _foto_com_zumbis(marca)
        classe, membros = classifica(pgid, start, foto_)
        vivos_membros = [p for p in membros if p['estado'] not in ('Z', 'X')]
        if classe not in ('nosso', 'nosso_orfao'):
            log(f'{papel} pgid={pgid}: {classe} — não sinalizado '
                f'({len(vivos_membros)} membro(s) vivo(s))')
            continue
        for p in vivos_membros:
            if sig is None:
                vivos += p['marcado']
                continue
            r = _mata_se_ainda_for(p, sig, marca, exige_pgid=pgid)
            log(f'{papel} pgid={pgid} ({classe}) pid={p["pid"]}: {r}')
    return vivos


def sinaliza_marcados(sig, marca, log=print):
    for p in foto(marca):
        if p['marcado'] and p['pid'] != os.getpid():
            log(f'marcado pid={p["pid"]} pgid={p["pgid"]} {p["argv"][:120]}: '
                f'{_mata_se_ainda_for(p, sig, marca)}')


def compara(antes, depois, grupos, externos, agora_ticks_antes):
    """Veredito puro (testável sem processo nenhum). Devolve dict."""
    ident = lambda p: (p['pid'], p['starttime'])  # noqa: E731
    vivos_depois = {ident(p) for p in depois}
    por_grupo, nossos = [], set()
    for pgid, start, papel in grupos:
        classe, membros = classifica(pgid, start, antes)
        vivos = [p for p in membros if ident(p) in vivos_depois]
        g = {'pgid': pgid, 'starttime': start, 'papel': papel, 'classe': classe,
             'membros_antes': len(membros),
             'membros_sem_marca': [p['pid'] for p in membros if not p['marcado']],
             'membros_vivos_depois': [p['pid'] for p in vivos]}
        g['ok'] = (classe in ('nosso', 'nosso_orfao') and bool(membros)
                   and not g['membros_sem_marca'] and not vivos)
        por_grupo.append(g)
        if classe in ('nosso', 'nosso_orfao'):
            nossos.update(ident(p) for p in membros)
    sumiram = [p for p in antes if ident(p) not in vivos_depois]
    fora = [p for p in sumiram if ident(p) not in nossos]
    curtos = [p for p in fora
              if agora_ticks_antes - p['starttime'] < VIDA_CURTA_S * HZ]
    fora_longos = [p for p in fora if p not in curtos]
    ext = []
    for pid, pgid, start, nome in externos:
        ext.append({'nome': nome, 'pid': pid, 'pgid': pgid, 'starttime': start,
                    'vivo_depois': (pid, start) in vivos_depois,
                    'antes': (pid, start) in {ident(p) for p in antes}})
    itens = {
        'cada_grupo_registrado_valido_e_morto': (
            bool(por_grupo) and all(g['ok'] for g in por_grupo)),
        'nenhum_outro_processo_morreu': not fora_longos,
        'externos_estavam_vivos_antes': all(e['antes'] for e in ext),
        'externos_vivos_depois_mesmo_starttime': all(e['vivo_depois'] for e in ext),
    }
    return {
        'veredito': 'APROVADO' if all(itens.values()) else 'REPROVADO',
        'itens': itens,
        'grupos': por_grupo,
        'outros_que_morreram': fora_longos,
        'vida_curta_que_sumiu': curtos,
        'externos': ext,
    }


def main(argv):
    if len(argv) >= 2 and argv[0] == 'foto':
        grava(foto(), argv[1])
        return 0
    if argv and argv[0] == 'marcados':
        m = [p for p in foto() if p['marcado'] and p['pid'] != os.getpid()]
        for p in m:
            print(f"{p['pid']} pgid={p['pgid']} {p['argv'][:160]}")
        return 1 if m else 0
    marca = os.environ.get(MARCA)
    if len(argv) == 3 and argv[0] == 'sinaliza_grupos' and argv[2] in SINAIS:
        if not marca:
            print(f'{MARCA} ausente — não sinalizo nada')
            return 2
        sinaliza_grupos(le_grupos(argv[1]), SINAIS[argv[2]], marca)
        return 0
    if len(argv) == 2 and argv[0] == 'vivos_grupos':
        print(sinaliza_grupos(le_grupos(argv[1]), None, marca, log=lambda _: None))
        return 0
    if len(argv) == 2 and argv[0] == 'sinaliza_marcados' and argv[1] in SINAIS:
        if not marca:
            print(f'{MARCA} ausente — não sinalizo nada')
            return 2
        sinaliza_marcados(SINAIS[argv[1]], marca)
        return 0
    if len(argv) == 6 and argv[0] == 'compara':
        antes, depois = le(argv[1]), le(argv[2])
        # O instante da foto de antes: o maior starttime nela é um limite
        # inferior honesto (a própria foto roda depois de todos).
        t_antes = max(p['starttime'] for p in antes) if antes else _uptime_ticks()
        r = compara(antes, depois, le_grupos(argv[3]), le_externos(argv[4]), t_antes)
        with open(argv[5], 'w') as f:
            yaml.safe_dump(r, f, allow_unicode=True, sort_keys=False)
        print(f"{r['veredito']}: " + ', '.join(
            f"{k}={'ok' if v else 'FALHOU'}" for k, v in r['itens'].items()))
        return 0 if r['veredito'] == 'APROVADO' else 1
    print(__doc__)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
