#!/usr/bin/env python3
"""Remoção manual CONTROLADA dos segmentos Fast DDS que o `fastdds shm clean`
não reconhece — regra do dono de 2026-09-28, 14h30 (decisão 061 §2.3.2).

    shm_recupera.py inventaria <dir_shm> <saida.tsv>
    shm_recupera.py confere <dir_shm>
    shm_recupera.py remove <dir_shm> <inventario_antes.tsv> <inventario_depois.tsv>

O `fastdds shm clean` só reconhece como zumbi o segmento que ainda tem a trava
`_el`. Quando um participante morre por SIGSEGV, as portas dos OUTROS (que
saíram limpos e apagaram as próprias travas) ficam sem `_el` e sem dono — e a
ferramenta não as vê (medido: 5 de 34 removidos, 29 ficaram). Este módulo
remove SÓ esses, e só se TUDO fechar:

  · candidato = nome no inventário DEPOIS e não no ANTES da corrida;
  · nome `fastrtps_port<N>` ou `sem.fastrtps_port<N>_mutex`, e SEM o `_el`
    irmão (`fastrtps_port<N>_el`);
  · arquivo regular, não symlink (lstat, nunca seguido), do usuário atual,
    direto no diretório de SHM;
  · tamanho e mtime iguais aos do inventário DEPOIS — mudou, interrompe;
  · `fuser` vazio.

Um arquivo presente que falhe qualquer item interrompe tudo e NENHUM é
removido. A remoção é `os.unlink` do caminho exato, um a um, a partir da lista
— sem padrão de nome expandido pelo sistema.

"Nada vivo" é RECONFERIDO aqui dentro, imediatamente antes do primeiro unlink
(decisão 061; dono, 28-09 noite), lendo só o /proc — sem ROS e sem criar
participante DDS (o `ros2 node list` limpa portas com `_el` ao iniciar e
deixou de ser gate): nenhum processo ROS/Gazebo/Fast DDS/auxiliar, nenhuma
das marcas VALIDA_ETAPA4_MARCA e REPRO_CM_MARCA, nenhum `maps`/`fd` apontando
para segmento Fast DDS do diretório. Fora da conta: este processo e os seus
ancestrais (o wrapper que chama carrega a marca). `confere` roda só isso.
"""
import hashlib
import os
import re
import stat
import subprocess
import sys

PADROES = (re.compile(r'fastrtps_port\d+'),
           re.compile(r'sem\.fastrtps_port\d+_mutex'))
CABECALHO = 'nome\ttipo\tuid\ttamanho\tmtime_ns\tsha256'
VIVO = re.compile(
    r'(^|[ /])(ros2|gz|rviz2|_ros2_daemon|fastdds)( |$)|/opt/ros/[a-z]+/(lib|bin)/'
    r'|/install/[a-z_]+/lib/|ros2cli\.daemon|gzserver|collision_monitor'
    r'|repro_cm_sigsegv/auxiliar\.py')
MARCAS = ('VALIDA_ETAPA4_MARCA', 'REPRO_CM_MARCA')


def _tipo(st):
    if stat.S_ISLNK(st.st_mode):
        return 'symlink'
    if stat.S_ISREG(st.st_mode):
        return 'regular'
    return 'outro'


def _sha(caminho, st):
    if not stat.S_ISREG(st.st_mode):
        return '-'
    h = hashlib.sha256()
    with open(caminho, 'rb') as f:
        for bloco in iter(lambda: f.read(1 << 16), b''):
            h.update(bloco)
    return h.hexdigest()


def _fast(nome):
    return 'fastrtps' in nome or 'fast_datasharing' in nome


def inventaria(dir_shm, saida):
    """TODAS as entradas do diretório — não só as do Fast DDS: arquivo novo
    de qualquer tipo é "inesperado" e tem de aparecer."""
    linhas = [CABECALHO]
    for nome in sorted(os.listdir(dir_shm)):
        c = os.path.join(dir_shm, nome)
        st = os.lstat(c)
        linhas.append('\t'.join((nome, _tipo(st), str(st.st_uid),
                                 str(st.st_size), str(st.st_mtime_ns),
                                 _sha(c, st))))
    with open(saida, 'w') as f:
        f.write('\n'.join(linhas) + '\n')


def _le_inventario(caminho):
    with open(caminho) as f:
        linhas = f.read().splitlines()
    if not linhas or linhas[0] != CABECALHO:
        raise ValueError(f'inventário ilegível: {caminho}')
    saida = {}
    for l in linhas[1:]:
        nome, tipo, uid, tam, mtime, sha = l.split('\t')
        saida[nome] = (tipo, int(uid), int(tam), int(mtime), sha)
    return saida


def _fuser_vazio(caminho):
    r = subprocess.run(['fuser', caminho], capture_output=True, text=True)
    return not r.stdout.strip()


def _le(caminho):
    try:
        with open(caminho, 'rb') as f:
            return f.read()
    except OSError:
        return None


def _cadeia(proc_root, eu):
    """eu e os meus ancestrais, pelo campo ppid do /proc/<pid>/stat."""
    cadeia, pid = set(), eu
    while pid > 0 and pid not in cadeia:
        cadeia.add(pid)
        stat_ = _le(os.path.join(proc_root, str(pid), 'stat'))
        if stat_ is None:
            break
        pid = int(stat_.rsplit(b')', 1)[1].split()[1])
    return cadeia


def _segmento_fast(dir_shm, caminho):
    return (os.path.dirname(caminho) == os.path.abspath(dir_shm)
            and _fast(os.path.basename(caminho)))


def confere_nada_vivo(dir_shm, proc_root='/proc', eu=None):
    """Lista de problemas (vazia = nada vivo). Só lê o /proc."""
    eu = os.getpid() if eu is None else eu
    fora = _cadeia(proc_root, eu)
    problemas = []
    for nome in sorted(os.listdir(proc_root)):
        if not nome.isdigit() or int(nome) in fora:
            continue
        d = os.path.join(proc_root, nome)
        cmd = _le(os.path.join(d, 'cmdline'))
        if not cmd:
            continue  # sumiu no meio da leitura, ou thread de kernel
        cmd = cmd.replace(b'\0', b' ').decode(errors='replace').strip()
        if VIVO.search(cmd):
            problemas.append(f'processo vivo: pid {nome}: {cmd[:160]}')
        env = _le(os.path.join(d, 'environ')) or b''
        for marca in MARCAS:
            if any(v.startswith(marca.encode() + b'=') for v in env.split(b'\0')):
                problemas.append(f'marca viva: pid {nome} tem {marca}: {cmd[:120]}')
        maps = (_le(os.path.join(d, 'maps')) or b'').decode(errors='replace')
        for linha in maps.splitlines():
            campos = linha.split(None, 5)
            if len(campos) == 6 and _segmento_fast(dir_shm, campos[5].strip()):
                problemas.append(f'dono por mapeamento: pid {nome}: {campos[5].strip()}')
        try:
            fds = os.listdir(os.path.join(d, 'fd'))
        except OSError:
            fds = []
        for fd in fds:
            try:
                alvo = os.readlink(os.path.join(d, 'fd', fd))
            except OSError:
                continue
            if _segmento_fast(dir_shm, alvo):
                problemas.append(f'dono por fd: pid {nome}: {alvo}')
    return problemas


def remove(dir_shm, inventario_antes, inventario_depois, uid=None,
           proc_root='/proc', eu=None):
    """(ok, relato). ok=False: nada foi removido (ou sobrou algo no fim)."""
    uid = os.getuid() if uid is None else uid
    try:
        antes = _le_inventario(inventario_antes)
        depois = _le_inventario(inventario_depois)
    except (OSError, ValueError) as e:
        return False, f'inventário: {e}'
    presentes = sorted(os.listdir(dir_shm))
    candidatos = set(depois) - set(antes)
    problemas, alvos = [], []
    for nome in presentes:
        c = os.path.join(dir_shm, nome)
        st = os.lstat(c)
        if nome in antes:
            # Já existia antes da corrida: se é do sistema (lttng), fica
            # intocado; se é Fast DDS, é resíduo anterior — não é desta corrida.
            if _fast(nome):
                problemas.append(f'{nome}: segmento Fast DDS que já existia '
                                 'ANTES da corrida')
            continue
        if nome not in candidatos:
            problemas.append(f'{nome}: inesperado (ausente do inventário DEPOIS)')
            continue
        if not any(p.fullmatch(nome) for p in PADROES):
            problemas.append(f'{nome}: nome fora dos padrões '
                             'fastrtps_port<N> / sem.fastrtps_port<N>_mutex')
            continue
        porta = nome[len('sem.'):-len('_mutex')] if nome.startswith('sem.') else nome
        if os.path.lexists(os.path.join(dir_shm, porta + '_el')):
            problemas.append(f'{nome}: tem o _el irmão ({porta}_el) — é do '
                             'fastdds shm clean, não daqui')
            continue
        tipo = _tipo(st)
        if tipo == 'symlink':
            problemas.append(f'{nome}: symlink — nunca seguido nem removido')
            continue
        if tipo != 'regular':
            problemas.append(f'{nome}: não é arquivo regular ({tipo})')
            continue
        if st.st_uid != uid:
            problemas.append(f'{nome}: dono {st.st_uid}, esperado {uid}')
            continue
        inv = depois[nome]
        if (tipo, st.st_uid, st.st_size, st.st_mtime_ns) != inv[:4]:
            problemas.append(f'{nome}: mudou entre o inventário e a remoção '
                             f'(inventário {inv[:4]}, agora '
                             f'{(tipo, st.st_uid, st.st_size, st.st_mtime_ns)})')
            continue
        if not _fuser_vazio(c):
            problemas.append(f'{nome}: fuser não vazio — algum processo o segura')
            continue
        alvos.append(nome)
    if not problemas:
        # Reconferência de "nada vivo" IMEDIATAMENTE antes do primeiro unlink.
        problemas = confere_nada_vivo(dir_shm, proc_root=proc_root, eu=eu)
    if problemas:
        return False, ('INTERROMPIDO, nada removido:\n  '
                       + '\n  '.join(problemas))
    removidos = []
    for nome in alvos:
        os.unlink(os.path.join(dir_shm, nome))
        removidos.append(nome)
    sobra = sorted(n for n in os.listdir(dir_shm) if _fast(n))
    relato = ('removidos (' + str(len(removidos)) + '):\n  '
              + '\n  '.join(removidos or ['(nenhum)']))
    if sobra:
        return False, relato + '\nSOBROU depois da remoção:\n  ' + '\n  '.join(sobra)
    return True, relato + '\nrecontagem: 0 segmento Fast DDS'


def main(argv):
    if len(argv) == 3 and argv[0] == 'inventaria':
        inventaria(argv[1], argv[2])
        return 0
    if len(argv) == 2 and argv[0] == 'confere':
        problemas = confere_nada_vivo(argv[1])
        print('\n'.join(problemas) if problemas else 'nada vivo: nenhum processo, marca ou dono')
        return 1 if problemas else 0
    if len(argv) == 4 and argv[0] == 'remove':
        ok, relato = remove(argv[1], argv[2], argv[3])
        print(relato)
        return 0 if ok else 1
    print(__doc__, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
