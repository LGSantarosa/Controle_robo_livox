#!/usr/bin/env python3
"""A remoção manual controlada dos segmentos Fast DDS que o `fastdds shm clean`
não reconhece (regra do dono, 2026-09-28, 14h30; decisão 061 §2.3.2).

🔴 NASCE VERMELHO: `shm_recupera.py` ainda não existe.

Contrato:
  · candidatos = inventário DEPOIS − inventário ANTES da corrida, por nome;
  · só `fastrtps_port<N>` e `sem.fastrtps_port<N>_mutex`, arquivo regular, não
    symlink, do usuário atual, direto no diretório de SHM, SEM o `_el` irmão;
  · tamanho e mtime iguais aos do inventário; `fuser` vazio;
  · TUDO validado antes de apagar QUALQUER um: um falhou, nenhum sai;
  · remoção por nome, um a um, sem glob; recontagem no fim.
"""
import importlib.util
import os
import subprocess
import sys

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))
MOD = os.path.join(AQUI, 'shm_recupera.py')


@pytest.fixture(scope='module')
def shm_recupera():
    if not os.path.exists(MOD):
        pytest.fail(f'{MOD} ainda não existe — regra de 28-09, 14h30: o teste '
                    'vem antes do código.')
    spec = importlib.util.spec_from_file_location('shm_recupera', MOD)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _cria(shm, *nomes):
    for n in nomes:
        (shm / n).write_bytes(b'x' * 32)


def _inventario(m, shm, destino):
    m.inventaria(str(shm), str(destino))
    return destino


def _prepara(tmp_path, m, restantes, antes=()):
    """`antes` existia antes da corrida; `restantes` apareceu depois."""
    shm = tmp_path / 'shm'
    shm.mkdir()
    _cria(shm, *antes)
    inv_antes = _inventario(m, shm, tmp_path / 'antes.tsv')
    _cria(shm, *restantes)
    inv_depois = _inventario(m, shm, tmp_path / 'depois.tsv')
    return shm, inv_antes, inv_depois


def _remove(m, shm, inv_antes, inv_depois, **kw):
    return m.remove(str(shm), str(inv_antes), str(inv_depois), **kw)


PAR = ('fastrtps_port7001', 'sem.fastrtps_port7001_mutex')


def test_remove_os_restantes_sem_el(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert ok, relato
    assert list(shm.iterdir()) == []
    assert all(n in relato for n in PAR)


def test_o_inventario_tem_sha256_tamanho_mtime_e_dono(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    texto = d.read_text()
    assert 'sha256' in texto.splitlines()[0]
    assert all(n in texto for n in PAR)


def test_arquivo_que_ja_existia_antes_nao_e_candidato(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, ('fastrtps_port7003',),
                         antes=('fastrtps_port7001',))
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert not ok and 'fastrtps_port7001' in relato
    assert sorted(p.name for p in shm.iterdir()) == ['fastrtps_port7001',
                                                     'fastrtps_port7003']


def test_arquivo_que_apareceu_depois_do_inventario_interrompe(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    _cria(shm, 'fastrtps_port7009')
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert not ok and 'fastrtps_port7009' in relato
    assert len(list(shm.iterdir())) == 3, 'um inválido e NENHUM sai'


def test_mudanca_entre_inventario_e_remocao_interrompe(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    (shm / 'fastrtps_port7001').write_bytes(b'y' * 64)
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert not ok and 'mudou' in relato
    assert len(list(shm.iterdir())) == 2


def test_com_el_irmao_nao_e_removido(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera,
                         PAR + ('fastrtps_port7001_el',))
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert not ok and 'tem o _el irmão' in relato, relato
    assert len(list(shm.iterdir())) == 3


@pytest.mark.parametrize('nome', ('fastrtps_8daeb86a2eaebf00',
                                  'fastrtps_port7001_el', 'fastrtps_portX',
                                  'sem.fastrtps_port7001', 'outro_arquivo',
                                  'sem.fastrtps_port7001_mutex_x'))
def test_nome_fora_dos_dois_padroes_interrompe(tmp_path, shm_recupera, nome):
    shm, a, d = _prepara(tmp_path, shm_recupera, (nome,))
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert not ok and nome in relato
    assert (shm / nome).exists()


def test_symlink_nunca_e_seguido_nem_removido(tmp_path, shm_recupera):
    alvo = tmp_path / 'alvo_precioso'
    alvo.write_text('não pode sumir')
    shm, a, d = _prepara(tmp_path, shm_recupera, ())
    (shm / 'fastrtps_port7001').symlink_to(alvo)
    d = _inventario(shm_recupera, shm, tmp_path / 'depois2.tsv')
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert not ok and 'symlink — nunca seguido' in relato, relato
    assert alvo.read_text() == 'não pode sumir'
    assert (shm / 'fastrtps_port7001').is_symlink()


def test_dono_diferente_interrompe(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    ok, relato = _remove(shm_recupera, shm, a, d, uid=os.getuid() + 1)
    assert not ok and 'dono' in relato
    assert len(list(shm.iterdir())) == 2


def test_arquivo_aberto_por_processo_interrompe(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    p = subprocess.Popen([sys.executable, '-c',
                          'import sys,time; f=open(sys.argv[1]); '
                          'print("ok", flush=True); time.sleep(30)',
                          str(shm / 'fastrtps_port7001')],
                         stdout=subprocess.PIPE, text=True)
    try:
        p.stdout.readline()
        ok, relato = _remove(shm_recupera, shm, a, d)
        assert not ok and 'fuser' in relato
        assert len(list(shm.iterdir())) == 2
    finally:
        p.kill()
        p.wait()


def test_subdiretorio_nao_e_arquivo_regular(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, ())
    (shm / 'fastrtps_port7001').mkdir()
    d = _inventario(shm_recupera, shm, tmp_path / 'depois2.tsv')
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert not ok and (shm / 'fastrtps_port7001').is_dir()


def test_nada_a_remover_e_ok_e_nao_toca_em_nada(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, ())
    ok, relato = _remove(shm_recupera, shm, a, d)
    assert ok


def test_o_codigo_nao_usa_glob_nem_rmtree(shm_recupera):
    texto = open(MOD).read()
    for proibido in ('glob', 'rmtree', 'shell=True', 'os.system'):
        assert proibido not in texto, proibido


def test_cli(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    r = subprocess.run([sys.executable, MOD, 'remove', str(shm), str(a),
                        str(d)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout + r.stderr
    assert list(shm.iterdir()) == []
    _cria(shm, 'outro_arquivo')
    d2 = _inventario(shm_recupera, shm, tmp_path / 'd2.tsv')
    r = subprocess.run([sys.executable, MOD, 'remove', str(shm), str(a),
                        str(d2)], capture_output=True, text=True)
    assert r.returncode == 1


# ── Reconferência interna de "nada vivo" (dono, 28-09 noite) ──────────────
# O remove() reconfere, imediatamente antes do primeiro unlink e SEM ROS nem
# participante DDS: processos ROS/Gazebo/Fast DDS/auxiliar, as duas marcas
# (VALIDA_ETAPA4_MARCA e REPRO_CM_MARCA) e donos por mapeamento ou fd. O
# /proc falso é só parâmetro de função; a CLI sempre lê o /proc real.

EU = 5000


def _proc(tmp_path, processos=(), eu=EU, ancestrais=(4000, 1)):
    """processos: (pid, cmdline, environ:dict, maps:list[str], fds:list[str], ppid)."""
    raiz = tmp_path / 'proc'
    raiz.mkdir(exist_ok=True)
    cadeia = [(eu, ancestrais[0])] + list(zip(ancestrais, list(ancestrais[1:]) + [0]))
    for pid, ppid in cadeia:
        _proc_um(raiz, pid, 'bash wrapper', {'VALIDA_ETAPA4_MARCA': '/x',
                                             'REPRO_CM_MARCA': '/y'}, [], [], ppid)
    for pid, cmd, env, maps, fds, ppid in processos:
        _proc_um(raiz, pid, cmd, env, maps, fds, ppid)
    return raiz


def _proc_um(raiz, pid, cmd, env, maps, fds, ppid):
    d = raiz / str(pid)
    d.mkdir()
    (d / 'cmdline').write_bytes(cmd.replace(' ', '\0').encode() + b'\0')
    (d / 'environ').write_bytes(b''.join(f'{k}={v}\0'.encode() for k, v in env.items()))
    (d / 'stat').write_text(f'{pid} (x) S {ppid} 0 0 0')
    (d / 'maps').write_text(''.join(f'7f00-7f01 rw-s 0 00:1a 9 {m}\n' for m in maps))
    (d / 'fd').mkdir()
    for i, alvo in enumerate(fds):
        (d / 'fd' / str(i + 3)).symlink_to(alvo)


def _remove_proc(m, shm, a, d, raiz):
    return m.remove(str(shm), str(a), str(d), proc_root=str(raiz), eu=EU)


def test_proc_limpo_remove(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path, [(6001, 'bash -c ls', {}, [], [], 1)])
    ok, relato = _remove_proc(shm_recupera, shm, a, d, raiz)
    assert ok, relato
    assert list(shm.iterdir()) == []


@pytest.mark.parametrize('cmd', (
    '/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor --ros-args',
    'gdb -batch -x g.cmd /opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor',
    'python3 /r/tools/repro_cm_sigsegv/auxiliar.py',
    '/usr/bin/python3 /opt/ros/jazzy/bin/ros2 node list',
    'python3 -c from ros2cli.daemon.daemonize import main',
    'fastdds shm clean',
    'gz sim -s -r mundo.sdf',
    '/r/install/robot_base/lib/robot_base/placa_simulada --ros-args'))
def test_processo_vivo_interrompe_e_nada_sai(tmp_path, shm_recupera, cmd):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path, [(6001, cmd, {}, [], [], 1)])
    ok, relato = _remove_proc(shm_recupera, shm, a, d, raiz)
    assert not ok and 'processo vivo' in relato and '6001' in relato, relato
    assert len(list(shm.iterdir())) == 2


@pytest.mark.parametrize('marca', ('VALIDA_ETAPA4_MARCA', 'REPRO_CM_MARCA'))
def test_marca_viva_fora_da_cadeia_interrompe(tmp_path, shm_recupera, marca):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path, [(6001, 'sleep 30', {marca: '/base'}, [], [], 1)])
    ok, relato = _remove_proc(shm_recupera, shm, a, d, raiz)
    assert not ok and 'marca viva' in relato and marca in relato, relato
    assert len(list(shm.iterdir())) == 2


def test_marca_na_propria_cadeia_de_ancestrais_nao_conta(tmp_path, shm_recupera):
    """O wrapper que chama carrega a marca: ele e os ancestrais não são resíduo."""
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path)
    ok, relato = _remove_proc(shm_recupera, shm, a, d, raiz)
    assert ok, relato


def test_segmento_mapeado_interrompe(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path, [(6001, 'sleep 30', {},
                             [str(shm / 'fastrtps_port7001')], [], 1)])
    ok, relato = _remove_proc(shm_recupera, shm, a, d, raiz)
    assert not ok and 'dono' in relato and '6001' in relato, relato
    assert len(list(shm.iterdir())) == 2


def test_segmento_com_fd_aberto_interrompe(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path, [(6001, 'sleep 30', {}, [],
                             [str(shm / 'sem.fastrtps_port7001_mutex')], 1)])
    ok, relato = _remove_proc(shm_recupera, shm, a, d, raiz)
    assert not ok and 'dono' in relato and '6001' in relato, relato
    assert len(list(shm.iterdir())) == 2


def test_dono_de_segmento_fast_dds_fora_dos_alvos_tambem_interrompe(tmp_path, shm_recupera):
    """Alguém segurando QUALQUER segmento Fast DDS do diretório = algo vivo."""
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path, [(6001, 'sleep 30', {},
                             [str(tmp_path / 'shm' / 'fastrtps_port9999')], [], 1)])
    ok, relato = _remove_proc(shm_recupera, shm, a, d, raiz)
    assert not ok and 'dono' in relato, relato


def test_processo_que_some_durante_a_leitura_e_ignorado(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path)
    (raiz / '6001').mkdir()          # diretório sem cmdline: sumiu no meio
    ok, relato = _remove_proc(shm_recupera, shm, a, d, raiz)
    assert ok, relato


def test_confere_so_relata_e_nao_remove(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    raiz = _proc(tmp_path, [(6001, 'fastdds shm clean', {}, [], [], 1)])
    problemas = shm_recupera.confere_nada_vivo(str(shm), proc_root=str(raiz), eu=EU)
    assert problemas and '6001' in problemas[0]
    (tmp_path / 'limpo').mkdir()
    raiz_limpa = _proc(tmp_path / 'limpo')
    assert shm_recupera.confere_nada_vivo(str(shm), proc_root=str(raiz_limpa), eu=EU) == []
    assert len(list(shm.iterdir())) == 2


def test_confere_cli_le_o_proc_real(tmp_path, shm_recupera):
    shm, a, d = _prepara(tmp_path, shm_recupera, PAR)
    r = subprocess.run([sys.executable, MOD, 'confere', str(shm)],
                       capture_output=True, text=True)
    assert r.returncode in (0, 1), r.stdout + r.stderr
    assert ('nada vivo' in r.stdout) == (r.returncode == 0), r.stdout
    assert len(list(shm.iterdir())) == 2


def test_nao_chama_ros_nem_cria_participante(shm_recupera):
    """Nenhum participante DDS: nada de rclpy, e o ÚNICO subprocesso é o fuser."""
    import re
    texto = open(MOD).read()
    assert 'rclpy' not in texto
    chamadas = re.findall(r'subprocess\.\w+\(\s*([^,)]*)', texto)
    assert chamadas and all(c.startswith("['fuser'") for c in chamadas), chamadas
