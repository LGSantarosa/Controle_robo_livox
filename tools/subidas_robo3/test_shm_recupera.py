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
