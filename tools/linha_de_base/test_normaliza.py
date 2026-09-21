"""Testes do normalizador da linha de base (etapa 4, passo 0).

O plano da etapa 4 (§1, §10.2) prova que o robô 2 NÃO muda comparando os
parâmetros vivos no Gazebo com uma linha de base. Essa prova só vale se a
comparação for semântica: o texto do `ros2 param dump` varia em ordem de chave,
formato de float e parâmetros automáticos sem significado nenhum. E vale menos
que nada se o comparador engolir diferença de verdade — por isso metade destes
testes é sobre o que ele NÃO pode aceitar (bool ≠ número, permissão com valor
errado, nó sumido).

Nada aqui sobe ROS.
"""
import importlib.util
import os

import pytest

AQUI = os.path.dirname(os.path.abspath(__file__))


def _carrega(nome):
    spec = importlib.util.spec_from_file_location(
        f'linha_de_base_{nome}', os.path.join(AQUI, f'{nome}.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


nz = _carrega('normaliza')


def dump(no, params):
    """Um dump no formato do `ros2 param dump` (aninhado, com ros__parameters)."""
    return {no: {'ros__parameters': params}}


# ─── normalização ────────────────────────────────────────────────────────────

def test_achata_nome_aninhado_com_ponto():
    n = nz.normaliza(dump('/no', {'a': {'b': {'c': 1.5}}, 'd': 'x'}))
    assert n == {'/no': {'a.b.c': 1.5, 'd': 'x'}}


def test_exclui_automaticos_da_lista_escrita():
    n = nz.normaliza(dump('/no', {
        'use_sim_time': True,
        'start_type_description_service': True,
        'qos_overrides': {'/tf': {'publisher': {'depth': 100}}},
        'fica': 2,
    }))
    assert n == {'/no': {'fica': 2.0}}


def test_qos_overrides_so_pela_chave_completa():
    # `qos_overrides` no meio do nome é parâmetro de alguém, não o automático.
    n = nz.normaliza(dump('/no', {'plugin': {'qos_overrides': {'x': 1}}}))
    assert n == {'/no': {'plugin.qos_overrides.x': 1.0}}


def test_lista_de_exclusao_e_explicita():
    # A lista é o contrato do §10.2: mudar exige mudar este teste.
    assert nz.EXCLUIDOS_EXATOS == ('use_sim_time', 'start_type_description_service')
    assert nz.EXCLUIDOS_PREFIXO == ('qos_overrides.',)


def test_inteiro_e_float_viram_o_mesmo_numero():
    a = nz.normaliza(dump('/no', {'x': 1, 'v': [1, 2]}))
    b = nz.normaliza(dump('/no', {'x': 1.0, 'v': [1.0, 2.0]}))
    assert a == b
    assert nz.compara(a, b).diferencas == []


def test_bool_nao_e_numero():
    # Em Python True == 1. Na comparação, não pode valer.
    a = nz.normaliza(dump('/no', {'x': True}))
    b = nz.normaliza(dump('/no', {'x': 1}))
    r = nz.compara(a, b)
    assert [d.caminho for d in r.diferencas] == ['/no:x']
    assert not r.aprovado


def test_bool_em_lista_nao_e_numero():
    a = nz.normaliza(dump('/no', {'v': [True, False]}))
    b = nz.normaliza(dump('/no', {'v': [1, 0]}))
    assert not nz.compara(a, b).aprovado


def test_normalizar_de_novo_nao_muda_nada():
    # O normalizado vai para arquivo e é relido depois: tem de ser ponto fixo.
    n = nz.normaliza(dump('/no', {'a': {'b': 3}, 'c': [1, 2], 'd': False}))
    assert nz.normaliza(nz.como_dump(n)) == n


def test_arquivo_ida_e_volta(tmp_path):
    n = nz.normaliza(dump('/b', {'z': 1, 'a': {'x': 'texto'}, 'f': 0.1}))
    arq = tmp_path / 'n.yaml'
    nz.grava(n, str(arq))
    assert nz.le(str(arq)) == n
    # Chaves ordenadas no arquivo: diff de texto entre duas capturas é legível.
    texto = arq.read_text()
    assert texto.index('a.x') < texto.index('f') < texto.index('z')


# ─── comparação ──────────────────────────────────────────────────────────────

def test_entrou_saiu_mudou():
    a = {'/no': {'fica': 1.0, 'sai': 2.0, 'muda': 3.0}}
    b = {'/no': {'fica': 1.0, 'entra': 4.0, 'muda': 5.0}}
    r = nz.compara(a, b)
    assert {(d.caminho, d.tipo) for d in r.diferencas} == {
        ('/no:sai', 'saiu'), ('/no:entra', 'entrou'), ('/no:muda', 'mudou')}
    assert not r.aprovado


def test_no_inteiro_sumido_ou_novo_e_diferenca():
    a = {'/a': {'x': 1.0}, '/b': {'y': 1.0}}
    b = {'/a': {'x': 1.0}, '/c': {'y': 1.0}}
    r = nz.compara(a, b)
    assert {(d.caminho, d.tipo) for d in r.diferencas} == {
        ('/b', 'saiu'), ('/c', 'entrou')}


def test_permitida_com_caminho_e_valor_exatos_aprova():
    a = {'/path_follower': {'x': 1.0}}
    b = {'/path_follower': {'x': 1.0, 'avanco_para_choque': 0.28}}
    perm = [nz.Permitida('/path_follower:avanco_para_choque', 'entrou', depois=0.28)]
    r = nz.compara(a, b, perm)
    assert r.aprovado
    assert r.nao_autorizadas == [] and r.permissoes_sem_uso == []


def test_permitida_com_valor_errado_reprova():
    a = {'/path_follower': {}}
    b = {'/path_follower': {'avanco_para_choque': 0.30}}
    perm = [nz.Permitida('/path_follower:avanco_para_choque', 'entrou', depois=0.28)]
    r = nz.compara(a, b, perm)
    assert not r.aprovado
    assert [d.caminho for d in r.nao_autorizadas] == ['/path_follower:avanco_para_choque']


def test_permitida_nao_e_prefixo():
    # Caminho exato: permitir `x` não permite `x_outro`.
    a = {'/no': {}}
    b = {'/no': {'x_outro': 1.0}}
    r = nz.compara(a, b, [nz.Permitida('/no:x', 'entrou')])
    assert not r.aprovado


def test_permitida_com_tipo_errado_reprova():
    a = {'/no': {'x': 1.0}}
    b = {'/no': {'x': 2.0}}
    r = nz.compara(a, b, [nz.Permitida('/no:x', 'entrou')])
    assert not r.aprovado


def test_permissao_sem_uso_e_avisada_sem_reprovar():
    # Ex.: o footprint_padding declarado no passo 1 com o mesmo valor vivo não
    # produz diferença nenhuma — a permissão sobra, e isso tem de aparecer.
    a = {'/no': {'x': 1.0}}
    r = nz.compara(a, a, [nz.Permitida('/no:y', 'entrou')])
    assert r.aprovado
    assert [p.caminho for p in r.permissoes_sem_uso] == ['/no:y']


def test_diferenca_nao_autorizada_ao_lado_de_permitida_reprova():
    a = {'/no': {'x': 1.0}}
    b = {'/no': {'x': 1.0, 'y': 2.0, 'z': 3.0}}
    r = nz.compara(a, b, [nz.Permitida('/no:y', 'entrou', depois=2.0)])
    assert not r.aprovado
    assert [d.caminho for d in r.nao_autorizadas] == ['/no:z']


def test_permitidas_lidas_de_arquivo(tmp_path):
    arq = tmp_path / 'p.yaml'
    arq.write_text(
        "- caminho: '/path_follower:avanco_para_choque'\n"
        "  tipo: entrou\n"
        "  depois: 0.28\n")
    assert nz.le_permitidas(str(arq)) == [
        nz.Permitida('/path_follower:avanco_para_choque', 'entrou', depois=0.28)]


def test_permitida_com_tipo_invalido_e_erro(tmp_path):
    with pytest.raises(ValueError):
        nz.Permitida('/no:x', 'alterou')
