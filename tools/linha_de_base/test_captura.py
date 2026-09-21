"""Testes da captura da linha de base (etapa 4, passo 0).

Duas camadas. A lógica do grafo (duplicado, faltando, sobrando, oculto,
estabilidade) é pura e testada sem ROS. A captura de ponta a ponta roda contra
nós fingidos em subprocessos, num ROS_DOMAIN_ID próprio e só em localhost —
para não conversar com nenhuma pilha que esteja de pé neste PC.
"""
import importlib.util
from types import SimpleNamespace
import os
import signal
import subprocess
import sys
import time

import pytest
import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))


def _carrega(nome):
    spec = importlib.util.spec_from_file_location(
        f'linha_de_base_{nome}', os.path.join(AQUI, f'{nome}.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


cap = _carrega('captura')

GC, LC = '/global_costmap/global_costmap', '/local_costmap/local_costmap'


# ─── lógica pura ─────────────────────────────────────────────────────────────

def test_nome_completo():
    assert cap.nome_completo('/', 'amcl') == '/amcl'
    assert cap.nome_completo('/global_costmap', 'global_costmap') == GC


def test_oculto_so_pelo_ultimo_componente():
    assert cap.oculto('/_linha_de_base')
    assert cap.oculto('/ns/_x')
    assert not cap.oculto('/_ns/x')


def test_avalia_grafo():
    g = cap.avalia_grafo(['/a', '/b', '/b', '/x', '/_linha_de_base'], ['/a', '/b', '/c'])
    assert g['visiveis'] == ['/a', '/b', '/x']
    assert g['duplicados'] == ['/b']
    assert g['faltando'] == ['/c']
    assert g['sobrando'] == ['/x']


def test_pronto_exige_tudo():
    g = cap.avalia_grafo(['/a', '/b'], ['/a', '/b'])
    assert cap.pronto(g, {'/a': 'active', '/b': 'responde'})
    assert not cap.pronto(g, {'/a': 'inactive', '/b': 'responde'})
    assert not cap.pronto(g, {'/a': 'active', '/b': 'sem_resposta'})
    assert not cap.pronto(g, {'/a': 'active'})
    assert not cap.pronto(cap.avalia_grafo(['/a', '/b', '/b'], ['/a', '/b']),
                          {'/a': 'active', '/b': 'responde'})


def test_estavel_exige_n_iguais_seguidos():
    assert not cap.estavel([1, 1], 3)
    assert cap.estavel([2, 1, 1, 1], 3)
    assert not cap.estavel([1, 1, 2], 3)
    assert not cap.estavel([1, 2, 1], 3)  # spawner que aparece e some


def test_esperados_exige_nome_completo_e_unico(tmp_path):
    arq = tmp_path / 'e.yaml'
    arq.write_text('nos: [/b, /a]\n')
    assert cap.le_esperados(str(arq)) == ['/a', '/b']
    arq.write_text('nos: [amcl]\n')
    with pytest.raises(ValueError):
        cap.le_esperados(str(arq))
    arq.write_text('nos: [/a, /a]\n')
    with pytest.raises(ValueError):
        cap.le_esperados(str(arq))


def test_grafo_somente_exato_e_volatil_com_cardinalidade(tmp_path):
    arq = tmp_path / 'e.yaml'
    arq.write_text(yaml.safe_dump({
        'nos': ['/a'],
        'grafo_somente': {
            'exatos': ['/aux'],
            'volateis': [{'padrao': r'^/listener_[0-9a-f]+$',
                          'quantidade': 2, 'alias': '/listener_<hex>'}],
        }}))
    cfg = cap.le_configuracao(str(arq))
    g = cap.avalia_grafo(['/a', '/aux', '/listener_12', '/listener_ab'],
                         cfg['nos'], cfg['grafo_somente'])
    assert not g['faltando'] and not g['sobrando'] and not g['volateis_invalidos']
    assert g['somente_grafo'] == ['/aux', '/listener_12', '/listener_ab']
    assert set(g['aliases'].values()) == {'/listener_<hex>'}
    estados = {'/a': 'responde', '/aux': 'presente_sem_parametros',
               '/listener_12': 'presente_sem_parametros',
               '/listener_ab': 'presente_sem_parametros'}
    assert cap.pronto(g, estados)

    g = cap.avalia_grafo(['/a', '/aux', '/listener_12'],
                         cfg['nos'], cfg['grafo_somente'])
    assert g['volateis_invalidos'] and not cap.pronto(g, estados)


@pytest.mark.parametrize('padrao', ['/listener_.*$', '^/listener_.*', '['])
def test_regra_volatil_exige_regex_valida_e_ancorada(tmp_path, padrao):
    arq = tmp_path / 'e.yaml'
    arq.write_text(yaml.safe_dump({
        'grafo_somente': {'volateis': [
            {'padrao': padrao, 'quantidade': 1, 'alias': '/listener_<id>'}]}}))
    with pytest.raises(ValueError):
        cap.le_configuracao(str(arq))


def test_nome_parecido_mas_fora_da_regex_continua_sobrando():
    gs = {'exatos': [], 'volateis': [
        {'padrao': r'^/listener_[0-9a-f]+$', 'quantidade': 1,
         'alias': '/listener_<hex>'}]}
    g = cap.avalia_grafo(['/a', '/listener_ab', '/listener_intruso'], ['/a'], gs)
    assert g['sobrando'] == ['/listener_intruso']


def test_regra_volatil_nao_pode_engolir_no_fixo(tmp_path):
    arq = tmp_path / 'e.yaml'
    arq.write_text(yaml.safe_dump({
        'nos': ['/listener_ab'],
        'grafo_somente': {'volateis': [
            {'padrao': r'^/listener_[0-9a-f]+$',
             'quantidade': 1, 'alias': '/listener_<hex>'}]}}))
    with pytest.raises(ValueError, match='nó fixo'):
        cap.le_configuracao(str(arq))


def test_padding_ausente_e_motivo():
    n = {GC: {'footprint_padding': 0.01}, LC: {}}
    assert cap.resumo_padding(n) == {GC: 0.01, LC: 'AUSENTE'}
    g = cap.avalia_grafo([GC, LC], [GC, LC])
    m = cap.motivos(g, {GC: 'active', LC: 'active'}, True, {}, cap.resumo_padding(n))
    assert m and 'footprint_padding' in m[0]


# ─── ponta a ponta contra nós fingidos ───────────────────────────────────────

@pytest.fixture
def ambiente():
    env = dict(os.environ)
    env['ROS_DOMAIN_ID'] = str(180 + os.getpid() % 40)
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    env.pop('ROS_LOCALHOST_ONLY', None)
    procs = []

    def sobe(*args):
        p = subprocess.Popen([sys.executable, os.path.join(AQUI, 'nos_fingidos.py'), *args],
                             env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        procs.append(p)
        return p

    def captura(esperados, pasta, prazo):
        arq = pasta / 'esperados.yaml'
        config = esperados if isinstance(esperados, dict) else {'nos': esperados}
        arq.write_text(yaml.safe_dump(config))
        r = subprocess.run([sys.executable, os.path.join(AQUI, 'captura.py'),
                            str(arq), str(pasta / 'saida'), '--prazo', str(prazo),
                            '--intervalo', '0.3', '--estavel', '3'],
                           env=env, capture_output=True, text=True, timeout=prazo + 60)
        with open(pasta / 'saida' / 'resumo.yaml') as f:
            return r.returncode, yaml.safe_load(f), pasta / 'saida'

    yield sobe, captura
    for p in procs:
        p.send_signal(signal.SIGINT)
    for p in procs:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()


def test_captura_aprovada_com_tudo_ativo(tmp_path, ambiente):
    sobe, captura = ambiente
    sobe('--comum', '/ns_a/comum', '--ciclo', f'{GC}:ativo', '--ciclo', f'{LC}:ativo')
    rc, r, saida = captura(['/ns_a/comum', GC, LC], tmp_path, prazo=20)
    assert r['veredito'] == 'APROVADO', r['motivos']
    assert rc == 0
    # O nó oculto da própria ferramenta não virou "sobrando".
    assert r['nos_visiveis'] == 3
    assert r['footprint_padding'] == {GC: 0.01, LC: 0.01}

    norm = yaml.safe_load((saida / 'parametros_normalizados.yaml').read_text())
    assert norm['/ns_a/comum'] == {'a.b': 1.0, 'flag': True, 'lista': [1.0, 2.0]}
    assert norm[GC]['footprint_padding'] == 0.01 and norm[LC]['footprint_padding'] == 0.01
    # O bruto guarda o tipo como veio (int continua int) e os automáticos.
    bruto = yaml.safe_load((saida / 'parametros_brutos.yaml').read_text())
    assert bruto['/ns_a/comum']['ros__parameters']['a.b'] == 1
    assert 'use_sim_time' in bruto['/ns_a/comum']['ros__parameters']
    # Só captura depois de 3 consultas PRONTAS seguidas (--estavel 3).
    linhas = (saida / 'consultas.csv').read_text().splitlines()[1:]
    prontas = [ln.endswith('True') for ln in linhas]
    assert prontas[-3:] == [True, True, True]


def test_captura_aceita_auxiliares_sem_parametros_e_normaliza_nome(
        tmp_path, ambiente):
    sobe, captura = ambiente
    sobe('--comum', '/comum', '--grafo', '/aux_fixo',
         '--grafo', '/listener_12', '--grafo', '/listener_ab')
    config = {
        'nos': ['/comum'],
        'grafo_somente': {
            'exatos': ['/aux_fixo'],
            'volateis': [{'padrao': r'^/listener_[0-9a-f]+$',
                          'quantidade': 2, 'alias': '/listener_<hex>'}],
        }}
    rc, r, saida = captura(config, tmp_path, prazo=20)
    assert rc == 0 and r['veredito'] == 'APROVADO', r['motivos']
    assert r['nos_esperados'] == 4 and r['nos_com_parametros'] == 1
    normalizado = (saida / 'grafo_normalizado.txt').read_text().splitlines()
    assert normalizado == ['/aux_fixo', '/comum', '/listener_<hex> x2']
    params = yaml.safe_load((saida / 'parametros_normalizados.yaml').read_text())
    assert set(params) == {'/comum'}


def test_nao_ativo_e_sobrando_reprovam_mas_o_dado_fica(tmp_path, ambiente):
    sobe, captura = ambiente
    sobe('--ciclo', f'{GC}:ativo', '--ciclo', f'{LC}:inativo', '--comum', '/intruso')
    rc, r, saida = captura([GC, LC], tmp_path, prazo=4)
    assert rc == 1 and r['veredito'] == 'REPROVADO'
    texto = ' '.join(r['motivos'])
    assert '/intruso' in texto and LC in texto
    assert (saida / 'parametros_normalizados.yaml').exists()


def test_duplicado_reprova_sem_consultar_parametros(tmp_path, ambiente):
    sobe, captura = ambiente
    sobe('--ciclo', f'{GC}:ativo')
    sobe('--ciclo', f'{GC}:ativo')
    time.sleep(1.0)
    rc, r, saida = captura([GC], tmp_path, prazo=4)
    assert rc == 1
    assert any('duplicado' in m for m in r['motivos'])
    assert not (saida / 'parametros_brutos.yaml').exists()
    # Nem o estado é perguntado: a resposta viria de um dos dois, sem dizer qual.
    assert (saida / 'estado_nos.csv').read_text().splitlines() == ['no,estado']
    assert r['footprint_padding'] == {}


# ─── cardinalidade da leitura (conserto da régua, 21-09) ─────────────────────
#
# O `get_parameters` do rclcpp é tudo-ou-nada: um nome que falha zera o lote.
# O `collision_monitor` lista `Polygon*.max_points` e não consegue lê-los; a
# captura fazia zip(55 nomes, 0 valores) e gravava dump VAZIO sem erro — e a
# baseline aprovada ficou sem os polígonos. (O `ros2 param dump` tem o mesmo
# defeito.) Agora: contagem diferente → leitura nome a nome; ilegível
# registrado e reprova; dump vazio reprova.

NOT_SET, INT = 0, 2


def _v(tipo=INT):
    return SimpleNamespace(type=tipo)


def test_lote_completo_nao_le_um_a_um():
    lidos = []
    ok, ruins = cap.resolve_lote(['a', 'b'], [_v(), _v()], lambda n: lidos.append(n))
    assert list(ok) == ['a', 'b'] and ruins == [] and lidos == []


@pytest.mark.parametrize('lote', [[], [_v()], None])
def test_qualquer_contagem_diferente_le_um_a_um(lote):
    # Não só o lote vazio: 1 de 2 também dispara o fallback.
    ok, ruins = cap.resolve_lote(['a', 'b'], lote, lambda n: [_v()])
    assert list(ok) == ['a', 'b'] and ruins == []


def test_not_set_no_lote_e_ilegivel():
    ok, ruins = cap.resolve_lote(['a', 'b'], [_v(), _v(NOT_SET)], lambda n: [_v(NOT_SET)])
    assert list(ok) == ['a'] and ruins == ['b']


@pytest.mark.parametrize('resposta', [None, [], [_v(), _v()], [_v(NOT_SET)]])
def test_leitura_individual_tem_de_dar_exatamente_um_valor_legivel(resposta):
    ok, ruins = cap.resolve_lote(['a', 'b'], [], lambda n: resposta if n == 'b' else [_v()])
    assert list(ok) == ['a'] and ruins == ['b']


def test_ilegivel_e_dump_vazio_sao_motivos():
    g = cap.avalia_grafo(['/a'], ['/a'])
    m = cap.motivos(g, {'/a': 'responde'}, True, {}, {},
                    ilegiveis={'/a': ['x']}, vazios=['/a'])
    assert any('ilegíve' in x for x in m) and any('vazio' in x for x in m)


def test_ilegivel_preserva_os_demais_registra_e_reprova(tmp_path, ambiente):
    sobe, captura = ambiente
    sobe('--param-falso', '/falso:ilegivel')
    rc, r, saida = captura(['/falso'], tmp_path, prazo=6)
    assert rc == 1 and r['veredito'] == 'REPROVADO'
    assert r['ilegiveis'] == {'/falso': ['ruim']}
    assert any('ilegíve' in m for m in r['motivos'])
    bruto = yaml.safe_load((saida / 'parametros_brutos.yaml').read_text())
    assert bruto['/falso']['ros__parameters'] == {'use_sim_time': False, 'a': 1}
    assert yaml.safe_load((saida / 'ilegiveis.yaml').read_text()) == {'/falso': ['ruim']}


def test_lote_que_falha_mas_responde_um_a_um_e_recuperado(tmp_path, ambiente):
    sobe, captura = ambiente
    sobe('--param-falso', '/falso:lote_falha')
    rc, r, saida = captura(['/falso'], tmp_path, prazo=10)
    assert r['veredito'] == 'APROVADO', r['motivos']
    assert rc == 0
    bruto = yaml.safe_load((saida / 'parametros_brutos.yaml').read_text())
    assert bruto['/falso']['ros__parameters'] == {'use_sim_time': False, 'a': 1, 'b': 0.5}
    # A ausência de ilegíveis é EXPLÍCITA, no resumo e em arquivo.
    assert r['ilegiveis'] == {}
    assert yaml.safe_load((saida / 'ilegiveis.yaml').read_text()) == {}


def test_lista_vazia_reprova(tmp_path, ambiente):
    sobe, captura = ambiente
    sobe('--param-falso', '/falso:vazio')
    rc, r, saida = captura(['/falso'], tmp_path, prazo=6)
    assert rc == 1
    assert any('vazio' in m and '/falso' in m for m in r['motivos'])
