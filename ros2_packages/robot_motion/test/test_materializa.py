"""A materialização do perfil — etapa 6, passo 3 (`PLANO_ETAPA6_ROBO3.md` §7).

Os costmaps são nós DENTRO dos servidores do Nav2: dicionário passado ao
`Node` do servidor não desce até eles, e quem chega lá é o arquivo. Por isso a
reescrita do perfil tem de virar arquivo em disco — e a pilha, que hoje aborta
de propósito quando o perfil pede reescrita, passa a aplicá-la.

Aqui a máquina é testada SOZINHA, sem launch e sem ROS. Três contratos:

1. **Sem reescrita, nada acontece.** O robô 2 continua recebendo os caminhos
   do `share/`, e nem a pasta é criada. Isto não é otimização: o caminho É o
   valor do parâmetro, e materializar para o robô 2 quebraria a comparação
   byte a byte que protege o robô que funciona (§6 do plano).
2. **Com reescrita, o arquivo escrito é `aplica_reescritas` do original** — e
   o original nunca é tocado.
3. **A escrita é atômica**: temporário no mesmo diretório e troca por
   `os.replace`. Falha no meio não deixa destino nem temporário.
"""
import importlib
import os

import pytest
import yaml


def _perfil():
    return importlib.import_module('robot_motion.perfil')


BASE_NAV2 = {'planner_server': {'ros__parameters': {'tolerance': 0.25}},
             'global_costmap': {'global_costmap': {'ros__parameters': {
                 'footprint': '[[1, 1]]', 'footprint_padding': 0.0}}}}
BASE_CM = {'collision_monitor': {'ros__parameters': {
    'PolygonStop': {'points': '[[1, 1]]'}}}}


@pytest.fixture
def base(tmp_path):
    """Um par de YAMLs de base, no papel do `share/` — e um perfil que os usa."""
    nav2 = tmp_path / 'share' / 'nav2.yaml'
    cm = tmp_path / 'share' / 'collision_monitor.yaml'
    nav2.parent.mkdir()
    nav2.write_text(yaml.safe_dump(BASE_NAV2))
    cm.write_text(yaml.safe_dump(BASE_CM))
    return {'nav2': str(nav2), 'collision_monitor': str(cm),
            'nav2_rewrites': {}, 'collision_monitor_rewrites': {},
            'path_follower': {}}


REESCRITAS = {
    'nav2_rewrites': {
        ('global_costmap', 'global_costmap', 'ros__parameters',
         'footprint'): '[[0.0825, 0.19]]',
        ('global_costmap', 'global_costmap', 'ros__parameters',
         'footprint_padding'): 0.01},
    'collision_monitor_rewrites': {
        ('collision_monitor', 'ros__parameters', 'PolygonStop',
         'points'): '[[0.2, 0.2]]'},
}


# ─── 1. sem reescrita, nada acontece ─────────────────────────────────────────

def test_sem_reescrita_os_destinos_sao_os_originais(base, tmp_path):
    destinos = _perfil().destinos(base, str(tmp_path / 'corrida'))
    assert destinos == {'nav2': base['nav2'],
                        'collision_monitor': base['collision_monitor']}


def test_sem_reescrita_nao_escreve_nem_cria_pasta(base, tmp_path):
    pasta = tmp_path / 'corrida'
    assert _perfil().materializa(base, str(pasta)) == {}
    assert not pasta.exists(), 'criou a pasta da corrida sem ter o que escrever'


# ─── 2. com reescrita, o arquivo é o original reescrito ──────────────────────

def test_com_reescrita_os_destinos_ficam_na_pasta_da_corrida(base, tmp_path):
    pasta = tmp_path / 'corrida'
    destinos = _perfil().destinos({**base, **REESCRITAS}, str(pasta))
    for chave, caminho in destinos.items():
        assert os.path.dirname(caminho) == str(pasta), (chave, caminho)
        assert os.path.isabs(caminho), caminho
    # Nomes distintos, e cada um dizendo a que veio — dois arquivos anônimos
    # numa pasta de evidência não contam história nenhuma.
    assert 'nav2' in os.path.basename(destinos['nav2'])
    assert 'collision_monitor' in os.path.basename(destinos['collision_monitor'])
    assert destinos['nav2'] != destinos['collision_monitor']


def test_com_reescrita_escreve_o_original_reescrito(base, tmp_path):
    perfil = _perfil()
    pasta = tmp_path / 'corrida'
    completo = {**base, **REESCRITAS}
    escritos = perfil.materializa(completo, str(pasta))

    assert set(escritos) == {'nav2', 'collision_monitor'}
    assert escritos == perfil.destinos(completo, str(pasta))
    for chave, origem in (('nav2', BASE_NAV2), ('collision_monitor', BASE_CM)):
        with open(escritos[chave]) as f:
            assert yaml.safe_load(f) == perfil.aplica_reescritas(
                origem, completo[f'{chave}_rewrites'])


def test_o_original_nao_e_tocado(base, tmp_path):
    antes = {c: open(base[c]).read() for c in ('nav2', 'collision_monitor')}
    _perfil().materializa({**base, **REESCRITAS}, str(tmp_path / 'corrida'))
    for chave, texto in antes.items():
        assert open(base[chave]).read() == texto, chave


def test_reescrita_que_nao_existe_reprova_antes_de_escrever(base, tmp_path):
    """`aplica_reescritas` já reprova caminho inexistente; o que se prova aqui
    é que a reprovação acontece ANTES de qualquer arquivo nascer."""
    pasta = tmp_path / 'corrida'
    quebrado = {**base, 'nav2_rewrites': {('nao', 'existe'): 1}}
    with pytest.raises(ValueError):
        _perfil().materializa(quebrado, str(pasta))
    assert not (pasta.exists() and list(pasta.iterdir()))


def test_o_segundo_yaml_invalido_nao_deixa_o_primeiro_nascer(base, tmp_path):
    """🔴 O caso que separa preflight de escrita incremental.

    O Nav2 é válido, o reflexo não. Validando um por vez, o YAML do Nav2 já
    estaria publicado quando o erro aparecesse — e a pasta da corrida ficaria
    com UM arquivo, que é pior do que nenhum: parece evidência completa, e o
    reflexo estaria lendo a configuração do robô errado.
    """
    pasta = tmp_path / 'corrida'
    meio_quebrado = {**base, **REESCRITAS,
                     'collision_monitor_rewrites': {('nao', 'existe'): 1}}
    with pytest.raises(ValueError):
        _perfil().materializa(meio_quebrado, str(pasta))
    nascidos = [p for p in pasta.rglob('*') if p.is_file()] if pasta.exists() else []
    assert not nascidos, f'o primeiro YAML nasceu antes do erro: {nascidos}'


# ─── 3. a escrita é atômica ──────────────────────────────────────────────────

def test_a_escrita_troca_por_os_replace_no_mesmo_diretorio(base, tmp_path,
                                                           monkeypatch):
    trocas = []
    real = os.replace

    def espia(origem, destino, *a, **kw):
        trocas.append((str(origem), str(destino)))
        return real(origem, destino, *a, **kw)

    monkeypatch.setattr(os, 'replace', espia)
    pasta = tmp_path / 'corrida'
    escritos = _perfil().materializa({**base, **REESCRITAS}, str(pasta))

    assert {d for _, d in trocas} == set(escritos.values()), trocas
    for origem, destino in trocas:
        assert os.path.dirname(origem) == os.path.dirname(destino), \
            f'temporário fora do diretório do destino: {origem} → {destino}'


def test_falha_no_meio_nao_deixa_destino_nem_temporario(base, tmp_path,
                                                        monkeypatch):
    """E a falha suja o arquivo antes de cair, senão o teste aceitaria uma
    implementação que serializa para texto e só depois abre o destino."""
    def explode(dados, fluxo=None, *_a, **_kw):
        if fluxo is not None:
            fluxo.write('# fragmento\n')
            fluxo.flush()
        raise RuntimeError('falha proposital no meio da serialização')

    monkeypatch.setattr(yaml, 'safe_dump', explode)
    pasta = tmp_path / 'corrida'
    with pytest.raises(RuntimeError, match='falha proposital'):
        _perfil().materializa({**base, **REESCRITAS}, str(pasta))
    sobrou = [p for p in pasta.rglob('*') if p.is_file()] if pasta.exists() else []
    assert not sobrou, f'sobrou depois da falha: {sobrou}'
