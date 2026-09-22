"""Reescrita de YAML por caminho exato — etapa 4, passo 4b (plano §3, D-c).

Os costmaps do Nav2 são nós DENTRO do `planner_server`/`controller_server`:
dicionário passado ao `Node` do servidor não chega neles. O perfil do robô 3
entrega o footprint reescrevendo o YAML, e o reflexo recebe os polígonos pelo
mesmo mecanismo. `perfil.aplica_reescritas(dados, reescritas)` é essa
reescrita, pura:

- copia a estrutura (o original não muda);
- troca SÓ folhas que já existem, por caminho completo (tupla de chaves);
- caminho inexistente, que para num ramo ou que atravessa uma folha REPROVA —
  criar a chave em silêncio seria o nó lendo um parâmetro que ninguém declarou,
  e o valor pedido sumindo sem aviso.

A prova de "nada mais mudou" é por folhas: o conjunto de caminhos fica igual, e
só os caminhos pedidos mudam de valor.
"""
import copy
import importlib
import os

import pytest
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
NAV2 = os.path.join(PKG, 'config', 'nav2.yaml')
CM = os.path.join(PKG, 'config', 'collision_monitor.yaml')
FOOTPRINT_GLOBAL = ('global_costmap', 'global_costmap', 'ros__parameters', 'footprint')


def _perfil():
    return importlib.import_module('robot_motion.perfil')


def _le(caminho):
    with open(caminho) as f:
        return yaml.safe_load(f)


def folhas(dados, prefixo=()):
    """{caminho: valor} de toda folha. Lista é folha (não se reescreve item)."""
    if isinstance(dados, dict):
        out = {}
        for k, v in dados.items():
            out.update(folhas(v, prefixo + (k,)))
        return out
    return {prefixo: dados}


# ─── troca só o que foi pedido ───────────────────────────────────────────────

def test_troca_so_o_caminho_pedido_e_nenhuma_outra_folha():
    base = _le(NAV2)
    novo = _perfil().aplica_reescritas(base, {FOOTPRINT_GLOBAL: '[[1.0, 1.0]]'})
    antes, depois = folhas(base), folhas(novo)
    assert set(depois) == set(antes), 'reescrita não cria nem apaga folha'
    mudou = {c for c in antes if antes[c] != depois[c]}
    assert mudou == {FOOTPRINT_GLOBAL}
    assert depois[FOOTPRINT_GLOBAL] == '[[1.0, 1.0]]'


def test_nao_altera_o_original():
    base = _le(CM)
    intacto = copy.deepcopy(base)
    caminho = ('collision_monitor', 'ros__parameters', 'PolygonStop', 'points')
    _perfil().aplica_reescritas(base, {caminho: '[[9.0, 9.0]]'})
    assert base == intacto


def test_devolve_copia_e_nao_o_mesmo_objeto():
    base = _le(CM)
    novo = _perfil().aplica_reescritas(base, {})
    assert novo == base
    assert novo is not base
    assert novo['collision_monitor'] is not base['collision_monitor']


# ─── caminho que não é folha existente reprova ───────────────────────────────

def test_folha_inexistente_reprova():
    caminho = ('global_costmap', 'global_costmap', 'ros__parameters', 'footprnt')
    with pytest.raises(ValueError, match='footprnt'):
        _perfil().aplica_reescritas(_le(NAV2), {caminho: '[]'})


def test_ramo_intermediario_inexistente_reprova():
    caminho = ('global_costmap', 'costmap_global', 'ros__parameters', 'footprint')
    with pytest.raises(ValueError, match='costmap_global'):
        _perfil().aplica_reescritas(_le(NAV2), {caminho: '[]'})


def test_caminho_que_para_num_ramo_reprova():
    """Trocar um dicionário inteiro por um valor apagaria as folhas dele."""
    caminho = ('global_costmap', 'global_costmap', 'ros__parameters', 'inflation_layer')
    with pytest.raises(ValueError, match='inflation_layer'):
        _perfil().aplica_reescritas(_le(NAV2), {caminho: 0.5})


@pytest.mark.parametrize('folha', ['footprint', 'footprint_padding'])
def test_caminho_que_atravessa_folha_reprova(folha):
    """Folha de texto E numérica: na numérica, `in` daria TypeError, não o erro
    que diz o caminho."""
    caminho = ('global_costmap', 'global_costmap', 'ros__parameters', folha, 'x')
    with pytest.raises(ValueError, match=folha):
        _perfil().aplica_reescritas(_le(NAV2), {caminho: 1.0})


@pytest.mark.parametrize('caminho', [
    'global_costmap.global_costmap.ros__parameters.footprint',   # texto com ponto
    (),                                                          # vazio
    ('global_costmap', 1, 'ros__parameters'),                    # chave não-texto
])
def test_caminho_so_como_tupla_de_texto(caminho):
    """Caminho por ponto é ambíguo (chave com ponto existe em ROS); só tupla."""
    with pytest.raises(ValueError):
        _perfil().aplica_reescritas(_le(NAV2), {caminho: '[]'})


def test_uma_invalida_reprova_o_lote_inteiro_sem_efeito_parcial():
    base = _le(NAV2)
    intacto = copy.deepcopy(base)
    lote = {FOOTPRINT_GLOBAL: '[[1.0, 1.0]]',
            ('global_costmap', 'global_costmap', 'ros__parameters', 'nao_existe'): 1}
    with pytest.raises(ValueError, match='nao_existe'):
        _perfil().aplica_reescritas(base, lote)
    assert base == intacto
