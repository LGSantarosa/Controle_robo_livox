"""Trava dos argumentos dos launches e da lista de nós do passo 0 (etapa 4, §8).

`test_os_argumentos_batem_com_a_trava` é o teste que os passos seguintes da
etapa não podem quebrar: nenhum argumento some, nenhum default muda, e o único
argumento novo permitido é `robo` (padrão 2) na pilha. Os outros testes provam
que o comparador reclama de cada um desses casos.

Carrega os launches de verdade: precisa do ROS e do `install/` do repo.
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


al = _carrega('argumentos_launch')
cap = _carrega('captura')
PILHA = 'ros2_packages/robot_motion/launch/pilha.launch.py'


def _trava():
    import yaml
    with open(al.TRAVA) as f:
        return yaml.safe_load(f)


# ─── a trava ─────────────────────────────────────────────────────────────────

def test_a_trava_cobre_exatamente_os_launches_do_paragrafo_8():
    assert set(_trava()) == set(al.LAUNCHES)


def test_os_argumentos_batem_com_a_trava():
    assert al.confere(_trava(), al.extrai_todos()) == []


def test_a_trava_nao_tem_caminho_da_maquina():
    import yaml
    texto = yaml.safe_dump(_trava())
    assert al.REPO not in texto
    assert os.path.expanduser('~') + '/' not in texto
    assert '/install/' not in texto


# ─── o comparador reclama ────────────────────────────────────────────────────

BASE = {PILHA: {'sim': 'false', 'bag': 'true'}}


def test_argumento_sumido_reprova():
    assert al.confere(BASE, {PILHA: {'sim': 'false'}})


def test_default_mudado_reprova():
    assert al.confere(BASE, {PILHA: {'sim': 'true', 'bag': 'true'}})


def test_novo_so_o_permitido_com_o_default_certo():
    ok = {PILHA: {'sim': 'false', 'bag': 'true', 'robo': '2'}}
    assert al.confere(BASE, ok) == []
    assert al.confere(BASE, {PILHA: {**ok[PILHA], 'robo': '3'}})
    assert al.confere(BASE, {PILHA: {**BASE[PILHA], 'outro': 'x'}})


def test_novo_permitido_so_na_pilha():
    outro = 'ros2_packages/robot_base/launch/base.launch.py'
    assert al.confere({outro: {}}, {outro: {'robo': '2'}})


def test_normaliza_caminhos():
    t = al.normaliza_texto(
        '/x/repo/install/robot_base/share/robot_base/worlds/a.sdf /x/repo/maps /h/logs',
        repo='/x/repo', home='/h')
    assert t == '<share:robot_base>/worlds/a.sdf <repo>/maps ~/logs'


# ─── a lista de nós esperados ────────────────────────────────────────────────

def test_lista_de_nos_do_robo2_e_valida_e_tem_os_dois_costmaps():
    nos = cap.le_esperados(os.path.join(AQUI, 'esperados_robo2_sim.yaml'))
    assert set(cap.COSTMAPS) <= set(nos)
    # Decisão do dono (passo 0): nada de nome variável na previsão.
    assert not [n for n in nos if n.startswith('/launch_ros_')]
    assert not [n for n in nos if cap.oculto(n)]


def test_lista_tem_so_o_lifecycle_manager_da_combinacao_default():
    nos = cap.le_esperados(os.path.join(AQUI, 'esperados_robo2_sim.yaml'))
    # sim:=true → mapa da pista e localizacao:=fixa: sem amcl, com tf fixa.
    assert '/amcl' not in nos and '/tf_map_odom' in nos and '/map_server' in nos


@pytest.mark.parametrize('no', ['/heading_controller', '/twist_mux', '/compensador_rumo',
                                '/path_follower', '/freeze_capture', '/collision_monitor'])
def test_cada_no_nosso_da_pilha_esta_na_lista(no):
    nos = cap.le_esperados(os.path.join(AQUI, 'esperados_robo2_sim.yaml'))
    assert no in nos
    with open(os.path.join(al.REPO, PILHA)) as f:
        assert f"name='{no[1:]}'" in f.read()
