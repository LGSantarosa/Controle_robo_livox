"""O perfil do robô 2 é a base de hoje — etapa 4, passo 3.

`robot_motion.perfil.parametros(robo, share)` é o ÚNICO ponto que diz o que
cada nó da pilha recebe (PLANO_ETAPA4_ROBO3.md §2.2). O robô 2 não tem arquivo
de sobreposição: o perfil dele devolve os mesmos `nav2.yaml` e
`collision_monitor.yaml` de sempre, nenhuma reescrita e nenhuma sobreposição do
`path_follower`. O perfil do robô 3 (passo 4) está em `test_perfil_robo3.py`.

A prova de equivalência é contra a BASELINE v2 (os valores VIVOS no Gazebo),
não contra os arquivos — os arquivos são o que o perfil devolve, e teste que
compara o alvo com ele mesmo não testa nada.
"""
import ast
import importlib
import os
import subprocess
import sys

import pytest
import yaml

PKG = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
RAIZ = os.path.abspath(os.path.join(PKG, '..', '..'))
SEGUIDOR = os.path.join(PKG, 'robot_motion', 'path_follower.py')
BASELINE_V2 = os.path.join(RAIZ, 'docs', 'dados', '2026-09-21-baseline-v2-robo2',
                           '02-baseline-v2-aprovada', 'parametros_normalizados.yaml')
COSTMAPS = ('global_costmap', 'local_costmap')
SEIS_DO_SEGUIDOR = ('passagem_meia_largura', 'passagem_margem', 're_largura',
                    're_recuo_para_choque', 'avanco_para_choque',
                    'desencalhe_pivo_folga')


def _perfil():
    return importlib.import_module('robot_motion.perfil')


# ─── a interface ─────────────────────────────────────────────────────────────

def test_robo2_devolve_os_caminhos_de_hoje():
    share = '/qualquer/share/robot_motion'
    p = _perfil().parametros(2, share)
    assert set(p) == {'nav2', 'nav2_rewrites', 'collision_monitor',
                      'collision_monitor_rewrites', 'path_follower'}
    assert p['nav2'] == os.path.join(share, 'config', 'nav2.yaml')
    assert p['collision_monitor'] == os.path.join(share, 'config', 'collision_monitor.yaml')


def test_robo2_nao_reescreve_nem_sobrepoe_nada():
    p = _perfil().parametros(2, PKG)
    assert p['nav2_rewrites'] == {}
    assert p['collision_monitor_rewrites'] == {}
    assert p['path_follower'] == {}


@pytest.mark.parametrize('robo', [0, 1, 4, '2', 2.0, True, None])
def test_robo_que_nao_existe_nao_tem_perfil(robo):
    """Só o INTEIRO 2. `'2'`, `2.0` e `True` não viram 2 por conversão."""
    with pytest.raises(_perfil().RoboSemPerfil):
        _perfil().parametros(robo, PKG)


def test_robo_sem_perfil_e_um_value_error():
    assert issubclass(_perfil().RoboSemPerfil, ValueError)


def test_o_perfil_e_puro_sem_ros():
    """Montar o perfil não pode exigir ROS: é função de dados, testável sozinha."""
    codigo = ('import sys; import robot_motion.perfil; '
              'print(sorted({m.split(".")[0] for m in sys.modules} & '
              '{"rclpy", "launch", "launch_ros", "ament_index_python"}))')
    saida = subprocess.run([sys.executable, '-c', codigo], cwd=PKG, check=True,
                           capture_output=True, text=True,
                           env={**os.environ, 'PYTHONPATH': PKG}).stdout.strip()
    assert saida == '[]'


# ─── a equivalência com a baseline v2 ────────────────────────────────────────

def _default_do_seguidor(nome):
    """Default declarado no `path_follower.py`, lido por AST (sem rclpy)."""
    with open(SEGUIDOR) as f:
        arvore = ast.parse(f.read())
    for no in ast.walk(arvore):
        if isinstance(no, ast.Tuple) and len(no.elts) == 2:
            chave, val = no.elts
            if (isinstance(chave, ast.Constant) and chave.value == nome
                    and isinstance(val, ast.Constant)):
                return val.value
    raise AssertionError(f'`{nome}` não é parâmetro declarado do seguidor')


def test_perfil_robo2_identico_a_linha_de_base():
    p = _perfil().parametros(2, PKG)
    with open(BASELINE_V2) as f:
        vivo = yaml.safe_load(f)
    assert p['nav2_rewrites'] == {}, 'reescrita no robô 2 mudaria o que os costmaps leem'
    assert p['collision_monitor_rewrites'] == {}, \
        'reescrita no robô 2 mudaria os polígonos do reflexo'

    with open(p['nav2']) as f:
        nav2 = yaml.safe_load(f)
    for c in COSTMAPS:
        montado = nav2[c][c]['ros__parameters']
        base = vivo[f'/{c}/{c}']
        assert yaml.safe_load(montado['footprint']) == yaml.safe_load(base['footprint']), c
        assert montado['footprint_padding'] == base['footprint_padding'], c

    with open(p['collision_monitor']) as f:
        cm = yaml.safe_load(f)['collision_monitor']['ros__parameters']
    for pol in ('PolygonStop', 'PolygonApproach'):
        assert (yaml.safe_load(cm[pol]['points'])
                == yaml.safe_load(vivo['/collision_monitor'][f'{pol}.points'])), pol

    efetivo = {n: _default_do_seguidor(n) for n in SEIS_DO_SEGUIDOR}
    efetivo.update(p['path_follower'])
    for n in SEIS_DO_SEGUIDOR:
        assert efetivo[n] == vivo['/path_follower'][n], n
