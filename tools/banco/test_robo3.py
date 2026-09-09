"""Provas sem hardware do protocolo de caracterização do robô 3.

Não fingem validar a física: travam a linha do tempo, a matriz de movimentos,
o analisador e o pareamento da geometria entre URDF/controlador real.
"""

import argparse
import csv
import importlib.util
import math
import os
import sys
import types

import pytest
import yaml


AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ = os.path.dirname(os.path.dirname(AQUI))


def _carrega(nome):
    spec = importlib.util.spec_from_file_location(
        f'teste_{nome}', os.path.join(AQUI, f'{nome}.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _carrega_perfil_sem_ros():
    for nome in ('rclpy', 'rclpy.node', 'rclpy.qos', 'geometry_msgs',
                 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
                 'std_msgs', 'std_msgs.msg'):
        sys.modules.setdefault(nome, types.ModuleType(nome))
    sys.modules['rclpy.node'].Node = object
    sys.modules['rclpy.qos'].QoSProfile = object
    sys.modules['rclpy.qos'].ReliabilityPolicy = types.SimpleNamespace(RELIABLE=1)
    sys.modules['geometry_msgs.msg'].TwistStamped = object
    sys.modules['nav_msgs.msg'].Odometry = object
    sys.modules['std_msgs.msg'].Float64 = object
    return _carrega('perfil_robo3')


perfil = _carrega_perfil_sem_ros()
medir = _carrega('mede_robo3')
sessao = _carrega('sessao_robo3')


def _cfg(tipo, **kw):
    base = dict(perfil=tipo, dur=30.0, liga=2.0, pausa=1.0, v=0.25, wz=0.2)
    base.update(kw)
    return argparse.Namespace(**base)


def test_perfil_suspenso_tem_quatro_pulsos_separados_por_zero():
    cfg = _cfg('suspenso')
    amostras = [perfil.comando_perfil(cfg, t) for t in
                (2.10, 2.50, 3.50, 3.90, 4.90, 5.30, 6.30)]
    assert amostras == [
        ('frente', 0.08, 0.0), ('pausa', 0.0, 0.0),
        ('re', -0.08, 0.0), ('pausa', 0.0, 0.0),
        ('giro_esquerda', 0.0, 0.15), ('pausa', 0.0, 0.0),
        ('giro_direita', 0.0, -0.15),
    ]
    assert perfil.comando_perfil(cfg, perfil.duracao_perfil(cfg)) == (
        'cauda', 0.0, 0.0)


def test_inversao_tem_zero_real_entre_os_dois_sentidos():
    cfg = _cfg('inversao', liga=2.0, pausa=0.25, v=-0.25, wz=0.0)
    assert perfil.comando_perfil(cfg, 2.1) == ('ida', -0.25, 0.0)
    assert perfil.comando_perfil(cfg, 4.1) == ('pausa', 0.0, 0.0)
    assert perfil.comando_perfil(cfg, 4.3) == ('volta', 0.25, -0.0)


def test_matriz_cobre_re_reta_giro_curvas_e_inversoes():
    nomes = [c['nome'] for p in sessao.PASSOS for c in p['corridas']]
    assert any('reta-curta-frente' in n for n in nomes)
    assert any('reta-curta-re' in n for n in nomes)
    assert any('pivo-esq' in n for n in nomes)
    assert any('pivo-dir' in n for n in nomes)
    for tamanho in ('curta', 'longa'):
        for marcha in ('frente', 're'):
            for lado in ('esq', 'dir'):
                assert any(f'{tamanho}-{marcha}-{lado}' in n for n in nomes)
    assert any('frente-re' in n for n in nomes)
    assert any('re-frente' in n for n in nomes)


def _csv_curva(caminho):
    campos = ['t', 'fase', 'cmd_v', 'cmd_wz', 'x', 'y', 'yaw', 'v_pose',
              'wz_pose', 'trajeto_pose', 'vel_esq', 'vel_dir', 'tensao']
    x = y = yaw = trajeto = 0.0
    linhas = []
    dt = 0.02
    for i in range(301):
        t = i * dt
        ativo = 2.0 <= t < 4.0
        responde = 2.10 <= t < 4.12
        v = 0.25 if responde else 0.0
        wz = 0.20 if responde else 0.0
        if i:
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            yaw += wz * dt
            trajeto += abs(v) * dt
        linhas.append(dict(
            t=t, fase='pulso' if ativo else ('assenta' if t < 2 else 'cauda'),
            cmd_v=0.25 if ativo else 0.0, cmd_wz=0.20 if ativo else 0.0,
            x=x, y=y, yaw=yaw, v_pose=v, wz_pose=wz,
            trajeto_pose=trajeto,
            vel_esq=(0.25 - 0.20 * 0.3225 / 2) / 0.0825 if responde else 0,
            vel_dir=(0.25 + 0.20 * 0.3225 / 2) / 0.0825 if responde else 0,
            tensao=40.0 - (0.2 if ativo else 0.0)))
    with open(caminho, 'w', newline='') as arq:
        w = csv.DictWriter(arq, fieldnames=campos)
        w.writeheader()
        w.writerows(linhas)


def test_analisador_recupera_curvatura_latencia_e_giro_das_rodas(tmp_path):
    caminho = tmp_path / 'curva.csv'
    _csv_curva(caminho)
    r = medir.mede(caminho)
    assert r['v_regime_m_s'] == pytest.approx(0.25, abs=0.01)
    assert r['wz_regime_rad_s'] == pytest.approx(0.20, abs=0.01)
    assert r['curvatura_1_m'] == pytest.approx(0.8, rel=0.05)
    assert r['latencia_s'] == pytest.approx(0.10, abs=0.03)
    assert r['giro_lio_sobre_roda'] == pytest.approx(1.0, rel=0.05)


def test_controlador_real_usa_geometria_do_robo3_e_encoder():
    caminho = os.path.join(
        RAIZ, 'ros2_packages', 'hoverboard_driver', 'bringup', 'config',
        'hoverboard_controllers_robo3.yaml')
    with open(caminho) as arq:
        p = yaml.safe_load(arq)['hoverboard_base_controller']['ros__parameters']
    assert p['wheel_separation'] == pytest.approx(sessao.BITOLA)
    assert p['wheel_radius'] == pytest.approx(sessao.RAIO)
    assert p['open_loop'] is False


def test_bancada_nao_promove_comando_baixo_a_cem_rpm():
    caminho = os.path.join(
        RAIZ, 'ros2_packages', 'robot_base', 'description', 'robo3.urdf.xacro')
    fonte = open(caminho).read()
    assert '<param name="deadband_enable">false</param>' in fonte


def test_base_real_nasce_sem_tracao_e_publica_descricao():
    caminho = os.path.join(
        RAIZ, 'ros2_packages', 'robot_base', 'launch', 'base_robo3.launch.py')
    fonte = open(caminho).read()
    assert "'tracao', default_value='false'" in fonte
    assert "'robot_state_publisher'" in fonte
    assert 'condition=UnlessCondition(tracao)' in fonte
