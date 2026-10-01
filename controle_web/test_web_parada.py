"""STOP da web: zero no /web_vel por ~1 s (decisão 059). Sem espera real:
relógio e disparo do laço são injetados."""
import ast
import os
from unittest.mock import Mock

import pytest

from controllers.robot_controller import ROS2Controller

AQUI = os.path.dirname(os.path.abspath(__file__))


class Relogio:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def ctrl(monkeypatch):
    import rclpy
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'create_node', lambda _nome: Mock())
    c = ROS2Controller(enable_publish=False)
    c._agora = Relogio()
    c._laços = []
    c._dispara = c._laços.append          # não sobe thread no teste
    c._publisher = Mock()
    return c


def publicados(c):
    return [(m.twist.linear.x, m.twist.angular.z)
            for (m,), _kw in c._publisher.publish.call_args_list]


def test_parada_publica_zero_mesmo_com_web_teleop_desligado(ctrl):
    assert ctrl._publish_enabled is False
    ctrl.parada_web()
    assert publicados(ctrl) == [(0.0, 0.0)]
    assert ctrl._laços == [ctrl._laco_parada]


def test_laco_segura_zeros_ate_o_fim_da_janela(ctrl):
    ctrl.parada_web()
    for dt in (0.05, 0.5, 0.99):
        ctrl._agora.t = 100.0 + dt
        assert ctrl._passo_parada() is True
    ctrl._agora.t = 100.0 + ctrl.PARADA_S
    assert ctrl._passo_parada() is False
    assert publicados(ctrl) == [(0.0, 0.0)] * 4


def test_nenhum_nao_zero_da_web_dentro_da_janela(ctrl):
    ctrl._publish_enabled = True              # web dirigindo
    ctrl.handle_key_event({'type': 'down', 'code': 'KeyW'})
    assert publicados(ctrl)[-1][0] > 0.0
    ctrl.parada_web()
    antes = len(publicados(ctrl))
    ctrl._agora.t = 100.5
    ctrl.handle_key_event({'type': 'down', 'code': 'KeyW'})
    ctrl.handle_gamepad_event({'type': 'axis', 'linear': 1.0, 'angular': 1.0})
    ctrl._publish(0.3, 1.0)
    assert publicados(ctrl)[antes:] == []      # nada, nem zero fora do laço
    assert all(p == (0.0, 0.0) for p in publicados(ctrl)[antes - 1:])


def test_parada_limpa_teclas_e_eixos_retidos(ctrl):
    ctrl._publish_enabled = True
    ctrl.handle_key_event({'type': 'down', 'code': 'KeyW'})
    ctrl.handle_gamepad_event({'type': 'axis', 'linear': 0.8, 'angular': 0.4})
    ctrl.parada_web()
    assert ctrl.pressed == set()
    assert ctrl._last_gamepad_linear == 0.0
    assert ctrl._last_gamepad_angular == 0.0


def test_depois_da_janela_a_web_volta_a_dirigir(ctrl):
    ctrl._publish_enabled = True
    ctrl.parada_web()
    ctrl._agora.t = 100.0 + ctrl.PARADA_S + 0.01
    ctrl.handle_key_event({'type': 'down', 'code': 'KeyW'})
    assert publicados(ctrl)[-1][0] > 0.0


def test_handler_do_stop_para_antes_e_sem_depender_do_mapa():
    """Contrato estrutural do app.py (importá-lo sobe ROS, câmera e
    monitores): a parada é a PRIMEIRA ação e vem antes do `map_bridge`."""
    arvore = ast.parse(open(os.path.join(AQUI, 'app.py')).read())
    func = next(n for n in ast.walk(arvore)
                if isinstance(n, ast.FunctionDef)
                and n.name == 'handle_stop_waypoints')
    fonte = ast.unparse(func)
    assert fonte.index('controller.parada_web()') < fonte.index(
        'if map_bridge is None')
    primeiro = func.body[0]
    assert isinstance(primeiro, ast.Try)
    assert 'parada_web' in ast.unparse(primeiro.body[0])


def _republica(c):
    """Um ciclo do republicador a 50 Hz, sem thread nem sleep."""
    if c.pressed:
        c._publish(*c._compute_cmd_vel())
    elif abs(c._last_gamepad_linear) > 0.01 or abs(c._last_gamepad_angular) > 0.01:
        c._publish(c._last_gamepad_linear * c.linear_speed,
                   -c._last_gamepad_angular * c.angular_speed)


def test_evento_dentro_da_janela_nao_ressuscita_depois_dela(ctrl):
    """Revisão do Codex: o comando era recusado na hora mas GUARDADO, e o
    republicador voltava a mover quando a janela acabava. Inclui o pacote
    enviado antes do STOP e processado depois dele."""
    ctrl._publish_enabled = True
    ctrl.parada_web()
    ctrl._agora.t = 100.5
    r = ctrl.handle_key_event({'type': 'down', 'code': 'KeyW'})
    g = ctrl.handle_gamepad_event({'type': 'axis', 'linear': 1.0, 'angular': 0.5})
    assert r['descartado'] == 'parada' and g['descartado'] == 'parada'
    assert ctrl.pressed == set()
    assert ctrl._last_gamepad_linear == 0.0 and ctrl._last_gamepad_angular == 0.0

    ctrl._agora.t = 100.0 + ctrl.PARADA_S + 0.5      # janela acabou
    antes = len(publicados(ctrl))
    for _ in range(5):
        _republica(ctrl)
    assert all(p == (0.0, 0.0) for p in publicados(ctrl)[antes:])
    assert not any(p != (0.0, 0.0) for p in publicados(ctrl)[antes:])

    # Só um comando NOVO, depois da janela, volta a mover.
    ctrl.handle_key_event({'type': 'down', 'code': 'KeyW'})
    assert publicados(ctrl)[-1][0] > 0.0
