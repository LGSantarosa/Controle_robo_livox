"""Calibração da velocidade manual da web para o robô 2 (A3 passo 2, 01-10).

O giro base era 6,0 rad/s (anti-skid do robô 1) e o multiplicador ia a 4×:
a web pediria até 24 rad/s e 1,2 m/s. Tetos SEPARADOS: o linear sobe até
0,50 m/s; o angular nasce em 1,25 rad/s e nunca passa disso.
"""
import os
import re
from unittest.mock import Mock

import pytest

from controllers.robot_controller import ROS2Controller

AQUI = os.path.dirname(os.path.abspath(__file__))


@pytest.fixture
def ctrl(monkeypatch):
    import rclpy
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'create_node', lambda _nome: Mock())
    return ROS2Controller(enable_publish=False)


def test_normal_e_o_normal_do_xbox(ctrl):
    assert ctrl.linear_speed == pytest.approx(0.30)
    assert ctrl.angular_speed == pytest.approx(1.25)


@pytest.mark.parametrize('pedido', [1.6667, 4.0, 10.0, 1e9])
def test_tetos_valem_mesmo_com_multiplicador_excessivo(ctrl, pedido):
    ctrl.set_speed_multiplier(pedido)
    assert ctrl.linear_speed <= 0.50 + 1e-9
    assert ctrl.angular_speed <= 1.25 + 1e-9


def test_no_maximo_o_linear_chega_a_meio_metro(ctrl):
    ctrl.set_speed_multiplier(99.0)
    assert ctrl.linear_speed == pytest.approx(0.50)
    assert ctrl.angular_speed == pytest.approx(1.25)


def test_abaixo_de_1x_o_giro_reduz(ctrl):
    ctrl.set_speed_multiplier(0.5)
    assert ctrl.linear_speed == pytest.approx(0.15)
    assert ctrl.angular_speed == pytest.approx(0.625)


def test_teclado_e_gamepad_respeitam_os_tetos(ctrl):
    ctrl.set_speed_multiplier(99.0)
    ctrl.pressed = {'KeyW', 'KeyA'}
    v, w = ctrl._compute_cmd_vel()
    assert abs(v) <= 0.50 + 1e-9 and abs(w) <= 1.25 + 1e-9
    r = ctrl.handle_gamepad_event({'type': 'axis', 'linear': 1.0, 'angular': -1.0})
    assert abs(r['linear']) <= 0.50 + 1e-9 and abs(r['angular']) <= 1.25 + 1e-9


def test_slider_e_presets_nao_passam_do_multiplicador_maximo():
    html = open(os.path.join(AQUI, 'templates', 'index.html')).read()
    slider = re.search(r'id="speed-slider"[^>]*max="([\d.]+)"', html)
    assert float(slider.group(1)) == pytest.approx(ROS2Controller.SPEED_MULT_MAX,
                                                    abs=1e-3)
    presets = [float(m) for m in re.findall(r'data-mult="([\d.]+)"', html)]
    assert presets and max(presets) <= ROS2Controller.SPEED_MULT_MAX + 1e-3
