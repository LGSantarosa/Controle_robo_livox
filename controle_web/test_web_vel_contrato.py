"""Contrato do /web_vel com o twist_mux do robô 2 (A3 da revisão de 01-10).

O mux roda com `use_stamped: true` e assina `TwistStamped`. A web publicava
`Twist` cru: os tipos não se conectam no DDS e nenhum comando da web chegava
à roda, nem o zero do STOP.
"""
from unittest.mock import Mock

import pytest
import yaml

from geometry_msgs.msg import TwistStamped

from controllers import robot_controller
from controllers.robot_controller import ROS2Controller


@pytest.fixture
def ctrl(monkeypatch):
    import rclpy
    no = Mock()
    no.get_clock.return_value.now.return_value.to_msg.return_value = (
        TwistStamped().header.stamp.__class__(sec=12, nanosec=34))
    monkeypatch.setattr(rclpy, 'ok', lambda: True)
    monkeypatch.setattr(rclpy, 'create_node', lambda _nome: no)
    c = ROS2Controller(enable_publish=False)
    return c, no


def test_web_vel_e_publicado_como_twist_stamped(ctrl):
    _c, no = ctrl
    tipos = {args[1]: args[0] for args, _kw in
             (chamada for chamada in no.create_publisher.call_args_list)}
    assert tipos['/web_vel'] is TwistStamped


def test_mensagem_tem_carimbo_atual_e_frame_do_corpo(ctrl):
    c, _no = ctrl
    msg = c._monta_web_vel(0.25, -0.5)
    assert isinstance(msg, TwistStamped)
    assert msg.header.frame_id == 'base_link'
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (12, 34)
    assert msg.twist.linear.x == pytest.approx(0.25)
    assert msg.twist.angular.z == pytest.approx(-0.5)


def test_o_mux_do_robo2_espera_stamped_no_canal_web():
    """Trava o outro lado do contrato: se o mux deixar de ser stamped, este
    teste avisa que a web tem de mudar junto."""
    import os
    raiz = os.path.dirname(os.path.dirname(os.path.abspath(
        robot_controller.__file__)))
    cfg = os.path.join(raiz, '..', 'ros2_packages', 'robot_motion', 'config',
                       'twist_mux.yaml')
    p = yaml.safe_load(open(cfg))['twist_mux']['ros__parameters']
    assert p['use_stamped'] is True
    assert p['topics']['web']['topic'] == 'web_vel'
