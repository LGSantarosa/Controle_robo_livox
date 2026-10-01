"""STOP da web (A1/A7 da revisão de 01-10).

A1: o STOP cancelava só o goal de rota já aceito; o clique-para-ir seguia.
A7: STOP entre o envio do goal e o aceite do Nav2 não cancelava nada.
"""
import threading
import types
from unittest.mock import Mock

from map_service import MapBridge


def _fake_bridge():
    fb = types.SimpleNamespace()
    fb._wp_lock = threading.Lock()
    fb._wp_stop = threading.Event()
    fb._wp_gen = 0
    fb._wp_goal_handle = None
    fb._wp_goal_status = None
    fb._wp_goal_done = threading.Event()
    fb._wp_thread = None
    fb._wp_active = True
    fb._sock = Mock()
    fb._nav_cancel_cli = Mock()
    fb._nav_cancel_cli.wait_for_service.return_value = True
    fb._node = Mock()
    fb._node.get_clock.return_value.now.return_value.to_msg.return_value = 'agora'
    fb._cancel_all_nav_goals = types.MethodType(MapBridge._cancel_all_nav_goals, fb)
    fb._on_goal_result = Mock()
    return fb


def test_stop_cancela_todos_os_goals_inclusive_o_clique():
    fb = _fake_bridge()
    MapBridge.stop_waypoints(fb)
    fb._nav_cancel_cli.call_async.assert_called_once()
    req = fb._nav_cancel_cli.call_async.call_args[0][0]
    assert bytes(req.goal_info.goal_id.uuid) == bytes(16)   # zerado = todos
    assert req.goal_info.stamp == 'agora'                   # aceitos até agora


def test_parar_para_comecar_rota_nova_nao_cancela_tudo():
    fb = _fake_bridge()
    MapBridge.stop_waypoints(fb, cancel_all=False)
    fb._nav_cancel_cli.call_async.assert_not_called()


def test_goal_aceito_depois_do_stop_e_cancelado_no_aceite():
    fb = _fake_bridge()
    gen_do_envio = fb._wp_gen
    MapBridge.stop_waypoints(fb)                 # STOP antes do aceite
    handle = Mock(accepted=True)
    futuro = Mock()
    futuro.result.return_value = handle
    MapBridge._on_goal_response(fb, futuro, gen_do_envio)
    handle.cancel_goal_async.assert_called_once()
    assert fb._wp_goal_handle is None


def test_goal_aceito_sem_stop_segue_normal():
    fb = _fake_bridge()
    handle = Mock(accepted=True)
    futuro = Mock()
    futuro.result.return_value = handle
    MapBridge._on_goal_response(fb, futuro, fb._wp_gen)
    handle.cancel_goal_async.assert_not_called()
    assert fb._wp_goal_handle is handle
