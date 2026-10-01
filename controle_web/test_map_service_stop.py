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


def _runner_bridge():
    """Bridge falsa para rodar o `_wp_runner` real com uma rota de 1 ponto."""
    fb = _fake_bridge()
    fb._wp_list = [{'x': 1.0, 'y': 2.0, 'yaw': 0.0}]
    fb._wp_current_idx = 0
    fb._wp_loop = False
    fb._clear_costmap_srv = Mock()
    fb._clear_costmap_srv.wait_for_service.return_value = True
    return fb


def _stop(fb):
    with fb._wp_lock:
        fb._wp_stop.set()
        fb._wp_gen += 1


def test_stop_antes_da_conferencia_nao_envia_goal(monkeypatch):
    """STOP durante a espera do `_send`: nada é enviado."""
    import map_service
    fb = _runner_bridge()
    fb._wp_send_goal_action = Mock()
    monkeypatch.setattr(map_service.time, 'sleep', lambda _s: _stop(fb))
    MapBridge._wp_runner(fb)
    fb._wp_send_goal_action.assert_not_called()


def test_stop_logo_depois_da_conferencia_cancela_no_aceite(monkeypatch):
    """A ordem que a revisão do Codex reproduziu: STOP depois da conferência,
    antes do aceite. O goal leva a geração VELHA e é cancelado no aceite."""
    import map_service
    fb = _runner_bridge()
    monkeypatch.setattr(map_service.time, 'sleep', lambda _s: None)
    enviados = []

    def envia(x, y, yaw, gen=None):
        enviados.append(gen)
        _stop(fb)                       # o STOP chega agora

    fb._wp_send_goal_action = envia
    fb._wp_goal_done.set()              # sai do laço logo depois do envio
    fb._wp_goal_status = 6
    MapBridge._wp_runner(fb)
    assert enviados == [0]

    handle = Mock(accepted=True)
    futuro = Mock()
    futuro.result.return_value = handle
    MapBridge._on_goal_response(fb, futuro, enviados[0])
    handle.cancel_goal_async.assert_called_once()
    assert fb._wp_goal_handle is None
