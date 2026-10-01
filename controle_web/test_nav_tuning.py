"""Contrato do ajuste ao vivo do Nav2, sem precisar de ROS no teste web."""
import math

import pytest

from controllers.robot_controller import EchoController


def test_ajuste_nav_nasce_neutro_e_expoe_limites():
    ctrl = EchoController()
    estado = ctrl.nav_tuning_state()
    assert estado['linear_scale'] == 1.0
    assert estado['curve_scale'] == 1.0
    assert estado['linear_max'] == 1.4  # 0,5 m/s × 1,4 = teto físico 0,7
    assert estado['curve_max'] == 3.0


def test_ajuste_nav_aplica_linear_e_curva_independentes():
    ctrl = EchoController()
    estado = ctrl.set_nav_tuning(1.20, 1.35)
    assert estado['linear_scale'] == pytest.approx(1.20)
    assert estado['curve_scale'] == pytest.approx(1.35)


@pytest.mark.parametrize('linear, curva', [
    (0.49, 1.0), (1.41, 1.0), (1.0, 0.49), (1.0, 3.01),
    (math.nan, 1.0), (1.0, math.inf),
])
def test_ajuste_nav_recusa_fora_da_faixa_sem_mudar_estado(linear, curva):
    ctrl = EchoController()
    with pytest.raises(ValueError):
        ctrl.set_nav_tuning(linear, curva)
    assert ctrl.nav_tuning_state()['linear_scale'] == 1.0
    assert ctrl.nav_tuning_state()['curve_scale'] == 1.0
