"""O vão da FRENTE mede do para-choque da frente — etapa 4, passo 2.

Até aqui o `vao_frente()` passava o `re_recuo_para_choque` (centro → para-choque
de TRÁS) como avanço do para-choque da FRENTE. No robô 2 isso passa por
coincidência (0,28 dos dois lados). No robô 3 a frente fica a 0,0825 m do
`base_link` e a traseira a 0,2913 m: com o recuo no lugar do avanço, a mira
adaptativa acharia 21 cm a menos de espaço à frente do que existe
(PLANO_ETAPA4_ROBO3.md §0.1).

Mesma técnica do `test_re_desligada.py`: o método é chamado como função solta
com um `self` de mentira — o que se trava é qual parâmetro entra na conta.
"""
import math
from types import SimpleNamespace

import pytest

from robot_motion.path_follower import PathFollower

AVANCO, RECUO = 0.10, 0.28   # diferentes de propósito


def _seguidor(angulo, alcance):
    """Um raio só, no ângulo pedido, e os parâmetros do vão."""
    scan = SimpleNamespace(ranges=[alcance], angle_min=angulo, angle_increment=0.0,
                           range_max=10.0)
    return SimpleNamespace(
        scan=scan, t_scan=0.0, agora=lambda: 0.0,
        par={'re_largura': 0.555, 're_recuo_para_choque': RECUO,
             'avanco_para_choque': AVANCO, 're_scan_velho_s': 0.8})


def test_o_vao_da_frente_usa_o_avanco():
    vao = PathFollower.vao_frente(_seguidor(0.0, 1.0))
    assert vao == pytest.approx(1.0 - AVANCO), \
        f'vão da frente {vao:.3f}: {1.0 - RECUO:.3f} seria o recuo de trás no lugar do avanço'


def test_o_vao_de_tras_continua_usando_o_recuo():
    vao = PathFollower.vao_traseiro(_seguidor(math.pi, 1.0))
    assert vao == pytest.approx(1.0 - RECUO)
