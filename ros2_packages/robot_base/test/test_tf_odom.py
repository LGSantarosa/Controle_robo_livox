"""Trava a composição de transformadas do `tf_odom`.

O nó publica `odom → base_link` a partir do `/Odometry` do FAST-LIO, e a única
parte não-óbvia é que **a pose do LIO é do SENSOR, não do corpo**. O Mid-360
está a 42 cm do chão (trena, 05-08), então publicar a pose crua como se fosse
do `base_link` embutiria esse offset em toda a navegação.

Esse erro seria **silencioso**: nada no ROS reclama de uma TF presente e
errada. Por isso a composição é travada aqui, com o caso de 42 cm explícito e
com o robô girado — que é onde uma composição errada deixa de ser um offset
constante e vira erro que depende do rumo.
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(__file__), '..', 'robot_base'))

# Importa o módulo PURO, não o nó: assim o teste roda sem ROS instalado.
from transformadas import compoe, q_mult, q_rot  # noqa: E402

IDENT = (0.0, 0.0, 0.0, 1.0)


def yaw_q(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def perto(a, b, tol=1e-9):
    assert len(a) == len(b)
    for x, y in zip(a, b):
        assert abs(x - y) < tol, f'{a} != {b}'


def test_identidade_nao_mexe_em_nada():
    t, q = compoe((1.0, 2.0, 3.0), yaw_q(0.7), (0.0, 0.0, 0.0), IDENT)
    perto(t, (1.0, 2.0, 3.0))
    perto(q, yaw_q(0.7))


def test_pose_ja_e_do_corpo_e_o_no_nao_compoe():
    """Quando child_frame_id já é base_link a segunda parcela é identidade."""
    t, q = compoe((5.0, -1.0, 0.0), IDENT, (0.0, 0.0, 0.0), IDENT)
    perto(t, (5.0, -1.0, 0.0))


def test_os_42_cm_do_mid360_descem_para_o_chao():
    """O caso que motiva o nó: sensor a +0,42 m, corpo é 0,42 abaixo dele."""
    # LIO diz: sensor em (2, 0, 0.42), sem rotação.
    # URDF diz: de livox_frame para base_link desce 0,42.
    t, q = compoe((2.0, 0.0, 0.42), IDENT, (0.0, 0.0, -0.42), IDENT)
    perto(t, (2.0, 0.0, 0.0))
    perto(q, IDENT)


def test_com_o_robo_girado_o_offset_gira_junto():
    """Se a composição fosse subtração ingênua, este é o teste que quebraria.

    Sensor 0,30 m à frente do corpo — ou seja, do sensor para o corpo anda-se
    0,30 m para TRÁS: `T(pose→base_link) = (−0,30, 0, 0)`. Com o robô girado
    90°, esse offset visto do `odom` aponta para −y, não para −x. Um código que
    somasse o offset sem rodá-lo daria (−0,30, 0) e passaria despercebido em
    qualquer teste feito só com o robô alinhado.
    """
    t, q = compoe((0.0, 0.0, 0.0), yaw_q(math.pi / 2), (-0.30, 0.0, 0.0), IDENT)
    perto(t, (0.0, -0.30, 0.0), tol=1e-9)


def test_rotacoes_se_somam():
    t, q = compoe((0.0, 0.0, 0.0), yaw_q(0.5), (0.0, 0.0, 0.0), yaw_q(0.25))
    perto(q, yaw_q(0.75), tol=1e-9)


def test_q_rot_roda_vetor_no_plano():
    perto(q_rot(yaw_q(math.pi / 2), (1.0, 0.0, 0.0)), (0.0, 1.0, 0.0), 1e-9)
    perto(q_rot(yaw_q(math.pi), (1.0, 0.0, 0.0)), (-1.0, 0.0, 0.0), 1e-9)


def test_q_mult_tem_identidade():
    perto(q_mult(yaw_q(0.3), IDENT), yaw_q(0.3))
    perto(q_mult(IDENT, yaw_q(0.3)), yaw_q(0.3))


def test_composicao_e_associativa_com_a_inversa():
    """Compor A→B com B→A tem de voltar à origem — pega erro de sinal."""
    q = yaw_q(0.9)
    qi = (-q[0], -q[1], -q[2], q[3])
    t_ab = (1.5, -0.4, 0.2)
    # T(A→B) ∘ T(B→A), com T(B→A) = (-R⁻¹·t, q⁻¹)
    t_ba = tuple(-x for x in q_rot(qi, t_ab))
    t, r = compoe(t_ab, q, t_ba, qi)
    perto(t, (0.0, 0.0, 0.0), 1e-9)
    perto(r, IDENT, 1e-9)
