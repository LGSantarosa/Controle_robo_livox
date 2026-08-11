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


# ------------------------------------------- o `odom` no CHÃO (decisão 018)
#
# Medido em 10-08: `tf2_echo odom base_link` deu z = −0,477 m. A composição só
# à direita põe o base_link certo em relação ao sensor e deixa a ORIGEM do odom
# em cima do Mid-360, a 0,42 m. O chão vai para z ≈ −0,42 e a percepção do Nav2
# rejeita a nuvem inteira (faixa de altura e `origin_z` da VoxelLayer, os dois
# no frame global).

from transformadas import inverte  # noqa: E402


def compoe_como_o_no(pose_t, pose_q, sensor_t, sensor_q):
    """Exatamente o que o `tf_odom.passo` faz: compõe com o URDF à direita e
    tira a origem do sensor à esquerda."""
    t, q = compoe(pose_t, pose_q, sensor_t, sensor_q)
    t_i, q_i = inverte(sensor_t, sensor_q)
    return compoe(t_i, q_i, t, q)


# T(body → base_link): o corpo está 42 cm ABAIXO do sensor.
SENSOR = ((0.0, 0.0, -0.42), IDENT)


def test_na_largada_o_base_link_esta_na_ORIGEM_do_odom():
    """Robô parado no instante zero: a TF tem de ser identidade. Antes desta
    conta ela dava z = −0,42, e o chão inteiro descia junto."""
    t, q = compoe_como_o_no((0.0, 0.0, 0.0), IDENT, *SENSOR)
    perto(t, (0.0, 0.0, 0.0))
    perto(q, IDENT)


def test_andar_para_a_frente_nao_muda_a_altura():
    """Andou 2 m no plano: z continua zero, e não −0,42."""
    t, _ = compoe_como_o_no((2.0, 0.0, 0.0), IDENT, *SENSOR)
    perto(t, (2.0, 0.0, 0.0))


def test_girado_no_lugar_o_corpo_NAO_sai_do_lugar():
    """O sensor é centrado (05-08: 42 cm, centrado), então girar no eixo não
    translada o corpo. Uma composição errada faria o robô 'orbitar'."""
    t, q = compoe_como_o_no((0.0, 0.0, 0.0), yaw_q(math.pi / 2), *SENSOR)
    perto(t, (0.0, 0.0, 0.0))
    perto(q, yaw_q(math.pi / 2))


def test_a_altura_do_sensor_sai_da_conta_mas_a_do_LIO_fica():
    """Se o LIO diz que o sensor subiu 3 cm (rampa, deriva), isso é medida e
    tem de aparecer. O que sai é o offset do URDF, não o dado."""
    t, _ = compoe_como_o_no((0.0, 0.0, 0.03), IDENT, *SENSOR)
    perto(t, (0.0, 0.0, 0.03))


def test_inverte_e_mesmo_a_inversa():
    t_i, q_i = inverte((1.0, 2.0, 3.0), yaw_q(0.9))
    t, q = compoe((1.0, 2.0, 3.0), yaw_q(0.9), t_i, q_i)
    perto(t, (0.0, 0.0, 0.0))
    perto(q, IDENT, tol=1e-9)


# ------------------------------------------------- oráculo independente
#
# As travas acima usam um sensor SEM rotação (o Mid-360 está centrado e
# nivelado), e por isso não exercitam a parte de quatérnio da composição: as
# mutações que quebravam a inversa do quatérnio derrubavam um teste só. Com um
# sensor INCLINADO, a conta passa a depender de tudo — e a referência aqui é uma
# implementação em matriz 4×4, escrita separada de propósito. Duas
# implementações erradas do mesmo jeito é o que este oráculo evita.


def _R(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def _M(t, q):
    r = _R(q)
    return [r[i] + [t[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def _inv(m):
    rt = [[m[j][i] for j in range(3)] for i in range(3)]
    t = [-sum(rt[i][k] * m[k][3] for k in range(3)) for i in range(3)]
    return [rt[i] + [t[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def _oraculo(pose_t, pose_q, sensor_t, sensor_q):
    """T = inv(S) · P · S, em matriz."""
    S = _M(sensor_t, sensor_q)
    P = _M(pose_t, pose_q)
    return _mul(_inv(S), _mul(P, S))


def _translacao(m):
    return (m[0][3], m[1][3], m[2][3])


TORTOS = [
    ((0.0, 0.0, -0.42), IDENT),                       # o Mid-360 de hoje
    ((0.05, -0.02, -0.42), yaw_q(0.3)),               # torto no plano
    ((0.05, -0.02, -0.42), (0.1, 0.2, 0.3, 0.927)),   # inclinado de verdade
]

POSES = [
    ((0.0, 0.0, 0.0), IDENT),
    ((2.0, 0.5, 0.0), yaw_q(0.7)),
    ((-1.3, 2.2, 0.04), yaw_q(-2.1)),
]


def test_a_composicao_do_no_bate_com_a_matriz():
    for s_t, s_q in TORTOS:
        # normaliza o quatérnio torto, que foi escrito à mão
        n = sum(c * c for c in s_q) ** 0.5
        s_q = tuple(c / n for c in s_q)
        for p_t, p_q in POSES:
            t, _ = compoe_como_o_no(p_t, p_q, s_t, s_q)
            perto(t, _translacao(_oraculo(p_t, p_q, s_t, s_q)), tol=1e-9)


def test_com_sensor_TORTO_a_largada_ainda_e_a_origem():
    """A propriedade que define a escolha: onde quer que o sensor esteja
    montado, no instante em que a pose é identidade o `base_link` está na
    origem do `odom`. É isto que põe o chão em z ≈ 0."""
    for s_t, s_q in TORTOS:
        n = sum(c * c for c in s_q) ** 0.5
        s_q = tuple(c / n for c in s_q)
        t, q = compoe_como_o_no((0.0, 0.0, 0.0), IDENT, s_t, s_q)
        perto(t, (0.0, 0.0, 0.0))
        perto(q, IDENT)
