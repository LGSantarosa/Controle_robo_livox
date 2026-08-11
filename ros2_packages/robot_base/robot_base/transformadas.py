#!/usr/bin/env python3
"""Composição de transformadas rígidas — matemática pura, sem ROS.

Separado de `tf_odom.py` de propósito: assim a parte que pode estar **errada em
silêncio** (a composição) é testável sem ROS instalado, e roda na suíte do dev.
Quatérnios no formato do ROS: `(x, y, z, w)`.
"""


def q_mult(a, b):
    """Produto de quatérnios (x, y, z, w) — a ∘ b."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def q_rot(q, v):
    """Roda o vetor v pelo quatérnio q."""
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2w(q_vec × v) + 2(q_vec × (q_vec × v))
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def compoe(t_ab, q_ab, t_bc, q_bc):
    """Compõe A→B com B→C, devolvendo (translação, rotação) de A→C.

    ⚠️ O offset da segunda transformada é **rodado** pela rotação da primeira.
    Somar sem rodar dá o mesmo resultado com o robô alinhado e erra assim que
    ele gira — erro que depende do rumo e não aparece em teste feito parado.
    """
    r = q_rot(q_ab, t_bc)
    return (
        (t_ab[0] + r[0], t_ab[1] + r[1], t_ab[2] + r[2]),
        q_mult(q_ab, q_bc),
    )


def inverte(t_ab, q_ab):
    """A→B vira B→A.

    Existe para o `tf_odom` poder colocar o `odom` **no chão**: a pose que o
    FAST-LIO entrega é do SENSOR, e a origem dela fica na altura dele (0,42 m).
    Compondo só à direita (`pose ∘ sensor→base_link`) o `odom` herda essa
    altura, e foi o que 10-08 mediu: `tf2_echo odom base_link` deu z = −0,477 m,
    ou seja, o robô 48 cm ABAIXO da origem do próprio odom.
    """
    q_inv = (-q_ab[0], -q_ab[1], -q_ab[2], q_ab[3])
    t = q_rot(q_inv, t_ab)
    return (-t[0], -t[1], -t[2]), q_inv
