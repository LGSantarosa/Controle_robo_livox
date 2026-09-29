#!/usr/bin/env python3
"""As seis tentativas do A/B do freio linear (decisão 062 §7) — dado puro.

Separado do executor de propósito: as poses e a ordem são o desenho do
experimento, e desenho errado não se descobre olhando um log de 40 minutos de
Gazebo. Aqui eles são dado, com teste.

## O desenho

**A–B–A**, seis tentativas, cada uma com **Gazebo e pilha novos**:

    A-chegada · A-porta · B-chegada · B-porta · A′-chegada · A′-porta

`A` é `freio_linear:=true` (o estado de hoje no robô 2 e o da linha de base de
29-09), `B` é `false`. A repetição do `A` no fim é o que separa "o freio mudou
o resultado" de "a máquina mudou entre as metades".

🔴 **Nada de teletransporte.** Reposicionar só a entidade do Gazebo deixaria
para trás a fila da placa simulada, os integradores do compensador, os costmaps
e os estados dos controladores — tudo contaminado pela tentativa anterior. Cada
tentativa nasce do zero, com `pose_x/pose_y/pose_yaw` idênticos.

## As duas provas, e por que as poses são diferentes

- **chegada**: nasce em (2,0; 5,0), o começo do percurso, e vai ao goal da ida.
  Mede o rebote na parada final.
- **porta**: nasce **onde a ida de 29-09 terminou** e vai ao goal da volta, que
  atravessa o aperto de 0,80 m logo nos primeiros segundos. Isso é o que torna
  a pose de ENTRADA na porta idêntica nas três tentativas.

⚠️ Na linha de base a volta começou onde a ida tinha parado — ou seja, **o
próprio freio escolheu a pose de entrada na porta**. Daí a prova da porta não
poder ser encadeada.

⚠️ A pose é idêntica **entre as três provas de chegada** e, separadamente,
**entre as três provas da porta**. Não existe uma pose comum às seis.

## As poses

Lidas do `/goal_pose` e da `/Odometry` do bag de 29-09
(`59e849ea…97c7d`), sem arredondar. O `pose_yaw` da launch é em **radianos**.
Os quaternions originais dos objetivos ficam guardados porque é o que vai no
`NavigateToPose`: reconstruir a partir do yaw introduziria erro que não estava
no experimento original.
"""

# ── os dois objetivos, exatamente como saíram do bag ─────────────────────────
GOAL_IDA = {
    'x': 11.019571304321289,
    'y': 1.4171677827835083,
    'yaw': -1.5857984884015157,
    'qz': -0.7123909035318096,
    'qw': 0.7017828728069189,
}
GOAL_VOLTA = {
    'x': 1.9895908832550049,
    'y': 4.9765424728393555,
    'yaw': 1.5173528616295557,
    'qz': 0.6879614713441097,
    'qw': 0.725747210773867,
}

# ── as duas poses de partida ─────────────────────────────────────────────────
POSE_CHEGADA = {'x': 2.0, 'y': 5.0, 'yaw': 0.0}
# onde a ida de 29-09 terminou — a entrada canônica da porta
POSE_PORTA = {
    'x': 11.050555349676653,
    'y': 1.557740268349154,
    'yaw': -2.1997542156984036,
}

PROVAS = {
    'chegada': {'pose': POSE_CHEGADA, 'goal': GOAL_IDA},
    'porta': {'pose': POSE_PORTA, 'goal': GOAL_VOLTA},
}

# A ordem importa e é parte do desenho: o A′ no fim é o controle temporal.
ORDEM = [
    ('A', True, 'chegada'),
    ('A', True, 'porta'),
    ('B', False, 'chegada'),
    ('B', False, 'porta'),
    ("A'", True, 'chegada'),
    ("A'", True, 'porta'),
]

# ── tolerâncias do protocolo, declaradas ANTES de rodar ──────────────────────
# Quanto a pose observada na largada pode diferir da pedida. O Gazebo assenta o
# corpo no chão e a odometria nasce com ruído; acima disto a tentativa é
# ABORTADA, porque comparar duas metades que partiram de lugares diferentes é
# exatamente o que este desenho existe para evitar.
TOL_POSE_INICIAL_M = 0.05
TOL_YAW_INICIAL_RAD = 0.05
# A tolerância viva do `goal_checker`. O juiz usa 0,25 m; se o perfil subir com
# outra, os números não são comparáveis com a linha de base e o executor para.
TOLERANCIA_GOAL_ESPERADA = 0.25


def tentativas():
    """As seis, na ordem, cada uma com identificador e tudo o que precisa."""
    saida = []
    for i, (rotulo, freio, prova) in enumerate(ORDEM, 1):
        p = PROVAS[prova]
        saida.append({
            'n': i,
            'id': f'{i:02d}-{rotulo.replace(chr(39), "linha")}-{prova}',
            'rotulo': rotulo,
            'freio_linear': freio,
            'prova': prova,
            'pose': dict(p['pose']),
            'goal': dict(p['goal']),
        })
    return saida


if __name__ == '__main__':
    import json
    import sys
    json.dump(tentativas(), sys.stdout, indent=2, ensure_ascii=False)
    print()
